// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The public shape constructors, one per shape kind. Each validates its
// geometry, builds any prebuilt content and journals the create.

#include "body.h"
#include "broad_phase.h"
#include "hull.h"
#include "journal.h"
#include "manifold.h"
#include "quickhull.h"
#include "shape.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

// The entry wall every constructor shares: a live body and a def with
// its cookie, valid materials and a valid compound pose. Returns the
// world, or NULL after refusing.
static m3World* ShapeTarget(m3BodyId bodyId, const m3ShapeDef* def, int32_t* bodyIndex)
{
    m3World* world = m3WorldFromTag(bodyId.world);
    *bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (*bodyIndex < 0 || def == NULL || def->internalValue != M3_SHAPE_COOKIE ||
        !m3ShapeDefValid(def))
    {
        m3Refuse(world, m3_errorInvalid);
        return NULL;
    }
    return world;
}

static m3ShapeId CreateShapeCommon(m3BodyId bodyId, const m3ShapeDef* def, uint8_t type,
                                   const m3ShapeGeom* geom)
{
    int32_t bodyIndex;
    m3World* world = ShapeTarget(bodyId, def, &bodyIndex);
    if (world == NULL)
    {
        return m3_nullShapeId;
    }
    if (!m3PlainGeomValid(type, geom))
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId;
    }
    int32_t index = m3CreateShapeInternal(world, bodyIndex, type, geom, def,
                                          &(m3ShapeContent){NULL, NULL, NULL, NULL});
    if (index < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->idWorld, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        m3CreateShapeOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.geom = *geom;
        record.body = bodyId;
        record.expected = id;
        record.type = type;
        m3JournalRecord(world, m3_opCreateShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

m3ShapeId m3CreateSphereShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Sphere* sphere)
{
    if (sphere == NULL || !(sphere->radius > 0.0f) || !m3FiniteF(sphere->radius) ||
        !m3FiniteV3(sphere->center))
    {
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
        return m3_nullShapeId;
    }
    m3World* world = m3WorldFromTag(bodyId.world);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId;
    }
    // The 2a off-origin refusal is gone: the full inertia tensor and
    // center-of-mass bookkeeping make offset spheres exact.
    m3ShapeGeom geom = {sphere->center, sphere->radius, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_sphereShape, &geom);
}

m3ShapeId m3CreatePlaneShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Plane* plane)
{
    if (plane == NULL || !m3FiniteV3(plane->normal) || !m3FiniteF(plane->offset) ||
        !(m3Dot3(plane->normal, plane->normal) > 1.0e-12f))
    {
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
        return m3_nullShapeId; // a zero or poisoned normal never
                               // reaches the normalize below
    }
    m3World* world = m3WorldFromTag(bodyId.world);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        // A plane on a dynamic body is refused loudly: an infinite
        // shape has no mass.
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId;
    }
    m3ShapeGeom geom = {m3Normalize3(plane->normal), plane->offset, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_planeShape, &geom);
}

m3ShapeId m3CreateCapsuleShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Capsule* capsule)
{
    if (capsule == NULL || !(capsule->radius > 0.0f) || !m3FiniteF(capsule->radius) ||
        !m3FiniteV3(capsule->point1) || !m3FiniteV3(capsule->point2))
    {
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
        return m3_nullShapeId;
    }
    m3Vec3 axis = m3Sub3(capsule->point2, capsule->point1);
    if (!(m3Dot3(axis, axis) > 0.0f))
    {
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
        // A zero-length capsule is a sphere; asking for one is a
        // contract violation, refused loudly (use m3CreateSphereShape).
        return m3_nullShapeId;
    }
    m3ShapeGeom geom;
    geom.v = capsule->point1;
    geom.s = capsule->radius;
    geom.v2 = capsule->point2;
    geom.s2 = 0.0f;
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_capsuleShape, &geom);
}

m3ShapeId m3CreateCylinderShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Cylinder* cylinder,
                                int32_t segments)
{
    // The cylinder is a 2N-vertex prism through the interned hull
    // path. Everything downstream (mass, SAT, casts, CCD, the blast's
    // projected area) treats the prism exactly; the N-gon side is the
    // documented trade.
    if (cylinder == NULL || !(cylinder->radius > 0.0f) || !m3FiniteF(cylinder->radius) ||
        !m3FiniteV3(cylinder->point1) || !m3FiniteV3(cylinder->point2))
    {
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
        return m3_nullShapeId;
    }
    m3Vec3 axis = m3Sub3(cylinder->point2, cylinder->point1);
    if (!(m3Dot3(axis, axis) > 0.0f))
    {
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
        return m3_nullShapeId; // a flat cylinder is a disc, refused
    }
    if (segments < 3)
    {
        segments = 3; // a quality knob clamps, it does not refuse
    }
    if (segments > 32)
    {
        segments = 32; // 2N stays inside the 64-vertex hull law
    }
    m3Vec3 n = m3Normalize3(axis);
    m3Vec3 t1;
    m3Vec3 t2;
    m3MakeTangentBasis(n, &t1, &t2);
    m3Vec3 points[64];
    m3real step = 2.0f * M3_PI / (m3real)segments;
    for (int32_t k = 0; k < segments; ++k)
    {
        m3CosSin cs = m3ComputeCosSin(step * (m3real)k);
        m3Vec3 rim =
            m3Add3(m3MulSV3(cylinder->radius * cs.c, t1), m3MulSV3(cylinder->radius * cs.s, t2));
        points[k] = m3Add3(cylinder->point1, rim);
        points[segments + k] = m3Add3(cylinder->point2, rim);
    }
    return m3CreateHullShape(bodyId, def, points, 2 * segments);
}

m3ShapeId m3CreateHullShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Vec3* points,
                            int32_t count)
{
    int32_t bodyIndex;
    m3World* world = ShapeTarget(bodyId, def, &bodyIndex);
    if (world == NULL)
    {
        return m3_nullShapeId;
    }
    if (points == NULL || count <= 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        if (!m3FiniteV3(points[i]))
        {
            m3Refuse(world, m3_errorInvalid);
            return m3_nullShapeId; // a poisoned cloud never reaches
                                   // QuickHull's arithmetic
        }
    }
    m3HullData data;
    if (!m3ComputeHull(points, count, &data))
    {
        m3Refuse(world, m3_errorInvalid);
        // Degenerate cloud or over the caps: contract, null id.
        return m3_nullShapeId;
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_hullShape, &geom, def,
                                          &(m3ShapeContent){&data, NULL, NULL, NULL});
    if (index < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->idWorld, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        m3CreateHullShapeOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.body = bodyId;
        record.expected = id;
        record.count = count;
        memcpy(record.points, points, (size_t)count * sizeof(m3Vec3));
        m3JournalRecord(world, m3_opCreateHullShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

m3ShapeId m3CreateMeshShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Vec3* vertices,
                            int32_t vertexCount, const uint16_t* indices, int32_t triangleCount)
{
    int32_t bodyIndex;
    m3World* world = ShapeTarget(bodyId, def, &bodyIndex);
    if (world == NULL)
    {
        return m3_nullShapeId;
    }
    if (world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        m3Refuse(world, m3_errorInvalid);
        // Meshes are static world geometry: a dynamic mesh body is
        // refused loudly (no mass model for triangle soup).
        return m3_nullShapeId;
    }
    if (vertices == NULL || indices == NULL || vertexCount < 3 || vertexCount > M3_MESH_MAX_VERTS ||
        triangleCount < 1 || triangleCount > M3_MESH_MAX_TRIS)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId;
    }
    for (int32_t i = 0; i < 3 * triangleCount; ++i)
    {
        if (indices[i] >= (uint16_t)vertexCount)
        {
            m3Refuse(world, m3_errorInvalid);
            return m3_nullShapeId; // out-of-range index: contract
        }
    }
    for (int32_t i = 0; i < vertexCount; ++i)
    {
        if (!m3FiniteV3(vertices[i]))
        {
            m3Refuse(world, m3_errorInvalid);
            return m3_nullShapeId; // poisoned vertex: contract
        }
    }
    m3MeshData mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.vertexCount = vertexCount;
    mesh.triangleCount = triangleCount;
    if (!m3MeshDataAlloc(&mesh))
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullShapeId;
    }
    memcpy(mesh.vertices, vertices, (size_t)vertexCount * sizeof(m3Vec3));
    memcpy(mesh.indices, indices, (size_t)(3 * triangleCount) * sizeof(uint16_t));
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    // The slot takes OWNERSHIP of the arrays on success (the struct
    // copy carries the pointers); failure frees them here.
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_meshShape, &geom, def,
                                          &(m3ShapeContent){NULL, &mesh, NULL, NULL});
    if (index < 0)
    {
        m3MeshDataFree(&mesh);
        m3Refuse(world, m3_errorCapacity);
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->idWorld, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        // The recipe: the header, then the raw vertex and index arrays.
        // Replay rebuilds the mesh and verifies the id.
        m3CreateMeshShapeOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.body = bodyId;
        record.expected = id;
        record.vertexCount = vertexCount;
        record.triangleCount = triangleCount;
        m3JournalPart parts[3] = {{&record, (int32_t)sizeof(record)},
                                  {vertices, vertexCount * (int32_t)sizeof(m3Vec3)},
                                  {indices, 3 * triangleCount * (int32_t)sizeof(uint16_t)}};
        m3JournalRecordParts(world, m3_opCreateMeshShape, parts, 3);
    }
    return id;
}

m3ShapeId m3CreateHeightFieldShape(m3BodyId bodyId, const m3ShapeDef* def, const float* heights,
                                   int32_t nx, int32_t nz, m3real cellSize)
{
    if (heights == NULL || nx < 2 || nx > 32 || nz < 2 || nz > 32 || !(cellSize > 0.0f) ||
        !m3FiniteF(cellSize))
    {
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
        return m3_nullShapeId; // grid contract: chunks tile larger terrain
    }
    for (int32_t i = 0; i < nx * nz; ++i)
    {
        if (!m3FiniteF(heights[i]))
        {
            m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
            return m3_nullShapeId; // poisoned sample: contract
        }
    }
    // Triangulate the grid (CCW seen from +y) and reuse the mesh
    // path whole: welding, journaling, snapshotting all come free.
    // Heap staging: 65k-scale grids no longer fit a stack.
    m3Vec3* verts = (m3Vec3*)m3AllocZeroed(nx * nz * (int32_t)sizeof(m3Vec3));
    uint16_t* tris = (uint16_t*)m3AllocZeroed(6 * (nx - 1) * (nz - 1) * (int32_t)sizeof(uint16_t));
    if (verts == NULL || tris == NULL)
    {
        m3Free(verts);
        m3Free(tris);
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorCapacity);
        return m3_nullShapeId;
    }
    for (int32_t iz = 0; iz < nz; ++iz)
    {
        for (int32_t ix = 0; ix < nx; ++ix)
        {
            verts[iz * nx + ix] =
                (m3Vec3){cellSize * (m3real)ix, heights[iz * nx + ix], cellSize * (m3real)iz};
        }
    }
    int32_t n = 0;
    for (int32_t iz = 0; iz < nz - 1; ++iz)
    {
        for (int32_t ix = 0; ix < nx - 1; ++ix)
        {
            uint16_t v00 = (uint16_t)(iz * nx + ix);
            uint16_t v10 = (uint16_t)(iz * nx + ix + 1);
            uint16_t v01 = (uint16_t)((iz + 1) * nx + ix);
            uint16_t v11 = (uint16_t)((iz + 1) * nx + ix + 1);
            tris[n++] = v00;
            tris[n++] = v11;
            tris[n++] = v10;
            tris[n++] = v00;
            tris[n++] = v01;
            tris[n++] = v11;
        }
    }
    m3ShapeId id = m3CreateMeshShape(bodyId, def, verts, nx * nz, tris, n / 3);
    m3Free(verts);
    m3Free(tris);
    return id;
}

m3ShapeId m3CreateHeightFieldGridShape(m3BodyId bodyId, const m3ShapeDef* def, const float* heights,
                                       int32_t nx, int32_t nz, m3real cellSize)
{
    int32_t bodyIndex;
    m3World* world = ShapeTarget(bodyId, def, &bodyIndex);
    if (world == NULL)
    {
        return m3_nullShapeId;
    }
    if (heights == NULL || world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId; // static bodies only, like meshes
    }
    // The full content wall, mirrored into the decode: grid
    // limits, finite samples, a positive cell.
    if (nx < 2 || nx > M3_HEIGHTFIELD_MAX_DIM || nz < 2 || nz > M3_HEIGHTFIELD_MAX_DIM ||
        !m3FiniteF(cellSize) || !(cellSize > 0.0f))
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId;
    }
    for (int32_t i = 0; i < nx * nz; ++i)
    {
        if (!m3FiniteF(heights[i]))
        {
            m3Refuse(world, m3_errorInvalid);
            return m3_nullShapeId;
        }
    }
    m3HeightFieldData hf;
    memset(&hf, 0, sizeof(hf));
    hf.nx = nx;
    hf.nz = nz;
    hf.cellSize = cellSize;
    if (!m3HeightFieldDataAlloc(&hf))
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullShapeId;
    }
    memcpy(hf.heights, heights, (size_t)(nx * nz) * sizeof(float));
    float lo = heights[0];
    float hi = heights[0];
    for (int32_t i = 1; i < nx * nz; ++i)
    {
        lo = heights[i] < lo ? heights[i] : lo;
        hi = heights[i] > hi ? heights[i] : hi;
    }
    hf.minHeight = lo;
    hf.maxHeight = hi;
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_heightFieldShape, &geom,
                                          def, &(m3ShapeContent){NULL, NULL, NULL, &hf});
    if (index < 0)
    {
        m3HeightFieldDataFree(&hf);
        m3Refuse(world, m3_errorCapacity);
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->idWorld, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        // The fixed head, then the raw samples.
        m3CreateHeightFieldGridOp head;
        memset(&head, 0, sizeof(head));
        head.body = bodyId;
        head.def = *def;
        head.nx = nx;
        head.nz = nz;
        head.cellSize = cellSize;
        head.expected = id;
        m3JournalPart parts[2] = {{&head, (int32_t)sizeof(head)},
                                  {heights, nx * nz * (int32_t)sizeof(float)}};
        m3JournalRecordParts(world, m3_opCreateHeightFieldGrid, parts, 2);
    }
    return id;
}

m3ShapeId m3CreateVoxelChunkShape(m3BodyId bodyId, const m3ShapeDef* def, const uint8_t* voxels,
                                  const uint16_t* payload, m3real cellSize)
{
    int32_t bodyIndex;
    m3World* world = ShapeTarget(bodyId, def, &bodyIndex);
    if (world == NULL)
    {
        return m3_nullShapeId;
    }
    if (voxels == NULL || !(cellSize > 0.0f) || !m3FiniteF(cellSize))
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId;
    }
    if (world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody || def->isSensor)
    {
        m3Refuse(world, m3_errorInvalid);
        // Voxel chunks are static level geometry, and sensors are
        // convex volumes by contract: both are refused.
        return m3_nullShapeId;
    }
    m3VoxelChunkData* chunk = (m3VoxelChunkData*)m3AllocZeroed((int32_t)sizeof(m3VoxelChunkData));
    if (chunk == NULL)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullShapeId;
    }
    int32_t filled = m3VoxelPack(chunk, voxels, payload, cellSize);
    if (filled == 0)
    {
        m3Refuse(world, m3_errorInvalid);
        m3Free(chunk);
        return m3_nullShapeId; // an empty chunk is a request for nothing
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.s = cellSize;
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_voxelShape, &geom, def,
                                          &(m3ShapeContent){NULL, NULL, chunk, NULL});
    m3Free(chunk);
    if (index < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->idWorld, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        // Header + the packed grid (bitset and payload): the exact
        // recipe, so replay rebuilds the identical chunk and surface.
        m3OpCreateVoxelChunkShape record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.body = bodyId;
        record.expected = id;
        record.cellSize = cellSize;
        uint8_t payloadBuf[sizeof(record) + sizeof(((m3VoxelChunkData*)0)->occupancy) +
                           sizeof(((m3VoxelChunkData*)0)->payload) +
                           sizeof(((m3VoxelChunkData*)0)->fill)];
        memcpy(payloadBuf, &record, sizeof(record));
        const m3VoxelChunkData* stored =
            &world->voxels.voxelData[world->shapes.shapeVoxelIndex[index]];
        memcpy(payloadBuf + sizeof(record), stored->occupancy, sizeof(stored->occupancy));
        memcpy(payloadBuf + sizeof(record) + sizeof(stored->occupancy), stored->payload,
               sizeof(stored->payload));
        memcpy(payloadBuf + sizeof(record) + sizeof(stored->occupancy) + sizeof(stored->payload),
               stored->fill, sizeof(stored->fill));
        m3JournalRecord(world, m3_opCreateVoxelChunkShape, payloadBuf, (int32_t)sizeof(payloadBuf));
    }
    return id;
}

m3ShapeId m3CreateBoxShape(m3BodyId bodyId, const m3ShapeDef* def, m3Vec3 halfExtents)
{
    if (!(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) || !(halfExtents.z > 0.0f) ||
        !m3FiniteV3(halfExtents))
    {
        m3Refuse(m3WorldFromTag(bodyId.world), m3_errorInvalid);
        return m3_nullShapeId; // contract: bad extents return null
    }
    m3ShapeGeom geom = {halfExtents, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_hullShape, &geom);
}
