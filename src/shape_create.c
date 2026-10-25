// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The public shape constructors, one per shape kind. Each validates its
// geometry, builds any prebuilt content and journals the create.

#include "body_state.h"
#include "fpenv.h"
#include "journal.h"
#include "manifold.h"
#include "misuse.h"
#include "quickhull.h"
#include "shape.h"
#include "world_internal.h"

#include <string.h>

// The entry wall every constructor shares: a live body and a def with
// its cookie, valid materials and a valid compound pose. Refuses and
// returns the status otherwise.
static m3Result ShapeTarget(const m3World* world, m3BodyId body, const m3ShapeDef* def,
                            int32_t* bodyIndex)
{
    *bodyIndex = m3BodySlot(world, body);
    if (*bodyIndex < 0)
    {
        return m3Refuse(world, m3MissingId(body.index1));
    }
    if (def == NULL || def->internalValue != M3_SHAPE_COOKIE || !m3ShapeDefValid(def))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    return m3_success;
}

static m3Result CreateShapeCommon(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                  uint8_t type, const m3ShapeGeom* geom, m3ShapeId* shapeOut)
{
    int32_t bodyIndex;
    m3Result target = ShapeTarget(world, body, def, &bodyIndex);
    if (target != m3_success)
    {
        return target;
    }
    if (!m3PlainGeomValid(type, geom))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    int32_t index = m3CreateShapeInternal(world, bodyIndex, type, geom, def,
                                          &(m3ShapeContent){NULL, NULL, NULL, NULL});
    if (index < 0)
    {
        return m3CreateRefusal(world, index);
    }
    m3ShapeId id = {index + 1, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        m3CreateShapeOp record;
        memset(&record, 0, sizeof(record));
        m3JournalShapeDef(&record.def, def);
        record.geom = *geom;
        record.body = body;
        record.expected = id;
        record.type = type;
        m3JournalRecord(world, m3_opCreateShape, &record, (int32_t)sizeof(record));
    }
    *shapeOut = id;
    return m3_success;
}

static M3_NOINLINE m3Result CreateSphereShapeWorker(m3World* world, m3BodyId body,
                                                    const m3ShapeDef* def, const m3Sphere* sphere,
                                                    m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (sphere == NULL || !(sphere->radius > 0.0f) || !m3FiniteF(sphere->radius) ||
        !m3FiniteV3(sphere->center))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    int32_t bodyIndex = m3BodySlot(world, body);
    if (bodyIndex < 0)
    {
        return m3Refuse(world, m3MissingId(body.index1));
    }
    // The 2a off-origin refusal is gone: the full inertia tensor and
    // center-of-mass bookkeeping make offset spheres exact.
    m3ShapeGeom geom = {sphere->center, sphere->radius, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(world, body, def, (uint8_t)m3_sphereShape, &geom, shapeOut);
}

m3Result m3CreateSphereShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                             const m3Sphere* sphere, m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateSphereShapeWorker(world, body, def, sphere, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreatePlaneShapeWorker(m3World* world, m3BodyId body,
                                                   const m3ShapeDef* def, const m3Plane* plane,
                                                   m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (plane == NULL || !m3FiniteV3(plane->normal) || !m3FiniteF(plane->offset) ||
        !(m3Dot3(plane->normal, plane->normal) > 1.0e-12f))
    {
        return m3Refuse(world, m3_errorInvalid); // a zero or poisoned normal never
                                                 // reaches the normalize below
    }
    int32_t bodyIndex = m3BodySlot(world, body);
    if (bodyIndex < 0)
    {
        return m3Refuse(world, m3MissingId(body.index1));
    }
    if (world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        // A plane on a dynamic body is refused loudly: an infinite
        // shape has no mass.
        return m3Refuse(world, m3_errorInvalid);
    }
    m3ShapeGeom geom = {m3Normalize3(plane->normal), plane->offset, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(world, body, def, (uint8_t)m3_planeShape, &geom, shapeOut);
}

m3Result m3CreatePlaneShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                            const m3Plane* plane, m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreatePlaneShapeWorker(world, body, def, plane, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreateCapsuleShapeWorker(m3World* world, m3BodyId body,
                                                     const m3ShapeDef* def,
                                                     const m3Capsule* capsule, m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (capsule == NULL || !(capsule->radius > 0.0f) || !m3FiniteF(capsule->radius) ||
        !m3FiniteV3(capsule->point1) || !m3FiniteV3(capsule->point2))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    m3Vec3 axis = m3Sub3(capsule->point2, capsule->point1);
    if (!(m3Dot3(axis, axis) > 0.0f))
    {
        // A zero-length capsule is a sphere; asking for one is a
        // contract violation, refused loudly (use m3CreateSphereShape).
        return m3Refuse(world, m3_errorInvalid);
    }
    m3ShapeGeom geom;
    geom.v = capsule->point1;
    geom.s = capsule->radius;
    geom.v2 = capsule->point2;
    geom.s2 = 0.0f;
    return CreateShapeCommon(world, body, def, (uint8_t)m3_capsuleShape, &geom, shapeOut);
}

m3Result m3CreateCapsuleShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                              const m3Capsule* capsule, m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateCapsuleShapeWorker(world, body, def, capsule, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreateCylinderShapeWorker(m3World* world, m3BodyId body,
                                                      const m3ShapeDef* def,
                                                      const m3Cylinder* cylinder, int32_t segments,
                                                      m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    // The cylinder is a 2N-vertex prism through the interned hull
    // path. Everything downstream (mass, SAT, casts, CCD, the blast's
    // projected area) treats the prism exactly; the N-gon side is the
    // documented trade.
    if (cylinder == NULL || !(cylinder->radius > 0.0f) || !m3FiniteF(cylinder->radius) ||
        !m3FiniteV3(cylinder->point1) || !m3FiniteV3(cylinder->point2))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    m3Vec3 axis = m3Sub3(cylinder->point2, cylinder->point1);
    if (!(m3Dot3(axis, axis) > 0.0f))
    {
        return m3Refuse(world, m3_errorInvalid); // a flat cylinder is a disc, refused
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
    return m3CreateHullShape(world, body, def, points, 2 * segments, shapeOut);
}

m3Result m3CreateCylinderShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                               const m3Cylinder* cylinder, int32_t segments, m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateCylinderShapeWorker(world, body, def, cylinder, segments, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreateHullShapeWorker(m3World* world, m3BodyId body,
                                                  const m3ShapeDef* def, const m3Vec3* points,
                                                  int32_t count, m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    int32_t bodyIndex;
    m3Result target = ShapeTarget(world, body, def, &bodyIndex);
    if (target != m3_success)
    {
        return target;
    }
    if (!m3HullCloudValid(points, count))
    {
        return m3Refuse(world, m3_errorInvalid); // a poisoned or far cloud never
                                                 // reaches QuickHull's arithmetic
    }
    m3HullData data;
    if (!m3ComputeHull(&world->heap, points, count, &data))
    {
        // Degenerate cloud or over the caps: contract, null id.
        return m3Refuse(world, m3_errorInvalid);
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_hullShape, &geom, def,
                                          &(m3ShapeContent){&data, NULL, NULL, NULL});
    if (index < 0)
    {
        return m3CreateRefusal(world, index);
    }
    m3ShapeId id = {index + 1, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        m3CreateHullShapeOp record;
        memset(&record, 0, sizeof(record));
        m3JournalShapeDef(&record.def, def);
        record.body = body;
        record.expected = id;
        record.count = count;
        memcpy(record.points, points, (size_t)count * sizeof(m3Vec3));
        m3JournalRecord(world, m3_opCreateHullShape, &record, (int32_t)sizeof(record));
    }
    *shapeOut = id;
    return m3_success;
}

m3Result m3CreateHullShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                           const m3Vec3* points, int32_t count, m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateHullShapeWorker(world, body, def, points, count, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreateMeshShapeWorker(m3World* world, m3BodyId body,
                                                  const m3ShapeDef* def, const m3Vec3* vertices,
                                                  int32_t vertexCount, const uint16_t* indices,
                                                  int32_t triangleCount, m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    int32_t bodyIndex;
    m3Result target = ShapeTarget(world, body, def, &bodyIndex);
    if (target != m3_success)
    {
        return target;
    }
    if (world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        // Meshes are static world geometry: a dynamic mesh body is
        // refused loudly (no mass model for triangle soup).
        return m3Refuse(world, m3_errorInvalid);
    }
    if (vertices == NULL || indices == NULL || vertexCount < 3 || vertexCount > M3_MESH_MAX_VERTS ||
        triangleCount < 1 || triangleCount > M3_MESH_MAX_TRIS)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    for (int32_t i = 0; i < 3 * triangleCount; ++i)
    {
        if (indices[i] >= (uint16_t)vertexCount)
        {
            return m3Refuse(world, m3_errorInvalid); // out-of-range index: contract
        }
    }
    for (int32_t i = 0; i < vertexCount; ++i)
    {
        if (!m3FiniteV3(vertices[i]))
        {
            return m3Refuse(world, m3_errorInvalid); // poisoned vertex: contract
        }
    }
    m3MeshData mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.vertexCount = vertexCount;
    mesh.triangleCount = triangleCount;
    if (!m3MeshDataAlloc(&world->heap, &mesh))
    {
        return m3Refuse(world, m3_errorCapacity);
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
        m3MeshDataFree(&world->heap, &mesh);
        return m3CreateRefusal(world, index);
    }
    m3ShapeId id = {index + 1, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        // The recipe: the header, then the raw vertex and index arrays.
        // Replay rebuilds the mesh and verifies the id.
        m3CreateMeshShapeOp record;
        memset(&record, 0, sizeof(record));
        m3JournalShapeDef(&record.def, def);
        record.body = body;
        record.expected = id;
        record.vertexCount = vertexCount;
        record.triangleCount = triangleCount;
        m3JournalPart parts[3] = {{&record, (int32_t)sizeof(record)},
                                  {vertices, vertexCount * (int32_t)sizeof(m3Vec3)},
                                  {indices, 3 * triangleCount * (int32_t)sizeof(uint16_t)}};
        m3JournalRecordParts(world, m3_opCreateMeshShape, parts, 3);
    }
    *shapeOut = id;
    return m3_success;
}

m3Result m3CreateMeshShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                           const m3Vec3* vertices, int32_t vertexCount, const uint16_t* indices,
                           int32_t triangleCount, m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateMeshShapeWorker(world, body, def, vertices, vertexCount, indices,
                                            triangleCount, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreateHeightFieldShapeWorker(m3World* world, m3BodyId body,
                                                         const m3ShapeDef* def,
                                                         const float* heights, int32_t nx,
                                                         int32_t nz, m3real cellSize,
                                                         m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (heights == NULL || nx < 2 || nx > 32 || nz < 2 || nz > 32 || !(cellSize > 0.0f) ||
        !m3FiniteF(cellSize))
    {
        return m3Refuse(world, m3_errorInvalid); // grid contract: chunks tile larger terrain
    }
    for (int32_t i = 0; i < nx * nz; ++i)
    {
        if (!m3FiniteF(heights[i]))
        {
            return m3Refuse(world, m3_errorInvalid); // poisoned sample: contract
        }
    }
    if (m3BodySlot(world, body) < 0)
    {
        return m3Refuse(world, m3MissingId(body.index1));
    }
    // Triangulate the grid (CCW seen from +y) and reuse the mesh
    // path whole: welding, journaling, snapshotting all come free.
    // Heap staging: 65k-scale grids do not fit a stack.
    m3Vec3* verts = (m3Vec3*)m3AllocZeroed(&world->heap, nx * nz * (int32_t)sizeof(m3Vec3));
    uint16_t* tris =
        (uint16_t*)m3AllocZeroed(&world->heap, 6 * (nx - 1) * (nz - 1) * (int32_t)sizeof(uint16_t));
    if (verts == NULL || tris == NULL)
    {
        m3Free(&world->heap, verts);
        m3Free(&world->heap, tris);
        return m3Refuse(world, m3_errorCapacity);
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
    m3Result result = m3CreateMeshShape(world, body, def, verts, nx * nz, tris, n / 3, shapeOut);
    m3Free(&world->heap, verts);
    m3Free(&world->heap, tris);
    return result;
}

m3Result m3CreateHeightFieldShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                  const float* heights, int32_t nx, int32_t nz, m3real cellSize,
                                  m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result =
        CreateHeightFieldShapeWorker(world, body, def, heights, nx, nz, cellSize, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreateHeightFieldGridShapeWorker(m3World* world, m3BodyId body,
                                                             const m3ShapeDef* def,
                                                             const float* heights, int32_t nx,
                                                             int32_t nz, m3real cellSize,
                                                             m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    int32_t bodyIndex;
    m3Result target = ShapeTarget(world, body, def, &bodyIndex);
    if (target != m3_success)
    {
        return target;
    }
    if (heights == NULL || world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        return m3Refuse(world, m3_errorInvalid); // static bodies only, like meshes
    }
    // The content wall the decode and restore share: grid limits, finite
    // samples, a positive cell.
    float lo;
    float hi;
    if (!m3HeightFieldSamplesValid(nx, nz, cellSize, heights, &lo, &hi))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    m3HeightFieldData hf;
    memset(&hf, 0, sizeof(hf));
    hf.nx = nx;
    hf.nz = nz;
    hf.cellSize = cellSize;
    if (!m3HeightFieldDataAlloc(&world->heap, &hf))
    {
        return m3Refuse(world, m3_errorCapacity);
    }
    memcpy(hf.heights, heights, (size_t)(nx * nz) * sizeof(float));
    hf.minHeight = lo;
    hf.maxHeight = hi;
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_heightFieldShape, &geom,
                                          def, &(m3ShapeContent){NULL, NULL, NULL, &hf});
    if (index < 0)
    {
        m3HeightFieldDataFree(&world->heap, &hf);
        return m3CreateRefusal(world, index);
    }
    m3ShapeId id = {index + 1, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        // The fixed head, then the raw samples.
        m3CreateHeightFieldGridOp head;
        memset(&head, 0, sizeof(head));
        head.body = body;
        m3JournalShapeDef(&head.def, def);
        head.nx = nx;
        head.nz = nz;
        head.cellSize = cellSize;
        head.expected = id;
        m3JournalPart parts[2] = {{&head, (int32_t)sizeof(head)},
                                  {heights, nx * nz * (int32_t)sizeof(float)}};
        m3JournalRecordParts(world, m3_opCreateHeightFieldGrid, parts, 2);
    }
    *shapeOut = id;
    return m3_success;
}

m3Result m3CreateHeightFieldGridShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                      const float* heights, int32_t nx, int32_t nz, m3real cellSize,
                                      m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result =
        CreateHeightFieldGridShapeWorker(world, body, def, heights, nx, nz, cellSize, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreateVoxelChunkShapeWorker(m3World* world, m3BodyId body,
                                                        const m3ShapeDef* def,
                                                        const uint8_t* voxels,
                                                        const uint16_t* payload, m3real cellSize,
                                                        m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    int32_t bodyIndex;
    m3Result target = ShapeTarget(world, body, def, &bodyIndex);
    if (target != m3_success)
    {
        return target;
    }
    if (voxels == NULL || !(cellSize > 0.0f) || !m3FiniteF(cellSize))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    if (world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody || def->isSensor)
    {
        // Voxel chunks are static level geometry, and sensors are
        // convex volumes by contract: both are refused.
        return m3Refuse(world, m3_errorInvalid);
    }
    m3VoxelChunkData* chunk =
        (m3VoxelChunkData*)m3AllocZeroed(&world->heap, (int32_t)sizeof(m3VoxelChunkData));
    if (chunk == NULL)
    {
        return m3Refuse(world, m3_errorCapacity);
    }
    int32_t filled = m3VoxelPack(chunk, voxels, payload, cellSize);
    if (filled == 0)
    {
        m3Free(&world->heap, chunk);
        return m3Refuse(world, m3_errorInvalid); // an empty chunk is a request for nothing
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.s = cellSize;
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_voxelShape, &geom, def,
                                          &(m3ShapeContent){NULL, NULL, chunk, NULL});
    m3Free(&world->heap, chunk);
    if (index < 0)
    {
        return m3CreateRefusal(world, index);
    }
    m3ShapeId id = {index + 1, world->shapes.shapePool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        // Header + the packed grid (bitset and payload): the exact
        // recipe, so replay rebuilds the identical chunk and surface.
        m3OpCreateVoxelChunkShape record;
        memset(&record, 0, sizeof(record));
        m3JournalShapeDef(&record.def, def);
        record.body = body;
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
    *shapeOut = id;
    return m3_success;
}

m3Result m3CreateVoxelChunkShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                 const uint8_t* voxels, const uint16_t* payload, m3real cellSize,
                                 m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result =
        CreateVoxelChunkShapeWorker(world, body, def, voxels, payload, cellSize, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result CreateBoxShapeWorker(m3World* world, m3BodyId body,
                                                 const m3ShapeDef* def, m3Vec3 halfExtents,
                                                 m3ShapeId* shapeOut)
{
    if (shapeOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *shapeOut = m3_nullShapeId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (!(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) || !(halfExtents.z > 0.0f) ||
        !m3LocalV3(halfExtents))
    {
        return m3Refuse(world, m3_errorInvalid); // contract: bad extents return null
    }
    m3ShapeGeom geom = {halfExtents, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(world, body, def, (uint8_t)m3_hullShape, &geom, shapeOut);
}

m3Result m3CreateBoxShape(m3World* world, m3BodyId body, const m3ShapeDef* def, m3Vec3 halfExtents,
                          m3ShapeId* shapeOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateBoxShapeWorker(world, body, def, halfExtents, shapeOut);
    m3FloatEnvLeave(env);
    return result;
}
