// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay for shape ops: creates of every shape kind, voxel
// edits, materials, flags and geometry updates.

#include "journal_replay.h"

#include "body.h"
#include "character.h"
#include "joint.h"
#include "journal.h"
#include "query.h"
#include "quickhull.h"
#include "shape.h"
#include "softbody.h"
#include "solver.h"
#include "vehicle.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

bool m3ReplayCreateShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateShapeOp record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.body.world = world->idWorld;
    int32_t bodyIndex = m3BodySlot(world, record.body);
    if (bodyIndex < 0)
    {
        return false;
    }
    m3NormalizeShapeDefBools(&record.def);
    int32_t index = m3CreateShapeInternal(world, bodyIndex, record.type, &record.geom, &record.def,
                                          &(m3ShapeContent){NULL, NULL, NULL, NULL});
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->shapes.shapePool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for shapes too
    }
    return true;
}

bool m3ReplayCreateMeshShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateMeshShapeOp record;
    if (bytes < (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (record.vertexCount < 3 || record.vertexCount > M3_MESH_MAX_VERTS ||
        record.triangleCount < 1 || record.triangleCount > M3_MESH_MAX_TRIS)
    {
        return false;
    }
    int32_t vertexBytes = record.vertexCount * (int32_t)sizeof(m3Vec3);
    int32_t indexBytes = 3 * record.triangleCount * (int32_t)sizeof(uint16_t);
    if (bytes != (int32_t)sizeof(record) + vertexBytes + indexBytes)
    {
        return false;
    }
    record.body.world = world->idWorld;
    int32_t bodyIndex = m3BodySlot(world, record.body);
    if (bodyIndex < 0)
    {
        return false;
    }
    m3MeshData mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.vertexCount = record.vertexCount;
    mesh.triangleCount = record.triangleCount;
    if (!m3MeshDataAlloc(&mesh))
    {
        return false;
    }
    memcpy(mesh.vertices, (const uint8_t*)payload + sizeof(record), (size_t)vertexBytes);
    memcpy(mesh.indices, (const uint8_t*)payload + sizeof(record) + vertexBytes,
           (size_t)indexBytes);
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    m3NormalizeShapeDefBools(&record.def);
    // On success the slot owns the arrays; an id mismatch
    // leaves them with the slot too, and the atomic-replay
    // restore reclaims them through the alloc gate.
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_meshShape, &geom,
                                          &record.def, &(m3ShapeContent){NULL, &mesh, NULL, NULL});
    if (index < 0)
    {
        m3MeshDataFree(&mesh);
        return false;
    }
    if (index + 1 != record.expected.index1 ||
        world->shapes.shapePool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for mesh shapes too
    }
    return true;
}

bool m3ReplayCreateHullShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateHullShapeOp record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.body.world = world->idWorld;
    int32_t bodyIndex = m3BodySlot(world, record.body);
    if (bodyIndex < 0)
    {
        return false;
    }
    m3HullData rebuilt;
    if (!m3ComputeHull(record.points, record.count, &rebuilt))
    {
        return false; // the recipe must rebuild
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    m3NormalizeShapeDefBools(&record.def);
    int32_t index =
        m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_hullShape, &geom, &record.def,
                              &(m3ShapeContent){&rebuilt, NULL, NULL, NULL});
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->shapes.shapePool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for hull shapes too
    }
    return true;
}

bool m3ReplayCreateVoxelChunkShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpCreateVoxelChunkShape record;
    int32_t occBytes = (int32_t)(M3_VOXEL_COUNT / 8);
    int32_t payBytes = (int32_t)(M3_VOXEL_COUNT * sizeof(uint16_t));
    int32_t fillBytes = (int32_t)M3_VOXEL_COUNT;
    if (bytes != (int32_t)sizeof(record) + occBytes + payBytes + fillBytes)
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.body.world = world->idWorld;
    int32_t bodyIndex = m3BodySlot(world, record.body);
    if (bodyIndex < 0 || !(record.cellSize > 0.0f))
    {
        return false;
    }
    m3VoxelChunkData* chunk = (m3VoxelChunkData*)m3AllocZeroed((int32_t)sizeof(m3VoxelChunkData));
    if (chunk == NULL)
    {
        return false;
    }
    chunk->cellSize = record.cellSize;
    memcpy(chunk->occupancy, (const uint8_t*)payload + sizeof(record), (size_t)occBytes);
    memcpy(chunk->payload, (const uint8_t*)payload + sizeof(record) + occBytes, (size_t)payBytes);
    memcpy(chunk->fill, (const uint8_t*)payload + sizeof(record) + occBytes + payBytes,
           (size_t)fillBytes);
    int32_t filled = 0;
    for (int32_t v = 0; v < M3_VOXEL_COUNT; ++v)
    {
        filled += (chunk->occupancy[v >> 3] >> (v & 7)) & 1;
    }
    chunk->filledCount = filled;
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.s = record.cellSize;
    m3NormalizeShapeDefBools(&record.def);
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_voxelShape, &geom,
                                          &record.def, &(m3ShapeContent){NULL, NULL, chunk, NULL});
    m3Free(chunk);
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->shapes.shapePool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for voxels too
    }
    return true;
}

bool m3ReplayVoxelSet(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpVoxelSet record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t shape = m3ShapeSlot(world, record.id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        return false;
    }
    m3VoxelSetInternal(world, shape, record.x, record.y, record.z, record.payload);
    return true;
}

bool m3ReplayVoxelClear(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpVoxelClear record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t shape = m3ShapeSlot(world, record.id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        return false;
    }
    m3VoxelClearInternal(world, shape, record.x, record.y, record.z);
    return true;
}

bool m3ReplayVoxelSetFill(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpVoxelSetFill record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t shape = m3ShapeSlot(world, record.id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape || record.fill == 0)
    {
        return false;
    }
    m3VoxelSetFillInternal(world, shape, record.x, record.y, record.z, record.fill);
    return true;
}

bool m3ReplayVoxelClearBox(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpVoxelClearBox record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t shape = m3ShapeSlot(world, record.id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        return false;
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        if (record.lo[k] < 0 || record.hi[k] >= M3_VOXEL_DIM || record.lo[k] > record.hi[k])
        {
            return false;
        }
    }
    m3VoxelClearBoxInternal(world, shape, record.lo, record.hi);
    return true;
}

bool m3ReplayDestroyShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3ShapeId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world = world->idWorld;
    int32_t index = m3ShapeSlot(world, id);
    if (index < 0)
    {
        return false;
    }
    int32_t bodyIndex = world->shapes.shapeBody[index];
    m3DestroyShapeInternal(world, index);
    m3RecomputeMass(world, bodyIndex);
    if (world->bodies.types[bodyIndex] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, bodyIndex, 1);
    }
    return true;
}

bool m3ReplayShapeScalar(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    m3OpShapeScalar record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.value) || record.value < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opSetShapeFriction)
    {
        m3SetShapeFrictionInternal(world, slot, record.value);
    }
    else if (op == m3_opSetShapeRestitution)
    {
        m3SetShapeRestitutionInternal(world, slot, record.value);
    }
    else
    {
        m3SetShapeRollingInternal(world, slot, record.value);
    }
    return true;
}

bool m3ReplaySetShapeDensity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetShapeDensity record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.value) || record.value <= 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetShapeDensityInternal(world, slot, record.value, record.updateMass);
    return true;
}

bool m3ReplayCreateHeightFieldGrid(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateHeightFieldGridOp head;
    if (bytes < (int32_t)sizeof(head))
    {
        return false;
    }
    memcpy(&head, payload, sizeof(head));
    m3NormalizeShapeDefBools(&head.def);
    head.body.world = world->idWorld;
    int32_t bodyIndex = m3BodySlot(world, head.body);
    if (bodyIndex < 0 || world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody || head.nx < 2 ||
        head.nx > M3_HEIGHTFIELD_MAX_DIM || head.nz < 2 || head.nz > M3_HEIGHTFIELD_MAX_DIM ||
        !m3FiniteF(head.cellSize) || !(head.cellSize > 0.0f) ||
        bytes != (int32_t)sizeof(head) + head.nx * head.nz * (int32_t)sizeof(float))
    {
        return false; // hostile grid bytes fail loudly
    }
    // Same alignment law as the tet decode above: the stream
    // is byte-packed, so every sample is read by memcpy.
    const uint8_t* sampleBytes = (const uint8_t*)payload + sizeof(head);
    float mn = 0.0f;
    float mx = 0.0f;
    for (int32_t i = 0; i < head.nx * head.nz; ++i)
    {
        float v;
        memcpy(&v, sampleBytes + (size_t)i * sizeof(float), sizeof(float));
        if (!m3FiniteF(v))
        {
            return false;
        }
        mn = i == 0 ? v : (v < mn ? v : mn);
        mx = i == 0 ? v : (v > mx ? v : mx);
    }
    m3HeightFieldData hf;
    memset(&hf, 0, sizeof(hf));
    hf.nx = head.nx;
    hf.nz = head.nz;
    hf.cellSize = head.cellSize;
    hf.minHeight = mn;
    hf.maxHeight = mx;
    if (!m3HeightFieldDataAlloc(&hf))
    {
        return false;
    }
    memcpy(hf.heights, sampleBytes, (size_t)(head.nx * head.nz) * sizeof(float));
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_heightFieldShape, &geom,
                                          &head.def, &(m3ShapeContent){NULL, NULL, NULL, &hf});
    if (index < 0)
    {
        m3HeightFieldDataFree(&hf);
        return false;
    }
    if (index + 1 != head.expected.index1 ||
        world->shapes.shapePool.generations[index] != head.expected.generation)
    {
        return false; // id determinism holds for terrain too
    }
    return true;
}

bool m3ReplaySetMeshMaterials(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3SetMeshMaterialsOp head;
    if (bytes < (int32_t)sizeof(head))
    {
        return false;
    }
    memcpy(&head, payload, sizeof(head));
    head.id.world = world->idWorld;
    int32_t slot = m3ShapeSlot(world, head.id);
    if (slot < 0 || world->shapes.shapeType[slot] != (uint8_t)m3_meshShape)
    {
        return false;
    }
    int32_t meshIndex = world->shapes.shapeMeshIndex[slot];
    if (meshIndex < 0 || head.triangleCount != world->meshes.meshData[meshIndex].triangleCount ||
        bytes != (int32_t)sizeof(head) + head.triangleCount)
    {
        return false; // the byte array must match THIS mesh
    }
    if (!m3SetMeshMaterialsInternal(world, meshIndex, head.materials, head.materialCount,
                                    (const uint8_t*)payload + sizeof(head)))
    {
        return false; // hostile paint fails loudly
    }
    return true;
}

bool m3ReplaySetShapeGeom(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetShapeGeom record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (record.type > 255u)
    {
        return false;
    }
    record.id.world = world->idWorld;
    int32_t slot = record.id.index1 - 1;
    if (slot < 0 || slot >= world->shapes.shapePool.maxIndex ||
        world->shapes.shapePool.alive[slot] == 0 ||
        world->shapes.shapePool.generations[slot] != record.id.generation)
    {
        return false;
    }
    if (!m3SetShapeGeomInternal(world, slot, (uint8_t)record.type, &record.geom))
    {
        return false; // hostile swaps fail the replay loudly
    }
    return true;
}

bool m3ReplaySetSurfaceVelocity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetSurfaceVelocity record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0 || !m3FiniteV3(record.v))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetSurfaceVelocityInternal(world, slot, record.v);
    return true;
}

bool m3ReplaySetFilter(m3World* world, const m3ReplayRecord* r)
{
    m3OpSetFilter record;
    if (r->bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, r->payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetFilterInternal(world, slot, record.categoryBits, record.maskBits, record.groupIndex);
    return true;
}

bool m3ReplayShapeFlag(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    m3OpShapeFlag record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0)
    {
        return false;
    }
    if (op == m3_opEnableShapeHitEvents)
    {
        m3EnableShapeHitEventsInternal(world, slot, record.on);
    }
    else
    {
        m3EnableShapePreSolveInternal(world, slot, record.on);
    }
    return true;
}
