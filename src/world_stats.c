// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's observers: its memory use, its counters and the last step's
// profile. Readers only, never hashed or journaled.

#include "fpenv.h"
#include "misuse.h"
#include "world_internal.h"

// Live slots without a scan: the pool hands out from the free queue
// or bumps maxIndex, and retirement is the only other exit.
static int32_t PoolLive(const m3IdPool* pool)
{
    return pool->maxIndex - pool->freeCount - pool->retiredCount;
}

static M3_NOINLINE m3MemoryUsage WorldGetMemoryUsageWorker(const m3World* world)
{
    m3MemoryUsage usage;
    memset(&usage, 0, sizeof(usage));
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return usage;
    }
    usage.persistentBytes = world->memoryBytes;
    // Count-derived content, summed live so it cannot drift: mesh
    // payloads plus their derived BVHs, and heightfield samples.
    for (int32_t m = 0; m < world->meshes.meshPool.maxIndex; ++m)
    {
        if (world->meshes.meshPool.alive[m] == 0)
        {
            continue;
        }
        const m3MeshData* mesh = &world->meshes.meshData[m];
        usage.contentBytes += (int64_t)mesh->vertexCount * (int64_t)sizeof(m3Vec3);
        usage.contentBytes += 3LL * mesh->triangleCount * (int64_t)sizeof(uint16_t);
        usage.contentBytes += 2LL * mesh->triangleCount; // edge flags + materials
        const m3MeshBvh* bvh = &world->meshes.meshBvh[m];
        usage.contentBytes += (int64_t)bvh->nodeCount * (int64_t)sizeof(m3MeshBvhNode);
        usage.contentBytes += (int64_t)mesh->triangleCount * (int64_t)sizeof(uint16_t); // order
    }
    for (int32_t h = 0; h < world->shapes.shapeCapacity; ++h)
    {
        const m3HeightFieldData* hf = &world->heightFields.hfData[h];
        usage.contentBytes += (int64_t)hf->nx * (int64_t)hf->nz * (int64_t)sizeof(float);
    }
    for (int32_t v = 0; v < world->voxels.voxelCapacity; ++v)
    {
        usage.contentBytes += world->voxels.voxelTrees[v] != NULL ? M3_VOXEL_TREE_BYTES : 0;
    }
    usage.scratchCapacity = world->scratch.capacity;
    usage.scratchPeak = world->lastScratchPeak;
    return usage;
}

m3MemoryUsage m3World_GetMemoryUsage(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3MemoryUsage result = WorldGetMemoryUsageWorker(world);
    m3FloatEnvLeave(env);
    return result;
}

m3Counters m3World_GetCounters(const m3World* world)
{
    m3Counters counters;
    memset(&counters, 0, sizeof(counters));
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return counters;
    }
    counters.bodyCount = PoolLive(&world->bodies.bodyPool);
    counters.shapeCount = PoolLive(&world->shapes.shapePool);
    counters.jointCount = PoolLive(&world->joints.jointPool);
    counters.contactCount = world->contacts.pairCount;
    counters.characterCount = PoolLive(&world->characters.charPool);
    counters.vehicleCount = PoolLive(&world->vehicles.vehPool);
    counters.softBodyCount = PoolLive(&world->softBodies.softPool);
    counters.voxelChunkCount = PoolLive(&world->voxels.voxelPool);
    counters.hullCount = PoolLive(&world->hulls.hullPool);
    counters.meshCount = PoolLive(&world->meshes.meshPool);
    int32_t awake = 0;
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        // A disabled body keeps its awake flag but takes no part in the step.
        if (world->bodies.bodyPool.alive[i] != 0 &&
            world->bodies.types[i] == (uint8_t)m3_dynamicBody && world->bodies.awake[i] != 0 &&
            world->bodies.bodyEnabled[i] != 0)
        {
            awake += 1;
        }
    }
    counters.awakeCount = awake;
    counters.islandCount = world->lastIslandCount;
    counters.colorCount = world->lastColorCount;
    counters.treeHeight = world->broadphase.tree.root != M3_TREE_NULL
                              ? world->broadphase.tree.nodes[world->broadphase.tree.root].height
                              : 0;
    counters.scratchPeak = world->lastScratchPeak;
    counters.scratchCapacity = world->scratch.capacity;
    counters.voxelEditsRefused = world->voxels.voxelEditsRefused;
    counters.pairsDropped = world->contacts.pairsDropped;
    counters.heldPushRetakeCount = world->heldPushRetakeCount;
    counters.snapshotBytes = m3World_GetSnapshotSize(world);
    counters.allocCount = world->heap.allocCount;
    counters.freeCount = world->heap.freeCount;
    counters.misuseCount = m3MisuseCount(world);
    return counters;
}

static M3_NOINLINE m3Profile WorldGetProfileWorker(const m3World* world)
{
    if (world == NULL)
    {
        m3Profile zero;
        memset(&zero, 0, sizeof(zero));
        return zero;
    }
    return world->profile;
}

m3Profile m3World_GetProfile(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3Profile result = WorldGetProfileWorker(world);
    m3FloatEnvLeave(env);
    return result;
}
