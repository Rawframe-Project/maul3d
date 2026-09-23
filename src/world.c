// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World lifecycle and the journal. Structure follows Maul2D's proven
// world.c (itself adapted from Box2D v3, MIT, Erin Catto): a static
// world table with generations, def-cookie validation, SoA arrays
// described once in the state table (world_state.c), and a journal whose replay goes through the
// same internal functions the public API uses.

#include "journal.h"
#include "world_internal.h"
#include "world_state.h"

#include <stddef.h>
#include <string.h>

static m3World* s_worlds[M3_MAX_WORLDS];
static uint16_t s_worldGenerations[M3_MAX_WORLDS];

m3World* m3WorldFromId(m3WorldId worldId)
{
    int32_t index = worldId.index1 - 1;
    if (index < 0 || index >= M3_MAX_WORLDS || s_worlds[index] == NULL ||
        s_worldGenerations[index] != worldId.generation)
    {
        return NULL;
    }
    return s_worlds[index];
}

m3World* m3WorldFromIndex0(uint16_t index0)
{
    return index0 < M3_MAX_WORLDS ? s_worlds[index0] : NULL;
}

m3WorldDef m3DefaultWorldDef(void)
{
    m3WorldDef def;
    memset(&def, 0, sizeof(def));
    def.gravity = (m3Vec3){0.0f, -10.0f, 0.0f};
    def.bodyCapacity = 1024;
    def.shapeCapacity = 2048;
    def.meshCapacity = 4;
    def.jointCapacity = 64;
    def.voxelCapacity = 4;
    def.characterCapacity = 4;
    def.vehicleCapacity = 2;
    def.softBodyCapacity = 2;
    def.workerCount = 1;
    def.contactHertz = M3_CONTACT_HERTZ_DEFAULT;
    def.contactDampingRatio = M3_CONTACT_DAMPING_RATIO_DEFAULT;
    def.contactPushMaxSpeed = M3_CONTACT_PUSH_MAX_SPEED_DEFAULT;
    def.restitutionThreshold = M3_RESTITUTION_THRESHOLD_DEFAULT;
    def.maximumLinearSpeed = M3_MAX_LINEAR_SPEED_DEFAULT;
    def.enableSleeping = 1;
    def.enableContinuous = 1;
    def.hitEventThreshold = M3_HIT_EVENT_THRESHOLD_DEFAULT;
    def.internalValue = M3_WORLD_COOKIE;
    return def;
}

// Frees every allocation a world owns, then the world itself. Safe on
// a partially created world: every field starts zeroed, m3Free
// accepts NULL, and the per-slot loops skip arrays that never arrived.
static void FreeWorldStorage(m3World* world)
{
    // Per-slot content first, while the arrays that hold it still exist.
    for (int32_t hf = 0; world->hfData != NULL && hf < world->shapeCapacity; ++hf)
    {
        m3HeightFieldDataFree(&world->hfData[hf]);
    }
    for (int32_t m = 0;
         world->meshData != NULL && world->meshBvh != NULL && m < world->meshCapacity; ++m)
    {
        m3MeshDataFree(&world->meshData[m]);
        m3MeshBvhFree(&world->meshBvh[m]);
    }
    for (int32_t v = 0; world->voxelSurface != NULL && v < world->voxelCapacity; ++v)
    {
        m3MeshBvhFree(&world->voxelSurface[v].bvh);
    }
    m3StateFree(world);
    m3IdPoolDestroy(&world->bodyPool);
    m3IdPoolDestroy(&world->shapePool);
    m3IdPoolDestroy(&world->hullPool);
    m3IdPoolDestroy(&world->jointPool);
    m3IdPoolDestroy(&world->hfPool);
    m3IdPoolDestroy(&world->waterPool);
    m3IdPoolDestroy(&world->charPool);
    m3IdPoolDestroy(&world->vehPool);
    m3IdPoolDestroy(&world->softPool);
    m3IdPoolDestroy(&world->meshPool);
    m3IdPoolDestroy(&world->voxelPool);
    m3TreeDestroy(&world->tree);
    m3StackDestroy(&world->scratch);
    m3Free(world);
}

m3WorldId m3CreateWorld(const m3WorldDef* def)
{
    m3WorldId nullId = {0, 0};
    if (def == NULL || def->internalValue != M3_WORLD_COOKIE || def->bodyCapacity <= 0 ||
        def->shapeCapacity <= 0 || def->meshCapacity <= 0 || def->jointCapacity <= 0 ||
        def->voxelCapacity <= 0 || def->characterCapacity <= 0 || def->vehicleCapacity <= 0 ||
        def->softBodyCapacity <= 0 || def->workerCount <= 0 || def->shapeCapacity > INT32_MAX / 8 ||
        def->voxelCapacity > INT32_MAX / 6 ||
        (def->enqueueTask == NULL) != (def->finishTask == NULL) || !m3FiniteV3(def->gravity) ||
        !m3FiniteF(def->contactHertz) || def->contactHertz <= 0.0f ||
        !m3FiniteF(def->contactDampingRatio) || def->contactDampingRatio <= 0.0f ||
        !m3FiniteF(def->contactPushMaxSpeed) || def->contactPushMaxSpeed <= 0.0f ||
        !m3FiniteF(def->restitutionThreshold) || def->restitutionThreshold < 0.0f ||
        !m3FiniteF(def->maximumLinearSpeed) || def->maximumLinearSpeed <= 0.0f ||
        !m3FiniteF(def->hitEventThreshold) || def->hitEventThreshold < 0.0f)
    {
        m3Refuse(NULL, m3_errorInvalid);
        // User-input validation is contract, not invariant: the API
        // promises a null id for a bad def (tests exercise this), so
        // no assert here. Asserts guard states that cannot happen.
        return nullId;
    }

    int32_t slot = -1;
    for (int32_t i = 0; i < M3_MAX_WORLDS; ++i)
    {
        if (s_worlds[i] == NULL)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        m3Refuse(NULL, m3_errorCapacity);
        return nullId; // table exhausted: loud, never silent, and a
                       // capacity refusal is contract, not invariant
    }

    m3World* world = (m3World*)m3AllocZeroed((int32_t)sizeof(m3World));
    if (world == NULL)
    {
        m3Refuse(NULL, m3_errorCapacity);
        return nullId; // out of memory
    }
    int32_t cap = def->bodyCapacity;
    world->gravity = def->gravity;
    world->contactHertz = def->contactHertz;
    world->contactDampingRatio = def->contactDampingRatio;
    world->contactPushMaxSpeed = def->contactPushMaxSpeed;
    world->restitutionThreshold = def->restitutionThreshold;
    world->maximumLinearSpeed = def->maximumLinearSpeed;
    // Not a def field: the def cookie stays put under 1.x.
    // Hosts tune it through the journaled setter.
    world->maximumAngularSpeed = M3_MAX_ANGULAR_SPEED_DEFAULT;
    world->sleepEnabled = def->enableSleeping != 0 ? 1 : 0;
    world->continuousEnabled = def->enableContinuous != 0 ? 1 : 0;
    world->hitEventThreshold = def->hitEventThreshold;
    world->preSolveFn = NULL;
    world->preSolveContext = NULL;
    world->lastInvH = 0.0f;
    world->windDir = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->windSpeed = 0.0f;
    world->windGustHertz = 0.0f;
    world->windGustScale = 0.0f;
    world->windPhase = 0.0f;
    world->bodyCapacity = cap;
    world->shapeCapacity = def->shapeCapacity;
    world->meshCapacity = def->meshCapacity;
    world->voxelCapacity = def->voxelCapacity;
    world->characterCapacity = def->characterCapacity;
    world->vehicleCapacity = def->vehicleCapacity;
    world->softBodyCapacity = def->softBodyCapacity;
    world->jointCapacity = def->jointCapacity;
    world->workerCount = def->workerCount;
    world->enqueueTask = def->enqueueTask;
    world->finishTask = def->finishTask;
    world->userTaskContext = def->userTaskContext;
    world->generation = s_worldGenerations[slot];
    world->worldIndex0 = (uint16_t)slot;
    world->pairCapacity = 8 * def->shapeCapacity;
    if (!m3StateAllocate(world))
    {
        goto allocFailed;
    }
    world->bodyPool = m3IdPoolCreate(cap);

    for (int32_t i = 0; i < cap; ++i)
    {
        world->bodyIsland[i] = -1; // observer label, no island yet
        world->bodyShapeHead[i] = -1;
    }

    int32_t shapeCap = def->shapeCapacity;
    world->shapePool = m3IdPoolCreate(shapeCap);
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeBody[i] = -1;
        world->shapeNext[i] = -1;
    }

    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeHullIndex[i] = -1;
    }
    world->hullPool = m3IdPoolCreate(shapeCap);
    world->jointPool = m3IdPoolCreate(def->jointCapacity);
    world->hfPool = m3IdPoolCreate(def->shapeCapacity);
    for (int32_t hf = 0; hf < def->shapeCapacity; ++hf)
    {
        world->shapeHfIndex[hf] = -1;
    }
    world->waterPool = m3IdPoolCreate(M3_MAX_WATER_VOLUMES);
    world->charPool = m3IdPoolCreate(def->characterCapacity);
    world->vehPool = m3IdPoolCreate(def->vehicleCapacity);
    world->softPool = m3IdPoolCreate(def->softBodyCapacity);
    for (int32_t v = 0; v < def->vehicleCapacity; ++v)
    {
        world->vehChassis[v] = -1;
    }
    for (int32_t i = 0; i < def->characterCapacity; ++i)
    {
        world->charBody[i] = -1;
    }
    for (int32_t i = 0; i < def->jointCapacity; ++i)
    {
        world->jointBodyA[i] = -1;
        world->jointBodyB[i] = -1;
        world->jointNextA[i] = -1;
        world->jointNextB[i] = -1;
    }
    for (int32_t i = 0; i < cap; ++i)
    {
        world->bodyJointHead[i] = -1;
    }
    world->meshPool = m3IdPoolCreate(def->meshCapacity);
    world->voxelPool = m3IdPoolCreate(def->voxelCapacity);
    for (int32_t i = 0; i < def->voxelCapacity; ++i)
    {
        world->voxelShape[i] = -1;
    }
    for (int32_t i = 0; i < def->voxelCapacity * 6; ++i)
    {
        world->voxelNeighbors[i] = -1;
    }
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeVoxelIndex[i] = -1;
    }
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeMeshIndex[i] = -1;
    }

    world->tree = m3TreeCreate(2 * shapeCap);
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->proxyIds[i] = M3_TREE_NULL;
    }

    world->hitEventCount = 0;
    world->hitEventsDropped = 0;
    world->moveEventCount = 0;
    world->jointBreakEventCount = 0;
    world->beginEventCount = 0;
    world->endEventCount = 0;
    world->pairCount = 0;

    // Step scratch: grows between steps on m3_errorCapacity, never
    // mid-step. 256 KiB is generous for the 2a sphere world.
    world->scratch = m3StackCreate(256 * 1024);
    // The slot pools and the proxy tree allocate outside the state table;
    // their footprints are closed-form and join the ledger here.
    {
        int64_t poolBytes = 0;
        int32_t poolCaps[] = {cap,
                              shapeCap,
                              shapeCap,
                              def->jointCapacity,
                              def->shapeCapacity,
                              M3_MAX_WATER_VOLUMES,
                              def->characterCapacity,
                              def->vehicleCapacity,
                              def->softBodyCapacity,
                              def->meshCapacity,
                              def->voxelCapacity};
        for (int32_t p = 0; p < (int32_t)(sizeof(poolCaps) / sizeof(poolCaps[0])); ++p)
        {
            poolBytes += (int64_t)poolCaps[p] *
                         (int64_t)(sizeof(uint16_t) + sizeof(uint8_t) + sizeof(int32_t));
        }
        world->memoryBytes += poolBytes;
        world->memoryBytes += 2LL * shapeCap * (int64_t)sizeof(m3TreeNode);
    }

    // The pools, the tree and the scratch report a refusal as zero
    // capacity rather than jumping, so check them once here.
    if (world->bodyPool.capacity == 0 || world->shapePool.capacity == 0 ||
        world->hullPool.capacity == 0 || world->jointPool.capacity == 0 ||
        world->hfPool.capacity == 0 || world->waterPool.capacity == 0 ||
        world->charPool.capacity == 0 || world->vehPool.capacity == 0 ||
        world->softPool.capacity == 0 || world->meshPool.capacity == 0 ||
        world->voxelPool.capacity == 0 || world->tree.capacity == 0 || world->scratch.capacity == 0)
    {
        goto allocFailed;
    }

    s_worlds[slot] = world;
    m3WorldId id = {slot + 1, world->generation};
    return id;

allocFailed:
    // Out of memory or an unrepresentable size: release what exists
    // and refuse. Nothing was registered, so the slot stays free.
    FreeWorldStorage(world);
    m3Refuse(NULL, m3_errorCapacity);
    return nullId;
}

void m3DestroyWorld(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale or foreign id: contract, not invariant
    }
    int32_t slot = world->worldIndex0;

    FreeWorldStorage(world);

    s_worlds[slot] = NULL;
    s_worldGenerations[slot] += 1;
}

bool m3World_IsValid(m3WorldId worldId)
{
    return m3WorldFromId(worldId) != NULL;
}

// --- Tuning knobs -----------------------------------------------------

void m3SetGravityInternal(m3World* world, m3Vec3 gravity)
{
    world->gravity = gravity;
}

void m3SetContactTuningInternal(m3World* world, float hertz, float dampingRatio, float pushSpeed)
{
    world->contactHertz = hertz;
    world->contactDampingRatio = dampingRatio;
    world->contactPushMaxSpeed = pushSpeed;
}

void m3SetRestitutionThresholdInternal(m3World* world, float value)
{
    world->restitutionThreshold = value;
}

void m3SetMaximumLinearSpeedInternal(m3World* world, float value)
{
    world->maximumLinearSpeed = value;
}

void m3SetMaximumAngularSpeedInternal(m3World* world, float value)
{
    world->maximumAngularSpeed = value;
}

void m3EnableSleepingInternal(m3World* world, int32_t on)
{
    world->sleepEnabled = on != 0 ? 1 : 0;
    if (on == 0)
    {
        // The reference wakes every sleeping set when sleeping turns
        // off; nothing may keep napping through the new regime.
        int32_t maxBody = world->bodyPool.maxIndex;
        for (int32_t i = 0; i < maxBody; ++i)
        {
            if (world->bodyPool.alive[i] != 0 && world->awake[i] == 0 &&
                world->types[i] == (uint8_t)m3_dynamicBody)
            {
                m3SetAwakeInternal(world, i, 1);
            }
        }
    }
}

void m3EnableContinuousInternal(m3World* world, int32_t on)
{
    world->continuousEnabled = on != 0 ? 1 : 0;
}

void m3World_SetGravity(m3WorldId worldId, m3Vec3 gravity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteV3(gravity))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetGravity, &gravity, (int32_t)sizeof(gravity));
    }
    m3SetGravityInternal(world, gravity);
}

m3Vec3 m3World_GetGravity(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    return world != NULL ? world->gravity : zero;
}

void m3World_SetContactTuning(m3WorldId worldId, float hertz, float dampingRatio,
                              float pushMaxSpeed)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(hertz) || hertz <= 0.0f || !m3FiniteF(dampingRatio) ||
        dampingRatio <= 0.0f || !m3FiniteF(pushMaxSpeed) || pushMaxSpeed <= 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            float hertz;
            float dampingRatio;
            float pushSpeed;
        } record;
        memset(&record, 0, sizeof(record));
        record.hertz = hertz;
        record.dampingRatio = dampingRatio;
        record.pushSpeed = pushMaxSpeed;
        m3JournalRecord(world, m3_opSetContactTuning, &record, (int32_t)sizeof(record));
    }
    m3SetContactTuningInternal(world, hertz, dampingRatio, pushMaxSpeed);
}

void m3World_SetRestitutionThreshold(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(value) || value < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetRestitutionThreshold, &value, (int32_t)sizeof(value));
    }
    m3SetRestitutionThresholdInternal(world, value);
}

void m3World_SetMaximumLinearSpeed(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(value) || value <= 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetMaximumLinearSpeed, &value, (int32_t)sizeof(value));
    }
    m3SetMaximumLinearSpeedInternal(world, value);
}

void m3World_SetMaximumAngularSpeed(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(value) || value <= 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetMaximumAngularSpeed, &value, (int32_t)sizeof(value));
    }
    m3SetMaximumAngularSpeedInternal(world, value);
}

// Live slots without a scan: the pool hands out from the free queue
// or bumps maxIndex, and retirement is the only other exit.
static int32_t PoolLive(const m3IdPool* pool)
{
    return pool->maxIndex - pool->freeCount - pool->retiredCount;
}

m3MemoryUsage m3World_MemoryUsage(m3WorldId worldId)
{
    m3MemoryUsage usage;
    memset(&usage, 0, sizeof(usage));
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return usage;
    }
    usage.persistentBytes = world->memoryBytes;
    // Count-derived content, summed live so it cannot drift: mesh
    // payloads plus their derived BVHs, and heightfield samples.
    for (int32_t m = 0; m < world->meshPool.maxIndex; ++m)
    {
        if (world->meshPool.alive[m] == 0)
        {
            continue;
        }
        const m3MeshData* mesh = &world->meshData[m];
        usage.contentBytes += (int64_t)mesh->vertexCount * (int64_t)sizeof(m3Vec3);
        usage.contentBytes += 3LL * mesh->triangleCount * (int64_t)sizeof(uint16_t);
        usage.contentBytes += 2LL * mesh->triangleCount; // edge flags + materials
        const m3MeshBvh* bvh = &world->meshBvh[m];
        usage.contentBytes += (int64_t)bvh->nodeCount * (int64_t)sizeof(m3MeshBvhNode);
        usage.contentBytes += (int64_t)mesh->triangleCount * (int64_t)sizeof(uint16_t); // order
    }
    for (int32_t h = 0; h < world->shapeCapacity; ++h)
    {
        const m3HeightFieldData* hf = &world->hfData[h];
        usage.contentBytes += (int64_t)hf->nx * (int64_t)hf->nz * (int64_t)sizeof(float);
    }
    usage.scratchCapacity = world->scratch.capacity;
    usage.scratchPeak = world->lastScratchPeak;
    return usage;
}

m3Counters m3World_GetCounters(m3WorldId worldId)
{
    m3Counters counters;
    memset(&counters, 0, sizeof(counters));
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return counters;
    }
    counters.bodyCount = PoolLive(&world->bodyPool);
    counters.shapeCount = PoolLive(&world->shapePool);
    counters.jointCount = PoolLive(&world->jointPool);
    counters.contactCount = world->pairCount;
    counters.characterCount = PoolLive(&world->charPool);
    counters.vehicleCount = PoolLive(&world->vehPool);
    counters.softBodyCount = PoolLive(&world->softPool);
    counters.voxelChunkCount = PoolLive(&world->voxelPool);
    counters.hullCount = PoolLive(&world->hullPool);
    counters.meshCount = PoolLive(&world->meshPool);
    int32_t awake = 0;
    int32_t maxBody = world->bodyPool.maxIndex;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] != 0 && world->types[i] == (uint8_t)m3_dynamicBody &&
            world->awake[i] != 0)
        {
            awake += 1;
        }
    }
    counters.awakeCount = awake;
    counters.islandCount = world->lastIslandCount;
    counters.colorCount = world->lastColorCount;
    counters.treeHeight =
        world->tree.root != M3_TREE_NULL ? world->tree.nodes[world->tree.root].height : 0;
    counters.scratchPeak = world->lastScratchPeak;
    counters.scratchCapacity = world->scratch.capacity;
    counters.snapshotBytes = m3World_SnapshotSize(worldId);
    m3DebugAllocCounts(&counters.allocCalls, &counters.freeCalls);
    counters.misuse = m3MisuseCount(world);
    return counters;
}

m3Profile m3World_GetProfile(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Profile zero;
        memset(&zero, 0, sizeof(zero));
        return zero;
    }
    return world->profile;
}

void m3World_EnableSleeping(m3WorldId worldId, bool flag)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        int32_t on = flag ? 1 : 0;
        m3JournalRecord(world, m3_opEnableSleeping, &on, (int32_t)sizeof(on));
    }
    m3EnableSleepingInternal(world, flag ? 1 : 0);
}

bool m3World_IsSleepingEnabled(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL && world->sleepEnabled != 0;
}

void m3World_EnableContinuous(m3WorldId worldId, bool flag)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        int32_t on = flag ? 1 : 0;
        m3JournalRecord(world, m3_opEnableContinuous, &on, (int32_t)sizeof(on));
    }
    m3EnableContinuousInternal(world, flag ? 1 : 0);
}

bool m3World_IsContinuousEnabled(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL && world->continuousEnabled != 0;
}

// --- Events -----------------------------------------------------------

void m3SetHitEventThresholdInternal(m3World* world, float value)
{
    world->hitEventThreshold = value;
}

void m3World_SetHitEventThreshold(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(value) || value < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetHitEventThreshold, &value, (int32_t)sizeof(value));
    }
    m3SetHitEventThresholdInternal(world, value);
}

const m3HitEvent* m3World_HitEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->hitEventCount;
    return world->hitEvents;
}

int32_t m3World_HitEventsDropped(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL ? world->hitEventsDropped : 0;
}

const m3BodyMoveEvent* m3World_BodyMoveEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->moveEventCount;
    return world->moveEvents;
}

const m3JointBreakEvent* m3World_JointBreakEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->jointBreakEventCount;
    return world->jointBreakEvents;
}

void m3SetWindInternal(m3World* world, m3Vec3 dir, float speed, float gustHertz, float gustScale)
{
    world->windDir = dir;
    world->windSpeed = speed;
    world->windGustHertz = gustHertz;
    world->windGustScale = gustScale;
    // The phase deliberately SURVIVES a retune: the wave continues.
}

void m3World_SetWind(m3WorldId worldId, m3Vec3 direction, float speed, float gustHertz,
                     float gustScale)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteV3(direction) || !m3FiniteF(speed) || speed < 0.0f ||
        !m3FiniteF(gustHertz) || gustHertz < 0.0f || !m3FiniteF(gustScale) || gustScale < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (speed > 0.0f)
    {
        m3real len2 = m3Dot3(direction, direction);
        if (len2 < 0.99f || len2 > 1.01f)
        {
            return; // a blowing wind demands a near-unit direction
        }
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3Vec3 dir;
            float speed;
            float gustHertz;
            float gustScale;
        } record;
        memset(&record, 0, sizeof(record));
        record.dir = direction;
        record.speed = speed;
        record.gustHertz = gustHertz;
        record.gustScale = gustScale;
        m3JournalRecord(world, m3_opSetWind, &record, (int32_t)sizeof(record));
    }
    m3SetWindInternal(world, direction, speed, gustHertz, gustScale);
}

void m3AppendJointBreakEvent(m3World* world, m3JointId joint)
{
    // Capacity is jointCapacity: at most every joint breaks once.
    world->jointBreakEvents[world->jointBreakEventCount].joint = joint;
    world->jointBreakEventCount += 1;
}

void m3World_SetPreSolveCallback(m3WorldId worldId, m3PreSolveFn* fn, void* context)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    world->preSolveFn = fn;
    world->preSolveContext = context;
}

// --- Journal ---------------------------------------------------------------

void m3RebuildBroadphaseInternal(m3World* world)
{
    // Collect the live proxies in shape-slot order (the canonical
    // list), carrying their CURRENT fat bounds: fatness is state,
    // and preserving it keeps every downstream pair decision
    // exactly where it was.
    int32_t maxShape = world->shapePool.maxIndex;
    int32_t count = 0;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapePool.alive[s] != 0 && world->proxyIds[s] >= 0)
        {
            count += 1;
        }
    }
    if (count == 0)
    {
        return;
    }
    double (*los)[3] = (double (*)[3])m3AllocZeroed(count * 3 * (int32_t)sizeof(double));
    double (*his)[3] = (double (*)[3])m3AllocZeroed(count * 3 * (int32_t)sizeof(double));
    int32_t* uds = (int32_t*)m3AllocZeroed(count * (int32_t)sizeof(int32_t));
    int32_t* outNodes = (int32_t*)m3AllocZeroed(count * (int32_t)sizeof(int32_t));
    if (los == NULL || his == NULL || uds == NULL || outNodes == NULL)
    {
        m3Free(los);
        m3Free(his);
        m3Free(uds);
        m3Free(outNodes);
        return; // no memory: the old tree stays, correct either way
    }
    int32_t n = 0;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapePool.alive[s] == 0 || world->proxyIds[s] < 0)
        {
            continue;
        }
        const m3TreeNode* leaf = &world->tree.nodes[world->proxyIds[s]];
        for (int32_t k = 0; k < 3; ++k)
        {
            los[n][k] = leaf->lo[k];
            his[n][k] = leaf->hi[k];
        }
        uds[n] = s;
        n += 1;
    }
    if (m3TreeRebuild(&world->tree, los, his, uds, n, outNodes))
    {
        for (int32_t i = 0; i < n; ++i)
        {
            world->proxyIds[uds[i]] = outNodes[i];
        }
    }
    m3Free(los);
    m3Free(his);
    m3Free(uds);
    m3Free(outNodes);
}

void m3World_RebuildBroadphase(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        int32_t zero = 0;
        m3JournalRecord(world, m3_opRebuildBroadphase, &zero, 4);
    }
    m3RebuildBroadphaseInternal(world);
}

#define M3_WATER_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3WaterVolumeDef) << 8) ^ 7))

m3WaterVolumeDef m3DefaultWaterVolumeDef(void)
{
    m3WaterVolumeDef def;
    memset(&def, 0, sizeof(def));
    def.hi = (m3Pos3){1.0, 1.0, 1.0};
    def.density = 1000.0f;
    def.linearDrag = 2.0f;
    def.angularDrag = 1.0f;
    def.internalValue = M3_WATER_COOKIE;
    return def;
}

static bool WakeInBoxFn(int32_t shape, void* context)
{
    m3World* world = (m3World*)context;
    int32_t body = world->shapeBody[shape];
    if (world->types[body] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, body, 1);
    }
    return true;
}

static void WakeAroundWater(m3World* world, int32_t slot)
{
    // The tide moves things: sleepers touching the volume wake on
    // create AND destroy (without water under it, a sleeper falls).
    double lo[3] = {world->waterLo[slot].x, world->waterLo[slot].y, world->waterLo[slot].z};
    double hi[3] = {world->waterHi[slot].x, world->waterHi[slot].y, world->waterHi[slot].z};
    m3TreeQuery(&world->tree, lo, hi, WakeInBoxFn, world);
}

int32_t m3CreateWaterVolumeInternal(m3World* world, const m3WaterVolumeDef* def)
{
    // The full wall, here because replay hands this function
    // raw journal bytes.
    if (!m3FinitePos3(def->lo) || !m3FinitePos3(def->hi) || !(def->hi.x > def->lo.x) ||
        !(def->hi.y > def->lo.y) || !(def->hi.z > def->lo.z) || !m3FiniteF(def->density) ||
        !(def->density > 0.0f) || !m3FiniteF(def->linearDrag) || def->linearDrag < 0.0f ||
        !m3FiniteF(def->angularDrag) || def->angularDrag < 0.0f || !m3FiniteV3(def->flow))
    {
        return -1;
    }
    int32_t slot = m3IdPoolAlloc(&world->waterPool);
    if (slot < 0)
    {
        return -1; // all 8 slots taken: loud at the caller
    }
    world->waterLo[slot] = def->lo;
    world->waterHi[slot] = def->hi;
    world->waterDensity[slot] = def->density;
    world->waterLinDrag[slot] = def->linearDrag;
    world->waterAngDrag[slot] = def->angularDrag;
    world->waterFlow[slot] = def->flow;
    WakeAroundWater(world, slot);
    return slot;
}

void m3DestroyWaterVolumeInternal(m3World* world, int32_t slot)
{
    WakeAroundWater(world, slot);
    world->waterLo[slot] = (m3Pos3){0.0, 0.0, 0.0};
    world->waterHi[slot] = (m3Pos3){0.0, 0.0, 0.0};
    world->waterDensity[slot] = 0.0f;
    world->waterLinDrag[slot] = 0.0f;
    world->waterAngDrag[slot] = 0.0f;
    world->waterFlow[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3IdPoolFree(&world->waterPool, slot);
}

m3WaterVolumeId m3CreateWaterVolume(m3WorldId worldId, const m3WaterVolumeDef* def)
{
    m3WaterVolumeId null = {0, 0, 0};
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_WATER_COOKIE)
    {
        m3Refuse(world, m3_errorInvalid);
        return null;
    }
    int32_t slot = m3CreateWaterVolumeInternal(world, def);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return null;
    }
    m3WaterVolumeId id = {slot + 1, world->worldIndex0, world->waterPool.generations[slot]};
    if (world->journalActive != 0)
    {
        struct
        {
            m3WaterVolumeDef def;
            m3WaterVolumeId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateWaterVolume, &record, (int32_t)sizeof(record));
    }
    return id;
}

static int32_t WaterSlot(const m3World* world, m3WaterVolumeId id)
{
    int32_t index = id.index1 - 1;
    if (world == NULL || id.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->waterPool, index, id.generation))
    {
        return -1;
    }
    return index;
}

bool m3WaterVolume_IsValid(m3WaterVolumeId id)
{
    m3World* world = m3WorldFromIndex0(id.world0);
    return world != NULL && WaterSlot(world, id) >= 0;
}

void m3DestroyWaterVolume(m3WaterVolumeId id)
{
    m3World* world = m3WorldFromIndex0(id.world0);
    int32_t slot = world != NULL ? WaterSlot(world, id) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyWaterVolume, &id, (int32_t)sizeof(id));
    }
    m3DestroyWaterVolumeInternal(world, slot);
}

void m3JournalRecord(m3World* world, int32_t op, const void* payload, int32_t bytes)
{
    if (world->journalActive == 0)
    {
        return;
    }
    int32_t need = 8 + bytes;
    if (world->journalCursor + need > world->journalCapacity)
    {
        // Loud overflow: latch, stop recording, End reports -1.
        world->journalOverflow = 1;
        world->journalActive = 0;
        return;
    }
    uint8_t* out = world->journalBuffer + world->journalCursor;
    memcpy(out, &op, 4);
    memcpy(out + 4, &bytes, 4);
    memcpy(out + 8, payload, (size_t)bytes);
    world->journalCursor += need;
}

void m3JournalAbandon(m3World* world)
{
    // The op could not be encoded (no memory for its payload). A tape
    // missing an op would replay a different world, so the recording
    // fails the same loud way an overflow does.
    world->journalOverflow = 1;
    world->journalActive = 0;
}

bool m3World_JournalBegin(m3WorldId worldId, void* buffer, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || buffer == NULL || capacity < 8 || world->journalActive != 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // contract, not invariant
    }
    world->journalBuffer = (uint8_t*)buffer;
    world->journalCapacity = capacity;
    world->journalCursor = 0;
    world->journalActive = 1;
    world->journalOverflow = 0;
    return true;
}

int32_t m3World_JournalEnd(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1; // contract, not invariant
    }
    int32_t bytes = world->journalOverflow != 0 ? -1 : world->journalCursor;
    world->journalBuffer = NULL;
    world->journalCapacity = 0;
    world->journalCursor = 0;
    world->journalActive = 0;
    world->journalOverflow = 0;
    return bytes;
}

const m3ContactEvent* m3World_SensorBeginEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->sensorBeginEventCount;
    return world->sensorBeginEvents;
}

const m3ContactEvent* m3World_SensorEndEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->sensorEndEventCount;
    return world->sensorEndEvents;
}

const m3FragmentEvent* m3World_FragmentEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    if (count != NULL)
    {
        *count = world->fragmentEventCount;
    }
    return world->fragmentEvents;
}

const uint16_t* m3World_FragmentRecipe(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    if (count != NULL)
    {
        *count = world->fragmentRecipeCount;
    }
    return world->fragmentRecipe;
}

int32_t m3World_FragmentEventsDropped(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL ? world->fragmentDropped : 0;
}

const m3ContactEvent* m3World_ContactBeginEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->beginEventCount;
    return world->beginEvents;
}

const m3ContactEvent* m3World_ContactEndEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->endEventCount;
    return world->endEvents;
}

bool m3World_JournalReplay(m3WorldId worldId, const void* data, int32_t size)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || data == NULL || size < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // contract, not invariant
    }
    // Atomic replay: the world either takes the whole session
    // or none of it. A pre-replay snapshot backs out any partial
    // application on refusal, so a corrupted or truncated journal
    // can never leave a half-built world behind.
    int32_t snapBytes = m3World_SnapshotSize(worldId);
    uint8_t* snap = (uint8_t*)m3AllocZeroed(snapBytes);
    if (snap == NULL)
    {
        m3Refuse(world, m3_errorCapacity);
        return false; // no memory for the guarantee means no replay
    }
    if (m3World_Snapshot(worldId, snap, snapBytes) != snapBytes)
    {
        m3Free(snap);
        m3Refuse(world, m3_errorCapacity);
        return false;
    }
    bool ok = m3JournalReplayApply(world, data, size);
    if (!ok)
    {
        bool restored = m3World_Restore(worldId, snap, snapBytes);
        M3_ASSERT(restored); // our own snapshot must restore: invariant
        (void)restored;
        m3Refuse(world, m3_errorInvalid);
    }
    m3Free(snap);
    return ok;
}
