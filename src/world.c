// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World lifecycle and the journal: a static world table with
// generations, def-cookie validation, SoA arrays described once in the
// state table (world_state.c), and a journal whose replay goes through
// the same internal functions the public API uses, the same shape as
// maul2d's.

#include "world.h"
#include "body.h"
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

m3World* m3WorldFromTag(uint16_t tag)
{
    m3World* world = s_worlds[tag & ((1u << M3_WORLD_SLOT_BITS) - 1u)];
    if (world == NULL || world->idWorld != tag)
    {
        m3Refuse(NULL, m3_errorInvalid); // an id of a world that is gone
        return NULL;
    }
    return world;
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
    def.maximumAngularSpeed = M3_MAX_ANGULAR_SPEED_DEFAULT;
    def.enableSleeping = true;
    def.enableContinuous = true;
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
    for (int32_t hf = 0; world->heightFields.hfData != NULL && hf < world->shapes.shapeCapacity;
         ++hf)
    {
        m3HeightFieldDataFree(&world->heightFields.hfData[hf]);
    }
    for (int32_t m = 0; world->meshes.meshData != NULL && world->meshes.meshBvh != NULL &&
                        m < world->meshes.meshCapacity;
         ++m)
    {
        m3MeshDataFree(&world->meshes.meshData[m]);
        m3MeshBvhFree(&world->meshes.meshBvh[m]);
    }
    for (int32_t v = 0; world->voxels.voxelSurface != NULL && v < world->voxels.voxelCapacity; ++v)
    {
        m3MeshBvhFree(&world->voxels.voxelSurface[v].bvh);
    }
    m3StateFree(world);
    m3IdPoolDestroy(&world->bodies.bodyPool);
    m3IdPoolDestroy(&world->shapes.shapePool);
    m3IdPoolDestroy(&world->hulls.hullPool);
    m3IdPoolDestroy(&world->joints.jointPool);
    m3IdPoolDestroy(&world->heightFields.hfPool);
    m3IdPoolDestroy(&world->water.waterPool);
    m3IdPoolDestroy(&world->characters.charPool);
    m3IdPoolDestroy(&world->vehicles.vehPool);
    m3IdPoolDestroy(&world->softBodies.softPool);
    m3IdPoolDestroy(&world->meshes.meshPool);
    m3IdPoolDestroy(&world->voxels.voxelPool);
    m3TreeDestroy(&world->broadphase.tree);
    m3StackDestroy(&world->scratch);
    m3Free(world);
}

// User-input validation is contract, not invariant: a bad def is a
// refusal, never an assert.
static bool ValidWorldDef(const m3WorldDef* def)
{
    if (def == NULL || def->internalValue != M3_WORLD_COOKIE)
    {
        return false;
    }
    bool capacities = def->bodyCapacity > 0 && def->shapeCapacity > 0 && def->meshCapacity > 0 &&
                      def->jointCapacity > 0 && def->voxelCapacity > 0 &&
                      def->characterCapacity > 0 && def->vehicleCapacity > 0 &&
                      def->softBodyCapacity > 0 && def->workerCount > 0 &&
                      def->shapeCapacity <= INT32_MAX / 8 && def->voxelCapacity <= INT32_MAX / 6;
    bool tasks = (def->enqueueTask == NULL) == (def->finishTask == NULL);
    bool tuning = m3FiniteV3(def->gravity) && m3FiniteF(def->contactHertz) &&
                  def->contactHertz > 0.0f && m3FiniteF(def->contactDampingRatio) &&
                  def->contactDampingRatio > 0.0f && m3FiniteF(def->contactPushMaxSpeed) &&
                  def->contactPushMaxSpeed > 0.0f && m3FiniteF(def->restitutionThreshold) &&
                  def->restitutionThreshold >= 0.0f && m3FiniteF(def->maximumLinearSpeed) &&
                  def->maximumLinearSpeed > 0.0f && m3FiniteF(def->maximumAngularSpeed) &&
                  def->maximumAngularSpeed > 0.0f && m3FiniteF(def->hitEventThreshold) &&
                  def->hitEventThreshold >= 0.0f;
    return capacities && tasks && tuning;
}

// Copies the def's settings and capacities into a zeroed world.
static void ApplyWorldDef(m3World* world, const m3WorldDef* def)
{
    world->gravity = def->gravity;
    world->contactHertz = def->contactHertz;
    world->contactDampingRatio = def->contactDampingRatio;
    world->contactPushMaxSpeed = def->contactPushMaxSpeed;
    world->restitutionThreshold = def->restitutionThreshold;
    world->maximumLinearSpeed = def->maximumLinearSpeed;
    world->maximumAngularSpeed = def->maximumAngularSpeed;
    world->sleepEnabled = def->enableSleeping ? 1 : 0;
    world->continuousEnabled = def->enableContinuous ? 1 : 0;
    world->hitEventThreshold = def->hitEventThreshold;
    world->bodies.bodyCapacity = def->bodyCapacity;
    world->shapes.shapeCapacity = def->shapeCapacity;
    world->meshes.meshCapacity = def->meshCapacity;
    world->voxels.voxelCapacity = def->voxelCapacity;
    world->characters.characterCapacity = def->characterCapacity;
    world->vehicles.vehicleCapacity = def->vehicleCapacity;
    world->softBodies.softBodyCapacity = def->softBodyCapacity;
    world->joints.jointCapacity = def->jointCapacity;
    world->workerCount = def->workerCount;
    world->enqueueTask = def->enqueueTask;
    world->finishTask = def->finishTask;
    world->userTaskContext = def->userTaskContext;
    world->contacts.pairCapacity = 8 * def->shapeCapacity;
}

// Sets every link and back-reference of fresh state to "none".
static void ClearLinks(m3World* world)
{
    for (int32_t i = 0; i < world->bodies.bodyCapacity; ++i)
    {
        world->bodies.bodyIsland[i] = -1; // observer label, no island yet
        world->bodies.bodyShapeHead[i] = -1;
        world->joints.bodyJointHead[i] = -1;
    }
    for (int32_t i = 0; i < world->shapes.shapeCapacity; ++i)
    {
        world->shapes.shapeBody[i] = -1;
        world->shapes.shapeNext[i] = -1;
        world->shapes.shapeHullIndex[i] = -1;
        world->shapes.shapeHfIndex[i] = -1;
        world->shapes.shapeVoxelIndex[i] = -1;
        world->shapes.shapeMeshIndex[i] = -1;
        world->broadphase.proxyIds[i] = M3_TREE_NULL;
    }
    for (int32_t v = 0; v < world->vehicles.vehicleCapacity; ++v)
    {
        world->vehicles.vehChassis[v] = -1;
    }
    for (int32_t i = 0; i < world->characters.characterCapacity; ++i)
    {
        world->characters.charBody[i] = -1;
    }
    for (int32_t i = 0; i < world->joints.jointCapacity; ++i)
    {
        world->joints.jointBodyA[i] = -1;
        world->joints.jointBodyB[i] = -1;
        world->joints.jointNextA[i] = -1;
        world->joints.jointNextB[i] = -1;
    }
    for (int32_t i = 0; i < world->voxels.voxelCapacity; ++i)
    {
        world->voxels.voxelShape[i] = -1;
    }
    for (int32_t i = 0; i < world->voxels.voxelCapacity * 6; ++i)
    {
        world->voxels.voxelNeighbors[i] = -1;
    }
}

// The slot pools, the proxy tree and the step scratch live outside the
// state table. Each reports a refusal as zero capacity; their closed-form
// footprints join the memory total.
static bool CreatePools(m3World* world)
{
    m3IdPool* pools[] = {
        &world->bodies.bodyPool,     &world->shapes.shapePool,    &world->hulls.hullPool,
        &world->joints.jointPool,    &world->heightFields.hfPool, &world->water.waterPool,
        &world->characters.charPool, &world->vehicles.vehPool,    &world->softBodies.softPool,
        &world->meshes.meshPool,     &world->voxels.voxelPool};
    int32_t caps[] = {world->bodies.bodyCapacity,          world->shapes.shapeCapacity,
                      world->shapes.shapeCapacity,         world->joints.jointCapacity,
                      world->shapes.shapeCapacity,         M3_MAX_WATER_VOLUMES,
                      world->characters.characterCapacity, world->vehicles.vehicleCapacity,
                      world->softBodies.softBodyCapacity,  world->meshes.meshCapacity,
                      world->voxels.voxelCapacity};
    bool ok = true;
    for (int32_t p = 0; p < (int32_t)(sizeof(caps) / sizeof(caps[0])); ++p)
    {
        *pools[p] = m3IdPoolCreate(caps[p]);
        ok = ok && pools[p]->capacity != 0;
        world->memoryBytes +=
            (int64_t)caps[p] * (int64_t)(sizeof(uint16_t) + sizeof(uint8_t) + sizeof(int32_t));
    }
    int32_t shapeCap = world->shapes.shapeCapacity;
    world->broadphase.tree = m3TreeCreate(2 * shapeCap);
    world->memoryBytes += 2LL * shapeCap * (int64_t)sizeof(m3TreeNode);
    // Step scratch grows between steps on m3_errorCapacity, never mid-step.
    world->scratch = m3StackCreate(256 * 1024);
    return ok && world->broadphase.tree.capacity != 0 && world->scratch.capacity != 0;
}

m3WorldId m3CreateWorld(const m3WorldDef* def)
{
    m3WorldId nullId = {0, 0};
    // Before any solver kernel runs, make sure this CPU can execute the
    // backend the binary was built for: a typed refusal beats a bare
    // illegal-instruction trap on pre-Haswell hardware.
    if (m3VerifyCpuBackend() == 0)
    {
        m3Refuse(NULL, m3_errorConfig);
        return nullId;
    }
    if (!ValidWorldDef(def))
    {
        m3Refuse(NULL, m3_errorInvalid);
        return nullId;
    }
    int32_t slot = -1;
    for (int32_t i = 0; i < M3_MAX_WORLDS && slot < 0; ++i)
    {
        slot = s_worlds[i] == NULL ? i : -1;
    }
    m3World* world = slot < 0 ? NULL : (m3World*)m3AllocZeroed((int32_t)sizeof(m3World));
    if (world == NULL)
    {
        m3Refuse(NULL, m3_errorCapacity); // the world table is full or memory is out
        return nullId;
    }
    ApplyWorldDef(world, def);
    world->generation = s_worldGenerations[slot];
    world->slot = (uint16_t)slot;
    world->idWorld =
        (uint16_t)(((uint32_t)world->generation << M3_WORLD_SLOT_BITS) | (uint32_t)slot);
    bool ok = m3StateAllocate(world);
    ok = ok && CreatePools(world);
    if (!ok)
    {
        // Out of memory or an unrepresentable size: release what exists
        // and refuse. Nothing was registered, so the slot stays free.
        FreeWorldStorage(world);
        m3Refuse(NULL, m3_errorCapacity);
        return nullId;
    }
    ClearLinks(world);
    s_worlds[slot] = world;
    return (m3WorldId){(uint16_t)(slot + 1), world->generation};
}

void m3DestroyWorld(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale or foreign id: contract, not invariant
    }
    int32_t slot = world->slot;

    FreeWorldStorage(world);

    s_worlds[slot] = NULL;
    s_worldGenerations[slot] += 1;
}

int32_t m3World_GetBodies(m3WorldId worldId, m3BodyId* ids, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    int32_t total = 0;
    for (int32_t b = 0; b < world->bodies.bodyPool.maxIndex; ++b)
    {
        if (world->bodies.bodyPool.alive[b] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m3BodyId){b + 1, world->idWorld, world->bodies.bodyPool.generations[b]};
        }
        total += 1;
    }
    return total;
}

int32_t m3World_GetJoints(m3WorldId worldId, m3JointId* ids, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    int32_t total = 0;
    for (int32_t j = 0; j < world->joints.jointPool.maxIndex; ++j)
    {
        if (world->joints.jointPool.alive[j] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m3JointId){j + 1, world->idWorld, world->joints.jointPool.generations[j]};
        }
        total += 1;
    }
    return total;
}

uint64_t m3World_GetStepCount(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL ? world->stepCount : 0u;
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
        // Turning sleep off wakes every sleeper; nothing may keep
        // napping through the new regime.
        int32_t maxBody = world->bodies.bodyPool.maxIndex;
        for (int32_t i = 0; i < maxBody; ++i)
        {
            if (world->bodies.bodyPool.alive[i] != 0 && world->bodies.awake[i] == 0 &&
                world->bodies.types[i] == (uint8_t)m3_dynamicBody)
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
    if (world->recorder.journalActive != 0)
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
    if (world->recorder.journalActive != 0)
    {
        m3OpSetContactTuning record;
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
    if (world->recorder.journalActive != 0)
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
    if (world->recorder.journalActive != 0)
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
    if (world->recorder.journalActive != 0)
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

m3MemoryUsage m3World_GetMemoryUsage(m3WorldId worldId)
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
        if (world->bodies.bodyPool.alive[i] != 0 &&
            world->bodies.types[i] == (uint8_t)m3_dynamicBody && world->bodies.awake[i] != 0)
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
    if (world->recorder.journalActive != 0)
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
    if (world->recorder.journalActive != 0)
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
    if (world->recorder.journalActive != 0)
    {
        m3OpSetWind record;
        memset(&record, 0, sizeof(record));
        record.dir = direction;
        record.speed = speed;
        record.gustHertz = gustHertz;
        record.gustScale = gustScale;
        m3JournalRecord(world, m3_opSetWind, &record, (int32_t)sizeof(record));
    }
    m3SetWindInternal(world, direction, speed, gustHertz, gustScale);
}

void m3RebuildBroadphaseInternal(m3World* world)
{
    // Collect the live proxies in shape-slot order (the canonical
    // list), carrying their CURRENT fat bounds: fatness is state,
    // and preserving it keeps every downstream pair decision
    // exactly where it was.
    int32_t maxShape = world->shapes.shapePool.maxIndex;
    int32_t count = 0;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapes.shapePool.alive[s] != 0 && world->broadphase.proxyIds[s] >= 0)
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
    uint32_t* masks = (uint32_t*)m3AllocZeroed(count * (int32_t)sizeof(uint32_t));
    int32_t* outNodes = (int32_t*)m3AllocZeroed(count * (int32_t)sizeof(int32_t));
    if (los == NULL || his == NULL || uds == NULL || masks == NULL || outNodes == NULL)
    {
        m3Free(los);
        m3Free(his);
        m3Free(uds);
        m3Free(masks);
        m3Free(outNodes);
        return; // no memory: the old tree stays, correct either way
    }
    int32_t n = 0;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapes.shapePool.alive[s] == 0 || world->broadphase.proxyIds[s] < 0)
        {
            continue;
        }
        const m3TreeNode* leaf = &world->broadphase.tree.nodes[world->broadphase.proxyIds[s]];
        for (int32_t k = 0; k < 3; ++k)
        {
            los[n][k] = leaf->lo[k];
            his[n][k] = leaf->hi[k];
        }
        uds[n] = s;
        masks[n] = leaf->mask;
        n += 1;
    }
    if (m3TreeRebuild(&world->broadphase.tree, los, his, uds, masks, n, outNodes))
    {
        for (int32_t i = 0; i < n; ++i)
        {
            world->broadphase.proxyIds[uds[i]] = outNodes[i];
        }
    }
    m3Free(los);
    m3Free(his);
    m3Free(uds);
    m3Free(masks);
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
    if (world->recorder.journalActive != 0)
    {
        int32_t zero = 0;
        m3JournalRecord(world, m3_opRebuildBroadphase, &zero, 4);
    }
    m3RebuildBroadphaseInternal(world);
}
