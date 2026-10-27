// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World lifecycle and the journal: a static world table with
// generations, def-cookie validation, SoA arrays described once in the
// state table (world_state.c), and a journal whose replay goes through
// the same internal functions the public API uses, the same shape as
// maul2d's.

#include "world.h"

#include "body_state.h"
#include "cpu.h"
#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "snapshot_walk.h"
#include "world_internal.h"
#include "world_state.h"

#include <stddef.h>
#include <string.h>

m3WorldDef m3DefaultWorldDefValues(void)
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
        m3HeightFieldDataFree(&world->heap, &world->heightFields.hfData[hf]);
    }
    for (int32_t m = 0; world->meshes.meshData != NULL && world->meshes.meshBvh != NULL &&
                        m < world->meshes.meshCapacity;
         ++m)
    {
        m3MeshDataFree(&world->heap, &world->meshes.meshData[m]);
        m3MeshBvhFree(&world->heap, &world->meshes.meshBvh[m]);
    }
    for (int32_t v = 0; world->voxels.voxelTrees != NULL && v < world->voxels.voxelCapacity; ++v)
    {
        m3Free(&world->heap, world->voxels.voxelTrees[v]);
    }
    m3Free(&world->heap, (void*)world->voxels.voxelTrees);
    m3Free(&world->heap, world->voxels.voxelTreeScratch);
    m3Free(&world->heap, world->voxels.voxelStage);
    m3StateFree(world);
    m3IdPoolDestroy(&world->heap, &world->bodies.bodyPool);
    m3IdPoolDestroy(&world->heap, &world->shapes.shapePool);
    m3IdPoolDestroy(&world->heap, &world->hulls.hullPool);
    m3IdPoolDestroy(&world->heap, &world->joints.jointPool);
    m3IdPoolDestroy(&world->heap, &world->heightFields.hfPool);
    m3IdPoolDestroy(&world->heap, &world->water.waterPool);
    m3IdPoolDestroy(&world->heap, &world->characters.charPool);
    m3IdPoolDestroy(&world->heap, &world->vehicles.vehPool);
    m3IdPoolDestroy(&world->heap, &world->softBodies.softPool);
    m3IdPoolDestroy(&world->heap, &world->meshes.meshPool);
    m3IdPoolDestroy(&world->heap, &world->voxels.voxelPool);
    m3TreeDestroy(&world->heap, &world->broadphase.tree);
    m3StackDestroy(&world->heap, &world->scratch);
    // The world's own block goes last, through a copy of the heap it lives in.
    m3Heap heap = world->heap;
    m3Free(&heap, world);
}

// User-input validation is contract, not invariant: a bad def is a
// refusal, never an assert.
bool m3GravityValid(m3Vec3 gravity)
{
    return m3WithinV3(gravity, M3_ACCEL_LIMIT);
}

bool m3ContactTuningValid(float hertz, float dampingRatio, float pushSpeed)
{
    return hertz > 0.0f && hertz <= M3_HERTZ_LIMIT && dampingRatio > 0.0f &&
           dampingRatio <= M3_RATIO_LIMIT && pushSpeed > 0.0f &&
           pushSpeed <= M3_SPEED_LIMIT; // false for NaN
}

bool m3SpeedCapValid(float value)
{
    return value > 0.0f && value <= M3_SPEED_LIMIT; // false for NaN
}

// A blowing wind demands a near-unit direction; a still one any finite
// direction.
bool m3WindValid(m3Vec3 direction, float speed, float gustHertz, float gustScale)
{
    if (!m3FiniteV3(direction) || !(speed >= 0.0f && speed <= M3_SPEED_LIMIT) ||
        !(gustHertz >= 0.0f && gustHertz <= M3_HERTZ_LIMIT) ||
        !(gustScale >= 0.0f && gustScale <= M3_RATIO_LIMIT))
    {
        return false;
    }
    m3real len2 = m3Dot3(direction, direction);
    return speed == 0.0f || (len2 >= 0.99f && len2 <= 1.01f);
}

bool m3StepValid(float dt, int32_t substeps)
{
    return dt >= M3_STEP_MIN && dt <= M3_STEP_LIMIT && substeps >= 1 &&
           substeps <= M3_MAX_SUBSTEPS; // false for NaN
}

static bool ValidWorldDef(const m3WorldDef* def)
{
    bool capacities = def->bodyCapacity > 0 && def->shapeCapacity > 0 && def->meshCapacity > 0 &&
                      def->jointCapacity > 0 && def->voxelCapacity > 0 &&
                      def->characterCapacity > 0 && def->vehicleCapacity > 0 &&
                      def->softBodyCapacity > 0 && def->shapeCapacity <= INT32_MAX / 8 &&
                      def->voxelCapacity <= INT32_MAX / 6;
    bool tasks = (def->enqueueTask == NULL) == (def->finishTask == NULL);
    bool tuning = m3GravityValid(def->gravity) &&
                  m3ContactTuningValid(def->contactHertz, def->contactDampingRatio,
                                       def->contactPushMaxSpeed) &&
                  m3FiniteF(def->restitutionThreshold) && def->restitutionThreshold >= 0.0f &&
                  m3SpeedCapValid(def->maximumLinearSpeed) &&
                  m3SpeedCapValid(def->maximumAngularSpeed) && m3FiniteF(def->hitEventThreshold) &&
                  def->hitEventThreshold >= 0.0f;
    return capacities && tasks && tuning && m3AllocatorValid(&def->allocator);
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
    world->enqueueTask = def->enqueueTask;
    world->finishTask = def->finishTask;
    world->userTaskContext = def->userTaskContext;
    world->contacts.pairCapacity = 8 * def->shapeCapacity;
}

// The proxy tree's nodes: a leaf per shape slot and the inner nodes
// above them.
static int32_t TreeCapacity(int32_t shapeCapacity)
{
    return 2 * shapeCapacity;
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
        *pools[p] = m3IdPoolCreate(&world->heap, caps[p]);
        ok = ok && pools[p]->capacity != 0;
        world->memoryBytes +=
            (int64_t)caps[p] * (int64_t)(sizeof(uint32_t) + sizeof(uint8_t) + sizeof(int32_t));
    }
    int32_t shapeCap = world->shapes.shapeCapacity;
    world->broadphase.tree = m3TreeCreate(&world->heap, TreeCapacity(shapeCap));
    world->memoryBytes += (int64_t)TreeCapacity(shapeCap) * (int64_t)sizeof(m3TreeNode);
    // Step scratch grows between steps on m3_errorCapacity, never mid-step.
    world->scratch = m3StackCreate(&world->heap, 256 * 1024);
    // The voxel slots' trees come with their chunks; the build scratch
    // and the edit staging copy serve them all.
    int32_t voxelCap = world->voxels.voxelCapacity;
    world->voxels.voxelTrees = (void**)m3AllocArray(&world->heap, voxelCap, (int64_t)sizeof(void*));
    world->voxels.voxelTreeScratch =
        m3AllocZeroed(&world->heap, m3MeshBvhScratchBytes(M3_VOXEL_MAX_BOXES));
    world->voxels.voxelStage =
        (m3VoxelChunkData*)m3AllocZeroed(&world->heap, (int32_t)sizeof(m3VoxelChunkData));
    world->voxels.voxelBoxLimit = M3_VOXEL_MAX_BOXES;
    world->memoryBytes += (int64_t)voxelCap * (int64_t)sizeof(void*) +
                          m3MeshBvhScratchBytes(M3_VOXEL_MAX_BOXES) +
                          (int64_t)sizeof(m3VoxelChunkData);
    return ok && world->broadphase.tree.capacity != 0 && world->scratch.capacity != 0 &&
           world->voxels.voxelTrees != NULL && world->voxels.voxelTreeScratch != NULL &&
           world->voxels.voxelStage != NULL;
}

static M3_NOINLINE m3Result CreateWorldWorker(const m3WorldDef* def, m3World** worldOut)
{
    if (worldOut == NULL)
    {
        return m3_errorInvalid;
    }
    *worldOut = NULL;
    // Before any solver kernel runs, make sure this CPU can execute the
    // backend the binary was built for: a typed refusal beats a bare
    // illegal-instruction trap on pre-Haswell hardware.
    if (m3CheckCpu(NULL) == 0)
    {
        return m3_errorUnsupported;
    }
    if (def == NULL || def->internalValue != M3_WORLD_COOKIE)
    {
        return m3_errorInvalid;
    }
    if (def->version != M3_ABI_VERSION)
    {
        return m3_errorVersion;
    }
    if (!ValidWorldDef(def))
    {
        return m3_errorInvalid;
    }
    // The world's snapshot must stay within M3_MAX_SNAPSHOT_BYTES whatever
    // mesh and hull content it takes: measured on the def's capacities
    // alone, before anything is allocated.
    m3World measured;
    memset(&measured, 0, sizeof(measured));
    ApplyWorldDef(&measured, def);
    measured.broadphase.tree.capacity = TreeCapacity(def->shapeCapacity);
    if (!m3SnapshotFitsCapacities(&measured))
    {
        return m3_errorCapacity;
    }
    m3Heap heap = m3HeapMake(&def->allocator);
    m3World* world = (m3World*)m3AllocZeroed(&heap, (int32_t)sizeof(m3World));
    if (world == NULL)
    {
        return m3_errorCapacity;
    }
    world->heap = heap;
    ApplyWorldDef(world, def);
    bool ok = m3StateAllocate(world);
    ok = ok && CreatePools(world);
    if (!ok)
    {
        // Out of memory or an unrepresentable size: release what exists.
        FreeWorldStorage(world);
        return m3_errorCapacity;
    }
    ClearLinks(world);
    *worldOut = world;
    return m3_success;
}

m3Result m3CreateWorld(const m3WorldDef* def, m3World** worldOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateWorldWorker(def, worldOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE void DestroyWorldWorker(m3World* world)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (world != NULL)
    {
        FreeWorldStorage(world);
    }
}

void m3DestroyWorld(m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    DestroyWorldWorker(world);
    m3FloatEnvLeave(env);
}

int32_t m3World_GetBodies(const m3World* world, m3BodyId* ids, int32_t capacity)
{
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
            ids[total] = (m3BodyId){b + 1, world->bodies.bodyPool.generations[b]};
        }
        total += 1;
    }
    return total;
}

int32_t m3World_GetJoints(const m3World* world, m3JointId* ids, int32_t capacity)
{
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
            ids[total] = (m3JointId){j + 1, world->joints.jointPool.generations[j]};
        }
        total += 1;
    }
    return total;
}

uint64_t m3World_GetStepCount(const m3World* world)
{
    return world != NULL ? world->stepCount : 0u;
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

static M3_NOINLINE void WorldSetGravityWorker(m3World* world, m3Vec3 gravity)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (world == NULL || !m3GravityValid(gravity))
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

void m3World_SetGravity(m3World* world, m3Vec3 gravity)
{
    uint64_t env = m3FloatEnvEnter();
    WorldSetGravityWorker(world, gravity);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE m3Vec3 WorldGetGravityWorker(const m3World* world)
{
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    return world != NULL ? world->gravity : zero;
}

m3Vec3 m3World_GetGravity(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = WorldGetGravityWorker(world);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE void WorldSetContactTuningWorker(m3World* world, float hertz, float dampingRatio,
                                                    float pushMaxSpeed)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (world == NULL || !m3ContactTuningValid(hertz, dampingRatio, pushMaxSpeed))
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

void m3World_SetContactTuning(m3World* world, float hertz, float dampingRatio, float pushMaxSpeed)
{
    uint64_t env = m3FloatEnvEnter();
    WorldSetContactTuningWorker(world, hertz, dampingRatio, pushMaxSpeed);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void WorldSetRestitutionThresholdWorker(m3World* world, float value)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
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

void m3World_SetRestitutionThreshold(m3World* world, float value)
{
    uint64_t env = m3FloatEnvEnter();
    WorldSetRestitutionThresholdWorker(world, value);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void WorldSetMaximumLinearSpeedWorker(m3World* world, float value)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (world == NULL || !m3SpeedCapValid(value))
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

void m3World_SetMaximumLinearSpeed(m3World* world, float value)
{
    uint64_t env = m3FloatEnvEnter();
    WorldSetMaximumLinearSpeedWorker(world, value);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void WorldSetMaximumAngularSpeedWorker(m3World* world, float value)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (world == NULL || !m3SpeedCapValid(value))
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

void m3World_SetMaximumAngularSpeed(m3World* world, float value)
{
    uint64_t env = m3FloatEnvEnter();
    WorldSetMaximumAngularSpeedWorker(world, value);
    m3FloatEnvLeave(env);
}

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
    counters.voxelEditsRefused = world->voxels.voxelEditsRefused;
    counters.heldPushRetakes = world->heldPushRetakes;
    counters.snapshotBytes = m3World_SnapshotSize(world);
    counters.allocCalls = world->heap.allocCalls;
    counters.freeCalls = world->heap.freeCalls;
    counters.misuse = m3MisuseCount(world);
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

static M3_NOINLINE void WorldEnableSleepingWorker(m3World* world, bool flag)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
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

void m3World_EnableSleeping(m3World* world, bool flag)
{
    uint64_t env = m3FloatEnvEnter();
    WorldEnableSleepingWorker(world, flag);
    m3FloatEnvLeave(env);
}

bool m3World_IsSleepingEnabled(const m3World* world)
{
    return world != NULL && world->sleepEnabled != 0;
}

static M3_NOINLINE void WorldEnableContinuousWorker(m3World* world, bool flag)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
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

void m3World_EnableContinuous(m3World* world, bool flag)
{
    uint64_t env = m3FloatEnvEnter();
    WorldEnableContinuousWorker(world, flag);
    m3FloatEnvLeave(env);
}

bool m3World_IsContinuousEnabled(const m3World* world)
{
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

static M3_NOINLINE void WorldSetWindWorker(m3World* world, m3Vec3 direction, float speed,
                                           float gustHertz, float gustScale)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (world == NULL || !m3WindValid(direction, speed, gustHertz, gustScale))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
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

void m3World_SetWind(m3World* world, m3Vec3 direction, float speed, float gustHertz,
                     float gustScale)
{
    uint64_t env = m3FloatEnvEnter();
    WorldSetWindWorker(world, direction, speed, gustHertz, gustScale);
    m3FloatEnvLeave(env);
}

// The broadphase rebuild's scratch, taken whole before anything is
// recorded or changed, so a refusing allocator leaves the tape and the
// tree as they were.
typedef struct RebuildScratch
{
    int32_t count;
    double (*los)[3];
    double (*his)[3];
    int32_t* uds;
    uint32_t* masks;
    int32_t* order; // the tree's sort buffer, twice count
    int32_t* outNodes;
} RebuildScratch;

static void DropRebuildScratch(m3World* world, RebuildScratch* scratch)
{
    m3Free(&world->heap, scratch->los);
    m3Free(&world->heap, scratch->his);
    m3Free(&world->heap, scratch->uds);
    m3Free(&world->heap, scratch->masks);
    m3Free(&world->heap, scratch->order);
    m3Free(&world->heap, scratch->outNodes);
    memset(scratch, 0, sizeof(*scratch));
}

// Takes the scratch for every live proxy. False, holding nothing, when
// the allocator refuses; a world without proxies needs none.
static bool TakeRebuildScratch(m3World* world, RebuildScratch* scratch)
{
    memset(scratch, 0, sizeof(*scratch));
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
        return true;
    }
    scratch->count = count;
    scratch->los = (double (*)[3])m3AllocZeroed(&world->heap, count * 3 * (int32_t)sizeof(double));
    scratch->his = (double (*)[3])m3AllocZeroed(&world->heap, count * 3 * (int32_t)sizeof(double));
    scratch->uds = (int32_t*)m3AllocZeroed(&world->heap, count * (int32_t)sizeof(int32_t));
    scratch->masks = (uint32_t*)m3AllocZeroed(&world->heap, count * (int32_t)sizeof(uint32_t));
    scratch->order = (int32_t*)m3AllocZeroed(&world->heap, 2 * count * (int32_t)sizeof(int32_t));
    scratch->outNodes = (int32_t*)m3AllocZeroed(&world->heap, count * (int32_t)sizeof(int32_t));
    if (scratch->los == NULL || scratch->his == NULL || scratch->uds == NULL ||
        scratch->masks == NULL || scratch->order == NULL || scratch->outNodes == NULL)
    {
        DropRebuildScratch(world, scratch);
        return false;
    }
    return true;
}

// Rebuilds the tree from the scratch's proxies; takes no memory.
static void RebuildWithScratch(m3World* world, RebuildScratch* scratch)
{
    if (scratch->count == 0)
    {
        return;
    }
    // Collect the live proxies in shape-slot order (the canonical
    // list), carrying their CURRENT fat bounds: fatness is state,
    // and preserving it keeps every downstream pair decision
    // exactly where it was.
    int32_t maxShape = world->shapes.shapePool.maxIndex;
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
            scratch->los[n][k] = leaf->lo[k];
            scratch->his[n][k] = leaf->hi[k];
        }
        scratch->uds[n] = s;
        scratch->masks[n] = leaf->mask;
        n += 1;
    }
    // n leaves already sit in the tree, so it has the 2n - 1 nodes.
    bool rebuilt = m3TreeRebuild(&world->broadphase.tree, scratch->los, scratch->his, scratch->uds,
                                 scratch->masks, n, scratch->order, scratch->outNodes);
    M3_ASSERT(rebuilt);
    (void)rebuilt;
    for (int32_t i = 0; i < n; ++i)
    {
        world->broadphase.proxyIds[scratch->uds[i]] = scratch->outNodes[i];
    }
}

bool m3RebuildBroadphaseInternal(m3World* world)
{
    RebuildScratch scratch;
    if (!TakeRebuildScratch(world, &scratch))
    {
        return false;
    }
    RebuildWithScratch(world, &scratch);
    DropRebuildScratch(world, &scratch);
    return true;
}

static M3_NOINLINE m3Result WorldRebuildBroadphaseWorker(m3World* world)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (world == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    // The scratch comes first: once the op is on the tape, the rebuild
    // must happen, or a replay with memory would rebuild alone.
    RebuildScratch scratch;
    if (!TakeRebuildScratch(world, &scratch))
    {
        return m3_errorCapacity; // the tree and the tape stay as they were
    }
    if (world->recorder.journalActive != 0)
    {
        int32_t zero = 0;
        m3JournalRecord(world, m3_opRebuildBroadphase, &zero, 4);
    }
    RebuildWithScratch(world, &scratch);
    DropRebuildScratch(world, &scratch);
    return m3_success;
}

m3Result m3World_RebuildBroadphase(m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = WorldRebuildBroadphaseWorker(world);
    m3FloatEnvLeave(env);
    return result;
}
