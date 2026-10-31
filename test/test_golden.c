// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The golden scenes: a few worlds that together reach every subsystem the
// world hash covers, stepped to hashes every platform, compiler, build
// type, SIMD backend and thread count must reproduce. The committed values
// come from test/hashes.txt as compile definitions (test/CMakeLists.txt);
// a scene that lands elsewhere fails and prints the line it got, so a
// deliberate change regenerates every line, the bench ones included, with
//
//   (out/linux-gcc-14/test_golden; out/linux-gcc-14/test_warmhold;
//    out/linux-gcc-14-bench/test_bench) | grep -E '^M3_[A-Z0-9_]+_HASH=' > test/hashes.txt
//
// Run as `test_golden hostile` (the golden_hostile test), it plays a
// hostile host instead: the floating-point settings test_fenv.h plays
// (every rounding mode, with flush-to-zero and denormals-are-zero off and
// on). The library computes in its own setting, so every run reaches the
// same values, and the host's setting is as the host left it after each.

#include "helpers.h"
#include "test_fenv.h"
#include "test_harness.h"

#include "maul3d/character.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "maul3d/vehicle.h"

// Only for the op enum and the formats' header sizes: the journal scene
// requires the tape to hold every op the journal defines. No internal
// function is called, so the suite still runs against the shared library.
#include "../src/journal.h"
#include "../src/state_prefix.h"

#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif

#ifndef M3_GOLDEN_STACK
#define M3_GOLDEN_STACK 0
#endif
#ifndef M3_GOLDEN_DESTRUCTION
#define M3_GOLDEN_DESTRUCTION 0
#endif
#ifndef M3_GOLDEN_VEHICLES
#define M3_GOLDEN_VEHICLES 0
#endif
#ifndef M3_GOLDEN_ROLLBACK
#define M3_GOLDEN_ROLLBACK 0
#endif
#ifndef M3_GOLDEN_JOURNAL
#define M3_GOLDEN_JOURNAL 0
#endif
#ifndef M3_GOLDEN_ROLLBACK_SNAPSHOT
#define M3_GOLDEN_ROLLBACK_SNAPSHOT 0
#endif
#ifndef M3_GOLDEN_JOURNAL_TAPE
#define M3_GOLDEN_JOURNAL_TAPE 0
#endif

#define DT        (1.0f / 60.0f)
#define VOXEL_DIM 16

// --- A host task system with real threads ----------------------------------
//
// The engine starts no threads; it hands ranges to the host. Any split and
// any schedule must give the serial bits, so this pool runs every range but
// the first on a thread of its own.

#define POOL_WORKERS 4

typedef struct PoolRange
{
    m3TaskFunction* task;
    void* taskContext;
    int32_t start;
    int32_t end;
} PoolRange;

typedef struct PoolTask
{
    PoolRange ranges[POOL_WORKERS];
    int32_t rangeCount;
#ifdef _WIN32
    HANDLE threads[POOL_WORKERS];
#else
    pthread_t threads[POOL_WORKERS];
#endif
    int32_t threadCount;
} PoolTask;

#ifdef _WIN32
static DWORD WINAPI PoolMain(LPVOID arg)
{
    PoolRange* r = (PoolRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return 0;
}
#else
static void* PoolMain(void* arg)
{
    PoolRange* r = (PoolRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return NULL;
}
#endif

static void* PoolEnqueue(m3TaskFunction* task, int32_t itemCount, int32_t minRange,
                         void* taskContext, void* userContext)
{
    PoolTask* t = (PoolTask*)userContext; // one task in flight, as the engine promises
    int32_t grain = (itemCount + POOL_WORKERS - 1) / POOL_WORKERS;
    grain = grain < minRange ? minRange : grain;
    t->rangeCount = 0;
    t->threadCount = 0;
    for (int32_t start = 0; start < itemCount && t->rangeCount < POOL_WORKERS; start += grain)
    {
        PoolRange* r = &t->ranges[t->rangeCount++];
        r->task = task;
        r->taskContext = taskContext;
        r->start = start;
        r->end = start + grain < itemCount ? start + grain : itemCount;
    }
    t->ranges[t->rangeCount - 1].end = itemCount;
    for (int32_t k = 1; k < t->rangeCount; ++k)
    {
#ifdef _WIN32
        t->threads[t->threadCount] = CreateThread(NULL, 0, PoolMain, &t->ranges[k], 0, NULL);
#else
        pthread_create(&t->threads[t->threadCount], NULL, PoolMain, &t->ranges[k]);
#endif
        t->threadCount += 1;
    }
    t->ranges[0].task(t->ranges[0].start, t->ranges[0].end, taskContext);
    return t;
}

static void PoolFinish(void* userTask, void* userContext)
{
    (void)userContext;
    PoolTask* t = (PoolTask*)userTask;
    for (int32_t k = 0; k < t->threadCount; ++k)
    {
#ifdef _WIN32
        WaitForSingleObject(t->threads[k], INFINITE);
        CloseHandle(t->threads[k]);
#else
        pthread_join(t->threads[k], NULL);
#endif
    }
}

static PoolTask s_pool;

static m3World* MakeWorld(int threaded)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    def.jointCapacity = 64;
    def.vehicleCapacity = 4;
    def.characterCapacity = 4;
    def.softBodyCapacity = 4;
    def.voxelCapacity = 8;
    if (threaded)
    {
        def.enqueueTask = PoolEnqueue;
        def.finishTask = PoolFinish;
        def.userTaskContext = &s_pool;
    }
    return NewWorld(&def);
}

static void StepN(m3World* world, int32_t steps)
{
    for (int32_t i = 0; i < steps; ++i)
    {
        (void)m3World_Step(world, DT, 4);
    }
}

// The host setting of the hostile run, -1 in the golden one, which alone
// prints the lines.
static int s_hostSetting = -1;

// Compares a scene's hash with its committed value and prints the line
// test/hashes.txt would need.
static void Expect(const char* name, uint64_t hash, uint64_t committed)
{
    if (s_hostSetting >= 0)
    {
        if (hash != committed)
        {
            printf("FAIL: the %s scene moved to %016llx on host setting %d\n", name,
                   (unsigned long long)hash, s_hostSetting);
            s_failures += 1;
        }
        return;
    }
    printf("M3_%s_HASH=%016llx\n", name, (unsigned long long)hash);
    if (hash != committed)
    {
        printf("FAIL: the %s scene hashed %016llx; test/hashes.txt has %016llx\n", name,
               (unsigned long long)hash, (unsigned long long)committed);
        s_failures += 1;
    }
}

// FNV-1a over a snapshot's or a tape's bytes. The formats are pinned on
// lines of their own, so a change of format (an id's layout, a new field)
// shows apart from a change of results. The hash starts at offset 8, past
// the magic and the writer's ABI version, so a release bump moves no
// line; a check pins the ABI word to this release's instead. Neither
// format here embeds another's header.
static uint64_t HashStateBytes(const uint8_t* bytes, int64_t count)
{
    uint32_t abi = 0;
    memcpy(&abi, bytes + 4, sizeof(abi));
    CHECK(count >= 16 && abi == M3_ABI_VERSION, "the bytes carry this release's ABI version");
    uint64_t h = 0xcbf29ce484222325ull;
    for (int32_t i = 8; i < count; ++i)
    {
        h = (h ^ bytes[i]) * 0x100000001b3ull;
    }
    return h;
}

static uint64_t s_snapshotBytesHash;
static uint64_t s_tapeBytesHash;

// --- Builders ----------------------------------------------------------------

static m3BodyId AddBody(m3World* world, m3BodyType type, double x, double y, double z)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = type;
    bd.position = (m3Pos3){x, y, z};
    return NewBody(world, &bd);
}

static m3BodyId AddGround(m3World* world)
{
    m3BodyId ground = AddBody(world, m3_staticBody, 0.0, 0.0, 0.0);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.8f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    return ground;
}

// A rock for the hull columns.
static const m3Vec3 s_rock[8] = {
    {-0.4f, -0.3f, -0.35f}, {0.45f, -0.3f, -0.3f}, {0.4f, -0.3f, 0.4f}, {-0.35f, -0.3f, 0.45f},
    {-0.3f, 0.35f, -0.25f}, {0.3f, 0.3f, -0.3f},   {0.25f, 0.4f, 0.3f}, {-0.3f, 0.3f, 0.35f}};

// A pyramid of 30 spheres with five droppers on top, and one column of
// every convex family: box, capsule, cylinder, hull and a two-child
// compound with offset children.
static void AddPiles(m3World* world)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    sd.restitution = 0.1f;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    for (int32_t layer = 0; layer < 4; ++layer)
    {
        int32_t size = 4 - layer;
        double base = 0.55 * (double)layer;
        for (int32_t i = 0; i < size * size; ++i)
        {
            m3BodyId body = AddBody(world, m3_dynamicBody, base + 1.02 * (double)(i % size),
                                    0.5 + 0.95 * (double)layer, base + 1.02 * (double)(i / size));
            NewSphereShape(world, body, &sd, &ball);
        }
    }
    for (int32_t d = 0; d < 5; ++d)
    {
        m3BodyId body = AddBody(world, m3_dynamicBody, 0.4 * (double)d, 7.0 + 0.8 * (double)d,
                                1.3 - 0.3 * (double)d);
        NewSphereShape(world, body, &sd, &ball);
    }

    for (int32_t level = 0; level < 3; ++level)
    {
        double y = 0.6 + 1.1 * (double)level;
        NewBoxShape(world, AddBody(world, m3_dynamicBody, -4.0, y, 0.0), &sd,
                    (m3Vec3){0.5f, 0.5f, 0.5f});
        m3Capsule pill = {{-0.4f, 0.0f, 0.0f}, {0.4f, 0.0f, 0.0f}, 0.3f};
        NewCapsuleShape(world, AddBody(world, m3_dynamicBody, -6.0, y, 0.0), &sd, &pill);
        m3Cylinder can = {{0.0f, -0.4f, 0.0f}, {0.0f, 0.4f, 0.0f}, 0.4f};
        NewCylinderShape(world, AddBody(world, m3_dynamicBody, -8.0, y, 0.0), &sd, &can, 12);
        NewHullShape(world, AddBody(world, m3_dynamicBody, -10.0, y, 0.0), &sd, s_rock, 8);
    }
    m3BodyId dumbbell = AddBody(world, m3_dynamicBody, -12.0, 2.0, 0.0);
    m3ShapeDef child = sd;
    child.localPosition = (m3Vec3){-0.6f, 0.0f, 0.0f};
    NewSphereShape(world, dumbbell, &child, &ball);
    child.localPosition = (m3Vec3){0.6f, 0.0f, 0.0f};
    NewBoxShape(world, dumbbell, &child, (m3Vec3){0.4f, 0.4f, 0.4f});
}

// A static ramp made of a triangle mesh and a bowl made of a height
// field, each with a ball rolling on it.
static void AddTerrain(m3World* world)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.4f;
    m3BodyId ramp = AddBody(world, m3_staticBody, 6.0, 0.0, -6.0);
    static const m3Vec3 corners[4] = {
        {-2.0f, 0.0f, -1.5f}, {2.0f, 1.5f, -1.5f}, {2.0f, 1.5f, 1.5f}, {-2.0f, 0.0f, 1.5f}};
    static const uint16_t quad[6] = {0, 2, 1, 0, 3, 2};
    NewMeshShape(world, ramp, &sd, corners, 4, quad, 2);

    m3BodyId bowl = AddBody(world, m3_staticBody, 10.0, 0.0, 6.0);
    static float heights[9 * 9];
    for (int32_t i = 0; i < 81; ++i)
    {
        float dx = (float)(i % 9 - 4);
        float dz = (float)(i / 9 - 4);
        heights[i] = 0.05f * (dx * dx + dz * dz);
    }
    NewHeightFieldMeshShape(world, bowl, &sd, heights, 9, 9, 1.0f);

    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
    NewSphereShape(world, AddBody(world, m3_dynamicBody, 7.5, 2.5, -6.0), &sd, &ball);
    NewSphereShape(world, AddBody(world, m3_dynamicBody, 12.5, 2.0, 7.0), &sd, &ball);
}

// --- Every joint kind with its rows switched on --------------------------------

static m3BodyId JointBox(m3World* world, double x, double y, double z)
{
    m3BodyId body = AddBody(world, m3_dynamicBody, x, y, z);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, body, &sd, (m3Vec3){0.2f, 0.15f, 0.1f});
    return body;
}

// A def joining the ground to a new box, anchored at the world point
// (x, 3, 0) on the ground's side.
static m3JointDef HangDef(m3World* world, m3BodyId ground, double x, double y, double z,
                          m3JointType type, m3BodyId* bodyOut)
{
    m3BodyId body = JointBox(world, x, y, z);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = type;
    jd.bodyIdA = ground;
    jd.bodyIdB = body;
    jd.localAnchorA = (m3Vec3){(float)x, 3.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){0.0f, (float)(3.0 - y), (float)(0.0 - z)};
    if (bodyOut != NULL)
    {
        *bodyOut = body;
    }
    return jd;
}

// Returns the ball joint, whose cone the tour moves.
static m3JointId AddHinges(m3World* world, m3BodyId ground)
{
    m3JointDef ball = HangDef(world, ground, 20.0, 2.2, 0.4, m3_sphericalJoint, NULL);
    ball.enableLimit = true;
    ball.lowerLimit = -0.4f;
    ball.upperLimit = 0.4f;
    ball.enableCone = true;
    ball.coneAngle = 0.5f;
    m3JointId joint = NewJoint(world, &ball);
    m3JointId shoulder = joint;
    m3Joint_SetSpringHertz(world, joint, 2.0f);
    m3Joint_SetSpringDampingRatio(world, joint, 0.3f);
    m3Joint_EnableSpring(world, joint, true);
    m3SphericalJoint_SetTargetRotation(
        world, joint, m3IntegrateRotation(m3MakeIdentityQuat(), (m3Vec3){0.2f, 0.0f, 0.0f}));

    m3JointDef hinge = HangDef(world, ground, 22.0, 2.0, 0.5, m3_revoluteJoint, NULL);
    hinge.enableLimit = true;
    hinge.lowerLimit = -0.8f;
    hinge.upperLimit = 0.8f;
    hinge.enableMotor = true;
    hinge.motorSpeed = 2.0f;
    hinge.maxMotorEffort = 3.0f;
    joint = NewJoint(world, &hinge);
    m3Joint_SetSpringHertz(world, joint, 1.0f);
    m3Joint_SetSpringDampingRatio(world, joint, 0.5f);
    m3Joint_EnableSpring(world, joint, true);
    m3RevoluteJoint_SetTargetAngle(world, joint, 0.3f);

    m3JointDef parallel = HangDef(world, ground, 24.0, 2.0, 0.3, m3_parallelJoint, NULL);
    parallel.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    parallel.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    NewJoint(world, &parallel);
    return shoulder;
}

static void AddSliders(m3World* world, m3BodyId ground)
{
    m3JointDef slider = HangDef(world, ground, 26.0, 2.5, 0.0, m3_prismaticJoint, NULL);
    slider.localAxisA = (m3Vec3){0.6f, 0.8f, 0.0f};
    slider.localAxisB = (m3Vec3){0.6f, 0.8f, 0.0f};
    slider.enableLimit = true;
    slider.lowerLimit = -0.5f;
    slider.upperLimit = 0.5f;
    slider.enableMotor = true;
    slider.motorSpeed = 0.5f;
    slider.maxMotorEffort = 20.0f;
    NewJoint(world, &slider);

    m3JointDef rope = HangDef(world, ground, 28.0, 2.0, 0.3, m3_distanceJoint, NULL);
    rope.localAnchorB = (m3Vec3){0.0f, 0.0f, 0.0f};
    rope.minLength = 0.5f;
    rope.maxLength = 1.5f;
    rope.length = 1.5f;
    rope.enableSpring = true;
    rope.springHertz = 3.0f;
    rope.springDampingRatio = 0.4f;
    NewJoint(world, &rope);

    m3JointDef six = HangDef(world, ground, 30.0, 2.0, 0.2, m3_genericJoint, NULL);
    six.genericLinear[0] = m3_axisLimited;
    six.genericLinear[1] = m3_axisLocked;
    six.genericLinear[2] = m3_axisLocked;
    six.genericLinearLower[0] = -0.2f;
    six.genericLinearUpper[0] = 0.2f;
    six.genericAngular[0] = m3_axisLocked;
    six.genericAngular[1] = m3_axisLocked;
    six.genericAngular[2] = m3_axisLimited;
    six.genericAngularLower[2] = -0.7f;
    six.genericAngularUpper[2] = 0.7f;
    six.genericMotorAxis = 5;
    six.motorSpeed = 1.0f;
    six.maxMotorEffort = 3.0f;
    NewJoint(world, &six);
}

static void AddWeldsAndDrives(m3World* world, m3BodyId ground)
{
    m3BodyId holder;
    m3JointDef hold = HangDef(world, ground, 32.0, 2.3, 0.0, m3_sphericalJoint, &holder);
    NewJoint(world, &hold);
    m3JointDef weld = m3DefaultJointDef();
    weld.type = m3_weldJoint;
    weld.bodyIdA = holder;
    weld.bodyIdB = JointBox(world, 32.5, 2.3, 0.0);
    weld.localAnchorA = (m3Vec3){0.25f, 0.0f, 0.0f};
    weld.localAnchorB = (m3Vec3){-0.25f, 0.0f, 0.0f};
    NewJoint(world, &weld);

    m3JointDef servo = HangDef(world, ground, 34.0, 2.0, 0.0, m3_motorJoint, NULL);
    m3JointId joint = NewJoint(world, &servo);
    m3Joint_SetSpringHertz(world, joint, 3.0f);
    m3Joint_SetSpringDampingRatio(world, joint, 1.0f);
    m3Joint_EnableSpring(world, joint, true);
    m3MotorJoint_SetOffsets(world, joint, (m3Vec3){0.0f, -1.2f, 0.3f},
                            m3IntegrateRotation(m3MakeIdentityQuat(), (m3Vec3){0.0f, 0.4f, 0.0f}));
    m3MotorJoint_SetMaxForce(world, joint, 50.0f);
    m3MotorJoint_SetMaxTorque(world, joint, 20.0f);

    m3JointDef wheel = HangDef(world, ground, 36.0, 2.4, 0.0, m3_wheelJoint, NULL);
    wheel.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    wheel.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    wheel.enableLimit = true;
    wheel.lowerLimit = -0.3f;
    wheel.upperLimit = 0.3f;
    wheel.enableMotor = true;
    wheel.motorSpeed = 5.0f;
    wheel.maxMotorEffort = 2.0f;
    joint = NewJoint(world, &wheel);
    m3Joint_SetSpringHertz(world, joint, 4.0f);
    m3Joint_SetSpringDampingRatio(world, joint, 0.7f);
    m3Joint_EnableSpring(world, joint, true);
    m3Joint_SetTargetTranslation(world, joint, 0.0f);
    m3WheelJoint_SetSteeringHertz(world, joint, 5.0f);
    m3WheelJoint_SetSteeringDampingRatio(world, joint, 0.7f);
    m3WheelJoint_SetMaxSteeringTorque(world, joint, 10.0f);
    m3WheelJoint_SetTargetSteeringAngle(world, joint, 0.2f);
    m3WheelJoint_EnableSteering(world, joint, true);

    m3BodyId gearA;
    m3BodyId gearB;
    m3JointDef axleA = HangDef(world, ground, 38.0, 3.0, 0.0, m3_revoluteJoint, &gearA);
    axleA.enableMotor = true;
    axleA.motorSpeed = 3.0f;
    axleA.maxMotorEffort = 1.0f;
    NewJoint(world, &axleA);
    m3JointDef axleB = HangDef(world, ground, 38.6, 3.0, 0.0, m3_revoluteJoint, &gearB);
    NewJoint(world, &axleB);
    m3JointDef mesh = m3DefaultJointDef();
    mesh.type = m3_gearJoint;
    mesh.bodyIdA = gearA;
    mesh.bodyIdB = gearB;
    mesh.ratio = 2.0f;
    NewJoint(world, &mesh);

    m3JointDef pulley = m3DefaultJointDef();
    pulley.type = m3_pulleyJoint;
    pulley.bodyIdA = JointBox(world, 40.0, 2.0, 0.0);
    pulley.bodyIdB = JointBox(world, 41.0, 1.5, 0.0);
    pulley.groundAnchorA = (m3Pos3){40.0, 4.0, 0.0};
    pulley.groundAnchorB = (m3Pos3){41.0, 4.0, 0.0};
    pulley.ratio = 1.0f;
    NewJoint(world, &pulley);
}

static void AddStack(m3World* world)
{
    m3BodyId ground = AddGround(world);
    AddPiles(world);
    AddTerrain(world);
    AddHinges(world, ground);
    AddSliders(world, ground);
    AddWeldsAndDrives(world, ground);
}

static m3World* BuildStack(int threaded)
{
    m3World* world = MakeWorld(threaded);
    AddStack(world);
    return world;
}

// --- The scenes ----------------------------------------------------------------

// Rigid bodies of every family, contacts, sleep and every joint. The same
// world stepped on four threads lands on the serial bits, and once warm,
// stepping allocates nothing.
static uint64_t SceneStack(void)
{
    m3World* world = BuildStack(0);
    m3Counters built = m3World_GetCounters(world);
    CHECK(built.bodyCount == 67 && built.jointCount == 14,
          "the stack holds every body and joint it builds");
    StepN(world, 150);
    m3Counters before = m3World_GetCounters(world);
    StepN(world, 150);
    m3Counters after = m3World_GetCounters(world);
    CHECK(after.allocCount == before.allocCount && after.freeCount == before.freeCount,
          "a warm world steps without allocating");
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);

    world = BuildStack(1);
    StepN(world, 300);
    CHECK(m3World_Hash(world) == hash, "four threads give the serial bits");
    m3DestroyWorld(world);
    return hash;
}

// A voxel wall with crates on its rim, (ox, oz) from the origin.
static m3ShapeId AddWall(m3World* world, double ox, double oz)
{
    static uint8_t voxels[VOXEL_DIM * VOXEL_DIM * VOXEL_DIM];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 6; z < 10; ++z)
    {
        for (int32_t y = 0; y < 12; ++y)
        {
            for (int32_t x = 0; x < VOXEL_DIM; ++x)
            {
                voxels[x + VOXEL_DIM * (y + VOXEL_DIM * z)] = 1;
            }
        }
    }
    m3BodyId holder = AddBody(world, m3_staticBody, -4.0 + ox, 0.0, -2.0 + oz);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId wall = NewVoxelChunkShape(world, holder, &sd, voxels, NULL, 0.5f);
    CHECK(m3Shape_IsValid(world, wall), "the voxel wall creates");
    for (int32_t i = 0; i < 6; ++i)
    {
        m3BodyId crate = AddBody(world, m3_dynamicBody, -3.5 + 1.3 * (double)i + ox, 6.5, 2.0 + oz);
        NewBoxShape(world, crate, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    }
    return wall;
}

// Carves the wall by a box edit and by an explosion; the islands it
// splits into fracture away.
static void Destroy(m3World* world, m3ShapeId wall, double ox, double oz)
{
    static const int32_t lo[3] = {6, 0, 6};
    static const int32_t hi[3] = {9, 11, 9};
    int32_t cleared = 0;
    CHECK(m3VoxelChunkShape_ClearBox(world, wall, lo, hi, &cleared) == m3_success,
          "the wall's core clears");
    m3ExplosionDef blast = m3DefaultExplosionDef();
    blast.position = (m3Pos3){0.0 + ox, 3.0, 0.5 + oz};
    blast.radius = 2.0f;
    blast.falloff = 2.0f;
    blast.impulsePerArea = 40.0f;
    blast.voxelCarve = 1.5f;
    m3World_Explode(world, &blast);
}

static uint64_t SceneDestruction(void)
{
    m3World* world = MakeWorld(0);
    AddGround(world);
    m3ShapeId wall = AddWall(world, 0.0, 0.0);
    StepN(world, 60);
    Destroy(world, wall, 0.0, 0.0);
    StepN(world, 180);
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

// What the vehicles builder hands back to drive.
typedef struct Cast
{
    m3VehicleId car;
    m3CharacterId hero;
    m3SoftBodyId flag;
    m3ShapeId stairs;
} Cast;

// A car with a gearbox, a character on voxel stairs, cloth and a jelly,
// and crates in moving water, (ox, oz) from the origin. The tour adds the
// wind.
static Cast AddVehicles(m3World* world, double ox, double oz)
{
    Cast cast;

    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId chassis = AddBody(world, m3_dynamicBody, 0.0 + ox, 1.0, 0.0 + oz);
    sd.density = 300.0f;
    NewBoxShape(world, chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    sd.density = 1000.0f;
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
        vd.wheels[w].steerable = (w & 1) != 0;
    }
    cast.car = NewVehicle(world, &vd);
    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    m3Vehicle_SetDrivetrain(world, cast.car, &dt);

    static uint8_t stairs[VOXEL_DIM * VOXEL_DIM * VOXEL_DIM];
    memset(stairs, 0, sizeof(stairs));
    for (int32_t z = 0; z < VOXEL_DIM; ++z)
    {
        for (int32_t x = 0; x < VOXEL_DIM; ++x)
        {
            for (int32_t y = 0; y <= x / 2; ++y)
            {
                stairs[x + VOXEL_DIM * (y + VOXEL_DIM * z)] = 1;
            }
        }
    }
    cast.stairs = NewVoxelChunkShape(world, AddBody(world, m3_staticBody, 20.0 + ox, 0.0, 0.0 + oz),
                                     &sd, stairs, NULL, 0.25f);
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.radius = 0.2f;
    cd.halfHeight = 0.3f;
    cd.position = (m3Pos3){20.5 + ox, 2.0, 2.0 + oz};
    cast.hero = NewCharacter(world, &cd);

    m3SoftBodyDef cloth = m3DefaultSoftBodyDef();
    cloth.position = (m3Pos3){-10.0 + ox, 3.0, -10.0 + oz};
    cloth.countX = 8;
    cloth.countY = 1;
    cloth.countZ = 8;
    cloth.spacing = 0.25f;
    cloth.radius = 0.08f;
    cloth.bendCompliance = 1.0e-3f;
    cast.flag = NewSoftBody(world, &cloth);
    CHECK(m3Vehicle_IsValid(world, cast.car) && m3Character_IsValid(world, cast.hero) &&
              m3SoftBody_IsValid(world, cast.flag),
          "the car, the character and the cloth create");
    CHECK(m3SoftBody_PinParticle(world, cast.flag, 0) == m3_success &&
              m3SoftBody_PinParticle(world, cast.flag, 7) == m3_success,
          "the flag pins");
    m3SoftBodyDef jelly = m3DefaultSoftBodyDef();
    jelly.position = (m3Pos3){-14.0 + ox, 1.0, -10.0 + oz};
    jelly.countX = 4;
    jelly.countY = 4;
    jelly.countZ = 4;
    jelly.spacing = 0.3f;
    jelly.compliance = 1.0e-4f;
    jelly.radius = 0.1f;
    jelly.pressure = 1.0f;
    NewSoftBody(world, &jelly);

    m3WaterVolumeDef pool = m3DefaultWaterVolumeDef();
    pool.bounds.lowerBound = (m3Pos3){-30.0 + ox, 0.0, 10.0 + oz};
    pool.bounds.upperBound = (m3Pos3){-20.0 + ox, 4.0, 20.0 + oz};
    pool.flow = (m3Vec3){0.5f, 0.0f, 0.0f};
    NewWaterVolume(world, &pool);
    for (int32_t i = 0; i < 3; ++i)
    {
        sd.density = i == 2 ? 3000.0f : 400.0f;
        NewBoxShape(world,
                    AddBody(world, m3_dynamicBody, -27.0 + 2.0 * (double)i + ox, 6.0, 15.0 + oz),
                    &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    }
    return cast;
}

static uint64_t SceneVehicles(void)
{
    m3World* world = MakeWorld(0);
    AddGround(world);
    Cast cast = AddVehicles(world, 0.0, 0.0);
    StepN(world, 60);
    m3Vehicle_SetCommands(world, cast.car, 0.6f, 0.8f, 0.0f);
    for (int32_t i = 0; i < 240; ++i)
    {
        m3Character_Move(world, cast.hero, (m3Vec3){0.03f, -0.03f, 0.0f});
        (void)m3World_Step(world, DT, 4);
    }
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

// --- The tour ------------------------------------------------------------------
//
// One world holding the terrain, the joints, the vehicles and the wall of
// the scenes above, and a script that makes every journaled call at least
// once: each create and destroy, each setter, each command and each world
// knob. The rollback scene snapshots it halfway and the journal scene
// records it from birth, so every subsystem's state is checked to ride the
// snapshot and every op to replay.

#define TOUR_FRAMES         240
#define TOUR_SNAPSHOT_FRAME 120

typedef struct Tour
{
    m3World* world;
    m3ShapeId wall;
    Cast cast;
    m3BodyId kinematic;
    m3BodyId crate;
    m3BodyId ball;
    m3ShapeId crateShape;
    m3ShapeId crateLid;
    m3ShapeId ballShape;
    m3ShapeId deck;
    m3JointId spare;
    m3JointId ghost; // a generic joint with every axis free: no rows
    m3JointId shoulder;
    m3VehicleId tank;
    m3CharacterId guest;
    m3SoftBodyId rope;
    m3SoftBodyId tet;
    m3WaterVolumeId tide;
    bool refusalSeen;
} Tour;

// Keeps the deck's contacts on its near half only: the far half lets
// bodies fall through. A pure function of its inputs, as replay needs.
static bool TourVeto(m3ShapeId shapeA, m3ShapeId shapeB, m3Pos3 point, m3Vec3 normal, void* context)
{
    (void)shapeA;
    (void)shapeB;
    (void)normal;
    (void)context;
    return point.x < 53.0;
}

static void BuildTour(Tour* t, m3World* world)
{
    memset(t, 0, sizeof(*t));
    t->world = world;
    m3World_SetPreSolveCallback(world, TourVeto, NULL);
    m3BodyId ground = AddGround(world);
    AddTerrain(world);
    t->shoulder = AddHinges(world, ground);
    AddSliders(world, ground);
    AddWeldsAndDrives(world, ground);
    t->cast = AddVehicles(world, 0.0, 60.0);
    t->wall = AddWall(world, 0.0, -30.0);
}

// The tour's own bodies: a kinematic paddle, a two-shape crate, a ball on
// a painted mesh deck, a hull rock, a cylinder, a native grid, a
// skid-steer tank, a tet jelly, a rope, a second character and a pool,
// joined by a spare joint.
static void TourCreate(Tour* t)
{
    m3World* world = t->world;
    m3ShapeDef sd = m3DefaultShapeDef();
    t->kinematic = AddBody(world, m3_kinematicBody, 50.0, 1.0, 0.0);
    NewBoxShape(t->world, t->kinematic, &sd, (m3Vec3){0.5f, 0.1f, 0.5f});
    t->crate = AddBody(world, m3_dynamicBody, 52.0, 1.0, 0.0);
    t->crateShape = NewBoxShape(t->world, t->crate, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    m3ShapeDef lid = sd;
    lid.localPosition = (m3Vec3){0.0f, 0.4f, 0.0f};
    t->crateLid = NewBoxShape(t->world, t->crate, &lid, (m3Vec3){0.3f, 0.1f, 0.3f});
    t->ball = AddBody(world, m3_dynamicBody, 54.0, 1.5, 0.0);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    t->ballShape = NewSphereShape(t->world, t->ball, &sd, &ball);
    NewHullShape(world, AddBody(world, m3_dynamicBody, 56.0, 1.0, -3.0), &sd, s_rock, 8);
    m3Cylinder can = {{0.0f, -0.4f, 0.0f}, {0.0f, 0.4f, 0.0f}, 0.4f};
    NewCylinderShape(world, AddBody(world, m3_dynamicBody, 56.0, 1.0, 3.0), &sd, &can, 12);

    static const m3Vec3 corners[4] = {
        {-3.0f, 0.0f, -2.0f}, {3.0f, 0.0f, -2.0f}, {3.0f, 0.0f, 2.0f}, {-3.0f, 0.0f, 2.0f}};
    static const uint16_t quad[6] = {0, 2, 1, 0, 3, 2};
    t->deck = NewMeshShape(world, AddBody(world, m3_staticBody, 52.0, 0.3, 0.0), &sd, corners, 4,
                           quad, 2);
    static float bumps[4 * 4] = {0.0f, 0.1f, 0.1f, 0.0f, 0.1f, 0.3f, 0.2f, 0.1f,
                                 0.1f, 0.2f, 0.3f, 0.1f, 0.0f, 0.1f, 0.1f, 0.0f};
    NewHeightFieldShape(world, AddBody(world, m3_staticBody, 58.0, 0.0, -2.0), &sd, bumps, 4, 4,
                        1.0f);

    m3BodyId tankBody = AddBody(world, m3_dynamicBody, 40.0, 1.0, 60.0);
    NewBoxShape(world, tankBody, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = tankBody;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
    }
    t->tank = NewVehicle(world, &vd);

    m3SoftBodyDef jelly = m3DefaultSoftBodyDef();
    jelly.position = (m3Pos3){64.0, 1.0, 0.0};
    jelly.compliance = 1.0e-4f;
    jelly.radius = 0.1f;
    static const m3Vec3 points[4] = {
        {0.0f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}, {0.0f, 0.6f, 0.0f}, {0.0f, 0.0f, 0.6f}};
    static const uint16_t tets[4] = {0, 1, 2, 3};
    t->tet = NewTetSoftBody(world, &jelly, points, 4, tets, 1);
    m3SoftBodyDef rope = m3DefaultSoftBodyDef();
    rope.position = (m3Pos3){65.0, 3.0, 0.0};
    rope.countX = 6;
    rope.countY = 1;
    rope.countZ = 1;
    rope.spacing = 0.25f;
    rope.radius = 0.06f;
    t->rope = NewSoftBody(world, &rope);

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.radius = 0.2f;
    cd.halfHeight = 0.3f;
    cd.position = (m3Pos3){70.0, 1.0, 0.0};
    t->guest = NewCharacter(world, &cd);

    m3WaterVolumeDef pool = m3DefaultWaterVolumeDef();
    pool.bounds.lowerBound = (m3Pos3){62.0, 0.0, -3.0};
    pool.bounds.upperBound = (m3Pos3){68.0, 1.5, 3.0};
    t->tide = NewWaterVolume(world, &pool);

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = t->crate;
    jd.bodyIdB = t->ball;
    jd.minLength = 1.0f;
    jd.maxLength = 3.0f;
    t->spare = NewJoint(world, &jd);
    // The motor record's joint: free on every axis, with no motor axis and
    // its bodies left colliding, so its calls reach the tape and move
    // nothing.
    jd = m3DefaultJointDef();
    jd.type = m3_genericJoint;
    jd.bodyIdA = t->crate;
    jd.bodyIdB = t->ball;
    jd.enableConnectedCollision = true;
    for (int32_t k = 0; k < 3; ++k)
    {
        jd.genericLinear[k] = m3_axisFree;
        jd.genericAngular[k] = m3_axisFree;
    }
    t->ghost = NewJoint(world, &jd);
    CHECK(m3Vehicle_IsValid(world, t->tank) && m3SoftBody_IsValid(world, t->tet) &&
              m3SoftBody_IsValid(world, t->rope) && m3Character_IsValid(world, t->guest) &&
              m3Joint_IsValid(world, t->spare) && m3Joint_IsValid(world, t->ghost),
          "the tour's own actors create");
}

static void TourPush(const Tour* t)
{
    m3Body_SetLinearVelocity(t->world, t->crate, (m3Vec3){0.5f, 0.0f, 0.0f});
    m3Body_SetAngularVelocity(t->world, t->crate, (m3Vec3){0.0f, 1.0f, 0.0f});
    m3Body_ApplyForce(t->world, t->crate, (m3Vec3){0.0f, 5.0f, 0.0f});
    m3Body_ApplyTorque(t->world, t->crate, (m3Vec3){0.0f, 0.0f, 0.5f});
    m3Body_ApplyLinearImpulse(t->world, t->crate, (m3Vec3){0.2f, 0.0f, 0.0f});
    m3Body_ApplyAngularImpulse(t->world, t->crate, (m3Vec3){0.1f, 0.0f, 0.0f});
    m3Body_ApplyForceAtPoint(t->world, t->crate, (m3Vec3){0.0f, 3.0f, 0.0f},
                             (m3Pos3){52.2, 1.0, 0.0});
    m3Body_ApplyLinearImpulseAtPoint(t->world, t->crate, (m3Vec3){0.0f, 0.0f, 0.3f},
                                     (m3Pos3){52.0, 1.2, 0.0});
    m3Quat turn = m3IntegrateRotation(m3MakeIdentityQuat(), (m3Vec3){0.0f, 0.3f, 0.0f});
    CHECK(m3Body_SetTargetTransform(t->world, t->kinematic, (m3Pos3){50.0, 1.5, 0.5}, turn) ==
              m3_success,
          "the kinematic body takes a target");
    m3Body_SetTransform(t->world, t->ball, (m3Pos3){54.5, 1.5, 0.0}, turn);
}

static void TourBodyState(const Tour* t)
{
    CHECK(m3Body_SetType(t->world, t->ball, m3_kinematicBody) == m3_success,
          "the ball turns kinematic");
    m3Body_SetMotionLocks(t->world, t->crate,
                          (m3MotionLocks){false, false, false, true, false, true});
    m3Body_EnableSleep(t->world, t->crate, false);
    m3Body_SetSleepThreshold(t->world, t->crate, 0.1f);
    m3Body_SetAwake(t->world, t->crate, true);
    m3Body_Enable(t->world, t->kinematic, false);
    m3Body_SetGravityScale(t->world, t->crate, 0.8f);
    m3Body_SetLinearDamping(t->world, t->crate, 0.1f);
    m3Body_SetAngularDamping(t->world, t->crate, 0.2f);
    m3Body_EnableBullet(t->world, t->crate, true);
    m3Body_SetUserData(t->world, t->crate, 0x1234u);
    m3Body_SetName(t->world, t->crate, "crate");
    m3Body_EnableFastRotation(t->world, t->crate, true);
}

static void TourWorldKnobs(const Tour* t)
{
    m3World_SetGravity(t->world, (m3Vec3){0.0f, -9.6f, 0.1f});
    m3World_SetContactTuning(t->world, 25.0f, 8.0f, 2.5f);
    m3World_SetRestitutionThreshold(t->world, 0.8f);
    m3World_SetMaximumLinearSpeed(t->world, 300.0f);
    m3World_SetMaximumAngularSpeed(t->world, 80.0f);
    m3World_SetHitEventThreshold(t->world, 0.5f);
    m3World_EnableContinuous(t->world, false);
    m3World_EnableSleep(t->world, false);
    m3World_SetWind(t->world, (m3Vec3){0.0f, 0.0f, 1.0f}, 3.0f, 0.4f, 0.3f);
}

static void TourShapes(const Tour* t)
{
    m3Shape_SetFriction(t->world, t->crateShape, 0.3f);
    m3Shape_SetRestitution(t->world, t->crateShape, 0.2f);
    m3Shape_SetRollingResistance(t->world, t->ballShape, 0.05f);
    CHECK(m3Shape_SetDensity(t->world, t->crateShape, 800.0f) == m3_success,
          "the crate takes a density");
    m3Shape_EnableHitEvents(t->world, t->crateShape, true);
    m3Shape_EnablePreSolveEvents(t->world, t->deck, true);
    m3Shape_SetFilter(t->world, t->crateLid, 0x2u, ~0x4ull, 0);
    m3Shape_SetUserData(t->world, t->crateShape, 0x5678u);
    m3Shape_SetSurfaceVelocity(t->world, t->deck, (m3Vec3){0.5f, 0.0f, 0.0f});
    static const m3MeshSurfaceMaterial paint[2] = {{0.9f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}},
                                                   {0.2f, 0.3f, 0.0f, {0.0f, 0.0f, 0.4f}}};
    static const uint8_t groups[2] = {0, 1};
    m3MeshShape_SetMaterials(t->world, t->deck, paint, 2, groups);
    m3Sphere bigger = {{0.0f, 0.0f, 0.0f}, 0.35f};
    CHECK(m3Shape_SetSphere(t->world, t->ballShape, &bigger) == m3_success, "the ball grows");
}

static void TourJointsAndSoft(const Tour* t)
{
    m3DistanceJoint_SetLengthRange(t->world, t->spare, 0.5f, 2.5f);
    m3DistanceJoint_SetLength(t->world, t->spare, 2.5f);
    m3Joint_SetSpringHertz(t->world, t->spare, 0.5f);
    m3Joint_SetSpringDampingRatio(t->world, t->spare, 2.0f);
    m3Joint_EnableSpring(t->world, t->spare, true);
    m3Joint_SetMotorSpeed(t->world, t->ghost, 0.5f);
    m3Joint_SetMaxMotorEffort(t->world, t->ghost, 2.0f);
    m3SphericalJoint_EnableCone(t->world, t->shoulder, false);
    m3SphericalJoint_SetConeAngle(t->world, t->shoulder, 0.6f);
    m3SphericalJoint_EnableCone(t->world, t->shoulder, true);
    m3Joint_EnableConnectedCollision(t->world, t->spare, true);
    m3Joint_SetBreakLimits(t->world, t->spare, 1.0e6f, 1.0e6f);
    CHECK(m3SoftBody_PinParticle(t->world, t->rope, 0) == m3_success &&
              m3SoftBody_AnchorParticle(t->world, t->rope, 5, t->crate) == m3_success &&
              m3SoftBody_AnchorToSoft(t->world, t->rope, 3, t->tet, 2) == m3_success,
          "the tour pins and anchors the rope");
}

static void TourActors(const Tour* t)
{
    (void)SetStance(t->world, t->cast.hero, 0.25f, 0.2f);
    m3Vehicle_SetGear(t->world, t->cast.car, 2);
    m3Vehicle_SetTankCommands(t->world, t->tank, 0.5f, -0.5f, 0.0f);
    CHECK(m3World_RebuildBroadphase(t->world) == m3_success, "the tour rebuilds the tree");
    CHECK(m3VoxelChunkShape_SetVoxel(t->world, t->cast.stairs, 15, 12, 3, 7) == m3_success &&
              m3VoxelChunkShape_SetFill(t->world, t->cast.stairs, 2, 0, 2, 128) == m3_success &&
              m3VoxelChunkShape_ClearVoxel(t->world, t->cast.stairs, 14, 7, 5) == m3_success,
          "the tour edits the stairs");
}

static void TourTeardown(const Tour* t)
{
    CHECK(m3Joint_IsValid(t->world, t->spare), "the spare joint holds until the teardown");
    DestroyJoint(t->world, t->spare);
    DestroyShape(t->world, t->crateLid);
    DestroyCharacter(t->world, t->guest);
    DestroyVehicle(t->world, t->tank);
    DestroySoftBody(t->world, t->rope);
    DestroyWaterVolume(t->world, t->tide);
    DestroyBody(t->world, t->ball);
}

// One frame of the tour's calls, before its step.
static void TourCalls(Tour* t, int32_t frame)
{
    switch (frame)
    {
    case 0:
        Destroy(t->world, t->wall, 0.0, -30.0);
        m3Vehicle_SetCommands(t->world, t->cast.car, 0.6f, 0.8f, 0.0f);
        break;
    case 2:
        TourCreate(t);
        break;
    case 10:
        TourPush(t);
        break;
    case 14:
        TourBodyState(t);
        break;
    case 18:
        TourWorldKnobs(t);
        break;
    case 20:
        TourShapes(t);
        break;
    case 24:
        TourJointsAndSoft(t);
        break;
    case 28:
        TourActors(t);
        break;
    case 40:
        CHECK(m3Body_SetType(t->world, t->ball, m3_dynamicBody) == m3_success,
              "the ball turns dynamic again");
        m3Body_Enable(t->world, t->kinematic, true);
        m3World_EnableContinuous(t->world, true);
        break;
    case 130:
        TourTeardown(t);
        break;
    case 140:
        CHECK(m3VoxelChunkShape_ClearVoxel(t->world, t->cast.stairs, 12, 5, 9) == m3_success &&
                  m3VoxelChunkShape_SetVoxel(t->world, t->cast.stairs, 3, 9, 9, 2) == m3_success,
              "the tour edits the stairs again");
        break;
    case 160:
        m3World_EnableSleep(t->world, true);
        m3World_SetGravity(t->world, (m3Vec3){0.0f, -9.81f, 0.0f});
        break;
    default:
        break;
    }
    m3Character_Move(t->world, t->cast.hero, (m3Vec3){0.03f, -0.03f, 0.0f});
    if (frame > 2 && frame < 130)
    {
        m3Character_Move(t->world, t->guest, (m3Vec3){0.0f, -0.02f, 0.02f});
    }
    // The tour's calls are all valid: a refused one would silently drop
    // its op from the tape and its state from the rollback. Creates are
    // checked for valid ids where they happen.
    if (m3World_GetCounters(t->world).misuseCount != 0 && !t->refusalSeen)
    {
        printf("FAIL: a tour call was refused by frame %d\n", frame);
        s_failures += 1;
        t->refusalSeen = true;
    }
}

// One frame of the tour: its calls, then the step.
static void TourFrame(Tour* t, int32_t frame)
{
    TourCalls(t, frame);
    (void)m3World_Step(t->world, DT, 4);
}

// Every frame of the tour snapshotted between its calls and its step, and
// restored: an engine-written snapshot restores whenever it is taken,
// including right after a destroy, before the step rebuilds the pairs.
// Restoring the world's own state changes nothing, so the tour ends on
// the bits of a tour that never restored.
static void CheckRestoreBetweenCallsAndStep(uint64_t expected)
{
    Tour t;
    BuildTour(&t, MakeWorld(0));
    StepN(t.world, 60);
    int32_t refused = -1;
    for (int32_t frame = 0; frame < TOUR_FRAMES; ++frame)
    {
        TourCalls(&t, frame);
        int64_t size = m3World_GetSnapshotSize(t.world);
        uint8_t* bytes = (uint8_t*)malloc((size_t)size);
        if (bytes == NULL || Snapshot(t.world, bytes, size) != size ||
            !Restore(t.world, bytes, size))
        {
            refused = refused < 0 ? frame : refused;
        }
        free(bytes);
        (void)m3World_Step(t.world, DT, 4);
    }
    if (refused >= 0)
    {
        printf("FAIL: the snapshot taken before frame %d's step does not restore\n", refused);
        s_failures += 1;
    }
    CHECK(m3World_Hash(t.world) == expected,
          "restoring between the calls and the step changes no bit");
    m3DestroyWorld(t.world);
}

// The tour rolled back across its own second half: a snapshot taken
// halfway restores every subsystem, the rerun lands on the same bits, the
// snapshot taken after it is byte for byte the first one's, and a
// different continuation diverges.
static uint64_t SceneRollback(void)
{
    Tour t;
    BuildTour(&t, MakeWorld(0));
    StepN(t.world, 60);
    for (int32_t frame = 0; frame < TOUR_SNAPSHOT_FRAME; ++frame)
    {
        TourFrame(&t, frame);
    }
    int64_t size = m3World_GetSnapshotSize(t.world);
    uint8_t* before = (uint8_t*)malloc((size_t)size);
    CHECK(Snapshot(t.world, before, size) == size, "the snapshot fits its size");

    for (int32_t frame = TOUR_SNAPSHOT_FRAME; frame < TOUR_FRAMES; ++frame)
    {
        TourFrame(&t, frame);
    }
    uint64_t first = m3World_Hash(t.world);
    int64_t afterSize = m3World_GetSnapshotSize(t.world);
    uint8_t* after = (uint8_t*)malloc((size_t)afterSize);
    Snapshot(t.world, after, afterSize);

    CHECK(Restore(t.world, before, size), "the snapshot restores");
    for (int32_t frame = TOUR_SNAPSHOT_FRAME; frame < TOUR_FRAMES; ++frame)
    {
        TourFrame(&t, frame);
    }
    CHECK(m3World_Hash(t.world) == first, "the rerun lands on the same bits");
    uint8_t* again = (uint8_t*)malloc((size_t)afterSize);
    CHECK(m3World_GetSnapshotSize(t.world) == afterSize &&
              Snapshot(t.world, again, afterSize) == afterSize &&
              memcmp(after, again, (size_t)afterSize) == 0,
          "the rerun's snapshot is byte for byte the first one's");

    s_snapshotBytesHash = HashStateBytes(after, afterSize);
    CHECK(Restore(t.world, before, size), "the snapshot restores twice");
    StepN(t.world, TOUR_FRAMES - TOUR_SNAPSHOT_FRAME);
    CHECK(m3World_Hash(t.world) != first, "a different continuation diverges");

    free(before);
    free(after);
    free(again);
    m3DestroyWorld(t.world);
    return first;
}

// --- The walk ------------------------------------------------------------------
//
// A seeded random session over the API: creates, joints, impulses,
// destroys, voxel edits, explosions and steps, beside the tour. Every
// draw is its own statement: C leaves the order of function arguments and
// initializers to the compiler, and GCC and Clang pick differently.

static uint64_t s_seed;

static uint32_t Next(void)
{
    s_seed ^= s_seed << 13;
    s_seed ^= s_seed >> 7;
    s_seed ^= s_seed << 17;
    return (uint32_t)(s_seed >> 32);
}

static float Unit(void)
{
    return (float)(Next() & 0xFFFF) / 65535.0f;
}

static double Coordinate(double lo, double span)
{
    return lo + span * (double)Unit();
}

#define WALK_BODIES 96
#define WALK_X      80.0

typedef struct Walk
{
    m3World* world;
    m3ShapeId wall;
    m3BodyId bodies[WALK_BODIES];
    int32_t bodyCount;
} Walk;

static m3BodyId WalkPick(const Walk* w)
{
    return w->bodyCount == 0 ? m3_nullBodyId : w->bodies[Next() % (uint32_t)w->bodyCount];
}

static void WalkCreate(Walk* w)
{
    if (w->bodyCount == WALK_BODIES)
    {
        return;
    }
    double x = Coordinate(WALK_X - 6.0, 12.0);
    double y = Coordinate(1.0, 6.0);
    m3BodyId body = AddBody(w->world, m3_dynamicBody, x, y, Coordinate(-6.0, 12.0));
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = Unit();
    sd.restitution = 0.5f * Unit();
    float size = 0.2f + 0.4f * Unit();
    switch (Next() % 3)
    {
    case 0:
    {
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, size};
        NewSphereShape(w->world, body, &sd, &ball);
        break;
    }
    case 1:
        NewBoxShape(w->world, body, &sd, (m3Vec3){size, 0.5f * size, size});
        break;
    default:
    {
        m3Capsule pill = {{-size, 0.0f, 0.0f}, {size, 0.0f, 0.0f}, 0.5f * size};
        NewCapsuleShape(w->world, body, &sd, &pill);
        break;
    }
    }
    w->bodies[w->bodyCount++] = body;
}

static void WalkDestroy(Walk* w)
{
    if (w->bodyCount == 0)
    {
        return;
    }
    int32_t k = (int32_t)(Next() % (uint32_t)w->bodyCount);
    DestroyBody(w->world, w->bodies[k]);
    w->bodies[k] = w->bodies[--w->bodyCount];
}

static void WalkJoin(Walk* w)
{
    m3BodyId a = WalkPick(w);
    m3BodyId b = WalkPick(w);
    if (!m3Body_IsValid(w->world, a) || !m3Body_IsValid(w->world, b) || a.index1 == b.index1)
    {
        return;
    }
    m3JointDef jd = m3DefaultJointDef();
    jd.type = (Next() & 1) != 0 ? m3_distanceJoint : m3_sphericalJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.enableLimit = jd.type == m3_sphericalJoint;
    jd.lowerLimit = 0.5f;
    jd.upperLimit = 2.0f;
    jd.minLength = 0.5f;
    jd.maxLength = 2.0f;
    NewJoint(w->world, &jd);
}

static void WalkOp(Walk* w)
{
    m3BodyId body = WalkPick(w);
    switch (Next() % 8)
    {
    case 0:
    case 1:
        WalkCreate(w);
        break;
    case 2:
        WalkDestroy(w);
        break;
    case 3:
        WalkJoin(w);
        break;
    case 4:
    {
        float x = 4.0f * Unit() - 2.0f;
        float y = 3.0f * Unit();
        m3Body_ApplyLinearImpulse(w->world, body, (m3Vec3){x, y, 0.0f});
        break;
    }
    case 5:
    {
        float y = 2.0f * Unit();
        float z = -1.0f * Unit();
        m3Body_SetLinearVelocity(w->world, body, (m3Vec3){0.0f, y, z});
        break;
    }
    case 6:
    {
        int32_t x = (int32_t)(Next() % VOXEL_DIM);
        int32_t y = (int32_t)(Next() % 6);
        int32_t z = (int32_t)(Next() % VOXEL_DIM);
        CHECK(m3VoxelChunkShape_ClearVoxel(w->world, w->wall, x, y, z) == m3_success,
              "a random clear lands");
        break;
    }
    default:
    {
        m3ExplosionDef blast = m3DefaultExplosionDef();
        double x = Coordinate(WALK_X - 4.0, 8.0);
        blast.position = (m3Pos3){x, 1.0, Coordinate(-4.0, 8.0)};
        blast.radius = 1.5f;
        blast.falloff = 1.0f;
        blast.impulsePerArea = 10.0f * Unit();
        blast.voxelCarve = Unit() < 0.3f ? 0.8f : 0.0f;
        m3World_Explode(w->world, &blast);
        break;
    }
    }
}

// Runs the walk for one seed beside whatever the world holds, on a voxel
// floor of its own, and returns the world's final hash.
static uint64_t RunWalk(Walk* w, m3World* world, uint64_t seed)
{
    s_seed = seed;
    memset(w, 0, sizeof(*w));
    w->world = world;
    static uint8_t floor[VOXEL_DIM * VOXEL_DIM * VOXEL_DIM];
    memset(floor, 0, sizeof(floor));
    for (int32_t z = 0; z < VOXEL_DIM; ++z)
    {
        for (int32_t y = 0; y < 6; ++y)
        {
            for (int32_t x = 0; x < VOXEL_DIM; ++x)
            {
                floor[x + VOXEL_DIM * (y + VOXEL_DIM * z)] = 1;
            }
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    w->wall = NewVoxelChunkShape(world, AddBody(world, m3_staticBody, WALK_X + 8.0, 0.0, 8.0), &sd,
                                 floor, NULL, 0.5f);
    for (int32_t frame = 0; frame < 120; ++frame)
    {
        int32_t ops = (int32_t)(Next() % 4);
        for (int32_t k = 0; k < ops; ++k)
        {
            WalkOp(w);
        }
        (void)m3World_Step(world, DT, 4);
    }
    return m3World_Hash(world);
}

// Requires every op the journal defines on the tape, and the tape to be
// its prefix and then a sequence of well-formed [i32 op][i32 size][payload]
// records. The tour
// makes every journaled call, so an op added without a tour call fails
// here instead of quietly going unreplayed.
static void CheckTapeHoldsEveryOp(const uint8_t* tape, int64_t bytes)
{
    bool seen[m3_opCount];
    memset(seen, 0, sizeof(seen));
    int32_t at = M3_JOURNAL_HEADER_BYTES;
    while (at + 8 <= bytes)
    {
        int32_t op;
        int32_t size;
        memcpy(&op, tape + at, 4);
        memcpy(&size, tape + at + 4, 4);
        if (op <= 0 || op >= m3_opCount || size < 0 || size > bytes - at - 8)
        {
            break;
        }
        seen[op] = true;
        at += 8 + size;
    }
    CHECK(at == bytes, "the tape is a sequence of well-formed records");
    for (int32_t op = 1; op < m3_opCount; ++op)
    {
        if (!seen[op])
        {
            printf("FAIL: journal op %d never reaches the tape; give it a tour call\n", op);
            s_failures += 1;
        }
    }
}

// The tour and then the walk in one world, recorded from its birth.
static uint64_t RunSession(m3World* world, Walk* w)
{
    Tour t;
    BuildTour(&t, world);
    StepN(world, 60);
    for (int32_t frame = 0; frame < TOUR_FRAMES; ++frame)
    {
        TourFrame(&t, frame);
    }
    return RunWalk(w, world, 0x9E3779B97F4A7C15ull);
}

// The whole session replays from its journal into a fresh world on the
// same bits, and the same session on four threads lands there too.
static uint64_t SceneJournal(void)
{
    enum
    {
        TAPE_BYTES = 4 << 20
    };
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    Walk walk;
    m3World* world = MakeWorld(0);
    CHECK(StartJournal(world, tape, TAPE_BYTES), "the session's journal starts");
    uint64_t hash = RunSession(world, &walk);
    CHECK(walk.bodyCount > 20, "the walk builds a populated corner");
    int64_t bytes = StopJournal(world);
    CHECK(bytes > 0, "the session fits its journal");
    s_tapeBytesHash = HashStateBytes(tape, bytes);
    CheckTapeHoldsEveryOp(tape, bytes);
    m3DestroyWorld(world);

    world = MakeWorld(0);
    CHECK(ReplayJournal(world, tape, bytes), "the session's journal replays");
    CHECK(m3World_Hash(world) == hash, "the replay lands on the session's bits");
    m3DestroyWorld(world);

    world = MakeWorld(1);
    CHECK(RunSession(world, &walk) == hash, "the session on four threads lands there too");
    m3DestroyWorld(world);
    free(tape);
    return hash;
}

// --- The hostile host -----------------------------------------------------
//
// The test's own arithmetic rounds in the host's mode too, so a scene
// built under a directed rounding mode gets other arguments. The stack
// world is therefore recorded once on the default setting, from its birth
// through HOSTILE_STEPS steps, and its tape replayed under each setting:
// the recorded calls keep their arguments. A replay steps through the
// journal's own path, so the replayed world then steps HOSTILE_TAIL more
// times through the public m3World_Step with a constant dt. Both must land
// where they land on the default setting. Serial runs cover every
// setting; the four-worker run, the most hostile one. The stack never
// reaches a subnormal, so flush-to-zero alone leaves it as it is; the
// rollback scene, whose tour makes every journaled public call with the
// test's own arguments, does, and runs whole under the setting that keeps
// the rounding mode and adds flush-to-zero and denormals-are-zero, which
// none of its inputs reach.
#define HOSTILE_STEPS 60
#define HOSTILE_TAIL  30

typedef struct HostileRun
{
    uint64_t replay;
    uint64_t tail;
} HostileRun;

static HostileRun ReplayStack(const uint8_t* tape, int64_t bytes, int threaded)
{
    m3World* world = MakeWorld(threaded);
    CHECK(ReplayJournal(world, tape, bytes), "the stack's tape replays");
    HostileRun run = {m3World_Hash(world), 0};
    StepN(world, HOSTILE_TAIL);
    run.tail = m3World_Hash(world);
    m3DestroyWorld(world);
    return run;
}

static void CheckRun(HostileRun run, HostileRun expected, int threaded)
{
    if (run.replay != expected.replay || run.tail != expected.tail)
    {
        printf("FAIL: host setting %d (threaded %d) replayed to %016llx and stepped to "
               "%016llx\n",
               s_hostSetting, threaded, (unsigned long long)run.replay,
               (unsigned long long)run.tail);
        s_failures += 1;
    }
}

// Runs one setting's checks with the thread in that setting, then puts
// the thread back and checks the host's setting was left as it was.
static void UnderHost(TestFloatSetting start, const uint8_t* tape, int64_t bytes,
                      HostileRun expected)
{
    TestFloatSetting host = TestFloatHostileAt(s_hostSetting);
    bool last = s_hostSetting == TestFloatHostileCount() - 1;
    bool whole = host.rounding == start.rounding && !TestFloatSame(host, start);
    TestFloatSet(host);
    CheckRun(ReplayStack(tape, bytes, 0), expected, 0);
    if (last)
    {
        CheckRun(ReplayStack(tape, bytes, 1), expected, 1);
    }
    if (whole)
    {
        Expect("ROLLBACK", SceneRollback(), M3_GOLDEN_ROLLBACK);
        Expect("ROLLBACK_SNAPSHOT", s_snapshotBytesHash, M3_GOLDEN_ROLLBACK_SNAPSHOT);
    }
    TestFloatSetting after = TestFloatGet();
    TestFloatSet(start);
    CHECK(TestFloatSame(after, host), "the host's setting is as it left it");
}

static int HostileMain(void)
{
    enum
    {
        TAPE_BYTES = 1 << 20
    };
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    m3World* world = MakeWorld(0);
    CHECK(StartJournal(world, tape, TAPE_BYTES), "the stack's journal starts");
    AddStack(world);
    StepN(world, HOSTILE_STEPS);
    int64_t bytes = StopJournal(world);
    CHECK(bytes > 0, "the stack fits its journal");
    m3DestroyWorld(world);
    HostileRun expected = ReplayStack(tape, bytes, 0);
    TestFloatSetting start = TestFloatGet();
    for (s_hostSetting = 1; s_hostSetting < TestFloatHostileCount(); ++s_hostSetting)
    {
        UnderHost(start, tape, bytes, expected);
    }
    s_hostSetting = -1;
    free(tape);
    if (s_failures == 0)
    {
        printf("test_golden hostile: all checks passed on %d host settings\n",
               TestFloatHostileCount() - 1);
    }
    return s_failures == 0 ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc > 1 && strcmp(argv[1], "hostile") == 0)
    {
        return HostileMain();
    }
    Expect("STACK", SceneStack(), M3_GOLDEN_STACK);
    Expect("DESTRUCTION", SceneDestruction(), M3_GOLDEN_DESTRUCTION);
    Expect("VEHICLES", SceneVehicles(), M3_GOLDEN_VEHICLES);
    uint64_t rollback = SceneRollback();
    Expect("ROLLBACK", rollback, M3_GOLDEN_ROLLBACK);
    CheckRestoreBetweenCallsAndStep(rollback);
    Expect("JOURNAL", SceneJournal(), M3_GOLDEN_JOURNAL);
    Expect("ROLLBACK_SNAPSHOT", s_snapshotBytesHash, M3_GOLDEN_ROLLBACK_SNAPSHOT);
    Expect("JOURNAL_TAPE", s_tapeBytesHash, M3_GOLDEN_JOURNAL_TAPE);
    if (s_failures == 0)
    {
        printf("test_golden: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
