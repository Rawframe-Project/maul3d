// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The new joint gate: FILTER is rowless by law (its whole
// effect is the connected-pair collision filter; a falling body
// wearing one falls FREE, never welded), and PARALLEL locks two
// axes parallel while every translation and the shared twist stay
// free. A parallel joint without an axis refuses.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* Yard(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    return world;
}

static void TestFilterIsRowlessAndFilters(void)
{
    // Two overlapping crates wearing a filter: they fall through
    // each other and land side by side inside the same space; the
    // moment the filter dies, the overlap resolves apart.
    m3World* world = Yard();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId a = NewBody(world, &bd);
    NewBoxShape(world, a, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    bd.position = (m3Pos3){0.2, 3.2, 0.0};
    m3BodyId b = NewBody(world, &bd);
    NewBoxShape(world, b, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_filterJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    m3JointId filter = NewJoint(world, &jd);
    CHECK(m3Joint_IsValid(world, filter), "the filter creates");
    for (int32_t i = 0; i < 180; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // Rowless proof: both crates rest ON THE FLOOR (y about 0.4),
    // still overlapping horizontally; a phantom weld would hold b
    // 0.2 above a instead.
    double ya = m3Body_GetPosition(world, a).y;
    double yb = m3Body_GetPosition(world, b).y;
    CHECK(ya > 0.3 && ya < 0.5, "crate a rests on the floor");
    CHECK(yb > 0.3 && yb < 0.5, "crate b overlaps a freely on the floor");
    DestroyJoint(world, filter);
    for (int32_t i = 0; i < 180; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double dx = fabs(m3Body_GetPosition(world, a).x - m3Body_GetPosition(world, b).x);
    CHECK(dx > 0.6, "without the filter the overlap resolves apart");
    m3DestroyWorld(world);
}

static void TestParallelKeepsAxesParallel(void)
{
    // Two floating plates joined parallel about y: a torque about x
    // on one drags the other's tilt along (the lock), while a shove
    // translates it freely and a twist about y stays private.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3World* world = NewWorld(&wd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = NewBody(world, &bd);
    NewBoxShape(world, a, &sd, (m3Vec3){0.6f, 0.05f, 0.6f});
    bd.position = (m3Pos3){2.0, 2.0, 0.0};
    m3BodyId b = NewBody(world, &bd);
    NewBoxShape(world, b, &sd, (m3Vec3){0.6f, 0.05f, 0.6f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_parallelJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the parallel joint creates");
    // Translation stays free: a shove separates them further.
    m3Body_ApplyLinearImpulse(world, b, (m3Vec3){2.0f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(world, b).x > 2.5, "translation is free");
    // The lock: torque body a about x, body b tilts WITH it.
    m3Body_ApplyAngularImpulse(world, a, (m3Vec3){0.6f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 wa = m3Body_GetAngularVelocity(world, a);
    m3Vec3 wb = m3Body_GetAngularVelocity(world, b);
    CHECK(fabsf(wa.x - wb.x) < 0.05f, "the tilt rate is shared through the lock");
    CHECK(fabsf(wa.x) > 0.1f, "the pair genuinely tilts");
    // The twist about the shared axis stays private to b.
    m3Body_ApplyAngularImpulse(world, b, (m3Vec3){0.0f, 1.0f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    wa = m3Body_GetAngularVelocity(world, a);
    wb = m3Body_GetAngularVelocity(world, b);
    CHECK(fabsf(wb.y) > fabsf(wa.y) + 0.2f, "the twist stays private");
    m3DestroyWorld(world);
}

static double ArmDroop(bool spring, bool motor, float budget)
{
    // A hinged arm under gravity, driven toward horizontal: the
    // droop below the target measures what the drive could not
    // afford. Four states, one scene.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3World* world = NewWorld(&wd);
    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId anchor = NewBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.8, 3.0, 0.0};
    m3BodyId arm = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, arm, &sd, (m3Vec3){0.7f, 0.08f, 0.08f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = arm;
    jd.localAnchorB = (m3Vec3){-0.8f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.enableMotor = motor;
    jd.motorSpeed = 0.0f;
    jd.maxMotorEffort = budget;
    m3JointId joint = NewJoint(world, &jd);
    if (spring)
    {
        m3Joint_SetSpringHertz(world, joint, 6.0f);
        m3Joint_SetSpringDampingRatio(world, joint, 1.0f);
        m3Joint_EnableSpring(world, joint, true);
        m3RevoluteJoint_SetTargetAngle(world, joint, 0.0f);
    }
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double droop = 3.0 - m3Body_GetPosition(world, arm).y;
    m3DestroyWorld(world);
    return droop;
}

static void TestFourStateDrive(void)
{
    // off: the arm falls and hangs.
    double off = ArmDroop(false, false, 0.0f);
    CHECK(off > 0.5, "state OFF lets the arm hang");
    // velocity only (speed 0, generous budget): a brake, not a hold;
    // it resists but gravity still wins ground slowly.
    double braked = ArmDroop(false, true, 50.0f);
    CHECK(braked < off, "state VELOCITY brakes the fall");
    // position only: the spring holds the arm near horizontal.
    double held = ArmDroop(true, false, 0.0f);
    CHECK(held < 0.15, "state POSITION holds the target");
    // position + velocity with a STARVED shared budget: the drive
    // may not spend more than the motor's allowance, so the arm
    // sags well below the unbudgeted hold.
    double starved = ArmDroop(true, true, 0.05f);
    CHECK(starved > held + 0.1, "the shared budget honestly starves the drive");
    // position + velocity with a rich budget matches the hold.
    double rich = ArmDroop(true, true, 50.0f);
    CHECK(rich < 0.2, "a rich shared budget still holds");
}

static void TestWheelSteering(void)
{
    // A hub and a rim on a wheel joint in zero gravity: the steer
    // drive yaws the rim about the strut to the target, reads back
    // through GetSteeringAngle, honors the journal, and refuses the
    // hostile wall.
    static uint8_t journal[65536];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        wd.jointCapacity = 4;
        m3World* world = NewWorld(&wd);
        bool recording = run == 0 && StartJournal(world, journal, (int32_t)sizeof(journal));
        m3BodyDef hd = m3DefaultBodyDef();
        hd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId hub = NewBody(world, &hd);
        m3BodyDef rd = m3DefaultBodyDef();
        rd.type = m3_dynamicBody;
        rd.position = (m3Pos3){0.0, 1.4, 0.0};
        m3BodyId rim = NewBody(world, &rd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
        NewSphereShape(world, rim, &sd, &ball);
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_wheelJoint;
        jd.bodyIdA = hub;
        jd.bodyIdB = rim;
        jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f}; // strut down
        jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};  // axle
        m3JointId wheel = NewJoint(world, &jd);
        CHECK(m3Joint_IsValid(world, wheel), "the wheel creates");
        m3WheelJoint_SetSteeringHertz(world, wheel, 8.0f);
        m3WheelJoint_SetSteeringDampingRatio(world, wheel, 1.0f);
        m3WheelJoint_SetMaxSteeringTorque(world, wheel, 0.0f);
        m3WheelJoint_SetTargetSteeringAngle(world, wheel, 0.5f);
        m3WheelJoint_EnableSteering(world, wheel, true);
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        float angle = m3WheelJoint_GetSteeringAngle(world, wheel);
        CHECK(fabsf(angle - 0.5f) < 0.05f, "the rim yaws to the steer target");
        m3WheelJoint_SetTargetSteeringAngle(world, wheel, -0.5f);
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(fabsf(m3WheelJoint_GetSteeringAngle(world, wheel) + 0.5f) < 0.05f,
              "the steer reverses");
        // The hostile wall runs in BOTH runs so the twins stay
        // symmetric: a refused steer never touches state.
        m3WheelJoint_SetTargetSteeringAngle(world, wheel, 2.0f); // beyond the range
        CHECK(m3WheelJoint_GetSteeringAngle(world, wheel) > -1.1f,
              "an absurd target refused, the drive holds");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int64_t bytes = StopJournal(world);
            CHECK(bytes > 0, "the steering session records");
            m3WorldDef fresh = wd;
            m3World* replayed = NewWorld(&fresh);
            CHECK(ReplayJournal(replayed, journal, bytes), "the steering replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin steering sessions are bit-identical");

    // The type wall lives outside the twins: a spherical never
    // steers, and asking changes nothing.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = NewBody(world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId b = NewBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    m3JointId ball = NewJoint(world, &jd);
    uint64_t before = m3World_Hash(world);
    m3WheelJoint_SetSteeringHertz(world, ball, 8.0f);
    m3WheelJoint_SetSteeringDampingRatio(world, ball, 1.0f);
    m3WheelJoint_SetMaxSteeringTorque(world, ball, 0.0f);
    m3WheelJoint_SetTargetSteeringAngle(world, ball, 0.3f);
    m3WheelJoint_EnableSteering(world, ball, true);
    CHECK(m3WheelJoint_GetSteeringAngle(world, ball) == 0.0f, "a spherical never steers");
    CHECK(m3World_Hash(world) == before, "the refused steer moved no bits");
    m3DestroyWorld(world);
}

// A wheel joint keeps its steering in the slots a spherical joint uses
// for its target rotation. A translation target once wrote the identity
// rotation there: the steering lost its frequency, damping and budget, the
// rim stopped yawing, and restore refused the world's own snapshot.
static void TestWheelTargetKeepsSteering(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&wd);
    m3BodyDef hd = m3DefaultBodyDef();
    hd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId hub = NewBody(world, &hd);
    m3BodyDef rd = m3DefaultBodyDef();
    rd.type = m3_dynamicBody;
    rd.position = (m3Pos3){0.0, 1.4, 0.0};
    m3BodyId rim = NewBody(world, &rd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    NewSphereShape(world, rim, &sd, &ball);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_wheelJoint;
    jd.bodyIdA = hub;
    jd.bodyIdB = rim;
    jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    m3JointId wheel = NewJoint(world, &jd);
    m3WheelJoint_SetSteeringHertz(world, wheel, 8.0f);
    m3WheelJoint_SetSteeringDampingRatio(world, wheel, 1.0f);
    m3WheelJoint_SetMaxSteeringTorque(world, wheel, 0.0f);
    m3WheelJoint_SetTargetSteeringAngle(world, wheel, 0.5f);
    m3WheelJoint_EnableSteering(world, wheel, true);
    m3Joint_SetTargetTranslation(world, wheel, 0.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(fabsf(m3WheelJoint_GetSteeringAngle(world, wheel) - 0.5f) < 0.05f,
          "a wheel given a translation target still steers");
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snapshot = (uint8_t*)malloc((size_t)size);
    int64_t bytes = Snapshot(world, snapshot, size);
    m3World* restored = NewWorld(&wd);
    CHECK(bytes > 0 && Restore(restored, snapshot, bytes) &&
              m3World_Hash(restored) == m3World_Hash(world),
          "and its snapshot restores");
    m3DestroyWorld(restored);
    free(snapshot);
    m3DestroyWorld(world);
}

// Records a wheel whose steering takes a frequency of 6.25 Hz and then
// switches on, with every 6.25 in the tape's records set to hertz, and
// tells whether a fresh world replays it.
static bool ReplaysSteering(float hertz)
{
    static uint8_t tape[16384];
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&wd);
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the steering tape records");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId hub = NewBody(world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.4, 0.0};
    m3BodyId rim = NewBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_wheelJoint;
    jd.bodyIdA = hub;
    jd.bodyIdB = rim;
    jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    m3JointId wheel = NewJoint(world, &jd);
    m3WheelJoint_SetSteeringHertz(world, wheel, 6.25f);
    m3WheelJoint_EnableSteering(world, wheel, true);
    int64_t bytes = StopJournal(world);
    m3DestroyWorld(world);
    const float recorded = 6.25f;
    int32_t patched = 0;
    for (int32_t i = 16; i + 4 <= bytes; ++i) // past the tape's 16-byte header
    {
        if (memcmp(tape + i, &recorded, 4) == 0)
        {
            memcpy(tape + i, &hertz, 4);
            ++patched;
        }
    }
    CHECK(patched == 2, "the frequency sits in the two steering records");
    world = NewWorld(&wd);
    bool replayed = ReplayJournal(world, tape, bytes);
    m3DestroyWorld(world);
    return replayed;
}

// Replay checks a steering record as the setters do: a tape whose
// enabled steering holds a zero frequency is refused, not applied.
static void TestSteeringReplayRefusesZeroHertz(void)
{
    CHECK(ReplaysSteering(3.5f), "a steering tape with a positive frequency replays");
    CHECK(!ReplaysSteering(0.0f), "a steering tape enabled at 0 Hz is refused");
}

// The steering switch and its four values are separate calls on one
// record: each value reads back as set, switching steering off keeps the
// values, steering needs a frequency before it switches on, and a joint
// that is not a wheel refuses every setter and getter.
static void TestSteeringSwitchKeepsValues(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId hub = NewBody(world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.4, 0.0};
    m3BodyId rim = NewBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_wheelJoint;
    jd.bodyIdA = hub;
    jd.bodyIdB = rim;
    jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    m3JointId wheel = NewJoint(world, &jd);
    uint64_t misuse = m3World_GetCounters(world).misuseCount;
    m3WheelJoint_EnableSteering(world, wheel, true);
    CHECK(!m3WheelJoint_IsSteeringEnabled(world, wheel) &&
              m3World_GetCounters(world).misuseCount == misuse + 1,
          "steering without a frequency refuses to switch on");
    m3WheelJoint_SetSteeringHertz(world, wheel, 6.0f);
    m3WheelJoint_SetSteeringDampingRatio(world, wheel, 0.8f);
    m3WheelJoint_SetMaxSteeringTorque(world, wheel, 30.0f);
    m3WheelJoint_SetTargetSteeringAngle(world, wheel, 0.4f);
    m3WheelJoint_EnableSteering(world, wheel, true);
    CHECK(m3WheelJoint_IsSteeringEnabled(world, wheel), "steering switches on");
    m3WheelJoint_EnableSteering(world, wheel, false);
    CHECK(!m3WheelJoint_IsSteeringEnabled(world, wheel) &&
              m3WheelJoint_GetSteeringHertz(world, wheel) == 6.0f &&
              m3WheelJoint_GetSteeringDampingRatio(world, wheel) == 0.8f &&
              m3WheelJoint_GetMaxSteeringTorque(world, wheel) == 30.0f &&
              m3WheelJoint_GetTargetSteeringAngle(world, wheel) == 0.4f,
          "switching steering off keeps its values");
    m3WheelJoint_EnableSteering(world, wheel, true);
    m3WheelJoint_SetSteeringHertz(world, wheel, 0.0f);
    CHECK(m3WheelJoint_GetSteeringHertz(world, wheel) == 6.0f &&
              m3World_GetCounters(world).misuseCount == misuse + 2,
          "enabled steering refuses a zero frequency");
    jd = m3DefaultJointDef();
    jd.bodyIdA = hub;
    jd.bodyIdB = rim;
    m3JointId ball = NewJoint(world, &jd);
    m3WheelJoint_SetSteeringHertz(world, ball, 6.0f);
    m3WheelJoint_EnableSteering(world, ball, true);
    CHECK(m3World_GetCounters(world).misuseCount == misuse + 4,
          "a spherical joint refuses steering");
    CHECK_REFUSES(world, CHECK(!m3WheelJoint_IsSteeringEnabled(world, ball), "steering off"),
                  "a spherical's steering switch reads off and counts misuse");
    CHECK_REFUSES(world, CHECK(m3WheelJoint_GetSteeringHertz(world, ball) == 0.0f, "zero hertz"),
                  "a spherical's steering frequency reads zero and counts misuse");
    m3DestroyWorld(world);
}

static double ServoDroop(float maxForce)
{
    // A cube hung on a servo weld under gravity, told to hold the
    // create pose: the droop below it measures what the force
    // budget could not afford (0 = uncapped).
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3World* world = NewWorld(&wd);
    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId anchor = NewBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId cube = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, cube, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_motorJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = cube;
    m3JointId servo = NewJoint(world, &jd);
    m3Joint_SetSpringHertz(world, servo, 8.0f);
    m3Joint_SetSpringDampingRatio(world, servo, 1.0f);
    m3Joint_EnableSpring(world, servo, true);
    if (maxForce > 0.0f)
    {
        m3MotorJoint_SetMaxForce(world, servo, maxForce);
    }
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double droop = 2.0 - m3Body_GetPosition(world, cube).y;
    m3DestroyWorld(world);
    return droop;
}

// Two bodies in zero gravity on a distance joint with the given range,
// and a spherical joint between the same bodies.
static m3World* RopeWorld(float minLength, float maxLength, m3JointId* ropeOut, m3JointId* ballOut)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId a = NewBody(world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId b = NewBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.minLength = minLength;
    jd.maxLength = maxLength;
    *ropeOut = NewJoint(world, &jd);
    jd = m3DefaultJointDef();
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    *ballOut = NewJoint(world, &jd);
    return world;
}

// The distance joint's range is its length range, set and read through
// its own calls: the limit calls refuse it, a range must be ordered and
// not negative, another joint type refuses the calls, and replay checks a
// recorded range the same way.
static void TestDistanceLengthRange(void)
{
    m3JointId rope;
    m3JointId ball;
    m3World* world = RopeWorld(0.5f, 2.0f, &rope, &ball);
    float minLength = 0.0f;
    float maxLength = 0.0f;
    m3DistanceJoint_GetLengthRange(world, rope, &minLength, &maxLength);
    CHECK(minLength == 0.5f && maxLength == 2.0f, "the def's range reads back");
    CHECK_ACCEPTS(world, m3DistanceJoint_SetLengthRange(world, rope, 1.0f, 1.0f),
                  "an equal range makes a rod");
    m3DistanceJoint_GetLengthRange(world, rope, &minLength, NULL);
    m3DistanceJoint_GetLengthRange(world, rope, NULL, &maxLength);
    CHECK(minLength == 1.0f && maxLength == 1.0f, "the rod reads back");
    CHECK_REFUSES(world, m3DistanceJoint_SetLengthRange(world, rope, 2.0f, 1.0f),
                  "an unordered range refuses");
    CHECK_REFUSES(world, m3DistanceJoint_SetLengthRange(world, rope, -0.5f, 1.0f),
                  "a negative minimum refuses");
    CHECK_REFUSES(world, m3DistanceJoint_SetLengthRange(world, rope, 0.5f, NAN),
                  "a NaN maximum refuses");
    m3DistanceJoint_GetLengthRange(world, rope, &minLength, &maxLength);
    CHECK(minLength == 1.0f && maxLength == 1.0f, "a refused range changes nothing");
    CHECK_REFUSES(world, m3Joint_EnableLimit(world, rope, true), "a rope takes no limit");
    CHECK_REFUSES(world, m3Joint_SetLimits(world, rope, 0.0f, 3.0f), "nor a limit range");
    CHECK(!m3Joint_IsLimitEnabled(world, rope), "and reads no limit");
    CHECK_REFUSES(world, m3DistanceJoint_SetLengthRange(world, ball, 0.5f, 1.0f),
                  "a spherical joint takes no length range");
    CHECK_REFUSES(world, m3DistanceJoint_GetLengthRange(world, ball, &minLength, &maxLength),
                  "and reading one counts misuse");
    CHECK(minLength == 0.0f && maxLength == 0.0f, "and reads zeros");
    m3DestroyWorld(world);
}

// Records a rope whose range is then set to 0.75 to 1.25, with every 0.75
// in the tape's records set to minLength, and tells whether a fresh world
// replays it.
static bool ReplaysLengthRange(float minLength)
{
    static uint8_t tape[16384];
    m3JointId rope;
    m3JointId ball;
    m3World* world = RopeWorld(0.5f, 2.0f, &rope, &ball);
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the rope tape records");
    m3DistanceJoint_SetLengthRange(world, rope, 0.75f, 1.25f);
    int64_t bytes = StopJournal(world);
    const float recorded = 0.75f;
    int32_t patched = 0;
    for (int32_t i = 16; i + 4 <= bytes; ++i) // past the tape's 16-byte header
    {
        if (memcmp(tape + i, &recorded, 4) == 0)
        {
            memcpy(tape + i, &minLength, 4);
            ++patched;
        }
    }
    CHECK(patched == 1, "the minimum sits in the one range record");
    m3World* replayed = RopeWorld(0.5f, 2.0f, &rope, &ball);
    bool ok = ReplayJournal(replayed, tape, bytes);
    if (ok && minLength == recorded)
    {
        CHECK(m3World_Hash(replayed) == m3World_Hash(world), "the range replays bit-identical");
    }
    m3DestroyWorld(replayed);
    m3DestroyWorld(world);
    return ok;
}

static void TestDistanceLengthRangeReplay(void)
{
    CHECK(ReplaysLengthRange(0.75f), "a recorded range replays");
    CHECK(!ReplaysLengthRange(1.5f), "a recorded range out of order is refused");
}

// The distance joint's spring is the joint spring part, pulling toward its
// rest length: the def's spring and length read back, SetLength takes any
// finite length not below 0, the motor calls refuse the joint, another
// type refuses the length calls, and a recorded length replays.
static void TestDistanceSpringAndLength(void)
{
    m3JointId rope;
    m3JointId ball;
    m3World* world = RopeWorld(0.5f, 2.0f, &rope, &ball);
    CHECK(!m3Joint_IsSpringEnabled(world, rope) && m3DistanceJoint_GetLength(world, rope) == 0.0f,
          "a rope from the default def has no spring and a zero rest length");
    CHECK_ACCEPTS(world, m3DistanceJoint_SetLength(world, rope, 3.0f),
                  "a rest length past the range is taken");
    CHECK(m3DistanceJoint_GetLength(world, rope) == 3.0f, "and reads back");
    CHECK_REFUSES(world, m3DistanceJoint_SetLength(world, rope, -0.5f),
                  "a negative rest length refuses");
    CHECK_REFUSES(world, m3DistanceJoint_SetLength(world, rope, NAN), "a NaN rest length refuses");
    CHECK(m3DistanceJoint_GetLength(world, rope) == 3.0f, "a refused length changes nothing");
    CHECK_ACCEPTS(world, m3Joint_SetSpringHertz(world, rope, 2.0f), "the rope takes a spring");
    CHECK_ACCEPTS(world, m3Joint_EnableSpring(world, rope, true), "and switches it on");
    CHECK_REFUSES(world, m3Joint_SetMotorSpeed(world, rope, 1.0f), "a rope takes no motor");
    CHECK_REFUSES(world, m3Joint_EnableMotor(world, rope, true), "nor a motor switch");
    CHECK_REFUSES(world, m3DistanceJoint_SetLength(world, ball, 1.0f),
                  "a spherical joint takes no rest length");
    CHECK_REFUSES(world, CHECK(m3DistanceJoint_GetLength(world, ball) == 0.0f, "zero length"),
                  "and reading one counts misuse");
    m3DestroyWorld(world);

    // The def's spring, on a rope and on a hinge.
    world = RopeWorld(0.5f, 2.0f, &rope, &ball);
    m3BodyId bodies[2];
    CHECK(m3World_GetBodies(world, bodies, 2) == 2, "two bodies");
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = bodies[0];
    jd.bodyIdB = bodies[1];
    jd.maxLength = 2.0f;
    jd.length = 1.25f;
    jd.enableSpring = true;
    jd.springHertz = 3.0f;
    jd.springDampingRatio = 0.5f;
    m3JointId sprung = NewJoint(world, &jd);
    CHECK(m3Joint_IsSpringEnabled(world, sprung) && m3Joint_GetSpringHertz(world, sprung) == 3.0f &&
              m3Joint_GetSpringDampingRatio(world, sprung) == 0.5f &&
              m3DistanceJoint_GetLength(world, sprung) == 1.25f,
          "the rope def's spring and length read back");
    jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = bodies[0];
    jd.bodyIdB = bodies[1];
    jd.enableSpring = true;
    jd.springHertz = 4.0f;
    m3JointId hinge = NewJoint(world, &jd);
    CHECK(m3Joint_IsSpringEnabled(world, hinge) && m3Joint_GetSpringHertz(world, hinge) == 4.0f,
          "a hinge def's spring reads back");
    jd.springHertz = 0.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a def spring at 0 Hz refuses");
    jd.type = m3_weldJoint;
    jd.springHertz = 4.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a weld refuses a def spring");
    m3DestroyWorld(world);
}

// Records a rope whose rest length is then set to 1.75, with every 1.75 in
// the tape's records set to length, and tells whether a fresh world
// replays it.
static bool ReplaysRestLength(float length)
{
    static uint8_t tape[16384];
    m3JointId rope;
    m3JointId ball;
    m3World* world = RopeWorld(0.5f, 2.0f, &rope, &ball);
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the rope tape records");
    m3DistanceJoint_SetLength(world, rope, 1.75f);
    int64_t bytes = StopJournal(world);
    const float recorded = 1.75f;
    int32_t patched = 0;
    for (int32_t i = 16; i + 4 <= bytes; ++i) // past the tape's 16-byte header
    {
        if (memcmp(tape + i, &recorded, 4) == 0)
        {
            memcpy(tape + i, &length, 4);
            ++patched;
        }
    }
    CHECK(patched == 1, "the length sits in the one length record");
    m3World* replayed = RopeWorld(0.5f, 2.0f, &rope, &ball);
    bool ok = ReplayJournal(replayed, tape, bytes);
    if (ok && length == recorded)
    {
        CHECK(m3World_Hash(replayed) == m3World_Hash(world), "the length replays bit-identical");
    }
    m3DestroyWorld(replayed);
    m3DestroyWorld(world);
    return ok;
}

static void TestRestLengthReplay(void)
{
    CHECK(ReplaysRestLength(1.75f), "a recorded rest length replays");
    CHECK(!ReplaysRestLength(-1.0f), "a recorded negative rest length is refused");
}

static void TestMotorJointServo(void)
{
    // The servo weld: springless it idles FREE (the joint
    // holds nothing), with a spring it flies to the commanded
    // offset AND rotation, budgets starve it, and the aim
    // rides twins, the journal, and the hostile wall.

    // Springless idle: the cube falls as if unjoined.
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        wd.jointCapacity = 4;
        m3World* world = NewWorld(&wd);
        m3BodyDef ad = m3DefaultBodyDef();
        ad.position = (m3Pos3){0.0, 3.0, 0.0};
        m3BodyId anchor = NewBody(world, &ad);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId cube = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        NewBoxShape(world, cube, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_motorJoint;
        jd.bodyIdA = anchor;
        jd.bodyIdB = cube;
        CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the servo creates");
        for (int32_t i = 0; i < 60; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(m3Body_GetPosition(world, cube).y < 0.0, "a springless servo idles free");
        m3DestroyWorld(world);
    }

    // Zero gravity: the sprung servo reaches a commanded pose.
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        wd.jointCapacity = 4;
        m3World* world = NewWorld(&wd);
        m3BodyDef ad = m3DefaultBodyDef();
        ad.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId anchor = NewBody(world, &ad);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 1.0, 0.0};
        m3BodyId cube = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        NewBoxShape(world, cube, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_motorJoint;
        jd.bodyIdA = anchor;
        jd.bodyIdB = cube;
        m3JointId servo = NewJoint(world, &jd);
        m3Joint_SetSpringHertz(world, servo, 6.0f);
        m3Joint_SetSpringDampingRatio(world, servo, 1.0f);
        m3Joint_EnableSpring(world, servo, true);
        m3Quat aim = {0.0f, 0.0f, 0.70710678f, 0.70710678f}; // 90 deg about z
        m3MotorJoint_SetOffsets(world, servo, (m3Vec3){0.6f, -1.0f, 0.0f}, aim);
        for (int32_t i = 0; i < 240; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Pos3 p = m3Body_GetPosition(world, cube);
        CHECK(fabs(p.x - 0.6) < 0.05 && fabs(p.y - 1.0) < 0.05 && fabs(p.z) < 0.05,
              "the servo reaches the commanded offset");
        m3Quat q = m3Body_GetRotation(world, cube);
        float dot = q.x * aim.x + q.y * aim.y + q.z * aim.z + q.w * aim.w;
        CHECK(fabsf(dot) > 0.995f, "the servo reaches the commanded rotation");
        m3DestroyWorld(world);
    }

    // The force budget starves under gravity.
    double rich = ServoDroop(0.0f);
    CHECK(rich < 0.15, "an uncapped servo holds the hang");
    double starved = ServoDroop(0.3f);
    CHECK(starved > rich + 0.3, "a starved force budget sags honestly");

    // Twins, journal, and the hostile wall in BOTH runs.
    static uint8_t journal[65536];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        wd.jointCapacity = 4;
        m3World* world = NewWorld(&wd);
        bool recording = run == 0 && StartJournal(world, journal, (int32_t)sizeof(journal));
        m3BodyDef ad = m3DefaultBodyDef();
        ad.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId anchor = NewBody(world, &ad);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 1.0, 0.0};
        m3BodyId cube = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        NewBoxShape(world, cube, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_motorJoint;
        jd.bodyIdA = anchor;
        jd.bodyIdB = cube;
        m3JointId servo = NewJoint(world, &jd);
        m3Joint_SetSpringHertz(world, servo, 6.0f);
        m3Joint_SetSpringDampingRatio(world, servo, 1.0f);
        m3Joint_EnableSpring(world, servo, true);
        m3MotorJoint_SetOffsets(world, servo, (m3Vec3){0.3f, -1.0f, 0.0f},
                                (m3Quat){0.0f, 0.38268343f, 0.0f, 0.92387953f});
        for (int32_t i = 0; i < 60; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3MotorJoint_SetOffsets(world, servo, (m3Vec3){-0.3f, -1.0f, 0.0f},
                                (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
        for (int32_t i = 0; i < 60; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        // Hostile aims run in BOTH runs so the twins stay symmetric:
        // a refused pose never touches state or the journal.
        m3MotorJoint_SetOffsets(world, servo, (m3Vec3){0.0f, 0.0f, 0.0f},
                                (m3Quat){0.0f, 0.0f, 0.0f, 2.0f}); // far from unit
        m3MotorJoint_SetOffsets(world, servo, (m3Vec3){NAN, 0.0f, 0.0f},
                                (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
        // The servo keeps its aim in the motor slot, so a motor call
        // must not reach it.
        uint64_t aimed = m3World_Hash(world);
        CHECK_REFUSES(world, m3Joint_SetMotorSpeed(world, servo, 2.0f),
                      "a motor joint takes no motor");
        CHECK(m3World_Hash(world) == aimed, "the refused motor moved no bits");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int64_t bytes = StopJournal(world);
            CHECK(bytes > 0, "the servo session records");
            m3WorldDef fresh = wd;
            m3World* replayed = NewWorld(&fresh);
            CHECK(ReplayJournal(replayed, journal, bytes), "the servo session replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin servo sessions are bit-identical");

    // The type wall lives outside the twins: a spherical takes no
    // servo aim, and asking moves no bits.
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        wd.jointCapacity = 4;
        m3World* world = NewWorld(&wd);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId a = NewBody(world, &bd);
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 1.0, 0.0};
        m3BodyId b = NewBody(world, &bd);
        m3JointDef jd = m3DefaultJointDef();
        jd.bodyIdA = a;
        jd.bodyIdB = b;
        m3JointId ball = NewJoint(world, &jd);
        uint64_t before = m3World_Hash(world);
        m3MotorJoint_SetOffsets(world, ball, (m3Vec3){0.1f, 0.0f, 0.0f},
                                (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
        CHECK(m3World_Hash(world) == before, "the refused aim moved no bits");
        CHECK_REFUSES(world, m3MotorJoint_SetMaxForce(world, ball, 1.0f),
                      "a spherical takes no servo budget");
        CHECK_REFUSES(world, CHECK(m3MotorJoint_GetMaxForce(world, ball) == 0.0f, "zero budget"),
                      "a spherical's servo budget reads zero and counts misuse");
        CHECK_REFUSES(world, CHECK(m3MotorJoint_GetLinearOffset(world, ball).x == 0.0f, "zero aim"),
                      "a spherical's servo aim reads zero and counts misuse");

        // The def's budgets, the aim and the budgets read back, and the
        // servo takes no limits.
        jd.type = m3_motorJoint;
        jd.maxForce = 5.0f;
        jd.maxTorque = 7.0f;
        m3JointId servo = NewJoint(world, &jd);
        CHECK(m3MotorJoint_GetMaxForce(world, servo) == 5.0f &&
                  m3MotorJoint_GetMaxTorque(world, servo) == 7.0f,
              "the def's budgets read back");
        m3MotorJoint_SetMaxTorque(world, servo, 9.0f);
        CHECK(m3MotorJoint_GetMaxForce(world, servo) == 5.0f &&
                  m3MotorJoint_GetMaxTorque(world, servo) == 9.0f,
              "setting the torque budget keeps the force budget");
        CHECK_REFUSES(world, m3MotorJoint_SetMaxForce(world, servo, -1.0f),
                      "a negative budget refuses");
        m3Quat half = {0.0f, 0.38268343f, 0.0f, 0.92387953f};
        m3MotorJoint_SetOffsets(world, servo, (m3Vec3){0.25f, -1.0f, 0.5f}, half);
        m3Vec3 linear = m3MotorJoint_GetLinearOffset(world, servo);
        m3Quat angular = m3MotorJoint_GetAngularOffset(world, servo);
        CHECK(linear.x == 0.25f && linear.y == -1.0f && linear.z == 0.5f &&
                  fabsf(angular.y - half.y) < 1e-6f && fabsf(angular.w - half.w) < 1e-6f,
              "the aim reads back");
        CHECK_REFUSES(world, m3Joint_EnableLimit(world, servo, true), "a servo takes no limit");
        CHECK_REFUSES(world, m3Joint_SetLimits(world, servo, 0.0f, 1.0f), "nor a limit range");
        m3DestroyWorld(world);
    }
}

static void TestJointWalls(void)
{
    // Hostile walls: a parallel joint with a zero axis refuses, a
    // filter with zero axes creates (it has no geometry to check).
    m3World* world = Yard();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = NewBody(world, &bd);
    bd.position = (m3Pos3){1.0, 2.0, 0.0};
    m3BodyId b = NewBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_parallelJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 0.0f};
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a zero-axis parallel refuses");
    jd.type = m3_filterJoint;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "a filter needs no axes");
    m3DestroyWorld(world);
}

int main(void)
{
    TestFilterIsRowlessAndFilters();
    TestParallelKeepsAxesParallel();
    TestFourStateDrive();
    TestWheelSteering();
    TestWheelTargetKeepsSteering();
    TestSteeringSwitchKeepsValues();
    TestSteeringReplayRefusesZeroHertz();
    TestDistanceLengthRange();
    TestDistanceLengthRangeReplay();
    TestDistanceSpringAndLength();
    TestRestLengthReplay();
    TestMotorJointServo();
    TestJointWalls();
    if (s_failures == 0)
    {
        printf("test_newjoints: all passed\n");
        return 0;
    }
    return 1;
}
