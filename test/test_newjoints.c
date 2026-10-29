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
    // through GetSteerAngle, honors the journal, and refuses the
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
        m3WheelJoint_SetSteer(world, wheel, true, 0.5f, 8.0f, 1.0f, 0.0f);
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        float angle = m3WheelJoint_GetSteerAngle(world, wheel);
        CHECK(fabsf(angle - 0.5f) < 0.05f, "the rim yaws to the steer target");
        m3WheelJoint_SetSteer(world, wheel, true, -0.5f, 8.0f, 1.0f, 0.0f);
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(fabsf(m3WheelJoint_GetSteerAngle(world, wheel) + 0.5f) < 0.05f, "the steer reverses");
        // The hostile wall runs in BOTH runs so the twins stay
        // symmetric: a refused steer never touches state.
        m3WheelJoint_SetSteer(world, wheel, true, 2.0f, 8.0f, 1.0f, 0.0f); // beyond the range
        CHECK(m3WheelJoint_GetSteerAngle(world, wheel) > -1.1f,
              "an absurd target refused, the drive holds");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = StopJournal(world);
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
    m3WheelJoint_SetSteer(world, ball, true, 0.3f, 8.0f, 1.0f, 0.0f);
    CHECK(m3WheelJoint_GetSteerAngle(world, ball) == 0.0f, "a spherical never steers");
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
    m3WheelJoint_SetSteer(world, wheel, true, 0.5f, 8.0f, 1.0f, 0.0f);
    m3Joint_SetTargetTranslation(world, wheel, 0.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(fabsf(m3WheelJoint_GetSteerAngle(world, wheel) - 0.5f) < 0.05f,
          "a wheel given a translation target still steers");
    int32_t size = m3World_GetSnapshotSize(world);
    uint8_t* snapshot = (uint8_t*)malloc((size_t)size);
    int32_t bytes = Snapshot(world, snapshot, size);
    m3World* restored = NewWorld(&wd);
    CHECK(bytes > 0 && Restore(restored, snapshot, bytes) &&
              m3World_Hash(restored) == m3World_Hash(world),
          "and its snapshot restores");
    m3DestroyWorld(restored);
    free(snapshot);
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
            int32_t bytes = StopJournal(world);
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
        CHECK(m3MotorJoint_GetMaxForce(world, ball) == 0.0f &&
                  m3MotorJoint_GetLinearOffset(world, ball).x == 0.0f,
              "a spherical's servo getters read zeros");

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
    TestMotorJointServo();
    TestJointWalls();
    if (s_failures == 0)
    {
        printf("test_newjoints: all passed\n");
        return 0;
    }
    return 1;
}
