// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The runtime-control gate: teleports wake both
// neighborhoods, the kinematic servo lands ON its target, type
// flips rebuild the books, disabled bodies vanish everywhere,
// motion locks hold their axes exact, sleep knobs obey, and every
// op journals and rolls back onto identical bits.

#include "maul3d/shape.h"
#include "test_harness.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Motion locks from bits 0..5: linear x, y, z, then angular x, y, z.
static m3MotionLocks Locks(uint32_t bits)
{
    m3MotionLocks l = {(bits & 0x01u) != 0, (bits & 0x02u) != 0, (bits & 0x04u) != 0,
                       (bits & 0x08u) != 0, (bits & 0x10u) != 0, (bits & 0x20u) != 0};
    return l;
}

static uint32_t Bits(m3MotionLocks l)
{
    return (l.linearX ? 0x01u : 0u) | (l.linearY ? 0x02u : 0u) | (l.linearZ ? 0x04u : 0u) |
           (l.angularX ? 0x08u : 0u) | (l.angularY ? 0x10u : 0u) | (l.angularZ ? 0x20u : 0u);
}

static m3WorldId PlaneWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

static m3BodyId Crate(m3WorldId world, m3Pos3 at)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return body;
}

static void TestTeleportWakes(void)
{
    // A crate sleeps on the plane; another crate teleports right
    // above it: BOTH the sleeper and the traveler end awake, and
    // the sleeper takes the landing.
    m3WorldId world = PlaneWorld();
    m3BodyId sleeper = Crate(world, (m3Pos3){0.0, 0.5, 0.0});
    m3BodyId traveler = Crate(world, (m3Pos3){10.0, 0.5, 0.0});
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m3Body_IsAwake(sleeper), "the crate sleeps before the teleport");
    m3Body_SetTransform(traveler, (m3Pos3){0.0, 2.0, 0.0}, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
    CHECK(m3Body_IsAwake(sleeper), "the arrival neighborhood wakes");
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 t = m3Body_GetPosition(traveler);
    CHECK(t.y > 1.3 && t.y < 1.7, "the traveler lands on the sleeper");
    m3DestroyWorld(world);
}

static void TestKinematicServo(void)
{
    // The servo lands the kinematic slab exactly on its target
    // after one step, then holds (the order clears).
    m3WorldId world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId slab = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(slab, &sd, (m3Vec3){1.0f, 0.2f, 1.0f});

    m3Body_SetTargetTransform(slab, (m3Pos3){1.5, 2.5, -0.5}, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3Pos3 p = m3Body_GetPosition(slab);
    CHECK(fabs(p.x - 1.5) < 1.0e-4 && fabs(p.y - 2.5) < 1.0e-4 && fabs(p.z + 0.5) < 1.0e-4,
          "the servo lands on the target in one step");
    // Documented contract: the servo clears its order but leaves
    // the exit velocity on the body; the host zeroes it or issues
    // the next target. Zero it here and prove no further pull.
    m3Body_SetLinearVelocity(slab, (m3Vec3){0.0f, 0.0f, 0.0f});
    m3Body_SetAngularVelocity(slab, (m3Vec3){0.0f, 0.0f, 0.0f});
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3Pos3 q = m3Body_GetPosition(slab);
    CHECK(fabs(q.x - p.x) < 1.0e-6 && fabs(q.y - p.y) < 1.0e-6,
          "the order cleared: no further servo pull");
    m3DestroyWorld(world);
}

static void TestTypeFlipAndDisable(void)
{
    // A static shelf HOVERS with a crate on top, then turns
    // DYNAMIC: both fall (a grounded pillar would just stand, the
    // first draft of this test learned that premise the hard way).
    // Then a crate disables mid-scene: contacts and queries lose
    // it, and enabling brings it back awake.
    m3WorldId world = PlaneWorld();
    m3BodyDef pd = m3DefaultBodyDef();
    pd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId shelf = m3CreateBody(world, &pd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(shelf, &sd, (m3Vec3){1.0f, 0.2f, 1.0f});
    m3BodyId crate = Crate(world, (m3Pos3){0.0, 3.8, 0.0});
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double restY = m3Body_GetPosition(crate).y;
    CHECK(restY > 3.5, "the crate rests on the hovering static shelf");
    m3Body_SetType(shelf, m3_dynamicBody);
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(crate).y < 1.8, "the freed shelf drops its rider");

    // Disable: the crate vanishes from a ray and from contacts.
    m3BodyId lid = Crate(world, (m3Pos3){10.0, 0.5, 0.0});
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3RayCastResult before = m3World_CastRayClosest(
        world, (m3Pos3){10.0, 5.0, 0.0}, (m3Vec3){0.0f, -6.0f, 0.0f}, m3DefaultQueryFilter());
    CHECK(before.hit && before.point.y > 0.9, "the ray sees the enabled crate");
    m3Body_Disable(lid);
    CHECK(!m3Body_IsEnabled(lid), "disabled reads back");
    m3RayCastResult after = m3World_CastRayClosest(
        world, (m3Pos3){10.0, 5.0, 0.0}, (m3Vec3){0.0f, -6.0f, 0.0f}, m3DefaultQueryFilter());
    CHECK(after.hit && after.point.y < 0.1, "the ray passes through the disabled crate");
    m3Body_Enable(lid);
    CHECK(m3Body_IsEnabled(lid) && m3Body_IsAwake(lid), "enabling wakes the body");
    m3DestroyWorld(world);
}

static void TestMotionLocks(void)
{
    // An upright 2.5D crate: linear z and ALL spin locked (a free
    // angular z let the first draft's crate trip over its leading
    // edge and tumble home: physical, but noise here). Shoved
    // diagonally, it slides in x only and never rotates.
    m3WorldId world = PlaneWorld();
    m3BodyId crate = Crate(world, (m3Pos3){0.0, 0.5, 0.0});
    // lock linear z (bit 2), angular x y z (bits 3, 4, 5).
    m3Body_SetMotionLocks(crate, Locks((1u << 2) | (1u << 3) | (1u << 4) | (1u << 5)));
    CHECK(Bits(m3Body_GetMotionLocks(crate)) == 0x3Cu, "locks read back");
    m3Body_ApplyLinearImpulse(crate, (m3Vec3){3.0f, 0.0f, 3.0f});
    m3Body_ApplyAngularImpulse(crate, (m3Vec3){0.0f, 0.4f, 0.0f});
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(crate);
    m3Quat q = m3Body_GetRotation(crate);
    // The criterion is the lock asymmetry (x free, z frozen), not
    // the slide distance, which the friction tests pin down.
    CHECK(p.x > 0.05, "the unlocked axis carries the shove");
    CHECK(fabs(p.z) < 1.0e-4, "the locked linear axis never moves");
    CHECK(fabsf(q.x) < 1.0e-4f && fabsf(q.y) < 1.0e-4f && fabsf(q.z) < 1.0e-4f,
          "the locked spin axes never turn");
    m3DestroyWorld(world);
}

static void TestSleepKnobsAndControlReplay(void)
{
    // A can-never-sleep crate stays awake forever; a forced sleep
    // freezes one instantly. The whole control script journals and
    // rolls back bit-exact.
    static uint8_t journal[262144];
    static uint8_t snap[786432];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 32;
        def.shapeCapacity = 32;
        m3WorldId world = m3CreateWorld(&def);
        bool recording = run == 0 && m3World_StartJournal(world, journal, (int32_t)sizeof(journal));
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sg = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3CreatePlaneShape(ground, &sg, &floor);
        m3BodyId restless = Crate(world, (m3Pos3){0.0, 0.5, 0.0});
        m3BodyId normal = Crate(world, (m3Pos3){3.0, 0.5, 0.0});
        m3Body_EnableSleep(restless, false);

        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 260; ++i)
        {
            if (i == 40)
            {
                m3Body_SetMotionLocks(normal, Locks(1u << 4));
            }
            if (i == 80)
            {
                m3Body_SetTransform(restless, (m3Pos3){0.0, 3.0, 1.0},
                                    (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
            }
            if (i == 160)
            {
                m3Body_SetAwake(normal, false); // force-sleep
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 120 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the control snapshot fits");
            }
        }
        CHECK(m3Body_IsAwake(restless), "the can-never-sleep crate is still awake");
        CHECK(!m3Body_IsAwake(normal), "the forced sleeper stays down");
        uint64_t final = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_StopJournal(world);
            CHECK(bytes > 0, "the control session records");
            m3WorldId fresh = m3CreateWorld(&def);
            CHECK(m3World_ReplayJournal(fresh, journal, bytes), "the control session replays");
            CHECK(m3World_Hash(fresh) == final, "the replay is bit-identical");
            m3DestroyWorld(fresh);
            CHECK(m3World_Restore(world, snap, snapBytes), "the control restore lands");
            for (int32_t i = 121; i < 260; ++i)
            {
                if (i == 160)
                {
                    m3Body_SetAwake(normal, false);
                }
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == final, "the re-run is bit-identical");
        }
        hashes[run] = final;
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "control twins are bit-identical");
}

static void TestDefsCarrySettings(void)
{
    // Every body, shape and world setting a setter changes can also be
    // given at creation, and lands the same way.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.maximumAngularSpeed = 5.0f;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef belt = m3DefaultShapeDef();
    belt.surfaceVelocity = (m3Vec3){2.0f, 0.0f, 0.0f};
    m3CreateBoxShape(ground, &belt, (m3Vec3){20.0f, 0.5f, 20.0f});

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    bd.motionLocks = Locks(0x28u); // linear z and angular y
    bd.enableSleep = false;
    bd.sleepThreshold = 0.25f;
    m3BodyId crate = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    CHECK(Bits(m3Body_GetMotionLocks(crate)) == 0x28u, "locks from the def");
    CHECK(!m3Body_IsSleepEnabled(crate), "sleep switch from the def");
    CHECK(m3Body_GetSleepThreshold(crate) == 0.25f, "sleep threshold from the def");

    bd.position = (m3Pos3){6.0, 4.0, 0.0};
    bd.motionLocks = Locks(0u);
    bd.angularVelocity = (m3Vec3){0.0f, 50.0f, 0.0f};
    m3BodyId spinner = m3CreateBody(world, &bd);
    m3CreateBoxShape(spinner, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    bd.enableFastRotation = true;
    bd.position = (m3Pos3){-6.0, 4.0, 0.0};
    m3BodyId wheel = m3CreateBody(world, &bd);
    m3CreateBoxShape(wheel, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    CHECK(m3Body_IsFastRotationEnabled(wheel), "fast rotation from the def");

    bd.isEnabled = false;
    bd.position = (m3Pos3){0.0, 8.0, 6.0};
    m3BodyId dormant = m3CreateBody(world, &bd);
    CHECK(!m3Body_IsEnabled(dormant), "created disabled from the def");

    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetLinearVelocity(crate).x > 1.0f,
          "the belt from the shape def carries the crate");
    CHECK(m3Body_GetPosition(dormant).y == 8.0, "a disabled body does not fall");
    m3Vec3 w = m3Body_GetAngularVelocity(spinner);
    CHECK(w.x * w.x + w.y * w.y + w.z * w.z <= 25.0f * 1.0001f, "the world def caps angular speed");
    m3Vec3 fast = m3Body_GetAngularVelocity(wheel);
    CHECK(fast.y > 5.0f, "a fast-rotation body passes the cap");
    m3DestroyWorld(world);
}

static void TestSettersReplay(void)
{
    // Gravity scale, damping, the bullet flag and user data change at
    // runtime, read back, and a recorded session replays them.
    static uint8_t tape[262144];
    m3WorldDef def = m3DefaultWorldDef();
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_StartJournal(world, tape, (int32_t)sizeof(tape)), "recording starts");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId shape = m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3Body_SetGravityScale(body, -0.5f);
    m3Body_SetLinearDamping(body, 0.3f);
    m3Body_SetAngularDamping(body, 0.6f);
    m3Body_SetBullet(body, true);
    m3Body_SetUserData(body, 7u);
    m3Shape_SetUserData(shape, 9u);
    m3Body_SetLinearDamping(body, -1.0f);
    CHECK(m3LastResult() == m3_errorInvalid, "negative damping is refused");
    CHECK(m3Body_GetGravityScale(body) == -0.5f && m3Body_GetLinearDamping(body) == 0.3f &&
              m3Body_GetAngularDamping(body) == 0.6f && m3Body_IsBullet(body) &&
              m3Body_GetUserData(body) == 7u && m3Shape_GetUserData(shape) == 9u,
          "the setters read back");
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(body).y > 0.1, "negative gravity scale lifts the body");
    int32_t bytes = m3World_StopJournal(world);
    m3WorldId twin = m3CreateWorld(&def);
    CHECK(m3World_ReplayJournal(twin, tape, bytes), "the session replays");
    CHECK(m3World_Hash(twin) == m3World_Hash(world), "onto the same bits");
    m3BodyId copy;
    CHECK(m3World_GetBodies(twin, &copy, 1) == 1, "the twin has the body");
    m3ShapeId copyShape;
    m3Body_GetShapes(copy, &copyShape, 1);
    CHECK(m3Body_GetLinearDamping(copy) == 0.3f && m3Body_IsBullet(copy) &&
              m3Body_GetUserData(copy) == 7u && m3Shape_GetUserData(copyShape) == 9u,
          "the replayed settings match");
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

int main(void)
{
    TestTeleportWakes();
    TestKinematicServo();
    TestTypeFlipAndDisable();
    TestMotionLocks();
    TestSleepKnobsAndControlReplay();
    TestDefsCarrySettings();
    TestSettersReplay();
    if (s_failures == 0)
    {
        printf("test_bodycontrol: all green\n");
        return 0;
    }
    printf("test_bodycontrol: %d failure(s)\n", s_failures);
    return 1;
}
