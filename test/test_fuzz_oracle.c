// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The value oracle of the snapshot fuzz target (fuzz_oracle.h). It holds
// for the fuzz scene as the engine leaves it and as a restore of it
// leaves it, and it fails a world holding a value no caller could have
// set, including a restored one that a restore check alone would let
// through.

#include "fuzz_oracle.h"
#include "fuzz_scene.h"
#include "joint_solver.h"
#include "state_prefix.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* SceneWorld(FuzzScene* scene)
{
    m3WorldDef def = FuzzWorldDef();
    m3World* world = NewWorld(&def);
    CHECK(FuzzBuild(world, scene) && FuzzCountsHold(world, scene), "the fuzz scene builds");
    FuzzSession(world, scene);
    return world;
}

// The slot of the scene's one joint of a type, or -1.
static int32_t JointSlotOfType(const m3World* world, m3JointType type)
{
    m3JointId joints[8];
    int32_t count = m3World_GetJoints(world, joints, 8);
    for (int32_t i = 0; i < count && i < 8; ++i)
    {
        if (m3Joint_GetType(world, joints[i]) == type)
        {
            return (int32_t)joints[i].index1 - 1;
        }
    }
    return -1;
}

// The rope between the crate and the ball.
static int32_t RopeSlot(const m3World* world)
{
    return JointSlotOfType(world, m3_distanceJoint);
}

// The shoulder the ball hangs from, its cone on.
static int32_t ShoulderSlot(const m3World* world)
{
    return JointSlotOfType(world, m3_sphericalJoint);
}

static void TestSceneHolds(void)
{
    FuzzScene scene;
    m3World* world = SceneWorld(&scene);
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snapshot = (uint8_t*)malloc((size_t)size);
    CHECK(snapshot != NULL && Snapshot(world, snapshot, size) == size, "snapshot taken");
    CHECK(FuzzValuesHold(world, scene.hero), "the scene's values are a caller's");
    m3WorldDef def = FuzzWorldDef();
    m3World* restored = NewWorld(&def);
    CHECK(snapshot != NULL && Restore(restored, snapshot, size), "the scene restores");
    CHECK(FuzzValuesHold(restored, scene.hero), "and so are the restored world's");
    m3DestroyWorld(restored);
    free(snapshot);
    m3DestroyWorld(world);
}

typedef enum Corruption
{
    corrupt_ropeOrder,
    corrupt_ropeMinimum,
    corrupt_ropeLength,
    corrupt_springHertz,
    corrupt_coneAngle,
    corrupt_gravityScale,
    corrupt_friction,
    corrupt_gravity,
    corrupt_rotation,
} Corruption;

// Writes a value no setter takes straight into the scene's state.
static void Corrupt(m3World* world, const FuzzScene* scene, Corruption which)
{
    int32_t rope = RopeSlot(world);
    int32_t crate = (int32_t)scene->crate.index1 - 1;
    m3ShapeId shapes[4];
    int32_t shapeCount = m3Body_GetShapes(world, scene->crate, shapes, 4);
    switch (which)
    {
    case corrupt_ropeOrder:
        world->joints.jointLimits[rope].x = 3.0f;
        world->joints.jointLimits[rope].y = 1.0f;
        break;
    case corrupt_ropeMinimum:
        world->joints.jointLimits[rope].x = -1.0f;
        break;
    case corrupt_ropeLength:
        // The rest length rides the limit slot's z.
        world->joints.jointLimits[rope].z = -1.0f;
        break;
    case corrupt_springHertz:
        world->joints.jointSpring[rope].x = 2.0f * M3_HERTZ_LIMIT;
        break;
    case corrupt_coneAngle:
        // The cone angle rides the limit slot's z.
        world->joints.jointLimits[ShoulderSlot(world)].z = -0.5f;
        break;
    case corrupt_gravityScale:
        world->bodies.gravityScales[crate] = 2.0f * M3_RATIO_LIMIT;
        break;
    case corrupt_friction:
        CHECK(shapeCount > 0, "the crate has a shape");
        world->shapes.shapeFriction[shapes[0].index1 - 1] = -1.0f;
        break;
    case corrupt_gravity:
        world->gravity.y = -2.0f * M3_ACCEL_LIMIT;
        break;
    case corrupt_rotation:
        // Twice a unit quaternion: no pose a caller could set.
        world->bodies.transforms[crate].q.w *= 2.0f;
        break;
    }
}

static void TestOracleCatches(void)
{
    static const struct
    {
        Corruption which;
        const char* name;
    } cases[] = {
        {corrupt_ropeOrder, "an unordered rope range"},
        {corrupt_ropeMinimum, "a negative rope minimum"},
        {corrupt_ropeLength, "a negative rope rest length"},
        {corrupt_springHertz, "a rope spring frequency past its limit"},
        {corrupt_coneAngle, "a negative angle under a switched-on cone"},
        {corrupt_gravityScale, "a gravity scale past its limit"},
        {corrupt_friction, "a negative friction"},
        {corrupt_gravity, "a world gravity past its limit"},
        {corrupt_rotation, "a body rotation far from unit length"},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c)
    {
        FuzzScene scene;
        m3World* world = SceneWorld(&scene);
        CHECK(RopeSlot(world) >= 0 && ShoulderSlot(world) >= 0, "the rope and the shoulder live");
        Corrupt(world, &scene, cases[c].which);
        bool held = FuzzValuesHold(world, scene.hero);
        if (held)
        {
            printf("  the oracle let through %s\n", cases[c].name);
        }
        CHECK(!held, "the oracle fails a value no caller could have set");
        m3DestroyWorld(world);
    }
}

// Snapshots of the scene with a rope range no caller could set: the limit
// bit cleared over (3, 1), and the bit kept over (-1, 2). Restore refuses
// each, and were one accepted, the oracle the fuzz target runs would fail
// the world it left.
static void TestRestoredRopeFailsOracle(void)
{
    static const struct
    {
        bool limitBit;
        float range[2];
        const char* name;
    } cases[] = {
        {false, {3.0f, 1.0f}, "the bit cleared over an unordered range"},
        {true, {-1.0f, 2.0f}, "a negative minimum"},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c)
    {
        FuzzScene scene;
        m3World* world = SceneWorld(&scene);
        int32_t rope = RopeSlot(world);
        int64_t size = m3World_GetSnapshotSize(world);
        uint8_t* snapshot = (uint8_t*)malloc((size_t)size);
        CHECK(snapshot != NULL && Snapshot(world, snapshot, size) == size, "snapshot taken");
        int64_t limits = m3StateFieldAt(world, offsetof(m3World, joints.jointLimits));
        int64_t flags = m3StateFieldAt(world, offsetof(m3World, joints.jointFlags));
        CHECK(limits >= 0 && flags >= 0 && rope >= 0, "the rope's range is snapshot state");
        if (snapshot == NULL || limits < 0 || flags < 0 || rope < 0)
        {
            free(snapshot);
            m3DestroyWorld(world);
            return;
        }
        uint8_t* bits = snapshot + M3_SNAPSHOT_HEADER_BYTES + flags + (size_t)rope;
        *bits = cases[c].limitBit ? (uint8_t)(*bits | M3_JOINT_LIMIT)
                                  : (uint8_t)(*bits & ~M3_JOINT_LIMIT);
        memcpy(snapshot + M3_SNAPSHOT_HEADER_BYTES + limits + sizeof(m3Vec3) * (size_t)rope,
               cases[c].range, sizeof(cases[c].range));
        m3WorldDef def = FuzzWorldDef();
        m3World* restored = NewWorld(&def);
        bool accepted = m3World_Restore(restored, snapshot, size) == m3_success;
        if (accepted)
        {
            printf("  restore took %s\n", cases[c].name);
            CHECK(!FuzzValuesHold(restored, scene.hero), "and the oracle fails the world it left");
        }
        CHECK(!accepted, "restore refuses a rope range no caller could set");
        m3DestroyWorld(restored);
        free(snapshot);
        m3DestroyWorld(world);
    }
}

// A snapshot of the scene with the shoulder's cone angle set to -0.5 under
// its switched-on cone. Restore refuses it, and were it accepted, the
// oracle would fail the world it left.
static void TestRestoredConeFailsOracle(void)
{
    FuzzScene scene;
    m3World* world = SceneWorld(&scene);
    int32_t shoulder = ShoulderSlot(world);
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snapshot = (uint8_t*)malloc((size_t)size);
    int64_t limits = m3StateFieldAt(world, offsetof(m3World, joints.jointLimits));
    bool ready = snapshot != NULL && Snapshot(world, snapshot, size) == size && limits >= 0 &&
                 shoulder >= 0 && m3SphericalJoint_IsConeEnabled(world, scene.shoulder);
    CHECK(ready, "the shoulder's switched-on cone is snapshot state");
    if (ready)
    {
        const float angle = -0.5f;
        memcpy(snapshot + M3_SNAPSHOT_HEADER_BYTES + limits + sizeof(m3Vec3) * (size_t)shoulder +
                   2 * sizeof(float),
               &angle, sizeof(angle));
        m3WorldDef def = FuzzWorldDef();
        m3World* restored = NewWorld(&def);
        bool accepted = m3World_Restore(restored, snapshot, size) == m3_success;
        if (accepted)
        {
            printf("  restore took a negative angle under a switched-on cone\n");
            CHECK(!FuzzValuesHold(restored, scene.hero), "and the oracle fails the world it left");
        }
        CHECK(!accepted, "restore refuses a cone no caller could set");
        m3DestroyWorld(restored);
    }
    free(snapshot);
    m3DestroyWorld(world);
}

int main(void)
{
    TestSceneHolds();
    TestOracleCatches();
    TestRestoredRopeFailsOracle();
    TestRestoredConeFailsOracle();
    if (s_failures == 0)
    {
        printf("test_fuzz_oracle: all checks passed\n");
        return 0;
    }
    return 1;
}
