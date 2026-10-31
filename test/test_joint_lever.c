// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint lever ratio (src/joint_lever.h) at every door that sets it. A
// thin rod, a 9e3 m box of half thickness t at density 1e6, hangs by an
// anchor 1e4 m off its axis, where the anchor turns it about that axis:
// at t = 0.25 its ratio is about 2.4e9, within M3_JOINT_LEVER_LIMIT, and
// at t = 0.0625 about 3.8e10, past it. Create, a shape's create, destroy,
// resize and density, a body's type, restore and replay each refuse what
// would carry a joint that solves blocks past the ratio, and change
// nothing; voxel edits and carves bear no mass, so they never move a
// jointed body's.

#include "helpers.h"
#include "joint_lever.h"
#include "journal.h"
#include "state_prefix.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A voxel chunk's cells per axis.
#define VOXEL_DIM 16

static const float s_fit = 0.25f;
static const float s_past = 0.0625f;
static const float s_arm = 1.0e4f;

static m3World* LeverWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, -9.81f};
    return NewWorld(&def);
}

static m3BodyId Ground(m3World* world)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 100.0, 0.0};
    return NewBody(world, &bd);
}

static m3BodyId Rod(m3World* world, float half)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 100.0 - (double)s_arm, 0.0};
    m3BodyId rod = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 1.0e6f;
    sd.maskBits = 0;
    NewBoxShape(world, rod, &sd, (m3Vec3){9.0e3f, half, half});
    return rod;
}

// A joint of this type from a's center to the anchor s_arm above b's.
static m3JointDef LeverDef(m3JointType type, m3BodyId a, m3BodyId b)
{
    m3JointDef jd = m3DefaultJointDef();
    jd.type = type;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.localAnchorB = (m3Vec3){0.0f, s_arm, 0.0f};
    jd.localAxisA = type == m3_wheelJoint ? (m3Vec3){0.0f, 1.0f, 0.0f} : (m3Vec3){1.0f, 0.0f, 0.0f};
    jd.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f};
    return jd;
}

static float Radius(const m3World* world, m3ShapeId shape)
{
    return world->shapes.shapeGeom[shape.index1 - 1].s;
}

static uint64_t Misuse(const m3World* world)
{
    return m3World_GetCounters(world).misuseCount;
}

// Every type that solves blocks takes the rod within the ratio and
// refuses the one past it with m3_errorLimit, counting no misuse, since
// the bodies' masses decide it; a distance or generic
// joint, which solves single rows, takes either.
static void TestCreateKeepsTheRatio(void)
{
    const m3JointType blocks[6] = {m3_sphericalJoint, m3_revoluteJoint, m3_prismaticJoint,
                                   m3_weldJoint,      m3_wheelJoint,    m3_motorJoint};
    for (int32_t k = 0; k < 6; ++k)
    {
        m3World* world = LeverWorld();
        m3BodyId ground = Ground(world);
        m3BodyId fit = Rod(world, s_fit);
        m3BodyId past = Rod(world, s_past);
        m3JointDef jd = LeverDef(blocks[k], ground, fit);
        m3JointId joint = m3_nullJointId;
        CHECK(m3CreateJoint(world, &jd, &joint) == m3_success, "a joint within the ratio creates");
        uint64_t misuse = Misuse(world);
        int32_t joints = m3World_GetCounters(world).jointCount;
        jd = LeverDef(blocks[k], ground, past);
        CHECK(m3CreateJoint(world, &jd, &joint) == m3_errorLimit, "one past the ratio is refused");
        CHECK(Misuse(world) == misuse && m3World_GetCounters(world).jointCount == joints,
              "counting no misuse and creating nothing");
        m3DestroyWorld(world);
    }
    const m3JointType rows[2] = {m3_distanceJoint, m3_genericJoint};
    for (int32_t k = 0; k < 2; ++k)
    {
        m3World* world = LeverWorld();
        m3JointDef jd = LeverDef(rows[k], Ground(world), Rod(world, s_past));
        if (rows[k] == m3_distanceJoint)
        {
            jd.minLength = 0.0f;
            jd.maxLength = 1.0f;
        }
        if (rows[k] == m3_genericJoint)
        {
            for (int32_t a = 0; a < 3; ++a)
            {
                jd.genericLinear[a] = m3_axisLocked;
                jd.genericAngular[a] = m3_axisFree;
            }
        }
        m3JointId joint = m3_nullJointId;
        CHECK(m3CreateJoint(world, &jd, &joint) == m3_success,
              "a joint of single rows takes the rod past the ratio");
        m3DestroyWorld(world);
    }
}

// The mass of each body, as the step reads it.
typedef struct MassBits
{
    float invMass;
    m3Mat3 invInertia;
    m3Mat3 inertia;
    m3Vec3 center;
} MassBits;

static MassBits MassOf(const m3World* world, m3BodyId body)
{
    int32_t b = body.index1 - 1;
    MassBits bits;
    memset(&bits, 0, sizeof(bits));
    bits.invMass = world->bodies.invMass[b];
    bits.invInertia = world->bodies.invInertiaLocal[b];
    bits.inertia = world->bodies.inertiaLocal[b];
    bits.center = world->bodies.localCenters[b];
    return bits;
}

static bool SameMass(const MassBits* a, const MassBits* b)
{
    return memcmp(a, b, sizeof(*a)) == 0;
}

// A ball of radius 5 holds the rod past the ratio from its own center.
// Light, it carries the joint: its inverse mass dwarfs the rod's turn.
// Heavy, or made static, it no longer does, so a density change, a second
// heavy shape and a type change that would make it so are each refused
// with m3_errorLimit, count no misuse and change nothing; a light enough
// change is taken.
static void TestBallChangesKeepTheRatio(void)
{
    m3World* world = LeverWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 100.0, 0.0};
    bd.gravityScale = 0.0f;
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.maskBits = 0;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 5.0f};
    m3ShapeId shape = NewSphereShape(world, ball, &sd, &sphere);
    m3BodyId rod = Rod(world, s_past);
    m3JointDef jd = LeverDef(m3_sphericalJoint, ball, rod);
    m3JointId joint = m3_nullJointId;
    CHECK(m3CreateJoint(world, &jd, &joint) == m3_success, "the light ball carries the rod");
    MassBits before = MassOf(world, ball);
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = Misuse(world);
    float density = m3Shape_GetDensity(world, shape);

    CHECK(m3Shape_SetDensity(world, shape, 1.0e6f) == m3_errorLimit &&
              m3Shape_GetDensity(world, shape) == density && Misuse(world) == misuse,
          "a density that makes it heavy is refused, counting no misuse");
    m3ShapeId heavy = m3_nullShapeId;
    sd.density = 1.0e6f;
    CHECK(m3CreateSphereShape(world, ball, &sd, &sphere, &heavy) == m3_errorLimit &&
              Misuse(world) == misuse,
          "so is a second shape that does");
    CHECK(m3Body_SetType(world, ball, m3_staticBody) == m3_errorLimit &&
              m3Body_GetType(world, ball) == m3_dynamicBody && Misuse(world) == misuse,
          "and turning it static");
    CHECK(m3Body_SetType(world, ball, m3_kinematicBody) == m3_errorLimit &&
              m3Body_GetType(world, ball) == m3_dynamicBody && Misuse(world) == misuse,
          "or kinematic");
    MassBits after = MassOf(world, ball);
    CHECK(SameMass(&before, &after) && m3World_Hash(world) == hash,
          "and the world is left as it was");
    CHECK(m3Shape_SetDensity(world, shape, 2000.0f) == m3_success &&
              m3Shape_GetDensity(world, shape) == 2000.0f && Misuse(world) == misuse,
          "a density that keeps it light is taken");
    m3DestroyWorld(world);
}

// The rod past the ratio with a ball of radius 1.5 at its center, which
// adds enough moment about its axis to bring the pair within it. Removing
// the ball or shrinking it is refused and changes nothing; growing it is
// taken.
static m3World* RodWithBall(m3BodyId* rodOut, m3ShapeId* ballOut)
{
    m3World* world = LeverWorld();
    m3BodyId ground = Ground(world);
    m3BodyId rod = Rod(world, s_past);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 1.0e6f;
    sd.maskBits = 0;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.5f};
    *ballOut = NewSphereShape(world, rod, &sd, &sphere);
    m3JointDef jd = LeverDef(m3_revoluteJoint, ground, rod);
    m3JointId joint = m3_nullJointId;
    CHECK(m3CreateJoint(world, &jd, &joint) == m3_success, "the rod with its ball is jointed");
    *rodOut = rod;
    return world;
}

static void TestRodChangesKeepTheRatio(void)
{
    m3BodyId rod;
    m3ShapeId ball;
    m3World* world = RodWithBall(&rod, &ball);
    MassBits before = MassOf(world, rod);
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = Misuse(world);
    CHECK(m3DestroyShape(world, ball) == m3_errorLimit && m3Shape_IsValid(world, ball) &&
              Misuse(world) == misuse,
          "removing the ball is refused, counting no misuse");
    m3Sphere small = {{0.0f, 0.0f, 0.0f}, 0.5f};
    CHECK(m3Shape_SetSphere(world, ball, &small) == m3_errorLimit && Radius(world, ball) == 1.5f &&
              Misuse(world) == misuse,
          "so is shrinking it");
    MassBits after = MassOf(world, rod);
    CHECK(SameMass(&before, &after) && m3World_Hash(world) == hash,
          "and the world is left as it was");
    m3Sphere large = {{0.0f, 0.0f, 0.0f}, 2.0f};
    CHECK(m3Shape_SetSphere(world, ball, &large) == m3_success && Radius(world, ball) == 2.0f &&
              Misuse(world) == misuse,
          "growing it is taken");
    m3DestroyWorld(world);
}

// A snapshot of the rod with its ball restores; the same bytes with the
// ball's density at 1 kg/m^3, which leaves the rod past the ratio, are
// refused with the world left as it was and no misuse counted.
static void TestRestoreKeepsTheRatio(void)
{
    m3BodyId rod;
    m3ShapeId ball;
    m3World* world = RodWithBall(&rod, &ball);
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "the world snapshots");
    if (snap != NULL)
    {
        uint64_t hash = m3World_Hash(world);
        uint64_t misuse = Misuse(world);
        CHECK(Restore(world, snap, size) && m3World_Hash(world) == hash, "and restores");
        int64_t at = m3StateFieldAt(world, offsetof(m3World, shapes.shapeDensity));
        CHECK(at >= 0, "the density is snapshot state");
        const float light = 1.0f;
        memcpy(snap + M3_SNAPSHOT_HEADER_BYTES + (at < 0 ? 0 : at) +
                   sizeof(float) * (size_t)(ball.index1 - 1),
               &light, sizeof(light));
        CHECK(m3World_Restore(world, snap, size) == m3_errorFormat,
              "a light ball in the bytes is refused");
        CHECK(m3World_Hash(world) == hash && Misuse(world) == misuse,
              "with the world left as it was and no misuse counted");
    }
    free(snap);
    m3DestroyWorld(world);
}

// The tape that builds the rod with its ball replays to the same world;
// the same tape with the ball's density at 1 kg/m^3 in its create, which
// leaves the joint past the ratio, is refused.
static void TestReplayKeepsTheRatio(void)
{
    enum
    {
        TAPE_BYTES = 1 << 16
    };
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    m3World* world = LeverWorld();
    CHECK(tape != NULL && StartJournal(world, tape, TAPE_BYTES), "the journal starts");
    if (tape == NULL)
    {
        m3DestroyWorld(world);
        return;
    }
    m3BodyId ground = Ground(world);
    m3BodyId rod = Rod(world, s_past);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 1.0e6f;
    sd.maskBits = 0;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.5f};
    (void)NewSphereShape(world, rod, &sd, &sphere);
    m3JointDef jd = LeverDef(m3_revoluteJoint, ground, rod);
    m3JointId joint = m3_nullJointId;
    CHECK(m3CreateJoint(world, &jd, &joint) == m3_success, "the rod with its ball is jointed");
    int64_t bytes = StopJournal(world);
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);

    m3World* again = LeverWorld();
    CHECK(ReplayJournal(again, tape, bytes) && m3World_Hash(again) == hash, "the tape replays");
    m3DestroyWorld(again);

    // The last shape create on the tape is the ball's.
    int32_t ball = -1;
    for (int32_t at = M3_JOURNAL_HEADER_BYTES; at + 8 <= bytes;)
    {
        int32_t op;
        int32_t size;
        memcpy(&op, tape + at, sizeof(op));
        memcpy(&size, tape + at + 4, sizeof(size));
        ball = op == (int32_t)m3_opCreateShape ? at + 8 : ball;
        at += 8 + size;
    }
    CHECK(ball >= 0, "the ball's create is on the tape");
    if (ball >= 0)
    {
        const float light = 1.0f;
        memcpy(tape + ball + offsetof(m3CreateShapeOp, def) + offsetof(m3ShapeDef, density), &light,
               sizeof(light));
        m3World* edited = LeverWorld();
        CHECK(m3World_ReplayJournal(edited, tape, bytes) == m3_errorFormat,
              "the tape with a light ball is refused");
        m3DestroyWorld(edited);
    }
    free(tape);
}

// A dynamic box hangs from a static voxel wall. Voxel edits and a blast
// that carves the wall leave both bodies' mass bit for bit, since voxels
// bear none: no carve can carry a joint past its ratio.
static void TestVoxelEditsKeepJointedMass(void)
{
    m3World* world = LeverWorld();
    uint8_t voxels[VOXEL_DIM * VOXEL_DIM * VOXEL_DIM];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t i = 0; i < VOXEL_DIM * VOXEL_DIM * 8; ++i)
    {
        voxels[i] = 1;
    }
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId holder = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId wall = NewVoxelChunkShape(world, holder, &sd, voxels, NULL, 0.5f);
    CHECK(m3Shape_IsValid(world, wall), "the voxel wall creates");
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){4.0, 3.0, 4.0};
    m3BodyId box = NewBody(world, &bd);
    NewBoxShape(world, box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3JointDef jd = LeverDef(m3_sphericalJoint, holder, box);
    jd.localAnchorA = (m3Vec3){4.0f, 4.0f, 4.0f};
    jd.localAnchorB = (m3Vec3){0.0f, 1.0f, 0.0f};
    m3JointId joint = m3_nullJointId;
    CHECK(m3CreateJoint(world, &jd, &joint) == m3_success, "the box hangs from the wall");
    MassBits holderBefore = MassOf(world, holder);
    MassBits boxBefore = MassOf(world, box);
    CHECK(m3VoxelChunkShape_SetVoxel(world, wall, 1, 9, 1, 0) == m3_success, "a voxel lands");
    CHECK(m3VoxelChunkShape_ClearVoxel(world, wall, 2, 2, 2) == m3_success, "a voxel clears");
    const int32_t lo[3] = {5, 0, 5};
    const int32_t hi[3] = {7, 3, 7};
    int32_t cleared = 0;
    CHECK(m3VoxelChunkShape_ClearBox(world, wall, lo, hi, &cleared) == m3_success, "a box clears");
    m3ExplosionDef blast = m3DefaultExplosionDef();
    blast.position = (m3Pos3){4.0, 3.5, 4.0};
    blast.radius = 2.0f;
    blast.impulsePerArea = 10.0f;
    blast.voxelCarve = 1.5f;
    m3World_Explode(world, &blast);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    MassBits holderAfter = MassOf(world, holder);
    MassBits boxAfter = MassOf(world, box);
    CHECK(SameMass(&holderBefore, &holderAfter) && SameMass(&boxBefore, &boxAfter),
          "edits and a carve leave the jointed bodies' mass as it was");
    m3DestroyWorld(world);
}

int main(void)
{
    TestCreateKeepsTheRatio();
    TestBallChangesKeepTheRatio();
    TestRodChangesKeepTheRatio();
    TestRestoreKeepsTheRatio();
    TestReplayKeepsTheRatio();
    TestVoxelEditsKeepJointedMass();
    if (s_failures == 0)
    {
        printf("test_joint_lever: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
