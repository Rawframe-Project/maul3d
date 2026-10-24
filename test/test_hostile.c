// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hostile inputs. Every def field takes
// NaN, both infinities, denormals, and hostile magnitudes; every
// geometry family takes its degenerate forms. The contract under
// attack: every refusal is a null id or a no-op, never a crash and
// never a half-create; nothing non-finite ever reaches state; and
// the refusal paths themselves are bit-deterministic (twin worlds
// walking the same hostile sequence land on the same hash). The
// ASan and UBSan cells turn every out-of-contract memory touch
// into a loud failure.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The poison palette, built from bit patterns so no compiler in the
// six-cell matrix gets to fold, warn about, or trap on a constant:
// a quiet NaN, both infinities, and zero.
static float FromBits(uint32_t bits)
{
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}
static float s_poisons[4];
static void InitPoisons(void)
{
    s_poisons[0] = FromBits(0x7FC00000u); // quiet NaN
    s_poisons[1] = FromBits(0x7F800000u); // +inf
    s_poisons[2] = FromBits(0xFF800000u); // -inf
    s_poisons[3] = 0.0f;
}
#define M3_TEST_NAN (s_poisons[0])
#define M3_TEST_INF (s_poisons[1])

static uint32_t s_rng = 0xF00DFACEu;
static uint32_t NextRand(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return s_rng;
}

static m3World* FreshWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    def.meshCapacity = 2;
    return NewWorld(&def);
}

static void TestHostileWorldDefs(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, M3_TEST_NAN, 0.0f};
    m3World* w = NewWorld(&def);
    CHECK(w == NULL, "NaN gravity refuses the world");
    def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){M3_TEST_INF, 0.0f, 0.0f};
    w = NewWorld(&def);
    CHECK(w == NULL, "infinite gravity refuses the world");
}

static void TestHostileBodyDefs(void)
{
    m3World* world = FreshWorld();

    // Every float field of the body def, every poison, one at a
    // time: all must refuse with the null id.
    for (int32_t p = 0; p < 3; ++p) // 0.0f is legal for these
    {
        float bad = s_poisons[p];
        m3BodyDef defs[8];
        for (int32_t i = 0; i < 8; ++i)
        {
            defs[i] = m3DefaultBodyDef();
            defs[i].type = m3_dynamicBody;
        }
        defs[0].position = (m3Pos3){(double)bad, 0.0, 0.0};
        defs[1].position = (m3Pos3){0.0, (double)bad, 0.0};
        defs[2].rotation = (m3Quat){bad, 0.0f, 0.0f, 1.0f};
        defs[3].linearVelocity = (m3Vec3){0.0f, bad, 0.0f};
        defs[4].angularVelocity = (m3Vec3){0.0f, 0.0f, bad};
        defs[5].gravityScale = bad;
        defs[6].linearDamping = bad;
        defs[7].angularDamping = bad;
        for (int32_t i = 0; i < 8; ++i)
        {
            CHECK(!m3Body_IsValid(world, NewBody(world, &defs[i])),
                  "a poisoned body def field refuses");
        }
    }

    // A zero quaternion and a wildly non-unit one are corrupted
    // defs, not requests.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.rotation = (m3Quat){0.0f, 0.0f, 0.0f, 0.0f};
    CHECK(!m3Body_IsValid(world, NewBody(world, &bd)), "a zero quaternion refuses");
    bd = m3DefaultBodyDef();
    bd.rotation = (m3Quat){0.0f, 0.0f, 0.0f, 2.0f};
    CHECK(!m3Body_IsValid(world, NewBody(world, &bd)), "a non-unit quaternion refuses");
    bd = m3DefaultBodyDef();
    bd.linearDamping = -1.0f;
    CHECK(!m3Body_IsValid(world, NewBody(world, &bd)), "negative damping refuses");

    // Legal but hostile magnitudes must be ACCEPTED and survive: a
    // denormal position and a kilometers-away position are valid
    // worlds (the double position law exists exactly for this).
    bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){5.0e-320, 1.0, 0.0};
    m3BodyId denormal = NewBody(world, &bd);
    CHECK(m3Body_IsValid(world, denormal), "a denormal position is legal");
    bd.position = (m3Pos3){1.0e6, 1.0, -1.0e6};
    m3BodyId faraway = NewBody(world, &bd);
    CHECK(m3Body_IsValid(world, faraway), "a far position is legal");
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, denormal, &sd, &ball);
    NewSphereShape(world, faraway, &sd, &ball);
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(world, faraway);
    CHECK(p.x == 1.0e6 && p.z == -1.0e6, "the far body falls straight, no precision drama");
    m3DestroyWorld(world);
}

static void TestHostileShapeDefs(void)
{
    m3World* world = FreshWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId body = NewBody(world, &bd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);

    // Poisoned materials.
    for (int32_t p = 0; p < 3; ++p)
    {
        m3ShapeDef sd = m3DefaultShapeDef();
        sd.density = s_poisons[p];
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
        CHECK(!m3Shape_IsValid(world, NewSphereShape(world, body, &sd, &ball)),
              "poisoned density refuses");
        sd = m3DefaultShapeDef();
        sd.friction = s_poisons[p];
        CHECK(!m3Shape_IsValid(world, NewSphereShape(world, body, &sd, &ball)),
              "poisoned friction refuses");
        sd = m3DefaultShapeDef();
        sd.restitution = s_poisons[p];
        CHECK(!m3Shape_IsValid(world, NewSphereShape(world, body, &sd, &ball)),
              "poisoned restitution refuses");
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 0.0f;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    CHECK(!m3Shape_IsValid(world, NewSphereShape(world, body, &sd, &ball)), "zero density refuses");
    sd = m3DefaultShapeDef();
    sd.friction = -0.5f;
    CHECK(!m3Shape_IsValid(world, NewSphereShape(world, body, &sd, &ball)),
          "negative friction refuses");

    // Poisoned and degenerate geometry, family by family.
    sd = m3DefaultShapeDef();
    m3Sphere badBall = {{M3_TEST_NAN, 0.0f, 0.0f}, 0.5f};
    CHECK(!m3Shape_IsValid(world, NewSphereShape(world, body, &sd, &badBall)),
          "NaN center refuses");
    badBall = (m3Sphere){{0.0f, 0.0f, 0.0f}, M3_TEST_INF};
    CHECK(!m3Shape_IsValid(world, NewSphereShape(world, body, &sd, &badBall)),
          "infinite radius refuses");
    badBall = (m3Sphere){{0.0f, 0.0f, 0.0f}, -1.0f};
    CHECK(!m3Shape_IsValid(world, NewSphereShape(world, body, &sd, &badBall)),
          "negative radius refuses");

    m3Plane badPlane = {{0.0f, 0.0f, 0.0f}, 0.0f};
    CHECK(!m3Shape_IsValid(world, NewPlaneShape(world, ground, &sd, &badPlane)),
          "zero normal refuses");
    badPlane = (m3Plane){{M3_TEST_NAN, 1.0f, 0.0f}, 0.0f};
    CHECK(!m3Shape_IsValid(world, NewPlaneShape(world, ground, &sd, &badPlane)),
          "NaN normal refuses");
    badPlane = (m3Plane){{0.0f, 1.0f, 0.0f}, M3_TEST_INF};
    CHECK(!m3Shape_IsValid(world, NewPlaneShape(world, ground, &sd, &badPlane)),
          "infinite offset refuses");

    CHECK(!m3Shape_IsValid(world, NewBoxShape(world, body, &sd, (m3Vec3){M3_TEST_INF, 0.5f, 0.5f})),
          "infinite extent refuses");
    CHECK(!m3Shape_IsValid(world, NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.0f, 0.5f})),
          "flat box refuses");

    m3Capsule badCap = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.3f};
    CHECK(!m3Shape_IsValid(world, NewCapsuleShape(world, body, &sd, &badCap)),
          "coincident capsule points refuse");
    badCap = (m3Capsule){{0.0f, -0.3f, 0.0f}, {0.0f, M3_TEST_NAN, 0.0f}, 0.3f};
    CHECK(!m3Shape_IsValid(world, NewCapsuleShape(world, body, &sd, &badCap)),
          "NaN capsule point refuses");

    // Hull clouds: coincident, collinear, coplanar, poisoned. All
    // degenerate, all refused by QuickHull or the finite wall.
    m3Vec3 cloud[8];
    for (int32_t i = 0; i < 8; ++i)
    {
        cloud[i] = (m3Vec3){0.1f, 0.2f, 0.3f};
    }
    CHECK(!m3Shape_IsValid(world, NewHullShape(world, body, &sd, cloud, 8)),
          "a coincident cloud refuses");
    for (int32_t i = 0; i < 8; ++i)
    {
        cloud[i] = (m3Vec3){(m3real)i * 0.1f, 0.0f, 0.0f};
    }
    CHECK(!m3Shape_IsValid(world, NewHullShape(world, body, &sd, cloud, 8)),
          "a collinear cloud refuses");
    for (int32_t i = 0; i < 8; ++i)
    {
        cloud[i] = (m3Vec3){(m3real)(i % 4) * 0.2f, (m3real)(i / 4) * 0.2f, 0.0f};
    }
    CHECK(!m3Shape_IsValid(world, NewHullShape(world, body, &sd, cloud, 8)),
          "a coplanar cloud refuses");
    cloud[3] = (m3Vec3){0.1f, 0.1f, M3_TEST_NAN};
    CHECK(!m3Shape_IsValid(world, NewHullShape(world, body, &sd, cloud, 8)),
          "a poisoned cloud refuses");
    CHECK(!m3Shape_IsValid(world, NewHullShape(world, body, &sd, NULL, 8)), "a null cloud refuses");
    CHECK(!m3Shape_IsValid(world, NewHullShape(world, body, &sd, cloud, 3)), "three points refuse");

    // Meshes: poisoned vertex, empty, and the legal-but-ugly forms
    // that must be ACCEPTED (a needle triangle and inverted winding
    // are geometry, not corruption; backface culling handles them).
    m3Vec3 verts[4] = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, M3_TEST_NAN, 0.5f}};
    uint16_t tris[3] = {0, 2, 1};
    CHECK(!m3Shape_IsValid(world, NewMeshShape(world, ground, &sd, verts, 4, tris, 1)),
          "a poisoned mesh vertex refuses");
    CHECK(!m3Shape_IsValid(world, NewMeshShape(world, ground, &sd, verts, 3, tris, 0)),
          "an empty mesh refuses");
    m3Vec3 needleVerts[3] = {{0.0f, 0.0f, 0.0f}, {4.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 1.0e-6f}};
    CHECK(m3Shape_IsValid(world, NewMeshShape(world, ground, &sd, needleVerts, 3, tris, 1)),
          "a needle triangle is legal geometry");
    uint16_t inverted[3] = {0, 1, 2}; // winds the other way: legal
    m3Vec3 quadVerts[3] = {{2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 1.0f}};
    CHECK(m3Shape_IsValid(world, NewMeshShape(world, ground, &sd, quadVerts, 3, inverted, 1)),
          "inverted winding is the author's business");

    float heights[4] = {0.0f, M3_TEST_NAN, 0.0f, 0.0f};
    CHECK(!m3Shape_IsValid(world, NewHeightFieldShape(world, ground, &sd, heights, 2, 2, 1.0f)),
          "a poisoned height sample refuses");

    // The world took all this abuse standing up.
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) != 0, "the abused world still steps and hashes");
    m3DestroyWorld(world);
}

static void TestHostileJointDefsAndCommands(void)
{
    m3World* world = FreshWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = NewBody(world, &bd);
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId b = NewBody(world, &bd);

    m3JointDef jd = m3DefaultJointDef();
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.localAnchorA = (m3Vec3){M3_TEST_NAN, 0.0f, 0.0f};
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a poisoned anchor refuses");

    jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.enableLimit = true;
    jd.lowerLimit = 1.0f;
    jd.upperLimit = -1.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "inverted limits refuse");

    jd = m3DefaultJointDef();
    jd.type = m3_prismaticJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.enableMotor = true;
    jd.motorSpeed = M3_TEST_INF;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "an infinite motor refuses");
    jd.motorSpeed = 1.0f;
    jd.maxMotorEffort = -5.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "negative motor effort refuses");

    jd = m3DefaultJointDef();
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.enableCone = true;
    jd.coneAngle = -0.5f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a negative cone refuses");

    // Hostile commands are no-ops: the velocity before is the
    // velocity after, and the world never sees the poison.
    m3Body_SetLinearVelocity(world, a, (m3Vec3){1.0f, 2.0f, 3.0f});
    m3Body_SetLinearVelocity(world, a, (m3Vec3){M3_TEST_NAN, 0.0f, 0.0f});
    m3Vec3 v = m3Body_GetLinearVelocity(world, a);
    CHECK(v.x == 1.0f && v.y == 2.0f && v.z == 3.0f, "a NaN velocity command is a no-op");
    m3Body_SetAngularVelocity(world, a, (m3Vec3){0.0f, M3_TEST_INF, 0.0f});
    v = m3Body_GetAngularVelocity(world, a);
    CHECK(v.y == 0.0f, "an infinite spin command is a no-op");
    m3DestroyWorld(world);
}

// The mixer: 400 rounds of LCG-chosen hostile creates against twin
// worlds, interleaved with legal creates and steps. The twins must
// agree bit for bit at every checkpoint: the refusal paths are as
// deterministic as the happy paths.
static uint64_t RunGauntlet(void)
{
    m3World* world = FreshWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);

    for (int32_t round = 0; round < 400; ++round)
    {
        uint32_t roll = NextRand() % 8u;
        float poison = s_poisons[NextRand() % 3u];
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){(double)(NextRand() % 9u) - 4.0, 1.0 + (double)(round % 7), 0.0};
        if (roll == 0)
        {
            bd.position.y = (double)poison;
            NewBody(world, &bd); // refused
        }
        else if (roll == 1)
        {
            m3BodyId body = NewBody(world, &bd);
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, poison};
            NewSphereShape(world, body, &sd, &ball); // refused
            DestroyBody(world, body);
        }
        else if (roll == 2 && (NextRand() & 1u) != 0u)
        {
            m3BodyId body = NewBody(world, &bd);
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
            NewSphereShape(world, body, &sd, &ball); // legal: lives on
        }
        else if (roll == 3)
        {
            m3ShapeDef poisoned = m3DefaultShapeDef();
            poisoned.friction = poison;
            m3BodyId body = NewBody(world, &bd);
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
            NewSphereShape(world, body, &poisoned, &ball); // refused
            DestroyBody(world, body);
        }
        if (round % 40 == 0)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
    }
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

static void TestGauntletTwins(uint32_t seed)
{
    s_rng = seed;
    uint64_t first = RunGauntlet();
    s_rng = seed;
    uint64_t second = RunGauntlet();
    CHECK(first == second, "the hostile gauntlet is bit-deterministic across twins");
    CHECK(first != 0, "the gauntlet world lived");
}

// An optional argument adds that many extra gauntlet seeds beyond the
// canonical one; the weekly CI run passes a large number.
int main(int argc, char** argv)
{
    int32_t extraSeeds = argc > 1 ? atoi(argv[1]) : 0;
    InitPoisons();
    TestHostileWorldDefs();
    TestHostileBodyDefs();
    TestHostileShapeDefs();
    TestHostileJointDefsAndCommands();
    TestGauntletTwins(0xF00DFACEu);
    for (int32_t k = 0; k < extraSeeds; ++k)
    {
        TestGauntletTwins(0x9E3779B9u * (uint32_t)(k + 1));
    }
    if (s_failures == 0)
    {
        printf("test_hostile: all green\n");
        return 0;
    }
    printf("test_hostile: %d failure(s)\n", s_failures);
    return 1;
}
