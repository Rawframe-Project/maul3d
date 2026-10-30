// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The filter gate: categories, masks, and groups decide who
// touches whom, queries carry their own filter, and sensors and the
// continuous phase obey the same rule.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    CAT_GROUND = 1u << 0,
    CAT_DEBRIS = 1u << 1,
    CAT_PLAYER = 1u << 2,
    CAT_GHOST = 1u << 3
};

static m3World* GroundWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.categoryBits = CAT_GROUND;
    sd.maskBits = ~0ull;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    return world;
}

static m3BodyId DropBall(m3World* world, double x, uint64_t category, uint64_t mask)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, 2.0, 0.0};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.categoryBits = category;
    sd.maskBits = mask;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, body, &sd, &ball);
    return body;
}

static void TestMaskGate(void)
{
    // A ghost whose mask excludes the ground falls straight
    // through; a debris ball with a full mask lands. The gate is
    // symmetric: BOTH directions must agree, so a one-sided mask
    // still separates the pair.
    m3World* world = GroundWorld();
    m3BodyId solid = DropBall(world, -2.0, CAT_DEBRIS, ~0ull);
    m3BodyId ghost = DropBall(world, 2.0, CAT_GHOST, CAT_DEBRIS); // sees debris only
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(world, solid).y > 0.4, "the full-mask ball lands on the plane");
    CHECK(m3Body_GetPosition(world, ghost).y < -3.0, "the masked ghost falls through the world");
    m3DestroyWorld(world);
}

static void TestGroupOverride(void)
{
    // Two debris balls whose masks EXCLUDE each other but share a
    // positive group: the group forces the collision. Then the
    // same pair with a negative group: never, even with full
    // masks.
    m3World* world = GroundWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId a = NewBody(world, &bd);
    m3ShapeDef sa = m3DefaultShapeDef();
    sa.categoryBits = CAT_DEBRIS;
    sa.maskBits = CAT_GROUND; // does NOT see debris
    sa.groupIndex = 7;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, a, &sa, &ball);
    bd.position = (m3Pos3){0.4, 2.0, 0.0}; // will fall onto a
    m3BodyId b = NewBody(world, &bd);
    NewSphereShape(world, b, &sa, &ball);
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // The positive group forced the pair: b rests on or beside a,
    // not inside it.
    m3Pos3 pa = m3Body_GetPosition(world, a);
    m3Pos3 pb = m3Body_GetPosition(world, b);
    double dx = pb.x - pa.x;
    double dy = pb.y - pa.y;
    double dz = pb.z - pa.z;
    CHECK(dx * dx + dy * dy + dz * dz > 0.9 * 0.9, "a positive group forces the collision");

    // Negative group: full masks, never collide.
    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){5.0, 0.5, 0.0};
    m3BodyId c = NewBody(world, &cd);
    m3ShapeDef sc = m3DefaultShapeDef();
    sc.groupIndex = -3;
    NewSphereShape(world, c, &sc, &ball);
    cd.position = (m3Pos3){5.0, 1.49, 0.0}; // starts exactly on top
    m3BodyId d = NewBody(world, &cd);
    NewSphereShape(world, d, &sc, &ball);
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 pd = m3Body_GetPosition(world, d);
    CHECK(pd.y < 1.1 && pd.y > 0.4, "a negative group never collides: d sinks into c's space");
    m3DestroyWorld(world);
}

static void TestQueryFilters(void)
{
    // The ray sees what its filter admits: a debris-only filter
    // skips the player capsule standing in the beam, and the
    // filtered sphere cast agrees. All-hits respects it too.
    m3World* world = GroundWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId player = NewBody(world, &bd);
    m3ShapeDef sp = m3DefaultShapeDef();
    sp.categoryBits = CAT_PLAYER;
    m3Capsule cap = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.4f};
    NewCapsuleShape(world, player, &sp, &cap);
    bd.position = (m3Pos3){0.0, 1.0, 3.0};
    m3BodyId crate = NewBody(world, &bd);
    m3ShapeDef sc = m3DefaultShapeDef();
    sc.categoryBits = CAT_DEBRIS;
    NewBoxShape(world, crate, &sc, (m3Vec3){0.5f, 0.5f, 0.5f});

    m3Pos3 origin = {0.0, 1.0, -3.0};
    m3Vec3 dir = {0.0f, 0.0f, 8.0f};
    m3CastHit any = {0};
    bool anyHit = m3World_CastRayClosest(world, origin, dir, m3DefaultQueryFilter(), &any);
    CHECK(anyHit && any.point.z < 1.0, "the unfiltered ray stops at the player");
    m3QueryFilter debrisOnly = m3DefaultQueryFilter();
    debrisOnly.maskBits = CAT_DEBRIS;
    m3CastHit through = {0};
    bool throughHit = m3World_CastRayClosest(world, origin, dir, debrisOnly, &through);
    CHECK(throughHit && through.point.z > 2.0, "the filtered ray skips the player");

    m3CastHit cast = {0};
    bool castHit = m3World_CastSphereClosest(world, origin, 0.2f, dir, debrisOnly, &cast);
    CHECK(castHit && cast.point.z > 2.0, "the filtered sphere cast agrees");

    m3CastHit hits[8];
    int32_t n = m3World_CastRayAll(world, origin, dir, debrisOnly, hits, 8);
    for (int32_t k = 0; k < n; ++k)
    {
        CHECK(hits[k].shapeId.index1 != 0, "all-hits entries are real");
        CHECK(hits[k].point.z > 2.0, "all-hits skips the player too");
    }
    m3DestroyWorld(world);
}

static void TestRuntimeFilterChange(void)
{
    // A ball that has come to rest and fallen asleep turns ghost at
    // runtime: it wakes and falls through. The session, recorded from
    // creation and replayed into a fresh world, lands on the same bits.
    static uint8_t tape[262144];
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m3World* world = NewWorld(&def);
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "recording starts");
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sg = m3DefaultShapeDef();
    sg.categoryBits = CAT_GROUND;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sg, &floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.categoryBits = CAT_DEBRIS;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId shape = NewSphereShape(world, ball, &sd, &sphere);
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m3Body_IsAwake(world, ball), "the ball rests and sleeps");
    m3Shape_SetFilter(world, shape, CAT_GHOST, CAT_DEBRIS, 0);
    uint64_t category = 0;
    uint64_t mask = 0;
    int32_t group = -1;
    m3Shape_GetFilter(world, shape, &category, &mask, &group);
    CHECK(category == CAT_GHOST && mask == CAT_DEBRIS && group == 0, "the filter reads back");
    CHECK(m3Body_IsAwake(world, ball), "the change wakes the sleeper");
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(world, ball).y < -3.0, "the ghost falls through the ground");
    int64_t bytes = StopJournal(world);
    m3World* twin = NewWorld(&def);
    CHECK(ReplayJournal(twin, tape, bytes), "the session replays");
    CHECK(m3World_Hash(twin) == m3World_Hash(world), "the replayed filter change lands the same");
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

int main(void)
{
    TestMaskGate();
    TestGroupOverride();
    TestQueryFilters();
    TestRuntimeFilterChange();
    if (s_failures == 0)
    {
        printf("test_filter: all green\n");
        return 0;
    }
    printf("test_filter: %d failure(s)\n", s_failures);
    return 1;
}
