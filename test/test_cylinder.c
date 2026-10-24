// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The cylinder checks: the factory builds a faceted prism
// through the hull path. Degenerate defs refuse, the quality knob
// clamps, the prism's mass behaves like the round closed form
// within the faceting error, and a spun cylinder rolls where a cube
// would skid.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static m3World* FlatWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.8f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    return world;
}

static void TestFactoryWall(void)
{
    m3World* world = FlatWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Cylinder keg = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.4f};
    m3ShapeId good = NewCylinderShape(world, body, &sd, &keg, 24);
    CHECK(m3Shape_IsValid(world, good), "a sane cylinder creates");
    m3Cylinder flat = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.4f};
    CHECK(!m3Shape_IsValid(world, NewCylinderShape(world, body, &sd, &flat, 24)),
          "a flat disc refuses");
    m3Cylinder thin = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.0f};
    CHECK(!m3Shape_IsValid(world, NewCylinderShape(world, body, &sd, &thin, 24)),
          "a zero radius refuses");
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    m3Cylinder nan = {{0.0f, -0.5f, 0.0f}, {0.0f, bad, 0.0f}, 0.4f};
    CHECK(!m3Shape_IsValid(world, NewCylinderShape(world, body, &sd, &nan, 24)),
          "a NaN cap refuses");
    CHECK(!m3Shape_IsValid(world, NewCylinderShape(world, body, &sd, NULL, 24)),
          "a null def refuses");
    // The quality knob clamps instead of refusing: both extremes
    // still mint valid prisms.
    bd.position = (m3Pos3){5.0, 2.0, 0.0};
    m3BodyId other = NewBody(world, &bd);
    CHECK(m3Shape_IsValid(world, NewCylinderShape(world, other, &sd, &keg, 1)),
          "segments clamp up");
    bd.position = (m3Pos3){-5.0, 2.0, 0.0};
    m3BodyId third = NewBody(world, &bd);
    CHECK(m3Shape_IsValid(world, NewCylinderShape(world, third, &sd, &keg, 999)),
          "segments clamp down");
    m3DestroyWorld(world);
}

static void TestMassBehavesLikeTheClosedForm(void)
{
    // No gravity, no ground contact: impulse over delta v is mass.
    // A 32-segment prism sits 0.64 percent under the round area;
    // three percent of slack covers it with room to spare.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 3.0f;
    m3Cylinder keg = {{0.0f, -0.7f, 0.0f}, {0.0f, 0.7f, 0.0f}, 0.5f};
    CHECK(m3Shape_IsValid(world, NewCylinderShape(world, body, &sd, &keg, 32)), "the keg creates");
    m3Body_ApplyLinearImpulse(world, body, (m3Vec3){10.0f, 0.0f, 0.0f});
    float dv = m3Body_GetLinearVelocity(world, body).x;
    CHECK(dv > 0.0f, "the impulse moves the keg");
    float measured = 10.0f / dv;
    float round = 3.0f * 3.14159265f * 0.5f * 0.5f * 1.4f;
    CHECK(fabsf(measured - round) < 0.03f * round, "the prism mass tracks the closed form");
    m3DestroyWorld(world);
}

static void TestRollsWhereACubeSkids(void)
{
    // Same spin, same ground: the cylinder converts it into travel,
    // the cube bleeds it into friction chatter and stays close.
    m3World* world = FlatWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.8f;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.45, 0.0};
    bd.angularVelocity = (m3Vec3){0.0f, 0.0f, -8.0f};
    m3BodyId roller = NewBody(world, &bd);
    m3Cylinder keg = {{0.0f, 0.0f, -0.5f}, {0.0f, 0.0f, 0.5f}, 0.45f};
    NewCylinderShape(world, roller, &sd, &keg, 24);
    bd.position = (m3Pos3){0.0, 0.45, 4.0};
    m3BodyId cube = NewBody(world, &bd);
    NewBoxShape(world, cube, &sd, (m3Vec3){0.45f, 0.45f, 0.45f});
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double rolled = m3Body_GetPosition(world, roller).x;
    double skidded = m3Body_GetPosition(world, cube).x;
    CHECK(rolled > 1.0, "the spun cylinder rolls away");
    CHECK(rolled > skidded + 0.5, "rolling beats skidding");
    m3DestroyWorld(world);
}

int main(void)
{
    TestFactoryWall();
    TestMassBehavesLikeTheClosedForm();
    TestRollsWhereACubeSkids();
    if (s_failures == 0)
    {
        printf("test_cylinder: all passed\n");
        return 0;
    }
    return 1;
}
