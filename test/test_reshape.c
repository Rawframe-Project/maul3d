// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The reshape gate: sphere and capsule geometry swaps in
// place, conversions included. Interned families refuse, hostile
// dimensions refuse and journal nothing, a shrink drops what leaned
// on the old silhouette and wakes the sleepers, and mass follows the
// new volume.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static m3World* Yard(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.7f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    return world;
}

static void TestSwapWall(void)
{
    m3World* world = Yard();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId boxBody = NewBody(world, &bd);
    m3ShapeId box = NewBoxShape(world, boxBody, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    CHECK_REFUSES(world,
                  CHECK(m3Shape_SetSphere(world, box, &ball) == m3_errorInvalid,
                        "a hull target refuses the swap"),
                  "a hull target refuses the swap");
    bd.position = (m3Pos3){3.0, 3.0, 0.0};
    m3BodyId orbBody = NewBody(world, &bd);
    m3ShapeId orb = NewSphereShape(world, orbBody, &sd, &ball);
    uint64_t before = m3World_Hash(world);
    m3Sphere dead = {{0.0f, 0.0f, 0.0f}, 0.0f};
    CHECK_REFUSES(
        world,
        CHECK(m3Shape_SetSphere(world, orb, &dead) == m3_errorInvalid, "a zero radius refuses"),
        "a zero radius refuses");
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    m3Sphere cursed = {{0.0f, bad, 0.0f}, 0.5f};
    CHECK_REFUSES(
        world,
        CHECK(m3Shape_SetSphere(world, orb, &cursed) == m3_errorInvalid, "a NaN center refuses"),
        "a NaN center refuses");
    m3Capsule squashed = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.3f};
    CHECK_REFUSES(world,
                  CHECK(m3Shape_SetCapsule(world, orb, &squashed) == m3_errorInvalid,
                        "a zero-length capsule refuses"),
                  "a zero-length capsule refuses");
    CHECK(m3World_Hash(world) == before, "refused swaps changed nothing");
    m3ShapeId stale = {99, 7};
    CHECK_REFUSES(
        world, CHECK(m3Shape_SetSphere(world, stale, &ball) == m3_errorStale, "a stale id refuses"),
        "a stale id refuses");
    CHECK_ACCEPTS(world,
                  CHECK(m3Shape_SetSphere(world, orb, &ball) == m3_success,
                        "the same-geometry swap re-applies"),
                  "the same-geometry swap re-applies");
    m3DestroyWorld(world);
}

static void TestShrinkDropsTheStackAndWakesIt(void)
{
    m3World* world = Yard();
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.7f;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId pedestal = NewBody(world, &bd);
    m3Sphere big = {{0.0f, 0.0f, 0.0f}, 1.0f};
    m3ShapeId orb = NewSphereShape(world, pedestal, &sd, &big);
    bd.position = (m3Pos3){0.0, 2.45, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    NewBoxShape(world, crate, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double restingY = m3Body_GetPosition(world, crate).y;
    CHECK(restingY > 2.0, "the crate rests on the pedestal");
    CHECK(!m3Body_IsAwake(world, crate) || m3Body_IsAwake(world, crate),
          "read the state either way");
    // The pedestal deflates under it: the crate must wake and fall.
    m3Sphere small = {{0.0f, 0.0f, 0.0f}, 0.25f};
    CHECK_ACCEPTS(world,
                  CHECK(m3Shape_SetSphere(world, orb, &small) == m3_success, "the deflate applies"),
                  "the deflate applies");
    CHECK(m3Body_IsAwake(world, crate), "the sleeper above the shrink wakes");
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(world, crate).y < restingY - 0.8,
          "the crate fell off the deflated orb");
    m3DestroyWorld(world);
}

static void TestMassFollowsTheVolume(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId orb = NewSphereShape(world, body, &sd, &ball);
    m3Body_ApplyLinearImpulse(world, body, (m3Vec3){1.0f, 0.0f, 0.0f});
    float v1 = m3Body_GetLinearVelocity(world, body).x;
    m3Body_SetLinearVelocity(world, body, (m3Vec3){0.0f, 0.0f, 0.0f});
    m3Sphere doubled = {{0.0f, 0.0f, 0.0f}, 1.0f};
    CHECK_ACCEPTS(
        world, CHECK(m3Shape_SetSphere(world, orb, &doubled) == m3_success, "the doubling applies"),
        "the doubling applies");
    m3Body_ApplyLinearImpulse(world, body, (m3Vec3){1.0f, 0.0f, 0.0f});
    float v2 = m3Body_GetLinearVelocity(world, body).x;
    CHECK(v1 > 0.0f && v2 > 0.0f, "both impulses moved the body");
    float ratio = v1 / v2;
    CHECK(ratio > 7.5f && ratio < 8.5f, "doubling the radius folds mass by eight");
    // Conversion is legal both ways.
    m3Capsule pill = {{0.0f, -0.3f, 0.0f}, {0.0f, 0.3f, 0.0f}, 0.4f};
    CHECK_ACCEPTS(
        world,
        CHECK(m3Shape_SetCapsule(world, orb, &pill) == m3_success, "sphere to capsule converts"),
        "sphere to capsule converts");
    CHECK_ACCEPTS(world,
                  CHECK(m3Shape_SetSphere(world, orb, &ball) == m3_success,
                        "capsule back to sphere converts"),
                  "capsule back to sphere converts");
    m3DestroyWorld(world);
}

int main(void)
{
    TestSwapWall();
    TestShrinkDropsTheStackAndWakesIt();
    TestMassFollowsTheVolume();
    if (s_failures == 0)
    {
        printf("test_reshape: all passed\n");
        return 0;
    }
    return 1;
}
