// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The C snippets of docs/user/, built and run as written. Each test sets
// up what its snippet assumes, runs the snippet unchanged, and checks the
// outcome the page describes. tools/check_guide.py fails when a snippet on
// a page no longer appears here.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/maul3d.h"

#include <stdio.h>

// A world with a floor plane at y = -5 and a light crate dropped above
// it at the origin.
static m3World* FloorWorld(m3BodyId* crateOut)
{
    m3WorldDef worldDef = m3DefaultWorldDef();
    worldDef.bodyCapacity = 8;
    worldDef.shapeCapacity = 8;
    m3World* world = NewWorld(&worldDef);
    m3BodyDef groundDef = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &groundDef);
    m3ShapeDef shapeDef = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, -5.0f};
    NewPlaneShape(world, ground, &shapeDef, &floor);
    m3BodyDef crateDef = m3DefaultBodyDef();
    crateDef.type = m3_dynamicBody;
    crateDef.position = (m3Pos3){0.0, 2.0, 0.0};
    *crateOut = NewBody(world, &crateDef);
    m3ShapeDef crateShape = m3DefaultShapeDef();
    crateShape.density = 400.0f; // lighter than water
    NewBoxShape(world, *crateOut, &crateShape, (m3Vec3){0.3f, 0.3f, 0.3f});
    return world;
}

// water.md: a pool with a current. The crate falls in, bobs, floats at
// the surface and drifts along x; after five seconds it is still in the
// pool, whose far edge is at x = 10.
static void TestWaterPool(void)
{
    m3BodyId crate;
    m3World* world = FloorWorld(&crate);

    m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
    def.bounds.lowerBound = (m3Pos3){-10.0, -5.0, -10.0};
    def.bounds.upperBound = (m3Pos3){10.0, 0.0, 10.0}; // the surface is y = 0
    def.flow = (m3Vec3){1.5f, 0.0f, 0.0f};             // a current along x
    m3WaterVolumeId pool;
    m3Result result = m3CreateWaterVolume(world, &def, &pool);

    CHECK(result == m3_success && m3WaterVolume_IsValid(world, pool), "the pool fills");
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(world, crate);
    CHECK(p.y > -0.3 && p.y < 0.8, "the light crate floats at the surface");
    CHECK(p.x > 3.0 && p.x < 9.0, "the current carries it along x");
    m3DestroyWorld(world);
}

int main(void)
{
    TestWaterPool();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
