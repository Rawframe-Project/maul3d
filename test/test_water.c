// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The water gate: a world-anchored box of water floats what
// should float, sinks what should sink, carries what drifts into
// its current, and wakes sleepers when the tide arrives or leaves.
// No fluid sim BY LAW.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static m3World* Pool(m3WaterVolumeId* outWater, float flowX)
{
    // A floor at y = 0 and a 4-meter-deep basin of water above it.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 16;
    wd.shapeCapacity = 16;
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    m3WaterVolumeDef wdf = m3DefaultWaterVolumeDef();
    wdf.lo = (m3Pos3){-10.0, 0.0, -10.0};
    wdf.hi = (m3Pos3){10.0, 4.0, 10.0};
    wdf.flow = (m3Vec3){flowX, 0.0f, 0.0f};
    m3WaterVolumeId water = NewWaterVolume(world, &wdf);
    if (outWater != NULL)
    {
        *outWater = water;
    }
    return world;
}

static m3BodyId DropCrate(m3World* world, double x, double y, float density)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, y, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = density;
    NewBoxShape(world, crate, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    return crate;
}

static void TestFloatAndSink(void)
{
    m3WaterVolumeId water;
    m3World* world = Pool(&water, 0.0f);
    CHECK(m3WaterVolume_IsValid(world, water), "the basin fills");
    m3BodyId cork = DropCrate(world, -2.0, 6.0, 400.0f);  // lighter than water
    m3BodyId anvil = DropCrate(world, 2.0, 6.0, 3000.0f); // heavier than water
    for (int32_t i = 0; i < 420; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double corkY = m3Body_GetPosition(world, cork).y;
    double anvilY = m3Body_GetPosition(world, anvil).y;
    CHECK(corkY > 2.5 && corkY < 4.6, "the light crate floats near the surface");
    CHECK(anvilY < 0.8, "the dense crate sinks to the floor");
    m3DestroyWorld(world);
}

static void TestFlowCarries(void)
{
    m3World* still = Pool(NULL, 0.0f);
    m3World* river = Pool(NULL, 1.5f);
    m3BodyId a = DropCrate(still, 0.0, 3.5, 400.0f);
    m3BodyId b = DropCrate(river, 0.0, 3.5, 400.0f);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(still, 1.0f / 60.0f, 4);
        m3World_Step(river, 1.0f / 60.0f, 4);
    }
    double ax = m3Body_GetPosition(still, a).x;
    double bx = m3Body_GetPosition(river, b).x;
    CHECK(fabs(ax) < 0.2, "still water leaves the floater in place");
    CHECK(bx > 2.0, "the current carries the floater downstream");
    m3DestroyWorld(still);
    m3DestroyWorld(river);
}

static void TestTideWakesSleepers(void)
{
    // A crate falls asleep on the dry floor; the tide comes in and
    // it wakes and floats off. Then the tide goes out under a
    // floater and it wakes to fall.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 16;
    wd.shapeCapacity = 16;
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    m3BodyId crate = DropCrate(world, 0.0, 0.6, 300.0f);
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m3Body_IsAwake(world, crate), "the crate sleeps on the dry floor");
    m3WaterVolumeDef wdf = m3DefaultWaterVolumeDef();
    wdf.lo = (m3Pos3){-5.0, 0.0, -5.0};
    wdf.hi = (m3Pos3){5.0, 3.0, 5.0};
    m3WaterVolumeId tide = NewWaterVolume(world, &wdf);
    CHECK(m3Body_IsAwake(world, crate), "the incoming tide wakes the sleeper");
    for (int32_t i = 0; i < 420; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(world, crate).y > 1.5, "the woken crate floats up");
    DestroyWaterVolume(world, tide);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(world, crate).y < 0.8, "the ebb drops the floater back");
    m3DestroyWorld(world);
}

static void TestWaterWalls(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    uint64_t before = m3World_Hash(world);
    m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
    def.hi = def.lo; // empty box
    CHECK(!m3WaterVolume_IsValid(world, NewWaterVolume(world, &def)), "an empty box refuses");
    def = m3DefaultWaterVolumeDef();
    def.density = -5.0f;
    CHECK(!m3WaterVolume_IsValid(world, NewWaterVolume(world, &def)), "negative density refuses");
    def = m3DefaultWaterVolumeDef();
    def.lo.x = (double)NAN;
    CHECK(!m3WaterVolume_IsValid(world, NewWaterVolume(world, &def)), "a NaN corner refuses");
    def = m3DefaultWaterVolumeDef();
    def.linearDrag = -1.0f;
    CHECK(!m3WaterVolume_IsValid(world, NewWaterVolume(world, &def)), "negative drag refuses");
    CHECK(m3World_Hash(world) == before, "refused water moved no bits");
    // The ninth volume refuses loudly.
    m3WaterVolumeId ids[8];
    def = m3DefaultWaterVolumeDef();
    for (int32_t k = 0; k < 8; ++k)
    {
        def.lo = (m3Pos3){(double)k * 3.0, 0.0, 0.0};
        def.hi = (m3Pos3){(double)k * 3.0 + 2.0, 2.0, 2.0};
        ids[k] = NewWaterVolume(world, &def);
        CHECK(m3WaterVolume_IsValid(world, ids[k]), "eight volumes fit");
    }
    def.lo = (m3Pos3){30.0, 0.0, 0.0};
    def.hi = (m3Pos3){32.0, 2.0, 2.0};
    CHECK(!m3WaterVolume_IsValid(world, NewWaterVolume(world, &def)), "the ninth refuses");
    DestroyWaterVolume(world, ids[3]);
    CHECK(!m3WaterVolume_IsValid(world, ids[3]), "a destroyed volume goes stale");
    DestroyWaterVolume(world, ids[3]); // stale destroy: quiet no-op
    m3DestroyWorld(world);
}

static void TestSoftBuoyancy(void)
{
    // The rho-1000 particle convention: in a density-1500
    // basin a cloth hangs high in the water column while its dry
    // twin crumples to the floor; a current drags the wet one
    // downstream.
    double wetY = 0.0;
    double dryY = 0.0;
    double wetX = 0.0;
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        m3World* world = NewWorld(&wd);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &sd, &floor);
        if (pass == 1)
        {
            m3WaterVolumeDef wdf = m3DefaultWaterVolumeDef();
            wdf.lo = (m3Pos3){-8.0, 0.0, -8.0};
            wdf.hi = (m3Pos3){8.0, 4.0, 8.0};
            wdf.density = 1500.0f;
            wdf.flow = (m3Vec3){0.8f, 0.0f, 0.0f};
            NewWaterVolume(world, &wdf);
        }
        m3SoftBodyDef sb = m3DefaultSoftBodyDef();
        sb.position = (m3Pos3){0.0, 3.0, 0.0};
        sb.countX = 4;
        sb.countY = 1;
        sb.countZ = 4;
        sb.spacing = 0.25f;
        m3SoftBodyId cloth = NewSoftBody(world, &sb);
        CHECK(m3SoftBody_IsValid(world, cloth), "the cloth creates");
        for (int32_t i = 0; i < 300; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Pos3 c = m3SoftBody_GetParticlePosition(world, cloth, 5);
        if (pass == 0)
        {
            dryY = c.y;
        }
        else
        {
            wetY = c.y;
            wetX = c.x;
        }
        m3DestroyWorld(world);
    }
    CHECK(dryY < 0.5, "the dry cloth crumples to the floor");
    CHECK(wetY > dryY + 0.8, "the wet cloth hangs high in the column");
    CHECK(wetX > 1.0, "the current drags the wet cloth downstream");
}

int main(void)
{
    TestFloatAndSink();
    TestFlowCarries();
    TestTideWakesSleepers();
    TestWaterWalls();
    TestSoftBuoyancy();
    if (s_failures == 0)
    {
        printf("test_water: all passed\n");
        return 0;
    }
    printf("test_water: %d FAILURES\n", s_failures);
    return 1;
}
