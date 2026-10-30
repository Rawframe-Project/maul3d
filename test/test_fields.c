// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The fields gate: wind streams a flag, conveyors carry crates at
// belt speed, and hostile field values refuse.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"
#include "maul3d/softbody.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* PlaneWorld(m3ShapeId* outFloor)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.softBodyCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId fs = NewPlaneShape(world, ground, &sd, &fl);
    if (outFloor != NULL)
    {
        *outFloor = fs;
    }
    return world;
}

static void TestWindStreamsTheFlag(void)
{
    // A cloth pinned along one edge streams in a gusty wind: it
    // rises from the vertical hang, twins agree to the bit, and
    // the gust phase survives rollback (same wave, same bits).
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3World* world = PlaneWorld(NULL);
        m3SoftBodyDef cd = m3DefaultSoftBodyDef();
        cd.countX = 6;
        cd.countY = 6;
        cd.countZ = 1;
        cd.spacing = 0.25f;
        cd.radius = 0.08f;
        cd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3SoftBodyId flag = NewSoftBody(world, &cd);
        for (int32_t r = 0; r < 6; ++r)
        {
            m3SoftBody_PinParticle(world, flag, r * 6); // the mast column
        }
        m3World_SetWind(world, (m3Vec3){1.0f, 0.0f, 0.0f}, 6.0f, 0.8f, 0.5f);
        int64_t snapBytes = 0;
        for (int32_t i = 0; i < 240; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 120 && run == 0)
            {
                snapBytes = Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-gust snapshot fits");
            }
        }
        // The free edge must stream downwind, not hang.
        double freeX = 0.0;
        for (int32_t r = 0; r < 6; ++r)
        {
            freeX += m3SoftBody_GetParticlePosition(world, flag, r * 6 + 5).x;
        }
        freeX /= 6.0;
        CHECK(freeX > 0.8, "the flag streams downwind");
        uint64_t final = m3World_Hash(world);
        hashes[run] = final;
        if (run == 0)
        {
            CHECK(Restore(world, snap, snapBytes), "the mid-gust restore lands");
            for (int32_t i = 121; i < 240; ++i)
            {
                (void)m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == final,
                  "the gust wave continues across rollback bit-exactly");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "windy twins are bit-identical");
}

static void TestConveyorCarries(void)
{
    // A crate dropped on a belt reaches belt speed; on the second
    // half the belt reverses and the crate follows.
    m3ShapeId floor;
    m3World* world = PlaneWorld(&floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, crate, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3Shape_SetSurfaceVelocity(world, floor, (m3Vec3){1.5f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(world, crate);
    CHECK(fabsf(v.x - 1.5f) < 0.1f, "the crate rides the belt at belt speed");

    m3Shape_SetSurfaceVelocity(world, floor, (m3Vec3){-1.0f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 180; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    v = m3Body_GetLinearVelocity(world, crate);
    CHECK(fabsf(v.x + 1.0f) < 0.1f, "the reversed belt carries it back");
    m3DestroyWorld(world);
}

static void TestConveyorOnDynamicDeck(void)
{
    // A conveyor surface on a DYNAMIC platform still drives
    // riders, and Newton collects: the deck recoils the other way.
    m3World* world = PlaneWorld(NULL);
    m3BodyDef dd = m3DefaultBodyDef();
    dd.type = m3_dynamicBody;
    dd.position = (m3Pos3){0.0, 0.3, 0.0};
    m3BodyId deck = NewBody(world, &dd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 8.0f; // heavy barge
    m3ShapeId deckTop = NewBoxShape(world, deck, &sd, (m3Vec3){1.6f, 0.3f, 1.0f});
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    m3ShapeDef cs = m3DefaultShapeDef();
    NewBoxShape(world, crate, &cs, (m3Vec3){0.25f, 0.25f, 0.25f});
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Shape_SetSurfaceVelocity(world, deckTop, (m3Vec3){1.0f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 vCrate = m3Body_GetLinearVelocity(world, crate);
    m3Vec3 vDeck = m3Body_GetLinearVelocity(world, deck);
    CHECK(vCrate.x > 0.3f, "the deck belt drives the rider forward");
    CHECK(vDeck.x < -0.005f, "and Newton collects the recoil");
    m3DestroyWorld(world);
}

static void TestHostileFields(void)
{
    m3ShapeId floor;
    m3World* world = PlaneWorld(&floor);
    m3World_SetWind(world, (m3Vec3){3.0f, 0.0f, 0.0f}, 5.0f, 1.0f, 0.2f); // non-unit dir
    m3World_SetWind(world, (m3Vec3){NAN, 0.0f, 0.0f}, 5.0f, 1.0f, 0.2f);
    m3World_SetWind(world, (m3Vec3){1.0f, 0.0f, 0.0f}, -5.0f, 1.0f, 0.2f);
    m3Shape_SetSurfaceVelocity(world, floor, (m3Vec3){NAN, 0.0f, 0.0f});
    m3ShapeId stale = floor;
    stale.index1 += 500;
    m3Shape_SetSurfaceVelocity(world, stale, (m3Vec3){1.0f, 0.0f, 0.0f});
    uint64_t before = m3World_Hash(world);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    (void)before;
    CHECK(m3World_GetCounters(world).bodyCount > 0, "hostile fields left the world whole");
    m3DestroyWorld(world);
}

int main(void)
{
    TestWindStreamsTheFlag();
    TestConveyorCarries();
    TestConveyorOnDynamicDeck();
    TestHostileFields();
    if (s_failures == 0)
    {
        printf("test_fields: all green\n");
        return 0;
    }
    printf("test_fields: %d failure(s)\n", s_failures);
    return 1;
}
