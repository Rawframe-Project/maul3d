// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The stance gate: crouch and resize with the stand-up veto.
// A character ducks under a slab it cannot walk under, is REFUSED
// the stand while the slab presses, stands the moment it clears the
// edge, lands shorter after a mid-air crouch, and cannot widen inside
// a slot. Hostile dimensions bounce.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/character.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Floor plane plus a slab roof over x in [2, 6]: 1.3 meters of
// clearance. The default character stands 1.8 tall and crouches
// to 1.2: the tunnel admits exactly one of those.
static m3World* TunnelWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.characterCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){4.0, 1.55, 0.0};
    m3BodyId roof = NewBody(world, &rd);
    NewBoxShape(world, roof, &sd, (m3Vec3){2.0f, 0.25f, 2.0f});
    return world;
}

// The tunnel run as a pure function of the tick index, so twins,
// replays, and rollback re-runs all issue the identical schedule.
static void TunnelTick(m3World* world, m3CharacterId hero, int32_t i)
{
    if (i < 60)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f}); // land
        return;
    }
    if (i < 120)
    {
        // Standing walk: jams against the slab edge and stays there.
        m3Character_Move(world, hero, (m3Vec3){0.05f, -0.1f, 0.0f});
        return;
    }
    if (i == 120)
    {
        SetStance(world, hero, 0.2f, 0.4f); // duck
    }
    if (i == 240)
    {
        SetStance(world, hero, 0.5f, 0.4f); // pressed: refused
    }
    if (i == 460)
    {
        SetStance(world, hero, 0.5f, 0.4f); // clear: stands
    }
    m3Character_Move(world, hero, (m3Vec3){0.03f, -0.1f, 0.0f}); // crawl east
}

static void TestDuckWalkStand(void)
{
    // The full arc, plus the journal and the twin in one pass.
    static uint8_t journal[131072];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3World* world = TunnelWorld();
        bool recording = run == 0 && StartJournal(world, journal, (int32_t)sizeof(journal));
        m3CharacterDef cd = m3DefaultCharacterDef();
        cd.position = (m3Pos3){0.0, 3.0, 0.0};
        m3CharacterId hero = NewCharacter(world, &cd);
        for (int32_t i = 0; i < 520; ++i)
        {
            TunnelTick(world, hero, i);
            if (run == 0)
            {
                if (i == 119)
                {
                    // Standing 1.8 against 1.3 of clearance: the
                    // slab edge is a wall long before the tunnel
                    // (sixty walk ticks would have covered 3 meters
                    // on open ground).
                    double blockedX = m3Character_GetPosition(world, hero).x;
                    CHECK(blockedX > 1.3 && blockedX < 1.9, "the standing character cannot enter");
                }
                if (i == 121)
                {
                    float hh;
                    float r;
                    m3Character_GetStance(world, hero, &hh, &r);
                    CHECK(hh == 0.2f && r == 0.4f, "the duck applies");
                    CHECK(m3Character_IsGrounded(world, hero), "the crouch keeps its feet");
                }
                if (i == 241)
                {
                    float hh;
                    float r;
                    m3Character_GetStance(world, hero, &hh, &r);
                    CHECK(hh == 0.2f, "the pressed stand is REFUSED, stance holds");
                    double x = m3Character_GetPosition(world, hero).x;
                    CHECK(x > 2.2 && x < 6.0, "the refusal happened inside the tunnel");
                }
                if (i == 461)
                {
                    float hh;
                    float r;
                    m3Character_GetStance(world, hero, &hh, &r);
                    CHECK(hh == 0.5f, "past the edge the stand applies");
                    CHECK(m3Character_GetPosition(world, hero).x > 6.5, "genuinely past the slab");
                }
            }
        }
        double standY = m3Character_GetPosition(world, hero).y;
        CHECK(standY > 0.89 && standY < 0.95, "the stood character is tall again");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int64_t bytes = StopJournal(world);
            CHECK(bytes > 0, "the tunnel run records");
            m3World* replayed = TunnelWorld();
            CHECK(ReplayJournal(replayed, journal, bytes), "the tunnel run replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replayed run is bit-identical");
            // The refused stand at tick 240 journaled NOTHING: the
            // replay just proved it by landing (a phantom op 64
            // would have re-applied and diverged or failed).
            m3DestroyWorld(replayed);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin tunnel runs are bit-identical");
}

static void TestMidAirCrouchLandsShorter(void)
{
    // Feet anchoring is the law even in the air: the crouch pulls
    // the center down, the capsule is shorter, and the landing
    // height is the crouched skin gap, not the standing one.
    m3World* world = TunnelWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){-3.0, 4.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 20; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.08f, 0.0f});
    }
    CHECK(!m3Character_IsGrounded(world, hero), "still airborne at the crouch");
    CHECK(SetStance(world, hero, 0.2f, 0.4f), "the mid-air crouch applies");
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.08f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(world, hero), "the crouched faller lands");
    double y = m3Character_GetPosition(world, hero).y;
    CHECK(y > 0.59 && y < 0.65, "the landing height is the crouched one");
    m3DestroyWorld(world);
}

static void TestSlotRefusesWidth(void)
{
    // Two walls 0.9 apart: the 0.8-wide character fits, a 1.2-wide
    // one cannot, and the veto sees the walls exactly like it sees
    // ceilings (the fraction-zero overlap report).
    m3World* world = TunnelWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){-4.0, 1.0, 0.65};
    m3BodyId wallA = NewBody(world, &wd);
    NewBoxShape(world, wallA, &sd, (m3Vec3){1.0f, 1.0f, 0.2f});
    wd.position = (m3Pos3){-4.0, 1.0, -0.65};
    m3BodyId wallB = NewBody(world, &wd);
    NewBoxShape(world, wallB, &sd, (m3Vec3){1.0f, 1.0f, 0.2f});
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){-4.0, 3.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(world, hero), "the character lands inside the slot");
    CHECK(!SetStance(world, hero, 0.5f, 0.6f), "widening inside the slot refuses");
    float hh;
    float r;
    m3Character_GetStance(world, hero, &hh, &r);
    CHECK(r == 0.4f, "the refused widening left the radius alone");
    CHECK(SetStance(world, hero, 0.5f, 0.3f), "narrowing inside the slot applies");
    m3DestroyWorld(world);
}

static void TestHostileWall(void)
{
    m3World* world = TunnelWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    CHECK(!SetStance(world, hero, bad, 0.4f), "a NaN height bounces");
    CHECK(!SetStance(world, hero, 0.5f, -0.4f), "a negative radius bounces");
    CHECK(!SetStance(world, hero, 0.01f, 0.4f), "a sub-minimum height bounces");
    CHECK(!SetStance(world, hero, 0.5f, 40.0f), "an absurd radius bounces");
    float hh;
    float r;
    m3Character_GetStance(world, hero, &hh, &r);
    CHECK(hh == 0.5f && r == 0.4f, "four hostile calls changed nothing");
    m3CharacterId stale = {99, 7};
    CHECK(!SetStance(world, stale, 0.5f, 0.4f), "a stale id bounces");
    m3Character_GetStance(world, stale, &hh, &r);
    CHECK(hh == 0.0f && r == 0.0f, "a stale stance reads zeros");
    CHECK(SetStance(world, hero, 0.5f, 0.4f), "the same-dims stance re-applies");
    m3DestroyWorld(world);
}

int main(void)
{
    TestDuckWalkStand();
    TestMidAirCrouchLandsShorter();
    TestSlotRefusesWidth();
    TestHostileWall();
    if (s_failures == 0)
    {
        printf("test_stance: all passed\n");
        return 0;
    }
    return 1;
}
