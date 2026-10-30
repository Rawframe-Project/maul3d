// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mover core gate: a kinematic capsule walks the world by
// collide-and-slide. Flat ground carries it at exact height, walls
// strip the blocked component and keep the rest, ceilings stop it,
// ledges drop it, and hostile moves leave the world finite.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/character.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* ArenaWorld(void)
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
    return world;
}

// The standing height for the default character: capsule bottom
// (center - halfHeight - radius) rides one skin above the floor.
static double StandingY(const m3CharacterDef* def)
{
    return (double)(def->halfHeight + def->radius + def->skin);
}

static void TestFlatWalk(void)
{
    m3World* world = ArenaWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    CHECK(m3Character_IsValid(world, hero), "the character creates");
    CHECK(!m3Character_IsGrounded(world, hero), "spawned in the air, honestly airborne");

    // Fall to the floor under host gravity, then walk.
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(world, hero), "the character lands");
    m3Pos3 p = m3Character_GetPosition(world, hero);
    double stand = StandingY(&cd);
    CHECK(p.y > stand - 0.03 && p.y < stand + 0.03, "standing height is the analytic skin gap");
    m3Vec3 n = m3Character_GetGroundNormal(world, hero);
    CHECK(n.y > 0.99f, "flat ground reports a vertical normal");

    // One hundred forward ticks: distance adds up exactly, the
    // height never wanders, grounded never flickers.
    double y0 = p.y;
    for (int32_t i = 0; i < 100; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.05f, -0.05f, 0.0f});
        CHECK(m3Character_IsGrounded(world, hero), "grounded holds through the walk");
    }
    p = m3Character_GetPosition(world, hero);
    CHECK(p.x > 4.9 && p.x < 5.1, "the walk covers the commanded distance");
    CHECK(fabs(p.y - y0) < 0.02, "the walk keeps its height");
    m3DestroyWorld(world);
}

static void TestWallSlideAndCeiling(void)
{
    m3World* world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){3.0, 2.0, 0.0};
    m3BodyId wallBody = NewBody(world, &wd);
    NewBoxShape(world, wallBody, &sd, (m3Vec3){0.2f, 2.0f, 4.0f}); // a wall at x = 2.8

    wd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId lidBody = NewBody(world, &wd);
    NewBoxShape(world, lidBody, &sd, (m3Vec3){1.5f, 0.2f, 1.5f}); // a lid at y = 2.8

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 30; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }

    // Charge the wall diagonally: x stops at the wall minus the
    // capsule radius and skin, z keeps flowing (the slide).
    for (int32_t i = 0; i < 100; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.06f, -0.02f, 0.03f});
    }
    m3Pos3 p = m3Character_GetPosition(world, hero);
    CHECK(p.x < 2.8 - 0.4 + 0.01, "the wall stops the blocked axis");
    CHECK(p.x > 2.8 - 0.4 - 0.1, "the character presses close to the wall");
    CHECK(p.z > 2.0, "the slide keeps the free axis flowing");
    CHECK(m3Character_IsGrounded(world, hero), "sliding along a wall stays grounded");

    // The ceiling: jump straight up under the lid; the head stops
    // below it and never penetrates.
    m3CharacterDef cd2 = m3DefaultCharacterDef();
    cd2.position = (m3Pos3){0.0, 1.0, 0.0};
    m3CharacterId jumper = NewCharacter(world, &cd2);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(world, jumper, (m3Vec3){0.0f, 0.08f, 0.0f});
    }
    p = m3Character_GetPosition(world, jumper);
    // Lid center y = 3 with half 0.2: the bottom face is 2.8, and
    // the head (center + halfHeight + radius) parks one skin below.
    double headroom = 2.8 - (double)(cd2.halfHeight + cd2.radius);
    CHECK(p.y < headroom + 0.01, "the ceiling stops the rise");
    CHECK(p.y > headroom - 0.1, "the character reaches the ceiling before stopping");
    m3DestroyWorld(world);
}

static void TestLedgeDropAndSnap(void)
{
    // A raised slab: walking off its edge must transition to
    // airborne (the snap gives up beyond snapDistance), and small
    // descents stay glued.
    m3World* world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef pd = m3DefaultBodyDef();
    pd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId slabBody = NewBody(world, &pd);
    NewBoxShape(world, slabBody, &sd, (m3Vec3){2.0f, 0.5f, 2.0f}); // top at y = 1

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    m3Pos3 p = m3Character_GetPosition(world, hero);
    CHECK(m3Character_IsGrounded(world, hero) && p.y > 1.8 && p.y < 2.0,
          "the character stands on the slab");

    // March off the edge with gravity-free forward ticks: past the
    // rim the snap has nothing walkable within reach, so grounded
    // must drop even though nothing pushed us down yet.
    bool wentAirborne = false;
    for (int32_t i = 0; i < 80; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.08f, 0.0f, 0.0f});
        if (!m3Character_IsGrounded(world, hero))
        {
            wentAirborne = true;
            break;
        }
    }
    CHECK(wentAirborne, "walking off the ledge reports airborne");
    m3DestroyWorld(world);
}

static void TestCharacterContracts(void)
{
    m3World* world = ArenaWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.radius = 0.0f;
    CHECK(!m3Character_IsValid(world, NewCharacter(world, &cd)), "a zero radius refuses");
    cd = m3DefaultCharacterDef();
    cd.skin = 0.0f;
    CHECK(!m3Character_IsValid(world, NewCharacter(world, &cd)), "a zero skin refuses");
    cd = m3DefaultCharacterDef();
    cd.maxSlopeAngle = 0.0f;
    CHECK(!m3Character_IsValid(world, NewCharacter(world, &cd)), "a zero slope refuses");
    cd = m3DefaultCharacterDef();
    cd.mass = 0.0f;
    CHECK(!m3Character_IsValid(world, NewCharacter(world, &cd)), "a zero mass refuses");
    cd = m3DefaultCharacterDef();
    cd.pushMaxMassRatio = -1.0f;
    CHECK(!m3Character_IsValid(world, NewCharacter(world, &cd)), "a negative push ratio refuses");

    cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3CharacterId one = NewCharacter(world, &cd);
    CHECK(m3Character_GetGroundBody(world, one).index1 == 0,
          "an airborne spawn grounds on nothing");
    cd.position = (m3Pos3){3.0, 2.0, 0.0};
    m3CharacterId two = NewCharacter(world, &cd);
    CHECK(m3Character_IsValid(world, one) && m3Character_IsValid(world, two), "the pool fills");
    CHECK(!m3Character_IsValid(world, NewCharacter(world, &cd)),
          "the character pool refuses past capacity");

    DestroyCharacter(world, one);
    CHECK(!m3Character_IsValid(world, one), "the destroyed character is stale");
    m3Character_Move(world, one, (m3Vec3){1.0f, 0.0f, 0.0f}); // stale: no-op
    m3Pos3 p = m3Character_GetPosition(world, one);
    CHECK(p.x == 0.0 && p.y == 0.0, "a stale getter returns zeros");
    float bad;
    uint32_t nanBits = 0x7FC00000u; // bit-built NaN: no compiler or
                                    // sanitizer gets an opinion
    memcpy(&bad, &nanBits, sizeof(bad));
    m3Character_Move(world, two, (m3Vec3){bad, 0.0f, 0.0f}); // NaN: no-op
    p = m3Character_GetPosition(world, two);
    CHECK(p.x == 3.0, "a hostile move never lands");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(world) != 0, "the world shrugs it off");
    m3DestroyWorld(world);
}

// The character found the way a program finds any shape: its capsule
// is a query's hit, and the body is the capsule's.
static m3ShapeId CapsuleOf(const m3World* world, m3Pos3 at)
{
    m3CastHit hit = {0};
    bool found = m3World_CastRayClosest(world, (m3Pos3){at.x, at.y + 5.0, at.z},
                                        (m3Vec3){0.0f, -5.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found, "a ray finds the character's capsule");
    return hit.shapeId;
}

// A character owns its body and capsule: they go with the character, and
// no call destroys, retypes, reshapes or adds to them on their own. Each
// such call refuses as invalid input and leaves the character whole, so
// its stance still changes and it still walks.
static void TestCharacterOwnsItsBodyAndCapsule(void)
{
    m3World* world = ArenaWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    m3ShapeId capsule = CapsuleOf(world, cd.position);
    m3BodyId body = m3Shape_GetBody(world, capsule);
    CHECK(m3Shape_GetType(world, capsule) == m3_capsuleShape, "the hit is the capsule");

    CHECK_REFUSES(world, CHECK(m3DestroyShape(world, capsule) == m3_errorInvalid, "destroy"),
                  "the capsule is not destroyed on its own");
    CHECK_REFUSES(world, CHECK(m3DestroyBody(world, body) == m3_errorInvalid, "destroy"),
                  "the body is not destroyed on its own");
    CHECK_REFUSES(world, m3Body_SetType(world, body, m3_dynamicBody), "the body is not retyped");
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    CHECK_REFUSES(world, m3Shape_SetSphere(world, capsule, &ball),
                  "the capsule is not made a sphere");
    m3Capsule rod = {{0.0f, -0.2f, 0.0f}, {0.0f, 0.2f, 0.0f}, 0.1f};
    CHECK_REFUSES(world, m3Shape_SetCapsule(world, capsule, &rod),
                  "the capsule is not reshaped but by the stance");
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId extra = m3_nullShapeId;
    CHECK_REFUSES(
        world,
        CHECK(m3CreateSphereShape(world, body, &sd, &ball, &extra) == m3_errorInvalid, "create"),
        "no shape is added to the body");

    CHECK(m3Shape_IsValid(world, capsule) && m3Body_IsValid(world, body) &&
              m3Body_GetType(world, body) == m3_kinematicBody &&
              m3Shape_GetType(world, capsule) == m3_capsuleShape,
          "the character is whole");
    CHECK(SetStance(world, hero, 0.25f, 0.2f), "its stance changes");
    m3Character_Move(world, hero, (m3Vec3){0.2f, -0.5f, 0.0f});
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3Character_GetPosition(world, hero).x > 0.1, "and it walks");

    DestroyCharacter(world, hero);
    CHECK(!m3Shape_IsValid(world, capsule) && !m3Body_IsValid(world, body),
          "destroying the character takes its body and capsule");
    m3DestroyWorld(world);
}

// A tape is data: a destroy or a retype aimed at a character's own body
// or capsule is refused on replay as on the public call. The tape records
// such a call on an ordinary sphere, and its id is then pointed at the
// character's.
static void TestReplayHoldsTheOwnership(void)
{
    enum
    {
        TAPE_BYTES = 4096
    };
    static uint8_t tape[TAPE_BYTES];
    for (int32_t kind = 0; kind < 3; ++kind)
    {
        m3World* world = ArenaWorld();
        m3CharacterDef cd = m3DefaultCharacterDef();
        cd.position = (m3Pos3){0.0, 2.0, 0.0};
        NewCharacter(world, &cd);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.position = (m3Pos3){4.0, 2.0, 0.0};
        m3BodyId other = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
        m3ShapeId otherShape = NewSphereShape(world, other, &sd, &ball);
        m3ShapeId capsule = CapsuleOf(world, cd.position);
        m3BodyId body = m3Shape_GetBody(world, capsule);
        int64_t size = m3World_GetSnapshotSize(world);
        uint8_t* start = (uint8_t*)malloc((size_t)size);
        CHECK(start != NULL && Snapshot(world, start, size) == size, "the start is saved");

        CHECK(StartJournal(world, tape, TAPE_BYTES), "the journal starts");
        if (kind == 0)
        {
            DestroyShape(world, otherShape);
        }
        else if (kind == 1)
        {
            DestroyBody(world, other);
        }
        else
        {
            m3Body_SetType(world, other, m3_dynamicBody);
        }
        int64_t bytes = StopJournal(world);
        // The record's payload starts with the id: retarget it.
        const void* from = kind == 0 ? (const void*)&otherShape : (const void*)&other;
        const void* to = kind == 0 ? (const void*)&capsule : (const void*)&body;
        size_t idBytes = kind == 0 ? sizeof(m3ShapeId) : sizeof(m3BodyId);
        int32_t found = 0;
        for (int32_t at = 0; at + (int32_t)idBytes <= bytes; ++at)
        {
            if (memcmp(tape + at, from, idBytes) == 0)
            {
                memcpy(tape + at, to, idBytes);
                found += 1;
            }
        }
        CHECK(found == 1, "the tape names the ordinary object once");

        CHECK(Restore(world, start, size), "the start is restored");
        CHECK(m3World_ReplayJournal(world, tape, bytes) == m3_errorFormat,
              "a tape aimed at the character's own body or capsule is refused");
        CHECK(m3Shape_IsValid(world, capsule) && m3Body_IsValid(world, body) &&
                  m3Body_GetType(world, body) == m3_kinematicBody,
              "and the character is whole");
        free(start);
        m3DestroyWorld(world);
    }
}

// Stairs, slopes, the voxel floor, and the welded seam.

static void TestCharacterOnCharacter(void)
{
    // One walker stands on another's head: the upper grounds on the
    // lower's kinematic body, pushes nothing (kinematic bodies are
    // not pushable), and when the lower walks away the upper's next
    // moves land it on the floor below.
    m3World* world = ArenaWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3CharacterId lower = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 20; ++i)
    {
        m3Character_Move(world, lower, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    // Lower stands at 0.92; its head (capsule top) is at 1.84.
    cd.position = (m3Pos3){0.0, 3.5, 0.0};
    m3CharacterId upper = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 20; ++i)
    {
        m3Character_Move(world, upper, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(world, upper), "the upper walker stands on the head");
    m3BodyId underBody = m3Character_GetGroundBody(world, upper);
    CHECK(underBody.index1 != 0, "the head is a real ground body");
    m3Pos3 lowerP = m3Character_GetPosition(world, lower);
    CHECK(lowerP.y > 0.9 && lowerP.y < 0.94, "the lower walker carries no weight");

    // The lower walks out; the upper's own moves find the truth.
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(world, lower, (m3Vec3){0.05f, 0.0f, 0.0f});
    }
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(world, upper, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    m3Pos3 upperP = m3Character_GetPosition(world, upper);
    CHECK(upperP.y > 0.9 && upperP.y < 0.94, "the upper walker lands on the floor");
    CHECK(m3Character_IsGrounded(world, upper), "the landing grounds honestly");
    m3DestroyWorld(world);
}

static void TestPushCrate(void)
{
    // A blocked walk shoves a light crate: impulse = mass * blocked
    // displacement, applied at the contact, so the crate gains both
    // speed and forward spin. Heavier than pushMaxMassRatio * mass
    // is a wall: not one millimeter per second.
    m3World* world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){2.0, 0.5, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    NewBoxShape(world, crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f}); // mass 1

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.9, 2.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    // Park against the crate face at x = 1.5, then shove once.
    for (int32_t i = 0; i < 4; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.05f, 0.0f, 0.0f});
    }
    m3Vec3 v0 = m3Body_GetLinearVelocity(world, crate);
    CHECK(v0.x == 0.0f, "the parked crate has no velocity yet");
    m3Character_Move(world, hero, (m3Vec3){0.05f, 0.0f, 0.0f});
    m3Vec3 v = m3Body_GetLinearVelocity(world, crate);
    m3Vec3 w = m3Body_GetAngularVelocity(world, crate);
    CHECK(v.x > 0.5f, "the pressed crate takes real speed");
    CHECK(v.x <= 80.0f * 0.05f + 0.01f, "the impulse never exceeds the full intent");
    CHECK(w.z != 0.0f, "the off-center contact spins the crate");

    // The heavy twin refuses: density 200 makes 200 kg against a
    // cap of pushMaxMassRatio 2 * mass 80 = 160.
    m3ShapeDef hd = m3DefaultShapeDef();
    hd.density = 200.0f;
    m3BodyDef hb = m3DefaultBodyDef();
    hb.type = m3_dynamicBody;
    hb.position = (m3Pos3){2.0, 0.5, 3.0};
    m3BodyId heavy = NewBody(world, &hb);
    NewBoxShape(world, heavy, &hd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3CharacterDef cd2 = m3DefaultCharacterDef();
    cd2.position = (m3Pos3){0.9, 2.0, 3.0};
    m3CharacterId mover = NewCharacter(world, &cd2);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(world, mover, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    for (int32_t i = 0; i < 8; ++i)
    {
        m3Character_Move(world, mover, (m3Vec3){0.05f, 0.0f, 0.0f});
    }
    m3Vec3 hv = m3Body_GetLinearVelocity(world, heavy);
    CHECK(hv.x == 0.0f && hv.y == 0.0f && hv.z == 0.0f, "the heavy crate is a wall");
    m3DestroyWorld(world);
}

static void TestPlatformRideAndElevator(void)
{
    // A kinematic platform ferries its rider sideways, then down,
    // with no Move commands at all: the carry is the step's job.
    // Twin worlds prove the ride deterministic; destroying the
    // platform clears the ground body reference immediately.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3World* world = ArenaWorld();
        m3ShapeDef sd = m3DefaultShapeDef();
        m3BodyDef pd = m3DefaultBodyDef();
        pd.type = m3_kinematicBody;
        pd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId platform = NewBody(world, &pd);
        NewBoxShape(world, platform, &sd, (m3Vec3){2.0f, 0.25f, 2.0f}); // top at 2.25

        m3CharacterDef cd = m3DefaultCharacterDef();
        cd.position = (m3Pos3){0.0, 3.5, 0.0};
        m3CharacterId hero = NewCharacter(world, &cd);
        for (int32_t i = 0; i < 20; ++i)
        {
            m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
        }
        double y0 = m3Character_GetPosition(world, hero).y;
        CHECK(m3Character_IsGrounded(world, hero), "the rider lands on the platform");
        m3BodyId under = m3Character_GetGroundBody(world, hero);
        CHECK(under.index1 == platform.index1 && under.generation == platform.generation,
              "the ground body is the platform");

        m3Body_SetLinearVelocity(world, platform, (m3Vec3){0.5f, 0.0f, 0.0f});
        for (int32_t i = 0; i < 60; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Pos3 p = m3Character_GetPosition(world, hero);
        CHECK(p.x > 0.48 && p.x < 0.52, "the ride carries the full half meter");
        CHECK(p.y > y0 - 0.02 && p.y < y0 + 0.02, "the ride keeps standing height");
        CHECK(m3Character_IsGrounded(world, hero), "the ride never breaks grounding");

        m3Body_SetLinearVelocity(world, platform, (m3Vec3){0.0f, -0.3f, 0.0f});
        for (int32_t i = 0; i < 60; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        p = m3Character_GetPosition(world, hero);
        CHECK(p.y > y0 - 0.32 && p.y < y0 - 0.28, "the elevator lowers its rider");
        CHECK(m3Character_IsGrounded(world, hero), "the descent stays glued");

        hashes[run] = m3World_Hash(world);
        if (run == 1)
        {
            DestroyBody(world, platform);
            m3BodyId gone = m3Character_GetGroundBody(world, hero);
            CHECK(gone.index1 == 0, "a destroyed platform grounds nobody");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin rides are bit-identical");
}

static void TestCarouselAndFragmentSurf(void)
{
    // A spinning platform sweeps its rider along the arc the rigid
    // carry predicts, and a drifting dynamic body (a fragment in
    // destruction terms) ferries a walker the same way. Standing
    // is not pushing: the fragment takes no impulse from feet.
    m3World* world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef pd = m3DefaultBodyDef();
    pd.type = m3_kinematicBody;
    pd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId carousel = NewBody(world, &pd);
    NewBoxShape(world, carousel, &sd, (m3Vec3){2.5f, 0.25f, 2.5f});

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){1.5, 3.5, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 20; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    m3Pos3 start = m3Character_GetPosition(world, hero);
    m3Body_SetAngularVelocity(world, carousel, (m3Vec3){0.0f, 0.4f, 0.0f});
    for (int32_t i = 0; i < 90; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // Expected: the start offset swept by the platform's own
    // accumulated rotation (read back, not assumed).
    m3Quat q = m3Body_GetRotation(world, carousel);
    m3Pos3 axis = m3Body_GetPosition(world, carousel);
    m3Vec3 rel = {(float)(start.x - axis.x), 0.0f, (float)(start.z - axis.z)};
    m3Vec3 swept = m3RotateVec3(q, rel);
    m3Pos3 p = m3Character_GetPosition(world, hero);
    CHECK(fabs(p.x - (axis.x + (double)swept.x)) < 0.03, "the carousel sweeps the rider (x)");
    CHECK(fabs(p.z - (axis.z + (double)swept.z)) < 0.03, "the carousel sweeps the rider (z)");
    CHECK(m3Character_IsGrounded(world, hero), "the carousel keeps its rider grounded");
    m3DestroyWorld(world);

    // Fragment surfing: gravity-free drifting box, walker on top.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.characterCapacity = 1;
    m3World* sea = NewWorld(&wd);
    m3ShapeDef fs = m3DefaultShapeDef();
    m3BodyDef fd = m3DefaultBodyDef();
    fd.type = m3_dynamicBody;
    fd.position = (m3Pos3){0.0, 0.5, 0.0};
    fd.gravityScale = 0.0f;
    m3BodyId fragment = NewBody(sea, &fd);
    NewBoxShape(sea, fragment, &fs, (m3Vec3){0.6f, 0.5f, 0.6f}); // top at 1.0

    m3CharacterDef cf = m3DefaultCharacterDef();
    cf.position = (m3Pos3){0.0, 2.5, 0.0};
    m3CharacterId surfer = NewCharacter(sea, &cf);
    for (int32_t i = 0; i < 20; ++i)
    {
        m3Character_Move(sea, surfer, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(sea, surfer), "the surfer boards the fragment");
    m3Body_SetLinearVelocity(sea, fragment, (m3Vec3){0.8f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(sea, 1.0f / 60.0f, 4);
    }
    m3Pos3 sp = m3Character_GetPosition(sea, surfer);
    CHECK(sp.x > 0.38 && sp.x < 0.42, "the fragment ferries its surfer");
    CHECK(m3Character_IsGrounded(sea, surfer), "surfing never breaks grounding");
    m3Vec3 fv = m3Body_GetLinearVelocity(sea, fragment);
    CHECK(fv.y == 0.0f, "standing is not pushing: no downward impulse from feet");
    m3DestroyWorld(sea);
}

static void TestVoxelStairs(void)
{
    // Four quarter-meter risers climb; a half-meter riser refuses
    // (stepHeight is 0.35). The staircase is a voxel chunk, so the
    // whole exercise runs on the merged-box surface.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 1;
    def.characterCapacity = 1;
    m3World* world = NewWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    // cellSize 0.25: floor at layer 0; stairs rise one layer per
    // two cells of x from x = 4; a double riser at x = 12.
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            int32_t top = 0;
            if (x >= 4)
            {
                top = (x - 4) / 2 + 1;
            }
            if (x >= 12)
            {
                top += 2; // the illegal cliff: two layers at once
            }
            if (top > 15)
            {
                top = 15;
            }
            for (int32_t y = 0; y <= top; ++y)
            {
                voxels[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    CHECK(m3Shape_IsValid(world, NewVoxelChunkShape(world, ground, &sd, voxels, NULL, 0.25f)),
          "the staircase creates");

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.radius = 0.2f;
    cd.halfHeight = 0.3f;
    cd.position = (m3Pos3){0.5, 2.0, 2.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(world, hero), "the character lands at the stair base");

    // March up the stairs.
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.03f, -0.03f, 0.0f});
    }
    m3Pos3 p = m3Character_GetPosition(world, hero);
    CHECK(p.x > 2.6 && p.x < 3.05, "the climb is stopped only by the double riser");
    // Feet on the step below the cliff: the top under x ~ 2.9 is
    // layer (2.9/0.25 - 4)/2 + 1 = 4 -> surface y = 5 * 0.25 = 1.25.
    double stand = 1.25 + (double)(cd.halfHeight + cd.radius + cd.skin);
    CHECK(p.y > stand - 0.05 && p.y < stand + 0.05, "the character stands on the fourth step");
    CHECK(m3Character_IsGrounded(world, hero), "stairs never break grounding");
    m3DestroyWorld(world);
}

static void TestSteepSlopeRefusal(void)
{
    // A sixty-degree ramp is a wall in walking terms: pressing into
    // it must not mint height, tick after tick.
    m3World* world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){4.0, 0.0, 0.0};
    // Yaw the box about z by sixty degrees to make the steep face.
    float half = 0.5235988f; // 60 degrees / 2
    rd.rotation = (m3Quat){0.0f, 0.0f, sinf(half), cosf(half)};
    m3BodyId rampBody = NewBody(world, &rd);
    NewBoxShape(world, rampBody, &sd, (m3Vec3){3.0f, 3.0f, 3.0f});

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    double y0 = m3Character_GetPosition(world, hero).y;
    for (int32_t i = 0; i < 150; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.05f, -0.02f, 0.0f});
    }
    m3Pos3 p = m3Character_GetPosition(world, hero);
    CHECK(p.y - y0 < 0.08, "the steep face mints no height");
    m3DestroyWorld(world);
}

static void TestCarvedFloorDrop(void)
{
    // The interplay nobody else can pose: the floor is carved out
    // from under a grounded character, and grounding drops THE SAME
    // step, before any move.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 1;
    def.characterCapacity = 2; // the hero AND the bystander
    m3World* world = NewWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (0 + 16 * z)] = 1; // a one-layer floor
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeId slab = NewVoxelChunkShape(world, ground, &sd, voxels, NULL, 1.0f);

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){8.5, 4.0, 8.5};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(world, hero), "the character stands on the voxel floor");

    // Carve the tiles under foot: NO move happens after this; the
    // grounding must drop from the edit alone.
    int32_t lo[3] = {7, 0, 7};
    int32_t hi[3] = {10, 0, 10};
    int32_t cleared = 0;
    CHECK(m3VoxelChunkShape_ClearBox(world, slab, lo, hi, &cleared) == m3_success && cleared == 16,
          "the floor vanishes under foot");
    CHECK(!m3Character_IsGrounded(world, hero), "grounding drops the same step the floor goes");

    // An edit elsewhere never touches a grounded character.
    m3CharacterDef cd2 = m3DefaultCharacterDef();
    cd2.position = (m3Pos3){2.5, 4.0, 2.5};
    m3CharacterId bystander = NewCharacter(world, &cd2);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(world, bystander, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(world, bystander), "the bystander stands");
    int32_t lo2[3] = {13, 0, 13};
    int32_t hi2[3] = {14, 0, 14};
    CHECK(m3VoxelChunkShape_ClearBox(world, slab, lo2, hi2, &cleared) == m3_success && cleared == 4,
          "the far carve lands");
    CHECK(m3Character_IsGrounded(world, bystander), "a far edit never shakes the bystander");
    m3DestroyWorld(world);
}

static void TestWeldedSeamWalk(void)
{
    // A character crosses the chunk border of two welded slabs:
    // height wander stays inside the skin and grounding never
    // flickers.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 2;
    def.characterCapacity = 1;
    m3World* world = NewWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            for (int32_t y = 0; y < 2; ++y)
            {
                voxels[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId groundA = NewBody(world, &gd);
    CHECK(m3Shape_IsValid(world, NewVoxelChunkShape(world, groundA, &sd, voxels, NULL, 1.0f)),
          "slab A creates");
    gd.position = (m3Pos3){16.0, 0.0, 0.0};
    m3BodyId groundB = NewBody(world, &gd);
    CHECK(m3Shape_IsValid(world, NewVoxelChunkShape(world, groundB, &sd, voxels, NULL, 1.0f)),
          "slab B creates and welds");

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){12.0, 4.0, 8.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    double y0 = m3Character_GetPosition(world, hero).y;
    double maxWander = 0.0;
    for (int32_t i = 0; i < 160; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.06f, -0.04f, 0.0f});
        CHECK(m3Character_IsGrounded(world, hero), "grounding never flickers at the seam");
        double w = m3Character_GetPosition(world, hero).y - y0;
        if (w < 0.0)
        {
            w = -w;
        }
        if (w > maxWander)
        {
            maxWander = w;
        }
    }
    m3Pos3 p = m3Character_GetPosition(world, hero);
    CHECK(p.x > 20.0, "the walk crossed the welded border");
    CHECK(maxWander < 0.03, "the seam never lifts or drops the walker");
    m3DestroyWorld(world);
}

// Stairs, slopes, the voxel floor, and the welded seam.
static void TestHostileMoves(void)
{
    // The walker takes abuse without corruption. A
    // planetary-scale move stops at the first wall like any other;
    // denormal dust moves change not one bit; the world stays
    // finite through all of it.
    m3World* world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){6.0, 2.0, 0.0};
    m3BodyId wall = NewBody(world, &wd);
    NewBoxShape(world, wall, &sd, (m3Vec3){0.5f, 4.0f, 6.0f}); // face at x = 5.5

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3CharacterId hero = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 20; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }

    // Past the caster's float budget: hostile, a bitwise no-op.
    m3Pos3 pre = m3Character_GetPosition(world, hero);
    m3Character_Move(world, hero, (m3Vec3){1.0e30f, 0.0f, 0.0f});
    m3Pos3 p = m3Character_GetPosition(world, hero);
    CHECK(memcmp(&pre, &p, sizeof(pre)) == 0, "a planetary move is a hostile no-op");
    // Merely enormous: legal, and the wall wins like always.
    m3Character_Move(world, hero, (m3Vec3){1.0e6f, 0.0f, 0.0f});
    p = m3Character_GetPosition(world, hero);
    CHECK(p.x < 5.5 && p.x > 4.9, "a kilometer-scale move parks at the wall");
    CHECK(isfinite(p.x) && isfinite(p.y) && isfinite(p.z), "the parked walker is finite");

    // Denormal dust: below the slide's own noise floor, the move
    // must be a bitwise no-op, a thousand times over.
    m3Pos3 before = m3Character_GetPosition(world, hero);
    float dust;
    uint32_t dustBits = 0x00000001u; // the smallest positive denormal
    memcpy(&dust, &dustBits, sizeof(dust));
    for (int32_t i = 0; i < 1000; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){dust, dust, dust});
    }
    m3Pos3 after = m3Character_GetPosition(world, hero);
    CHECK(memcmp(&before, &after, sizeof(before)) == 0, "denormal dust moves nothing");

    m3Character_Move(world, hero, (m3Vec3){-1.0e6f, 0.0f, 0.0f});
    p = m3Character_GetPosition(world, hero);
    CHECK(isfinite(p.x), "the return trip stays finite");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(world) != 0, "the abused world still hashes");
    m3DestroyWorld(world);
}

int main(void)
{
    TestFlatWalk();
    TestWallSlideAndCeiling();
    TestLedgeDropAndSnap();
    TestCharacterContracts();
    TestCharacterOwnsItsBodyAndCapsule();
    TestReplayHoldsTheOwnership();
    TestVoxelStairs();
    TestSteepSlopeRefusal();
    TestCarvedFloorDrop();
    TestWeldedSeamWalk();
    TestPushCrate();
    TestPlatformRideAndElevator();
    TestCarouselAndFragmentSurf();
    TestCharacterOnCharacter();
    TestHostileMoves();
    if (s_failures == 0)
    {
        printf("test_character: all green\n");
        return 0;
    }
    printf("test_character: %d failure(s)\n", s_failures);
    return 1;
}
