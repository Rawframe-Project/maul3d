// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal gate: a recorded session replayed into a fresh world must
// reproduce the original bit for bit, including the minted ids (id
// determinism). Equality is
// checked field by field with memcmp. Black box: public headers only.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/character.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

static int SamePos(m3Pos3 a, m3Pos3 b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

static int SameVec(m3Vec3 a, m3Vec3 b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

// A refused create is not on the tape, so it must leave no trace in the
// id pools: a replay of the session mints the same ids.
static void TestRefusedCreatesMoveNoIds(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.meshCapacity = 1;
    def.bodyCapacity = 16;
    m3World* world = NewWorld(&def);
    static uint8_t tape[65536];
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the tape starts");

    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static const m3Vec3 tri[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    static const uint16_t idx[3] = {0, 1, 2};
    CHECK(m3Shape_IsValid(world, NewMeshShape(world, ground, &sd, tri, 3, idx, 1)),
          "the first mesh fits");
    CHECK(!m3Shape_IsValid(world, NewMeshShape(world, ground, &sd, tri, 3, idx, 1)),
          "the second mesh finds no mesh slot");
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    CHECK(m3Shape_IsValid(world, NewSphereShape(world, ball, &sd, &sphere)), "a sphere follows");

    // Four clusters at the corners of a large tetrahedron, one point of
    // each per tet, so almost every tet edge is new: more than the edge
    // budget of one soft body.
    static m3Vec3 points[512];
    static uint16_t tets[4 * 1024];
    const m3Vec3 corners[4] = {
        {0.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f}, {0.0f, 10.0f, 0.0f}, {0.0f, 0.0f, 10.0f}};
    for (int32_t i = 0; i < 512; ++i)
    {
        m3Vec3 c = corners[i / 128];
        float j = 0.001f * (float)(i % 128);
        points[i] = (m3Vec3){c.x + j, c.y + 2.0f * j, c.z + 3.0f * j};
    }
    for (int32_t t = 0; t < 1024; ++t)
    {
        tets[4 * t + 0] = (uint16_t)(t % 128);
        tets[4 * t + 1] = (uint16_t)(128 + (t / 8) % 128);
        tets[4 * t + 2] = (uint16_t)(256 + (t * 3 + t / 128) % 128);
        tets[4 * t + 3] = (uint16_t)(384 + (t * 5 + t / 32) % 128);
    }
    m3SoftBodyDef soft = m3DefaultSoftBodyDef();
    CHECK(!m3SoftBody_IsValid(world, NewSoftBodyTet(world, &soft, points, 512, tets, 1024)),
          "a tet body past the edge budget refuses");
    soft.countX = 2;
    soft.countY = 2;
    soft.countZ = 2;
    CHECK(m3SoftBody_IsValid(world, NewSoftBody(world, &soft)), "a lattice follows");

    // A character needs a body: with the body pool full it refuses, and
    // the next character after a body frees up gets the first free id.
    m3BodyId filler[64];
    int32_t fillers = 0;
    bd.type = m3_staticBody;
    while (fillers < 64)
    {
        filler[fillers] = NewBody(world, &bd);
        if (!m3Body_IsValid(world, filler[fillers]))
        {
            break;
        }
        fillers += 1;
    }
    CHECK(fillers > 0 && fillers < 64, "the body pool fills up");
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){5.0, 2.0, 0.0};
    CHECK(!m3Character_IsValid(world, NewCharacter(world, &cd)),
          "a character refuses without a body");
    DestroyBody(world, filler[fillers - 1]);
    CHECK(m3Character_IsValid(world, NewCharacter(world, &cd)), "and fits once one frees up");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    int64_t bytes = StopJournal(world);
    CHECK(bytes > 0, "the tape closes");

    m3World* twin = NewWorld(&def);
    CHECK(ReplayJournal(twin, tape, bytes), "the tape replays with the same ids");
    CHECK(m3World_Hash(twin) == m3World_Hash(world), "into the same world");
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

// Records one dynamic body with a sphere, with the body def's bool at
// `offset` set to `value`, and returns the tape's size.
static int64_t RecordBodyWithFlag(uint8_t* tape, int32_t capacity, size_t offset, bool value)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    StartJournal(world, tape, capacity);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    memcpy((uint8_t*)&bd + offset, &value, sizeof(value));
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, body, &sd, &sphere);
    int64_t bytes = StopJournal(world);
    m3DestroyWorld(world);
    return bytes;
}

// Replays a tape into a fresh world, steps it, and returns its hash, or
// 0 when the replay refuses.
static uint64_t ReplayAndStep(const uint8_t* tape, int64_t bytes)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    uint64_t hash = 0;
    if (ReplayJournal(world, tape, bytes))
    {
        for (int i = 0; i < 30; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hash = m3World_Hash(world);
    }
    m3DestroyWorld(world);
    return hash;
}

// A tape is untrusted bytes: a bool of an embedded body def that holds
// neither 0 nor 1 reads as true, exactly as if it held 1.
static void TestBodyDefBoolBytesReadAsTrue(void)
{
    static const struct
    {
        size_t offset;
        const char* name;
    } fields[] = {
        {offsetof(m3BodyDef, motionLocks.linearX), "motionLocks.linearX"},
        {offsetof(m3BodyDef, motionLocks.linearY), "motionLocks.linearY"},
        {offsetof(m3BodyDef, motionLocks.linearZ), "motionLocks.linearZ"},
        {offsetof(m3BodyDef, motionLocks.angularX), "motionLocks.angularX"},
        {offsetof(m3BodyDef, motionLocks.angularY), "motionLocks.angularY"},
        {offsetof(m3BodyDef, motionLocks.angularZ), "motionLocks.angularZ"},
        {offsetof(m3BodyDef, enableBullet), "enableBullet"},
        {offsetof(m3BodyDef, enableSleep), "enableSleep"},
        {offsetof(m3BodyDef, enable), "enable"},
        {offsetof(m3BodyDef, enableFastRotation), "enableFastRotation"},
    };
    static uint8_t on[4096];
    static uint8_t off[4096];
    static uint8_t again[4096];
    for (size_t f = 0; f < sizeof(fields) / sizeof(fields[0]); ++f)
    {
        // Each recording runs in a new world generation, which the ids
        // on the tape carry; a second "on" tape tells those bytes apart
        // from the field's own byte.
        int64_t bytes = RecordBodyWithFlag(on, (int32_t)sizeof(on), fields[f].offset, true);
        int64_t offBytes = RecordBodyWithFlag(off, (int32_t)sizeof(off), fields[f].offset, false);
        int64_t againBytes =
            RecordBodyWithFlag(again, (int32_t)sizeof(again), fields[f].offset, true);
        int32_t at = -1;
        int32_t differing = 0;
        for (int32_t i = 0; bytes > 0 && bytes == offBytes && bytes == againBytes && i < bytes; ++i)
        {
            if (on[i] != off[i] && on[i] == again[i])
            {
                at = i;
                differing += 1;
            }
        }
        if (differing != 1)
        {
            printf("FAIL: %s is not one byte apart on the tape (%d)\n", fields[f].name, differing);
            s_failures += 1;
            continue;
        }
        uint64_t expected = ReplayAndStep(on, bytes);
        on[at] = 0x81;
        uint64_t damaged = ReplayAndStep(on, bytes);
        if (expected == 0 || damaged != expected)
        {
            printf("FAIL: %s = 0x81 on the tape does not replay as true\n", fields[f].name);
            s_failures += 1;
        }
    }
}

int main(void)
{
    TestRefusedCreatesMoveNoIds();
    TestBodyDefBoolBytesReadAsTrue();
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;

    // Record a session on world A: creates, setters, a destroy, and a
    // recreate into the recycled slot.
    m3World* a = NewWorld(&def);
    uint8_t buffer[4096];
    CHECK(StartJournal(a, buffer, (int32_t)sizeof(buffer)), "journal begins");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 5.0, 0.0};
    m3BodyId a0 = NewBody(a, &bd);
    bd.position = (m3Pos3){2.0, 5.0, -1.0};
    m3BodyId a1 = NewBody(a, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3ShapeId shape0 = NewSphereShape(a, a0, &sd, &ball);
    CHECK(m3Shape_IsValid(a, shape0), "journaled sphere created");
    m3Body_SetLinearVelocity(a, a0, (m3Vec3){1.0f, 0.0f, 0.0f});
    m3Body_SetAngularVelocity(a, a1, (m3Vec3){0.0f, 3.0f, 0.0f});
    DestroyBody(a, a0);
    bd.position = (m3Pos3){-4.0, 1.0, 2.0};
    m3BodyId a2 = NewBody(a, &bd); // recycles a0's slot, new generation
    CHECK(a2.index1 == a0.index1 && a2.generation != a0.generation, "recycle in the journal");

    int64_t bytes = StopJournal(a);
    CHECK(bytes > 0, "journal ends with bytes");

    // Replay into a fresh world B: identical ids, identical state.
    m3World* b = NewWorld(&def);
    CHECK(ReplayJournal(b, buffer, bytes), "replay succeeds");

    // The replay mints the recorded ids, so world A's ids name the same
    // bodies in world B.
    CHECK(m3Body_IsValid(b, a1) && m3Body_IsValid(b, a2), "replayed ids validate in world B");
    CHECK(SamePos(m3Body_GetPosition(b, a1), m3Body_GetPosition(a, a1)), "b1 position matches");
    CHECK(SamePos(m3Body_GetPosition(b, a2), m3Body_GetPosition(a, a2)), "b2 position matches");
    CHECK(SameVec(m3Body_GetAngularVelocity(b, a1), m3Body_GetAngularVelocity(a, a1)),
          "b1 angular velocity matches");
    CHECK(!m3Body_IsValid(b, a0), "the destroyed body is stale in the replay too");

    // A truncated stream is rejected loudly and never half-applies
    // silently.
    m3World* c = NewWorld(&def);
    CHECK(!ReplayJournal(c, buffer, bytes - 3), "a truncated journal is rejected");

    // Overflow: a too-small buffer latches and End reports -1.
    m3World* d = NewWorld(&def);
    uint8_t tiny[16];
    CHECK(StartJournal(d, tiny, (int32_t)sizeof(tiny)), "tiny journal begins");
    NewBody(d, &bd);
    CHECK(StopJournal(d) == -1, "journal overflow reports -1, never silence");

    m3DestroyWorld(a);
    m3DestroyWorld(b);
    m3DestroyWorld(c);
    m3DestroyWorld(d);

    if (s_failures == 0)
    {
        printf("test_journal: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
