// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal gate: a recorded session replayed into a fresh world must
// reproduce the original bit for bit, including the minted ids (id
// determinism). Equality is
// checked field by field with memcmp. Black box: public headers only.

#include "maul3d/body.h"
#include "maul3d/character.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "test_harness.h"

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
    m3WorldId world = m3CreateWorld(&def);
    static uint8_t tape[65536];
    CHECK(m3World_StartJournal(world, tape, (int32_t)sizeof(tape)), "the tape starts");

    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static const m3Vec3 tri[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    static const uint16_t idx[3] = {0, 1, 2};
    CHECK(m3Shape_IsValid(m3CreateMeshShape(ground, &sd, tri, 3, idx, 1)), "the first mesh fits");
    CHECK(!m3Shape_IsValid(m3CreateMeshShape(ground, &sd, tri, 3, idx, 1)),
          "the second mesh finds no mesh slot");
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    CHECK(m3Shape_IsValid(m3CreateSphereShape(ball, &sd, &sphere)), "a sphere follows");

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
    CHECK(!m3SoftBody_IsValid(m3CreateSoftBodyTet(world, &soft, points, 512, tets, 1024)),
          "a tet body past the edge budget refuses");
    soft.countX = 2;
    soft.countY = 2;
    soft.countZ = 2;
    CHECK(m3SoftBody_IsValid(m3CreateSoftBody(world, &soft)), "a lattice follows");

    // A character needs a body: with the body pool full it refuses, and
    // the next character after a body frees up gets the first free id.
    m3BodyId filler[64];
    int32_t fillers = 0;
    bd.type = m3_staticBody;
    while (fillers < 64)
    {
        filler[fillers] = m3CreateBody(world, &bd);
        if (!m3Body_IsValid(filler[fillers]))
        {
            break;
        }
        fillers += 1;
    }
    CHECK(fillers > 0 && fillers < 64, "the body pool fills up");
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){5.0, 2.0, 0.0};
    CHECK(!m3Character_IsValid(m3CreateCharacter(world, &cd)),
          "a character refuses without a body");
    m3DestroyBody(filler[fillers - 1]);
    CHECK(m3Character_IsValid(m3CreateCharacter(world, &cd)), "and fits once one frees up");
    m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t bytes = m3World_StopJournal(world);
    CHECK(bytes > 0, "the tape closes");

    m3WorldId twin = m3CreateWorld(&def);
    CHECK(m3World_ReplayJournal(twin, tape, bytes), "the tape replays with the same ids");
    CHECK(m3World_Hash(twin) == m3World_Hash(world), "into the same world");
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

int main(void)
{
    TestRefusedCreatesMoveNoIds();
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;

    // Record a session on world A: creates, setters, a destroy, and a
    // recreate into the recycled slot.
    m3WorldId a = m3CreateWorld(&def);
    uint8_t buffer[4096];
    CHECK(m3World_StartJournal(a, buffer, (int32_t)sizeof(buffer)), "journal begins");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 5.0, 0.0};
    m3BodyId a0 = m3CreateBody(a, &bd);
    bd.position = (m3Pos3){2.0, 5.0, -1.0};
    m3BodyId a1 = m3CreateBody(a, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3ShapeId shape0 = m3CreateSphereShape(a0, &sd, &ball);
    CHECK(m3Shape_IsValid(shape0), "journaled sphere created");
    m3Body_SetLinearVelocity(a0, (m3Vec3){1.0f, 0.0f, 0.0f});
    m3Body_SetAngularVelocity(a1, (m3Vec3){0.0f, 3.0f, 0.0f});
    m3DestroyBody(a0);
    bd.position = (m3Pos3){-4.0, 1.0, 2.0};
    m3BodyId a2 = m3CreateBody(a, &bd); // recycles a0's slot, new generation
    CHECK(a2.index1 == a0.index1 && a2.generation != a0.generation, "recycle in the journal");

    int32_t bytes = m3World_StopJournal(a);
    CHECK(bytes > 0, "journal ends with bytes");

    // Replay into a fresh world B: identical ids, identical state.
    m3WorldId b = m3CreateWorld(&def);
    CHECK(m3World_ReplayJournal(b, buffer, bytes), "replay succeeds");

    m3BodyId b1 = {a1.index1, b.index1 - 1 == 0 ? 0 : (uint16_t)(b.index1 - 1), a1.generation};
    b1.world0 = (uint16_t)(b.index1 - 1);
    m3BodyId b2 = {a2.index1, (uint16_t)(b.index1 - 1), a2.generation};
    CHECK(m3Body_IsValid(b1) && m3Body_IsValid(b2), "replayed ids validate in world B");
    CHECK(SamePos(m3Body_GetPosition(b1), m3Body_GetPosition(a1)), "b1 position matches");
    CHECK(SamePos(m3Body_GetPosition(b2), m3Body_GetPosition(a2)), "b2 position matches");
    CHECK(SameVec(m3Body_GetAngularVelocity(b1), m3Body_GetAngularVelocity(a1)),
          "b1 angular velocity matches");
    m3BodyId b0 = {a0.index1, (uint16_t)(b.index1 - 1), a0.generation};
    CHECK(!m3Body_IsValid(b0), "the destroyed body is stale in the replay too");

    // A truncated stream is rejected loudly and never half-applies
    // silently.
    m3WorldId c = m3CreateWorld(&def);
    CHECK(!m3World_ReplayJournal(c, buffer, bytes - 3), "a truncated journal is rejected");

    // Overflow: a too-small buffer latches and End reports -1.
    m3WorldId d = m3CreateWorld(&def);
    uint8_t tiny[16];
    CHECK(m3World_StartJournal(d, tiny, (int32_t)sizeof(tiny)), "tiny journal begins");
    m3CreateBody(d, &bd);
    CHECK(m3World_StopJournal(d) == -1, "journal overflow reports -1, never silence");

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
