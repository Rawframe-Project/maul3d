// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core gate: world creation (its checks in order, the allocator the def
// carries) and the refusals every call makes, each with its status or
// counted as misuse.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/core_math.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// An allocator that counts what it hands out and can be told to refuse
// after a number of blocks.
typedef struct Ledger
{
    int64_t blocks;
    int64_t bytes;
    int64_t refuseAfter; // negative: never refuse
    bool misaligned;
} Ledger;

static void* LedgerAlloc(size_t size, size_t alignment, void* context)
{
    Ledger* ledger = (Ledger*)context;
    if (ledger->refuseAfter >= 0 && ledger->blocks >= ledger->refuseAfter)
    {
        return NULL;
    }
    void* memory = malloc(size);
    if (memory != NULL)
    {
        ledger->blocks += 1;
        ledger->bytes += (int64_t)size;
        ledger->misaligned = ledger->misaligned || ((uintptr_t)memory % alignment) != 0;
    }
    return memory;
}

static void LedgerFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)alignment;
    Ledger* ledger = (Ledger*)context;
    ledger->blocks -= 1;
    ledger->bytes -= (int64_t)size;
    free(memory);
}

static void TestCreateWorld(void)
{
    // The checks run in order, and every failure leaves the out pointer
    // null.
    m3World* world = NULL;
    m3WorldDef def = m3DefaultWorldDef();
    CHECK(m3CreateWorld(&def, NULL) == m3_errorInvalid, "a null out pointer is refused");
    CHECK(m3CreateWorld(NULL, &world) == m3_errorInvalid && world == NULL, "a null def is refused");
    m3WorldDef zeroed;
    memset(&zeroed, 0, sizeof(zeroed));
    CHECK(m3CreateWorld(&zeroed, &world) == m3_errorInvalid, "a zeroed def is refused");
    m3WorldDef stale = def;
    stale.version = M3_ABI_VERSION ^ 1u;
    CHECK(m3CreateWorld(&stale, &world) == m3_errorVersion && world == NULL,
          "a def built for another binary interface is refused");
    m3WorldDef unstamped = m3DefaultWorldDefValues();
    CHECK(m3CreateWorld(&unstamped, &world) == m3_errorVersion,
          "the unstamped defaults are refused");
    m3WorldDef empty = def;
    empty.bodyCapacity = 0;
    CHECK(m3CreateWorld(&empty, &world) == m3_errorInvalid, "a zero capacity is refused");
    m3WorldDef half = def;
    half.allocator.alloc = LedgerAlloc;
    CHECK(m3CreateWorld(&half, &world) == m3_errorInvalid, "an allocator without free is refused");

    // Every block goes through the def's allocator and comes back by the
    // size it went out with.
    Ledger ledger = {0, 0, -1, false};
    m3WorldDef counted = def;
    counted.allocator = (m3Allocator){LedgerAlloc, LedgerFree, &ledger};
    CHECK(m3CreateWorld(&counted, &world) == m3_success && world != NULL, "the world creates");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3Counters counters = m3World_GetCounters(world);
    CHECK(ledger.blocks > 0 && counters.allocCalls - counters.freeCalls == ledger.blocks,
          "the counters see every block the allocator handed out");
    CHECK(!ledger.misaligned, "every block met its alignment");
    m3DestroyWorld(world);
    CHECK(ledger.blocks == 0 && ledger.bytes == 0, "destroy returns every block and byte");

    // A refusing allocator fails the create part way, and what it did
    // hand out comes back.
    for (int64_t after = 0; after < 40; after += 7)
    {
        Ledger tight = {0, 0, after, false};
        m3WorldDef starved = def;
        starved.allocator = (m3Allocator){LedgerAlloc, LedgerFree, &tight};
        CHECK(m3CreateWorld(&starved, &world) == m3_errorCapacity && world == NULL,
              "an allocator that runs dry refuses the create");
        CHECK(tight.blocks == 0 && tight.bytes == 0, "and nothing leaks");
    }
    m3DestroyWorld(NULL);
}

static void TestEveryRefusalHasAReason(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId a = NewBody(world, &bd);
    m3BodyId b = NewBody(world, &bd);
    CHECK(m3Body_IsValid(world, a) && m3Body_IsValid(world, b), "two bodies fit");
    m3BodyId third = m3_nullBodyId;
    CHECK(m3CreateBody(world, &bd, &third) == m3_errorCapacity && third.index1 == 0,
          "a third body is refused because the pool is full");

    uint64_t misuse = m3World_GetCounters(world).misuse;
    m3World_SetGravity(world, (m3Vec3){0.0f, NAN, 0.0f});
    CHECK(m3World_GetCounters(world).misuse == misuse + 1, "a NaN gravity counts as misuse");
    CHECK(m3DestroyBody(world, b) == m3_success, "the body destroys");
    CHECK(m3DestroyBody(world, b) == m3_errorStale, "a second destroy finds it stale");
    CHECK(m3DestroyBody(world, m3_nullBodyId) == m3_errorInvalid, "the null id is invalid");
    misuse += 2;
    CHECK(!m3Body_IsValid(world, b), "a validity query on a stale id answers false");
    CHECK(m3World_GetCounters(world).misuse == misuse + 1, "without counting as misuse");
    m3Pos3 gone = m3Body_GetPosition(world, b);
    CHECK(gone.x == 0.0 && gone.y == 0.0 && gone.z == 0.0, "reading through a stale id reads zero");
    CHECK(m3World_GetCounters(world).misuse == misuse + 2, "and counts once, against its world");
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = -1.0f;
    m3ShapeId shape = m3_nullShapeId;
    CHECK(m3CreateBoxShape(world, a, &sd, (m3Vec3){0.5f, 0.5f, 0.5f}, &shape) == m3_errorInvalid &&
              shape.index1 == 0,
          "a negative density is refused");
    CHECK(m3World_GetCounters(world).misuse == misuse + 3, "and counts as misuse");

    // Query inputs are caller data too: non-finite origins, radii and
    // boxes refuse and count instead of walking the broadphase.
    m3ShapeId found[4];
    CHECK(!m3World_CastRayClosest(world, (m3Pos3){(double)NAN, 0.0, 0.0},
                                  (m3Vec3){0.0f, -1.0f, 0.0f}, m3DefaultQueryFilter())
               .hit,
          "a NaN ray origin hits nothing");
    CHECK(!m3World_CastSphereClosest(world, (m3Pos3){0.0, 2.0, 0.0}, NAN,
                                     (m3Vec3){0.0f, -4.0f, 0.0f}, m3DefaultQueryFilter())
               .hit,
          "a NaN sphere radius casts nothing");
    CHECK(m3World_OverlapAabb(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Pos3){(double)INFINITY, 1.0, 1.0},
                              found, 4, m3DefaultQueryFilter()) == 0,
          "an infinite box overlaps nothing");
    CHECK(m3World_OverlapSphere(world, (m3Pos3){0.0, (double)NAN, 0.0}, 1.0f, found, 4,
                                m3DefaultQueryFilter()) == 0,
          "a NaN sphere center overlaps nothing");
    m3World_Step(world, INFINITY, 4);
    CHECK(m3World_GetCounters(world).misuse == misuse + 8, "each refuses once");

    m3DestroyWorld(world);
}

int main(void)
{
    TestCreateWorld();
    TestEveryRefusalHasAReason();
    if (s_failures == 0)
    {
        printf("test_core: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
