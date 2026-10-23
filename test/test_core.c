// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core gate: the step scratch stack (never aborts, loud overflow) and
// the id pool behind every generation-tagged handle (FIFO canonical
// reuse, generation bump on free, retire instead of wrap). White-box:
// includes the internal allocator header.

#include "allocator.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/core_math.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void TestIdsOutliveNoWorld(void)
{
    // A world recycling a slot refuses the ids of the world before it,
    // even when the object slot and generation line up exactly.
    m3WorldDef def = m3DefaultWorldDef();
    m3WorldId first = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId old = m3CreateBody(first, &bd);
    m3DestroyWorld(first);
    m3WorldId second = m3CreateWorld(&def);
    CHECK(second.index1 == first.index1 && second.generation != first.generation,
          "the new world takes the same slot");
    m3BodyId fresh = m3CreateBody(second, &bd);
    CHECK(fresh.index1 == old.index1 && fresh.generation == old.generation,
          "the new body takes the same slot and generation");
    CHECK(!m3Body_IsValid(old), "the old world's id is refused");
    CHECK(m3Body_IsValid(fresh), "the new world's id resolves");
    m3DestroyWorld(second);
}

static int s_handled = 0;
static int HandleAssert(const char* condition, const char* file, int line, void* context)
{
    (void)condition;
    (void)file;
    (void)line;
    *(int*)context += 1;
    return 1; // handled: no abort
}

static void TestAssertHandler(void)
{
    // The hook sees a failed invariant with its context and suppresses
    // the abort. Refusing bad input is not a failed invariant: it
    // records a reason and never reaches the hook.
    s_handled = 0;
    m3SetAssertHandler(HandleAssert, &s_handled);
    m3AssertFail("test", __FILE__, __LINE__);
    CHECK(s_handled == 1, "the handler saw the failure and carried its context");
    m3WorldDef bad = m3DefaultWorldDef();
    bad.internalValue = 0;
    CHECK(!m3World_IsValid(m3CreateWorld(&bad)), "a hand-rolled def is refused");
    CHECK(s_handled == 1, "without an assertion");
    CHECK(m3LastResult() == m3_errorInvalid, "and the refusal carries its reason");
    m3SetAssertHandler(NULL, NULL);
}

static void TestEveryRefusalHasAReason(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId a = m3CreateBody(world, &bd);
    m3BodyId b = m3CreateBody(world, &bd);
    CHECK(m3Body_IsValid(a) && m3Body_IsValid(b), "two bodies fit");
    CHECK(m3CreateBody(world, &bd).index1 == 0, "a third body is refused");
    CHECK(m3LastResult() == m3_errorCapacity, "because the pool is full");

    uint64_t misuse = m3World_GetCounters(world).misuse;
    m3World_SetGravity(world, (m3Vec3){0.0f, NAN, 0.0f});
    CHECK(m3LastResult() == m3_errorInvalid, "a NaN gravity is refused as invalid");
    CHECK(m3World_GetCounters(world).misuse == misuse + 1, "and counts as misuse");
    m3DestroyBody(b);
    CHECK(!m3Body_IsValid(b), "a validity query on a stale id answers false");
    CHECK(m3World_GetCounters(world).misuse == misuse + 1, "without counting as misuse");
    (void)m3Body_GetPosition(b);
    CHECK(m3LastResult() == m3_errorInvalid, "reading through a stale id is refused");
    CHECK(m3World_GetCounters(world).misuse == misuse + 2, "and counts once, against its world");
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = -1.0f;
    CHECK(m3CreateBoxShape(a, &sd, (m3Vec3){0.5f, 0.5f, 0.5f}).index1 == 0,
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
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3LastResult() == m3_errorInvalid, "stepping a destroyed world is refused");
}

int main(void)
{
    TestAssertHandler();
    TestEveryRefusalHasAReason();
    TestIdsOutliveNoWorld();
    if (s_failures == 0)
    {
        printf("test_core: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
