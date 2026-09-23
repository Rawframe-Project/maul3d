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

static void TestStack(void)
{
    m3Stack stack = m3StackCreate(256);
    CHECK(stack.capacity == 256, "stack capacity");

    void* a = m3StackAlloc(&stack, 10);
    void* b = m3StackAlloc(&stack, 20);
    CHECK(a != NULL && b != NULL, "allocations succeed");
    CHECK(((uintptr_t)a & 15u) == 0 && ((uintptr_t)b & 15u) == 0, "16-byte aligned");
    CHECK((uint8_t*)b - (uint8_t*)a == 16, "10 bytes rounds up to one 16-byte block");

    // Overflow is loud but survivable: NULL plus a latched flag, and
    // the stack keeps serving what still fits.
    void* big = m3StackAlloc(&stack, 512);
    CHECK(big == NULL, "overflow returns NULL, never aborts");
    CHECK(stack.overflow == 1, "overflow is latched for the step to read");
    void* still = m3StackAlloc(&stack, 16);
    CHECK(still != NULL, "the stack keeps working after an overflow");

    // Reset starts the next step clean and reuses the same memory.
    m3StackReset(&stack);
    CHECK(stack.overflow == 0 && stack.top == 0, "reset clears top and flag");
    void* again = m3StackAlloc(&stack, 10);
    CHECK(again == a, "reset reuses the same base");

    m3StackDestroy(&stack);
    CHECK(stack.base == NULL, "destroy clears the struct");
}

static void TestIdPool(void)
{
    m3IdPool pool = m3IdPoolCreate(4);

    int32_t i0 = m3IdPoolAlloc(&pool);
    int32_t i1 = m3IdPoolAlloc(&pool);
    int32_t i2 = m3IdPoolAlloc(&pool);
    CHECK(i0 == 0 && i1 == 1 && i2 == 2, "fresh slots come in order");
    CHECK(m3IdPoolValid(&pool, i1, 0), "a live slot at generation 0 validates");

    // Free bumps the generation: the old handle must die.
    m3IdPoolFree(&pool, i1);
    CHECK(!m3IdPoolValid(&pool, i1, 0), "a stale generation is rejected");

    // FIFO reuse: free two, get them back in the order they were freed.
    m3IdPoolFree(&pool, i0);
    int32_t r0 = m3IdPoolAlloc(&pool);
    int32_t r1 = m3IdPoolAlloc(&pool);
    CHECK(r0 == i1 && r1 == i0, "recycling is FIFO, the canonical order");
    CHECK(m3IdPoolValid(&pool, r0, 1), "the recycled slot validates at its new generation");

    // Exhaustion is a -1, not a hidden growth.
    int32_t i3 = m3IdPoolAlloc(&pool);
    CHECK(i3 == 3, "last fresh slot");
    CHECK(m3IdPoolAlloc(&pool) == -1, "an exhausted pool fails loudly");

    m3IdPoolDestroy(&pool);
}

static void TestIdShapes(void)
{
    // The null id is all zero, and the structs stay padding-free sizes
    // the snapshot can rely on.
    m3BodyId null = {0, 0, 0};
    CHECK(null.index1 == 0, "null id");
    CHECK(sizeof(m3WorldId) == 8 && sizeof(m3BodyId) == 8 && sizeof(m3ShapeId) == 8,
          "ids are 8 bytes");
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

    m3DestroyWorld(world);
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3LastResult() == m3_errorInvalid, "stepping a destroyed world is refused");
}

int main(void)
{
    TestAssertHandler();
    TestEveryRefusalHasAReason();
    TestStack();
    TestIdPool();
    TestIdShapes();
    if (s_failures == 0)
    {
        printf("test_core: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
