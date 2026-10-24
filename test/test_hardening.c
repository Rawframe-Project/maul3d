// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hardening: the capacity sweeps (every pool exhausted on purpose;
// every refusal loud and clean, the world never corrupted), an
// allocator that runs dry, and restores fed poisoned bytes.

#include "test_harness.h"

#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void TestCapacityExhaustion(void)
{
    // Every pool runs dry ON PURPOSE; every refusal is a null id and
    // the world keeps stepping, hash-stable, never corrupted.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.meshCapacity = 1;
    def.jointCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody; // joints below need dynamics
    m3BodyId bodies[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        bodies[i] = m3CreateBody(world, &bd);
        CHECK(m3Body_IsValid(bodies[i]), "bodies up to capacity create");
    }
    m3BodyId extra = m3CreateBody(world, &bd);
    CHECK(!m3Body_IsValid(extra), "the body pool refuses past capacity");

    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    for (int32_t i = 0; i < 4; ++i)
    {
        CHECK(m3Shape_IsValid(m3CreateSphereShape(bodies[i % 4], &sd, &ball)),
              "shapes up to capacity create");
    }
    CHECK(!m3Shape_IsValid(m3CreateSphereShape(bodies[0], &sd, &ball)),
          "the shape pool refuses past capacity");

    // The joint pool: fill, refuse, recycle, and the invalid defs
    // that must never mint an id in the first place.
    m3JointDef pin = m3DefaultJointDef();
    pin.type = m3_sphericalJoint;
    pin.bodyIdA = bodies[0];
    pin.bodyIdB = bodies[1];
    m3JointId j1 = m3CreateJoint(world, &pin);
    CHECK(m3Joint_IsValid(j1), "joints up to capacity create");
    pin.bodyIdA = bodies[1];
    pin.bodyIdB = bodies[2];
    m3JointId j2 = m3CreateJoint(world, &pin);
    CHECK(m3Joint_IsValid(j2), "the second joint fills the pool");
    pin.bodyIdA = bodies[2];
    pin.bodyIdB = bodies[3];
    CHECK(!m3Joint_IsValid(m3CreateJoint(world, &pin)), "the joint pool refuses past capacity");
    m3DestroyJoint(j2);
    m3JointId j3 = m3CreateJoint(world, &pin);
    CHECK(m3Joint_IsValid(j3), "a freed joint slot recycles");
    CHECK(!m3Joint_IsValid(j2), "the destroyed joint id is stale");

    pin.bodyIdA = bodies[0];
    pin.bodyIdB = bodies[0];
    CHECK(!m3Joint_IsValid(m3CreateJoint(world, &pin)), "a self joint refuses");
    m3JointDef bad = m3DefaultJointDef();
    bad.type = m3_revoluteJoint;
    bad.bodyIdA = bodies[0];
    bad.bodyIdB = bodies[1];
    bad.localAxisA = (m3Vec3){0.0f, 0.0f, 0.0f};
    CHECK(!m3Joint_IsValid(m3CreateJoint(world, &bad)), "a hinge with no axis refuses");

    m3DestroyWorld(world);

    // Mesh slots: one fills, the second refuses, and the failed
    // create leaves no half-registered shape behind.
    m3WorldDef md = m3DefaultWorldDef();
    md.bodyCapacity = 4;
    md.shapeCapacity = 8;
    md.meshCapacity = 1;
    m3WorldId mworld = m3CreateWorld(&md);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(mworld, &gd);
    m3ShapeDef msd = m3DefaultShapeDef();
    m3Vec3 verts[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    uint16_t tris[3] = {0, 2, 1};
    CHECK(m3Shape_IsValid(m3CreateMeshShape(ground, &msd, verts, 3, tris, 1)),
          "the single mesh slot fills");
    CHECK(!m3Shape_IsValid(m3CreateMeshShape(ground, &msd, verts, 3, tris, 1)),
          "the mesh pool refuses past capacity");
    m3World_Step(mworld, 1.0f / 60.0f, 4);
    uint64_t hashAfterRefusal = m3World_Hash(mworld);

    // A refused create legitimately advances pool identity (the slot
    // was minted and retired; identity IS state). The property that
    // matters is that the FAILURE PATH is deterministic: a twin world
    // walking the same refusals lands on the identical hash.
    m3WorldId twin = m3CreateWorld(&md);
    m3BodyId tground = m3CreateBody(twin, &gd);
    CHECK(m3Shape_IsValid(m3CreateMeshShape(tground, &msd, verts, 3, tris, 1)),
          "the twin's mesh slot fills");
    CHECK(!m3Shape_IsValid(m3CreateMeshShape(tground, &msd, verts, 3, tris, 1)),
          "the twin's refusal repeats");
    m3World_Step(twin, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(twin) == hashAfterRefusal, "the failure path is bit-deterministic");
    m3DestroyWorld(twin);
    m3DestroyWorld(mworld);
}

static void TestJournalOverflowIsLoud(void)
{
    // A journal too small for the session latches overflow, stops
    // recording, and End reports -1: the caller can never mistake a
    // truncated recording for a good one.
    uint8_t tiny[64];
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_StartJournal(world, tiny, (int32_t)sizeof(tiny)), "the tiny journal arms");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t i = 0; i < 4; ++i)
    {
        m3CreateBody(world, &bd); // each op needs more than 64 bytes
    }
    CHECK(m3World_StopJournal(world) == -1, "journal overflow reports -1, loudly");
    m3World_Step(world, 1.0f / 60.0f, 4); // the world itself is fine
    m3DestroyWorld(world);
}

// A host allocator that refuses once a budget of successful calls is
// spent, and counts live blocks so a leak shows up as a nonzero total.
typedef struct FailingAllocator
{
    int32_t budget;
    int32_t live;
} FailingAllocator;

static void* FailingAlloc(size_t bytes, void* context)
{
    FailingAllocator* fa = (FailingAllocator*)context;
    if (fa->budget <= 0)
    {
        return NULL;
    }
    fa->budget -= 1;
    void* memory = malloc(bytes);
    if (memory != NULL)
    {
        fa->live += 1;
    }
    return memory;
}

static void FailingFree(void* memory, void* context)
{
    FailingAllocator* fa = (FailingAllocator*)context;
    if (memory != NULL)
    {
        fa->live -= 1;
        free(memory);
    }
}

static void TestCreateWorldOutOfMemory(void)
{
    // Refuse the first, second, third... allocation of world creation
    // in turn. Every attempt must either refuse cleanly with a null id
    // and nothing left allocated, or succeed and destroy cleanly.
    m3WorldDef def = m3DefaultWorldDef();
    FailingAllocator fa = {0, 0};
    int32_t refusals = 0;
    bool created = false;
    for (int32_t budget = 0; budget < 4096 && !created; ++budget)
    {
        fa.budget = budget;
        fa.live = 0;
        m3SetAllocator(FailingAlloc, FailingFree, &fa);
        m3WorldId world = m3CreateWorld(&def);
        if (m3World_IsValid(world))
        {
            created = true;
            m3DestroyWorld(world);
        }
        else
        {
            refusals += 1;
        }
        m3SetAllocator(NULL, NULL, NULL);
        if (fa.live != 0)
        {
            printf("FAIL: %d block(s) leaked with an allocation budget of %d\n", fa.live, budget);
            s_failures += 1;
            break;
        }
    }
    CHECK(created, "world creation succeeds once the allocator stops refusing");
    CHECK(refusals > 100, "every allocation of world creation was refused once");

    // Capacities whose derived byte sizes cannot be represented are
    // refused up front instead of overflowing.
    m3WorldDef huge = m3DefaultWorldDef();
    huge.shapeCapacity = INT32_MAX / 4;
    CHECK(!m3World_IsValid(m3CreateWorld(&huge)), "an unrepresentable shape capacity is refused");
    huge = m3DefaultWorldDef();
    huge.softBodyCapacity = INT32_MAX / 1024;
    CHECK(!m3World_IsValid(m3CreateWorld(&huge)),
          "an unrepresentable soft body capacity is refused");
}

static m3WorldId CursorWorld(int32_t bodies)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    for (int32_t i = 0; i < bodies; ++i)
    {
        bd.position = (m3Pos3){2.0 * i, 1.0, 0.0};
        m3CreateSphereShape(m3CreateBody(world, &bd), &sd, &sphere);
    }
    return world;
}

static void TestRestoreRefusesBadCursors(void)
{
    // The body and shape pool cursors are the words that read 1 with one
    // body and 2 with two. Pointed past their pools, they are refused
    // before anything is overwritten.
    m3WorldId world = CursorWorld(1);
    m3WorldId twin = CursorWorld(2);
    int32_t size = m3World_SnapshotSize(world);
    CHECK(m3World_SnapshotSize(twin) == size, "one world shape, one snapshot size");
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    uint8_t* other = (uint8_t*)malloc((size_t)size);
    CHECK(m3World_Snapshot(world, snap, size) == size, "snapshot taken");
    CHECK(m3World_Snapshot(twin, other, size) == size, "twin snapshot taken");
    m3World_Step(world, 1.0f / 60.0f, 4);
    uint64_t before = m3World_Hash(world);

    int32_t poisoned = 0;
    int32_t hostile = 1 << 30;
    for (int32_t at = 0; at + 4 <= size; ++at) // the stream is unaligned
    {
        int32_t one;
        int32_t two;
        memcpy(&one, snap + at, sizeof(one));
        memcpy(&two, other + at, sizeof(two));
        if (one == 1 && two == 2)
        {
            memcpy(snap + at, &hostile, sizeof(hostile));
            poisoned += 1;
        }
    }
    CHECK(poisoned >= 2, "the cursors are found");
    CHECK(!m3World_Restore(world, snap, size), "out-of-range cursors are refused");
    CHECK(m3LastResult() == m3_errorInvalid, "as invalid input");
    CHECK(m3World_Hash(world) == before, "and the world is untouched");

    free(other);
    free(snap);
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

// Restore checks every incoming block before any of it lands: poison
// each 32-bit word of a snapshot in turn with a huge value. A refused
// restore leaves the world untouched and records a reason.
static void TestRestoreRefusesPoisonedWords(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.jointCapacity = 2;
    def.meshCapacity = 1;
    def.voxelCapacity = 1;
    def.characterCapacity = 1;
    def.vehicleCapacity = 1;
    def.softBodyCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(ground, &sd, (m3Vec3){5.0f, 0.5f, 5.0f});
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.2, 0.0};
    m3BodyId box = m3CreateBody(world, &bd);
    m3CreateBoxShape(box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3World_Step(world, 1.0f / 60.0f, 4);

    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    uint8_t* poisoned = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && poisoned != NULL, "snapshot buffers");
    if (clean == NULL || poisoned == NULL)
    {
        free(clean);
        free(poisoned);
        return;
    }
    m3World_Snapshot(world, clean, size);
    uint64_t cleanHash = m3World_Hash(world);
    int32_t refused = 0;
    int32_t untouched = 0;
    int32_t reasoned = 0;
    for (int32_t at = 0; at + 4 <= size; ++at) // the stream is unaligned
    {
        memcpy(poisoned, clean, (size_t)size);
        int32_t huge = 0x7FFFFFFF;
        memcpy(poisoned + at, &huge, sizeof(huge));
        if (!m3World_Restore(world, poisoned, size))
        {
            refused += 1;
            untouched += m3World_Hash(world) == cleanHash ? 1 : 0;
            reasoned += m3LastResult() != m3_success ? 1 : 0;
            continue;
        }
        m3World_Restore(world, clean, size);
    }
    CHECK(refused > 0, "hostile words are refused");
    CHECK(untouched == refused, "a refused restore leaves the world as it was");
    CHECK(reasoned == refused, "and records why");
    free(clean);
    free(poisoned);
    m3DestroyWorld(world);
}

// Every shape constructor checks the def's materials before it builds
// anything: a poisoned def is invalid input, never a capacity refusal.
static void TestShapeDefsRefuseAsInvalid(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = NAN;
    static const m3Vec3 tri[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    static const uint16_t idx[3] = {0, 1, 2};
    CHECK(!m3Shape_IsValid(m3CreateMeshShape(ground, &sd, tri, 3, idx, 1)),
          "a mesh with a NaN density refuses");
    CHECK(m3LastResult() == m3_errorInvalid, "as invalid input");
    CHECK(!m3Shape_IsValid(m3CreateHullShape(ground, &sd, tri, 3)),
          "a hull with a NaN density refuses");
    CHECK(m3LastResult() == m3_errorInvalid, "as invalid input too");
    static const float heights[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    CHECK(!m3Shape_IsValid(m3CreateHeightFieldGridShape(ground, &sd, heights, 2, 2, 1.0f)),
          "a height field with a NaN density refuses");
    CHECK(m3LastResult() == m3_errorInvalid, "as invalid input as well");
    sd = m3DefaultShapeDef();
    m3Sphere bad = {{0.0f, 0.0f, 0.0f}, 0.5f};
    sd.friction = -1.0f;
    CHECK(!m3Shape_IsValid(m3CreateSphereShape(ground, &sd, &bad)),
          "a sphere with negative friction refuses");
    CHECK(m3LastResult() == m3_errorInvalid, "and says why");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRestoreRefusesPoisonedWords();
    TestRestoreRefusesBadCursors();
    TestCreateWorldOutOfMemory();
    TestCapacityExhaustion();
    TestJournalOverflowIsLoud();
    TestShapeDefsRefuseAsInvalid();
    if (s_failures == 0)
    {
        printf("test_hardening: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
