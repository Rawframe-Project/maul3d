// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hardening: the capacity sweeps (every pool exhausted on purpose;
// every refusal loud and clean, the world never corrupted), an
// allocator that runs dry, and restores fed poisoned bytes.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/joint.h"
#include "maul3d/replay.h"
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
    m3World* world = NewWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody; // joints below need dynamics
    m3BodyId bodies[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        bodies[i] = NewBody(world, &bd);
        CHECK(m3Body_IsValid(world, bodies[i]), "bodies up to capacity create");
    }
    m3BodyId extra = NewBody(world, &bd);
    CHECK(!m3Body_IsValid(world, extra), "the body pool refuses past capacity");

    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    for (int32_t i = 0; i < 4; ++i)
    {
        CHECK(m3Shape_IsValid(world, NewSphereShape(world, bodies[i % 4], &sd, &ball)),
              "shapes up to capacity create");
    }
    CHECK(!m3Shape_IsValid(world, NewSphereShape(world, bodies[0], &sd, &ball)),
          "the shape pool refuses past capacity");

    // The joint pool: fill, refuse, recycle, and the invalid defs
    // that must never mint an id in the first place.
    m3JointDef pin = m3DefaultJointDef();
    pin.type = m3_sphericalJoint;
    pin.bodyIdA = bodies[0];
    pin.bodyIdB = bodies[1];
    m3JointId j1 = NewJoint(world, &pin);
    CHECK(m3Joint_IsValid(world, j1), "joints up to capacity create");
    pin.bodyIdA = bodies[1];
    pin.bodyIdB = bodies[2];
    m3JointId j2 = NewJoint(world, &pin);
    CHECK(m3Joint_IsValid(world, j2), "the second joint fills the pool");
    pin.bodyIdA = bodies[2];
    pin.bodyIdB = bodies[3];
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &pin)), "the joint pool refuses past capacity");
    DestroyJoint(world, j2);
    m3JointId j3 = NewJoint(world, &pin);
    CHECK(m3Joint_IsValid(world, j3), "a freed joint slot recycles");
    CHECK(!m3Joint_IsValid(world, j2), "the destroyed joint id is stale");

    pin.bodyIdA = bodies[0];
    pin.bodyIdB = bodies[0];
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &pin)), "a self joint refuses");
    m3JointDef bad = m3DefaultJointDef();
    bad.type = m3_revoluteJoint;
    bad.bodyIdA = bodies[0];
    bad.bodyIdB = bodies[1];
    bad.localAxisA = (m3Vec3){0.0f, 0.0f, 0.0f};
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &bad)), "a hinge with no axis refuses");

    m3DestroyWorld(world);

    // Mesh slots: one fills, the second refuses, and the failed
    // create leaves no half-registered shape behind.
    m3WorldDef md = m3DefaultWorldDef();
    md.bodyCapacity = 4;
    md.shapeCapacity = 8;
    md.meshCapacity = 1;
    m3World* mworld = NewWorld(&md);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(mworld, &gd);
    m3ShapeDef msd = m3DefaultShapeDef();
    m3Vec3 verts[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    uint16_t tris[3] = {0, 2, 1};
    CHECK(m3Shape_IsValid(mworld, NewMeshShape(mworld, ground, &msd, verts, 3, tris, 1)),
          "the single mesh slot fills");
    CHECK(!m3Shape_IsValid(mworld, NewMeshShape(mworld, ground, &msd, verts, 3, tris, 1)),
          "the mesh pool refuses past capacity");
    m3World_Step(mworld, 1.0f / 60.0f, 4);
    uint64_t hashAfterRefusal = m3World_Hash(mworld);

    // A refused create legitimately advances pool identity (the slot
    // was minted and retired; identity IS state). The property that
    // matters is that the FAILURE PATH is deterministic: a twin world
    // walking the same refusals lands on the identical hash.
    m3World* twin = NewWorld(&md);
    m3BodyId tground = NewBody(twin, &gd);
    CHECK(m3Shape_IsValid(twin, NewMeshShape(twin, tground, &msd, verts, 3, tris, 1)),
          "the twin's mesh slot fills");
    CHECK(!m3Shape_IsValid(twin, NewMeshShape(twin, tground, &msd, verts, 3, tris, 1)),
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
    m3World* world = NewWorld(&def);
    CHECK(StartJournal(world, tiny, (int32_t)sizeof(tiny)), "the tiny journal arms");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t i = 0; i < 4; ++i)
    {
        NewBody(world, &bd); // each op needs more than 64 bytes
    }
    CHECK(StopJournal(world) == -1, "journal overflow reports -1, loudly");
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

static void* FailingAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
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

static void FailingFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
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
        def.allocator = (m3Allocator){FailingAlloc, FailingFree, &fa};
        m3World* world = NULL;
        m3Result result = m3CreateWorld(&def, &world);
        if (result == m3_success)
        {
            created = true;
            m3DestroyWorld(world);
        }
        else
        {
            refusals += 1;
            CHECK(result == m3_errorCapacity && world == NULL,
                  "a refused allocation is a capacity refusal");
        }
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
    m3World* refused = NULL;
    CHECK(m3CreateWorld(&huge, &refused) == m3_errorLimit,
          "a shape capacity past M3_MAX_SHAPE_CAPACITY is refused");
    huge = m3DefaultWorldDef();
    huge.softBodyCapacity = INT32_MAX / 1024;
    CHECK(m3CreateWorld(&huge, &refused) != m3_success && refused == NULL,
          "an unrepresentable soft body capacity is refused");
}

static m3World* CursorWorld(int32_t bodies)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    for (int32_t i = 0; i < bodies; ++i)
    {
        bd.position = (m3Pos3){2.0 * i, 1.0, 0.0};
        NewSphereShape(world, NewBody(world, &bd), &sd, &sphere);
    }
    return world;
}

// Finds a byte pattern in a buffer: its offset, or -1.
static int32_t FindBytes(const uint8_t* buffer, int32_t size, const void* pattern, int32_t bytes)
{
    for (int32_t at = 0; at + bytes <= size; ++at)
    {
        if (memcmp(buffer + at, pattern, (size_t)bytes) == 0)
        {
            return at;
        }
    }
    return -1;
}

// A restore lands mesh content only when it is content a mesh create
// and m3Shape_SetMeshMaterials would take: a triangle index past the
// vertices, a vertex that is not finite, a material the paint wall
// refuses or a group past the material count is refused as invalid
// before the BVH build or a contact reads it.
static void TestRestoreRefusesBadMeshContent(void)
{
    static const m3Vec3 verts[4] = {
        {-3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, 3.0f}, {-3.0f, 0.0f, 3.0f}};
    static const uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId floor = NewMeshShape(world, NewBody(world, &bd), &sd, verts, 4, tris, 2);
    m3MeshSurfaceMaterial paint[2];
    memset(paint, 0, sizeof(paint));
    paint[0].friction = 0.37f;
    paint[1].friction = 0.61f;
    uint8_t groups[2] = {0, 1};
    m3Shape_SetMeshMaterials(world, floor, paint, 2, groups);
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    uint8_t* poisoned = (uint8_t*)malloc((size_t)size);
    uint8_t* after = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && poisoned != NULL && after != NULL &&
              Snapshot(world, clean, size) == size,
          "the mesh world is snapshotted");
    int32_t indicesAt = FindBytes(clean, size, tris, (int32_t)sizeof(tris));
    int32_t vertexAt = FindBytes(clean, size, &verts[1], (int32_t)sizeof(m3Vec3));
    int32_t paintAt = FindBytes(clean, size, paint, (int32_t)sizeof(paint));
    // The group bytes follow the eight-entry material table.
    int32_t groupsAt = paintAt + 8 * (int32_t)sizeof(m3MeshSurfaceMaterial);
    CHECK(indicesAt >= 0 && vertexAt >= 0 && paintAt >= 0 && groupsAt + 2 <= size &&
              memcmp(clean + groupsAt, groups, sizeof(groups)) == 0,
          "the mesh content is found");

    memcpy(poisoned, clean, (size_t)size);
    uint16_t past = 200;
    memcpy(poisoned + indicesAt + 2, &past, sizeof(past));
    CHECK(m3World_Restore(world, poisoned, size) == m3_errorFormat,
          "an index past the vertices is refused as invalid");
    CHECK(Snapshot(world, after, size) == size && memcmp(after, clean, (size_t)size) == 0,
          "and the world is untouched");

    memcpy(poisoned, clean, (size_t)size);
    float nan = NAN;
    memcpy(poisoned + vertexAt, &nan, sizeof(nan));
    CHECK(m3World_Restore(world, poisoned, size) == m3_errorFormat,
          "a vertex that is not finite is refused as invalid");
    CHECK(Snapshot(world, after, size) == size && memcmp(after, clean, (size_t)size) == 0,
          "and the world is untouched again");

    struct
    {
        int32_t at;
        float friction;
        uint8_t group;
        const char* msg;
    } kPaint[3] = {{paintAt, NAN, 0, "a material with a NaN friction is refused as invalid"},
                   {paintAt, -1.0f, 0, "a material with a negative friction is refused as invalid"},
                   {groupsAt, 0.0f, 2, "a group past the material count is refused as invalid"}};
    for (int32_t i = 0; i < 3; ++i)
    {
        memcpy(poisoned, clean, (size_t)size);
        if (kPaint[i].at == paintAt)
        {
            memcpy(poisoned + paintAt, &kPaint[i].friction, sizeof(float));
        }
        else
        {
            poisoned[groupsAt + 1] = kPaint[i].group;
        }
        CHECK(m3World_Restore(world, poisoned, size) == m3_errorFormat, kPaint[i].msg);
        CHECK(Snapshot(world, after, size) == size && memcmp(after, clean, (size_t)size) == 0,
              "and the paint is untouched");
    }
    free(after);
    free(poisoned);
    free(clean);
    m3DestroyWorld(world);
}

// One poisoning of a snapshot: write bytes at an offset.
typedef struct Poison
{
    int32_t at;
    uint8_t bytes[8];
    int32_t count;
    const char* msg;
} Poison;

// Restores each poisoned copy of a clean snapshot: each must be refused
// as invalid with the world's bytes unchanged. The world then steps, so
// code that took the poison would read it, and goes back to clean.
static void CheckPoisonsRefused(m3World* world, const uint8_t* clean, int32_t size,
                                const Poison* poisons, int32_t poisonCount)
{
    uint8_t* poisoned = (uint8_t*)malloc((size_t)size);
    uint8_t* after = (uint8_t*)malloc((size_t)size);
    CHECK(poisoned != NULL && after != NULL, "poison buffers");
    for (int32_t i = 0; poisoned != NULL && after != NULL && i < poisonCount; ++i)
    {
        memcpy(poisoned, clean, (size_t)size);
        memcpy(poisoned + poisons[i].at, poisons[i].bytes, (size_t)poisons[i].count);
        CHECK(m3World_Restore(world, poisoned, size) == m3_errorFormat, poisons[i].msg);
        CHECK(Snapshot(world, after, size) == size && memcmp(after, clean, (size_t)size) == 0,
              "and the world is untouched");
        m3World_Step(world, 1.0f / 60.0f, 4);
        CHECK(Restore(world, clean, size), "the clean snapshot restores");
    }
    free(after);
    free(poisoned);
}

static Poison PoisonU16(int32_t at, uint16_t value, const char* msg)
{
    Poison p = {at, {0}, 2, msg};
    memcpy(p.bytes, &value, sizeof(value));
    return p;
}

static Poison PoisonF32(int32_t at, float value, const char* msg)
{
    Poison p = {at, {0}, 4, msg};
    memcpy(p.bytes, &value, sizeof(value));
    return p;
}

// A restore lands hull content only when it is a hull the builders
// make: a half edge's twin or next, a face's start in the index list, a
// face's vertex index or a vertex that the builders would never write is
// refused as invalid before collision reads it.
static void TestRestoreRefusesBadHullContent(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){5.0f, 0.5f, 5.0f});
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.91, 0.0}; // resting on the floor, face to face
    const m3Vec3 half = {0.37f, 0.41f, 0.53f};
    NewBoxShape(world, NewBody(world, &bd), &sd, half);
    m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && Snapshot(world, clean, size) == size, "the hull world is snapshotted");

    // A box's first two vertices are (-hx, -hy, -hz) and (hx, -hy, -hz),
    // a pair no contact holds; after the vertices come the 6 face normals
    // and offsets, the counts, the starts, the 24 indices and the 24 half
    // edges (twin, next, origin, face).
    const m3Vec3 first[2] = {{-half.x, -half.y, -half.z}, {half.x, -half.y, -half.z}};
    int32_t vertsAt = clean != NULL ? FindBytes(clean, size, first, (int32_t)sizeof(first)) : -1;
    int32_t startsAt = vertsAt + 8 * 12 + 6 * 12 + 6 * 4 + 6;
    int32_t indicesAt = startsAt + 6 * 2;
    int32_t edgesAt = indicesAt + 24;
    CHECK(vertsAt >= 0 && edgesAt + 24 * 6 <= size, "the hull content is found");
    if (vertsAt >= 0 && edgesAt + 24 * 6 <= size)
    {
        Poison poisons[9];
        int32_t n = 0;
        poisons[n++] = PoisonU16(edgesAt, 999, "a twin past the half edges is refused as invalid");
        poisons[n++] = PoisonU16(edgesAt, 2, "a twin that is not the edge's pair is refused");
        poisons[n++] =
            PoisonU16(edgesAt + 2, 999, "a next past the half edges is refused as invalid");
        for (int32_t f = 0; f < 4; ++f)
        {
            poisons[n++] = PoisonU16(startsAt + 2 * f, 60000,
                                     "a face start past the index list is refused as invalid");
        }
        poisons[n] = PoisonU16(indicesAt, 0, "a face vertex past the hull is refused as invalid");
        poisons[n++].bytes[0] = 200;
        poisons[n++] = PoisonF32(vertsAt, NAN, "a vertex that is not finite is refused as invalid");
        CheckPoisonsRefused(world, clean, size, poisons, n);
    }
    free(clean);
    m3DestroyWorld(world);
}

static uint64_t NextRandom(uint64_t* state)
{
    *state = *state * 6364136223846793005ull + 1442695040888963407ull;
    return *state >> 33;
}

static float RandomIn(uint64_t* state, float lo, float hi)
{
    return lo + (hi - lo) * (float)NextRandom(state) / 2147483648.0f;
}

// A point cloud of one kind: a blob, a small rock, a slab, a needle, four
// tight or loose clusters of near-duplicate points, or a large blob.
static int32_t HullCloud(uint64_t* rng, int32_t kind, m3Vec3* pts)
{
    static const m3Vec3 corners[4] = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {-1.0f, -1.0f, -1.0f}};
    int32_t count = 8 + (int32_t)(NextRandom(rng) % 17);
    for (int32_t i = 0; i < count; ++i)
    {
        m3Vec3 p = {RandomIn(rng, -1.0f, 1.0f), RandomIn(rng, -1.0f, 1.0f),
                    RandomIn(rng, -1.0f, 1.0f)};
        switch (kind)
        {
        case 1:
            p = m3MulSV3(0.3f, p);
            break;
        case 2:
            p.y *= 1.0e-3f;
            break;
        case 3:
            p.y *= 1.0e-3f;
            p.z *= 1.0e-3f;
            break;
        case 4:
            p = m3Add3(corners[i % 4], m3MulSV3(1.0e-4f, p));
            break;
        case 5:
            p = m3Add3(corners[i % 4], m3MulSV3(1.0e-2f, p));
            break;
        case 6:
            p = m3MulSV3(9.0e3f, p);
            break;
        default:
            break;
        }
        pts[i] = p;
    }
    return count;
}

// A restore accepts every hull a create makes: hulls from every kind of
// cloud, near-degenerate ones included, created, snapshotted and
// restored. Every cloud creates, tight clusters of near-duplicate points
// too: QuickHull welds them and its planes hold to the wall.
static void TestRestoreAcceptsEveryCreatedHull(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 800;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    uint64_t rng = 0xC10D5ull;
    int32_t created = 0;
    for (int32_t kind = 0; kind < 7; ++kind)
    {
        for (int32_t round = 0; round < 100; ++round)
        {
            m3Vec3 pts[24];
            int32_t count = HullCloud(&rng, kind, pts);
            m3ShapeId shape = m3_nullShapeId;
            m3Result result = m3CreateHullShape(world, ground, &sd, pts, count, &shape);
            created += result == m3_success ? 1 : 0;
        }
    }
    printf("M3_HULL_RESTORE created=%d of 700\n", created);
    CHECK(created == 700, "every cloud creates, tight clusters included");
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "the hull world is snapshotted");
    CHECK(snap != NULL && Restore(world, snap, size), "every created hull restores");
    free(snap);
    m3DestroyWorld(world);
}

// A small hull far from its body's origin creates and restores: the
// floats there are coarse (an ulp is about 1e-3 at 9e3), and the wall's
// plane slack takes their rounding as well as its share of the extent.
// Offsets run along an axis and a diagonal up to the local limit, sizes
// from a centimetre to a metre.
static void TestFarHullsCreate(void)
{
    static const float offsets[7] = {0.0f, 1.0e1f, 1.0e2f, 1.0e3f, 3.0e3f, 9.0e3f, 9.9e3f};
    static const float sizes[3] = {1.0e-2f, 1.0e-1f, 1.0f};
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 900;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    uint64_t rng = 0xFA2ull;
    int32_t refused = 0;
    for (int32_t i = 0; i < 7 * 2 * 3 * 20; ++i)
    {
        float offset = offsets[i / 120];
        float size = sizes[i % 3];
        m3Vec3 center = (i / 60) % 2 == 0 ? (m3Vec3){offset, 0.0f, 0.0f}
                                          : m3MulSV3(offset * 0.57735f, (m3Vec3){1.0f, 1.0f, 1.0f});
        m3Vec3 pts[16];
        for (int32_t k = 0; k < 16; ++k)
        {
            m3Vec3 p = {RandomIn(&rng, -0.5f, 0.5f), RandomIn(&rng, -0.5f, 0.5f),
                        RandomIn(&rng, -0.5f, 0.5f)};
            pts[k] = m3Add3(center, m3MulSV3(size, p));
        }
        m3ShapeId shape = m3_nullShapeId;
        if (m3CreateHullShape(world, ground, &sd, pts, 16, &shape) != m3_success)
        {
            refused += 1;
            if (refused == 1)
            {
                printf("M3_FAR_HULL refused offset=%g size=%g\n", (double)offset, (double)size);
            }
        }
    }
    printf("M3_FAR_HULL refused=%d of 840\n", refused);
    CHECK(refused == 0, "a small hull far from its body's origin creates");
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "the far hulls are snapshotted");
    CHECK(snap != NULL && Restore(world, snap, size), "every far hull restores");
    free(snap);
    m3DestroyWorld(world);
}

// A restore lands a hull's planes and mass properties only as the
// builders make them: a plane that misplaces its face or leaves a vertex
// outside, a NaN in a plane, mass properties the vertices do not give,
// or a vertex past the local limit is refused as invalid.
static void TestRestoreRefusesBadHullDerived(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){5.0f, 0.5f, 5.0f});
    // A perturbed cube: every point is a hull vertex, in input order, and
    // the hull is no box, so its planes and mass come from QuickHull.
    const m3Vec3 rock[8] = {{-0.31f, -0.29f, -0.33f}, {0.32f, -0.30f, -0.28f},
                            {0.30f, 0.33f, -0.31f},   {-0.28f, 0.31f, -0.30f},
                            {-0.33f, -0.28f, 0.29f},  {0.29f, -0.31f, 0.32f},
                            {0.31f, 0.30f, 0.33f},    {-0.30f, 0.32f, 0.28f}};
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.85, 0.0};
    NewHullShape(world, NewBody(world, &bd), &sd, rock, 8);
    m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && Snapshot(world, clean, size) == size, "the rock world is snapshotted");
    // Before the vertices come the four counts, the unit mass, the
    // centroid, the inertia and the center; after them the face normals
    // and offsets.
    int32_t vertsAt =
        clean != NULL ? FindBytes(clean, size, rock, 2 * (int32_t)sizeof(m3Vec3)) : -1;
    int32_t vc = 0;
    int32_t fc = 0;
    if (vertsAt >= 80)
    {
        memcpy(&vc, clean + vertsAt - 80, 4);
        memcpy(&fc, clean + vertsAt - 76, 4);
    }
    int32_t normalsAt = vertsAt + vc * 12;
    int32_t offsetsAt = normalsAt + fc * 12;
    CHECK(vertsAt >= 80 && vc == 8 && fc >= 6 && offsetsAt + fc * 4 <= size,
          "the rock's content is found");
    if (vertsAt >= 80 && vc == 8 && fc >= 6 && offsetsAt + fc * 4 <= size)
    {
        float n0[3];
        float off0;
        float mass;
        float comX;
        float inertiaXX;
        memcpy(n0, clean + normalsAt, sizeof(n0));
        memcpy(&off0, clean + offsetsAt, 4);
        memcpy(&mass, clean + vertsAt - 64, 4);
        memcpy(&comX, clean + vertsAt - 60, 4);
        memcpy(&inertiaXX, clean + vertsAt - 48, 4);
        Poison poisons[8];
        int32_t n = 0;
        poisons[n] = PoisonF32(normalsAt, -n0[0], "a flipped normal is refused as invalid");
        n += 1;
        poisons[n] = PoisonF32(offsetsAt, off0 - 0.2f, "a plane below its face is refused");
        n += 1;
        poisons[n++] = PoisonF32(normalsAt + 4, NAN, "a NaN normal is refused as invalid");
        poisons[n++] = PoisonF32(offsetsAt, NAN, "a NaN offset is refused as invalid");
        poisons[n++] = PoisonF32(vertsAt - 64, 2.0f * mass, "a mass the vertices do not give");
        poisons[n++] = PoisonF32(vertsAt - 60, comX + 0.01f, "a centroid the vertices do not give");
        poisons[n++] =
            PoisonF32(vertsAt - 48, 1.5f * inertiaXX, "an inertia the vertices do not give");
        poisons[n++] = PoisonF32(vertsAt, -2.0e4f, "a vertex past the local limit is refused");
        CheckPoisonsRefused(world, clean, size, poisons, n);
    }
    free(clean);
    m3DestroyWorld(world);
}

// A needle's side planes are held to a share of the hull's extent, so a
// slack of 1e-2 of the extent would let a forged side plane sit several
// of the needle's widths off. QuickHull's planes hold to 1e-5 of the
// extent and the slack is 1e-4: a side plane pushed out by a quarter of
// the width along it is refused.
static void TestRestoreRefusesForgedNeedlePlane(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){5.0f, 0.5f, 5.0f});
    // A needle 2 long and 4e-3 thick along a slanted axis. The first
    // point is the origin, inside the needle, so QuickHull works about it
    // and the corners are stored as given, in input order.
    const m3Vec3 axis = {0.8f, 0.48f, 0.36f};
    const m3Vec3 side = {-0.6f, 0.8f, 0.0f};
    const m3Vec3 up = m3Cross3(axis, side);
    m3Vec3 pts[9] = {{0.0f, 0.0f, 0.0f}};
    for (int32_t i = 0; i < 8; ++i)
    {
        float a = (i & 1) != 0 ? 1.0f : -1.0f;
        float s = (i & 2) != 0 ? 2.0e-3f : -2.0e-3f;
        float u = (i & 4) != 0 ? 2.0e-3f : -2.0e-3f;
        pts[i + 1] = m3Add3(m3Add3(m3MulSV3(a, axis), m3MulSV3(s, side)), m3MulSV3(u, up));
    }
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    NewHullShape(world, NewBody(world, &bd), &sd, pts, 9);
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && Snapshot(world, clean, size) == size, "the needle world is snapshotted");
    int32_t vertsAt =
        clean != NULL ? FindBytes(clean, size, &pts[1], 2 * (int32_t)sizeof(m3Vec3)) : -1;
    int32_t vc = 0;
    int32_t fc = 0;
    if (vertsAt >= 80)
    {
        memcpy(&vc, clean + vertsAt - 80, 4);
        memcpy(&fc, clean + vertsAt - 76, 4);
    }
    int32_t normalsAt = vertsAt + vc * 12;
    int32_t offsetsAt = normalsAt + fc * 12;
    CHECK(vertsAt >= 80 && vc == 8 && fc == 6 && offsetsAt + fc * 4 <= size,
          "the needle's content is found");
    if (vertsAt >= 80 && vc == 8 && fc == 6 && offsetsAt + fc * 4 <= size)
    {
        // The face across which the needle is thinnest: a side face.
        int32_t thin = 0;
        float thinWidth = 1.0e30f;
        for (int32_t f = 0; f < fc; ++f)
        {
            m3Vec3 n;
            memcpy(&n, clean + normalsAt + 12 * f, sizeof(n));
            float lo = 1.0e30f;
            float hi = -1.0e30f;
            for (int32_t i = 1; i < 9; ++i)
            {
                lo = fminf(lo, m3Dot3(n, pts[i]));
                hi = fmaxf(hi, m3Dot3(n, pts[i]));
            }
            thin = hi - lo < thinWidth ? f : thin;
            thinWidth = fminf(thinWidth, hi - lo);
        }
        float offset;
        memcpy(&offset, clean + offsetsAt + 4 * thin, 4);
        CHECK(thinWidth < 5.0e-3f, "the needle is about 4e-3 thick");
        Poison poison = PoisonF32(offsetsAt + 4 * thin, offset + 0.25f * thinWidth,
                                  "a needle's side plane a quarter width out is refused");
        CheckPoisonsRefused(world, clean, size, &poison, 1);
    }
    free(clean);
    m3DestroyWorld(world);
}

// A restore lands height field content only when the grid door would
// take it: finite samples, a finite positive cell, and the extremes the
// samples give.
static void TestRestoreRefusesBadHeightFieldContent(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    static const float heights[9] = {0.125f, 0.25f,  0.375f, 0.5f,  0.625f,
                                     0.75f,  0.875f, 1.0f,   1.125f};
    NewHeightFieldGridShape(world, NewBody(world, &bd), &sd, heights, 3, 3, 1.0f);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.0, 2.0, 1.0};
    NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && Snapshot(world, clean, size) == size,
          "the height field world is snapshotted");
    // The cell size, then the minimum and maximum, sit before the samples.
    int32_t samplesAt =
        clean != NULL ? FindBytes(clean, size, heights, (int32_t)sizeof(heights)) : -1;
    CHECK(samplesAt >= 12, "the samples are found");
    if (samplesAt >= 12)
    {
        Poison poisons[4] = {
            PoisonF32(samplesAt + 4, NAN, "a sample that is not finite is refused as invalid"),
            PoisonF32(samplesAt - 8, 0.0f, "a minimum the samples do not give is refused"),
            PoisonF32(samplesAt - 4, 9.0f, "a maximum the samples do not give is refused"),
            PoisonF32(samplesAt - 12, NAN, "a cell size that is not finite is refused")};
        CheckPoisonsRefused(world, clean, size, poisons, 4);
    }
    free(clean);
    m3DestroyWorld(world);
}

static void TestRestoreRefusesBadCursors(void)
{
    // The body and shape pool cursors are the words that read 1 with one
    // body and 2 with two. Pointed past their pools, they are refused
    // before anything is overwritten.
    m3World* world = CursorWorld(1);
    m3World* twin = CursorWorld(2);
    int32_t size = m3World_SnapshotSize(world);
    CHECK(m3World_SnapshotSize(twin) == size, "one world shape, one snapshot size");
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    uint8_t* other = (uint8_t*)malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot taken");
    CHECK(Snapshot(twin, other, size) == size, "twin snapshot taken");
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
    CHECK(m3World_Restore(world, snap, size) == m3_errorFormat,
          "out-of-range cursors are refused as invalid input");
    CHECK(m3World_Hash(world) == before, "and the world is untouched");

    free(other);
    free(snap);
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

// Restore checks every incoming block before any of it lands: poison
// the snapshot with a huge 32-bit word at every third byte, which still
// writes every byte, since each poison is four bytes wide. Each field
// meets the poison at one class of shifts mod 3, not at every shift. No
// poisoned word may crash restore, and a refused restore leaves the world
// untouched and records a reason.
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
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, ground, &sd, (m3Vec3){5.0f, 0.5f, 5.0f});
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.2, 0.0};
    m3BodyId box = NewBody(world, &bd);
    NewBoxShape(world, box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
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
    Snapshot(world, clean, size);
    uint64_t cleanHash = m3World_Hash(world);
    int32_t refused = 0;
    int32_t untouched = 0;
    int32_t reasoned = 0;
    memcpy(poisoned, clean, (size_t)size);
    for (int32_t at = 0; at + 4 <= size; at += 3)
    {
        int32_t huge = 0x7FFFFFFF;
        memcpy(poisoned + at, &huge, sizeof(huge));
        m3Result result = m3World_Restore(world, poisoned, size);
        memcpy(poisoned + at, clean + at, sizeof(huge));
        if (result != m3_success)
        {
            refused += 1;
            untouched += m3World_Hash(world) == cleanHash ? 1 : 0;
            // A word over the release's fields, the ABI and format numbers
            // and the build hash within the header's first 24 bytes, says
            // another release wrote the bytes; any other says they are
            // damaged.
            reasoned += result == m3_errorFormat || (result == m3_errorVersion && at < 24) ? 1 : 0;
            continue;
        }
        Restore(world, clean, size);
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
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = NAN;
    static const m3Vec3 tri[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    static const uint16_t idx[3] = {0, 1, 2};
    m3ShapeId shape = m3_nullShapeId;
    CHECK(m3CreateMeshShape(world, ground, &sd, tri, 3, idx, 1, &shape) == m3_errorInvalid,
          "a mesh with a NaN density refuses as invalid input");
    CHECK(m3CreateHullShape(world, ground, &sd, tri, 3, &shape) == m3_errorInvalid,
          "a hull with a NaN density refuses as invalid input too");
    static const float heights[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    CHECK(m3CreateHeightFieldGridShape(world, ground, &sd, heights, 2, 2, 1.0f, &shape) ==
              m3_errorInvalid,
          "a height field with a NaN density refuses as invalid input as well");
    sd = m3DefaultShapeDef();
    m3Sphere bad = {{0.0f, 0.0f, 0.0f}, 0.5f};
    sd.friction = -1.0f;
    CHECK(m3CreateSphereShape(world, ground, &sd, &bad, &shape) == m3_errorInvalid,
          "a sphere with negative friction refuses and says why");
    CHECK(shape.index1 == 0, "every refusal leaves a null id");
    m3DestroyWorld(world);
}

// A snapshot or a tape is untrusted data: a damaged, truncated or empty
// one is refused with its status and counts no misuse. A null pointer
// with a nonzero size is an argument a correct program never passes, so
// it does count.
static void TestUntrustedDataCountsNoMisuse(void)
{
    m3World* world = CursorWorld(3);
    m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    static uint8_t tape[4096];
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "the snapshot is taken");
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts");
    m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t tapeBytes = StopJournal(world);
    CHECK(tapeBytes > 16 + 8, "the journal holds a step after its prefix");
    CHECK(Restore(world, snap, size), "the clean snapshot restores");
    uint64_t before = m3World_GetCounters(world).misuse;
    uint64_t hash = m3World_Hash(world);

    CHECK(m3World_Restore(world, snap, size - 1) == m3_errorFormat,
          "a snapshot one byte short is refused");
    CHECK(m3World_Restore(world, snap, 4) == m3_errorFormat, "a stub shorter than the header too");
    CHECK(m3World_Restore(world, snap, 0) == m3_errorFormat, "and an empty one");
    snap[0] ^= 0xFFu;
    CHECK(m3World_Restore(world, snap, size) == m3_errorFormat, "a damaged magic is refused");
    snap[0] ^= 0xFFu;
    // Format 4 had no state prefix; its snapshots are refused by version,
    // not by layout.
    uint32_t format = 0;
    memcpy(&format, snap + 8, 4);
    CHECK(format == 5u, "a snapshot is written in format 5");
    format = 4u;
    memcpy(snap + 8, &format, 4);
    CHECK(m3World_Restore(world, snap, size) == m3_errorVersion, "a format 4 snapshot is refused");
    format = 5u;
    memcpy(snap + 8, &format, 4);
    CHECK(m3World_ReplayJournal(world, tape, tapeBytes - 1) == m3_errorFormat,
          "a tape one byte short is refused");
    CHECK(m3World_GetCounters(world).misuse == before, "and none of it counts as misuse");
    CHECK(m3World_Hash(world) == hash, "nor changes the world");

    CHECK(m3World_Restore(world, NULL, size) == m3_errorInvalid, "a null snapshot is refused");
    CHECK(m3World_ReplayJournal(world, NULL, tapeBytes) == m3_errorInvalid,
          "a null tape is refused");
    CHECK(m3World_GetCounters(world).misuse == before + 2, "and those two count");
    CHECK(m3World_ReplayJournal(world, NULL, 0) == m3_errorFormat,
          "no bytes are not a tape: it starts with its prefix");
    CHECK(m3World_ReplayJournal(world, tape, 16) == m3_success, "a tape of no op replays nothing");
    CHECK(m3World_GetCounters(world).misuse == before + 2, "and neither counts");
    free(snap);
    m3DestroyWorld(world);
}

// One header word changed at a time, every other byte equal: another
// release's or another build's bytes are refused with m3_errorVersion,
// bytes that are not the format or are corrupt with m3_errorInvalid. A
// refusal changes nothing and counts no misuse, and the bytes as written
// still restore, replay and decode after them. Offsets are the state
// prefix's (docs/internals/state-formats.md): the ABI version at 4, the
// format number at 8, the header size at 12, the snapshot's build hash
// at 16, and a container's snapshot after its 40-byte header.
static void TestOtherReleasesAreRefused(void)
{
    m3World* world = CursorWorld(3);
    m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    uint8_t* bad = (uint8_t*)malloc((size_t)snapBytes + 8192);
    static uint8_t tape[4096];
    CHECK(snap != NULL && bad != NULL && Snapshot(world, snap, snapBytes) == snapBytes,
          "the snapshot is taken");
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts");
    m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t tapeBytes = StopJournal(world);
    int32_t boxBytes = m3GetEncodedReplaySize(snapBytes, tapeBytes);
    uint8_t* box = (uint8_t*)malloc((size_t)boxBytes);
    int32_t written = 0;
    CHECK(box != NULL && m3EncodeReplay(snap, snapBytes, tape, tapeBytes, m3World_Hash(world), box,
                                        boxBytes, &written) == m3_success,
          "the container is encoded");
    CHECK(Restore(world, snap, snapBytes), "the snapshot restores");
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = m3World_GetCounters(world).misuse;
    // Each magic is its name in file order, as a hex dump reads it.
    CHECK(memcmp(snap, "M3SN", 4) == 0 && memcmp(tape, "M3JN", 4) == 0 &&
              memcmp(box, "M3RP", 4) == 0,
          "each format's first four bytes spell its name");

    enum
    {
        SNAPSHOT,
        TAPE,
        CONTAINER
    };
    const struct
    {
        int32_t format;
        int32_t at;
        m3Result expected;
        const char* what;
    } cases[] = {
        {SNAPSHOT, 4, m3_errorVersion, "a snapshot of another ABI version"},
        {SNAPSHOT, 8, m3_errorVersion, "a snapshot of another format number"},
        {SNAPSHOT, 16, m3_errorVersion, "a snapshot of another build"},
        {SNAPSHOT, 0, m3_errorFormat, "a snapshot with another magic"},
        {SNAPSHOT, 12, m3_errorFormat, "a snapshot with another header size"},
        {TAPE, 4, m3_errorVersion, "a tape of another ABI version"},
        {TAPE, 8, m3_errorVersion, "a tape of another format number"},
        {TAPE, 0, m3_errorFormat, "a tape with another magic"},
        {TAPE, 12, m3_errorFormat, "a tape with another header size"},
        {CONTAINER, 4, m3_errorVersion, "a container of another ABI version"},
        {CONTAINER, 8, m3_errorVersion, "a container of another format number"},
        {CONTAINER, 0, m3_errorFormat, "a container with another magic"},
        {CONTAINER, 12, m3_errorFormat, "a container with another header size"},
        {CONTAINER, 40 + 4, m3_errorVersion, "a container holding another release's snapshot"},
        {CONTAINER, 40 + 8, m3_errorVersion, "a container holding another snapshot format"},
        {CONTAINER, -4, m3_errorVersion, "a container holding another release's tape"},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c)
    {
        const uint8_t* clean = cases[c].format == SNAPSHOT ? snap
                               : cases[c].format == TAPE   ? tape
                                                           : box;
        int32_t size = cases[c].format == SNAPSHOT ? snapBytes
                       : cases[c].format == TAPE   ? tapeBytes
                                                   : written;
        // A negative offset names the container's tape: its ABI version.
        int32_t at = cases[c].at >= 0 ? cases[c].at : 40 + snapBytes + 4;
        memcpy(bad, clean, (size_t)size);
        uint32_t word = 0;
        memcpy(&word, bad + at, 4);
        word += 1u;
        memcpy(bad + at, &word, 4);
        m3Result result = m3_success;
        if (cases[c].format == SNAPSHOT)
        {
            result = m3World_Restore(world, bad, size);
        }
        else if (cases[c].format == TAPE)
        {
            result = m3World_ReplayJournal(world, bad, size);
            m3JournalInfo info;
            CHECK(m3DescribeJournal(bad, size, &info) == result,
                  "describing the tape refuses it the same way");
        }
        else
        {
            m3ReplayView view;
            result = m3DecodeReplay(bad, size, &view);
        }
        if (result != cases[c].expected)
        {
            printf("FAIL: %s returned %d, not %d\n", cases[c].what, (int)result,
                   (int)cases[c].expected);
            s_failures += 1;
        }
    }
    CHECK(m3World_Hash(world) == hash, "no refusal changed the world");
    CHECK(m3World_GetCounters(world).misuse == misuse, "and none counted as misuse");

    CHECK(Restore(world, snap, snapBytes), "the snapshot as written still restores");
    CHECK(ReplayJournal(world, tape, tapeBytes), "the tape as written still replays");
    m3ReplayView view;
    CHECK(m3DecodeReplay(box, written, &view) == m3_success, "the container as written decodes");
    // A valid prefix whose header runs past the bytes is refused before the
    // header is read: 20 bytes of a snapshot or a container, in allocations
    // of exactly that size, so a sanitizer sees any read of the 56 or 40.
    uint8_t* stub = (uint8_t*)malloc(20);
    CHECK(stub != NULL, "the stub is allocated");
    if (stub != NULL)
    {
        memcpy(stub, snap, 20);
        CHECK(m3World_Restore(world, stub, 20) == m3_errorFormat,
              "a snapshot cut inside its header is refused");
        memcpy(stub, box, 20);
        CHECK(m3DecodeReplay(stub, 20, &view) == m3_errorFormat,
              "a container cut inside its header is refused");
        free(stub);
    }
    // Nor does a container wrap what another release wrote.
    memcpy(bad, snap, (size_t)snapBytes);
    bad[4] ^= 1u;
    CHECK(m3EncodeReplay(bad, snapBytes, tape, tapeBytes, 0, box, boxBytes, &written) ==
              m3_errorVersion,
          "encoding another release's snapshot is refused");
    memcpy(bad, tape, (size_t)tapeBytes);
    bad[4] ^= 1u;
    CHECK(m3EncodeReplay(snap, snapBytes, bad, tapeBytes, 0, box, boxBytes, &written) ==
              m3_errorVersion,
          "and so is encoding another release's tape");
    free(box);
    free(bad);
    free(snap);
    m3DestroyWorld(world);
}

typedef enum StaticDoor
{
    planeDoor,
    meshDoor,
    voxelDoor,
    doorCount
} StaticDoor;

static void CreateOnGround(m3World* world, m3BodyId ground, StaticDoor door)
{
    static const m3Vec3 quad[4] = {
        {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, 1.0f}, {-1.0f, 0.0f, 1.0f}};
    static const uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    static uint8_t voxels[16 * 16 * 16];
    voxels[0] = 1;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane plane = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId shape = m3_nullShapeId;
    if (door == planeDoor)
    {
        shape = NewPlaneShape(world, ground, &sd, &plane);
    }
    else if (door == meshDoor)
    {
        shape = NewMeshShape(world, ground, &sd, quad, 4, tris, 2);
    }
    else
    {
        shape = NewVoxelChunkShape(world, ground, &sd, voxels, NULL, 1.0f);
    }
    CHECK(m3Shape_IsValid(world, shape), "the clean static shape is created");
}

// Replays a tape over the snapshot, after an optional change the tape
// does not hold. Whether it is refused as untrusted data: m3_errorInvalid,
// no misuse, and the world as it was.
static bool TapeRefused(m3World* world, const uint8_t* snap, int32_t snapBytes, const uint8_t* tape,
                        int32_t tapeBytes, m3BodyId makeDynamic)
{
    if (!Restore(world, snap, snapBytes))
    {
        return false;
    }
    if (makeDynamic.index1 != 0)
    {
        m3Body_SetType(world, makeDynamic, m3_dynamicBody);
    }
    uint64_t misuse = m3World_GetCounters(world).misuse;
    uint64_t hash = m3World_Hash(world);
    return m3World_ReplayJournal(world, tape, tapeBytes) == m3_errorFormat &&
           m3World_GetCounters(world).misuse == misuse && m3World_Hash(world) == hash;
}

// Replay builds shapes through the same walls as the public creates: a
// tape cannot put a plane, a mesh or a voxel chunk on a dynamic body,
// nor give a mesh an index past its vertices.
static void TestReplayHoldsTheShapeRules(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    int32_t capacity = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)capacity);
    int32_t snapBytes = Snapshot(world, snap, capacity);
    CHECK(snap != NULL && snapBytes > 0, "the empty ground is snapshotted");
    static uint8_t tape[65536];
    for (int32_t door = 0; door < doorCount; ++door)
    {
        CHECK(Restore(world, snap, snapBytes), "the ground restores");
        CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts");
        CreateOnGround(world, ground, (StaticDoor)door);
        int32_t tapeBytes = StopJournal(world);
        CHECK(tapeBytes > 8, "the create is recorded");
        CHECK(TapeRefused(world, snap, snapBytes, tape, tapeBytes, ground),
              "a static-only shape on a dynamic body is refused by replay");
        if (door == meshDoor)
        {
            tape[tapeBytes - 1] = 0xFFu; // the last index, past the four vertices
            tape[tapeBytes - 2] = 0xFFu;
            CHECK(TapeRefused(world, snap, snapBytes, tape, tapeBytes, m3_nullBodyId),
                  "a mesh index past its vertices is refused by replay");
        }
    }
    free(snap);
    m3DestroyWorld(world);
}

// Host code that Step or Draw runs (the pre-solve callback, debug draw
// callbacks) cannot change the world under it: a writer it calls is
// refused with m3_errorBusy, counts one misuse and changes nothing.
typedef struct BusyProbe
{
    m3World* world;
    m3BodyId body;
    int32_t calls;
    int32_t refused;
} BusyProbe;

static void TryWriters(BusyProbe* probe)
{
    probe->calls += 1;
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId created = m3_nullBodyId;
    probe->refused += m3CreateBody(probe->world, &bd, &created) == m3_errorBusy ? 1 : 0;
    probe->refused += m3DestroyBody(probe->world, probe->body) == m3_errorBusy ? 1 : 0;
    m3Body_SetLinearVelocity(probe->world, probe->body, (m3Vec3){0.0f, 50.0f, 0.0f});
    m3World_Step(probe->world, 1.0f / 60.0f, 4);
}

static bool BusyPreSolve(m3ShapeId shapeA, m3ShapeId shapeB, m3Pos3 point, m3Vec3 normal,
                         void* context)
{
    (void)shapeA;
    (void)shapeB;
    (void)point;
    (void)normal;
    TryWriters((BusyProbe*)context);
    return true;
}

static void BusyDrawSegment(m3Pos3 p1, m3Pos3 p2, uint32_t color, void* context)
{
    (void)p1;
    (void)p2;
    (void)color;
    BusyProbe* probe = (BusyProbe*)context;
    if (probe->calls < 3)
    {
        TryWriters(probe);
    }
}

static void TestWritersRefuseWhileBusy(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, NewBody(world, &gd), &sd, &floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.45, 0.0};
    m3ShapeDef presolve = m3DefaultShapeDef();
    presolve.enablePreSolveEvents = true;
    BusyProbe probe = {world, NewBody(world, &bd), 0, 0};
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, probe.body, &presolve, &ball);
    m3World_SetPreSolveCallback(world, BusyPreSolve, &probe);

    uint64_t misuse = m3World_GetCounters(world).misuse;
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(probe.calls > 0, "the pre-solve callback ran");
    CHECK(probe.refused == 2 * probe.calls, "every status writer it called was refused");
    CHECK(m3World_GetCounters(world).misuse == misuse + 4u * (uint64_t)probe.calls,
          "and every writer it called counted once");
    CHECK(m3Body_IsValid(world, probe.body) && m3World_GetCounters(world).bodyCount == 2,
          "nothing was created or destroyed under the step");
    CHECK(m3Body_GetLinearVelocity(world, probe.body).y < 1.0f, "the velocity write did not land");
    CHECK(m3World_GetStepCount(world) == 1, "the nested step did not run");

    m3World_SetPreSolveCallback(world, NULL, NULL);
    probe.calls = 0;
    probe.refused = 0;
    m3DebugDraw draw;
    memset(&draw, 0, sizeof(draw));
    draw.drawSegment = BusyDrawSegment;
    draw.context = &probe;
    draw.drawShapes = true;
    m3World_Draw(world, &draw);
    CHECK(probe.calls > 0 && probe.refused == 2 * probe.calls,
          "a writer called from a draw callback is refused too");
    CHECK(m3World_GetStepCount(world) == 1, "and steps nothing");

    CHECK(NewBody(world, &bd).index1 != 0, "after the step and the draw, writers work again");
    m3DestroyWorld(world);
}

// A create past a published bound returns m3_errorLimit, an infinity
// included, and a NaN returns m3_errorInvalid, even beside a value past
// a bound. Either counts one misuse and changes nothing.
static void TestBoundsRefuseAsLimit(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* refused = NULL;
    wd.gravity.y = -2.0f * M3_ACCEL_LIMIT;
    CHECK(m3CreateWorld(&wd, &refused) == m3_errorLimit && refused == NULL,
          "a world def's gravity past its bound is refused as limit");
    wd.contactHertz = NAN;
    CHECK(m3CreateWorld(&wd, &refused) == m3_errorInvalid, "a NaN outranks a value past a bound");
    wd = m3DefaultWorldDef();
    wd.voxelCapacity = M3_MAX_VOXEL_CAPACITY + 1;
    CHECK(m3CreateWorld(&wd, &refused) == m3_errorLimit,
          "a voxel capacity past M3_MAX_VOXEL_CAPACITY is refused as limit");

    wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId body = NewBody(world, &bd);
    m3BodyDef staticDef = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &staticDef);
    m3ShapeDef sd = m3DefaultShapeDef();
    (void)NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3BodyId bodyOut = m3_nullBodyId;
    m3ShapeId shapeOut = m3_nullShapeId;
    m3Result result = m3_success;

    const double positions[3] = {2.0 * M3_POSITION_LIMIT, (double)INFINITY, -(double)INFINITY};
    for (int32_t i = 0; i < 3; ++i)
    {
        m3BodyDef far = bd;
        far.position.x = positions[i];
        CHECK_REFUSES(world, result = m3CreateBody(world, &far, &bodyOut), "a far body misuses");
        CHECK(result == m3_errorLimit && bodyOut.index1 == 0,
              "a body past M3_POSITION_LIMIT is refused as limit");
    }
    m3BodyDef poisoned = bd;
    poisoned.position.x = (double)NAN;
    poisoned.linearVelocity.x = 2.0f * M3_SPEED_LIMIT;
    CHECK_REFUSES(world, result = m3CreateBody(world, &poisoned, &bodyOut), "a NaN misuses");
    CHECK(result == m3_errorInvalid, "a NaN position is invalid beside a speed past its bound");

    CHECK_REFUSES(world,
                  result = m3CreateBoxShape(world, body, &sd,
                                            (m3Vec3){2.0f * M3_LOCAL_LIMIT, 1.0f, 1.0f}, &shapeOut),
                  "a box past the local bound misuses");
    CHECK(result == m3_errorLimit, "a box past M3_LOCAL_LIMIT is refused as limit");
    m3ShapeDef dense = sd;
    dense.density = 2.0f * M3_DENSITY_LIMIT;
    CHECK_REFUSES(world,
                  result = m3CreateBoxShape(world, body, &dense, (m3Vec3){1, 1, 1}, &shapeOut),
                  "a density past its bound misuses");
    CHECK(result == m3_errorLimit, "a density past M3_DENSITY_LIMIT is refused as limit");
    CHECK_REFUSES(
        world,
        result = m3CreateBoxShape(world, body, &sd, (m3Vec3){1.0e-4f, 1.0e-4f, 1.0e-4f}, &shapeOut),
        "a box below the mass floor misuses");
    CHECK(result == m3_errorLimit, "a shape below M3_MIN_MASS is refused as limit");

    static m3Vec3 cloud[M3_MAX_HULL_POINTS + 1];
    for (int32_t i = 0; i <= M3_MAX_HULL_POINTS; ++i)
    {
        m3CosSin cs = m3ComputeCosSin(0.1f * (float)i);
        cloud[i] = (m3Vec3){cs.c, cs.s, 0.01f * (float)(i % 7)};
    }
    CHECK_REFUSES(world,
                  result =
                      m3CreateHullShape(world, body, &sd, cloud, M3_MAX_HULL_POINTS + 1, &shapeOut),
                  "a cloud past the point count misuses");
    CHECK(result == m3_errorLimit, "a cloud past M3_MAX_HULL_POINTS is refused as limit");

    static float heights[(M3_MAX_HEIGHTFIELD_SIZE + 1) * 2];
    CHECK_REFUSES(world,
                  result = m3CreateHeightFieldGridShape(
                      world, ground, &sd, heights, M3_MAX_HEIGHTFIELD_SIZE + 1, 2, 1.0f, &shapeOut),
                  "a grid past its size misuses");
    CHECK(result == m3_errorLimit, "a grid past M3_MAX_HEIGHTFIELD_SIZE is refused as limit");
    CHECK_REFUSES(world,
                  result = m3CreateHeightFieldShape(world, ground, &sd, heights,
                                                    M3_MAX_HEIGHTFIELD_MESH_SIZE + 1, 2, 1.0f,
                                                    &shapeOut),
                  "a mesh height field past its size misuses");
    CHECK(result == m3_errorLimit,
          "a mesh height field past M3_MAX_HEIGHTFIELD_MESH_SIZE is refused as limit");

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = ground;
    jd.bodyIdB = body;
    jd.enableMotor = true;
    jd.motorSpeed = 2.0f * M3_SPEED_LIMIT;
    m3JointId jointOut = m3_nullJointId;
    CHECK_REFUSES(world, result = m3CreateJoint(world, &jd, &jointOut), "a fast motor misuses");
    CHECK(result == m3_errorLimit && jointOut.index1 == 0,
          "a motor speed past M3_SPEED_LIMIT is refused as limit");

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.mass = 0.5f * M3_MIN_MASS;
    m3CharacterId characterOut = m3_nullCharacterId;
    CHECK_REFUSES(world, result = m3CreateCharacter(world, &cd, &characterOut),
                  "a light character misuses");
    CHECK(result == m3_errorLimit, "a character below M3_MIN_MASS is refused as limit");
    cd = m3DefaultCharacterDef();
    CHECK(m3CreateCharacter(world, &cd, &characterOut) == m3_success, "a character creates");
    CHECK_REFUSES(world,
                  result = m3Character_SetStance(world, characterOut, 2.0f * M3_STANCE_LIMIT, 0.4f),
                  "a tall stance misuses");
    CHECK(result == m3_errorLimit, "a stance past M3_STANCE_LIMIT is refused as limit");
    CHECK_REFUSES(world, result = m3Character_SetStance(world, characterOut, NAN, 0.4f),
                  "a NaN stance misuses");
    CHECK(result == m3_errorInvalid, "a NaN stance is invalid");

    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = body;
    vd.wheelCount = M3_MAX_VEHICLE_WHEELS + 1;
    m3VehicleId vehicleOut = m3_nullVehicleId;
    CHECK_REFUSES(world, result = m3CreateVehicle(world, &vd, &vehicleOut),
                  "a vehicle past its wheels misuses");
    CHECK(result == m3_errorLimit, "a vehicle past M3_MAX_VEHICLE_WHEELS is refused as limit");
    vd.wheelCount = 4;
    vd.wheels[2].radius = 0.5f * M3_MIN_WHEEL_RADIUS;
    CHECK_REFUSES(world, result = m3CreateVehicle(world, &vd, &vehicleOut),
                  "a small wheel misuses");
    CHECK(result == m3_errorLimit, "a wheel below M3_MIN_WHEEL_RADIUS is refused as limit");

    m3SoftBodyDef soft = m3DefaultSoftBodyDef();
    soft.countX = M3_MAX_SOFTBODY_PARTICLES + 1;
    m3SoftBodyId softOut = m3_nullSoftBodyId;
    CHECK_REFUSES(world, result = m3CreateSoftBody(world, &soft, &softOut),
                  "a lattice past its particles misuses");
    CHECK(result == m3_errorLimit, "a lattice past M3_MAX_SOFTBODY_PARTICLES is refused as limit");

    m3WaterVolumeDef water = m3DefaultWaterVolumeDef();
    water.flow.x = (float)INFINITY;
    m3WaterVolumeId waterOut = m3_nullWaterVolumeId;
    CHECK_REFUSES(world, result = m3CreateWaterVolume(world, &water, &waterOut),
                  "an infinite flow misuses");
    CHECK(result == m3_errorLimit, "an infinite flow is past M3_SPEED_LIMIT");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRestoreRefusesPoisonedWords();
    TestRestoreRefusesBadCursors();
    TestRestoreRefusesBadMeshContent();
    TestRestoreRefusesBadHullContent();
    TestRestoreAcceptsEveryCreatedHull();
    TestFarHullsCreate();
    TestRestoreRefusesBadHullDerived();
    TestRestoreRefusesForgedNeedlePlane();
    TestRestoreRefusesBadHeightFieldContent();
    TestCreateWorldOutOfMemory();
    TestCapacityExhaustion();
    TestJournalOverflowIsLoud();
    TestShapeDefsRefuseAsInvalid();
    TestUntrustedDataCountsNoMisuse();
    TestOtherReleasesAreRefused();
    TestReplayHoldsTheShapeRules();
    TestWritersRefuseWhileBusy();
    TestBoundsRefuseAsLimit();
    if (s_failures == 0)
    {
        printf("test_hardening: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
