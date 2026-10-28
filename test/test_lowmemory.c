// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The low-memory gate: under a host allocator that runs out after a
// given number of blocks, a call either refuses with m3_errorCapacity
// and leaves the world as it was, or does all of its work. Each test
// sweeps the budget from zero until the call succeeds, so every
// allocation the call makes is refused once. A replay refused memory
// says so with m3_errorCapacity and backs out whole.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHUNK_DIM 16 // a voxel chunk is 16 voxels a side

// A host allocator that hands out `left` more blocks, or any number
// while left is negative.
typedef struct Budget
{
    int32_t left;
} Budget;

static void* BudgetAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
    Budget* budget = (Budget*)context;
    if (budget->left == 0)
    {
        return NULL;
    }
    if (budget->left > 0)
    {
        budget->left -= 1;
    }
    return malloc(bytes);
}

static void BudgetFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    (void)context;
    free(memory);
}

static m3World* BudgetWorld(Budget* budget)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.allocator = (m3Allocator){BudgetAlloc, BudgetFree, budget};
    return NewWorld(&wd);
}

static const m3Vec3 kQuadVerts[4] = {
    {-3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, 3.0f}, {-3.0f, 0.0f, 3.0f}};
static const uint16_t kQuadTris[6] = {0, 2, 1, 0, 3, 2};

static m3Result CreateFloor(m3World* world, m3BodyId ground, m3ShapeId* floorOut)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    return m3CreateMeshShape(world, ground, &sd, kQuadVerts, 4, kQuadTris, 2, floorOut);
}

// One layer of voxels at the bottom of a chunk on a body at x = 10.
static m3Result CreateSlab(m3World* world, m3BodyId slabBody, m3ShapeId* slabOut)
{
    static uint8_t voxels[CHUNK_DIM * CHUNK_DIM * CHUNK_DIM];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < CHUNK_DIM; ++z)
    {
        for (int32_t x = 0; x < CHUNK_DIM; ++x)
        {
            voxels[x + CHUNK_DIM * CHUNK_DIM * z] = 1;
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    return m3CreateVoxelChunkShape(world, slabBody, &sd, voxels, NULL, 0.5f, slabOut);
}

// Whether a ray straight down onto the shape strikes it: an empty mesh
// or voxel tree answers no query.
static bool RayStrikes(const m3World* world, m3Pos3 origin, m3ShapeId shape)
{
    m3RayCastResult hit =
        m3World_CastRayClosest(world, origin, (m3Vec3){0.0f, -10.0f, 0.0f}, m3DefaultQueryFilter());
    return hit.hit && hit.shapeId.index1 == shape.index1 &&
           hit.shapeId.generation == shape.generation;
}

static const m3Pos3 kOverFloor = {0.5, 5.0, 0.5};
static const m3Pos3 kOverSlab = {12.0, 5.0, 2.0};

// Whether a world's snapshot equals the given bytes.
static bool SnapshotIs(const m3World* world, const uint8_t* bytes, int32_t size)
{
    if (m3World_SnapshotSize(world) != size)
    {
        return false;
    }
    uint8_t* now = (uint8_t*)malloc((size_t)size);
    bool same =
        now != NULL && Snapshot(world, now, size) == size && memcmp(now, bytes, (size_t)size) == 0;
    free(now);
    return same;
}

static uint8_t* TakeSnapshot(const m3World* world, int32_t* sizeOut)
{
    *sizeOut = m3World_SnapshotSize(world);
    uint8_t* bytes = (uint8_t*)malloc((size_t)*sizeOut);
    if (bytes != NULL && Snapshot(world, bytes, *sizeOut) != *sizeOut)
    {
        free(bytes);
        bytes = NULL;
    }
    return bytes;
}

static void TestRestoreUnderBudget(void)
{
    // A floor, a slab and a box falling onto the floor: restoring the
    // first frame over the thirtieth rebuilds both trees.
    Budget budget = {-1};
    m3World* world = BudgetWorld(&budget);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    gd.position = (m3Pos3){10.0, 0.0, 0.0};
    m3BodyId slabBody = NewBody(world, &gd);
    m3ShapeId floor = m3_nullShapeId;
    m3ShapeId slab = m3_nullShapeId;
    CHECK(CreateFloor(world, ground, &floor) == m3_success &&
              CreateSlab(world, slabBody, &slab) == m3_success,
          "the floor and the slab create");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-1.0, 1.0, -1.0};
    m3BodyId box = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, box, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});

    int32_t size = 0;
    uint8_t* first = TakeSnapshot(world, &size);
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t laterSize = 0;
    uint8_t* later = TakeSnapshot(world, &laterSize);
    CHECK(first != NULL && later != NULL && laterSize == size &&
              memcmp(first, later, (size_t)size) != 0,
          "the two frames differ");

    bool refusedWhole = true;
    bool restored = false;
    int32_t refusals = 0;
    for (int32_t n = 0; n < 1000 && !restored; ++n)
    {
        budget.left = n;
        m3Result result = m3World_Restore(world, first, size);
        budget.left = -1;
        if (result == m3_success)
        {
            restored = true;
            break;
        }
        refusals += 1;
        refusedWhole = refusedWhole && result == m3_errorCapacity &&
                       SnapshotIs(world, later, size) && RayStrikes(world, kOverFloor, floor) &&
                       RayStrikes(world, kOverSlab, slab);
    }
    CHECK(refusals > 0 && refusedWhole,
          "a restore the allocator refuses leaves the world as it was");
    CHECK(restored && SnapshotIs(world, first, size), "with enough memory the restore lands");
    CHECK(RayStrikes(world, kOverFloor, floor), "the restored floor's tree answers");
    CHECK(RayStrikes(world, kOverSlab, slab), "the restored slab's tree answers");
    free(later);
    free(first);
    m3DestroyWorld(world);
}

// A world with one static body, at x = 10 for the slab, and its shape
// created with memory to spare when create is set.
static m3World* CreateWorld(Budget* budget, bool slabNotFloor, bool create, m3BodyId* bodyOut)
{
    m3World* world = BudgetWorld(budget);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = slabNotFloor ? (m3Pos3){10.0, 0.0, 0.0} : (m3Pos3){0.0, 0.0, 0.0};
    *bodyOut = NewBody(world, &gd);
    m3ShapeId shape = m3_nullShapeId;
    if (create)
    {
        (void)(slabNotFloor ? CreateSlab(world, *bodyOut, &shape)
                            : CreateFloor(world, *bodyOut, &shape));
    }
    return world;
}

// Sweeps a create from a budget of zero blocks up. Each refusal must be
// m3_errorCapacity, count no misuse and leave the hash as it was; the
// create that succeeds must leave the same bytes as a create that never
// ran short (the refusals left no trace in the pools), and a shape whose
// tree answers.
static void SweepCreate(bool slabNotFloor, m3Pos3 over, const char* refusedMsg,
                        const char* createdMsg)
{
    Budget budget = {-1};
    m3BodyId body;
    m3World* world = CreateWorld(&budget, slabNotFloor, false, &body);
    Budget plenty = {-1};
    m3BodyId referenceBody;
    m3World* reference = CreateWorld(&plenty, slabNotFloor, true, &referenceBody);
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = m3World_GetCounters(world).misuse;
    bool refusedWhole = true;
    int32_t refusals = 0;
    m3ShapeId shape = m3_nullShapeId;
    m3Result result = m3_errorCapacity;
    for (int32_t n = 0; n < 1000 && result != m3_success; ++n)
    {
        budget.left = n;
        result = slabNotFloor ? CreateSlab(world, body, &shape) : CreateFloor(world, body, &shape);
        budget.left = -1;
        if (result != m3_success)
        {
            refusals += 1;
            refusedWhole = refusedWhole && result == m3_errorCapacity &&
                           m3World_Hash(world) == hash &&
                           m3World_GetCounters(world).misuse == misuse;
        }
    }
    CHECK(refusals > 0 && refusedWhole, refusedMsg);
    int32_t size = 0;
    uint8_t* expected = TakeSnapshot(reference, &size);
    CHECK(result == m3_success && expected != NULL && SnapshotIs(world, expected, size),
          "the create that succeeds leaves the world a create with memory to spare does");
    CHECK(result == m3_success && RayStrikes(world, over, shape), createdMsg);
    free(expected);
    m3DestroyWorld(reference);
    m3DestroyWorld(world);
}

// A hull create takes QuickHull's working blocks from the allocator. A
// refusal once read as a degenerate cloud: m3_errorInvalid, counted as
// the caller's misuse, for a cloud that builds once memory is back.
static void SweepHullCreate(void)
{
    static const m3Vec3 cloud[6] = {{0.3f, 0.0f, 0.0f},  {-0.3f, 0.0f, 0.0f}, {0.0f, 0.3f, 0.0f},
                                    {0.0f, -0.3f, 0.0f}, {0.0f, 0.0f, 0.3f},  {0.0f, 0.0f, -0.3f}};
    Budget budget = {-1};
    m3World* world = BudgetWorld(&budget);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = m3World_GetCounters(world).misuse;
    bool refusedWhole = true;
    int32_t refusals = 0;
    m3ShapeId shape = m3_nullShapeId;
    m3Result result = m3_errorCapacity;
    for (int32_t n = 0; n < 1000 && result != m3_success; ++n)
    {
        budget.left = n;
        result = m3CreateHullShape(world, body, &sd, cloud, 6, &shape);
        budget.left = -1;
        if (result != m3_success)
        {
            refusals += 1;
            refusedWhole = refusedWhole && result == m3_errorCapacity &&
                           m3World_Hash(world) == hash &&
                           m3World_GetCounters(world).misuse == misuse;
        }
    }
    CHECK(refusals > 0 && refusedWhole,
          "a hull create the allocator refuses is m3_errorCapacity, counts no misuse and leaves "
          "the world as it was");
    CHECK(result == m3_success && m3Shape_IsValid(world, shape),
          "with memory back the same cloud builds");
    m3DestroyWorld(world);
}

static void TestCreateUnderBudget(void)
{
    SweepHullCreate();
    SweepCreate(false, kOverFloor, "a mesh create the allocator refuses leaves the world as it was",
                "the created mesh's tree answers");
    SweepCreate(true, kOverSlab, "a voxel create the allocator refuses leaves the world as it was",
                "the created chunk's tree answers");
}

// One voxel edit on the slab, with a pillar of two voxels at (3, 1..2, 3)
// that a clear under it fractures away. kOverSlab looks down at voxel
// (4, 0, 4), which no edit touches.
static void VoxelEdit(m3World* world, m3ShapeId slab, int32_t edit)
{
    static const int32_t boxLo[3] = {8, 0, 8};
    static const int32_t boxHi[3] = {9, 0, 9};
    m3ExplosionDef blast = m3DefaultExplosionDef();
    switch (edit)
    {
    case 0:
        m3VoxelChunk_SetVoxel(world, slab, 6, 1, 6, 7);
        break;
    case 1:
        m3VoxelChunk_ClearVoxel(world, slab, 0, 0, 0);
        break;
    case 2:
        (void)m3VoxelChunk_ClearBox(world, slab, boxLo, boxHi);
        break;
    case 3:
        m3VoxelChunk_ClearVoxel(world, slab, 3, 1, 3); // the pillar's top fractures
        break;
    default:
        blast.position = (m3Pos3){10.0, 0.25, 7.5};
        blast.radius = 0.5f;
        blast.falloff = 0.0f;
        blast.voxelCarve = 1.0f;
        m3World_Explode(world, &blast);
        break;
    }
}

// Sweeps each voxel edit from a budget of zero blocks up. An edit either
// refuses, with the world's hash as it was, no misuse counted and nothing
// on the journal, or lands with the chunk's tree still answering a ray.
// A rebuild once took the tree's arrays from the allocator, and a refusal
// left the edited grid with an empty tree that answered nothing.
static void TestVoxelEditsUnderBudget(void)
{
    static const char* const names[5] = {"a voxel set", "a voxel clear", "a box clear",
                                         "a clear that fractures", "an explosion's carve"};
    static uint8_t tape[4096];
    for (int32_t edit = 0; edit < 5; ++edit)
    {
        bool whole = true;
        int32_t landed = 0;
        for (int32_t n = 0; n < 8; ++n)
        {
            Budget budget = {-1};
            m3BodyId body;
            m3World* world = CreateWorld(&budget, true, false, &body);
            m3ShapeId slab = m3_nullShapeId;
            CHECK(CreateSlab(world, body, &slab) == m3_success, "the slab creates");
            m3VoxelChunk_SetVoxel(world, slab, 3, 1, 3, 1);
            m3VoxelChunk_SetVoxel(world, slab, 3, 2, 3, 1);
            uint64_t hash = m3World_Hash(world);
            uint64_t misuse = m3World_GetCounters(world).misuse;
            CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts");
            budget.left = n;
            VoxelEdit(world, slab, edit);
            budget.left = -1;
            int32_t journaled = StopJournal(world);
            bool refused = m3World_Hash(world) == hash;
            landed += refused ? 0 : 1;
            // A refused edit journals nothing past the tape's 16-byte prefix.
            whole =
                whole && (refused ? journaled == 16 && m3World_GetCounters(world).misuse == misuse
                                  : RayStrikes(world, kOverSlab, slab));
            m3DestroyWorld(world);
        }
        printf("M3_LOWMEM_VOXEL %s: %s, landed at %d of 8 budgets\n", names[edit],
               whole ? "whole" : "BROKEN", landed);
        CHECK(whole, "a voxel edit refuses whole or leaves a tree that answers");
    }
}

static void TestReplayUnderBudget(void)
{
    // A tape whose ops take memory: a mesh floor, a voxel slab, a tree
    // rebuild and a few steps of a falling box, replayed over the
    // starting world. A replay the allocator turns down is
    // m3_errorCapacity, never m3_errorInvalid (the tape is good), and
    // backs out to the starting world, whatever op it was refused in.
    Budget budget = {-1};
    m3World* world = BudgetWorld(&budget);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    gd.position = (m3Pos3){10.0, 0.0, 0.0};
    m3BodyId slabBody = NewBody(world, &gd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-1.0, 1.0, -1.0};
    m3BodyId box = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, box, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    int32_t startSize = 0;
    uint8_t* start = TakeSnapshot(world, &startSize);

    static uint8_t tape[1 << 16];
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts");
    m3ShapeId floor = m3_nullShapeId;
    m3ShapeId slab = m3_nullShapeId;
    CHECK(CreateFloor(world, ground, &floor) == m3_success &&
              CreateSlab(world, slabBody, &slab) == m3_success,
          "the floor and the slab create on the tape");
    CHECK(m3World_RebuildBroadphase(world) == m3_success, "the tree rebuilds on the tape");
    for (int32_t i = 0; i < 10; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t tapeBytes = StopJournal(world);
    int32_t endSize = 0;
    uint8_t* end = TakeSnapshot(world, &endSize);
    CHECK(start != NULL && end != NULL && tapeBytes > 0, "the tape and both frames are taken");
    CHECK(Restore(world, start, startSize), "the world goes back to the start");

    bool refusedWhole = true;
    int32_t refusals = 0;
    m3Result result = m3_errorCapacity;
    for (int32_t n = 0; n < 1000 && result != m3_success; ++n)
    {
        budget.left = n;
        result = m3World_ReplayJournal(world, tape, tapeBytes);
        budget.left = -1;
        if (result != m3_success)
        {
            refusals += 1;
            refusedWhole =
                refusedWhole && result == m3_errorCapacity && SnapshotIs(world, start, startSize);
        }
    }
    CHECK(refusals > 0 && refusedWhole,
          "a replay the allocator turns down is m3_errorCapacity and backs out whole");
    CHECK(result == m3_success && SnapshotIs(world, end, endSize),
          "with enough memory the tape replays to the recorder's world");
    CHECK(result == m3_success && RayStrikes(world, kOverFloor, floor) &&
              RayStrikes(world, kOverSlab, slab),
          "and the replayed trees answer");
    // A tape of no op holds nothing to back out, so it replays with no
    // memory at all.
    budget.left = 0;
    CHECK(m3World_ReplayJournal(world, tape, 16) == m3_success,
          "a tape of no op replays on an allocator that refuses everything");
    budget.left = -1;
    free(end);
    free(start);
    m3DestroyWorld(world);
}

// A step whose world outgrew its scratch takes a larger block before it
// frees the old one. A growth the allocator refused once dropped the old
// block first and left a scratch with no room, and the next step's
// growth doubled 0 forever: that step never returned.
static void TestScratchGrowthUnderBudget(void)
{
    Budget budget = {-1};
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 1200;
    wd.shapeCapacity = 8;
    wd.allocator = (m3Allocator){BudgetAlloc, BudgetFree, &budget};
    m3World* world = NewWorld(&wd);
    // 1100 bodies ask a step for more than the scratch a world starts
    // with, and a box falls onto a floor.
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    for (int32_t i = 0; i < 1100; ++i)
    {
        (void)NewBody(world, &gd);
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, ground, &sd, (m3Vec3){5.0f, 0.5f, 5.0f});
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId box = NewBody(world, &bd);
    NewBoxShape(world, box, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    int32_t start = m3World_GetCounters(world).scratchCapacity;

    budget.left = 0;
    for (int32_t i = 0; i < 3; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    budget.left = -1;
    CHECK(m3World_GetCounters(world).scratchCapacity == start,
          "steps whose growth the allocator refuses return and keep the scratch they had");
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_GetCounters(world).scratchCapacity > start,
          "with memory back the next step grows the scratch");
    CHECK(m3Body_GetPosition(world, box).y < 2.5, "and the world simulates");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRestoreUnderBudget();
    TestCreateUnderBudget();
    TestVoxelEditsUnderBudget();
    TestReplayUnderBudget();
    TestScratchGrowthUnderBudget();
    if (s_failures == 0)
    {
        printf("test_lowmemory: all passed\n");
        return 0;
    }
    printf("test_lowmemory: %d FAILURES\n", s_failures);
    return 1;
}
