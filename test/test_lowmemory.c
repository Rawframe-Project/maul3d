// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The low-memory gate: under a host allocator that runs out after a
// given number of blocks, a call either refuses with m3_errorCapacity
// and leaves the world as it was, or does all of its work. Each test
// sweeps the budget from zero until the call succeeds, so every
// allocation the call makes is refused once. A replay refused memory
// says so with m3_errorCapacity and backs out whole, a world create
// leaves no block behind, and a step asks the allocator for nothing.

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
    int32_t live; // blocks handed out and not yet given back
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
    void* memory = malloc(bytes);
    budget->live += memory != NULL ? 1 : 0;
    return memory;
}

static void BudgetFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    Budget* budget = (Budget*)context;
    budget->live -= memory != NULL ? 1 : 0;
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
    m3CastHit hit = {0};
    bool found = m3World_CastRayClosest(world, origin, (m3Vec3){0.0f, -10.0f, 0.0f},
                                        m3DefaultQueryFilter(), &hit);
    return found && hit.shapeId.index1 == shape.index1 &&
           hit.shapeId.generation == shape.generation;
}

static const m3Pos3 kOverFloor = {0.5, 5.0, 0.5};
static const m3Pos3 kOverSlab = {12.0, 5.0, 2.0};

// Whether a world's snapshot equals the given bytes.
static bool SnapshotIs(const m3World* world, const uint8_t* bytes, int64_t size)
{
    if (m3World_GetSnapshotSize(world) != size)
    {
        return false;
    }
    uint8_t* now = (uint8_t*)malloc((size_t)size);
    bool same =
        now != NULL && Snapshot(world, now, size) == size && memcmp(now, bytes, (size_t)size) == 0;
    free(now);
    return same;
}

static uint8_t* TakeSnapshot(const m3World* world, int64_t* sizeOut)
{
    *sizeOut = m3World_GetSnapshotSize(world);
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
    Budget budget = {-1, 0};
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

    int64_t size = 0;
    uint8_t* first = TakeSnapshot(world, &size);
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int64_t laterSize = 0;
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
    Budget budget = {-1, 0};
    m3BodyId body;
    m3World* world = CreateWorld(&budget, slabNotFloor, false, &body);
    Budget plenty = {-1, 0};
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
    int64_t size = 0;
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
    Budget budget = {-1, 0};
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
        m3VoxelChunkShape_SetVoxel(world, slab, 6, 1, 6, 7);
        break;
    case 1:
        m3VoxelChunkShape_ClearVoxel(world, slab, 0, 0, 0);
        break;
    case 2:
        (void)m3VoxelChunkShape_ClearBox(world, slab, boxLo, boxHi);
        break;
    case 3:
        m3VoxelChunkShape_ClearVoxel(world, slab, 3, 1, 3); // the pillar's top fractures
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
            Budget budget = {-1, 0};
            m3BodyId body;
            m3World* world = CreateWorld(&budget, true, false, &body);
            m3ShapeId slab = m3_nullShapeId;
            CHECK(CreateSlab(world, body, &slab) == m3_success, "the slab creates");
            m3VoxelChunkShape_SetVoxel(world, slab, 3, 1, 3, 1);
            m3VoxelChunkShape_SetVoxel(world, slab, 3, 2, 3, 1);
            uint64_t hash = m3World_Hash(world);
            uint64_t misuse = m3World_GetCounters(world).misuse;
            CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts");
            budget.left = n;
            VoxelEdit(world, slab, edit);
            budget.left = -1;
            int64_t journaled = StopJournal(world);
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
    Budget budget = {-1, 0};
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
    int64_t startSize = 0;
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
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int64_t tapeBytes = StopJournal(world);
    int64_t endSize = 0;
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

// A world create takes every block it will hold, the step scratch
// included, and a refusal at any of them gives back the ones it took.
static void TestWorldCreateUnderBudget(void)
{
    bool refusedWhole = true;
    int32_t refusals = 0;
    m3Result result = m3_errorCapacity;
    for (int32_t n = 0; n < 10000 && result != m3_success; ++n)
    {
        Budget budget = {n, 0};
        m3WorldDef wd = m3DefaultWorldDef();
        wd.allocator = (m3Allocator){BudgetAlloc, BudgetFree, &budget};
        m3World* world = NULL;
        result = m3CreateWorld(&wd, &world);
        if (result != m3_success)
        {
            refusals += 1;
            refusedWhole =
                refusedWhole && result == m3_errorCapacity && world == NULL && budget.live == 0;
        }
        else
        {
            m3DestroyWorld(world);
            CHECK(budget.live == 0, "a destroyed world gives back every block");
        }
    }
    CHECK(refusals > 0 && refusedWhole,
          "a world create the allocator refuses is m3_errorCapacity and leaves no block behind");
    CHECK(result == m3_success, "with memory enough the world is created");
}

// A busy world of every kind of step work: a pile on a height field grid
// and a mesh, joints, a vehicle, a character and a water volume.
static m3World* BusyWorld(Budget* budget)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 256;
    wd.shapeCapacity = 256;
    wd.allocator = (m3Allocator){BudgetAlloc, BudgetFree, budget};
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float heights[17 * 17];
    for (int32_t i = 0; i < 17 * 17; ++i)
    {
        heights[i] = 0.05f * (float)((i * 7) % 5);
    }
    (void)NewHeightFieldShape(world, ground, &sd, heights, 17, 17, 1.0f);
    gd.position = (m3Pos3){-10.0, 0.0, 0.0};
    m3ShapeId floor = m3_nullShapeId;
    (void)CreateFloor(world, NewBody(world, &gd), &floor);
    m3BodyId previous = ground;
    for (int32_t i = 0; i < 60; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){1.0 + 1.2 * (double)(i % 6), 0.6 + 0.6 * (double)(i / 36),
                               1.0 + 1.2 * (double)((i / 6) % 6)};
        m3BodyId box = NewBody(world, &bd);
        NewBoxShape(world, box, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
        if (i < 4)
        {
            m3JointDef jd = m3DefaultJointDef();
            jd.type = m3_sphericalJoint;
            jd.bodyIdA = previous;
            jd.bodyIdB = box;
            (void)NewJoint(world, &jd);
            previous = box;
        }
    }
    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){-10.0, 1.0, 0.0};
    m3BodyId chassis = NewBody(world, &cd);
    NewBoxShape(world, chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
    }
    (void)NewVehicle(world, &vd);
    m3CharacterDef chd = m3DefaultCharacterDef();
    chd.position = (m3Pos3){-12.0, 1.0, 0.0};
    (void)NewCharacter(world, &chd);
    m3WaterVolumeDef water = m3DefaultWaterVolumeDef();
    water.lo = (m3Pos3){0.0, -1.0, 0.0};
    water.hi = (m3Pos3){4.0, 0.5, 4.0};
    (void)NewWaterVolume(world, &water);
    return world;
}

// Once a world is created, no step asks its allocator: steps under an
// allocator that refuses every block all run, and land where steps with
// memory to spare land. A step that needed more scratch once grew it
// between steps, and a refused growth stalled the world.
static void TestStepsNeverAllocate(void)
{
    Budget plenty = {-1, 0};
    m3World* reference = BusyWorld(&plenty);
    Budget none = {-1, 0};
    m3World* world = BusyWorld(&none);
    int32_t scratch = m3World_GetCounters(world).scratchCapacity;
    none.left = 0;
    bool stepped = true;
    for (int32_t i = 0; i < 120; ++i)
    {
        stepped = stepped && m3World_Step(world, 1.0f / 60.0f, 4) == m3_success;
        (void)m3World_Step(reference, 1.0f / 60.0f, 4);
    }
    none.left = -1;
    CHECK(stepped, "every step runs under an allocator that refuses everything");
    CHECK(m3World_Hash(world) == m3World_Hash(reference),
          "and lands where steps with memory to spare land");
    CHECK(m3World_GetCounters(world).scratchCapacity == scratch &&
              m3World_GetCounters(world).scratchPeak > 0 &&
              m3World_GetCounters(world).scratchPeak <= scratch,
          "on the scratch the world was created with");
    m3DestroyWorld(reference);
    m3DestroyWorld(world);

    // A lattice that fills nearly every pair the table holds, each with a
    // touching manifold, colored into all 16 contact colors and packed
    // into lane blocks: 256 small dynamic spheres on a grid, each inside
    // 16 of 289 large static spheres on the grid offset by half a cell.
    // It takes nearly all the scratch the world was sized for, and still
    // steps.
    Budget latticeBudget = {-1, 0};
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 257;
    wd.shapeCapacity = 545; // 4360 pairs
    wd.allocator = (m3Allocator){BudgetAlloc, BudgetFree, &latticeBudget};
    m3World* lattice = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(lattice, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    for (int32_t k = 0; k < 17 * 17; ++k)
    {
        m3Sphere big = {{(m3real)(k % 17) - 0.5f, 0.0f, (m3real)(k / 17) - 0.5f}, 2.3f};
        NewSphereShape(lattice, ground, &sd, &big);
    }
    m3Sphere small = {{0.0f, 0.0f, 0.0f}, 0.1f};
    for (int32_t k = 0; k < 16 * 16; ++k)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.gravityScale = 0.0f;
        bd.position = (m3Pos3){(double)(k % 16), 0.0, (double)(k / 16)};
        NewSphereShape(lattice, NewBody(lattice, &bd), &sd, &small);
    }
    latticeBudget.left = 0;
    m3Result packed = m3World_Step(lattice, 1.0f / 60.0f, 4);
    latticeBudget.left = -1;
    m3Counters counters = m3World_GetCounters(lattice);
    CHECK(packed == m3_success && counters.contactCount == 62 * 62 && counters.colorCount == 16,
          "a step of 3844 touching pairs in 16 colors runs on the world's scratch");
    CHECK(counters.scratchPeak > counters.scratchCapacity / 4 * 3,
          "and takes most of it, with the table 88 percent full");
    m3DestroyWorld(lattice);
}

int main(void)
{
    TestRestoreUnderBudget();
    TestCreateUnderBudget();
    TestVoxelEditsUnderBudget();
    TestReplayUnderBudget();
    TestWorldCreateUnderBudget();
    TestStepsNeverAllocate();
    if (s_failures == 0)
    {
        printf("test_lowmemory: all passed\n");
        return 0;
    }
    printf("test_lowmemory: %d FAILURES\n", s_failures);
    return 1;
}
