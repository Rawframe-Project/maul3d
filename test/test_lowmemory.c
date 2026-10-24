// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The low-memory gate: under a host allocator that runs out after a
// given number of blocks, a call either refuses with m3_errorCapacity
// and leaves the world as it was, or does all of its work. Each test
// sweeps the budget from zero until the call succeeds, so every
// allocation the call makes is refused once.

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

// Sweeps a create from a budget of zero blocks up: each refusal must be
// m3_errorCapacity with the world unchanged and no misuse counted, and
// the first success must give a shape whose tree answers.
static void SweepCreate(bool slabNotFloor, m3Pos3 over, const char* refusedMsg,
                        const char* createdMsg)
{
    Budget budget = {-1};
    m3World* world = BudgetWorld(&budget);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = slabNotFloor ? (m3Pos3){10.0, 0.0, 0.0} : (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId body = NewBody(world, &gd);
    int32_t size = 0;
    uint8_t* before = TakeSnapshot(world, &size);
    uint64_t misuse = m3World_GetCounters(world).misuse;
    bool refusedWhole = before != NULL;
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
                           SnapshotIs(world, before, size) &&
                           m3World_GetCounters(world).misuse == misuse;
        }
    }
    CHECK(refusals > 0 && refusedWhole, refusedMsg);
    CHECK(result == m3_success && RayStrikes(world, over, shape), createdMsg);
    free(before);
    m3DestroyWorld(world);
}

static void TestCreateUnderBudget(void)
{
    SweepCreate(false, kOverFloor, "a mesh create the allocator refuses leaves the world as it was",
                "the created mesh's tree answers");
    SweepCreate(true, kOverSlab, "a voxel create the allocator refuses leaves the world as it was",
                "the created chunk's tree answers");
}

int main(void)
{
    TestRestoreUnderBudget();
    TestCreateUnderBudget();
    if (s_failures == 0)
    {
        printf("test_lowmemory: all passed\n");
        return 0;
    }
    printf("test_lowmemory: %d FAILURES\n", s_failures);
    return 1;
}
