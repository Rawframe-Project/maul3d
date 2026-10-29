// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The snapshot bound with real worlds next to it. This suite links a copy
// of the library whose bound is lowered to 4 MiB (test/CMakeLists.txt), so
// a world at the bound is cheap to build. Create, the height field grid
// create, restore and the replayed grid op each refuse what would carry a
// snapshot past the bound, and take what stays within it.

#include "snapshot_bound.h"
#include "snapshot_walk.h"
#include "state_prefix.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(M3_SNAPSHOT_BOUND < M3_MAX_SNAPSHOT_BYTES,
               "the suite links the library with the lowered bound");

enum
{
    SIDE = M3_MAX_HEIGHTFIELD_SIZE,
    TAPE_BYTES = 1 << 20
};

// Zero samples for every grid: flat, so a grid's heights are its extent
// alone.
static float s_heights[SIDE * SIDE];

// A world with room for grids on one static ground body. It holds no mesh
// or hull content, so each mesh and hull slot of its snapshot's tail is
// its counts alone.
static m3World* GridWorld(m3BodyId* ground)
{
    m3WorldDef def = SmallDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 32;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    *ground = NewBody(world, &bd);
    return world;
}

// The bytes left under the bound.
static int64_t Room(const m3World* world)
{
    return M3_SNAPSHOT_BOUND - m3World_GetSnapshotSize(world);
}

static m3Result Grid(m3World* world, m3BodyId ground, int32_t nx, int32_t nz)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId id = m3_nullShapeId;
    return m3CreateHeightFieldGridShape(world, ground, &sd, s_heights, nx, nz, 1.0f, &id);
}

// The bytes a grid adds to a snapshot: its extremes and its samples.
static int64_t GridBytes(int32_t nx, int32_t nz)
{
    return 8 + 4 * (int64_t)nx * nz;
}

// Grids until the room under the bound is 24 to 2047 bytes: 255-wide ones
// while one leaves at least 24, then a 2 by 2 one if 2048 or more is left.
static void FillToTheBound(m3World* world, m3BodyId ground)
{
    for (int64_t room = Room(world); room >= 2072; room = Room(world))
    {
        int32_t rows = (int32_t)((room - 32) / 1020);
        rows = rows < SIDE ? rows : SIDE;
        if (Grid(world, ground, SIDE, rows) != m3_success)
        {
            CHECK(false, "a grid within the room is taken");
            return;
        }
    }
    if (Room(world) >= 2048)
    {
        CHECK(Grid(world, ground, 2, 2) == m3_success, "a small grid within the room is taken");
    }
    CHECK(Room(world) >= 24 && Room(world) < 2048, "the world stands next to the bound");
}

// A copy of a snapshot of world with its first from-by-from grid given n
// by n zero samples, cellSize apart. Its extent and heights stay the
// same, so every check but the size reads the copy as a state the world
// could hold.
static uint8_t* ResizeGrid(const m3World* world, const uint8_t* snap, int32_t size, int32_t from,
                           int32_t n, float cellSize, int32_t* sizeOut)
{
    *sizeOut = 0;
    size_t at = M3_SNAPSHOT_HEADER_BYTES + (size_t)m3StateSnapshotBytes(world) +
                8 * (size_t)world->meshes.meshCapacity + 16 * (size_t)world->shapes.shapeCapacity;
    size_t found = 0;
    size_t foundBytes = 0;
    for (int32_t slot = 0; slot < world->shapes.shapeCapacity; ++slot)
    {
        int32_t nx;
        int32_t nz;
        memcpy(&nx, snap + at, sizeof(nx));
        memcpy(&nz, snap + at + 4, sizeof(nz));
        size_t bytes = 12 + (nx > 0 ? 8 + 4 * (size_t)nx * (size_t)nz : 0);
        if (foundBytes == 0 && nx == from && nz == from)
        {
            found = at;
            foundBytes = bytes;
        }
        at += bytes;
    }
    CHECK(at == (size_t)size && foundBytes > 0, "the tail parses to the snapshot's end");
    if (at != (size_t)size || foundBytes == 0)
    {
        return NULL;
    }
    size_t resized = 20 + 4 * (size_t)n * (size_t)n;
    size_t total = (size_t)size - foundBytes + resized;
    uint8_t* out = (uint8_t*)calloc(total, 1);
    memcpy(out, snap, found);
    memcpy(out + found, &n, sizeof(n));
    memcpy(out + found + 4, &n, sizeof(n));
    memcpy(out + found + 8, &cellSize, sizeof(cellSize)); // the extremes and samples stay zero
    memcpy(out + found + resized, snap + found + foundBytes, (size_t)size - found - foundBytes);
    *sizeOut = (int32_t)total;
    return out;
}

static uint8_t* TakeSnapshot(const m3World* world, int32_t* sizeOut)
{
    *sizeOut = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)*sizeOut);
    CHECK(snap != NULL && Snapshot(world, snap, *sizeOut) == *sizeOut, "the world snapshots");
    return snap;
}

static void TestCreateAtTheBound(void)
{
    // At 4 MiB the bound counted on mesh slots and on shape slots is a
    // world small enough to create on either axis.
    m3WorldDef def = SmallDef();
    int32_t most =
        MostSlots(&def.meshCapacity, &def, MeshContentMost(), HullContentMost(), M3_SNAPSHOT_BOUND);
    def.meshCapacity = most;
    m3World* world = NULL;
    CHECK(m3CreateWorld(&def, &world) == m3_success,
          "a world of mesh slots at the bound is created");
    m3DestroyWorld(world);
    def.meshCapacity = most + 1;
    def.allocator = (m3Allocator){RefuseAlloc, RefuseFree, NULL};
    s_asked = 0;
    CHECK(m3CreateWorld(&def, &world) == m3_errorCapacity && world == NULL && s_asked == 0,
          "one mesh slot more is refused before anything is allocated");

    def = SmallDef();
    most = MostSlots(&def.shapeCapacity, &def, HullContentMost(), MeshContentMost(),
                     M3_SNAPSHOT_BOUND);
    def.shapeCapacity = most;
    CHECK(m3CreateWorld(&def, &world) == m3_success,
          "a world of shape slots at the bound is created");
    m3DestroyWorld(world);
    def.shapeCapacity = most + 1;
    def.allocator = (m3Allocator){RefuseAlloc, RefuseFree, NULL};
    s_asked = 0;
    CHECK(m3CreateWorld(&def, &world) == m3_errorCapacity && world == NULL && s_asked == 0,
          "one shape slot more is refused before anything is allocated");
}

static void TestGridAtTheBound(void)
{
    m3BodyId ground;
    m3World* world = GridWorld(&ground);
    FillToTheBound(world, ground);
    // A 2 by rows grid fits the room and one more row crosses it by at
    // most 8 bytes.
    int64_t room = Room(world);
    int32_t rows = (int32_t)((room - 8) / 8);
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = m3World_GetCounters(world).misuse;
    CHECK(Grid(world, ground, 2, rows + 1) == m3_errorCapacity,
          "a grid that would carry the snapshot past the bound is refused");
    CHECK(m3World_Hash(world) == hash && Room(world) == room &&
              m3World_GetCounters(world).misuse == misuse,
          "with the world as it was, and no misuse counted");
    CHECK(Grid(world, ground, 2, rows) == m3_success && Room(world) == room - GridBytes(2, rows),
          "a grid that keeps the snapshot within the bound is taken");
    m3DestroyWorld(world);
}

static void TestRestorePastTheBound(void)
{
    // A 3 by 3 grid of cell 1 and a 129 by 129 grid of cell 1/64 span the
    // same 2 m, so a snapshot that gives the first the second's samples is
    // one the world could hold, 66,528 bytes larger.
    m3BodyId ground;
    m3World* roomy = GridWorld(&ground);
    CHECK(Grid(roomy, ground, 3, 3) == m3_success, "the small grid is taken");
    int32_t size = 0;
    uint8_t* snap = TakeSnapshot(roomy, &size);
    int32_t grownSize = 0;
    uint8_t* grown = ResizeGrid(roomy, snap, size, 3, 129, 1.0f / 64.0f, &grownSize);
    CHECK(grown != NULL && Restore(roomy, grown, grownSize) &&
              m3World_GetSnapshotSize(roomy) == grownSize,
          "the grown grid restores into a world with room for it");
    free(grown);
    free(snap);
    m3DestroyWorld(roomy);

    m3World* world = GridWorld(&ground);
    CHECK(Grid(world, ground, 3, 3) == m3_success, "the small grid is taken");
    FillToTheBound(world, ground);
    snap = TakeSnapshot(world, &size);
    grown = ResizeGrid(world, snap, size, 3, 129, 1.0f / 64.0f, &grownSize);
    CHECK(grownSize > M3_SNAPSHOT_BOUND, "the grown snapshot passes the bound");
    uint64_t hash = m3World_Hash(world);
    CHECK(grown != NULL && m3World_Restore(world, grown, grownSize) == m3_errorFormat,
          "a snapshot past the bound is refused");
    CHECK(m3World_Hash(world) == hash && m3World_GetSnapshotSize(world) == size,
          "with the world as it was");
    free(grown);
    free(snap);
    m3DestroyWorld(world);
}

static void TestReplayPastTheBound(void)
{
    // A twin of a world at the bound, with one full grid shrunk to 2 by 2
    // samples over the same 254 m, has the room to take a full grid more.
    // The tape of that create replays on another twin and is refused on the
    // world at the bound, which backs out to the state it had.
    m3BodyId ground;
    m3World* full = GridWorld(&ground);
    FillToTheBound(full, ground);
    int32_t size = 0;
    uint8_t* snap = TakeSnapshot(full, &size);
    int32_t shrunkSize = 0;
    uint8_t* shrunk = ResizeGrid(full, snap, size, SIDE, 2, (float)(SIDE - 1), &shrunkSize);

    m3BodyId twinGround;
    m3World* twin = GridWorld(&twinGround);
    CHECK(shrunk != NULL && Restore(twin, shrunk, shrunkSize), "the shrunk twin restores");
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    CHECK(tape != NULL && StartJournal(twin, tape, TAPE_BYTES), "the journal starts");
    CHECK(Grid(twin, ground, SIDE, SIDE) == m3_success, "the twin takes a full grid");
    int32_t tapeBytes = StopJournal(twin);
    CHECK(tapeBytes > 0, "the create is recorded");

    m3World* other = GridWorld(&twinGround);
    CHECK(Restore(other, shrunk, shrunkSize) && ReplayJournal(other, tape, tapeBytes) &&
              m3World_Hash(other) == m3World_Hash(twin),
          "the tape replays on another twin");

    uint64_t hash = m3World_Hash(full);
    CHECK(m3World_ReplayJournal(full, tape, tapeBytes) == m3_errorFormat,
          "a replayed grid that would carry the snapshot past the bound is refused");
    CHECK(m3World_Hash(full) == hash && m3World_GetSnapshotSize(full) == size,
          "and the world backs out to the state it had");
    free(tape);
    free(shrunk);
    free(snap);
    m3DestroyWorld(other);
    m3DestroyWorld(twin);
    m3DestroyWorld(full);
}

int main(void)
{
    TestCreateAtTheBound();
    TestGridAtTheBound();
    TestRestorePastTheBound();
    TestReplayPastTheBound();
    if (s_failures == 0)
    {
        printf("test_snapshot_lowbound: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
