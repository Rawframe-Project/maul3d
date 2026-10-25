// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Replay of the voxel edits asks the cell check their public calls ask: a
// journal recorded at a cell in the chunk replays, and the same journal
// with that cell moved out of the chunk on any axis fails to replay and
// leaves the world as it was.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DIM 16

typedef enum Edit
{
    edit_set,
    edit_clear,
    edit_fill,
} Edit;

static uint8_t s_journal[1 << 16];

typedef struct Chunk
{
    m3World* world;
    m3ShapeId shape;
} Chunk;

// A full chunk on a static body, so every edit lands.
static Chunk BuildChunk(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    Chunk c;
    c.world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId body = NewBody(c.world, &bd);
    uint8_t voxels[DIM * DIM * DIM];
    memset(voxels, 1, sizeof(voxels));
    m3ShapeDef sd = m3DefaultShapeDef();
    c.shape = NewVoxelChunkShape(c.world, body, &sd, voxels, NULL, 0.5f);
    return c;
}

static void Apply(Chunk* c, Edit edit, int32_t x, int32_t y, int32_t z)
{
    if (edit == edit_set)
    {
        m3VoxelChunk_SetVoxel(c->world, c->shape, x, y, z, 7);
    }
    else if (edit == edit_clear)
    {
        m3VoxelChunk_ClearVoxel(c->world, c->shape, x, y, z);
    }
    else
    {
        m3VoxelChunk_SetFill(c->world, c->shape, x, y, z, 99);
    }
}

// Replaces the one occurrence of from's bytes in the record with to's.
static bool Patch(int32_t bytes, int32_t from, int32_t to)
{
    int32_t at = -1;
    for (int32_t i = 0; i + 4 <= bytes; ++i)
    {
        if (memcmp(s_journal + i, &from, 4) == 0)
        {
            if (at >= 0)
            {
                return false;
            }
            at = i;
        }
    }
    if (at < 0)
    {
        return false;
    }
    memcpy(s_journal + at, &to, 4);
    return true;
}

static void CheckEdit(const char* name, Edit edit)
{
    // Distinct coordinates, each a value nothing else in the journal holds.
    const int32_t cell[3] = {10, 4, 6};
    const int32_t outside[] = {DIM, -1, 1 << 24, -(1 << 24)};
    for (int32_t axis = 0; axis < 3; ++axis)
    {
        for (size_t k = 0; k < sizeof(outside) / sizeof(outside[0]); ++k)
        {
            char what[160];
            Chunk c = BuildChunk();
            CHECK(m3Shape_IsValid(c.world, c.shape), "the chunk builds");
            int32_t snapshotBytes = m3World_SnapshotSize(c.world);
            uint8_t* snapshot = (uint8_t*)malloc((size_t)snapshotBytes);
            CHECK(snapshot != NULL, "the snapshot buffer");
            Snapshot(c.world, snapshot, snapshotBytes);
            StartJournal(c.world, s_journal, (int32_t)sizeof(s_journal));
            Apply(&c, edit, cell[0], cell[1], cell[2]);
            int32_t bytes = StopJournal(c.world);
            m3DestroyWorld(c.world);

            m3WorldDef def = m3DefaultWorldDef();
            m3World* replayed = NewWorld(&def);
            CHECK(Restore(replayed, snapshot, snapshotBytes), "the snapshot restores");
            snprintf(what, sizeof(what), "%s: the journal replays as recorded", name);
            CHECK(ReplayJournal(replayed, s_journal, bytes), what);
            m3DestroyWorld(replayed);

            snprintf(what, sizeof(what), "%s: the record holds the cell once", name);
            CHECK(Patch(bytes, cell[axis], outside[k]), what);
            replayed = NewWorld(&def);
            CHECK(Restore(replayed, snapshot, snapshotBytes), "the snapshot restores");
            uint64_t hash = m3World_Hash(replayed);
            snprintf(what, sizeof(what), "%s: replay refuses axis %d at %d", name, (int)axis,
                     (int)outside[k]);
            CHECK(!ReplayJournal(replayed, s_journal, bytes), what);
            snprintf(what, sizeof(what), "%s: the refused replay changes nothing", name);
            CHECK(m3World_Hash(replayed) == hash, what);
            m3DestroyWorld(replayed);
            free(snapshot);
        }
    }
}

int main(void)
{
    CheckEdit("set voxel", edit_set);
    CheckEdit("clear voxel", edit_clear);
    CheckEdit("set fill", edit_fill);
    if (s_failures == 0)
    {
        printf("test_replay_voxel: all green\n");
        return 0;
    }
    printf("test_replay_voxel: %d failure(s)\n", s_failures);
    return 1;
}
