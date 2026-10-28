// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A soft lattice's counts are each within the particle cap, like their
// product: a def whose counts pass it alone refuses, counts once and
// changes nothing, through the public create and through replay.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/softbody.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t s_journal[1 << 16];

// Counts whose product, taken in 64 bits, is not what it seems.
static const int32_t s_wide[][3] = {
    {1 << 21, 1 << 21, 1 << 22},
    {1 << 30, 1 << 30, 1 << 30},
    {M3_MAX_SOFTBODY_PARTICLES + 1, 1, 1},
    {1, 1, 0x7FFFFFFF},
};

static m3World* NewSoftWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.softBodyCapacity = 2;
    return NewWorld(&def);
}

static void TestPublicCreate(void)
{
    for (size_t i = 0; i < sizeof(s_wide) / sizeof(s_wide[0]); ++i)
    {
        m3World* world = NewSoftWorld();
        uint64_t misuse = m3World_GetCounters(world).misuse;
        uint64_t hash = m3World_Hash(world);
        m3SoftBodyDef def = m3DefaultSoftBodyDef();
        def.countX = s_wide[i][0];
        def.countY = s_wide[i][1];
        def.countZ = s_wide[i][2];
        m3SoftBodyId id = m3_nullSoftBodyId;
        CHECK(m3CreateSoftBody(world, &def, &id) != m3_success, "the wide lattice refuses");
        CHECK(m3World_GetCounters(world).misuse == misuse + 1, "the refusal counts once");
        CHECK(m3World_Hash(world) == hash, "the refusal changes nothing");
        m3DestroyWorld(world);
    }
}

// Replaces the one occurrence of from's bytes in the journal with to's.
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

static void TestReplay(void)
{
    // Counts that occur nowhere else in the journal: 7 by 5 by 3.
    const int32_t recorded[3] = {7, 5, 3};
    for (size_t i = 0; i < sizeof(s_wide) / sizeof(s_wide[0]); ++i)
    {
        m3World* world = NewSoftWorld();
        int32_t snapshotBytes = m3World_SnapshotSize(world);
        uint8_t* snapshot = (uint8_t*)malloc((size_t)snapshotBytes);
        CHECK(snapshot != NULL, "the snapshot buffer");
        Snapshot(world, snapshot, snapshotBytes);
        StartJournal(world, s_journal, (int32_t)sizeof(s_journal));
        m3SoftBodyDef def = m3DefaultSoftBodyDef();
        def.countX = recorded[0];
        def.countY = recorded[1];
        def.countZ = recorded[2];
        CHECK(m3SoftBody_IsValid(world, NewSoftBody(world, &def)), "the lattice builds");
        int32_t bytes = StopJournal(world);
        m3DestroyWorld(world);

        world = NewSoftWorld();
        CHECK(Restore(world, snapshot, snapshotBytes), "the snapshot restores");
        CHECK(ReplayJournal(world, s_journal, bytes), "the journal replays as recorded");
        m3DestroyWorld(world);

        for (int32_t axis = 0; axis < 3; ++axis)
        {
            CHECK(Patch(bytes, recorded[axis], s_wide[i][axis]),
                  "the journal holds each count once");
        }
        world = NewSoftWorld();
        CHECK(Restore(world, snapshot, snapshotBytes), "the snapshot restores");
        uint64_t hash = m3World_Hash(world);
        CHECK(!ReplayJournal(world, s_journal, bytes), "replay refuses the wide lattice");
        CHECK(m3World_Hash(world) == hash, "the refused replay changes nothing");
        m3DestroyWorld(world);
        free(snapshot);
    }
}

int main(void)
{
    TestPublicCreate();
    TestReplay();
    if (s_failures == 0)
    {
        printf("test_lattice_counts: all green\n");
        return 0;
    }
    printf("test_lattice_counts: %d failure(s)\n", s_failures);
    return 1;
}
