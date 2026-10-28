// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// libFuzzer target for m3World_ReplayJournal. Any bytes, replayed as a
// tape into a fresh world, are refused with m3_errorFormat or
// m3_errorVersion and leave the world byte for byte as it was, or are
// accepted; an accepted tape replays into a second fresh world on the
// same hash, and that world steps on. Every op of the tape goes through the calls a program makes,
// so the steps it holds run on a world the API built.

#include "fuzz_scene.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static void Require(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

static uint8_t* SnapshotOf(const m3World* world, int32_t* bytesOut)
{
    int32_t size = m3World_SnapshotSize(world);
    Require(size > 0);
    uint8_t* bytes = (uint8_t*)malloc((size_t)size);
    Require(bytes != NULL);
    int32_t written = 0;
    Require(m3World_Snapshot(world, bytes, size, &written) == m3_success && written == size);
    *bytesOut = size;
    return bytes;
}

static m3World* FreshWorld(void)
{
    m3WorldDef def = FuzzWorldDef();
    m3World* world = NewWorld(&def);
    Require(world != NULL);
    return world;
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (size > INT32_MAX)
    {
        return 0;
    }
    m3World* world = FreshWorld();
    int32_t beforeBytes = 0;
    uint8_t* before = SnapshotOf(world, &beforeBytes);
    m3Result result = m3World_ReplayJournal(world, data, (int32_t)size);
    Require(result == m3_success || result == m3_errorFormat || result == m3_errorVersion);
    if (result != m3_success)
    {
        int32_t afterBytes = 0;
        uint8_t* after = SnapshotOf(world, &afterBytes);
        Require(afterBytes == beforeBytes && memcmp(after, before, (size_t)afterBytes) == 0);
        free(after);
    }
    else
    {
        m3World* twin = FreshWorld();
        Require(m3World_ReplayJournal(twin, data, (int32_t)size) == m3_success);
        Require(m3World_Hash(twin) == m3World_Hash(world));
        FuzzStep(twin, 2);
        m3DestroyWorld(twin);
    }
    free(before);
    m3DestroyWorld(world);
    return 0;
}
