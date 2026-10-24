// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// libFuzzer target for m3World_Restore. Any bytes are refused with
// m3_errorInvalid and leave the world byte for byte as it was, or are
// accepted; a world restored from them writes a snapshot that restores
// into a fresh world and writes the same bytes again. The restored world
// is not stepped: restore checks every index, count, flag and float, not
// every structural invariant, so stepping is promised only for snapshots
// this engine wrote (world.h).

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

// The world each input lands on: the shared scene after a short
// session, so a refusal has real state to leave alone.
static uint8_t* s_scene;
static int32_t s_sceneBytes;

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

int LLVMFuzzerInitialize(int* argc, char*** argv);
int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerInitialize(int* argc, char*** argv)
{
    (void)argc;
    (void)argv;
    m3World* world = FreshWorld();
    FuzzScene scene;
    Require(FuzzBuild(world, &scene) && FuzzCountsHold(world, &scene));
    FuzzSession(world, &scene);
    s_scene = SnapshotOf(world, &s_sceneBytes);
    m3DestroyWorld(world);
    return 0;
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (size > INT32_MAX)
    {
        return 0;
    }
    m3World* world = FreshWorld();
    Require(m3World_Restore(world, s_scene, s_sceneBytes) == m3_success);
    m3Result result = m3World_Restore(world, data, (int32_t)size);
    Require(result == m3_success || result == m3_errorInvalid);
    int32_t bytes = 0;
    uint8_t* written = SnapshotOf(world, &bytes);
    if (result != m3_success)
    {
        Require(bytes == s_sceneBytes && memcmp(written, s_scene, (size_t)bytes) == 0);
    }
    else
    {
        m3World* again = FreshWorld();
        Require(m3World_Restore(again, written, bytes) == m3_success);
        int32_t againBytes = 0;
        uint8_t* rewritten = SnapshotOf(again, &againBytes);
        Require(againBytes == bytes && memcmp(rewritten, written, (size_t)bytes) == 0);
        free(rewritten);
        m3DestroyWorld(again);
    }
    free(written);
    m3DestroyWorld(world);
    return 0;
}
