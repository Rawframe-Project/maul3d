// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// libFuzzer target for what a restored world does next. Any bytes are
// restored over the shared scene; a world restore accepts must step, be
// queried and snapshot like one the API built, with no sanitizer report,
// no broken assertion and no endless walk. Restore validates every link
// and cross-reference a step follows, so a crafted snapshot or replay
// file is refused before any byte of it lands.

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

static uint8_t* s_scene;
static int32_t s_sceneBytes;

static m3World* FreshWorld(void)
{
    m3WorldDef def = FuzzWorldDef();
    m3World* world = NewWorld(&def);
    Require(world != NULL);
    return world;
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
    if (m3World_Restore(world, data, (int32_t)size) == m3_success)
    {
        FuzzStep(world, 3);
        m3Pos3 lo = {-50.0, -50.0, -50.0};
        m3Pos3 hi = {50.0, 50.0, 50.0};
        m3ShapeId hits[16];
        (void)m3World_OverlapAabb(world, lo, hi, hits, 16, m3DefaultQueryFilter());
        (void)m3World_Hash(world);
        int32_t bytes = 0;
        free(SnapshotOf(world, &bytes));
    }
    m3DestroyWorld(world);
    return 0;
}
