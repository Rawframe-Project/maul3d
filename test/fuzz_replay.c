// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// libFuzzer target for m3DecodeReplay, the M3J1 container. Any bytes are
// refused with m3_errorInvalid or decode to a view that encodes back to
// the same bytes and whose journal m3DescribeJournal counts as the view
// does. The view's snapshot then restores into a fresh world or is
// refused, and its journal replays into another fresh world or is
// refused; fuzz_snapshot and fuzz_journal hold those calls to their full
// contracts.

#include "fuzz_scene.h"

#include "maul3d/replay.h"

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
    m3ReplayView view;
    memset(&view, 0, sizeof(view));
    m3Result result = m3DecodeReplay(data, (int32_t)size, &view);
    Require(result == m3_success || result == m3_errorInvalid);
    if (result != m3_success)
    {
        return 0;
    }
    int32_t encodedSize = m3GetEncodedReplaySize(view.snapshotBytes, view.journalBytes);
    Require(encodedSize == (int32_t)size);
    uint8_t* encoded = (uint8_t*)malloc((size_t)encodedSize);
    Require(encoded != NULL);
    int32_t written = 0;
    Require(m3EncodeReplay(view.snapshot, view.snapshotBytes, view.journal, view.journalBytes,
                           view.finalHash, encoded, encodedSize, &written) == m3_success);
    Require(written == encodedSize && memcmp(encoded, data, size) == 0);
    free(encoded);

    m3JournalInfo info;
    Require(m3DescribeJournal(view.journal, view.journalBytes, &info) == m3_success);
    Require(info.opCount == view.info.opCount && info.stepCount == view.info.stepCount);

    m3World* restored = FreshWorld();
    m3Result restore = m3World_Restore(restored, view.snapshot, view.snapshotBytes);
    Require(restore == m3_success || restore == m3_errorInvalid);
    m3DestroyWorld(restored);

    m3World* replayed = FreshWorld();
    m3Result replay = m3World_ReplayJournal(replayed, view.journal, view.journalBytes);
    Require(replay == m3_success || replay == m3_errorInvalid);
    m3DestroyWorld(replayed);
    return 0;
}
