// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The container gate: a replay container's session round-trips in pure
// memory onto the recorder's hash, the header's counts match the
// stream, and every kind of corruption refuses loudly without
// touching a world.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/character.h"
#include "maul3d/joint.h"
#include "maul3d/replay.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "maul3d/vehicle.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3WorldDef Def(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    return def;
}

// Record a small session; returns malloc'd container + its size.
static uint8_t* RecordSession(int32_t* outBytes, uint64_t* outHash, int32_t* outOps,
                              int32_t* outSteps)
{
    m3WorldDef def = Def();
    m3World* world = NewWorld(&def);
    int32_t snapBytes = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    Snapshot(world, snap, snapBytes);

    static uint8_t journal[131072];
    StartJournal(world, journal, (int32_t)sizeof(journal));
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &fl);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    NewBoxShape(world, crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    for (int32_t i = 0; i < 90; ++i)
    {
        if (i == 30)
        {
            m3Body_ApplyLinearImpulse(world, crate, (m3Vec3){1.5f, 0.0f, 0.0f});
        }
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = StopJournal(world);
    uint64_t final = m3World_Hash(world);

    // 4 creates + 1 impulse + 90 steps = 95 records.
    *outOps = 95;
    *outSteps = 90;
    int32_t need = m3GetEncodedReplaySize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    int32_t wrote = EncodeReplay(snap, snapBytes, journal, journalBytes, final, blob, need);
    CHECK(wrote == need, "the encode fills its own quoted size");
    *outBytes = wrote;
    *outHash = final;
    free(snap);
    m3DestroyWorld(world);
    return blob;
}

static void TestRoundTrip(void)
{
    int32_t bytes;
    uint64_t recorded;
    int32_t ops;
    int32_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);

    m3ReplayView view;
    CHECK(DecodeReplay(blob, bytes, &view), "the container decodes");
    CHECK(view.info.opCount == ops, "the op count matches the script");
    CHECK(view.info.stepCount == steps, "the step count matches the script");
    CHECK(view.finalHash == recorded, "the header carries the recorder's hash");

    m3WorldDef def = Def();
    m3World* world = NewWorld(&def);
    CHECK(Restore(world, view.snapshot, view.snapshotBytes), "the embedded snapshot restores");
    CHECK(ReplayJournal(world, view.journal, view.journalBytes), "the embedded journal replays");
    CHECK(m3World_Hash(world) == recorded, "the round trip lands on the recorder's hash");
    m3DestroyWorld(world);
    free(blob);
}

static void TestRefusals(void)
{
    int32_t bytes;
    uint64_t recorded;
    int32_t ops;
    int32_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);
    m3ReplayView view;

    // Bad magic.
    uint8_t* bad = (uint8_t*)malloc((size_t)bytes);
    memcpy(bad, blob, (size_t)bytes);
    bad[0] ^= 0xFF;
    CHECK(!DecodeReplay(bad, bytes, &view), "bad magic refuses");

    // Truncation, three depths.
    memcpy(bad, blob, (size_t)bytes);
    CHECK(!DecodeReplay(bad, bytes - 1, &view), "truncated tail refuses");
    CHECK(!DecodeReplay(bad, 39, &view), "truncated header refuses");
    CHECK(!DecodeReplay(bad, 0, &view), "empty refuses");

    // A corrupted record length inside the journal.
    memcpy(bad, blob, (size_t)bytes);
    DecodeReplay(blob, bytes, &view);
    int32_t journalOffset = (int32_t)((const uint8_t*)view.journal - blob);
    int32_t huge = 1 << 30;
    memcpy(bad + journalOffset + 16 + 4, &huge, 4); // first record's length, after the prefix
    CHECK(!DecodeReplay(bad, bytes, &view), "a corrupt record length refuses");

    // A header that lies about its counts: the step count follows the
    // prefix, the final hash and the two sizes.
    memcpy(bad, blob, (size_t)bytes);
    int32_t lie = 9999;
    memcpy(bad + 32, &lie, 4);
    CHECK(!DecodeReplay(bad, bytes, &view), "a lying step count refuses");

    // Describe on garbage: a real tape's prefix, then a record whose
    // payload runs past the end.
    m3JournalInfo info;
    uint8_t garbage[32] = {0};
    memcpy(garbage, view.journal, 16);
    const uint8_t record[16] = {7, 0, 0, 0, 99, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8};
    memcpy(garbage + 16, record, sizeof(record));
    CHECK(!DescribeJournal(garbage, (int32_t)sizeof(garbage), &info),
          "an overlong payload refuses");
    CHECK(DescribeJournal(NULL, 0, &info) == false, "null journal refuses");
    CHECK(EncodeReplay(blob, 10, NULL, 4, 0, bad, bytes) == 0, "a null journal with bytes refuses");
    CHECK(EncodeReplay(NULL, 10, NULL, 0, 0, bad, bytes) == 0, "a null snapshot refuses");
    CHECK(m3GetEncodedReplaySize(-5, 10) == 0, "negative sizes refuse");

    free(bad);
    free(blob);
}

static void TestSmallCapacity(void)
{
    int32_t bytes;
    uint64_t recorded;
    int32_t ops;
    int32_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);
    m3ReplayView view;
    CHECK(DecodeReplay(blob, bytes, &view), "decode for the capacity test");
    uint8_t* out = (uint8_t*)malloc((size_t)bytes);
    CHECK(EncodeReplay(view.snapshot, view.snapshotBytes, view.journal, view.journalBytes,
                       view.finalHash, out, bytes - 1) == 0,
          "one byte short refuses");
    free(out);
    free(blob);
}

// A session recorded WITH a vetoing pre-solve
// callback must replay to the same bits WITHOUT it: the veto
// annex (op 79) makes the tape self-sufficient.
static bool VetoAll(m3ShapeId a, m3ShapeId b, m3Pos3 point, m3Vec3 normal, void* context)
{
    (void)a;
    (void)b;
    (void)point;
    (void)normal;
    int* calls = (int*)context;
    *calls += 1;
    return false; // veto every flagged contact
}

static void TestPreSolveReplay(void)
{
    m3WorldDef def = Def();
    m3World* world = NewWorld(&def);
    int32_t snapBytes = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    Snapshot(world, snap, snapBytes);
    static uint8_t journal[262144];
    StartJournal(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floor = NewPlaneShape(world, ground, &sd, &fl);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId faller = NewBody(world, &bd);
    NewBoxShape(world, faller, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3Shape_EnablePreSolveEvents(world, floor, true);
    int calls = 0;
    m3World_SetPreSolveCallback(world, VetoAll, &calls);
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = StopJournal(world);
    uint64_t recorded = m3World_Hash(world);
    CHECK(calls > 0, "the callback actually ran");
    CHECK(m3Body_GetPosition(world, faller).y < -1.0,
          "the veto let the box fall through the floor");
    m3DestroyWorld(world);

    // The bare replay: no callback anywhere, the annex drives.
    m3World* replay = NewWorld(&def);
    CHECK(Restore(replay, snap, snapBytes), "the base snapshot restores");
    CHECK(ReplayJournal(replay, journal, journalBytes), "the veto tape replays");
    CHECK(m3World_Hash(replay) == recorded, "a bare replay lands on the recorded bits");
    m3DestroyWorld(replay);
    free(snap);
}

static void TestFuzz(void)
{
    // The fuzz rule: 200 LCG bit-mutations of a valid container
    // must never crash. Decode either refuses or yields a view; a
    // view that decodes must survive restore + replay attempts on a
    // fresh world (the atomic-replay guarantee backs out partial
    // application). The sanitizer cell turns any slip into a
    // failure.
    int32_t bytes;
    uint64_t recorded;
    int32_t ops;
    int32_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);
    uint8_t* mutant = (uint8_t*)malloc((size_t)bytes);
    // The container is snapshot-dominated, so pure-random offsets
    // almost never touch the 32-byte header or the journal tail
    // (200 random draws can hit the framing zero times).
    // Half the mutations aim at the framing on purpose; the other
    // half roam the whole container.
    m3ReplayView valid;
    CHECK(DecodeReplay(blob, bytes, &valid), "decode for the fuzz split");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint32_t rng = 87654321u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 200; ++t)
    {
        memcpy(mutant, blob, (size_t)bytes);
        rng = rng * 1664525u + 1013904223u;
        int32_t where;
        if (t % 2 == 0)
        {
            int32_t zone = 32 + (bytes - journalStart);
            int32_t pick = (int32_t)(rng % (uint32_t)zone);
            where = pick < 32 ? pick : journalStart + (pick - 32);
        }
        else
        {
            where = (int32_t)(rng % (uint32_t)bytes);
        }
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!DecodeReplay(mutant, bytes, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef def = Def();
        m3World* world = NewWorld(&def);
        if (Restore(world, view.snapshot, view.snapshotBytes))
        {
            ReplayJournal(world, view.journal, view.journalBytes);
        }
        m3DestroyWorld(world);
        survived += 1;
    }
    // Every mutation lands somewhere; the split just has to add up
    // and nothing above may crash or trip the sanitizers.
    CHECK(refused + survived == 200, "every mutant either refused or survived");
    CHECK(refused > 0, "at least some mutations hit the framing");
    free(mutant);
    free(blob);
}

int main(void)
{
    TestPreSolveReplay();
    TestRoundTrip();
    TestRefusals();
    TestSmallCapacity();
    TestFuzz();
    if (s_failures == 0)
    {
        printf("test_replayfile: all green\n");
        return 0;
    }
    printf("test_replayfile: %d failure(s)\n", s_failures);
    return 1;
}
