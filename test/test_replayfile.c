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
static uint8_t* RecordSession(int64_t* outBytes, uint64_t* outHash, int64_t* outOps,
                              int64_t* outSteps)
{
    m3WorldDef def = Def();
    m3World* world = NewWorld(&def);
    int64_t snapBytes = m3World_GetSnapshotSize(world);
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
    int64_t journalBytes = StopJournal(world);
    uint64_t final = m3World_Hash(world);

    // 4 creates + 1 impulse + 90 steps = 95 records.
    *outOps = 95;
    *outSteps = 90;
    int64_t need = m3GetEncodedReplaySize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    int64_t wrote = EncodeReplay(snap, snapBytes, journal, journalBytes, final, blob, need);
    CHECK(wrote == need, "the encode fills its own quoted size");
    *outBytes = wrote;
    *outHash = final;
    free(snap);
    m3DestroyWorld(world);
    return blob;
}

static void TestRoundTrip(void)
{
    int64_t bytes;
    uint64_t recorded;
    int64_t ops;
    int64_t steps;
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
    int64_t bytes;
    uint64_t recorded;
    int64_t ops;
    int64_t steps;
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
    int64_t bytes;
    uint64_t recorded;
    int64_t ops;
    int64_t steps;
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

// What a failure leaves in a size output: the world's calls clear it before
// any check, a null world's included, and the encoder, which takes no
// owner, leaves it as it was.
static void TestSizeOutputsOnFailure(void)
{
    int64_t bytes;
    uint64_t recorded;
    int64_t ops;
    int64_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);
    m3ReplayView view;
    CHECK(DecodeReplay(blob, bytes, &view), "decode for the size output test");
    uint8_t* out = (uint8_t*)malloc((size_t)bytes);
    int64_t written = 1234;
    CHECK(m3EncodeReplay(view.snapshot, view.snapshotBytes, view.journal, view.journalBytes,
                         view.finalHash, out, bytes - 1, &written) == m3_errorCapacity &&
              written == 1234,
          "a short encode leaves the size as it was");
    CHECK(m3EncodeReplay(NULL, view.snapshotBytes, view.journal, view.journalBytes, view.finalHash,
                         out, bytes, &written) == m3_errorInvalid &&
              written == 1234,
          "and so does a refused one");

    written = 1234;
    memset(out, 0x5A, (size_t)bytes);
    bool zeroed = m3World_Snapshot(NULL, out, bytes, &written) == m3_errorInvalid && written == 0;
    for (int64_t k = 0; k < bytes; ++k)
    {
        zeroed = zeroed && out[k] == 0;
    }
    CHECK(zeroed, "a snapshot of no world leaves zero and zeroes its buffer");
    written = 1234;
    CHECK(m3World_StopJournal(NULL, &written) == m3_errorInvalid && written == 0,
          "and so does a stop of no world's journal");
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

// A journal stop made from the world's own pre-solve callback, mid-step:
// refused as busy, it leaves the caller's size as it was.
typedef struct BusyStop
{
    m3World* world;
    int64_t bytes;
    m3Result result;
    int calls;
} BusyStop;

static bool StopFromCallback(m3ShapeId a, m3ShapeId b, m3Pos3 point, m3Vec3 normal, void* context)
{
    (void)a;
    (void)b;
    (void)point;
    (void)normal;
    BusyStop* stop = (BusyStop*)context;
    if (stop->calls++ == 0)
    {
        stop->bytes = 1234;
        stop->result = m3World_StopJournal(stop->world, &stop->bytes);
    }
    return true;
}

static void TestBusyStopKeepsSize(void)
{
    m3WorldDef def = Def();
    m3World* world = NewWorld(&def);
    static uint8_t journal[65536];
    CHECK(StartJournal(world, journal, (int32_t)sizeof(journal)), "the journal starts");
    m3BodyDef gd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId floor = NewBoxShape(world, NewBody(world, &gd), &sd, (m3Vec3){5.0f, 0.5f, 5.0f});
    m3Shape_EnablePreSolveEvents(world, floor, true);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.9, 0.0};
    NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    BusyStop stop = {world, 0, m3_success, 0};
    m3World_SetPreSolveCallback(world, StopFromCallback, &stop);
    for (int32_t i = 0; i < 4 && stop.calls == 0; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(stop.calls > 0, "the pre-solve callback runs");
    CHECK(stop.result == m3_errorBusy && stop.bytes == 1234,
          "a stop from the world's own callback is refused busy and keeps the size");
    m3World_SetPreSolveCallback(world, NULL, NULL);
    int64_t bytes = 99;
    CHECK(m3World_StopJournal(world, &bytes) == m3_success && bytes > 16,
          "the journal it left running stops after the step");
    m3DestroyWorld(world);
}

static void TestPreSolveReplay(void)
{
    m3WorldDef def = Def();
    m3World* world = NewWorld(&def);
    int64_t snapBytes = m3World_GetSnapshotSize(world);
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
    int64_t journalBytes = StopJournal(world);
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
    int64_t bytes;
    uint64_t recorded;
    int64_t ops;
    int64_t steps;
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
            int64_t zone = 32 + (bytes - journalStart);
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

// Sizes are 64-bit with no cap of their own. A world with more mesh slots
// than the old 2 GiB cap let in creates; a caller's capacity past
// INT32_MAX takes a snapshot and a tape, of which only what they need is
// written; the container's sizes add up past INT32_MAX and its header
// fields are read whole. On a 32-bit target a size past SIZE_MAX is a
// caller's bug, refused before any pointer is formed.
static void TestSizesPastInt32(void)
{
    m3WorldDef wide = m3DefaultWorldDef();
    wide.meshCapacity = 2000; // the old cap refused 1,639 and up
    m3World* world = NULL;
    CHECK(m3CreateWorld(&wide, &world) == m3_success,
          "a world whose meshes could pass 2 GiB at their largest creates");
    m3DestroyWorld(world);

    m3WorldDef def = Def();
    world = NewWorld(&def);
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    int64_t written = 0;
    CHECK(snap != NULL &&
              m3World_Snapshot(world, snap, (int64_t)INT32_MAX + 1, &written) == m3_success &&
              written == size,
          "a capacity past INT32_MAX takes the snapshot, which writes its own size");
    static uint8_t tape[4096];
    CHECK(m3World_StartJournal(world, tape, (int64_t)INT32_MAX + 1) == m3_success,
          "a journal capacity past INT32_MAX starts");
    m3BodyDef bd = m3DefaultBodyDef();
    NewBody(world, &bd);
    int64_t tapeBytes = StopJournal(world);
    CHECK(tapeBytes > 16 && tapeBytes < (int64_t)sizeof(tape), "and records into the real buffer");

    int64_t big = (int64_t)3 * 1024 * 1024 * 1024;
    CHECK(m3GetEncodedReplaySize(big, big) == 56 + 2 * big,
          "the container's size adds past INT32_MAX");
    CHECK(m3GetEncodedReplaySize(INT64_MAX - 40, 16) == 0, "and refuses a sum past INT64_MAX");

    int64_t bytes = 0;
    uint64_t recorded = 0;
    int64_t ops = 0;
    int64_t steps = 0;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);
    int64_t claimed = 0;
    memcpy(&claimed, blob + 24, sizeof(claimed));
    int64_t lying = claimed + ((int64_t)1 << 32); // the same low half
    memcpy(blob + 24, &lying, sizeof(lying));
    m3ReplayView view;
    CHECK(m3DecodeReplay(blob, bytes, &view) == m3_errorFormat,
          "a snapshot size off by 2^32 is read whole and refused");
    free(blob);

#if SIZE_MAX < INT64_MAX
    int64_t past = (int64_t)SIZE_MAX + 1;
    uint64_t misuse = m3World_GetCounters(world).misuse;
    CHECK(m3World_Restore(world, snap, past) == m3_errorInvalid &&
              m3World_StartJournal(world, tape, past) == m3_errorInvalid &&
              m3World_ReplayJournal(world, tape, past) == m3_errorInvalid,
          "a size past SIZE_MAX is refused as invalid");
    CHECK(m3World_GetCounters(world).misuse == misuse + 3u, "each counting one misuse");
    m3JournalInfo info;
    int64_t encoded = 77;
    CHECK(m3DescribeJournal(tape, past, &info) == m3_errorInvalid &&
              m3DecodeReplay(tape, past, &view) == m3_errorInvalid &&
              m3EncodeReplay(snap, past, tape, tapeBytes, 0u, tape, past, &encoded) ==
                  m3_errorInvalid &&
              m3EncodeReplay(snap, size, tape, past, 0u, tape, past, &encoded) == m3_errorInvalid &&
              encoded == 77,
          "and so is one handed to the container calls, which leave bytesOut as it was");
#endif
    free(snap);
    m3DestroyWorld(world);
}

int main(void)
{
    TestPreSolveReplay();
    TestRoundTrip();
    TestRefusals();
    TestSmallCapacity();
    TestSizeOutputsOnFailure();
    TestBusyStopKeepsSize();
    TestFuzz();
    TestSizesPastInt32();
    if (s_failures == 0)
    {
        printf("test_replayfile: all green\n");
        return 0;
    }
    printf("test_replayfile: %d failure(s)\n", s_failures);
    return 1;
}
