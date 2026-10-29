// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The rebuilt tree gate: m3World_RebuildBroadphase replaces
// a lopsided insertion-order tree with a balanced one WITHOUT
// touching a single answer: queries return the same shapes.
// A rebuild the allocator refuses changes nothing and records
// nothing. Plus the carry check: a crate on a gliding kinematic
// platform rides along by friction.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A host allocator the test can turn off.
static void* ToggleAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
    return *(const bool*)context ? NULL : malloc(bytes);
}

static void ToggleFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    (void)context;
    free(memory);
}

// A city on the default allocator, or, given a switch, on one that
// refuses while the switch is on.
static m3World* City(bool* refuse)
{
    // A worst-case insertion order: a long strip of statics created
    // left to right (the classic lopsided tree), plus a rain of
    // dynamic balls.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 64;
    wd.shapeCapacity = 64;
    if (refuse != NULL)
    {
        wd.allocator = (m3Allocator){ToggleAlloc, ToggleFree, refuse};
    }
    m3World* world = NewWorld(&wd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    for (int32_t i = 0; i < 24; ++i)
    {
        bd.position = (m3Pos3){-23.0 + 2.0 * (double)i, 0.0, 0.0};
        m3BodyId post = NewBody(world, &bd);
        NewBoxShape(world, post, &sd, (m3Vec3){1.0f, 0.5f, 1.0f});
    }
    m3BodyDef db = m3DefaultBodyDef();
    db.type = m3_dynamicBody;
    for (int32_t i = 0; i < 8; ++i)
    {
        db.position = (m3Pos3){-14.0 + 4.0 * (double)i, 2.0 + 0.3 * (double)(i % 3), 0.0};
        m3BodyId ball = NewBody(world, &db);
        m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
        NewSphereShape(world, ball, &sd, &s);
    }
    return world;
}

static void TestRebuildChangesNoAnswer(void)
{
    // Twin cities: one rebuilds, one keeps its lopsided tree. The
    // tree is never hashed and pairs are canonical, so 180 stepped
    // frames must land on the SAME hash, and a query must return
    // the same shapes before and after the rebuild.
    m3World* plain = City(NULL);
    m3World* rebuilt = City(NULL);

    m3ShapeId before[64];
    int32_t beforeCount = m3World_OverlapSphere(rebuilt, (m3Pos3){0.0, 0.5, 0.0}, 3.0f, before, 64,
                                                m3DefaultQueryFilter());
    CHECK(m3World_RebuildBroadphase(rebuilt) == m3_success, "the tree rebuilds");
    m3ShapeId after[64];
    int32_t afterCount = m3World_OverlapSphere(rebuilt, (m3Pos3){0.0, 0.5, 0.0}, 3.0f, after, 64,
                                               m3DefaultQueryFilter());
    CHECK(beforeCount == afterCount, "the rebuild keeps the overlap count");
    bool same = beforeCount == afterCount;
    for (int32_t i = 0; same && i < beforeCount; ++i)
    {
        same = before[i].index1 == after[i].index1;
    }
    CHECK(same, "the rebuild keeps the overlap list, in order");

    m3RayCastResult h1 = m3World_CastRayClosest(
        plain, (m3Pos3){0.0, 5.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f}, m3DefaultQueryFilter());
    m3RayCastResult h2 = m3World_CastRayClosest(
        rebuilt, (m3Pos3){0.0, 5.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f}, m3DefaultQueryFilter());
    CHECK(h1.hit && h2.hit && h1.shapeId.index1 == h2.shapeId.index1 && h1.fraction == h2.fraction,
          "the rebuilt tree casts the same ray");

    for (int32_t i = 0; i < 180; ++i)
    {
        (void)m3World_Step(plain, 1.0f / 60.0f, 4);
        (void)m3World_Step(rebuilt, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(plain) == m3World_Hash(rebuilt),
          "physics is bit-identical across the rebuild");
    m3DestroyWorld(plain);
    m3DestroyWorld(rebuilt);
}

// Whether a world's snapshot, which carries the tree, equals the given
// bytes.
static bool SnapshotIs(const m3World* world, const uint8_t* bytes, int32_t size)
{
    uint8_t* now = (uint8_t*)malloc((size_t)size);
    bool same =
        now != NULL && Snapshot(world, now, size) == size && memcmp(now, bytes, (size_t)size) == 0;
    free(now);
    return same;
}

static void TestRebuildWithoutMemory(void)
{
    // The rebuild takes its scratch before it records the op, so a host
    // allocator that refuses leaves the tree and the tape alike: the
    // tape replays over the starting world to the world the recorder
    // holds. With memory the op is recorded and replays to the same tree.
    bool refuse = false;
    m3World* world = City(&refuse);
    int32_t size = m3World_GetSnapshotSize(world);
    uint8_t* start = (uint8_t*)malloc((size_t)size);
    uint8_t* rebuilt = (uint8_t*)malloc((size_t)size);
    CHECK(start != NULL && rebuilt != NULL && Snapshot(world, start, size) == size,
          "the city is snapshotted");
    static uint8_t tape[4096];
    m3JournalInfo info;

    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts");
    m3Result result = m3_success;
    refuse = true;
    CHECK_ACCEPTS(world, result = m3World_RebuildBroadphase(world),
                  "a refusing allocator counts no misuse");
    refuse = false;
    CHECK(result == m3_errorCapacity, "the rebuild refuses with capacity");
    CHECK(SnapshotIs(world, start, size), "the refused rebuild keeps the old tree");
    int32_t tapeBytes = StopJournal(world);
    CHECK(tapeBytes >= 0 && DescribeJournal(tape, tapeBytes, &info) && info.opCount == 0,
          "the refused rebuild is not on the tape");
    CHECK(ReplayJournal(world, tape, tapeBytes) && SnapshotIs(world, start, size),
          "the tape replays to the world the recorder holds");

    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts again");
    CHECK(m3World_RebuildBroadphase(world) == m3_success, "with memory the tree rebuilds");
    CHECK(Snapshot(world, rebuilt, size) == size && !SnapshotIs(world, start, size),
          "the rebuilt tree is state");
    tapeBytes = StopJournal(world);
    CHECK(tapeBytes > 0 && DescribeJournal(tape, tapeBytes, &info) && info.opCount == 1,
          "the rebuild is on the tape");
    CHECK(Restore(world, start, size) && ReplayJournal(world, tape, tapeBytes) &&
              SnapshotIs(world, rebuilt, size),
          "the tape replays to the rebuilt tree");
    free(rebuilt);
    free(start);
    m3DestroyWorld(world);
}

static void TestPlatformCarry(void)
{
    // The carry check, rigid-body edition: a crate resting on a
    // kinematic platform gliding sideways rides along by friction.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3BodyDef pd = m3DefaultBodyDef();
    pd.type = m3_kinematicBody;
    pd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId platform = NewBody(world, &pd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.8f;
    NewBoxShape(world, platform, &sd, (m3Vec3){1.5f, 0.2f, 1.5f});
    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){0.0, 1.5, 0.0};
    m3BodyId crate = NewBody(world, &cd);
    NewBoxShape(world, crate, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    for (int32_t i = 0; i < 240; ++i)
    {
        double t = (double)(i + 1) / 60.0;
        m3Body_SetTargetTransform(world, platform, (m3Pos3){0.8 * t, 1.0, 0.0},
                                  (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double px = m3Body_GetPosition(world, platform).x;
    double cx = m3Body_GetPosition(world, crate).x;
    CHECK(px > 3.0, "the platform genuinely glides");
    CHECK(fabs(cx - px) < 0.4, "the crate rides the platform");
    CHECK(m3Body_GetPosition(world, crate).y > 1.2, "the crate stays aboard");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRebuildChangesNoAnswer();
    TestRebuildWithoutMemory();
    TestPlatformCarry();
    if (s_failures == 0)
    {
        printf("test_rebuild: all passed\n");
        return 0;
    }
    printf("test_rebuild: %d FAILURES\n", s_failures);
    return 1;
}
