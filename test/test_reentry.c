// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The host's allocator cannot change the world under the call that runs
// it. Whatever call asks for or hands back a block, a writer the
// allocator calls is refused with m3_errorBusy, counts one misuse and
// changes nothing, and the call goes on to the world it would have made
// without it. A create once went on to attach its shape to a body the
// allocator had destroyed, reading before the start of the body arrays,
// and a writer called from the free of a world's own block worked on
// pools already freed.

#include "helpers.h"
#include "test_harness.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHUNK_DIM 16 // a voxel chunk is 16 voxels a side

// What the allocator calls back into: the world and the ids it aims at.
typedef struct Reentry
{
    m3World* world; // NULL while a scene is built: no probe runs then
    int32_t at;     // the alloc or free call that runs the probe, from 1
    int32_t calls;
    int32_t probes;
    int32_t refused;
    m3BodyId body;
    m3ShapeId shape;
    const uint8_t* snapshot;
    int64_t snapshotSize;
} Reentry;

static const m3Vec3 kQuadVerts[4] = {
    {-3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, 3.0f}, {-3.0f, 0.0f, 3.0f}};
static const uint16_t kQuadTris[6] = {0, 2, 1, 0, 3, 2};

// The seven writers a probe tries, six with a status and one without.
#define PROBE_WRITERS 7

static void Probe(Reentry* r)
{
    m3World* world = r->world;
    r->probes += 1;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId mesh = m3_nullShapeId;
    r->refused += m3DestroyBody(world, r->body) == m3_errorBusy ? 1 : 0;
    r->refused += m3DestroyShape(world, r->shape) == m3_errorBusy ? 1 : 0;
    r->refused += m3World_Restore(world, r->snapshot, r->snapshotSize) == m3_errorBusy ? 1 : 0;
    r->refused +=
        m3CreateMeshShape(world, r->body, &sd, kQuadVerts, 4, kQuadTris, 2, &mesh) == m3_errorBusy
            ? 1
            : 0;
    r->refused += m3World_RebuildBroadphase(world) == m3_errorBusy ? 1 : 0;
    r->refused += m3World_Step(world, 1.0f / 60.0f, 4) == m3_errorBusy ? 1 : 0;
    m3DestroyWorld(world);
}

static void* ReentryAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
    Reentry* r = (Reentry*)context;
    if (r->world != NULL && ++r->calls == r->at)
    {
        Probe(r);
    }
    return malloc(bytes);
}

static void ReentryFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    Reentry* r = (Reentry*)context;
    if (r->world != NULL && ++r->calls == r->at)
    {
        Probe(r);
    }
    free(memory);
}

// The calls that reach the allocator.
enum
{
    OpHull,
    OpMesh,
    OpHeightField,
    OpHeightFieldGrid,
    OpVoxel,
    OpRebuild,
    OpRestore,
    OpReplay,
    OpDestroyMesh,
    OpDestroyBody,
    OpDestroyWorld,
    OpCount
};

static const char* const kOpNames[OpCount] = {
    "a hull create",      "a mesh create",      "a height field create",
    "a grid create",      "a voxel create",     "a broadphase rebuild",
    "a restore",          "a replay",           "a mesh's destroy",
    "its body's destroy", "the world's destroy"};

// A scene of a static body with a mesh floor and a dynamic box, and a
// snapshot and a tape (a mesh create) of it.
typedef struct Scene
{
    m3World* world;
    m3BodyId ground;
    m3ShapeId floor;
    uint8_t* snapshot;
    int64_t snapshotSize;
    uint8_t tape[4096];
    int64_t tapeBytes;
} Scene;

static void BuildScene(Scene* scene, Reentry* r)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.allocator = (m3Allocator){ReentryAlloc, ReentryFree, r};
    scene->world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    scene->ground = NewBody(scene->world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    (void)m3CreateMeshShape(scene->world, scene->ground, &sd, kQuadVerts, 4, kQuadTris, 2,
                            &scene->floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    NewBoxShape(scene->world, NewBody(scene->world, &bd), &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    (void)m3World_Step(scene->world, 1.0f / 60.0f, 4);
    scene->snapshotSize = m3World_GetSnapshotSize(scene->world);
    scene->snapshot = (uint8_t*)malloc((size_t)scene->snapshotSize);
    (void)Snapshot(scene->world, scene->snapshot, scene->snapshotSize);
    m3ShapeId recorded = m3_nullShapeId;
    (void)StartJournal(scene->world, scene->tape, (int32_t)sizeof(scene->tape));
    (void)m3CreateMeshShape(scene->world, scene->ground, &sd, kQuadVerts, 4, kQuadTris, 2,
                            &recorded);
    scene->tapeBytes = StopJournal(scene->world);
    (void)Restore(scene->world, scene->snapshot, scene->snapshotSize);
}

static void FreeScene(Scene* scene)
{
    m3DestroyWorld(scene->world); // NULL once the op destroyed it
    free(scene->snapshot);
}

static m3Result RunOp(Scene* scene, int32_t op)
{
    static const m3Vec3 cloud[6] = {{0.3f, 0.0f, 0.0f},  {-0.3f, 0.0f, 0.0f}, {0.0f, 0.3f, 0.0f},
                                    {0.0f, -0.3f, 0.0f}, {0.0f, 0.0f, 0.3f},  {0.0f, 0.0f, -0.3f}};
    static float heights[9 * 9];
    static uint8_t voxels[CHUNK_DIM * CHUNK_DIM * CHUNK_DIM];
    for (int32_t i = 0; i < 9 * 9; ++i)
    {
        heights[i] = 0.1f * (float)(i % 5);
    }
    memset(voxels, 1, CHUNK_DIM * CHUNK_DIM);
    m3World* world = scene->world;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId shape = m3_nullShapeId;
    switch (op)
    {
    case OpHull:
        return m3CreateHullShape(world, scene->ground, &sd, cloud, 6, &shape);
    case OpMesh:
        return m3CreateMeshShape(world, scene->ground, &sd, kQuadVerts, 4, kQuadTris, 2, &shape);
    case OpHeightField:
        return m3CreateHeightFieldShape(world, scene->ground, &sd, heights, 9, 9, 1.0f, &shape);
    case OpHeightFieldGrid:
        return m3CreateHeightFieldGridShape(world, scene->ground, &sd, heights, 9, 9, 1.0f, &shape);
    case OpVoxel:
        return m3CreateVoxelChunkShape(world, scene->ground, &sd, voxels, NULL, 0.5f, &shape);
    case OpRebuild:
        return m3World_RebuildBroadphase(world);
    case OpRestore:
        return m3World_Restore(world, scene->snapshot, scene->snapshotSize);
    case OpReplay:
        return m3World_ReplayJournal(world, scene->tape, scene->tapeBytes);
    case OpDestroyMesh:
        return m3DestroyShape(world, scene->floor);
    case OpDestroyBody:
        return m3DestroyBody(world, scene->ground);
    default:
        m3DestroyWorld(world);
        scene->world = NULL;
        return m3_success;
    }
}

// Runs one call with the probe at each of its allocator calls in turn,
// against a twin that runs it with no probe.
static void SweepOp(int32_t op)
{
    bool whole = true;
    int32_t probed = 0;
    for (int32_t at = 1; at < 1000; ++at)
    {
        Reentry quiet = {0};
        Scene twin;
        BuildScene(&twin, &quiet);
        m3Result expected = RunOp(&twin, op);

        Reentry r = {0};
        Scene scene;
        BuildScene(&scene, &r);
        r.body = scene.ground;
        r.shape = scene.floor;
        r.snapshot = scene.snapshot;
        r.snapshotSize = scene.snapshotSize;
        uint64_t steps = m3World_GetStepCount(scene.world);
        uint64_t misuse = m3World_GetCounters(scene.world).misuse;
        r.world = scene.world;
        r.at = at;
        m3Result result = RunOp(&scene, op);
        r.world = NULL;
        if (r.probes == 0)
        {
            FreeScene(&scene);
            FreeScene(&twin);
            break; // the call made fewer allocator calls than at
        }
        probed += 1;
        if (op == OpDestroyWorld)
        {
            // Nothing is left to read: the probe's writers saw a world
            // whose pools were gone, and ASan holds them to not touching
            // them.
            whole = whole && r.refused == PROBE_WRITERS - 1;
            FreeScene(&scene);
            FreeScene(&twin);
            continue;
        }
        uint64_t counted = m3World_GetCounters(scene.world).misuse - misuse;
        bool ok = r.refused == PROBE_WRITERS - 1 && counted == PROBE_WRITERS &&
                  result == expected && m3World_GetStepCount(scene.world) == steps &&
                  m3World_Hash(scene.world) == m3World_Hash(twin.world);
        if (!ok)
        {
            printf("  %s, allocator call %d: %d of %d refused, %llu counted, result %d of %d\n",
                   kOpNames[op], at, r.refused, PROBE_WRITERS - 1, (unsigned long long)counted,
                   (int)result, (int)expected);
        }
        whole = whole && ok;
        FreeScene(&scene);
        FreeScene(&twin);
    }
    printf("M3_REENTRY %s: %s at %d allocator calls\n", kOpNames[op], whole ? "refused" : "BROKEN",
           probed);
    CHECK(probed > 0, "the call reaches the allocator");
    CHECK(whole, "a writer the allocator calls is refused busy, counted, and changes nothing");
}

int main(void)
{
    for (int32_t op = 0; op < OpCount; ++op)
    {
        SweepOp(op);
    }
    if (s_failures == 0)
    {
        printf("test_reentry: all passed\n");
        return 0;
    }
    printf("test_reentry: %d FAILURES\n", s_failures);
    return 1;
}
