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

#include "maul3d/character.h"
#include "maul3d/draw.h"
#include "maul3d/softbody.h"

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
        return m3CreateHeightFieldMeshShape(world, scene->ground, &sd, heights, 9, 9, 1.0f, &shape);
    case OpHeightFieldGrid:
        return m3CreateHeightFieldShape(world, scene->ground, &sd, heights, 9, 9, 1.0f, &shape);
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
        uint64_t misuse = m3World_GetCounters(scene.world).misuseCount;
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
        uint64_t counted = m3World_GetCounters(scene.world).misuseCount - misuse;
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

// A destroy makes its id stale before the host's free hook runs on the
// object's memory: host code that reads the shape or its body back from
// the hook finds them gone, and a getter returns zeros without reaching
// freed content. A shape's id once stayed live until its mesh, hull and
// height field were freed, and a body's until its whole cascade had run.
typedef struct Watch
{
    m3World* world; // NULL outside the destroy under watch
    m3ShapeId shape;
    m3BodyId body;
    int32_t frees;
    int32_t liveShape;
    int32_t liveBody;
    int32_t typeRead;
} Watch;

static void* WatchAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    return malloc(bytes);
}

static void WatchFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    Watch* w = (Watch*)context;
    if (w->world != NULL)
    {
        w->frees += 1;
        w->liveShape += m3Shape_IsValid(w->world, w->shape) ? 1 : 0;
        w->liveBody += m3Body_IsValid(w->world, w->body) ? 1 : 0;
        // The box getter reads a mesh's or height field's content record,
        // so it must return its zero before it reaches freed memory.
        m3WorldAabb box = m3Shape_GetAabb(w->world, w->shape);
        w->typeRead += m3Shape_GetType(w->world, w->shape) != 0 || box.upperBound.x != 0.0 ? 1 : 0;
    }
    free(memory);
}

enum
{
    KindMesh,
    KindHull,
    KindHeightField,
    KindVoxel,
    KindCount
};

static m3ShapeId WatchedShape(m3World* world, m3BodyId body, int32_t kind)
{
    static const m3Vec3 cloud[5] = {{-0.4f, -0.3f, -0.2f},
                                    {0.5f, -0.2f, -0.3f},
                                    {0.1f, 0.6f, -0.1f},
                                    {-0.2f, 0.1f, 0.5f},
                                    {0.3f, 0.2f, 0.4f}};
    static const float heights[9] = {0.0f, 0.1f, 0.0f, 0.2f, 0.3f, 0.1f, 0.0f, 0.1f, 0.0f};
    static uint8_t voxels[CHUNK_DIM * CHUNK_DIM * CHUNK_DIM];
    m3ShapeDef sd = m3DefaultShapeDef();
    switch (kind)
    {
    case KindMesh:
        return NewMeshShape(world, body, &sd, kQuadVerts, 4, kQuadTris, 2);
    case KindHull:
        return NewHullShape(world, body, &sd, cloud, 5);
    case KindHeightField:
        return NewHeightFieldShape(world, body, &sd, heights, 3, 3, 1.0f);
    default:
        memset(voxels, 1, CHUNK_DIM * CHUNK_DIM);
        return NewVoxelChunkShape(world, body, &sd, voxels, NULL, 1.0f);
    }
}

static void TestDestroyStalesIdsBeforeFree(void)
{
    static const char* const names[KindCount] = {"mesh", "hull", "height field", "voxel chunk"};
    for (int32_t kind = 0; kind < KindCount; ++kind)
    {
        for (int32_t viaBody = 0; viaBody < 2; ++viaBody)
        {
            Watch w;
            memset(&w, 0, sizeof(w));
            m3WorldDef wd = m3DefaultWorldDef();
            wd.bodyCapacity = 4;
            wd.shapeCapacity = 4;
            wd.voxelCapacity = 1;
            wd.allocator = (m3Allocator){WatchAlloc, WatchFree, &w};
            m3World* world = NewWorld(&wd);
            m3BodyDef bd = m3DefaultBodyDef();
            m3BodyId body = NewBody(world, &bd);
            m3ShapeId shape = WatchedShape(world, body, kind);
            char what[96];
            snprintf(what, sizeof(what), "the %s creates", names[kind]);
            CHECK(m3Shape_IsValid(world, shape), what);
            w.shape = shape;
            w.body = body;
            w.world = world;
            if (viaBody != 0)
            {
                CHECK(m3DestroyBody(world, body) == m3_success, "the body destroys");
            }
            else
            {
                CHECK(m3DestroyShape(world, shape) == m3_success, "the shape destroys");
            }
            w.world = NULL;
            snprintf(what, sizeof(what), "a free hook in the %s's %s destroy finds its id stale",
                     names[kind], viaBody != 0 ? "body" : "shape");
            CHECK(w.liveShape == 0 && w.typeRead == 0, what);
            if (viaBody != 0)
            {
                snprintf(what, sizeof(what), "and the %s's body stale", names[kind]);
                CHECK(w.liveBody == 0, what);
            }
            // A hull and a voxel chunk keep their records in the world's own
            // slots; a mesh and a height field free theirs through the hook.
            if (kind == KindMesh || kind == KindHeightField)
            {
                snprintf(what, sizeof(what), "the %s's destroy frees its content", names[kind]);
                CHECK(w.frees > 0, what);
            }
            m3DestroyWorld(world);
        }
    }
}

// A world's destroy hands its blocks back one by one. Host code in the
// free hook may still read the world: every reader must find an empty
// world, never a pool already freed. Each run destroys the same world and,
// at one free in turn, runs every kind of reader from the hook.
typedef struct DyingWorld
{
    const m3World* world; // set while the destroy under watch runs
    int32_t at;           // the free that reads, from 1; 0 counts only
    int32_t frees;
    m3BodyId body;
    m3ShapeId mesh;
    m3JointId joint;
    m3SoftBodyId soft;
    m3CharacterId hero;
} DyingWorld;

static void IgnoreSegment(m3Pos3 p1, m3Pos3 p2, uint32_t color, void* context)
{
    (void)p1;
    (void)p2;
    (void)color;
    (void)context;
}

static void IgnorePoint(m3Pos3 p, float size, uint32_t color, void* context)
{
    (void)p;
    (void)size;
    (void)color;
    (void)context;
}

static void IgnoreTriangle(m3Pos3 a, m3Pos3 b, m3Pos3 c, uint32_t color, void* context)
{
    (void)a;
    (void)b;
    (void)c;
    (void)color;
    (void)context;
}

static void ReadDyingWorld(const DyingWorld* d)
{
    const m3World* world = d->world;
    (void)m3World_GetCounters(world);
    (void)m3World_GetMemoryUsage(world);
    (void)m3World_GetProfile(world);
    (void)m3World_GetSnapshotSize(world);
    (void)m3World_Hash(world);
    (void)m3Body_GetPosition(world, d->body);
    (void)m3Body_IsValid(world, d->body);
    (void)m3Shape_GetAabb(world, d->mesh);
    (void)m3Joint_IsValid(world, d->joint);
    (void)m3SoftBody_GetParticleCount(world, d->soft);
    (void)m3Character_IsGrounded(world, d->hero);
    m3CastHit hit;
    (void)m3World_CastRayClosest(world, (m3Pos3){0.0, 5.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f},
                                 m3DefaultQueryFilter(), &hit);
    m3ShapeId ids[8];
    (void)m3World_OverlapAabb(world, (m3WorldAabb){{-5.0, -5.0, -5.0}, {5.0, 5.0, 5.0}},
                              m3DefaultQueryFilter(), ids, 8);
    m3DebugDraw draw;
    memset(&draw, 0, sizeof(draw));
    draw.drawSegment = IgnoreSegment;
    draw.drawPoint = IgnorePoint;
    draw.drawTriangle = IgnoreTriangle;
    draw.drawShapes = true;
    draw.drawSolidShapes = true;
    draw.drawContacts = true;
    draw.drawJoints = true;
    draw.drawAabbs = true;
    draw.drawSleepTint = true;
    draw.drawIslands = true;
    draw.drawMassAxes = true;
    draw.drawTreeBoxes = true;
    m3World_Draw(world, &draw);
}

static void* DyingAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    return malloc(bytes);
}

static void DyingFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    DyingWorld* d = (DyingWorld*)context;
    if (d->world != NULL)
    {
        d->frees += 1;
        if (d->frees == d->at)
        {
            ReadDyingWorld(d);
        }
    }
    free(memory);
}

static m3World* BuildDyingWorld(DyingWorld* d)
{
    static uint8_t voxels[CHUNK_DIM * CHUNK_DIM * CHUNK_DIM];
    static const float heights[9] = {0.0f, 0.1f, 0.0f, 0.2f, 0.3f, 0.1f, 0.0f, 0.1f, 0.0f};
    static const m3Vec3 cloud[5] = {{-0.4f, -0.3f, -0.2f},
                                    {0.5f, -0.2f, -0.3f},
                                    {0.1f, 0.6f, -0.1f},
                                    {-0.2f, 0.1f, 0.5f},
                                    {0.3f, 0.2f, 0.4f}};
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.voxelCapacity = 1;
    wd.softBodyCapacity = 1;
    wd.characterCapacity = 1;
    wd.allocator = (m3Allocator){DyingAlloc, DyingFree, d};
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    d->mesh = NewMeshShape(world, ground, &sd, kQuadVerts, 4, kQuadTris, 2);
    (void)NewHeightFieldShape(world, ground, &sd, heights, 3, 3, 1.0f);
    memset(voxels, 1, CHUNK_DIM * CHUNK_DIM);
    gd.position = (m3Pos3){20.0, 0.0, 0.0};
    (void)NewVoxelChunkShape(world, NewBody(world, &gd), &sd, voxels, NULL, 1.0f);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    d->body = NewBody(world, &bd);
    (void)NewHullShape(world, d->body, &sd, cloud, 5);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = ground;
    jd.bodyIdB = d->body;
    d->joint = NewJoint(world, &jd);
    m3SoftBodyDef sbd = m3DefaultSoftBodyDef();
    sbd.position = (m3Pos3){-4.0, 2.0, 0.0};
    d->soft = NewSoftBody(world, &sbd);
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){4.0, 2.0, 0.0};
    d->hero = NewCharacter(world, &cd);
    m3WaterVolumeDef water = m3DefaultWaterVolumeDef();
    water.bounds.lowerBound = (m3Pos3){-10.0, -2.0, -10.0};
    water.bounds.upperBound = (m3Pos3){10.0, 0.5, 10.0};
    (void)NewWaterVolume(world, &water);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    return world;
}

static void TestReadersDuringWorldDestroy(void)
{
    DyingWorld d;
    memset(&d, 0, sizeof(d));
    m3World* world = BuildDyingWorld(&d);
    d.world = world;
    m3DestroyWorld(world);
    int32_t total = d.frees;
    CHECK(total > 10, "the world's destroy hands back its blocks one by one");
    for (int32_t at = 1; at <= total; ++at)
    {
        memset(&d, 0, sizeof(d));
        world = BuildDyingWorld(&d);
        d.at = at;
        d.world = world;
        m3DestroyWorld(world);
        CHECK(d.frees == total, "every destroy hands back the same blocks");
    }
    printf("M3_REENTRY the world's destroy: every reader ran at each of %d frees\n", total);
}

int main(void)
{
    for (int32_t op = 0; op < OpCount; ++op)
    {
        SweepOp(op);
    }
    TestDestroyStalesIdsBeforeFree();
    TestReadersDuringWorldDestroy();
    if (s_failures == 0)
    {
        printf("test_reentry: all passed\n");
        return 0;
    }
    printf("test_reentry: %d FAILURES\n", s_failures);
    return 1;
}
