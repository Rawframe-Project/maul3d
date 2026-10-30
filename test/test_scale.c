// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Worker twins: three hundred mixed bodies and a jelly rain into a voxel
// block, carved mid-run, and a dense pile that forces the solver's
// overflow colors. The serial run and a real four-thread host pool land
// on identical bits. The full-size scenes (five and ten thousand bodies)
// are pinned by test_bench in the nightly check.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"
#include "maul3d/softbody.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif

// The test-grade host pool, the determinism suite's pattern: real
// threads, spawn per enqueue, any split must produce identical bits.
#define POOL_WORKERS 4

typedef struct PoolRange
{
    m3TaskFunction* task;
    void* taskContext;
    int32_t start;
    int32_t end;
} PoolRange;

typedef struct PoolTask
{
    PoolRange ranges[POOL_WORKERS];
    int32_t rangeCount;
#ifdef _WIN32
    HANDLE threads[POOL_WORKERS];
#else
    pthread_t threads[POOL_WORKERS];
#endif
    int32_t threadCount;
} PoolTask;

#ifdef _WIN32
static DWORD WINAPI PoolMain(LPVOID arg)
{
    PoolRange* r = (PoolRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return 0;
}
#else
static void* PoolMain(void* arg)
{
    PoolRange* r = (PoolRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return NULL;
}
#endif

static void* PoolEnqueue(m3TaskFunction* task, int32_t itemCount, int32_t minRange,
                         void* taskContext, void* userContext)
{
    (void)userContext;
    static PoolTask taskSlot;
    PoolTask* t = &taskSlot;
    int32_t workers = POOL_WORKERS;
    int32_t grain = (itemCount + workers - 1) / workers;
    grain = grain < minRange ? minRange : grain;
    t->rangeCount = 0;
    t->threadCount = 0;
    for (int32_t start = 0; start < itemCount; start += grain)
    {
        int32_t end = start + grain < itemCount ? start + grain : itemCount;
        PoolRange* r = &t->ranges[t->rangeCount++];
        r->task = task;
        r->taskContext = taskContext;
        r->start = start;
        r->end = end;
        if (t->rangeCount == POOL_WORKERS)
        {
            r->end = itemCount;
            break;
        }
    }
    for (int32_t k = 1; k < t->rangeCount; ++k)
    {
#ifdef _WIN32
        t->threads[t->threadCount] = CreateThread(NULL, 0, PoolMain, &t->ranges[k], 0, NULL);
#else
        pthread_create(&t->threads[t->threadCount], NULL, PoolMain, &t->ranges[k]);
#endif
        t->threadCount += 1;
    }
    t->ranges[0].task(t->ranges[0].start, t->ranges[0].end, taskContext);
    return t;
}

static void PoolFinish(void* userTask, void* userContext)
{
    (void)userContext;
    PoolTask* t = (PoolTask*)userTask;
    for (int32_t k = 0; k < t->threadCount; ++k)
    {
#ifdef _WIN32
        WaitForSingleObject(t->threads[k], INFINITE);
        CloseHandle(t->threads[k]);
#else
        pthread_join(t->threads[k], NULL);
#endif
    }
}

static uint64_t SplitMix(uint64_t* s)
{
    *s += 0x9E3779B97F4A7C15ull;
    uint64_t z = *s;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static double RandRange(uint64_t* s, double lo, double hi)
{
    return lo + (hi - lo) * ((double)(SplitMix(s) >> 11) / 9007199254740992.0);
}

// The mini block: two welded tower chunks and a thousand mixed
// bodies over a 20 x 20 yard. The city block's little sibling,
// small enough for every CI cell, big enough to mean it.
static m3ShapeId BuildMiniBlock(m3World* world)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef pd = m3DefaultShapeDef();
    pd.friction = 0.6f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &pd, &floor);

    static uint8_t tower[16 * 16 * 16];
    memset(tower, 0, sizeof(tower));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            tower[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y < 16; ++y)
    {
        for (int32_t z = 5; z <= 10; ++z)
        {
            for (int32_t x = 5; x <= 10; ++x)
            {
                tower[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    m3BodyDef td = m3DefaultBodyDef();
    td.position = (m3Pos3){-4.0, 0.0, -4.0};
    m3ShapeId lower = NewVoxelChunkShape(world, NewBody(world, &td), &sd, tower, NULL, 0.5f);
    td.position = (m3Pos3){-4.0, 8.0, -4.0}; // welded stack
    NewVoxelChunkShape(world, NewBody(world, &td), &sd, tower, NULL, 0.5f);

    // A jelly rides the block: soft bodies are in the scale gate
    // and the worker-twin proof.
    m3SoftBodyDef jd = m3DefaultSoftBodyDef();
    jd.position = (m3Pos3){6.0, 10.0, 6.0};
    jd.countX = 6;
    jd.countY = 6;
    jd.countZ = 6;
    jd.spacing = 0.2f;
    jd.compliance = 1.0e-4f;
    jd.radius = 0.08f;
    NewSoftBody(world, &jd);

    m3ShapeDef bs = m3DefaultShapeDef();
    bs.friction = 0.6f;
    bs.rollingResistance = 0.05f;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    uint64_t rng = 0x5CA1Eull;
    for (int32_t k = 0; k < 300; ++k)
    {
        bd.position = (m3Pos3){RandRange(&rng, -10.0, 10.0), RandRange(&rng, 6.0, 26.0),
                               RandRange(&rng, -10.0, 10.0)};
        m3BodyId body = NewBody(world, &bd);
        int32_t f = k % 4;
        if (f == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, (float)RandRange(&rng, 0.2, 0.4)};
            NewSphereShape(world, body, &bs, &ball);
        }
        else if (f == 1)
        {
            NewBoxShape(world, body, &bs,
                        (m3Vec3){(float)RandRange(&rng, 0.15, 0.35),
                                 (float)RandRange(&rng, 0.15, 0.35),
                                 (float)RandRange(&rng, 0.15, 0.35)});
        }
        else if (f == 2)
        {
            m3Capsule c = {
                {-0.2f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f}, (float)RandRange(&rng, 0.12, 0.2)};
            NewCapsuleShape(world, body, &bs, &c);
        }
        else
        {
            m3Vec3 cl[8];
            for (int32_t c = 0; c < 8; ++c)
            {
                cl[c] = (m3Vec3){(float)RandRange(&rng, -0.25, 0.25),
                                 (float)RandRange(&rng, -0.25, 0.25),
                                 (float)RandRange(&rng, -0.25, 0.25)};
            }
            if (!m3Shape_IsValid(world, NewHullShape(world, body, &bs, cl, 8)))
            {
                NewBoxShape(world, body, &bs, (m3Vec3){0.2f, 0.2f, 0.2f});
            }
        }
    }
    return lower;
}

static m3WorldDef ScaleDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2048;
    def.shapeCapacity = 2048;
    def.voxelCapacity = 4;
    return def;
}

static void TestWorkerTwinsAtScale(void)
{
    // Serial versus a real four-thread pool: identical bits after
    // 120 storm steps with mid-run carves. The worker count is a
    // performance knob, never a state input: the scale gate's core
    // promise.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = ScaleDef();
        if (run == 1)
        {
            def.enqueueTask = PoolEnqueue;
            def.finishTask = PoolFinish;
        }
        m3World* world = NewWorld(&def);
        m3ShapeId chunk = BuildMiniBlock(world);
        for (int32_t i = 0; i < 120; ++i)
        {
            if (i >= 40 && i % 10 == 0)
            {
                int32_t cx = 5 + (i / 10) % 6;
                int32_t lo[3] = {cx, 2, 5};
                int32_t hi[3] = {cx, 2, 10};
                int32_t cleared = 0;
                (void)m3VoxelChunkShape_ClearBox(world, chunk, lo, hi, &cleared);
            }
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "serial and four-thread scale twins are bit-identical");
}

static void TestColoredPileTwins(void)
{
    // The colored solver under real threads: a dense pile (boxes and
    // spheres jammed together, many shared bodies, overflow colors
    // guaranteed) must hash identically serial versus pooled. This is
    // the worker-twin gate aimed straight at the solver.
    uint64_t hashes[2];
    for (int32_t variant = 0; variant < 2; ++variant)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 64;
        def.shapeCapacity = 64;
        if (variant == 1)
        {
            def.enqueueTask = PoolEnqueue;
            def.finishTask = PoolFinish;
        }
        m3World* world = NewWorld(&def);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        sd.friction = 0.6f;
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &sd, &floor);

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        for (int32_t k = 0; k < 24; ++k)
        {
            double x = 0.55 * (double)(k % 4) - 0.8;
            double z = 0.55 * (double)((k / 4) % 3) - 0.5;
            double y = 0.5 + 0.65 * (double)(k / 12);
            bd.position = (m3Pos3){x, y, z};
            m3BodyId body = NewBody(world, &bd);
            if (k % 2 == 0)
            {
                NewBoxShape(world, body, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
            }
            else
            {
                m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.27f};
                NewSphereShape(world, body, &sd, &ball);
            }
        }
        for (int32_t i = 0; i < 150; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[variant] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the colored pile is bit-identical serial versus pooled");
}

int main(void)
{
    TestWorkerTwinsAtScale();
    TestColoredPileTwins();
    if (s_failures == 0)
    {
        printf("test_scale: all green\n");
        return 0;
    }
    printf("test_scale: %d failure(s)\n", s_failures);
    return 1;
}
