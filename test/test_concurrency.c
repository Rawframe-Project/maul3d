// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The multi-world concurrency contract,
// proven, not promised: two DISTINCT worlds stepped on two host
// threads land bit-identical to their serially-stepped twins, and
// read-only queries from a second thread between steps disturb
// nothing, and one world whose narrow phase runs on host threads lands
// on its serial twin's bits. One writer per world remains the law;
// create/destroy stay host-serialized. The TSAN CI cell runs this suite
// to turn any data race into a red build.
#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static m3World* BuildWorld(int32_t seed)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    sd.rollingResistance = 0.05f;
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &fl);
    for (int32_t i = 0; i < 150; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.9 * (double)(i % 10) - 4.5 + 0.01 * (double)seed,
                               0.6 + 1.0 * (double)(i / 50), 0.9 * (double)((i / 10) % 5) - 1.8};
        m3BodyId b = NewBody(world, &bd);
        if (i % 2 == 0)
        {
            NewBoxShape(world, b, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        }
        else
        {
            m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.42f};
            NewSphereShape(world, b, &sd, &s);
        }
    }
    return world;
}

typedef struct StepJob
{
    m3World* world;
    int32_t steps;
} StepJob;

static void* StepMain(void* arg)
{
    StepJob* job = (StepJob*)arg;
    for (int32_t i = 0; i < job->steps; ++i)
    {
        (void)m3World_Step(job->world, 1.0f / 60.0f, 4);
    }
    return NULL;
}

// Two worlds, two threads, versus two serial twins.
static void TestParallelWorldsMatchSerial(void)
{
    m3World* serialA = BuildWorld(1);
    m3World* serialB = BuildWorld(2);
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(serialA, 1.0f / 60.0f, 4);
        (void)m3World_Step(serialB, 1.0f / 60.0f, 4);
    }
    uint64_t wantA = m3World_Hash(serialA);
    uint64_t wantB = m3World_Hash(serialB);
    m3DestroyWorld(serialA);
    m3DestroyWorld(serialB);

    m3World* parA = BuildWorld(1);
    m3World* parB = BuildWorld(2);
    StepJob jobA = {parA, 300};
    StepJob jobB = {parB, 300};
    pthread_t tA;
    pthread_t tB;
    pthread_create(&tA, NULL, StepMain, &jobA);
    pthread_create(&tB, NULL, StepMain, &jobB);
    pthread_join(tA, NULL);
    pthread_join(tB, NULL);
    CHECK(m3World_Hash(parA) == wantA, "world A is thread-placement blind");
    CHECK(m3World_Hash(parB) == wantB, "world B is thread-placement blind");
    m3DestroyWorld(parA);
    m3DestroyWorld(parB);
}

typedef struct ReaderJob
{
    m3World* world;
    int32_t rounds;
    int32_t hits;
} ReaderJob;

static void* ReaderMain(void* arg)
{
    ReaderJob* job = (ReaderJob*)arg;
    for (int32_t i = 0; i < job->rounds; ++i)
    {
        m3CastHit hit = {0};
        bool found =
            m3World_CastRayClosest(job->world, (m3Pos3){0.0, 8.0, 0.0},
                                   (m3Vec3){0.0f, -16.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
        job->hits += found ? 1 : 0;
    }
    return NULL;
}

// Readers between steps: two query threads on ONE settled world,
// stepped by nobody. The contract says concurrent readers are
// free; the TSAN cell holds the proof to that promise.
static void TestConcurrentReadersBetweenSteps(void)
{
    m3World* world = BuildWorld(3);
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    ReaderJob r1 = {world, 4000, 0};
    ReaderJob r2 = {world, 4000, 0};
    pthread_t t1;
    pthread_t t2;
    pthread_create(&t1, NULL, ReaderMain, &r1);
    pthread_create(&t2, NULL, ReaderMain, &r2);
    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    CHECK(r1.hits == r1.rounds, "reader one hit the pile every time");
    CHECK(r2.hits == r2.rounds, "reader two hit the pile every time");
    uint64_t before = m3World_Hash(world);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    (void)before;
    m3DestroyWorld(world);
}

typedef struct StaleReader
{
    const m3World* world;
    m3BodyId stale;
    int32_t rounds;
} StaleReader;

static void* StaleReaderMain(void* arg)
{
    const StaleReader* reader = (const StaleReader*)arg;
    for (int32_t i = 0; i < reader->rounds; ++i)
    {
        (void)m3Body_GetPosition(reader->world, reader->stale);
    }
    return NULL;
}

static void TestMisuseFromConcurrentReaders(void)
{
    // Readers on several threads may refuse at once; each refusal counts.
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId gone = NewBody(world, &bd);
    DestroyBody(world, gone);
    uint64_t before = m3World_GetCounters(world).misuseCount;
    StaleReader reader = {world, gone, 2000};
    pthread_t one;
    pthread_t two;
    pthread_create(&one, NULL, StaleReaderMain, &reader);
    pthread_create(&two, NULL, StaleReaderMain, &reader);
    pthread_join(one, NULL);
    pthread_join(two, NULL);
    CHECK(m3World_GetCounters(world).misuseCount == before + 4000u,
          "every concurrent refusal counts");
    m3DestroyWorld(world);
}

// Task hooks that run the ranges of one task on threads of their own.
#define POOL_WORKERS 4

typedef struct PoolRange
{
    m3TaskFunction* task;
    void* taskContext;
    int32_t start;
    int32_t end;
} PoolRange;

typedef struct Pool
{
    PoolRange ranges[POOL_WORKERS];
    pthread_t threads[POOL_WORKERS];
    int32_t threadCount;
} Pool;

static void* PoolMain(void* arg)
{
    PoolRange* r = (PoolRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return NULL;
}

static void* PoolEnqueue(m3TaskFunction* task, int32_t itemCount, int32_t minRange,
                         void* taskContext, void* userContext)
{
    Pool* pool = (Pool*)userContext;
    int32_t grain = (itemCount + POOL_WORKERS - 1) / POOL_WORKERS;
    grain = grain < minRange ? minRange : grain;
    int32_t rangeCount = 0;
    for (int32_t start = 0; start < itemCount && rangeCount < POOL_WORKERS; start += grain)
    {
        int32_t end = start + grain < itemCount ? start + grain : itemCount;
        pool->ranges[rangeCount++] = (PoolRange){task, taskContext, start, end};
    }
    pool->ranges[rangeCount - 1].end = itemCount;
    pool->threadCount = 0;
    for (int32_t k = 1; k < rangeCount; ++k)
    {
        pthread_create(&pool->threads[pool->threadCount++], NULL, PoolMain, &pool->ranges[k]);
    }
    task(pool->ranges[0].start, pool->ranges[0].end, taskContext);
    return pool;
}

static void PoolFinish(void* userTask, void* userContext)
{
    (void)userContext;
    Pool* pool = (Pool*)userTask;
    for (int32_t k = 0; k < pool->threadCount; ++k)
    {
        pthread_join(pool->threads[k], NULL);
    }
}

// Boxes resting on a native height field grid, its narrow phase serial
// or on the pool's threads.
static uint64_t SettleOnHeightField(Pool* pool)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    if (pool != NULL)
    {
        def.enqueueTask = PoolEnqueue;
        def.finishTask = PoolFinish;
        def.userTaskContext = pool;
    }
    m3World* world = NewWorld(&def);
    static float heights[17 * 17];
    for (int32_t i = 0; i < 17 * 17; ++i)
    {
        heights[i] = 0.05f * (float)((i * 7) % 5);
    }
    m3BodyDef gd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    (void)NewHeightFieldShape(world, NewBody(world, &gd), &sd, heights, 17, 17, 1.0f);
    for (int32_t i = 0; i < 100; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){1.0 + 1.5 * (double)(i % 10), 0.6, 1.0 + 1.5 * (double)(i / 10)};
        NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    }
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

// Height field pairs build their cell windows in the narrow phase's task
// ranges. The windows once came from the shared step scratch, which the
// ranges' threads bumped at once.
static void TestHeightFieldPairsOnThreads(void)
{
    Pool pool;
    uint64_t serial = SettleOnHeightField(NULL);
    uint64_t threaded = SettleOnHeightField(&pool);
    CHECK(threaded == serial, "a height field world on threads lands on its serial twin");
}

// A heap whose pairs overflow the table, its narrow phase serial or on
// the pool's threads.
static uint64_t SettleOverflowingHeap(Pool* pool)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.pairCapacity = 200;
    if (pool != NULL)
    {
        def.enqueueTask = PoolEnqueue;
        def.finishTask = PoolFinish;
        def.userTaskContext = pool;
    }
    m3World* world = NewWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    for (int32_t i = 0; i < 40; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.05 * (double)(i % 8), 0.05 * (double)(i / 8), 0.0};
        NewSphereShape(world, NewBody(world, &bd), &sd, &ball);
    }
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_GetCounters(world).pairsDropped > 0, "the heap overflows its pair table");
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

// The pairs a full table keeps depend on the pair set alone, so a split
// across threads keeps the same ones.
static void TestOverflowingPairsOnThreads(void)
{
    Pool pool;
    uint64_t serial = SettleOverflowingHeap(NULL);
    uint64_t threaded = SettleOverflowingHeap(&pool);
    CHECK(threaded == serial, "an overflowing pair table on threads lands on its serial twin");
}

int main(void)
{
    TestMisuseFromConcurrentReaders();
    TestParallelWorldsMatchSerial();
    TestConcurrentReadersBetweenSteps();
    TestHeightFieldPairsOnThreads();
    TestOverflowingPairsOnThreads();
    if (s_failures == 0)
    {
        printf("test_concurrency: all green\n");
        return 0;
    }
    return 1;
}
