// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The task hook docs ask for 64 KiB of stack on the threads that run
// ranges. Boxes resting on a mesh, a height field grid and a voxel chunk
// step with their narrow phase on threads given just that, and land on
// their serial twins' bits. A mesh pair once kept two 128 KiB arrays on
// the stack and needed about 288 KiB.

#include "helpers.h"
#include "test_harness.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>

#define POOL_WORKERS      4
#define RANGE_STACK_BYTES (64 * 1024)
#define CHUNK_DIM         16

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
    bool failed;
} Pool;

static void* PoolMain(void* arg)
{
    PoolRange* r = (PoolRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return NULL;
}

// Every range runs on a thread of its own with the small stack.
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
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, RANGE_STACK_BYTES);
    pool->threadCount = 0;
    for (int32_t k = 0; k < rangeCount; ++k)
    {
        if (pthread_create(&pool->threads[pool->threadCount], &attr, PoolMain, &pool->ranges[k]) !=
            0)
        {
            pool->failed = true;
            pool->ranges[k].task(pool->ranges[k].start, pool->ranges[k].end, taskContext);
            continue;
        }
        pool->threadCount += 1;
    }
    pthread_attr_destroy(&attr);
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

enum
{
    FloorMesh,
    FloorHeightField,
    FloorVoxels,
    FloorCount
};

static const char* const kFloorNames[FloorCount] = {"a mesh", "a height field grid",
                                                    "a voxel chunk"};

// Boxes resting on a floor of one kind, stepped serially or on the pool.
static uint64_t Settle(int32_t floor, Pool* pool)
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
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId shape = m3_nullShapeId;
    if (floor == FloorMesh)
    {
        static const m3Vec3 quad[4] = {
            {0.0f, 0.0f, 0.0f}, {16.0f, 0.0f, 0.0f}, {16.0f, 0.0f, 16.0f}, {0.0f, 0.0f, 16.0f}};
        static const uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
        (void)m3CreateMeshShape(world, ground, &sd, quad, 4, tris, 2, &shape);
    }
    else if (floor == FloorHeightField)
    {
        static float heights[17 * 17];
        for (int32_t i = 0; i < 17 * 17; ++i)
        {
            heights[i] = 0.05f * (float)((i * 7) % 5);
        }
        shape = NewHeightFieldShape(world, ground, &sd, heights, 17, 17, 1.0f);
    }
    else
    {
        static uint8_t voxels[CHUNK_DIM * CHUNK_DIM * CHUNK_DIM];
        memset(voxels, 0, sizeof(voxels));
        memset(voxels, 1, CHUNK_DIM * CHUNK_DIM); // the bottom layer, a meter thick
        (void)m3CreateVoxelChunkShape(world, ground, &sd, voxels, NULL, 1.0f, &shape);
    }
    CHECK(shape.index1 != 0, "the floor creates");
    for (int32_t i = 0; i < 100; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){1.0 + 1.5 * (double)(i % 10), floor == FloorVoxels ? 1.3 : 0.5,
                               1.0 + 1.5 * (double)(i / 10)};
        NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    }
    for (int32_t i = 0; i < 20; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_GetCounters(world).contactCount >= 100, "every box rests on the floor");
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

int main(void)
{
    for (int32_t floor = 0; floor < FloorCount; ++floor)
    {
        Pool pool;
        memset(&pool, 0, sizeof(pool));
        uint64_t serial = Settle(floor, NULL);
        uint64_t threaded = Settle(floor, &pool);
        printf("M3_SMALLSTACK %s: %s\n", kFloorNames[floor],
               threaded == serial ? "same bits" : "DIFFERENT BITS");
        CHECK(!pool.failed, "every range got a thread of 64 KiB");
        CHECK(threaded == serial, "a narrow phase on 64 KiB threads lands on its serial twin");
    }
    if (s_failures == 0)
    {
        printf("test_smallstack: all passed\n");
        return 0;
    }
    printf("test_smallstack: %d FAILURES\n", s_failures);
    return 1;
}
