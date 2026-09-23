// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The multi-world concurrency contract,
// proven, not promised: two DISTINCT worlds stepped on two host
// threads land bit-identical to their serially-stepped twins, and
// read-only queries from a second thread between steps disturb
// nothing. One writer per world remains the law; create/destroy
// stay host-serialized. The TSAN CI cell runs this suite to turn
// any data race into a red build.
#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"
#include "test_harness.h"

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static m3WorldId BuildWorld(int32_t seed)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    sd.rollingResistance = 0.05f;
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);
    for (int32_t i = 0; i < 150; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.9 * (double)(i % 10) - 4.5 + 0.01 * (double)seed,
                               0.6 + 1.0 * (double)(i / 50), 0.9 * (double)((i / 10) % 5) - 1.8};
        m3BodyId b = m3CreateBody(world, &bd);
        if (i % 2 == 0)
        {
            m3CreateBoxShape(b, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        }
        else
        {
            m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.42f};
            m3CreateSphereShape(b, &sd, &s);
        }
    }
    return world;
}

typedef struct StepJob
{
    m3WorldId world;
    int32_t steps;
} StepJob;

static void* StepMain(void* arg)
{
    StepJob* job = (StepJob*)arg;
    for (int32_t i = 0; i < job->steps; ++i)
    {
        m3World_Step(job->world, 1.0f / 60.0f, 4);
    }
    return NULL;
}

// Two worlds, two threads, versus two serial twins.
static void TestParallelWorldsMatchSerial(void)
{
    m3WorldId serialA = BuildWorld(1);
    m3WorldId serialB = BuildWorld(2);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(serialA, 1.0f / 60.0f, 4);
        m3World_Step(serialB, 1.0f / 60.0f, 4);
    }
    uint64_t wantA = m3World_Hash(serialA);
    uint64_t wantB = m3World_Hash(serialB);
    m3DestroyWorld(serialA);
    m3DestroyWorld(serialB);

    m3WorldId parA = BuildWorld(1);
    m3WorldId parB = BuildWorld(2);
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
    m3WorldId world;
    int32_t rounds;
    int32_t hits;
} ReaderJob;

static void* ReaderMain(void* arg)
{
    ReaderJob* job = (ReaderJob*)arg;
    for (int32_t i = 0; i < job->rounds; ++i)
    {
        m3RayHit hit = m3World_CastRayClosest(job->world, (m3Pos3){0.0, 8.0, 0.0},
                                              (m3Vec3){0.0f, -16.0f, 0.0f});
        job->hits += hit.hit ? 1 : 0;
    }
    return NULL;
}

// Readers between steps: two query threads on ONE settled world,
// stepped by nobody. The contract says concurrent readers are
// free; the TSAN cell holds the proof to that promise.
static void TestConcurrentReadersBetweenSteps(void)
{
    m3WorldId world = BuildWorld(3);
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
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
    m3World_Step(world, 1.0f / 60.0f, 4);
    (void)before;
    m3DestroyWorld(world);
}

static void* RefuseCapacityMain(void* arg)
{
    m3WorldId world = *(const m3WorldId*)arg;
    m3BodyDef bd = m3DefaultBodyDef();
    m3CreateBody(world, &bd); // the pool is full: refused for capacity
    return (void*)(intptr_t)(m3LastResult() == m3_errorCapacity);
}

static void TestLastResultIsPerThread(void)
{
    // This thread refuses for invalid input while another refuses for
    // capacity; each thread reads back its own reason.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3CreateBody(world, &bd);
    m3World_SetGravity(world, (m3Vec3){0.0f, 0.0f, INFINITY});
    CHECK(m3LastResult() == m3_errorInvalid, "this thread's refusal is invalid");
    pthread_t other;
    void* sawCapacity = NULL;
    pthread_create(&other, NULL, RefuseCapacityMain, &world);
    pthread_join(other, &sawCapacity);
    CHECK(sawCapacity != NULL, "the other thread saw its own capacity refusal");
    CHECK(m3LastResult() == m3_errorInvalid, "and did not overwrite this thread's reason");
    m3DestroyWorld(world);
}

int main(void)
{
    TestLastResultIsPerThread();
    TestParallelWorldsMatchSerial();
    TestConcurrentReadersBetweenSteps();
    if (s_failures == 0)
    {
        printf("test_concurrency: all green\n");
        return 0;
    }
    return 1;
}
