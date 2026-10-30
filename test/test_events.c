// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The events gate: hit events fire once per impact at the
// analytic speed and only for opted-in shapes; body move events
// mirror every mover's transform and mark the sleep edge; the
// pre-solve veto builds a one-way platform under the loud purity
// contract; streams replay identically on twins.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* PlaneWorld(m3ShapeId* outFloor)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId fs = NewPlaneShape(world, ground, &sd, &floor);
    if (outFloor != NULL)
    {
        *outFloor = fs;
    }
    return world;
}

static void TestHitEvents(void)
{
    // A ball dropped from 2m hits at sqrt(2*g*(h - r)) with g = 10:
    // about 5.48 m/s. Opted in, the hit fires once with that speed;
    // opted out or gated by a raised threshold, silence.
    for (int32_t mode = 0; mode < 3; ++mode)
    {
        m3ShapeId floor;
        m3World* world = PlaneWorld(&floor);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId ball = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.5f};
        m3ShapeId shape = NewSphereShape(world, ball, &sd, &s);
        (void)shape;
        if (mode >= 1)
        {
            m3Shape_EnableHitEvents(world, floor, true);
            CHECK(m3Shape_IsHitEventsEnabled(world, floor), "hit flag reads back");
        }
        if (mode == 2)
        {
            m3World_SetHitEventThreshold(world, 50.0f);
        }
        int32_t totalHits = 0;
        double speed = 0.0;
        double normalY = 0.0;
        double pointY = 1.0e9;
        for (int32_t i = 0; i < 90; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            const m3ContactHitEvent* ev = m3World_GetContactEvents(world).hitEvents;
            int32_t n = m3World_GetContactEvents(world).hitCount;
            if (n > 0)
            {
                speed = (double)ev[0].approachSpeed;
                normalY = (double)ev[0].normal.y;
                pointY = ev[0].point.y;
            }
            totalHits += n;
        }
        if (mode == 0)
        {
            CHECK(totalHits == 0, "no opt-in, no hit events");
        }
        else if (mode == 1)
        {
            CHECK(totalHits == 1, "one impact, one hit event");
            CHECK(speed > 5.0 && speed < 6.0, "the hit speed is the analytic impact speed");
            CHECK(fabs(normalY) > 0.99, "the hit normal is the plane normal");
            CHECK(fabs(pointY) < 0.2, "the hit point sits at the floor");
        }
        else
        {
            CHECK(totalHits == 0, "a raised threshold silences the hit");
        }
        CHECK(m3World_GetContactEvents(world).hitsDropped == 0, "nothing dropped");
        m3DestroyWorld(world);
    }
}

static void TestBodyMoveEvents(void)
{
    // The falling crate reports its transform every step; the step
    // it drops off to sleep is marked, then silence until disturbed.
    m3World* world = PlaneWorld(NULL);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    int32_t sleepEdges = 0;
    int32_t silentSteps = 0;
    bool transformsMatch = true;
    for (int32_t i = 0; i < 400; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        const m3BodyMoveEvent* ev = m3World_GetBodyEvents(world).moveEvents;
        int32_t n = m3World_GetBodyEvents(world).moveCount;
        bool found = false;
        for (int32_t k = 0; k < n; ++k)
        {
            if (ev[k].bodyId.index1 == crate.index1)
            {
                found = true;
                m3Pos3 p = m3Body_GetPosition(world, crate);
                if (fabs(ev[k].transform.p.y - p.y) > 1.0e-12)
                {
                    transformsMatch = false;
                }
                if (ev[k].fellAsleep)
                {
                    sleepEdges += 1;
                }
            }
        }
        if (!found)
        {
            silentSteps += 1;
        }
    }
    CHECK(transformsMatch, "move events mirror the body transform");
    CHECK(sleepEdges == 1, "exactly one fell-asleep edge");
    CHECK(silentSteps > 100, "a sleeping body emits nothing");
    CHECK(!m3Body_IsAwake(world, crate), "the crate did sleep");
    m3DestroyWorld(world);
}

// The one-way platform: pass upward, land coming down. Pure
// function of its arguments, as the loud contract demands.
static bool OneWayVeto(m3ShapeId shapeA, m3ShapeId shapeB, m3Pos3 point, m3Vec3 normal, void* ctx)
{
    (void)shapeA;
    (void)shapeB;
    (void)point;
    int* calls = (int*)ctx;
    *calls += 1;
    // The normal runs from shape A to shape B. Support only when it
    // carries an upward component: approaches from below veto out.
    return fabsf(normal.y) > 0.5f && normal.y > 0.0f ? true : false;
}

static void TestPreSolveOneWay(void)
{
    // A ball flung upward through a thin plate: the veto lets it
    // through on the way up, then it lands ON the plate.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &fl);
    m3BodyDef pd = m3DefaultBodyDef();
    pd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId plate = NewBody(world, &pd);
    m3ShapeId plateShape = NewBoxShape(world, plate, &sd, (m3Vec3){2.0f, 0.1f, 2.0f});
    m3Shape_EnablePreSolveEvents(world, plateShape, true);
    CHECK(m3Shape_IsPreSolveEventsEnabled(world, plateShape), "pre-solve flag reads back");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
    NewSphereShape(world, ball, &sd, &s);

    int calls = 0;
    m3World_SetPreSolveCallback(world, OneWayVeto, &calls);
    m3Body_SetLinearVelocity(world, ball, (m3Vec3){0.0f, 12.0f, 0.0f});
    double peak = 0.0;
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        double y = m3Body_GetPosition(world, ball).y;
        if (y > peak)
        {
            peak = y;
        }
    }
    CHECK(peak > 4.0, "the ball passes the plate on the way up");
    double restY = m3Body_GetPosition(world, ball).y;
    CHECK(restY > 3.3 && restY < 3.8, "the ball lands ON the plate coming down");
    CHECK(calls > 0, "the veto was consulted");

    // Clearing the callback restores plain solid contact.
    m3World_SetPreSolveCallback(world, NULL, NULL);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    m3DestroyWorld(world);
}

static void TestEventTwinsAndJointStream(void)
{
    // Twin worlds running the same script produce byte-identical
    // hit streams; the joint break stream exists and stays empty
    // until joints learn to break (the next slice).
    uint64_t hitDigest[2] = {0, 0};
    for (int32_t run = 0; run < 2; ++run)
    {
        m3ShapeId floor;
        m3World* world = PlaneWorld(&floor);
        m3Shape_EnableHitEvents(world, floor, true);
        for (int32_t b = 0; b < 4; ++b)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.position = (m3Pos3){(double)b * 1.5, 1.5 + (double)b * 0.5, 0.0};
            m3BodyId ball = NewBody(world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
            NewSphereShape(world, ball, &sd, &s);
        }
        uint64_t digest = 1469598103934665603ull;
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            const m3ContactHitEvent* ev = m3World_GetContactEvents(world).hitEvents;
            int32_t n = m3World_GetContactEvents(world).hitCount;
            for (int32_t k = 0; k < n; ++k)
            {
                m3ContactHitEvent hit = ev[k];
                const uint8_t* bytes = (const uint8_t*)&hit;
                for (size_t j = 0; j < sizeof(m3ContactHitEvent); ++j)
                {
                    digest = (digest ^ bytes[j]) * 1099511628211ull;
                }
            }
            int32_t jb = m3World_GetJointEvents(world).breakCount;
            CHECK(jb == 0, "the joint break stream is armed and empty");
        }
        hitDigest[run] = digest;
        m3DestroyWorld(world);
    }
    CHECK(hitDigest[0] == hitDigest[1], "twin hit streams are byte-identical");
}

static void TestHostileEvents(void)
{
    m3ShapeId floor;
    m3World* world = PlaneWorld(&floor);
    m3World_SetHitEventThreshold(world, -1.0f);
    m3World_SetHitEventThreshold(world, NAN);
    m3ShapeId stale = floor;
    stale.index1 += 999;
    m3Shape_EnableHitEvents(world, stale, true);
    CHECK(!m3Shape_IsHitEventsEnabled(world, stale), "stale flag set refuses");
    m3ContactEvents none = m3World_GetContactEvents(world);
    CHECK(none.hitCount == 0 && none.hitsDropped == 0, "no events before any step");
    m3DestroyWorld(world);
}

// What one step reports for a pair whose shape was destroyed after the
// pair touched, when the next create takes the freed slot.
typedef struct ReuseEvents
{
    int32_t begins;
    int32_t ends;
    bool newerNamed; // a begin names the shape that took the slot
} ReuseEvents;

// Shape 0, a box (a sensor when sensor is set), overlaps ball X, shape
// 1, with no gravity. After a step, X is destroyed and the next create
// takes slot 1: far from the box, or over it again when overlap is set.
// The new shape is a sensor when the box is, so it never meets X's
// pair as a fresh pair of its own unless it overlaps.
static ReuseEvents ReuseAfterDestroy(bool sensor, bool overlap)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3ShapeDef boxDef = m3DefaultShapeDef();
    boxDef.isSensor = sensor;
    m3ShapeId box = NewBoxShape(world, NewBody(world, &gd), &boxDef, (m3Vec3){1.0f, 1.0f, 1.0f});
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef ballDef = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId x = NewSphereShape(world, NewBody(world, &bd), &ballDef, &sphere);
    CHECK(box.index1 == 1 && x.index1 == 2, "the box takes slot 0 and the ball slot 1");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t touched = sensor ? m3World_GetSensorEvents(world).beginCount
                             : m3World_GetContactEvents(world).beginCount;
    CHECK(touched == 1, "the box and the ball begin touching");

    CHECK(m3DestroyShape(world, x) == m3_success, "the ball's shape is destroyed");
    gd.position = overlap ? (m3Pos3){0.0, 0.0, 0.0} : (m3Pos3){50.0, 0.0, 0.0};
    gd.type = overlap ? m3_dynamicBody : m3_staticBody;
    m3ShapeDef newerDef = m3DefaultShapeDef();
    newerDef.isSensor = sensor && !overlap;
    m3ShapeId newer = NewSphereShape(world, NewBody(world, &gd), &newerDef, &sphere);
    CHECK(newer.index1 == x.index1 && newer.generation != x.generation,
          "the next create takes the freed slot");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);

    ReuseEvents out = {0, 0, false};
    m3ContactEvents contacts = m3World_GetContactEvents(world);
    m3SensorEvents sensors = m3World_GetSensorEvents(world);
    out.begins = contacts.beginCount + sensors.beginCount;
    out.ends = contacts.endCount + sensors.endCount;
    for (int32_t i = 0; i < contacts.beginCount; ++i)
    {
        out.newerNamed =
            out.newerNamed || (contacts.beginEvents[i].shapeIdB.index1 == newer.index1 &&
                               contacts.beginEvents[i].shapeIdB.generation == newer.generation);
    }
    for (int32_t i = 0; i < sensors.beginCount; ++i)
    {
        out.newerNamed = out.newerNamed ||
                         (sensors.beginEvents[i].visitorShapeId.index1 == newer.index1 &&
                          sensors.beginEvents[i].visitorShapeId.generation == newer.generation);
    }
    m3DestroyWorld(world);
    return out;
}

static void TestDestroyedPairAndReusedSlot(void)
{
    // A destroyed shape's pair ends silently: no end event, whatever
    // takes its slot, since an id would name a shape that never touched.
    for (int32_t sensor = 0; sensor < 2; ++sensor)
    {
        ReuseEvents far = ReuseAfterDestroy(sensor != 0, false);
        if (far.begins != 0 || far.ends != 0)
        {
            printf("  %s, far: %d begins, %d ends\n", sensor ? "sensor" : "contact",
                   (int)far.begins, (int)far.ends);
        }
        CHECK(far.begins == 0 && far.ends == 0,
              "a destroyed shape's pair emits nothing when its slot is taken far away");
        ReuseEvents over = ReuseAfterDestroy(sensor != 0, true);
        if (over.begins != 1 || over.ends != 0 || !over.newerNamed)
        {
            printf("  %s, over: %d begins, %d ends, newer named %d\n",
                   sensor ? "sensor" : "contact", (int)over.begins, (int)over.ends,
                   (int)over.newerNamed);
        }
        CHECK(over.begins == 1 && over.ends == 0 && over.newerNamed,
              "the shape that takes the slot over the box begins its own touch");
    }
}

// The same reuse with the ball asleep on the box, so the pair waits in
// the frozen buffer: the static shape that takes the slot far away gets
// no event, and the world steps as one restored from a snapshot taken
// right after the destroy, which rebuilds its pairs from the tree.
static void TestColdPairAndReusedSlot(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, NewBody(world, &gd), &sd, (m3Vec3){1.0f, 1.0f, 1.0f});
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.5, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId x = NewSphereShape(world, ball, &sd, &sphere);
    for (int32_t i = 0; i < 600 && m3Body_IsAwake(world, ball); ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m3Body_IsAwake(world, ball), "the ball falls asleep on the box");

    CHECK(m3DestroyShape(world, x) == m3_success, "the sleeping ball's shape is destroyed");
    gd.position = (m3Pos3){50.0, 0.0, 0.0};
    m3ShapeId newer = NewSphereShape(world, NewBody(world, &gd), &sd, &sphere);
    CHECK(newer.index1 == x.index1, "a static shape far away takes the freed slot");
    int64_t size = m3World_GetSnapshotSize(world);
    void* snap = malloc((size_t)size);
    int64_t written = 0;
    CHECK(snap != NULL && m3World_Snapshot(world, snap, size, &written) == m3_success,
          "the world snapshots after the destroy");
    m3World* twin = NewWorld(&def);
    CHECK(m3World_Restore(twin, snap, written) == m3_success, "a twin restores the snapshot");

    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    (void)m3World_Step(twin, 1.0f / 60.0f, 4);
    m3ContactEvents events = m3World_GetContactEvents(world);
    CHECK(events.beginCount == 0 && events.endCount == 0,
          "the cold pair of a destroyed shape emits nothing");
    CHECK(m3World_Hash(world) == m3World_Hash(twin),
          "the world steps as the twin restored after the destroy");
    free(snap);
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

// A floor plane at slot 0 and six touching balls at slots 1 to 6 on it:
// a middle ball is the higher slot of its pairs with the floor and its
// left neighbour, and the lower slot of its pair with its right one.
static m3World* BallRow(m3ShapeId balls[6])
{
    m3World* world = PlaneWorld(NULL);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    for (int32_t i = 0; i < 6; ++i)
    {
        bd.position = (m3Pos3){0.99 * i, 0.5, 0.0};
        balls[i] = NewSphereShape(world, NewBody(world, &bd), &sd, &sphere);
    }
    for (int32_t i = 0; i < 2; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    return world;
}

static bool NamesShape(m3ContactEvents events, m3ShapeId shape)
{
    bool named = false;
    for (int32_t i = 0; i < events.beginCount; ++i)
    {
        named = named || events.beginEvents[i].shapeIdA.index1 == shape.index1 ||
                events.beginEvents[i].shapeIdB.index1 == shape.index1;
    }
    for (int32_t i = 0; i < events.endCount; ++i)
    {
        named = named || events.endEvents[i].shapeIdA.index1 == shape.index1 ||
                events.endEvents[i].shapeIdB.index1 == shape.index1;
    }
    return named;
}

static void TestForgetBothHalvesOfAShape(void)
{
    // The middle ball's pairs on both sides of its slot die with it: the
    // shape that takes the slot far away is named by no event.
    m3ShapeId balls[6];
    m3World* world = BallRow(balls);
    CHECK(balls[0].index1 == 2 && balls[3].index1 == 5, "the floor holds slot 0, the balls 1 to 6");
    CHECK(m3DestroyShape(world, balls[3]) == m3_success, "the middle ball's shape is destroyed");
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){50.0, 5.0, 0.0};
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId newer = NewSphereShape(world, NewBody(world, &gd), &sd, &sphere);
    CHECK(newer.index1 == balls[3].index1, "a static shape far away takes the slot");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(!NamesShape(m3World_GetContactEvents(world), newer),
          "no event names the shape that took a slot of both halves");
    m3DestroyWorld(world);
}

static void TestForgetAfterThePairsChange(void)
{
    // One destroy builds the pair index; a step changes the pairs; the
    // next destroy must find them anew. A twin restored before it builds
    // its index from the restored pairs, so both forget the same pairs.
    m3ShapeId balls[6];
    m3World* world = BallRow(balls);
    CHECK(m3DestroyShape(world, balls[0]) == m3_success, "the first ball's shape is destroyed");
    CHECK(m3DestroyShape(world, balls[1]) == m3_success, "and the second, on the same index");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t size = m3World_GetSnapshotSize(world);
    void* snap = malloc((size_t)size);
    int32_t written = 0;
    CHECK(snap != NULL && m3World_Snapshot(world, snap, size, &written) == m3_success,
          "the world snapshots after the step");
    m3World* twin = PlaneWorld(NULL);
    CHECK(m3World_Restore(twin, snap, written) == m3_success, "a twin restores it");
    CHECK(m3DestroyShape(world, balls[4]) == m3_success &&
              m3DestroyShape(twin, balls[4]) == m3_success,
          "both destroy the fifth ball's shape");
    CHECK(m3World_Hash(world) == m3World_Hash(twin), "both forget the same pairs");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    (void)m3World_Step(twin, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(world) == m3World_Hash(twin), "and step alike");
    free(snap);
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

// A ball asleep on the floor is destroyed and a sphere that takes its
// slot is put to sleep before any step. The frozen buffer then holds the
// dead ball's floor pair and the new shape's own, under one slot: the
// next pair update must drop the first and keep the second, as a twin
// restored after the sleep rebuilds only the second.
static void TestSleepInAReusedSlot(void)
{
    m3World* world = PlaneWorld(NULL);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeId x = NewSphereShape(world, ball, &sd, &sphere);
    for (int32_t i = 0; i < 600 && m3Body_IsAwake(world, ball); ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m3Body_IsAwake(world, ball), "the ball falls asleep on the floor");
    CHECK(m3DestroyShape(world, x) == m3_success, "the sleeping ball's shape is destroyed");
    bd.position = (m3Pos3){3.0, 0.5, 0.0};
    m3BodyId sleeper = NewBody(world, &bd);
    m3ShapeId newer = NewSphereShape(world, sleeper, &sd, &sphere);
    CHECK(newer.index1 == x.index1, "a sphere on the floor takes the freed slot");
    m3Body_SetAwake(world, sleeper, false);
    CHECK(!m3Body_IsAwake(world, sleeper), "and is put to sleep before any step");
    int32_t size = m3World_GetSnapshotSize(world);
    void* snap = malloc((size_t)size);
    int32_t written = 0;
    CHECK(snap != NULL && m3World_Snapshot(world, snap, size, &written) == m3_success,
          "the world snapshots after the sleep");
    m3World* twin = PlaneWorld(NULL);
    CHECK(m3World_Restore(twin, snap, written) == m3_success, "a twin restores it");
    for (int32_t i = 0; i < 2; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        (void)m3World_Step(twin, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) == m3World_Hash(twin),
          "the sleeping sphere keeps its floor pair, as the restored twin does");
    free(snap);
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

int main(void)
{
    TestHitEvents();
    TestBodyMoveEvents();
    TestPreSolveOneWay();
    TestEventTwinsAndJointStream();
    TestHostileEvents();
    TestDestroyedPairAndReusedSlot();
    TestColdPairAndReusedSlot();
    TestForgetBothHalvesOfAShape();
    TestForgetAfterThePairsChange();
    TestSleepInAReusedSlot();
    if (s_failures == 0)
    {
        printf("test_events: all green\n");
        return 0;
    }
    printf("test_events: %d failure(s)\n", s_failures);
    return 1;
}
