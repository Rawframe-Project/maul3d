// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The force gate: host forces and impulses land by the
// analytic books (F = m a, T = I alpha, off-center impulses split
// into the linear and angular ledgers), live for exactly one step
// when they are forces, and refuse hostile or non-dynamic targets
// quietly.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* FreeSpace(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f}; // clean ledgers
    return NewWorld(&def);
}

static m3BodyId UnitCube(m3World* world, m3Pos3 at)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    bd.linearDamping = 0.0f; // clean analytic books
    bd.angularDamping = 0.0f;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    // half extents 0.5: volume 1, density 1: mass 1, I = m/6 = 1/6.
    NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return body;
}

static void TestForceAndTorqueBooks(void)
{
    m3World* world = FreeSpace();
    m3BodyId cube = UnitCube(world, (m3Pos3){0.0, 0.0, 0.0});

    // A 2 N force held for sixty steps of one second total on a
    // one kilogram cube: v = F t / m = 2 m/s. The force must be
    // re-applied per step (it clears after each one).
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Body_ApplyForce(world, cube, (m3Vec3){2.0f, 0.0f, 0.0f});
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(world, cube);
    CHECK(fabsf(v.x - 2.0f) < 0.01f, "F equals m a over one second");

    // The accumulator clears: sixty more steps with NO application
    // add nothing.
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v2 = m3Body_GetLinearVelocity(world, cube);
    CHECK(fabsf(v2.x - v.x) < 1.0e-6f, "a force lives for exactly one step");

    // Torque: T = I alpha; unit cube I = 1/6, T = 0.5 N m for one
    // second: w = T t / I = 3 rad/s.
    // Far from the drifting force cube: at x = 5 the 2 m/s drifter
    // would sail straight through the spin measurement.
    m3BodyId top = UnitCube(world, (m3Pos3){0.0, 50.0, 0.0});
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Body_ApplyTorque(world, top, (m3Vec3){0.0f, 0.5f, 0.0f});
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 w = m3Body_GetAngularVelocity(world, top);
    CHECK(fabsf(w.y - 3.0f) < 0.02f, "T equals I alpha over one second");
    m3DestroyWorld(world);
}

static void TestImpulseBooks(void)
{
    m3World* world = FreeSpace();
    m3BodyId cube = UnitCube(world, (m3Pos3){0.0, 0.0, 0.0});

    // A 3 kg m/s impulse at the center: v = J / m = 3, no spin.
    m3Body_ApplyLinearImpulse(world, cube, (m3Vec3){3.0f, 0.0f, 0.0f});
    m3Vec3 v = m3Body_GetLinearVelocity(world, cube);
    m3Vec3 w = m3Body_GetAngularVelocity(world, cube);
    CHECK(fabsf(v.x - 3.0f) < 1.0e-6f, "a center impulse is J over m");
    CHECK(fabsf(w.x) + fabsf(w.y) + fabsf(w.z) < 1.0e-6f, "a center impulse adds no spin");

    // The same impulse at a corner-height point: SAME linear
    // change, plus spin = invI (r x J): r = (0, 0.5, 0),
    // J = (3,0,0): r x J = (0,0,-1.5), invI = 6: w.z = -9.
    m3BodyId two = UnitCube(world, (m3Pos3){5.0, 0.0, 0.0});
    m3Body_ApplyLinearImpulseAtPoint(world, two, (m3Vec3){3.0f, 0.0f, 0.0f},
                                     (m3Pos3){5.0, 0.5, 0.0});
    m3Vec3 v2 = m3Body_GetLinearVelocity(world, two);
    m3Vec3 w2 = m3Body_GetAngularVelocity(world, two);
    CHECK(fabsf(v2.x - 3.0f) < 1.0e-5f, "an off-center impulse keeps the linear ledger");
    CHECK(fabsf(w2.z + 9.0f) < 0.01f, "the angular ledger takes invI times r cross J");

    // Angular impulse direct: L = 0.5: w = invI L = 3.
    m3BodyId three = UnitCube(world, (m3Pos3){10.0, 0.0, 0.0});
    m3Body_ApplyAngularImpulse(world, three, (m3Vec3){0.0f, 0.0f, 0.5f});
    CHECK(fabsf(m3Body_GetAngularVelocity(world, three).z - 3.0f) < 0.01f,
          "an angular impulse is invI times L");

    // The books hold at any size: a 1 km sphere of density 1 has
    // I = 0.4 m r^2 = 0.4 (4/3 pi 1e9) 1e6, about 1.68e15, so L = I
    // turns it at 1 rad/s.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 5000.0, 0.0};
    m3BodyId hill = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 1000.0f};
    NewSphereShape(world, hill, &sd, &ball);
    float inertia = 0.4f * (4.0f / 3.0f * 3.14159265f * 1.0e9f) * 1.0e6f;
    m3Body_ApplyAngularImpulse(world, hill, (m3Vec3){0.0f, 0.0f, inertia});
    CHECK(fabsf(m3Body_GetAngularVelocity(world, hill).z - 1.0f) < 1.0e-3f,
          "a large body's angular impulse is invI times L too");
    m3DestroyWorld(world);
}

static void TestForceContracts(void)
{
    m3World* world = FreeSpace();
    m3BodyId cube = UnitCube(world, (m3Pos3){0.0, 0.0, 0.0});
    m3BodyDef sd2 = m3DefaultBodyDef();
    sd2.position = (m3Pos3){3.0, 0.0, 0.0};
    m3BodyId rock = NewBody(world, &sd2); // static

    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    m3Body_ApplyForce(world, cube, (m3Vec3){bad, 0.0f, 0.0f});          // hostile: no-op
    m3Body_ApplyLinearImpulse(world, rock, (m3Vec3){5.0f, 0.0f, 0.0f}); // static: no-op
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(fabsf(m3Body_GetLinearVelocity(world, cube).x) < 1.0e-6f, "a NaN force never lands");
    CHECK(m3Body_GetPosition(world, rock).x == 3.0, "a static target never moves");

    DestroyBody(world, cube);
    m3Body_ApplyForce(world, cube, (m3Vec3){1.0f, 0.0f, 0.0f}); // stale: no-op
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    m3DestroyWorld(world);
}

typedef enum Stop
{
    stop_static,
    stop_kinematic,
    stop_sleep,
    stop_disable,
} Stop;

static void StopMoving(m3World* world, m3BodyId body, Stop stop, bool stopped)
{
    if (stop == stop_static || stop == stop_kinematic)
    {
        m3BodyType type = stop == stop_static ? m3_staticBody : m3_kinematicBody;
        CHECK(m3Body_SetType(world, body, stopped ? type : m3_dynamicBody) == m3_success,
              "the type changes");
    }
    else if (stop == stop_sleep)
    {
        m3Body_SetAwake(world, body, !stopped);
    }
    else
    {
        m3Body_Enable(world, body, !stopped);
    }
}

// A force acts over the next step or not at all: a body that stops moving
// before that step drops it, and moves again from rest, however many
// steps later.
static void TestStoppedBodyDropsItsForce(void)
{
    static const char* const names[] = {
        "a body turned static drops its force",
        "a body turned kinematic drops its force",
        "a body put to sleep drops its force",
        "a disabled body drops its force",
    };
    for (int32_t stop = stop_static; stop <= stop_disable; ++stop)
    {
        m3World* world = FreeSpace();
        m3BodyId cube = UnitCube(world, (m3Pos3){0.0, 0.0, 0.0});
        m3Body_ApplyForce(world, cube, (m3Vec3){600.0f, 0.0f, 0.0f});
        m3Body_ApplyTorque(world, cube, (m3Vec3){0.0f, 60.0f, 0.0f});
        StopMoving(world, cube, (Stop)stop, true);
        // The engine wrote this state, so its own restore takes it.
        int64_t size = m3World_GetSnapshotSize(world);
        uint8_t* snap = (uint8_t*)malloc((size_t)size);
        m3World* copy = FreeSpace();
        CHECK(snap != NULL && Snapshot(world, snap, size) == size && Restore(copy, snap, size),
              "the stopped body's snapshot restores");
        m3DestroyWorld(copy);
        free(snap);
        for (int32_t i = 0; i < 3; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        StopMoving(world, cube, (Stop)stop, false);
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Vec3 v = m3Body_GetLinearVelocity(world, cube);
        m3Vec3 w = m3Body_GetAngularVelocity(world, cube);
        CHECK(v.x == 0.0f && w.y == 0.0f, names[stop]);
        m3DestroyWorld(world);
    }
}

// A disabled body takes no force or torque, live or on replay: the step
// does not move it, so nothing would act over the next step.
static void TestDisabledBodyTakesNoForce(void)
{
    m3World* world = FreeSpace();
    static uint8_t tape[4096];
    CHECK(StartJournal(world, tape, (int64_t)sizeof(tape)), "the tape opens");
    m3BodyId off = UnitCube(world, (m3Pos3){0.0, 0.0, 0.0});
    m3BodyId on = UnitCube(world, (m3Pos3){3.0, 0.0, 0.0});
    m3Body_Enable(world, off, false);
    uint64_t misuse = m3World_GetCounters(world).misuseCount;
    uint64_t hash = m3World_Hash(world);
    m3Body_ApplyForce(world, off, (m3Vec3){1.0f, 0.0f, 0.0f});
    m3Body_ApplyTorque(world, off, (m3Vec3){1.0f, 0.0f, 0.0f});
    m3Body_ApplyForceAtPoint(world, off, (m3Vec3){1.0f, 0.0f, 0.0f}, (m3Pos3){0.0, 1.0, 0.0});
    CHECK(m3World_GetCounters(world).misuseCount == misuse + 3 && m3World_Hash(world) == hash,
          "each apply call refuses the disabled body as misuse");
    m3Body_ApplyLinearImpulse(world, off, (m3Vec3){1.0f, 0.0f, 0.0f});
    CHECK(m3Body_GetLinearVelocity(world, off).x == 1.0f, "an impulse still lands at once");

    // Each call recorded on the enabled body, pointed at the disabled one,
    // is refused on replay as the call refuses it.
    static const float marks[] = {123.25f, 234.5f, 345.75f};
    m3Body_ApplyForce(world, on, (m3Vec3){marks[0], 0.0f, 0.0f});
    m3Body_ApplyTorque(world, on, (m3Vec3){marks[1], 0.0f, 0.0f});
    m3Body_ApplyForceAtPoint(world, on, (m3Vec3){marks[2], 0.0f, 0.0f}, (m3Pos3){3.0, 1.0, 0.0});
    int64_t bytes = StopJournal(world);
    m3World* twin = FreeSpace();
    CHECK(ReplayJournal(twin, tape, bytes), "the tape replays");
    m3DestroyWorld(twin);
    static const char* const names[] = {
        "a replayed force on the disabled body is refused",
        "a replayed torque on the disabled body is refused",
        "a replayed force at a point on the disabled body is refused",
    };
    static uint8_t forged[4096];
    for (int32_t c = 0; c < 3; ++c)
    {
        int64_t at = -1;
        for (int64_t i = 16 + (int64_t)sizeof(m3BodyId); i + 4 <= bytes && at < 0; ++i)
        {
            at = memcmp(tape + i, &marks[c], sizeof(float)) == 0 ? i : -1;
        }
        CHECK(at > 0, "the call is on the tape");
        if (at < 0)
        {
            continue;
        }
        m3BodyId named;
        memcpy(&named, tape + at - (int64_t)sizeof(named), sizeof(named));
        CHECK(named.index1 == on.index1 && named.generation == on.generation,
              "and names the enabled body");
        memcpy(forged, tape, (size_t)bytes);
        memcpy(forged + at - (int64_t)sizeof(off), &off, sizeof(off));
        twin = FreeSpace();
        CHECK(!ReplayJournal(twin, forged, bytes), names[c]);
        m3DestroyWorld(twin);
    }
    m3DestroyWorld(world);
}

int main(void)
{
    TestForceAndTorqueBooks();
    TestImpulseBooks();
    TestForceContracts();
    TestStoppedBodyDropsItsForce();
    TestDisabledBodyTakesNoForce();
    if (s_failures == 0)
    {
        printf("test_force: all green\n");
        return 0;
    }
    printf("test_force: %d failure(s)\n", s_failures);
    return 1;
}
