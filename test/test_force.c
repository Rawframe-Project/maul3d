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
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(world, cube);
    CHECK(fabsf(v.x - 2.0f) < 0.01f, "F equals m a over one second");

    // The accumulator clears: sixty more steps with NO application
    // add nothing.
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v2 = m3Body_GetLinearVelocity(world, cube);
    CHECK(fabsf(v2.x - v.x) < 1.0e-6f, "a force lives for exactly one step");

    // Torque: T = I alpha; unit cube I = 1/6, T = 0.5 N m for one
    // second: w = T t / I = 3 rad/s.
    // Far from the drifting force cube: the first draft parked
    // this at x = 5 and the 2 m/s drifter sailed straight through
    // the spin measurement.
    m3BodyId top = UnitCube(world, (m3Pos3){0.0, 50.0, 0.0});
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Body_ApplyTorque(world, top, (m3Vec3){0.0f, 0.5f, 0.0f});
        m3World_Step(world, 1.0f / 60.0f, 4);
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
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(fabsf(m3Body_GetLinearVelocity(world, cube).x) < 1.0e-6f, "a NaN force never lands");
    CHECK(m3Body_GetPosition(world, rock).x == 3.0, "a static target never moves");

    DestroyBody(world, cube);
    m3Body_ApplyForce(world, cube, (m3Vec3){1.0f, 0.0f, 0.0f}); // stale: no-op
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3DestroyWorld(world);
}

int main(void)
{
    TestForceAndTorqueBooks();
    TestImpulseBooks();
    TestForceContracts();
    if (s_failures == 0)
    {
        printf("test_force: all green\n");
        return 0;
    }
    printf("test_force: %d failure(s)\n", s_failures);
    return 1;
}
