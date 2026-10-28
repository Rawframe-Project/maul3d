// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The angular cap gate: the spin clamp and the fast-rotation flag
// that waives it. A legal tumbler keeps its spin, a pathological
// spinner clamps to the cap, a flagged wheel sails past it, the cap
// tunes at runtime, the flag survives motion-lock writes, and hostile
// tape bytes bounce loudly.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Motion locks from bits 0..5: linear x, y, z, then angular x, y, z.
static m3MotionLocks Locks(uint32_t bits)
{
    m3MotionLocks l = {(bits & 0x01u) != 0, (bits & 0x02u) != 0, (bits & 0x04u) != 0,
                       (bits & 0x08u) != 0, (bits & 0x10u) != 0, (bits & 0x20u) != 0};
    return l;
}

static uint32_t Bits(m3MotionLocks l)
{
    return (l.linearX ? 0x01u : 0u) | (l.linearY ? 0x02u : 0u) | (l.linearZ ? 0x04u : 0u) |
           (l.angularX ? 0x08u : 0u) | (l.angularY ? 0x10u : 0u) | (l.angularZ ? 0x20u : 0u);
}

// Zero gravity, no contacts: spin evolves through damping, gyro,
// and the cap alone. Cubes have isotropic inertia, so a clean spin
// about any axis is gyroscopically silent.
static m3World* SpinWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    return NewWorld(&def);
}

static m3BodyId Spinner(m3World* world, double x, float spinY)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, 0.0, 0.0};
    bd.angularVelocity = (m3Vec3){0.0f, spinY, 0.0f};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return body;
}

static float SpinOf(const m3World* world, m3BodyId body)
{
    m3Vec3 w = m3Body_GetAngularVelocity(world, body);
    return sqrtf(w.x * w.x + w.y * w.y + w.z * w.z);
}

static void TestCapAndEscape(void)
{
    m3World* world = SpinWorld();
    m3BodyId legal = Spinner(world, -4.0, 50.0f);
    m3BodyId wild = Spinner(world, 0.0, 2000.0f);
    m3BodyId wheel = Spinner(world, 4.0, 2000.0f);
    m3Body_EnableFastRotation(world, wheel, true);
    CHECK(m3Body_IsFastRotationEnabled(world, wheel), "the flag reads back");
    CHECK(!m3Body_IsFastRotationEnabled(world, wild), "the default is off");
    CHECK(Bits(m3Body_GetMotionLocks(world, wheel)) == 0u, "the flag is not a motion lock");
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(SpinOf(world, legal) > 49.9f, "a legal tumbler keeps its spin");
    CHECK(SpinOf(world, wild) < 800.5f, "the wild spinner is capped");
    CHECK(SpinOf(world, wild) > 799.0f, "the cap clamps, it does not kill");
    CHECK(SpinOf(world, wheel) > 1999.0f, "the flagged wheel sails past the cap");
    m3DestroyWorld(world);
}

// The flag waives the world's cap but not the engine's speed limit
// (1e5 rad/s). A 1e7 N m torque on the unit cube (I = 1/6) would add
// 2.5e5 rad/s in the first substep; the step holds it at the limit
// instead.
static void TestSpeedLimitHoldsFlaggedSpin(void)
{
    m3World* world = SpinWorld();
    m3BodyId wheel = Spinner(world, 0.0, 0.0f);
    m3Body_EnableFastRotation(world, wheel, true);
    m3Body_ApplyTorque(world, wheel, (m3Vec3){0.0f, 1.0e7f, 0.0f});
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(SpinOf(world, wheel) <= 1.0e5f * 1.0001f, "the speed limit holds a flagged body");
    CHECK(SpinOf(world, wheel) > 0.999e5f, "the limit clamps, it does not kill");
    m3DestroyWorld(world);
}

static void TestRuntimeCapAndLockInterplay(void)
{
    m3World* world = SpinWorld();
    m3BodyId top = Spinner(world, 0.0, 50.0f);
    m3World_SetMaximumAngularSpeed(world, 10.0f);
    // Hostile knob values bounce at the public wall and change
    // nothing: the spinner still lands on the 10 rad/s cap.
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    m3World_SetMaximumAngularSpeed(world, bad);
    m3World_SetMaximumAngularSpeed(world, -5.0f);
    m3World_SetMaximumAngularSpeed(world, 0.0f);
    for (int32_t i = 0; i < 10; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(SpinOf(world, top) < 10.5f && SpinOf(world, top) > 9.5f, "the runtime cap applies");
    // The flag and the locks share a byte but never each other's
    // bits: lock writes preserve the flag, the flag write preserves
    // the locks, and the locks getter never leaks bit 6.
    m3Body_EnableFastRotation(world, top, true);
    m3Body_SetMotionLocks(world, top, Locks(0x3Fu));
    CHECK(m3Body_IsFastRotationEnabled(world, top), "locks do not clobber the flag");
    CHECK(Bits(m3Body_GetMotionLocks(world, top)) == 0x3Fu, "the locks read back clean");
    m3Body_SetMotionLocks(world, top, Locks(0u));
    CHECK(m3Body_IsFastRotationEnabled(world, top), "clearing locks keeps the flag");
    m3Body_EnableFastRotation(world, top, false);
    CHECK(!m3Body_IsFastRotationEnabled(world, top), "the flag clears");
    CHECK(Bits(m3Body_GetMotionLocks(world, top)) == 0u, "the flag write left the locks alone");
    m3BodyId stale = {99, 7};
    m3Body_EnableFastRotation(world, stale, true);
    CHECK(!m3Body_IsFastRotationEnabled(world, stale), "a stale id bounces");
    m3DestroyWorld(world);
}

static void TestHashFoldsOffDefaultOnly(void)
{
    m3World* a = SpinWorld();
    m3World* b = SpinWorld();
    CHECK(m3World_Hash(a) == m3World_Hash(b), "twin worlds start identical");
    m3World_SetMaximumAngularSpeed(b, 799.0f);
    CHECK(m3World_Hash(a) != m3World_Hash(b), "an off-default cap folds");
    m3World_SetMaximumAngularSpeed(b, 800.0f);
    CHECK(m3World_Hash(a) == m3World_Hash(b), "the exact default does not fold");
    m3DestroyWorld(a);
    m3DestroyWorld(b);
}

static void TestHostileTape(void)
{
    // The tape is the 16-byte state prefix, then [op:4][bytes:4][payload].
    // One op per tape, then surgical byte damage: every mutation must
    // fail the replay loudly and a clean tape must land.
    static uint8_t tape[64];
    m3World* world = SpinWorld();
    m3BodyId body = Spinner(world, 0.0, 10.0f);
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the tape opens");
    m3Body_EnableFastRotation(world, body, true);
    int32_t bytes = StopJournal(world);
    CHECK(bytes == 16 + 20, "one flag op is the prefix, 8 header and 12 payload");
    m3DestroyWorld(world);

    m3World* twin = SpinWorld();
    m3BodyId twinBody = Spinner(twin, 0.0, 10.0f);
    uint8_t hostile[64];
    memcpy(hostile, tape, (size_t)bytes);
    hostile[16 + 16] = 7; // allow must be 0 or 1
    CHECK(!ReplayJournal(twin, hostile, bytes), "allow=7 bounces");
    CHECK(!ReplayJournal(twin, tape, bytes - 1), "a truncated tape bounces");
    CHECK(!m3Body_IsFastRotationEnabled(twin, twinBody), "the bounced tapes changed nothing");
    CHECK(ReplayJournal(twin, tape, bytes), "the clean tape lands");
    CHECK(m3Body_IsFastRotationEnabled(twin, twinBody),
          "the replayed flag landed on the twin body");
    m3DestroyWorld(twin);

    m3World* world2 = SpinWorld();
    CHECK(StartJournal(world2, tape, (int32_t)sizeof(tape)), "the cap tape opens");
    m3World_SetMaximumAngularSpeed(world2, 50.0f);
    bytes = StopJournal(world2);
    CHECK(bytes == 16 + 12, "one cap op is the prefix, 8 header and 4 payload");
    m3DestroyWorld(world2);

    m3World* twin2 = SpinWorld();
    memcpy(hostile, tape, (size_t)bytes);
    uint32_t nanBits = 0x7FC00000u;
    memcpy(hostile + 16 + 8, &nanBits, 4);
    CHECK(!ReplayJournal(twin2, hostile, bytes), "a NaN cap bounces");
    float negative = -5.0f;
    memcpy(hostile + 16 + 8, &negative, 4);
    CHECK(!ReplayJournal(twin2, hostile, bytes), "a negative cap bounces");
    CHECK(ReplayJournal(twin2, tape, bytes), "the clean cap tape lands");
    m3DestroyWorld(twin2);
}

int main(void)
{
    TestCapAndEscape();
    TestSpeedLimitHoldsFlaggedSpin();
    TestRuntimeCapAndLockInterplay();
    TestHashFoldsOffDefaultOnly();
    TestHostileTape();
    if (s_failures == 0)
    {
        printf("test_angularcap: all passed\n");
        return 0;
    }
    return 1;
}
