// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The holds that keep what the step lands within the speed limit, each
// driven where it takes effect. The pushes the step lands on a dynamic
// body itself, at the mass limits: a walker's, an anchored particle's, a
// soft particle's out of a body and a wheel's on loose ground. Each is
// built so that the push overflows the body's float velocity or spin, and
// m3PushHeld takes it again in double. The body leaves held at the speed
// limit, finite, 1e5 m/s and 1e5 rad/s in length, and in the direction
// the same push takes at a lighter mass, where nothing overflows; each
// retake counts in m3Counters.heldPushRetakes, which stays at zero for
// the lighter push and for an ordinary scene that walks, drapes and
// drives. Then a
// blast's push and its soft kick, a bounce and a suspension, whose holds
// keep the direction what remains of the motion takes, or the world's
// snapshot restorable. Every value is within the magnitude limits; the
// wheel needs every one of its own at once.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/character.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "maul3d/vehicle.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static double Length(m3Vec3 v)
{
    return sqrt((double)v.x * (double)v.x + (double)v.y * (double)v.y + (double)v.z * (double)v.z);
}

static bool Finite(m3Vec3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// Finite and exactly at the speed limit in length, to float rounding.
static bool AtLimit(m3Vec3 v)
{
    return Finite(v) && fabs(Length(v) - 1.0e5) < 1.0;
}

// A dynamic box of half extent half weighing 1.01e-6 kg, just over the
// mass floor; without gravity unless asked.
static m3BodyId LightBox(m3World* world, m3Pos3 at, m3Quat rotation, float half)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    bd.rotation = rotation;
    bd.gravityScale = 0.0f;
    m3BodyId box = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 1.01e-6f / (8.0f * half * half * half);
    NewBoxShape(world, box, &sd, (m3Vec3){half, half, half});
    return box;
}

static const m3Quat s_identity = {0.0f, 0.0f, 0.0f, 1.0f};

// What a push left on the pushed body, and how many pushes the world
// took again in double.
typedef struct Held
{
    m3Vec3 v;
    m3Vec3 w;
    uint64_t retakes;
} Held;

static Held HeldOf(const m3World* world, m3BodyId body)
{
    Held held = {m3Body_GetLinearVelocity(world, body), m3Body_GetAngularVelocity(world, body),
                 m3World_GetCounters(world).heldPushRetakes};
    return held;
}

// The relative gap between two held vectors.
static double Gap(m3Vec3 a, m3Vec3 b)
{
    return Length(m3Sub3(a, b)) / Length(b);
}

// A push held at the limit keeps the direction it had: the same as a
// lighter one that overflows nothing and is only held. The velocity
// agrees to 1e-4; the spin to 1e-3, since a moment that is rounding on
// one axis reaches tens of rad/s once held.
static void CheckHeld(const char* what, Held heavy, Held light, bool spinHeld)
{
    char line[160];
    snprintf(line, sizeof(line), "%s: the velocity is held at the limit", what);
    CHECK(AtLimit(heavy.v), line);
    snprintf(line, sizeof(line), "%s: in the direction a lighter push takes", what);
    CHECK(AtLimit(light.v) && Gap(heavy.v, light.v) < 1.0e-4, line);
    snprintf(line, sizeof(line), "%s: the spin is finite", what);
    CHECK(Finite(heavy.w), line);
    snprintf(line, sizeof(line), "%s: the heavy push is taken again and counted", what);
    CHECK(heavy.retakes > 0, line);
    snprintf(line, sizeof(line), "%s: the lighter push is not", what);
    CHECK(light.retakes == 0, line);
    if (spinHeld)
    {
        snprintf(line, sizeof(line), "%s: the spin is held in the lighter push's direction", what);
        CHECK(AtLimit(heavy.w) && AtLimit(light.w) && Gap(heavy.w, light.w) < 1.0e-3, line);
    }
}

// A walker of mass mass walks along x past a ball of radius 1 at the
// center of a 7 microgram body, 0.75 m to one side, so that it meets the
// ball once, glancing, and slides away: one push, along a normal with two
// axes that passes through the ball's center, so with no moment but for
// rounding. Six more 1 microgram balls, which collide with nothing, sit
// 1e4 m out along each axis and give the body a moment of 400 kg m^2
// about every axis. At 1e20 kg the push, mass * blocked or about 1e38
// N s, is a float, but the body's velocity is not on either axis, while
// its spin, over that moment, stays finite. That spin is rounding, held,
// so only its finiteness is checked.
static Held WalkerPush(float mass)
{
    const float density = 1.01e-6f / 4.18879f;
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    bd.gravityScale = 0.0f;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = density;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(world, body, &sd, &ball);
    sd.maskBits = 0;
    const m3Vec3 out[6] = {{1.0e4f, 0.0f, 0.0f},  {-1.0e4f, 0.0f, 0.0f}, {0.0f, 1.0e4f, 0.0f},
                           {0.0f, -1.0e4f, 0.0f}, {0.0f, 0.0f, 1.0e4f},  {0.0f, 0.0f, -1.0e4f}};
    for (int32_t k = 0; k < 6; ++k)
    {
        m3Sphere far = {out[k], 1.0f};
        NewSphereShape(world, body, &sd, &far);
    }
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){-3.0, 1.0, 0.75};
    cd.radius = 0.5f;
    cd.halfHeight = 0.5f;
    cd.mass = mass;
    cd.pushMaxMassRatio = 1.0e4f;
    m3CharacterId walker = NewCharacter(world, &cd);
    m3Character_Move(world, walker, (m3Vec3){1.0e18f, 0.0f, 0.0f});
    Held held = HeldOf(world, body);
    m3DestroyWorld(world);
    return held;
}

// A particle of mass mass anchored to the box off its center, the box
// teleported 1e14 m on a slant: at 1e20 kg the anchor's pull, mass *
// distance / h, overflows the box's velocity and spin on every axis they
// have.
static Held AnchorPush(float mass)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.gravityScale = 0.0f;
    m3BodyId box = NewBody(world, &bd);
    m3ShapeDef bs = m3DefaultShapeDef();
    bs.density = 1.01e-6f / 8.0f;
    bs.maskBits = 0; // the particle never touches it: only the anchor pulls
    NewBoxShape(world, box, &bs, (m3Vec3){1.0f, 1.0f, 1.0f});
    m3SoftBodyDef sd = m3DefaultSoftBodyDef();
    sd.position = (m3Pos3){0.5, 0.5, 0.5};
    sd.countX = 1;
    sd.countY = 1;
    sd.countZ = 1;
    sd.particleMass = mass;
    sd.radius = 0.1f;
    sd.gravityScale = 0.0f;
    m3SoftBodyId soft = NewSoftBody(world, &sd);
    m3SoftBody_AnchorParticle(world, soft, 0, box);
    m3Body_SetTransform(world, box, (m3Pos3){1.0e14, 3.0e13, 0.0}, s_identity);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    Held held = HeldOf(world, box);
    m3DestroyWorld(world);
    return held;
}

// A particle of mass mass and radius 1e4 starts inside a box at the
// inertia floor, and one step of 1e-4 s in 256 substeps pushes it out: at
// 1e20 kg, depth * mass / h is a float, but the box's spin is not.
static Held ParticleContactPush(float mass)
{
    const float half = 1.3e-3f;
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyId box = LightBox(world, (m3Pos3){0.0, 0.0, 0.0}, s_identity, half);
    m3SoftBodyDef sd = m3DefaultSoftBodyDef();
    sd.position = (m3Pos3){0.5 * (double)half, 0.3 * (double)half, 0.1 * (double)half};
    sd.countX = 1;
    sd.countY = 1;
    sd.countZ = 1;
    sd.particleMass = mass;
    sd.radius = 1.0e4f;
    sd.gravityScale = 0.0f;
    (void)NewSoftBody(world, &sd);
    (void)m3World_Step(world, 1.0e-4f, 256);
    Held held = HeldOf(world, box);
    m3DestroyWorld(world);
    return held;
}

static void TestPushesKeepDirection(void)
{
    Held walker = WalkerPush(1.0e20f);
    CheckHeld("a walker's push", walker, WalkerPush(1.0e6f), false);
    CHECK(walker.retakes == 1, "the walker's one push is counted once");
    CheckHeld("an anchor's pull", AnchorPush(1.0e20f), AnchorPush(1.0e10f), true);
    CheckHeld("a particle's push", ParticleContactPush(1.0e20f), ParticleContactPush(1.0e10f),
              true);
}

// The strongest blast, 1e10 N s per m^2, beside a 1 microgram box, off
// its center so that it spins the box too: the push, 4e10 N s, would
// leave it at 4e16 m/s. It leaves the box held at the limit at once.
static void TestBlastPush(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyId box = LightBox(world, (m3Pos3){3.0, 0.5, 0.0}, s_identity, 1.0f);
    m3ExplosionDef ed = m3DefaultExplosionDef();
    ed.position = (m3Pos3){0.0, 0.0, 0.0};
    ed.impulsePerArea = 1.0e10f;
    m3World_Explode(world, &ed);
    Held held = HeldOf(world, box);
    CHECK(AtLimit(held.v) && held.v.x > 0.0f, "a blast leaves the box held at the limit");
    CHECK(AtLimit(held.w), "and its spin held at the limit");
    m3DestroyWorld(world);
}

// The same blast beside a 1 microgram soft particle: its kick, which the
// next step lands, is held at the limit too, so the world's snapshot
// restores, as every snapshot the engine writes must.
static void TestBlastSoftKick(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3SoftBodyDef sd = m3DefaultSoftBodyDef();
    sd.position = (m3Pos3){3.0, 0.5, 0.0};
    sd.countX = 1;
    sd.countY = 1;
    sd.countZ = 1;
    sd.particleMass = 1.0e-6f;
    sd.radius = 1.0f;
    (void)NewSoftBody(world, &sd);
    m3ExplosionDef ed = m3DefaultExplosionDef();
    ed.impulsePerArea = 1.0e10f;
    m3World_Explode(world, &ed);
    int32_t size = m3World_GetSnapshotSize(world);
    void* snapshot = malloc((size_t)size);
    int32_t bytes = 0;
    CHECK(snapshot != NULL && m3World_Snapshot(world, snapshot, size, &bytes) == m3_success,
          "the world after a blast snapshots");
    m3World* again = NewWorld(&wd);
    CHECK(m3World_Restore(again, snapshot, bytes) == m3_success &&
              m3World_Hash(again) == m3World_Hash(world),
          "and the snapshot restores");
    free(snapshot);
    m3DestroyWorld(again);
    m3DestroyWorld(world);
}

// A frictionless ball of restitution 1e4 meets the ground at 1e4 m/s down
// and 1e3 m/s across. The bounce asks for at most 1e5 m/s of separation,
// not 1e8, so once the step holds the ball at the limit most of its 1e3
// m/s across survives.
static void TestBounceHold(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.maximumLinearSpeed = 1.0e5f;
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef gs = m3DefaultShapeDef();
    gs.friction = 0.0f;
    NewBoxShape(world, ground, &gs, (m3Vec3){1.0e4f, 1.0f, 1.0e4f});
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.01, 0.0};
    bd.gravityScale = 0.0f;
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.0f;
    sd.restitution = 1.0e4f;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(world, ball, &sd, &sphere);
    m3Body_SetLinearVelocity(world, ball, (m3Vec3){1.0e3f, -1.0e4f, 0.0f});
    (void)m3World_Step(world, 1.0e-4f, 1);
    m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
    CHECK(AtLimit(v) && v.y > 0.0f, "the bounce leaves the ball held at the limit");
    CHECK(v.x > 900.0f, "and keeps its speed across");
    m3DestroyWorld(world);
}

// A one-wheeled chassis lands at 9.9e4 m/s with 1.4e4 m/s of slip across
// on a grip of 0.01, its suspension at 1e5 Hz. The suspension reverses
// at most its share of the chassis from the speed limit, and that bounds
// the tire's friction budget too, so most of the slip survives.
static void TestSuspensionHold(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.maximumLinearSpeed = 1.0e5f;
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef gs = m3DefaultShapeDef();
    NewBoxShape(world, ground, &gs, (m3Vec3){1.0e4f, 1.0f, 1.0e4f});
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.5, 0.0};
    bd.gravityScale = 0.0f;
    m3BodyId chassis = NewBody(world, &bd);
    m3ShapeDef cs = m3DefaultShapeDef();
    cs.maskBits = 0;
    NewBoxShape(world, chassis, &cs, (m3Vec3){1.0f, 0.5f, 1.0f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 1;
    vd.wheels[0].anchor = (m3Vec3){0.0f, -0.5f, 0.0f};
    vd.wheels[0].direction = (m3Vec3){0.0f, -1.0f, 0.0f};
    vd.wheels[0].restLength = 1.0f;
    vd.wheels[0].travel = 1.0f;
    vd.wheels[0].radius = 0.5f;
    vd.wheels[0].hertz = 1.0e5f;
    vd.tireGrip = 0.01f;
    m3VehicleId vehicle = NewVehicle(world, &vd);
    m3Body_SetLinearVelocity(world, chassis, (m3Vec3){0.0f, -9.9e4f, 1.4e4f});
    (void)m3World_Step(world, 1.0e-4f, 1);
    CHECK(m3Vehicle_IsWheelGrounded(world, vehicle, 0), "the wheel lands");
    m3Vec3 v = m3Body_GetLinearVelocity(world, chassis);
    CHECK(AtLimit(v) && v.y > 0.0f, "the suspension leaves the chassis held at the limit");
    CHECK(v.z > 1.0e4f, "and most of the slip survives");
    m3DestroyWorld(world);
}

// A wheel on a 1 microgram chip at the inertia floor, turned 45 degrees
// so the wheel lands on a corner, with every wheel bound at its ceiling:
// a 1e20 kg chassis on one wheel, engine torque 1e20 N m through gear and
// final ratios of 1e4 on a 1e-3 m wheel, tire grip 1e4. The friction
// circle holds the tire to 2e29 N s, whose moment at the corner overflows
// the chip's spin by a percent. The step is taken from a snapshot of the
// world before it, and restoring that snapshot leaves the count as it is.
static void TestWheelPush(void)
{
    const float chipHalf = 1.2248e-3f; // every principal moment at 1e-12 kg m^2
    const float boxHalf = 1.0e4f;
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef gs = m3DefaultShapeDef();
    NewBoxShape(world, ground, &gs, (m3Vec3){100.0f, 1.0f, 100.0f});
    m3BodyId chip = LightBox(world, (m3Pos3){0.0, 1.0 + (double)chipHalf, 0.0},
                             (m3Quat){0.0f, 0.38268343f, 0.0f, 0.92387953f}, chipHalf);
    bd.type = m3_dynamicBody;
    bd.position =
        (m3Pos3){0.0, 1.0 + 2.0 * (double)chipHalf + 1.0e-3, 0.999 * 1.41421356 * (double)chipHalf};
    m3BodyId chassis = NewBody(world, &bd);
    m3ShapeDef ss = m3DefaultShapeDef();
    ss.density = 1.0e6f;
    ss.maskBits = 0;
    for (int32_t k = 0; k < 12; ++k)
    {
        NewBoxShape(world, chassis, &ss, (m3Vec3){boxHalf, boxHalf, boxHalf}); // 8e18 kg each
    }
    NewBoxShape(world, chassis, &ss, (m3Vec3){7937.0f, 7937.0f, 7937.0f}); // to 1e20 kg
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 1;
    vd.wheels[0].anchor = (m3Vec3){0.0f, 0.0f, 0.0f};
    vd.wheels[0].direction = (m3Vec3){0.0f, -1.0f, 0.0f};
    vd.wheels[0].restLength = 1.0e-3f;
    vd.wheels[0].travel = 1.0e-3f;
    vd.wheels[0].radius = 1.0e-3f;
    vd.wheels[0].driven = true;
    vd.wheels[0].hertz = 1.0e5f;
    vd.tireGrip = 1.0e4f;
    m3VehicleId vehicle = NewVehicle(world, &vd);
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    for (int32_t c = 0; c < dd.curveCount; ++c)
    {
        dd.curveTorque[c] = 1.0e20f;
    }
    dd.gearCount = 1;
    dd.gearRatio[0] = 1.0e4f;
    dd.finalDrive = 1.0e4f;
    dd.autoShift = false;
    m3Vehicle_SetDrivetrain(world, vehicle, &dd);
    m3Vehicle_SetGear(world, vehicle, 1);
    m3Vehicle_SetCommands(world, vehicle, 1.0f, 0.0f, 0.0f);
    CHECK(m3Body_GetMass(world, chassis) > 0.999e20f, "the chassis weighs the mass limit");
    int32_t size = m3World_GetSnapshotSize(world);
    void* before = malloc((size_t)size);
    int32_t written = 0;
    CHECK(before != NULL && m3World_Snapshot(world, before, size, &written) == m3_success,
          "the world before the step snapshots");
    (void)m3World_Step(world, 10.0f, 1);
    CHECK(m3Vehicle_IsWheelGrounded(world, vehicle, 0), "the wheel stands on the chip");
    m3Vec3 v = m3Body_GetLinearVelocity(world, chip);
    m3Vec3 w = m3Body_GetAngularVelocity(world, chip);
    CHECK(Finite(v) && Length(v) <= 1.0e5 * 1.0001,
          "the wheel's push leaves the chip finite within the limit");
    CHECK(Finite(w) && Length(w) <= 1.0e5 * 1.0001, "and its spin as well");
    uint64_t retakes = m3World_GetCounters(world).heldPushRetakes;
    CHECK(retakes > 0, "the wheel's push is taken again and counted");
    CHECK(m3World_Restore(world, before, size) == m3_success, "the world restores");
    CHECK(m3World_GetCounters(world).heldPushRetakes == retakes, "and the count is left as it is");
    free(before);
    m3DestroyWorld(world);
}

// A walker shoulders a crate, a jelly rides on it anchored by one corner
// and a car drives off on four wheels, at everyday masses for two seconds: every
// route a held push takes, and none of them near the limits, so the
// count stays at zero.
static void TestOrdinarySceneRetakesNothing(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef gs = m3DefaultShapeDef();
    NewBoxShape(world, ground, &gs, (m3Vec3){100.0f, 1.0f, 100.0f});
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.5, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    m3ShapeDef cs = m3DefaultShapeDef();
    cs.density = 10.0f; // 10 kg, light enough for an 80 kg walker to slide
    NewBoxShape(world, crate, &cs, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3SoftBodyDef sd = m3DefaultSoftBodyDef();
    sd.position = (m3Pos3){-0.2, 2.1, -0.2};
    sd.countX = 3;
    sd.countY = 2;
    sd.countZ = 3;
    sd.spacing = 0.2f;
    sd.radius = 0.08f;
    m3SoftBodyId jelly = NewSoftBody(world, &sd);
    m3SoftBody_AnchorParticle(world, jelly, 0, crate);
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){-2.0, 2.0, 0.0};
    m3CharacterId walker = NewCharacter(world, &cd);
    bd.position = (m3Pos3){0.0, 2.0, 10.0};
    m3BodyId chassis = NewBody(world, &bd);
    m3ShapeDef hs = m3DefaultShapeDef();
    NewBoxShape(world, chassis, &hs, (m3Vec3){1.0f, 0.3f, 2.0f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 4;
    for (int32_t k = 0; k < 4; ++k)
    {
        vd.wheels[k].anchor = (m3Vec3){(k & 1) != 0 ? 0.9f : -0.9f, -0.3f, k < 2 ? 1.5f : -1.5f};
        vd.wheels[k].direction = (m3Vec3){0.0f, -1.0f, 0.0f};
        vd.wheels[k].restLength = 0.4f;
        vd.wheels[k].travel = 0.3f;
        vd.wheels[k].radius = 0.35f;
        vd.wheels[k].driven = k >= 2;
    }
    m3VehicleId car = NewVehicle(world, &vd);
    m3Vehicle_SetCommands(world, car, 1.0f, 0.0f, 0.0f);
    m3Pos3 crateStart = m3Body_GetPosition(world, crate);
    bool grounded = false;
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(world, walker, (m3Vec3){2.0f / 60.0f, -0.1f, 0.0f});
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        grounded = grounded || m3Vehicle_IsWheelGrounded(world, car, 0);
    }
    CHECK(m3Body_GetPosition(world, crate).x > crateStart.x + 0.1, "the walker moves the crate");
    CHECK(grounded, "the car's wheels reach the ground");
    CHECK(m3World_GetCounters(world).heldPushRetakes == 0, "no push is taken again");
    m3DestroyWorld(world);
}

int main(void)
{
    TestPushesKeepDirection();
    TestBlastPush();
    TestBlastSoftKick();
    TestBounceHold();
    TestSuspensionHold();
    TestWheelPush();
    TestOrdinarySceneRetakesNothing();
    if (s_failures == 0)
    {
        printf("test_heldpush: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
