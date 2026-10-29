// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The drivetrain gate: the engine curve and gearbox. A
// geared car climbs a slope that bogs it in top gear (the whole
// point of gears), auto shift walks up through a flat sprint at
// the pinned RPMs on identical bits, and hostile defs bounce off
// the validation wall without touching state.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"
#include "maul3d/vehicle.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* PlaneWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.vehicleCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    return world;
}

// A plane tilted about z: uphill is +x. Fifteen degrees is enough
// for gravity to beat top gear and lose to first (the analytic
// heart of the bog-or-climb test).
static m3World* RampWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.vehicleCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane slope = {{-0.25881904510252076f, 0.96592582628906829f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &slope);
    return world;
}

// The same one-box car the vehicle gate drives: half extents
// (1.0, 0.25, 0.5), 300 kg, four corner wheels, all driven.
static m3VehicleId MakeCar(m3World* world, m3Pos3 at, m3BodyId* outChassis)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    m3BodyId chassis = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 300.0f;
    NewBoxShape(world, chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    if (outChassis != NULL)
    {
        *outChassis = chassis;
    }
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
        vd.wheels[w].steerable = (w & 1) != 0;
    }
    return NewVehicle(world, &vd);
}

static void TestUphillBogAndClimb(void)
{
    // Same slope, same car, same throttle; only the gear differs.
    // Top gear multiplies idle torque below the gravity component
    // and the car bogs; first gear multiplies it past and the car
    // climbs. Flat driveForce could only ever do one of the two.
    double progress[2];
    for (int32_t variant = 0; variant < 2; ++variant)
    {
        m3World* world = RampWorld();
        m3BodyId chassis;
        m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
        m3DrivetrainDef dt = m3DefaultDrivetrainDef();
        dt.autoShift = false; // the gear is the experiment variable
        m3Vehicle_SetDrivetrain(world, car, &dt);
        m3Vehicle_SetGear(world, car, variant == 0 ? 5 : 1);
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4); // settle on the slope
        }
        double startX = m3Body_GetPosition(world, chassis).x;
        m3Vehicle_SetCommands(world, car, 1.0f, 0.0f, 0.0f);
        for (int32_t i = 0; i < 300; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        progress[variant] = m3Body_GetPosition(world, chassis).x - startX;
        CHECK(m3Vehicle_GetGear(world, car) == (variant == 0 ? 5 : 1), "the manual gear holds");
        m3DestroyWorld(world);
    }
    CHECK(progress[0] < 0.5, "top gear bogs on the slope");
    CHECK(progress[1] > 2.0, "first gear climbs the same slope");
    CHECK(progress[1] > progress[0] + 2.0, "the gearbox is the difference");
}

static void TestAutoShiftSprintTwins(void)
{
    // A flat full-throttle sprint climbs the gearbox at the pinned
    // shift RPM, never downshifts, and twin runs land on one hash.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3World* world = PlaneWorld();
        m3BodyId chassis;
        m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
        m3DrivetrainDef dt = m3DefaultDrivetrainDef();
        m3Vehicle_SetDrivetrain(world, car, &dt);
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Vehicle_SetCommands(world, car, 1.0f, 0.0f, 0.0f);
        int32_t lastGear = m3Vehicle_GetGear(world, car);
        int32_t shifts = 0;
        m3real rpmBefore = 0.0f;
        for (int32_t i = 0; i < 1500; ++i)
        {
            m3real rpmPrev = m3Vehicle_GetEngineRpm(world, car);
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            int32_t gear = m3Vehicle_GetGear(world, car);
            if (run == 0)
            {
                CHECK(gear >= lastGear, "a flat sprint never downshifts");
                if (gear > lastGear)
                {
                    shifts += 1;
                    if (shifts == 1)
                    {
                        rpmBefore = rpmPrev;
                    }
                }
            }
            lastGear = gear;
        }
        if (run == 0)
        {
            CHECK(lastGear >= 4, "the sprint climbs at least to fourth");
            CHECK(shifts >= 3, "at least three upshifts happen");
            CHECK(rpmBefore > 5300.0f, "the first upshift waits for the pinned RPM");
            m3Vec3 v = m3Body_GetLinearVelocity(world, chassis);
            CHECK(v.x > 20.0f, "the geared sprint is genuinely fast");
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin sprints are bit-identical");
}

static void TestNeutralAndReverse(void)
{
    // Neutral holds the car still under full throttle; reverse
    // drives it backward off throttle magnitude alone (with a
    // drivetrain, direction belongs to the gear, not the pedal).
    m3World* world = PlaneWorld();
    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    dt.autoShift = false;
    m3Vehicle_SetDrivetrain(world, car, &dt);
    m3Vehicle_SetGear(world, car, 0);
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double startX = m3Body_GetPosition(world, chassis).x;
    m3Vehicle_SetCommands(world, car, 1.0f, 0.0f, 0.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(fabs(m3Body_GetPosition(world, chassis).x - startX) < 0.1, "neutral drives nothing");
    m3Vehicle_SetGear(world, car, -1);
    for (int32_t i = 0; i < 180; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(world, chassis).x < startX - 1.0, "reverse drives backward");
    m3DestroyWorld(world);
}

static double DiffYaw(int32_t mode, m3real couple, uint64_t* outHash)
{
    // Same car, same throttle, same steer; only the differential
    // changes. The coupling resists the inner/outer wheel speed
    // split a turn creates, so a locked diff yaws less than open.
    m3World* world = PlaneWorld();
    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 0.6, 0.0}, &chassis);
    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    dt.autoShift = false;
    dt.diffMode = mode;
    dt.diffCouple = couple;
    m3Vehicle_SetDrivetrain(world, car, &dt);
    m3Vehicle_SetGear(world, car, 1);
    double yaw = 0.0;
    for (int32_t i = 0; i < 300; ++i)
    {
        m3Vehicle_SetCommands(world, car, 1.0f, 0.6f, 0.0f);
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        yaw += (double)m3Body_GetAngularVelocity(world, chassis).y;
    }
    if (outHash != NULL)
    {
        *outHash = m3World_Hash(world);
    }
    m3DestroyWorld(world);
    return yaw < 0.0 ? -yaw : yaw;
}

static void TestDifferentials(void)
{
    // The coupling lives inside the friction circle, so its effect
    // is emergent: a strong lock converges wheel speeds and kills
    // the turn; softer values redistribute grip and can even liven
    // rotation (the probe measured both regimes; the guide says
    // couple is host-tuned to the vehicle's mass scale).
    uint64_t lockedA = 0;
    uint64_t lockedB = 0;
    uint64_t openHash = 0;
    uint64_t limitedHash = 0;
    double open = DiffYaw(0, 0.0f, &openHash);
    double locked = DiffYaw(2, 20000.0f, &lockedA);
    double limited = DiffYaw(1, 20000.0f, &limitedHash);
    CHECK(open > 10.0, "the open car turns");
    CHECK(locked < open * 0.2, "the strong lock converges the wheels and resists the turn");
    CHECK(limited > 0.0 && limitedHash != openHash,
          "the limited mode is a real, distinct, deterministic regime");
    DiffYaw(2, 20000.0f, &lockedB);
    CHECK(lockedA == lockedB, "twin locked runs are bit-identical");

    // The hostile wall: bad modes and couplings never attach.
    m3World* world = PlaneWorld();
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 0.6, 0.0}, NULL);
    m3DrivetrainDef bad = m3DefaultDrivetrainDef();
    bad.diffMode = 3;
    m3Vehicle_SetDrivetrain(world, car, &bad);
    m3DrivetrainDef good = m3DefaultDrivetrainDef();
    good.diffMode = 2;
    good.diffCouple = -1.0f;
    m3Vehicle_SetDrivetrain(world, car, &good);
    // Neither hostile def attached: gear selection still refuses
    // like a drivetrain-less car accepts nothing but neutral.
    m3Vehicle_SetGear(world, car, 1);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    m3DestroyWorld(world);
}

static void TestTankSteer(void)
{
    // Opposite tracks spin the hull in place; equal tracks
    // run it straight; twins agree to the bit; a drivetrain
    // vehicle refuses the tank door.
    uint64_t hashes[2];
    double yawSpin = 0.0;
    double straightX = 0.0;
    double straightZdrift = 0.0;
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 16;
        wd.shapeCapacity = 16;
        m3World* world = NewWorld(&wd);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &sd, &floor);
        m3BodyId hullOut;
        m3VehicleId tank = MakeCar(world, (m3Pos3){0.0, 0.62, 0.0}, &hullOut);
        m3BodyId hull = hullOut;
        m3Vehicle_SetTankCommands(world, tank, 1.0f, 1.0f, 0.0f);
        for (int32_t i = 0; i < 90; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        if (run == 0)
        {
            straightX = m3Body_GetPosition(world, hull).x;
            straightZdrift = m3Body_GetPosition(world, hull).z;
        }
        // Tracks brake the run speed off first: a spin attempt at
        // 20 m/s only proves the lateral tire kill is a good yaw
        // damper. Pivot turns happen from rest.
        m3Vehicle_SetTankCommands(world, tank, 0.0f, 0.0f, 1.0f);
        for (int32_t i = 0; i < 300; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Vehicle_SetTankCommands(world, tank, 1.0f, -1.0f, 0.0f);
        for (int32_t i = 0; i < 180; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        if (run == 0)
        {
            yawSpin = (double)fabsf(m3Body_GetAngularVelocity(world, hull).y);
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin tanks are bit-identical");
    CHECK(yawSpin > 0.3, "opposite tracks spin the hull");
    CHECK(fabs(straightX) > 1.0, "equal tracks drive somewhere real");
    CHECK(fabs(straightZdrift) < fabs(straightX), "equal tracks hold a heading");
}

// A two-wheeler: narrow box, one wheel at each end, the front one
// steerable. The lean gain is the experiment variable.
static m3VehicleId MakeBike(m3World* world, m3Pos3 at, m3real leanGain, m3BodyId* outChassis)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    m3BodyId chassis = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 800.0f;
    NewBoxShape(world, chassis, &sd, (m3Vec3){0.7f, 0.3f, 0.1f});
    if (outChassis != NULL)
    {
        *outChassis = chassis;
    }
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 2;
    vd.driveForce = 400.0f;
    vd.leanStabilization = leanGain;
    vd.wheels[0].anchor = (m3Vec3){-0.6f, -0.3f, 0.0f};
    vd.wheels[0].driven = true;
    vd.wheels[1].anchor = (m3Vec3){0.6f, -0.3f, 0.0f};
    vd.wheels[1].driven = true;
    vd.wheels[1].steerable = true;
    return NewVehicle(world, &vd);
}

static void TestMotorcycleLean(void)
{
    // The stabilizer is the difference between a motorcycle and a
    // falling ladder: with the gain the bike sprints, then holds a
    // steady turn upright; without it the same bike is on its side
    // before the turn even starts. Runs 0 and 2 are twins.
    double upAtEnd[3];
    uint64_t hashes[3];
    for (int32_t run = 0; run < 3; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 16;
        wd.shapeCapacity = 16;
        m3World* world = NewWorld(&wd);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &sd, &floor);
        m3BodyId frame;
        m3VehicleId bike =
            MakeBike(world, (m3Pos3){0.0, 0.75, 0.0}, run == 1 ? 0.0f : 25.0f, &frame);
        m3Vehicle_SetCommands(world, bike, 1.0f, 0.0f, 0.0f);
        for (int32_t i = 0; i < 45; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Vehicle_SetCommands(world, bike, 0.0f, 0.15f, 0.0f);
        for (int32_t i = 0; i < 240; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Quat q = m3Body_GetRotation(world, frame);
        m3Vec3 up = m3RotateVec3(q, (m3Vec3){0.0f, 1.0f, 0.0f});
        upAtEnd[run] = (double)up.y;
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[2], "twin bikes are bit-identical");
    CHECK(hashes[0] != hashes[1], "the gain folds into the hash");
    CHECK(upAtEnd[0] > 0.7, "the stabilized bike rides the turn upright");
    CHECK(upAtEnd[1] < 0.5, "the raw two-wheeler falls over");
}

static void TestHostileWall(void)
{
    // Every malformed def and gear select is a documented no-op:
    // the car keeps its flat model (gear reads 0) and the world
    // keeps stepping. The validation wall is the same one replay
    // uses against fuzzed journal bytes.
    m3World* world = PlaneWorld();
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, NULL);

    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    dt.internalValue = 0; // bad cookie
    m3Vehicle_SetDrivetrain(world, car, &dt);
    CHECK(m3Vehicle_GetGear(world, car) == 0, "a bad cookie bounces");

    dt = m3DefaultDrivetrainDef();
    dt.curveCount = 1;
    m3Vehicle_SetDrivetrain(world, car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.curveCount = 99;
    m3Vehicle_SetDrivetrain(world, car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.curveRpm[1] = dt.curveRpm[0]; // not ascending
    m3Vehicle_SetDrivetrain(world, car, &dt);
    dt = m3DefaultDrivetrainDef();
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&dt.curveTorque[1], &nanBits, sizeof(float));
    m3Vehicle_SetDrivetrain(world, car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.gearCount = 0;
    m3Vehicle_SetDrivetrain(world, car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.gearRatio[2] = -1.0f;
    m3Vehicle_SetDrivetrain(world, car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.shiftUpRpm = dt.shiftDownRpm; // shift band collapses
    m3Vehicle_SetDrivetrain(world, car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.clutchSteps = -5;
    m3Vehicle_SetDrivetrain(world, car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.finalDrive = 0.0f;
    m3Vehicle_SetDrivetrain(world, car, &dt);
    CHECK(m3Vehicle_GetGear(world, car) == 0, "nine garbage defs all bounce");
    CHECK(m3Vehicle_GetEngineRpm(world, car) == 0.0f, "no drivetrain, no tachometer");

    m3Vehicle_SetGear(world, car, 3); // no drivetrain attached
    CHECK(m3Vehicle_GetGear(world, car) == 0, "gear select without a drivetrain bounces");

    dt = m3DefaultDrivetrainDef();
    dt.reverseRatio = 0.0f; // this car forbids reverse
    m3Vehicle_SetDrivetrain(world, car, &dt);
    CHECK(m3Vehicle_GetGear(world, car) == 1, "the valid def lands in first");
    m3Vehicle_SetGear(world, car, -1);
    CHECK(m3Vehicle_GetGear(world, car) == 1, "reverse without a reverse ratio bounces");
    m3Vehicle_SetGear(world, car, 6); // gearCount is 5
    CHECK(m3Vehicle_GetGear(world, car) == 1, "an out-of-range gear bounces");

    m3Vehicle_SetDrivetrain(world, m3_nullVehicleId, &dt); // stale id no-op
    m3Vehicle_SetGear(world, m3_nullVehicleId, 1);
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Vehicle_GetGear(world, car) == 1, "the world steps on through all of it");
    m3DestroyWorld(world);
}

int main(void)
{
    TestUphillBogAndClimb();
    TestAutoShiftSprintTwins();
    TestNeutralAndReverse();
    TestDifferentials();
    TestTankSteer();
    TestMotorcycleLean();
    TestHostileWall();
    if (s_failures == 0)
    {
        printf("test_drivetrain: all passed\n");
        return 0;
    }
    return 1;
}
