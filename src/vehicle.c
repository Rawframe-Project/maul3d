// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The raycast vehicle: the suspension pass. Wheels cast rays
// from the chassis (the chassis ignores itself through the ray
// hook), springs push back through hertz and zeta scaled by a
// quarter of the chassis mass per wheel, and every impulse lands
// before the solver prepares contacts, so the solver sees a sprung
// chassis exactly the way it sees gravity. Serial, slot order,
// canonical: twin worlds drive on identical bits.

#include "maul3d/vehicle.h"

#include "body_state.h"
#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "raycast.h"
#include "solver_math.h"
#include "vehicle.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

#define M3_VEHICLE_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3VehicleDef) << 8) ^ 8))

// Freed and freshly created vehicles carry no drivetrain: the flat
// force model is the default, and the hash walk skips these fields
// until a drivetrain is set, so worlds without one keep their hashes.
static void ResetDrivetrain(m3World* world, int32_t slot)
{
    world->vehicles.vehDtActive[slot] = 0;
    world->vehicles.vehDtCurveCount[slot] = 0;
    world->vehicles.vehDtGearCount[slot] = 0;
    for (int32_t c = 0; c < M3_MAX_DRIVETRAIN_CURVE; ++c)
    {
        world->vehicles.vehDtCurveRpm[slot * M3_MAX_DRIVETRAIN_CURVE + c] = 0.0f;
        world->vehicles.vehDtCurveTorque[slot * M3_MAX_DRIVETRAIN_CURVE + c] = 0.0f;
    }
    for (int32_t g = 0; g < M3_MAX_DRIVETRAIN_GEARS; ++g)
    {
        world->vehicles.vehDtGearRatio[slot * M3_MAX_DRIVETRAIN_GEARS + g] = 0.0f;
    }
    world->vehicles.vehDtReverse[slot] = 0.0f;
    world->vehicles.vehDtFinal[slot] = 0.0f;
    world->vehicles.vehDtDiffMode[slot] = 0;
    world->vehicles.vehDtDiffCouple[slot] = 0.0f;
    world->vehicles.vehDtShiftUp[slot] = 0.0f;
    world->vehicles.vehDtShiftDown[slot] = 0.0f;
    world->vehicles.vehDtClutchSteps[slot] = 0;
    world->vehicles.vehDtAutoShift[slot] = 0;
    world->vehicles.vehDtGear[slot] = 0;
    world->vehicles.vehDtClutch[slot] = 0;
    world->vehicles.vehDtRpm[slot] = 0.0f;
}

m3VehicleDef m3DefaultVehicleDef(void)
{
    m3VehicleDef def;
    memset(&def, 0, sizeof(def));
    for (int32_t w = 0; w < M3_MAX_VEHICLE_WHEELS; ++w)
    {
        def.wheels[w].direction = (m3Vec3){0.0f, -1.0f, 0.0f};
        def.wheels[w].restLength = 0.4f;
        def.wheels[w].travel = 0.25f;
        def.wheels[w].springHertz = 1.5f;
        def.wheels[w].springDampingRatio = 0.6f;
        def.wheels[w].radius = 0.3f;
        def.wheels[w].brakeShare = 0.25f;
    }
    def.maxSteerAngle = 0.6f;
    def.driveForce = 800.0f;
    def.brakeForce = 1600.0f;
    def.tireGrip = 1.5f;
    def.leanStabilization = 0.0f;
    def.internalValue = M3_VEHICLE_COOKIE;
    return def;
}

int32_t m3VehicleSlot(const m3World* world, m3VehicleId vehicleId)
{
    int32_t index = vehicleId.index1 - 1;
    if (world == NULL || !m3IdPoolValid(&world->vehicles.vehPool, index, vehicleId.generation))
    {
        return -1;
    }
    return index;
}

// A wheel lies in its chassis frame: anchor, rest length and radius
// within M3_LOCAL_LIMIT, the radius at least M3_MIN_WHEEL_RADIUS.
m3Result m3WheelDefCheck(const m3WheelDef* wd)
{
    float d2 = m3Dot3(wd->direction, wd->direction);
    m3Result result = m3CheckValid(m3FiniteV3(wd->direction) && d2 > 0.81f && d2 < 1.21f &&
                                   wd->travel > 0.0f && wd->travel <= wd->restLength);
    result = m3BothChecks(result, m3CheckLocalV3(wd->anchor));
    result = m3BothChecks(result, m3CheckPositive(wd->restLength, M3_LOCAL_LIMIT));
    result = m3BothChecks(result, m3CheckPositive(wd->springHertz, M3_HERTZ_LIMIT));
    result = m3BothChecks(result, m3CheckNonNegative(wd->springDampingRatio, M3_RATIO_LIMIT));
    result = m3BothChecks(result, m3CheckPositive(wd->radius, M3_LOCAL_LIMIT));
    result = m3BothChecks(result, m3CheckAtLeast(wd->radius, M3_MIN_WHEEL_RADIUS));
    return m3BothChecks(result, m3CheckNonNegative(wd->brakeShare, M3_RATIO_LIMIT));
}

// Every field check lives here, where the public door and replay both
// pass: replay hands raw journal bytes, and a flipped wheel count would
// overrun every per-wheel array. The chassis must be a live dynamic body.
static m3Result VehicleDefCheck(const m3World* world, const m3VehicleDef* def)
{
    if (def->wheelCount < 1)
    {
        return m3_errorInvalid;
    }
    int32_t chassis = def->chassisId.index1 - 1;
    m3Result result = m3CheckValid(
        m3FiniteF(def->maxSteerAngle) && def->maxSteerAngle >= 0.0f && chassis >= 0 &&
        chassis < world->bodies.bodyCapacity && world->bodies.bodyPool.alive[chassis] != 0 &&
        world->bodies.bodyPool.generations[chassis] == def->chassisId.generation &&
        world->bodies.types[chassis] == (uint8_t)m3_dynamicBody);
    result = m3BothChecks(result, m3CheckCount(def->wheelCount, M3_MAX_VEHICLE_WHEELS));
    result = m3BothChecks(result, m3CheckNonNegative(def->driveForce, M3_FORCE_LIMIT));
    result = m3BothChecks(result, m3CheckNonNegative(def->brakeForce, M3_FORCE_LIMIT));
    result = m3BothChecks(result, m3CheckNonNegative(def->tireGrip, M3_RATIO_LIMIT));
    result = m3BothChecks(result, m3CheckNonNegative(def->leanStabilization, M3_RATIO_LIMIT));
    int32_t wheels =
        def->wheelCount < M3_MAX_VEHICLE_WHEELS ? def->wheelCount : M3_MAX_VEHICLE_WHEELS;
    for (int32_t w = 0; w < wheels; ++w)
    {
        result = m3BothChecks(result, m3WheelDefCheck(&def->wheels[w]));
    }
    return result;
}

// One wheel slot: the def's wheel with a unit direction, or a cleared
// slot past the wheel count.
static void InitWheel(m3World* world, int32_t k, const m3WheelDef* wd)
{
    m3Vehicles* v = &world->vehicles;
    m3WheelDef none;
    memset(&none, 0, sizeof(none));
    none.direction = (m3Vec3){0.0f, -1.0f, 0.0f};
    const m3WheelDef* src = wd != NULL ? wd : &none;
    m3Vec3 dir = src->direction;
    if (wd != NULL)
    {
        dir = m3MulSV3(1.0f / sqrtf(m3Dot3(wd->direction, wd->direction)), wd->direction);
    }
    v->vehWheelAnchor[k] = src->anchor;
    v->vehWheelDir[k] = dir;
    v->vehWheelRest[k] = src->restLength;
    v->vehWheelTravel[k] = src->travel;
    v->vehWheelHertz[k] = src->springHertz;
    v->vehWheelZeta[k] = src->springDampingRatio;
    v->vehWheelRadius[k] = src->radius;
    v->vehWheelFlags[k] = (uint8_t)((src->steerable ? 1u : 0u) | (src->driven ? 2u : 0u));
    v->vehWheelBrake[k] = src->brakeShare;
    v->vehWheelCompression[k] = 0.0f;
    v->vehWheelContact[k] = 0;
    v->vehWheelSpin[k] = 0.0f;
}

int32_t m3CreateVehicleInternal(m3World* world, const m3VehicleDef* def)
{
    m3Result checked = VehicleDefCheck(world, def);
    if (checked != m3_success)
    {
        return (int32_t)checked;
    }
    int32_t slot = m3IdPoolAlloc(&world->vehicles.vehPool);
    if (slot < 0)
    {
        return M3_CREATE_FULL;
    }
    m3Vehicles* v = &world->vehicles;
    int32_t chassis = def->chassisId.index1 - 1;
    v->vehChassis[slot] = chassis;
    v->vehChassisGen[slot] = world->bodies.bodyPool.generations[chassis];
    v->vehWheelCount[slot] = def->wheelCount;
    v->vehMaxSteer[slot] = def->maxSteerAngle;
    v->vehDriveForce[slot] = def->driveForce;
    v->vehBrakeForce[slot] = def->brakeForce;
    v->vehTireGrip[slot] = def->tireGrip;
    v->vehLeanGain[slot] = def->leanStabilization;
    v->vehThrottle[slot] = 0.0f;
    v->vehTrackMode[slot] = 0;
    v->vehTrackLeft[slot] = 0.0f;
    v->vehTrackRight[slot] = 0.0f;
    v->vehSteer[slot] = 0.0f;
    v->vehBrake[slot] = 0.0f;
    v->vehUserData[slot] = def->userData;
    for (int32_t w = 0; w < M3_MAX_VEHICLE_WHEELS; ++w)
    {
        InitWheel(world, slot * M3_MAX_VEHICLE_WHEELS + w,
                  w < def->wheelCount ? &def->wheels[w] : NULL);
    }
    ResetDrivetrain(world, slot);
    return slot;
}

void m3DestroyVehicleInternal(m3World* world, int32_t slot)
{
    world->vehicles.vehChassis[slot] = -1;
    world->vehicles.vehChassisGen[slot] = 0;
    world->vehicles.vehWheelCount[slot] = 0;
    world->vehicles.vehMaxSteer[slot] = 0.0f;
    world->vehicles.vehDriveForce[slot] = 0.0f;
    world->vehicles.vehBrakeForce[slot] = 0.0f;
    world->vehicles.vehTireGrip[slot] = 0.0f;
    world->vehicles.vehLeanGain[slot] = 0.0f;
    world->vehicles.vehThrottle[slot] = 0.0f;
    world->vehicles.vehTrackMode[slot] = 0;
    world->vehicles.vehTrackLeft[slot] = 0.0f;
    world->vehicles.vehTrackRight[slot] = 0.0f;
    world->vehicles.vehSteer[slot] = 0.0f;
    world->vehicles.vehBrake[slot] = 0.0f;
    world->vehicles.vehUserData[slot] = 0;
    for (int32_t w = 0; w < M3_MAX_VEHICLE_WHEELS; ++w)
    {
        int32_t k = slot * M3_MAX_VEHICLE_WHEELS + w;
        world->vehicles.vehWheelAnchor[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->vehicles.vehWheelDir[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->vehicles.vehWheelRest[k] = 0.0f;
        world->vehicles.vehWheelTravel[k] = 0.0f;
        world->vehicles.vehWheelHertz[k] = 0.0f;
        world->vehicles.vehWheelZeta[k] = 0.0f;
        world->vehicles.vehWheelRadius[k] = 0.0f;
        world->vehicles.vehWheelFlags[k] = 0;
        world->vehicles.vehWheelBrake[k] = 0.0f;
        world->vehicles.vehWheelCompression[k] = 0.0f;
        world->vehicles.vehWheelContact[k] = 0;
        world->vehicles.vehWheelSpin[k] = 0.0f;
    }
    ResetDrivetrain(world, slot);
    m3IdPoolFree(&world->vehicles.vehPool, slot);
}

// The friction circle: the tangent impulses (lon, lat) scaled onto the
// circle of radius budget when they leave it.
static void HoldInCircle(float* lon, float* lat, float budget)
{
    float mag2 = *lon * *lon + *lat * *lat;
    float budgetSq = budget * budget;
    if (!m3FiniteF(mag2) || !m3FiniteF(budgetSq))
    {
        // When the squares are not finite, the circle is taken at the
        // scale of its largest term: the same clamp in real arithmetic.
        float big = m3MaxF(m3MaxF(m3AbsF(*lon), m3AbsF(*lat)), budget);
        float l = *lon / big;
        float t = *lat / big;
        float b = budget / big;
        float m = l * l + t * t;
        if (m > b * b)
        {
            float scale = b / sqrtf(m);
            *lon *= scale;
            *lat *= scale;
        }
    }
    else if (mag2 > budgetSq && mag2 > 0.0f)
    {
        float scale = budget / sqrtf(mag2);
        *lon *= scale;
        *lat *= scale;
    }
}

// The suspension pass: velocity impulses on the chassis, before the
// solver prepares anything. A sleeping or vanished chassis skips
// (a parked car sleeps like any body and its wheel state freezes).
void m3VehicleApplySuspension(m3World* world, float dt)
{
    for (int32_t slot = 0; slot < world->vehicles.vehPool.maxIndex; ++slot)
    {
        if (world->vehicles.vehPool.alive[slot] == 0)
        {
            continue;
        }
        int32_t chassis = world->vehicles.vehChassis[slot];
        if (chassis < 0 || world->bodies.bodyPool.alive[chassis] == 0 ||
            world->bodies.bodyPool.generations[chassis] != world->vehicles.vehChassisGen[slot] ||
            world->bodies.types[chassis] != (uint8_t)m3_dynamicBody ||
            world->bodies.awake[chassis] == 0)
        {
            continue;
        }
        const m3Transform* xf = &world->bodies.transforms[chassis];
        m3Vec3 rlc = m3RotateVec3(xf->q, world->bodies.localCenters[chassis]);
        m3Pos3 com = {xf->p.x + (double)rlc.x, xf->p.y + (double)rlc.y, xf->p.z + (double)rlc.z};
        float mass =
            world->bodies.invMass[chassis] > 0.0f ? 1.0f / world->bodies.invMass[chassis] : 0.0f;
        float wheelMass = mass / (float)world->vehicles.vehWheelCount[slot];
        m3Mat3 invI = m3WorldInvInertia(world, chassis);

        // Two phases on purpose: every wheel reads the SAME pass
        // start velocities, then all impulses land together. A
        // sequential update lets wheel one's impulse leak into wheel
        // two's damper and the settled car holds a permanent tilt
        // (the first probe measured a centimeter of it).
        m3Vec3 v0 = world->bodies.linearVelocities[chassis];
        m3Vec3 w0 = world->bodies.angularVelocities[chassis];
        m3Vec3 impulses[M3_MAX_VEHICLE_WHEELS];
        m3Vec3 arms[M3_MAX_VEHICLE_WHEELS];
        int32_t applied = 0;

        // The drivetrain: one engine per vehicle, computed
        // BEFORE the wheel loop from pass-start velocities like
        // everything else. Engine speed derives from chassis forward
        // speed through the driven wheels' mean radius, the active
        // ratio, and the final drive; the curve turns it into crank
        // torque; gears and the final drive turn that into a force
        // per driven wheel. The friction circle still has the last
        // word at each tire. All arithmetic is +,-,*,/: bit-stable
        // on every platform.
        float dtForce = 0.0f;
        if (world->vehicles.vehDtActive[slot] != 0)
        {
            int32_t drivenCount = 0;
            float radiusSum = 0.0f;
            for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
            {
                int32_t k = slot * M3_MAX_VEHICLE_WHEELS + w;
                if ((world->vehicles.vehWheelFlags[k] & 2u) != 0)
                {
                    drivenCount += 1;
                    radiusSum += world->vehicles.vehWheelRadius[k];
                }
            }
            float radius = drivenCount > 0 ? radiusSum / (float)drivenCount : 0.0f;
            int32_t base = slot * M3_MAX_DRIVETRAIN_CURVE;
            int32_t count = world->vehicles.vehDtCurveCount[slot];
            int8_t gear = world->vehicles.vehDtGear[slot];
            float ratio = 0.0f;
            if (gear > 0)
            {
                ratio = world->vehicles.vehDtGearRatio[slot * M3_MAX_DRIVETRAIN_GEARS + (gear - 1)];
            }
            else if (gear < 0)
            {
                ratio = world->vehicles.vehDtReverse[slot];
            }

            // Raw engine speed, signed by rolling direction relative
            // to the gear: rolling against the gear reads as zero
            // (the idle floor catches it below).
            m3Vec3 fwd = m3RotateVec3(xf->q, (m3Vec3){1.0f, 0.0f, 0.0f});
            float vFwd = m3Dot3(v0, fwd);
            float wheelRps = radius > 0.0f ? vFwd / radius : 0.0f;
            float rawRpm = wheelRps * ratio * world->vehicles.vehDtFinal[slot] *
                           (60.0f / 6.28318530717958647692f);
            if (gear < 0)
            {
                rawRpm = -rawRpm;
            }
            if (rawRpm < 0.0f)
            {
                rawRpm = 0.0f;
            }

            // Auto shift manages forward gears only, and only with
            // the clutch closed: a shift opens it for clutchSteps,
            // which is also the thrash brake.
            if (world->vehicles.vehDtAutoShift[slot] != 0 && gear >= 1 &&
                world->vehicles.vehDtClutch[slot] == 0)
            {
                if (rawRpm > world->vehicles.vehDtShiftUp[slot] &&
                    gear < (int8_t)world->vehicles.vehDtGearCount[slot])
                {
                    gear += 1;
                    world->vehicles.vehDtGear[slot] = gear;
                    world->vehicles.vehDtClutch[slot] = world->vehicles.vehDtClutchSteps[slot];
                    ratio =
                        world->vehicles.vehDtGearRatio[slot * M3_MAX_DRIVETRAIN_GEARS + (gear - 1)];
                }
                else if (rawRpm < world->vehicles.vehDtShiftDown[slot] && gear > 1)
                {
                    gear -= 1;
                    world->vehicles.vehDtGear[slot] = gear;
                    world->vehicles.vehDtClutch[slot] = world->vehicles.vehDtClutchSteps[slot];
                    ratio =
                        world->vehicles.vehDtGearRatio[slot * M3_MAX_DRIVETRAIN_GEARS + (gear - 1)];
                }
            }

            // The tachometer: idle-floored at the first control
            // point, flat past the last. This is what the curve
            // reads and what the API reports.
            float rpm = rawRpm;
            if (rpm < world->vehicles.vehDtCurveRpm[base])
            {
                rpm = world->vehicles.vehDtCurveRpm[base];
            }
            if (rpm > world->vehicles.vehDtCurveRpm[base + count - 1])
            {
                rpm = world->vehicles.vehDtCurveRpm[base + count - 1];
            }
            world->vehicles.vehDtRpm[slot] = rpm;

            int32_t cut = world->vehicles.vehDtClutch[slot] > 0;
            if (cut)
            {
                world->vehicles.vehDtClutch[slot] -= 1;
            }
            float thr = world->vehicles.vehThrottle[slot];
            if (thr < 0.0f)
            {
                thr = 0.0f; // with a drivetrain, reverse is a gear
            }
            if (!cut && gear != 0 && drivenCount > 0 && thr > 0.0f)
            {
                float seg = 0.0f;
                for (int32_t c = 0; c < count - 1; ++c)
                {
                    if (rpm <= world->vehicles.vehDtCurveRpm[base + c + 1] || c == count - 2)
                    {
                        float r0 = world->vehicles.vehDtCurveRpm[base + c];
                        float r1 = world->vehicles.vehDtCurveRpm[base + c + 1];
                        float t = (rpm - r0) / (r1 - r0);
                        seg = world->vehicles.vehDtCurveTorque[base + c] +
                              t * (world->vehicles.vehDtCurveTorque[base + c + 1] -
                                   world->vehicles.vehDtCurveTorque[base + c]);
                        break;
                    }
                }
                float sign = gear < 0 ? -1.0f : 1.0f;
                dtForce = sign * seg * thr * ratio * world->vehicles.vehDtFinal[slot] /
                          (radius * (float)drivenCount);
            }
        }

        // Differentials: the driven mean of the LAST step's
        // contact speeds, one pass and one step of lag like the
        // engine's own wheel reading.
        float dtMeanLon = 0.0f;
        if (world->vehicles.vehDtActive[slot] != 0 && world->vehicles.vehDtDiffMode[slot] != 0)
        {
            int32_t drivenSeen = 0;
            for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
            {
                int32_t k = slot * M3_MAX_VEHICLE_WHEELS + w;
                if ((world->vehicles.vehWheelFlags[k] & 2u) != 0)
                {
                    dtMeanLon += world->vehicles.vehWheelLon[k];
                    drivenSeen += 1;
                }
            }
            if (drivenSeen > 0)
            {
                dtMeanLon /= (float)drivenSeen;
            }
        }

        for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
        {
            int32_t k = slot * M3_MAX_VEHICLE_WHEELS + w;
            m3Vec3 anchorR = m3RotateVec3(xf->q, world->vehicles.vehWheelAnchor[k]);
            m3Pos3 anchor = {xf->p.x + (double)anchorR.x, xf->p.y + (double)anchorR.y,
                             xf->p.z + (double)anchorR.z};
            m3Vec3 dir = m3RotateVec3(xf->q, world->vehicles.vehWheelDir[k]);
            float reach = world->vehicles.vehWheelRest[k] + world->vehicles.vehWheelRadius[k];
            m3CastResult hit = m3RayClosestExcept(world, anchor, m3MulSV3(reach, dir), chassis);
            if (!hit.hit)
            {
                world->vehicles.vehWheelCompression[k] = 0.0f;
                world->vehicles.vehWheelContact[k] = 0;
                continue;
            }
            float suspLen = hit.fraction * reach - world->vehicles.vehWheelRadius[k];
            float floorLen = world->vehicles.vehWheelRest[k] - world->vehicles.vehWheelTravel[k];
            if (suspLen < floorLen)
            {
                suspLen = floorLen; // bottomed out: travel is a hard book
            }
            float x = world->vehicles.vehWheelRest[k] - suspLen;
            if (x < 0.0f)
            {
                x = 0.0f;
            }
            world->vehicles.vehWheelCompression[k] = x;
            world->vehicles.vehWheelContact[k] = 1;

            // The anchor's velocity along the suspension: positive
            // means compressing.
            m3Vec3 arm = {(float)(anchor.x - com.x), (float)(anchor.y - com.y),
                          (float)(anchor.z - com.z)};
            m3Vec3 vAnchor = m3Add3(v0, m3Cross3(w0, arm));
            float compressSpeed = m3Dot3(vAnchor, dir);

            float omega = 2.0f * 3.14159265358979323846f * world->vehicles.vehWheelHertz[k];
            float stiffness = wheelMass * omega * omega;
            float damping = 2.0f * wheelMass * world->vehicles.vehWheelZeta[k] * omega;
            float force = stiffness * x + damping * compressSpeed;
            if (force < 0.0f)
            {
                force = 0.0f; // suspension pushes, never pulls
            }
            // In one step the suspension at most reverses its share of
            // the chassis from the speed limit, which also bounds the
            // friction budget below.
            float most = 2.0f * M3_SPEED_LIMIT * wheelMass / dt;
            if (force > most)
            {
                force = most;
            }
            m3Vec3 total = m3MulSV3(-force * dt, dir);
            // Impulses apply at the WHEEL HUB, not the ground
            // contact. Along the suspension axis the two points
            // torque identically (the offset is parallel to the
            // normal impulse), but for tangent impulses the ground
            // point adds a wheel radius of fake pitch lever: the
            // first probe showed full throttle unloading the front
            // axle to an eighth of its share, the friction circle
            // strangling the front tires, and a yaw instability
            // walking the car sideways at zero steer.
            m3Vec3 hubArm = {arm.x + dir.x * suspLen, arm.y + dir.y * suspLen,
                             arm.z + dir.z * suspLen};

            // The tire: drive, brake, and lateral impulses in
            // the contact plane, all clamped by one friction circle
            // against this wheel's suspension load. The chassis
            // local +x axis is forward by convention; a steerable
            // wheel's frame rotates about its suspension axis.
            m3Vec3 fLocal = {1.0f, 0.0f, 0.0f};
            if ((world->vehicles.vehWheelFlags[k] & 1u) != 0 &&
                world->vehicles.vehSteer[slot] != 0.0f)
            {
                float a = world->vehicles.vehSteer[slot] * world->vehicles.vehMaxSteer[slot];
                float half = 0.5f * a;
                m3Vec3 up = m3MulSV3(-1.0f, world->vehicles.vehWheelDir[k]);
                m3CosSin halfCs = m3ComputeCosSin(half);
                m3Quat qa = {up.x * halfCs.s, up.y * halfCs.s, up.z * halfCs.s, halfCs.c};
                fLocal = m3RotateVec3(qa, fLocal);
            }
            m3Vec3 fWorld = m3RotateVec3(xf->q, fLocal);
            m3Vec3 n = hit.normal;
            m3Vec3 forward = m3Sub3(fWorld, m3MulSV3(m3Dot3(fWorld, n), n));
            float fLen2 = m3Dot3(forward, forward);
            if (fLen2 > 1.0e-8f)
            {
                forward = m3MulSV3(1.0f / sqrtf(fLen2), forward);
                m3Vec3 side = m3Cross3(n, forward);
                // Tire velocities are RELATIVE to the surface under
                // the wheel: a car parked on a ferry must ride the
                // ferry, not fight it (an absolute kill drags every
                // moving platform to a halt under its passenger).
                int32_t hitShape = hit.shapeId.index1 - 1;
                int32_t hitBody = world->shapes.shapeBody[hitShape];
                m3Vec3 vSurf = {0.0f, 0.0f, 0.0f};
                if (world->bodies.types[hitBody] != (uint8_t)m3_staticBody)
                {
                    m3Vec3 rlcH = m3RotateVec3(world->bodies.transforms[hitBody].q,
                                               world->bodies.localCenters[hitBody]);
                    m3Vec3 armH = {
                        (float)(hit.point.x - world->bodies.transforms[hitBody].p.x) - rlcH.x,
                        (float)(hit.point.y - world->bodies.transforms[hitBody].p.y) - rlcH.y,
                        (float)(hit.point.z - world->bodies.transforms[hitBody].p.z) - rlcH.z};
                    vSurf = m3Add3(world->bodies.linearVelocities[hitBody],
                                   m3Cross3(world->bodies.angularVelocities[hitBody], armH));
                }
                m3Vec3 vContact = m3Sub3(m3Add3(v0, m3Cross3(w0, hubArm)), vSurf);
                float vLon = m3Dot3(vContact, forward);
                world->vehicles.vehWheelLon[k] = vLon; // the diff's next-step read
                float vLat = m3Dot3(vContact, side);

                // Velocity kills use the solver's own effective
                // mass (linear plus angular response at the hub),
                // split by wheel count: four wheels each killing
                // the SHARED lateral velocity in full is a fourfold
                // overcorrection, and the hub's angular feedback
                // pushes the loop gain past one. The first probe
                // watched that pump grow a 1e-9 yaw seed into a
                // full sideways walk at 2.3x per step.
                m3Vec3 rxf = m3Cross3(hubArm, forward);
                float kLon = world->bodies.invMass[chassis] + m3Dot3(rxf, m3MulMV3(invI, rxf));
                float effLon = kLon > 0.0f ? 1.0f / kLon : 0.0f;
                m3Vec3 rxs = m3Cross3(hubArm, side);
                float kLat = world->bodies.invMass[chassis] + m3Dot3(rxs, m3MulMV3(invI, rxs));
                float effLat = kLat > 0.0f ? 1.0f / kLat : 0.0f;
                float share = 1.0f / (float)world->vehicles.vehWheelCount[slot];

                float lon = 0.0f;
                if ((world->vehicles.vehWheelFlags[k] & 2u) != 0)
                {
                    if (world->vehicles.vehDtActive[slot] != 0)
                    {
                        float driveForce = dtForce;
                        if (world->vehicles.vehDtDiffMode[slot] != 0)
                        {
                            float coupling = world->vehicles.vehDtDiffCouple[slot] *
                                             (dtMeanLon - world->vehicles.vehWheelLon[k]);
                            if (world->vehicles.vehDtDiffMode[slot] == 1)
                            {
                                // Limited slip: the coupling may not
                                // exceed the engine's own share.
                                float cap = m3AbsF(dtForce);
                                coupling = m3MaxF(-cap, m3MinF(cap, coupling));
                            }
                            driveForce += coupling;
                        }
                        lon += driveForce * dt;
                    }
                    else if (world->vehicles.vehTrackMode[slot] != 0)
                    {
                        // Skid steer: the side picks its own
                        // throttle by the anchor's chassis-local z
                        // (+z right by convention).
                        float trackThr = world->vehicles.vehWheelAnchor[k].z >= 0.0f
                                             ? world->vehicles.vehTrackRight[slot]
                                             : world->vehicles.vehTrackLeft[slot];
                        lon += trackThr * world->vehicles.vehDriveForce[slot] * dt;
                    }
                    else
                    {
                        lon += world->vehicles.vehThrottle[slot] *
                               world->vehicles.vehDriveForce[slot] * dt;
                    }
                }
                if (world->vehicles.vehBrake[slot] > 0.0f)
                {
                    // A brake opposes rolling and never reverses it.
                    float budget = world->vehicles.vehBrake[slot] *
                                   world->vehicles.vehBrakeForce[slot] *
                                   world->vehicles.vehWheelBrake[k] * dt;
                    float want = -vLon * effLon * share;
                    lon += want > budget ? budget : (want < -budget ? -budget : want);
                }
                // The tire kills its share of lateral slip; the
                // friction circle decides how much survives (that
                // surrender is the drift).
                float lat = -vLat * effLat * share;
                float budget2 = world->vehicles.vehTireGrip[slot] * force * dt;
                HoldInCircle(&lon, &lat, budget2);
                total = m3Add3(total, m3Add3(m3MulSV3(lon, forward), m3MulSV3(lat, side)));
                world->vehicles.vehWheelSpin[k] += (vLon / world->vehicles.vehWheelRadius[k]) * dt;
            }

            impulses[applied] = total;
            arms[applied] = hubArm;
            applied += 1;

            // Newton's third law for dynamic ground: a wheel
            // pressing or driving on a fragment pushes the fragment
            // back, or cars would mint momentum from loose rubble.
            int32_t under = world->shapes.shapeBody[hit.shapeId.index1 - 1];
            if (world->bodies.types[under] == (uint8_t)m3_dynamicBody &&
                world->bodies.invMass[under] > 0.0f)
            {
                m3Vec3 rlcU = m3RotateVec3(world->bodies.transforms[under].q,
                                           world->bodies.localCenters[under]);
                m3Vec3 armU = {(float)(hit.point.x - world->bodies.transforms[under].p.x) - rlcU.x,
                               (float)(hit.point.y - world->bodies.transforms[under].p.y) - rlcU.y,
                               (float)(hit.point.z - world->bodies.transforms[under].p.z) - rlcU.z};
                m3PushHeld(world, under, -1.0f, total, armU);
                world->bodies.awake[under] = 1;
                world->bodies.sleepTimes[under] = 0.0f;
            }
        }
        for (int32_t a = 0; a < applied; ++a)
        {
            m3PushHeld(world, chassis, 1.0f, impulses[a], arms[a]);
        }

        // The lean stabilizer: a two-wheeler is an inverted
        // pendulum, so an opt-in controller rolls the chassis toward
        // the lean the turn demands. It works in angular velocity
        // directly (inertia-free, the gain reads in 1/s^2) and only
        // about the forward axis, so it cannot mint yaw or pitch.
        // The target comes from the steer command, not the measured
        // yaw, because the command is journaled state and the
        // measurement would feed the controller its own noise.
        float leanGain = world->vehicles.vehLeanGain[slot];
        if (leanGain > 0.0f)
        {
            m3Vec3 fwd = m3RotateVec3(xf->q, (m3Vec3){1.0f, 0.0f, 0.0f});
            m3Vec3 up = m3RotateVec3(xf->q, (m3Vec3){0.0f, 1.0f, 0.0f});
            float g2 = m3Dot3(world->gravity, world->gravity);
            float gMag = g2 > 1.0e-6f ? sqrtf(g2) : 10.0f;
            m3Vec3 worldUp =
                g2 > 1.0e-6f ? m3MulSV3(-1.0f / gMag, world->gravity) : (m3Vec3){0.0f, 1.0f, 0.0f};
            float xMin = 0.0f;
            float xMax = 0.0f;
            for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
            {
                float ax = world->vehicles.vehWheelAnchor[slot * M3_MAX_VEHICLE_WHEELS + w].x;
                xMin = ax < xMin ? ax : xMin;
                xMax = ax > xMax ? ax : xMax;
            }
            float wheelbase = xMax - xMin > 0.1f ? xMax - xMin : 0.1f;
            float steerA = world->vehicles.vehSteer[slot] * world->vehicles.vehMaxSteer[slot];
            m3CosSin steerCs = m3ComputeCosSin(steerA);
            float tanSteer = steerCs.c > 0.1f ? steerCs.s / steerCs.c : 0.0f;
            float v = m3Dot3(world->bodies.linearVelocities[chassis], fwd);
            float latOverG = v * (v * tanSteer / wheelbase) / gMag;
            float sTarget = latOverG / sqrtf(1.0f + latOverG * latOverG);
            float sLean = m3Dot3(m3Cross3(up, worldUp), fwd);
            float rollRate = m3Dot3(world->bodies.angularVelocities[chassis], fwd);
            float damp = 2.0f * sqrtf(leanGain);
            float drr = (-leanGain * (sTarget - sLean) - damp * rollRate) * dt;
            // A bike leans about the CONTACT line, not its center:
            // a pure roll about the CG sweeps the hubs sideways and
            // the lateral tire kill vetoes it the same substep (the
            // first probe watched gain 25 lose to its own tires).
            // Rolling about the hub line means pairing the angular
            // change with its conjugate lateral velocity at the CG.
            float hubDrop = 0.0f;
            for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
            {
                int32_t k = slot * M3_MAX_VEHICLE_WHEELS + w;
                m3Vec3 hub = m3Add3(
                    world->vehicles.vehWheelAnchor[k],
                    m3MulSV3(world->vehicles.vehWheelRest[k], world->vehicles.vehWheelDir[k]));
                hubDrop -= hub.y;
            }
            hubDrop /= (float)world->vehicles.vehWheelCount[slot];
            m3Vec3 side = m3RotateVec3(xf->q, (m3Vec3){0.0f, 0.0f, 1.0f});
            world->bodies.angularVelocities[chassis] =
                m3Add3(world->bodies.angularVelocities[chassis], m3MulSV3(drr, fwd));
            world->bodies.linearVelocities[chassis] =
                m3Add3(world->bodies.linearVelocities[chassis], m3MulSV3(drr * hubDrop, side));
        }
    }
}

void m3VehicleTankCommandsInternal(m3World* world, int32_t slot, float left, float right,
                                   float brake)
{
    world->vehicles.vehTrackMode[slot] = 1;
    world->vehicles.vehTrackLeft[slot] = left < -1.0f ? -1.0f : (left > 1.0f ? 1.0f : left);
    world->vehicles.vehTrackRight[slot] = right < -1.0f ? -1.0f : (right > 1.0f ? 1.0f : right);
    world->vehicles.vehBrake[slot] = brake < 0.0f ? 0.0f : (brake > 1.0f ? 1.0f : brake);
    world->vehicles.vehThrottle[slot] = 0.0f;
    world->vehicles.vehSteer[slot] = 0.0f;
    int32_t chassis = world->vehicles.vehChassis[slot];
    if (world->bodies.types[chassis] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, chassis, 1);
    }
}

void m3VehicleCommandsInternal(m3World* world, int32_t slot, float throttle, float steer,
                               float brake)
{
    world->vehicles.vehTrackMode[slot] = 0; // normal commands disengage the tracks
    world->vehicles.vehThrottle[slot] =
        throttle < -1.0f ? -1.0f : (throttle > 1.0f ? 1.0f : throttle);
    world->vehicles.vehSteer[slot] = steer < -1.0f ? -1.0f : (steer > 1.0f ? 1.0f : steer);
    world->vehicles.vehBrake[slot] = brake < 0.0f ? 0.0f : (brake > 1.0f ? 1.0f : brake);
    int32_t chassis = world->vehicles.vehChassis[slot];
    if (chassis >= 0 && world->bodies.bodyPool.alive[chassis] != 0 &&
        world->bodies.bodyPool.generations[chassis] == world->vehicles.vehChassisGen[slot])
    {
        world->bodies.awake[chassis] = 1; // a commanded car always wakes
        world->bodies.sleepTimes[chassis] = 0.0f;
    }
}

static M3_NOINLINE m3Result CreateVehicleWorker(m3World* world, const m3VehicleDef* def,
                                                m3VehicleId* vehicleOut)
{
    if (vehicleOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *vehicleOut = m3_nullVehicleId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorBusy);
    }
    if (world == NULL || def == NULL || def->internalValue != M3_VEHICLE_COOKIE)
    {
        return m3Refuse(world, m3_errorInvalid); // field checks live in the internal
    }
    int32_t slot = m3CreateVehicleInternal(world, def);
    if (slot < 0)
    {
        return m3CreateRefusal(world, slot);
    }
    m3VehicleId id = {slot + 1, world->vehicles.vehPool.generations[slot]};
    if (world->recorder.journalActive != 0)
    {
        m3OpCreateVehicle record;
        memset(&record, 0, sizeof(record));
        m3JournalVehicleDef(&record.def, def);
        record.expected = id;
        m3JournalRecord(world, m3_opCreateVehicle, &record, (int32_t)sizeof(record));
    }
    *vehicleOut = id;
    return m3_success;
}

m3Result m3CreateVehicle(m3World* world, const m3VehicleDef* def, m3VehicleId* vehicleOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateVehicleWorker(world, def, vehicleOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result DestroyVehicleWorker(m3World* world, m3VehicleId vehicle)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorBusy);
    }
    int32_t slot = m3VehicleSlot(world, vehicle);
    if (slot < 0)
    {
        return m3Refuse(world, m3MissingId(world, vehicle.index1));
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyVehicle, &vehicle, (int32_t)sizeof(vehicle));
    }
    m3DestroyVehicleInternal(world, slot);
    return m3_success;
}

m3Result m3DestroyVehicle(m3World* world, m3VehicleId vehicle)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = DestroyVehicleWorker(world, vehicle);
    m3FloatEnvLeave(env);
    return result;
}

bool m3Vehicle_IsValid(const m3World* world, m3VehicleId vehicle)
{
    return m3VehicleSlot(world, vehicle) >= 0;
}

static M3_NOINLINE float VehicleGetCompressionWorker(const m3World* world, m3VehicleId vehicle,
                                                     int32_t wheel)
{
    int32_t slot = m3VehicleSlot(world, vehicle);
    if (slot < 0 || wheel < 0 || wheel >= world->vehicles.vehWheelCount[slot])
    {
        m3Refuse(world, m3_errorInvalid);
        return 0.0f;
    }
    return world->vehicles.vehWheelCompression[slot * M3_MAX_VEHICLE_WHEELS + wheel];
}

float m3Vehicle_GetCompression(const m3World* world, m3VehicleId vehicle, int32_t wheel)
{
    uint64_t env = m3FloatEnvEnter();
    float result = VehicleGetCompressionWorker(world, vehicle, wheel);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE void VehicleSetCommandsWorker(m3World* world, m3VehicleId vehicle,
                                                 float throttle, float steer, float brake)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t slot = m3VehicleSlot(world, vehicle);
    if (slot < 0 || !m3FiniteF(throttle) || !m3FiniteF(steer) || !m3FiniteF(brake))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale id or hostile command: a documented no-op
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVehicleCommands record;
        memset(&record, 0, sizeof(record));
        record.id = vehicle;
        record.throttle = throttle;
        record.steer = steer;
        record.brake = brake;
        m3JournalRecord(world, m3_opVehicleCommands, &record, (int32_t)sizeof(record));
    }
    m3VehicleCommandsInternal(world, slot, throttle, steer, brake);
}

void m3Vehicle_SetCommands(m3World* world, m3VehicleId vehicle, float throttle, float steer,
                           float brake)
{
    uint64_t env = m3FloatEnvEnter();
    VehicleSetCommandsWorker(world, vehicle, throttle, steer, brake);
    m3FloatEnvLeave(env);
}

bool m3VehicleTankCommandsValid(const m3World* world, int32_t slot, float left, float right,
                                float brake)
{
    // A gearbox and a skid steer are different machines.
    return m3FiniteF(left) && m3FiniteF(right) && m3FiniteF(brake) &&
           world->vehicles.vehDtActive[slot] == 0;
}

static M3_NOINLINE void VehicleSetTankCommandsWorker(m3World* world, m3VehicleId vehicle,
                                                     float left, float right, float brake)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t slot = m3VehicleSlot(world, vehicle);
    if (slot < 0 || !m3VehicleTankCommandsValid(world, slot, left, right, brake))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVehicleTankCommands record;
        memset(&record, 0, sizeof(record));
        record.id = vehicle;
        record.left = left;
        record.right = right;
        record.brake = brake;
        m3JournalRecord(world, m3_opVehicleTankCommands, &record, (int32_t)sizeof(record));
    }
    m3VehicleTankCommandsInternal(world, slot, left, right, brake);
}

void m3Vehicle_SetTankCommands(m3World* world, m3VehicleId vehicle, float left, float right,
                               float brake)
{
    uint64_t env = m3FloatEnvEnter();
    VehicleSetTankCommandsWorker(world, vehicle, left, right, brake);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE float VehicleGetWheelSpinWorker(const m3World* world, m3VehicleId vehicle,
                                                   int32_t wheel)
{
    int32_t slot = m3VehicleSlot(world, vehicle);
    if (slot < 0 || wheel < 0 || wheel >= world->vehicles.vehWheelCount[slot])
    {
        m3Refuse(world, m3_errorInvalid);
        return 0.0f;
    }
    return world->vehicles.vehWheelSpin[slot * M3_MAX_VEHICLE_WHEELS + wheel];
}

float m3Vehicle_GetWheelSpin(const m3World* world, m3VehicleId vehicle, int32_t wheel)
{
    uint64_t env = m3FloatEnvEnter();
    float result = VehicleGetWheelSpinWorker(world, vehicle, wheel);
    m3FloatEnvLeave(env);
    return result;
}

bool m3Vehicle_IsWheelGrounded(const m3World* world, m3VehicleId vehicle, int32_t wheel)
{
    int32_t slot = m3VehicleSlot(world, vehicle);
    if (slot < 0 || wheel < 0 || wheel >= world->vehicles.vehWheelCount[slot])
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    return world->vehicles.vehWheelContact[slot * M3_MAX_VEHICLE_WHEELS + wheel] != 0;
}
