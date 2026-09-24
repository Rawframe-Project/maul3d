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

#include "body.h"
#include "journal.h"
#include "raycast.h"
#include "solver.h"
#include "vehicle.h"
#include "world.h"
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
    for (int32_t c = 0; c < M3_DRIVETRAIN_MAX_CURVE; ++c)
    {
        world->vehicles.vehDtCurveRpm[slot * M3_DRIVETRAIN_MAX_CURVE + c] = 0.0f;
        world->vehicles.vehDtCurveTorque[slot * M3_DRIVETRAIN_MAX_CURVE + c] = 0.0f;
    }
    for (int32_t g = 0; g < M3_DRIVETRAIN_MAX_GEARS; ++g)
    {
        world->vehicles.vehDtGearRatio[slot * M3_DRIVETRAIN_MAX_GEARS + g] = 0.0f;
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
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        def.wheels[w].direction = (m3Vec3){0.0f, -1.0f, 0.0f};
        def.wheels[w].restLength = 0.4f;
        def.wheels[w].travel = 0.25f;
        def.wheels[w].hertz = 1.5f;
        def.wheels[w].zeta = 0.6f;
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
    if (world == NULL || vehicleId.world != world->idWorld ||
        !m3IdPoolValid(&world->vehicles.vehPool, index, vehicleId.generation))
    {
        return -1;
    }
    return index;
}

static bool WheelDefValid(const m3WheelDef* wd)
{
    m3real d2 = m3Dot3(wd->direction, wd->direction);
    return m3FiniteV3(wd->anchor) && m3FiniteV3(wd->direction) && d2 > 0.81f && d2 < 1.21f &&
           m3FiniteF(wd->restLength) && wd->restLength > 0.0f && m3FiniteF(wd->travel) &&
           wd->travel > 0.0f && wd->travel <= wd->restLength && m3FiniteF(wd->hertz) &&
           wd->hertz > 0.0f && m3FiniteF(wd->zeta) && wd->zeta >= 0.0f && m3FiniteF(wd->radius) &&
           wd->radius > 0.0f && m3FiniteF(wd->brakeShare) && wd->brakeShare >= 0.0f;
}

// Every field check lives here, where the public door and replay both
// pass: replay hands raw journal bytes, and a flipped wheel count would
// overrun every per-wheel array. The chassis must be a live dynamic body.
static bool VehicleDefValid(const m3World* world, const m3VehicleDef* def)
{
    if (def->wheelCount < 1 || def->wheelCount > M3_VEHICLE_MAX_WHEELS ||
        !m3FiniteF(def->maxSteerAngle) || def->maxSteerAngle < 0.0f ||
        !m3FiniteF(def->driveForce) || def->driveForce < 0.0f || !m3FiniteF(def->brakeForce) ||
        def->brakeForce < 0.0f || !m3FiniteF(def->tireGrip) || def->tireGrip < 0.0f ||
        !m3FiniteF(def->leanStabilization) || def->leanStabilization < 0.0f)
    {
        return false;
    }
    for (int32_t w = 0; w < def->wheelCount; ++w)
    {
        if (!WheelDefValid(&def->wheels[w]))
        {
            return false;
        }
    }
    int32_t chassis = def->chassisId.index1 - 1;
    return chassis >= 0 && chassis < world->bodies.bodyCapacity &&
           world->bodies.bodyPool.alive[chassis] != 0 &&
           world->bodies.bodyPool.generations[chassis] == def->chassisId.generation &&
           world->bodies.types[chassis] == (uint8_t)m3_dynamicBody;
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
    v->vehWheelHertz[k] = src->hertz;
    v->vehWheelZeta[k] = src->zeta;
    v->vehWheelRadius[k] = src->radius;
    v->vehWheelFlags[k] = (uint8_t)((src->steerable ? 1u : 0u) | (src->driven ? 2u : 0u));
    v->vehWheelBrake[k] = src->brakeShare;
    v->vehWheelCompression[k] = 0.0f;
    v->vehWheelContact[k] = 0;
    v->vehWheelSpin[k] = 0.0f;
}

int32_t m3CreateVehicleInternal(m3World* world, const m3VehicleDef* def)
{
    if (!VehicleDefValid(world, def))
    {
        return -1;
    }
    int32_t slot = m3IdPoolAlloc(&world->vehicles.vehPool);
    if (slot < 0)
    {
        return -1;
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
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        InitWheel(world, slot * M3_VEHICLE_MAX_WHEELS + w,
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
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        int32_t k = slot * M3_VEHICLE_MAX_WHEELS + w;
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
        m3real mass =
            world->bodies.invMass[chassis] > 0.0f ? 1.0f / world->bodies.invMass[chassis] : 0.0f;
        m3real wheelMass = mass / (m3real)world->vehicles.vehWheelCount[slot];
        m3Mat3 invI = m3WorldInvInertia(world, chassis);

        // Two phases on purpose: every wheel reads the SAME pass
        // start velocities, then all impulses land together. A
        // sequential update lets wheel one's impulse leak into wheel
        // two's damper and the settled car holds a permanent tilt
        // (the first probe measured a centimeter of it).
        m3Vec3 v0 = world->bodies.linearVelocities[chassis];
        m3Vec3 w0 = world->bodies.angularVelocities[chassis];
        m3Vec3 impulses[M3_VEHICLE_MAX_WHEELS];
        m3Vec3 arms[M3_VEHICLE_MAX_WHEELS];
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
        m3real dtForce = 0.0f;
        if (world->vehicles.vehDtActive[slot] != 0)
        {
            int32_t drivenCount = 0;
            m3real radiusSum = 0.0f;
            for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
            {
                int32_t k = slot * M3_VEHICLE_MAX_WHEELS + w;
                if ((world->vehicles.vehWheelFlags[k] & 2u) != 0)
                {
                    drivenCount += 1;
                    radiusSum += world->vehicles.vehWheelRadius[k];
                }
            }
            m3real radius = drivenCount > 0 ? radiusSum / (m3real)drivenCount : 0.0f;
            int32_t base = slot * M3_DRIVETRAIN_MAX_CURVE;
            int32_t count = world->vehicles.vehDtCurveCount[slot];
            int8_t gear = world->vehicles.vehDtGear[slot];
            m3real ratio = 0.0f;
            if (gear > 0)
            {
                ratio = world->vehicles.vehDtGearRatio[slot * M3_DRIVETRAIN_MAX_GEARS + (gear - 1)];
            }
            else if (gear < 0)
            {
                ratio = world->vehicles.vehDtReverse[slot];
            }

            // Raw engine speed, signed by rolling direction relative
            // to the gear: rolling against the gear reads as zero
            // (the idle floor catches it below).
            m3Vec3 fwd = m3RotateVec3(xf->q, (m3Vec3){1.0f, 0.0f, 0.0f});
            m3real vFwd = m3Dot3(v0, fwd);
            m3real wheelRps = radius > 0.0f ? vFwd / radius : 0.0f;
            m3real rawRpm = wheelRps * ratio * world->vehicles.vehDtFinal[slot] *
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
                        world->vehicles.vehDtGearRatio[slot * M3_DRIVETRAIN_MAX_GEARS + (gear - 1)];
                }
                else if (rawRpm < world->vehicles.vehDtShiftDown[slot] && gear > 1)
                {
                    gear -= 1;
                    world->vehicles.vehDtGear[slot] = gear;
                    world->vehicles.vehDtClutch[slot] = world->vehicles.vehDtClutchSteps[slot];
                    ratio =
                        world->vehicles.vehDtGearRatio[slot * M3_DRIVETRAIN_MAX_GEARS + (gear - 1)];
                }
            }

            // The tachometer: idle-floored at the first control
            // point, flat past the last. This is what the curve
            // reads and what the API reports.
            m3real rpm = rawRpm;
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
            m3real thr = world->vehicles.vehThrottle[slot];
            if (thr < 0.0f)
            {
                thr = 0.0f; // with a drivetrain, reverse is a gear
            }
            if (!cut && gear != 0 && drivenCount > 0 && thr > 0.0f)
            {
                m3real seg = 0.0f;
                for (int32_t c = 0; c < count - 1; ++c)
                {
                    if (rpm <= world->vehicles.vehDtCurveRpm[base + c + 1] || c == count - 2)
                    {
                        m3real r0 = world->vehicles.vehDtCurveRpm[base + c];
                        m3real r1 = world->vehicles.vehDtCurveRpm[base + c + 1];
                        m3real t = (rpm - r0) / (r1 - r0);
                        seg = world->vehicles.vehDtCurveTorque[base + c] +
                              t * (world->vehicles.vehDtCurveTorque[base + c + 1] -
                                   world->vehicles.vehDtCurveTorque[base + c]);
                        break;
                    }
                }
                m3real sign = gear < 0 ? -1.0f : 1.0f;
                dtForce = sign * seg * thr * ratio * world->vehicles.vehDtFinal[slot] /
                          (radius * (m3real)drivenCount);
            }
        }

        // Differentials: the driven mean of the LAST step's
        // contact speeds, one pass and one step of lag like the
        // engine's own wheel reading.
        m3real dtMeanLon = 0.0f;
        if (world->vehicles.vehDtActive[slot] != 0 && world->vehicles.vehDtDiffMode[slot] != 0)
        {
            int32_t drivenSeen = 0;
            for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
            {
                int32_t k = slot * M3_VEHICLE_MAX_WHEELS + w;
                if ((world->vehicles.vehWheelFlags[k] & 2u) != 0)
                {
                    dtMeanLon += world->vehicles.vehWheelLon[k];
                    drivenSeen += 1;
                }
            }
            if (drivenSeen > 0)
            {
                dtMeanLon /= (m3real)drivenSeen;
            }
        }

        for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
        {
            int32_t k = slot * M3_VEHICLE_MAX_WHEELS + w;
            m3Vec3 anchorR = m3RotateVec3(xf->q, world->vehicles.vehWheelAnchor[k]);
            m3Pos3 anchor = {xf->p.x + (double)anchorR.x, xf->p.y + (double)anchorR.y,
                             xf->p.z + (double)anchorR.z};
            m3Vec3 dir = m3RotateVec3(xf->q, world->vehicles.vehWheelDir[k]);
            m3real reach = world->vehicles.vehWheelRest[k] + world->vehicles.vehWheelRadius[k];
            m3RayCastResult hit = m3RayClosestExcept(world, anchor, m3MulSV3(reach, dir), chassis);
            if (!hit.hit)
            {
                world->vehicles.vehWheelCompression[k] = 0.0f;
                world->vehicles.vehWheelContact[k] = 0;
                continue;
            }
            m3real suspLen = hit.fraction * reach - world->vehicles.vehWheelRadius[k];
            m3real floorLen = world->vehicles.vehWheelRest[k] - world->vehicles.vehWheelTravel[k];
            if (suspLen < floorLen)
            {
                suspLen = floorLen; // bottomed out: travel is a hard book
            }
            m3real x = world->vehicles.vehWheelRest[k] - suspLen;
            if (x < 0.0f)
            {
                x = 0.0f;
            }
            world->vehicles.vehWheelCompression[k] = x;
            world->vehicles.vehWheelContact[k] = 1;

            // The anchor's velocity along the suspension: positive
            // means compressing.
            m3Vec3 arm = {(m3real)(anchor.x - com.x), (m3real)(anchor.y - com.y),
                          (m3real)(anchor.z - com.z)};
            m3Vec3 vAnchor = m3Add3(v0, m3Cross3(w0, arm));
            m3real compressSpeed = m3Dot3(vAnchor, dir);

            m3real omega = 2.0f * 3.14159265358979323846f * world->vehicles.vehWheelHertz[k];
            m3real stiffness = wheelMass * omega * omega;
            m3real damping = 2.0f * wheelMass * world->vehicles.vehWheelZeta[k] * omega;
            m3real force = stiffness * x + damping * compressSpeed;
            if (force < 0.0f)
            {
                force = 0.0f; // suspension pushes, never pulls
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
                m3real a = world->vehicles.vehSteer[slot] * world->vehicles.vehMaxSteer[slot];
                m3real half = 0.5f * a;
                m3Vec3 up = m3MulSV3(-1.0f, world->vehicles.vehWheelDir[k]);
                m3CosSin halfCs = m3ComputeCosSin(half);
                m3Quat qa = {up.x * halfCs.s, up.y * halfCs.s, up.z * halfCs.s, halfCs.c};
                fLocal = m3RotateVec3(qa, fLocal);
            }
            m3Vec3 fWorld = m3RotateVec3(xf->q, fLocal);
            m3Vec3 n = hit.normal;
            m3Vec3 forward = m3Sub3(fWorld, m3MulSV3(m3Dot3(fWorld, n), n));
            m3real fLen2 = m3Dot3(forward, forward);
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
                        (m3real)(hit.point.x - world->bodies.transforms[hitBody].p.x) - rlcH.x,
                        (m3real)(hit.point.y - world->bodies.transforms[hitBody].p.y) - rlcH.y,
                        (m3real)(hit.point.z - world->bodies.transforms[hitBody].p.z) - rlcH.z};
                    vSurf = m3Add3(world->bodies.linearVelocities[hitBody],
                                   m3Cross3(world->bodies.angularVelocities[hitBody], armH));
                }
                m3Vec3 vContact = m3Sub3(m3Add3(v0, m3Cross3(w0, hubArm)), vSurf);
                m3real vLon = m3Dot3(vContact, forward);
                world->vehicles.vehWheelLon[k] = vLon; // the diff's next-step read
                m3real vLat = m3Dot3(vContact, side);

                // Velocity kills use the solver's own effective
                // mass (linear plus angular response at the hub),
                // split by wheel count: four wheels each killing
                // the SHARED lateral velocity in full is a fourfold
                // overcorrection, and the hub's angular feedback
                // pushes the loop gain past one. The first probe
                // watched that pump grow a 1e-9 yaw seed into a
                // full sideways walk at 2.3x per step.
                m3Vec3 rxf = m3Cross3(hubArm, forward);
                m3real kLon = world->bodies.invMass[chassis] + m3Dot3(rxf, m3MulMV3(invI, rxf));
                m3real effLon = kLon > 0.0f ? 1.0f / kLon : 0.0f;
                m3Vec3 rxs = m3Cross3(hubArm, side);
                m3real kLat = world->bodies.invMass[chassis] + m3Dot3(rxs, m3MulMV3(invI, rxs));
                m3real effLat = kLat > 0.0f ? 1.0f / kLat : 0.0f;
                m3real share = 1.0f / (m3real)world->vehicles.vehWheelCount[slot];

                m3real lon = 0.0f;
                if ((world->vehicles.vehWheelFlags[k] & 2u) != 0)
                {
                    if (world->vehicles.vehDtActive[slot] != 0)
                    {
                        m3real driveForce = dtForce;
                        if (world->vehicles.vehDtDiffMode[slot] != 0)
                        {
                            m3real coupling = world->vehicles.vehDtDiffCouple[slot] *
                                              (dtMeanLon - world->vehicles.vehWheelLon[k]);
                            if (world->vehicles.vehDtDiffMode[slot] == 1)
                            {
                                // Limited slip: the coupling may not
                                // exceed the engine's own share.
                                m3real cap = m3AbsF(dtForce);
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
                        m3real trackThr = world->vehicles.vehWheelAnchor[k].z >= 0.0f
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
                    m3real budget = world->vehicles.vehBrake[slot] *
                                    world->vehicles.vehBrakeForce[slot] *
                                    world->vehicles.vehWheelBrake[k] * dt;
                    m3real want = -vLon * effLon * share;
                    lon += want > budget ? budget : (want < -budget ? -budget : want);
                }
                // The tire kills its share of lateral slip; the
                // friction circle decides how much survives (that
                // surrender is the drift).
                m3real lat = -vLat * effLat * share;
                m3real budget2 = world->vehicles.vehTireGrip[slot] * force * dt;
                m3real mag2 = lon * lon + lat * lat;
                if (mag2 > budget2 * budget2 && mag2 > 0.0f)
                {
                    m3real scale = budget2 / sqrtf(mag2);
                    lon *= scale;
                    lat *= scale;
                }
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
                m3Vec3 armU = {(m3real)(hit.point.x - world->bodies.transforms[under].p.x) - rlcU.x,
                               (m3real)(hit.point.y - world->bodies.transforms[under].p.y) - rlcU.y,
                               (m3real)(hit.point.z - world->bodies.transforms[under].p.z) -
                                   rlcU.z};
                m3Vec3 back = m3MulSV3(-1.0f, total);
                world->bodies.linearVelocities[under] =
                    m3Add3(world->bodies.linearVelocities[under],
                           m3MulSV3(world->bodies.invMass[under], back));
                world->bodies.angularVelocities[under] =
                    m3Add3(world->bodies.angularVelocities[under],
                           m3MulMV3(m3WorldInvInertia(world, under), m3Cross3(armU, back)));
                world->bodies.awake[under] = 1;
                world->bodies.sleepTimes[under] = 0.0f;
            }
        }
        for (int32_t a = 0; a < applied; ++a)
        {
            world->bodies.linearVelocities[chassis] =
                m3Add3(world->bodies.linearVelocities[chassis],
                       m3MulSV3(world->bodies.invMass[chassis], impulses[a]));
            world->bodies.angularVelocities[chassis] =
                m3Add3(world->bodies.angularVelocities[chassis],
                       m3MulMV3(invI, m3Cross3(arms[a], impulses[a])));
        }

        // The lean stabilizer: a two-wheeler is an inverted
        // pendulum, so an opt-in controller rolls the chassis toward
        // the lean the turn demands. It works in angular velocity
        // directly (inertia-free, the gain reads in 1/s^2) and only
        // about the forward axis, so it cannot mint yaw or pitch.
        // The target comes from the steer command, not the measured
        // yaw, because the command is journaled state and the
        // measurement would feed the controller its own noise.
        m3real leanGain = world->vehicles.vehLeanGain[slot];
        if (leanGain > 0.0f)
        {
            m3Vec3 fwd = m3RotateVec3(xf->q, (m3Vec3){1.0f, 0.0f, 0.0f});
            m3Vec3 up = m3RotateVec3(xf->q, (m3Vec3){0.0f, 1.0f, 0.0f});
            m3real g2 = m3Dot3(world->gravity, world->gravity);
            m3real gMag = g2 > 1.0e-6f ? sqrtf(g2) : 10.0f;
            m3Vec3 worldUp =
                g2 > 1.0e-6f ? m3MulSV3(-1.0f / gMag, world->gravity) : (m3Vec3){0.0f, 1.0f, 0.0f};
            m3real xMin = 0.0f;
            m3real xMax = 0.0f;
            for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
            {
                m3real ax = world->vehicles.vehWheelAnchor[slot * M3_VEHICLE_MAX_WHEELS + w].x;
                xMin = ax < xMin ? ax : xMin;
                xMax = ax > xMax ? ax : xMax;
            }
            m3real wheelbase = xMax - xMin > 0.1f ? xMax - xMin : 0.1f;
            m3real steerA = world->vehicles.vehSteer[slot] * world->vehicles.vehMaxSteer[slot];
            m3CosSin steerCs = m3ComputeCosSin(steerA);
            m3real tanSteer = steerCs.c > 0.1f ? steerCs.s / steerCs.c : 0.0f;
            m3real v = m3Dot3(world->bodies.linearVelocities[chassis], fwd);
            m3real latOverG = v * (v * tanSteer / wheelbase) / gMag;
            m3real sTarget = latOverG / sqrtf(1.0f + latOverG * latOverG);
            m3real sLean = m3Dot3(m3Cross3(up, worldUp), fwd);
            m3real rollRate = m3Dot3(world->bodies.angularVelocities[chassis], fwd);
            m3real damp = 2.0f * sqrtf(leanGain);
            m3real drr = (-leanGain * (sTarget - sLean) - damp * rollRate) * dt;
            // A bike leans about the CONTACT line, not its center:
            // a pure roll about the CG sweeps the hubs sideways and
            // the lateral tire kill vetoes it the same substep (the
            // first probe watched gain 25 lose to its own tires).
            // Rolling about the hub line means pairing the angular
            // change with its conjugate lateral velocity at the CG.
            m3real hubDrop = 0.0f;
            for (int32_t w = 0; w < world->vehicles.vehWheelCount[slot]; ++w)
            {
                int32_t k = slot * M3_VEHICLE_MAX_WHEELS + w;
                m3Vec3 hub = m3Add3(
                    world->vehicles.vehWheelAnchor[k],
                    m3MulSV3(world->vehicles.vehWheelRest[k], world->vehicles.vehWheelDir[k]));
                hubDrop -= hub.y;
            }
            hubDrop /= (m3real)world->vehicles.vehWheelCount[slot];
            m3Vec3 side = m3RotateVec3(xf->q, (m3Vec3){0.0f, 0.0f, 1.0f});
            world->bodies.angularVelocities[chassis] =
                m3Add3(world->bodies.angularVelocities[chassis], m3MulSV3(drr, fwd));
            world->bodies.linearVelocities[chassis] =
                m3Add3(world->bodies.linearVelocities[chassis], m3MulSV3(drr * hubDrop, side));
        }
    }
}

void m3VehicleTankCommandsInternal(m3World* world, int32_t slot, m3real left, m3real right,
                                   m3real brake)
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

void m3VehicleCommandsInternal(m3World* world, int32_t slot, m3real throttle, m3real steer,
                               m3real brake)
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

m3VehicleId m3CreateVehicle(m3WorldId worldId, const m3VehicleDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_VEHICLE_COOKIE)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullVehicleId; // field checks live in the internal
    }
    int32_t slot = m3CreateVehicleInternal(world, def);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullVehicleId;
    }
    m3VehicleId id = {slot + 1, world->idWorld, world->vehicles.vehPool.generations[slot]};
    if (world->recorder.journalActive != 0)
    {
        m3OpCreateVehicle record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateVehicle, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyVehicle(m3VehicleId vehicleId)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale: the quiet destroy contract
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyVehicle, &vehicleId, (int32_t)sizeof(vehicleId));
    }
    m3DestroyVehicleInternal(world, slot);
}

bool m3Vehicle_IsValid(m3VehicleId vehicleId)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    return world != NULL && m3VehicleSlot(world, vehicleId) >= 0;
}

m3real m3Vehicle_GetCompression(m3VehicleId vehicleId, int32_t wheel)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || wheel < 0 || wheel >= world->vehicles.vehWheelCount[slot])
    {
        m3Refuse(world, m3_errorInvalid);
        return 0.0f;
    }
    return world->vehicles.vehWheelCompression[slot * M3_VEHICLE_MAX_WHEELS + wheel];
}

void m3Vehicle_SetCommands(m3VehicleId vehicleId, m3real throttle, m3real steer, m3real brake)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || !m3FiniteF(throttle) || !m3FiniteF(steer) || !m3FiniteF(brake))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale id or hostile command: a documented no-op
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVehicleCommands record;
        memset(&record, 0, sizeof(record));
        record.id = vehicleId;
        record.throttle = throttle;
        record.steer = steer;
        record.brake = brake;
        m3JournalRecord(world, m3_opVehicleCommands, &record, (int32_t)sizeof(record));
    }
    m3VehicleCommandsInternal(world, slot, throttle, steer, brake);
}

void m3Vehicle_SetTankCommands(m3VehicleId vehicleId, m3real left, m3real right, m3real brake)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || !m3FiniteF(left) || !m3FiniteF(right) || !m3FiniteF(brake) ||
        world->vehicles.vehDtActive[slot] != 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // a gearbox and a skid steer are different machines
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVehicleTankCommands record;
        memset(&record, 0, sizeof(record));
        record.id = vehicleId;
        record.left = left;
        record.right = right;
        record.brake = brake;
        m3JournalRecord(world, m3_opVehicleTankCommands, &record, (int32_t)sizeof(record));
    }
    m3VehicleTankCommandsInternal(world, slot, left, right, brake);
}

m3real m3Vehicle_GetWheelSpin(m3VehicleId vehicleId, int32_t wheel)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || wheel < 0 || wheel >= world->vehicles.vehWheelCount[slot])
    {
        m3Refuse(world, m3_errorInvalid);
        return 0.0f;
    }
    return world->vehicles.vehWheelSpin[slot * M3_VEHICLE_MAX_WHEELS + wheel];
}

bool m3Vehicle_IsWheelGrounded(m3VehicleId vehicleId, int32_t wheel)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || wheel < 0 || wheel >= world->vehicles.vehWheelCount[slot])
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    return world->vehicles.vehWheelContact[slot * M3_VEHICLE_MAX_WHEELS + wheel] != 0;
}
