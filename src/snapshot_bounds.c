// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restore's magnitude checks. Every value a live record holds must be
// within the bound its create, setter and replay op keep (src/core.h),
// asked through the same predicates where they exist, so a restored world
// steps within the bounds a world the API built does. The block checks
// have made every float finite.
//
// Positions are bounded by M3_POSITION_LIMIT, where creates place every
// body and soft particle and the step holds it.

#include "snapshot_bounds.h"

#include "body.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "shape.h"
#include "vehicle.h"
#include "world.h"

#include "maul3d/softbody.h"

#include <stddef.h>
#include <string.h>

#define M3_IN(field) m3InField(inc, offsetof(m3World, field))

static m3Vec3 InV3(const uint8_t* block, int32_t i)
{
    m3Vec3 v;
    memcpy(&v, block + sizeof(m3Vec3) * (size_t)i, sizeof(v));
    return v;
}

static m3Pos3 InPos(const uint8_t* block, int32_t i)
{
    m3Pos3 p;
    memcpy(&p, block + sizeof(m3Pos3) * (size_t)i, sizeof(p));
    return p;
}

static bool InRange(float x, float lo, float hi)
{
    return x >= lo && x <= hi; // false for NaN
}

// --- The world ---------------------------------------------------------------

// The world's tuning as its setters took it, and the gust phase the step
// wraps back each time it passes a turn, which lands within a turn of it.
static bool WorldValid(const m3Incoming* inc)
{
    m3Vec3 gravity = InV3(M3_IN(gravity), 0);
    m3Vec3 windDir = InV3(M3_IN(windDir), 0);
    return m3GravityValid(gravity) &&
           m3ContactTuningValid(m3InF32(M3_IN(contactHertz), 0),
                                m3InF32(M3_IN(contactDampingRatio), 0),
                                m3InF32(M3_IN(contactPushMaxSpeed), 0)) &&
           m3InF32(M3_IN(restitutionThreshold), 0) >= 0.0f &&
           m3SpeedCapValid(m3InF32(M3_IN(maximumLinearSpeed), 0)) &&
           m3SpeedCapValid(m3InF32(M3_IN(maximumAngularSpeed), 0)) &&
           m3InF32(M3_IN(hitEventThreshold), 0) >= 0.0f &&
           m3WindValid(windDir, m3InF32(M3_IN(windSpeed), 0), m3InF32(M3_IN(windGustHertz), 0),
                       m3InF32(M3_IN(windGustScale), 0)) &&
           m3WithinF(m3InF32(M3_IN(windPhase), 0), 4.0f * M3_PI);
}

// --- Bodies and shapes ---------------------------------------------------------

// A live body's position within reach, its velocities by its type, its
// scales and damping, the force and torque it holds for the step, the
// time it has spent below its sleep threshold, and a servo target within
// reach of a kinematic body.
static bool BodiesValid(const m3Incoming* inc)
{
    const uint8_t* alive = M3_IN(bodies.bodyPool.alive);
    const uint8_t* types = M3_IN(bodies.types);
    const uint8_t* poses = M3_IN(bodies.transforms);
    const uint8_t* linear = M3_IN(bodies.linearVelocities);
    const uint8_t* angular = M3_IN(bodies.angularVelocities);
    const uint8_t* gravityScales = M3_IN(bodies.gravityScales);
    const uint8_t* linearDamping = M3_IN(bodies.linearDamping);
    const uint8_t* angularDamping = M3_IN(bodies.angularDamping);
    const uint8_t* forces = M3_IN(bodies.bodyForce);
    const uint8_t* torques = M3_IN(bodies.bodyTorque);
    const uint8_t* sleepThresholds = M3_IN(bodies.bodySleepThreshold);
    const uint8_t* sleepTimes = M3_IN(bodies.sleepTimes);
    const uint8_t* hasTarget = M3_IN(bodies.bodyHasTarget);
    const uint8_t* targets = M3_IN(bodies.bodyTarget);
    for (int32_t b = 0; b < inc->world->bodies.bodyPool.capacity; ++b)
    {
        if (alive[b] == 0)
        {
            continue;
        }
        m3Transform pose;
        memcpy(&pose, poses + sizeof(m3Transform) * (size_t)b, sizeof(pose));
        if (!m3BoundedPos3(pose.p) || !m3BodyVelocityValid(types[b], InV3(linear, b)) ||
            !m3BodySpinValid(InV3(angular, b)) ||
            !m3BodyParamValid(m3_bodyParamGravityScale, m3InF32(gravityScales, b)) ||
            !m3BodyParamValid(m3_bodyParamLinearDamping, m3InF32(linearDamping, b)) ||
            !m3BodyParamValid(m3_bodyParamAngularDamping, m3InF32(angularDamping, b)) ||
            !m3WithinV3(InV3(forces, b), M3_FORCE_LIMIT) ||
            !m3WithinV3(InV3(torques, b), M3_FORCE_LIMIT) || m3InF32(sleepThresholds, b) < 0.0f ||
            m3InF32(sleepTimes, b) < 0.0f)
        {
            return false;
        }
        if (hasTarget[b] != 0)
        {
            m3Transform target;
            memcpy(&target, targets + sizeof(m3Transform) * (size_t)b, sizeof(target));
            if (!m3TargetFits(types[b], pose.p, target))
            {
                return false;
            }
        }
    }
    return true;
}

// A live shape's material and offset as its def or setters took them.
static bool ShapesValid(const m3Incoming* inc)
{
    const uint8_t* alive = M3_IN(shapes.shapePool.alive);
    const uint8_t* density = M3_IN(shapes.shapeDensity);
    const uint8_t* friction = M3_IN(shapes.shapeFriction);
    const uint8_t* restitution = M3_IN(shapes.shapeRestitution);
    const uint8_t* rolling = M3_IN(shapes.shapeRollingResistance);
    const uint8_t* surface = M3_IN(shapes.shapeSurfaceVel);
    const uint8_t* offsets = M3_IN(shapes.shapeLocalPos);
    for (int32_t s = 0; s < inc->world->shapes.shapePool.capacity; ++s)
    {
        if (alive[s] != 0 &&
            (!m3DensityF(m3InF32(density, s)) ||
             !m3ShapeScalarValid(m3_opSetShapeFriction, m3InF32(friction, s)) ||
             !m3ShapeScalarValid(m3_opSetShapeRestitution, m3InF32(restitution, s)) ||
             !m3ShapeScalarValid(m3_opSetShapeRolling, m3InF32(rolling, s)) ||
             !m3SurfaceVelocityValid(InV3(surface, s)) || !m3LocalV3(InV3(offsets, s))))
        {
            return false;
        }
    }
    return true;
}

// --- Joints --------------------------------------------------------------------

typedef struct JointFields
{
    const uint8_t* types;
    const uint8_t* flags;
    const uint8_t* motors;
    const uint8_t* limits;
    const uint8_t* springs;
    const uint8_t* targetQ;
    const uint8_t* breaks;
    const uint8_t* groundA;
    const uint8_t* groundB;
} JointFields;

// A joint's drive by its type: a spring as its setter took it (all zeros
// until first set, then kept when switched off), and the motor, limit and
// steering slots each type uses, the gear, pulley and motor joint reusing
// them for their own values.
static bool JointDriveValid(const JointFields* f, int32_t j)
{
    uint8_t type = f->types[j];
    uint8_t flags = f->flags[j];
    m3Vec3 motor = InV3(f->motors, j);
    m3Vec3 limits = InV3(f->limits, j);
    m3Vec3 spring = InV3(f->springs, j);
    bool springNeverSet = (flags & M3_JOINT_SPRING) == 0 && spring.x == 0.0f && spring.y == 0.0f;
    if (!springNeverSet && !m3JointSpringFits(type, spring.x, spring.y))
    {
        return false;
    }
    if (type == (uint8_t)m3_gearJoint)
    {
        return m3WithinF(motor.z, M3_RATIO_LIMIT) && motor.z != 0.0f;
    }
    if (type == (uint8_t)m3_motorJoint)
    {
        return m3JointLimitsValid(type, limits.x, limits.y); // the motor slot holds the aim
    }
    if (!m3JointMotorValid(type, (flags & M3_JOINT_MOTOR) != 0, motor.x, motor.y))
    {
        return false;
    }
    if (type == (uint8_t)m3_pulleyJoint)
    {
        return motor.z > 0.0f && motor.z <= M3_RATIO_LIMIT && m3BoundedPos3(InPos(f->groundA, j)) &&
               m3BoundedPos3(InPos(f->groundB, j));
    }
    if (type == (uint8_t)m3_wheelJoint)
    {
        m3Quat steer;
        memcpy(&steer, f->targetQ + sizeof(m3Quat) * (size_t)j, sizeof(steer));
        return m3JointSteerFits(type, (flags & M3_JOINT_STEER) != 0, motor.z, steer.x, steer.y,
                                steer.z);
    }
    return true;
}

// A joint's target as its setter took it: an angle within a half turn on
// a revolute joint, any travel on a prismatic or wheel joint, and none on
// a type that takes no scalar target.
static bool TargetScalarFits(uint8_t type, float scalar)
{
    if (type == (uint8_t)m3_revoluteJoint)
    {
        return scalar >= -M3_PI && scalar <= M3_PI;
    }
    return type == (uint8_t)m3_prismaticJoint || type == (uint8_t)m3_wheelJoint || scalar == 0.0f;
}

// A generic joint's axis ranges as its def took them: on each limited
// axis of the three whose modes start at bit shift of the packed modes,
// the lower bound at most the upper. Every other joint packs no mode.
static bool GenericRangesFit(uint32_t modes, int32_t shift, m3Vec3 lower, m3Vec3 upper)
{
    const float lo[3] = {lower.x, lower.y, lower.z};
    const float hi[3] = {upper.x, upper.y, upper.z};
    for (int32_t k = 0; k < 3; ++k)
    {
        if (((modes >> (shift + 2 * k)) & 3u) == (uint32_t)m3_axisLimited && !(lo[k] <= hi[k]))
        {
            return false;
        }
    }
    return true;
}

// A live joint's anchors in its bodies' frames, its break thresholds, its
// target, its generic ranges and its drive.
static bool JointsValid(const m3Incoming* inc)
{
    const uint8_t* alive = M3_IN(joints.jointPool.alive);
    const uint8_t* localA = M3_IN(joints.jointLocalA);
    const uint8_t* localB = M3_IN(joints.jointLocalB);
    const uint8_t* targets = M3_IN(joints.jointTargetScalar);
    const uint8_t* modes = M3_IN(joints.jointGenericModes);
    const uint8_t* linLower = M3_IN(joints.jointGenLinLower);
    const uint8_t* linUpper = M3_IN(joints.jointGenLinUpper);
    const uint8_t* angLower = M3_IN(joints.jointGenAngLower);
    const uint8_t* angUpper = M3_IN(joints.jointGenAngUpper);
    JointFields f = {
        M3_IN(joints.jointType),   M3_IN(joints.jointFlags),   M3_IN(joints.jointMotor),
        M3_IN(joints.jointLimits), M3_IN(joints.jointSpring),  M3_IN(joints.jointTargetQ),
        M3_IN(joints.jointBreak),  M3_IN(joints.jointGroundA), M3_IN(joints.jointGroundB)};
    for (int32_t j = 0; j < inc->world->joints.jointPool.capacity; ++j)
    {
        if (alive[j] == 0)
        {
            continue;
        }
        m3Vec3 thresholds = InV3(f.breaks, j);
        if (!m3LocalV3(InV3(localA, j)) || !m3LocalV3(InV3(localB, j)) || thresholds.x < 0.0f ||
            thresholds.y < 0.0f || !TargetScalarFits(f.types[j], m3InF32(targets, j)) ||
            !GenericRangesFit(m3InU16(modes, j), 0, InV3(linLower, j), InV3(linUpper, j)) ||
            !GenericRangesFit(m3InU16(modes, j), 6, InV3(angLower, j), InV3(angUpper, j)) ||
            !JointDriveValid(&f, j))
        {
            return false;
        }
    }
    return true;
}

// --- Characters, vehicles, soft bodies and water -------------------------------

// A live character's capsule and walker within the bounds its create and
// stance changes keep: the cosine of a slope limit of at most 1.5 rad,
// and a ground normal that is none or a contact's unit normal.
static bool CharactersValid(const m3Incoming* inc)
{
    const uint8_t* alive = M3_IN(characters.charPool.alive);
    const uint8_t* radius = M3_IN(characters.charRadius);
    const uint8_t* halfHeight = M3_IN(characters.charHalfHeight);
    const uint8_t* mass = M3_IN(characters.charMass);
    const uint8_t* pushMax = M3_IN(characters.charPushMax);
    const uint8_t* cosSlope = M3_IN(characters.charCosSlope);
    const uint8_t* ground = M3_IN(characters.charGroundNormal);
    for (int32_t c = 0; c < inc->world->characters.charPool.capacity; ++c)
    {
        m3Vec3 n = InV3(ground, c);
        if (alive[c] != 0 &&
            (!(m3InF32(radius, c) > 0.0f && m3LocalF(m3InF32(radius, c))) ||
             !InRange(m3InF32(halfHeight, c), 0.0f, M3_LOCAL_LIMIT) ||
             !InRange(m3InF32(mass, c), M3_MASS_FLOOR, M3_MASS_LIMIT) ||
             !InRange(m3InF32(pushMax, c), 0.0f, M3_RATIO_LIMIT) ||
             !InRange(m3InF32(cosSlope, c), 0.0f, 1.0f) || !(m3Dot3(n, n) <= 1.01f)))
        {
            return false;
        }
    }
    return true;
}

typedef struct VehicleFields
{
    const uint8_t* wheelCount;
    const uint8_t* anchor;
    const uint8_t* dir;
    const uint8_t* rest;
    const uint8_t* travel;
    const uint8_t* hertz;
    const uint8_t* zeta;
    const uint8_t* radius;
    const uint8_t* brakeShare;
    const uint8_t* lon;
    const uint8_t* compression;
} VehicleFields;

// Vehicle v's wheels as their def kept them, the compression each held at
// the last step, within its rest length, and the contact speed each
// carries to the next step's differential: a point on the chassis against
// one on a kinematic body, within twice the kinematic limit.
static bool WheelsValid(const VehicleFields* f, int32_t v)
{
    for (int32_t w = 0; w < m3InI32(f->wheelCount, v); ++w)
    {
        int32_t k = v * M3_VEHICLE_MAX_WHEELS + w;
        m3WheelDef wheel;
        memset(&wheel, 0, sizeof(wheel));
        wheel.anchor = InV3(f->anchor, k);
        wheel.direction = InV3(f->dir, k);
        wheel.restLength = m3InF32(f->rest, k);
        wheel.travel = m3InF32(f->travel, k);
        wheel.hertz = m3InF32(f->hertz, k);
        wheel.zeta = m3InF32(f->zeta, k);
        wheel.radius = m3InF32(f->radius, k);
        wheel.brakeShare = m3InF32(f->brakeShare, k);
        if (!m3WheelDefValid(&wheel) ||
            !InRange(m3InF32(f->compression, k), 0.0f, wheel.restLength) ||
            !m3WithinF(m3InF32(f->lon, k), 2.0f * M3_KINEMATIC_SPEED_LIMIT))
        {
            return false;
        }
    }
    return true;
}

// Vehicle v's drivetrain, when it has one, as its setter took it.
static bool DrivetrainValid(const m3Incoming* inc, int32_t v)
{
    if (M3_IN(vehicles.vehDtActive)[v] == 0)
    {
        return true;
    }
    m3DrivetrainDef def;
    memset(&def, 0, sizeof(def));
    def.curveCount = m3InI32(M3_IN(vehicles.vehDtCurveCount), v);
    for (int32_t c = 0; c < M3_DRIVETRAIN_MAX_CURVE; ++c)
    {
        def.curveRpm[c] = m3InF32(M3_IN(vehicles.vehDtCurveRpm), v * M3_DRIVETRAIN_MAX_CURVE + c);
        def.curveTorque[c] =
            m3InF32(M3_IN(vehicles.vehDtCurveTorque), v * M3_DRIVETRAIN_MAX_CURVE + c);
    }
    def.gearCount = m3InI32(M3_IN(vehicles.vehDtGearCount), v);
    for (int32_t g = 0; g < M3_DRIVETRAIN_MAX_GEARS; ++g)
    {
        def.gearRatio[g] = m3InF32(M3_IN(vehicles.vehDtGearRatio), v * M3_DRIVETRAIN_MAX_GEARS + g);
    }
    def.reverseRatio = m3InF32(M3_IN(vehicles.vehDtReverse), v);
    def.finalDrive = m3InF32(M3_IN(vehicles.vehDtFinal), v);
    def.shiftUpRpm = m3InF32(M3_IN(vehicles.vehDtShiftUp), v);
    def.shiftDownRpm = m3InF32(M3_IN(vehicles.vehDtShiftDown), v);
    def.clutchSteps = m3InI32(M3_IN(vehicles.vehDtClutchSteps), v);
    def.diffMode = m3InI32(M3_IN(vehicles.vehDtDiffMode), v);
    def.diffCouple = m3InF32(M3_IN(vehicles.vehDtDiffCouple), v);
    return m3DrivetrainDefValid(&def);
}

// A live vehicle's forces and grips as its create took them, its commands
// as their setters clamp them, its wheels and its drivetrain.
static bool VehiclesValid(const m3Incoming* inc)
{
    const uint8_t* alive = M3_IN(vehicles.vehPool.alive);
    VehicleFields f = {M3_IN(vehicles.vehWheelCount),      M3_IN(vehicles.vehWheelAnchor),
                       M3_IN(vehicles.vehWheelDir),        M3_IN(vehicles.vehWheelRest),
                       M3_IN(vehicles.vehWheelTravel),     M3_IN(vehicles.vehWheelHertz),
                       M3_IN(vehicles.vehWheelZeta),       M3_IN(vehicles.vehWheelRadius),
                       M3_IN(vehicles.vehWheelBrake),      M3_IN(vehicles.vehWheelLon),
                       M3_IN(vehicles.vehWheelCompression)};
    for (int32_t v = 0; v < inc->world->vehicles.vehPool.capacity; ++v)
    {
        if (alive[v] == 0)
        {
            continue;
        }
        if (m3InF32(M3_IN(vehicles.vehMaxSteer), v) < 0.0f ||
            !InRange(m3InF32(M3_IN(vehicles.vehDriveForce), v), 0.0f, M3_FORCE_LIMIT) ||
            !InRange(m3InF32(M3_IN(vehicles.vehBrakeForce), v), 0.0f, M3_FORCE_LIMIT) ||
            !InRange(m3InF32(M3_IN(vehicles.vehTireGrip), v), 0.0f, M3_RATIO_LIMIT) ||
            !InRange(m3InF32(M3_IN(vehicles.vehLeanGain), v), 0.0f, M3_RATIO_LIMIT) ||
            !m3WithinF(m3InF32(M3_IN(vehicles.vehThrottle), v), 1.0f) ||
            !m3WithinF(m3InF32(M3_IN(vehicles.vehSteer), v), 1.0f) ||
            !InRange(m3InF32(M3_IN(vehicles.vehBrake), v), 0.0f, 1.0f) ||
            !m3WithinF(m3InF32(M3_IN(vehicles.vehTrackLeft), v), 1.0f) ||
            !m3WithinF(m3InF32(M3_IN(vehicles.vehTrackRight), v), 1.0f) || !WheelsValid(&f, v) ||
            !DrivetrainValid(inc, v))
        {
            return false;
        }
    }
    return true;
}

// Soft body s's rest values as its create took them: each edge's length
// and each tet's volume within what its points allow, and the lattice's
// closed volume.
static bool SoftRestValid(const m3Incoming* inc, int32_t s)
{
    const uint8_t* edgeRest = M3_IN(softBodies.softEdgeRest);
    const uint8_t* tetRest = M3_IN(softBodies.softTetRestV6);
    if (!InRange(m3InF32(M3_IN(softBodies.softRestVolume), s), 0.0f, M3_SOFT_SHAPE_LIMIT))
    {
        return false;
    }
    for (int32_t e = 0; e < m3InI32(M3_IN(softBodies.softEdgeCount), s); ++e)
    {
        if (!InRange(m3InF32(edgeRest, s * M3_SOFTBODY_MAX_EDGES + e), 0.0f, M3_SOFT_EDGE_LIMIT))
        {
            return false;
        }
    }
    for (int32_t t = 0; t < m3InI32(M3_IN(softBodies.softTetCount), s); ++t)
    {
        if (!m3WithinF(m3InF32(tetRest, s * M3_SOFTBODY_MAX_TETS + t), M3_SOFT_TET_LIMIT))
        {
            return false;
        }
    }
    return true;
}

// A live soft body's particle scalars as its create took them: each
// particle within M3_POSITION_LIMIT, where its create placed it and the
// step holds it, now, a substep ago and in its bind pose, its
// inverse mass that of a mass within the floor and ceiling, or zero when
// pinned, and a pending kick within the speed limit the step held it to;
// its rest values; its anchors in their bodies' frames.
static bool SoftBodiesValid(const m3Incoming* inc)
{
    const uint8_t* alive = M3_IN(softBodies.softPool.alive);
    const uint8_t* counts = M3_IN(softBodies.softParticleCount);
    const uint8_t* positions = M3_IN(softBodies.softPos);
    const uint8_t* previous = M3_IN(softBodies.softPrev);
    const uint8_t* binds = M3_IN(softBodies.softBindPos);
    const uint8_t* invMass = M3_IN(softBodies.softInvMass);
    const uint8_t* kicks = M3_IN(softBodies.softKick);
    const uint8_t* anchorCount = M3_IN(softBodies.softAnchorCount);
    const uint8_t* anchorLocal = M3_IN(softBodies.softAnchorLocal);
    for (int32_t s = 0; s < inc->world->softBodies.softPool.capacity; ++s)
    {
        if (alive[s] == 0)
        {
            continue;
        }
        float radius = m3InF32(M3_IN(softBodies.softRadius), s);
        if (!(radius > 0.0f && m3LocalF(radius)) ||
            !m3WithinF(m3InF32(M3_IN(softBodies.softGravityScale), s), M3_RATIO_LIMIT) ||
            m3InF32(M3_IN(softBodies.softCompliance), s) < 0.0f ||
            m3InF32(M3_IN(softBodies.softBendCompliance), s) < 0.0f ||
            m3InF32(M3_IN(softBodies.softMaxDeviation), s) < 0.0f ||
            m3InF32(M3_IN(softBodies.softPressure), s) > M3_SOFTBODY_MAX_PRESSURE ||
            !SoftRestValid(inc, s))
        {
            return false;
        }
        for (int32_t i = 0; i < m3InI32(counts, s); ++i)
        {
            int32_t k = s * M3_SOFTBODY_MAX_PARTICLES + i;
            float w = m3InF32(invMass, k);
            if (!m3BoundedPos3(InPos(positions, k)) || !m3BoundedPos3(InPos(previous, k)) ||
                !m3BoundedPos3(InPos(binds, k)) ||
                (w != 0.0f && !InRange(w, 1.0f / M3_MASS_LIMIT, 1.0f / M3_MASS_FLOOR)) ||
                !m3WithinV3(InV3(kicks, k), M3_SPEED_LIMIT))
            {
                return false;
            }
        }
        for (int32_t a = 0; a < m3InI32(anchorCount, s); ++a)
        {
            if (!m3LocalV3(InV3(anchorLocal, s * M3_SOFTBODY_MAX_ANCHORS + a)))
            {
                return false;
            }
        }
    }
    return true;
}

// A live water volume as its create took it.
static bool WaterValid(const m3Incoming* inc)
{
    const uint8_t* alive = M3_IN(water.waterPool.alive);
    for (int32_t v = 0; v < inc->world->water.waterPool.capacity; ++v)
    {
        if (alive[v] == 0)
        {
            continue;
        }
        m3Pos3 lo = InPos(M3_IN(water.waterLo), v);
        m3Pos3 hi = InPos(M3_IN(water.waterHi), v);
        if (!m3BoundedPos3(lo) || !m3BoundedPos3(hi) || !(hi.x > lo.x) || !(hi.y > lo.y) ||
            !(hi.z > lo.z) || !m3DensityF(m3InF32(M3_IN(water.waterDensity), v)) ||
            m3InF32(M3_IN(water.waterLinDrag), v) < 0.0f ||
            m3InF32(M3_IN(water.waterAngDrag), v) < 0.0f ||
            !m3WithinV3(InV3(M3_IN(water.waterFlow), v), M3_SPEED_LIMIT))
        {
            return false;
        }
    }
    return true;
}

bool m3SnapshotBoundsValid(const m3Incoming* inc)
{
    return WorldValid(inc) && BodiesValid(inc) && ShapesValid(inc) && JointsValid(inc) &&
           CharactersValid(inc) && VehiclesValid(inc) && SoftBodiesValid(inc) && WaterValid(inc);
}
