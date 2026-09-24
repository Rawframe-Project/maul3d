// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint lifecycle: the same law as bodies and shapes. Public
// functions validate and journal, internal functions mutate, replay
// drives the internals and verifies minted ids. The warm-start
// impulse lives in the persistent arena and rides the snapshot.

#include "joint.h"
#include "body.h"
#include "joint_solver.h"
#include "journal.h"
#include "manifold.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

#define M3_JOINT_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3JointDef) << 8) ^ 3))

m3JointDef m3DefaultJointDef(void)
{
    m3JointDef def;
    memset(&def, 0, sizeof(def));
    def.type = m3_sphericalJoint;
    def.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    def.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    def.genericMotorAxis = 255; // no motor unless chosen
    def.ratio = 1.0f;
    def.internalValue = M3_JOINT_COOKIE;
    return def;
}

// Quaternion from an orthonormal right-handed column basis
// (deterministic: Shepperd's branch on the largest diagonal).
static m3Quat QuatFromBasis(m3Vec3 t1, m3Vec3 t2, m3Vec3 axis)
{
    m3real m00 = t1.x, m01 = t2.x, m02 = axis.x;
    m3real m10 = t1.y, m11 = t2.y, m12 = axis.y;
    m3real m20 = t1.z, m21 = t2.z, m22 = axis.z;
    m3Quat q;
    m3real trace = m00 + m11 + m22;
    if (trace > 0.0f)
    {
        m3real s = sqrtf(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    }
    else if (m00 > m11 && m00 > m22)
    {
        m3real s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m21 - m12) / s;
        q.x = 0.25f * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
        m3real s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25f * s;
        q.z = (m12 + m21) / s;
    }
    else
    {
        m3real s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25f * s;
    }
    return m3NormalizeQuat(q);
}

// Quaternion whose z-axis is the given unit axis, with the tangent
// basis rule fixing the other two columns.
static m3Quat QuatFromAxisZ(m3Vec3 axis)
{
    m3Vec3 t1;
    m3Vec3 t2;
    m3MakeTangentBasis(axis, &t1, &t2);
    return QuatFromBasis(t1, t2, axis);
}

int32_t m3JointSlot(const m3World* world, m3JointId jointId)
{
    int32_t index = jointId.index1 - 1;
    if (world == NULL || jointId.world != world->idWorld ||
        !m3IdPoolValid(&world->joints.jointPool, index, jointId.generation))
    {
        return -1;
    }
    return index;
}

// Joint geometry derived from the bodies' create poses before a slot is
// taken, so a refused geometry leaks nothing.
typedef struct JointBake
{
    m3real pulleyLen1;
    m3real pulleyLen2;
    m3Quat wheelQA;
    m3Quat wheelQB;
} JointBake;

// A gear needs real axes and a real ratio.
static bool GearDefValid(const m3JointDef* def)
{
    return m3FiniteV3(def->localAxisA) && m3FiniteV3(def->localAxisB) &&
           m3Dot3(def->localAxisA, def->localAxisA) > 1.0e-8f &&
           m3Dot3(def->localAxisB, def->localAxisB) > 1.0e-8f && m3FiniteF(def->ratio) &&
           def->ratio != 0.0f;
}

// The rope lengths from each anchor to its ground pulley at create. A
// rope end on its pulley has no direction and refuses.
static bool BakePulley(const m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB,
                       JointBake* bake)
{
    if (!m3FinitePos3(def->groundAnchorA) || !m3FinitePos3(def->groundAnchorB) ||
        !m3FiniteF(def->ratio) || !(def->ratio > 0.0f) || !m3FiniteV3(def->localAnchorA) ||
        !m3FiniteV3(def->localAnchorB))
    {
        return false;
    }
    const m3Transform* xfA = &world->bodies.transforms[bodyA];
    const m3Transform* xfB = &world->bodies.transforms[bodyB];
    m3Vec3 aA = m3RotateVec3(xfA->q, def->localAnchorA);
    m3Vec3 aB = m3RotateVec3(xfB->q, def->localAnchorB);
    m3Vec3 u1 = {(m3real)(xfA->p.x + (double)aA.x - def->groundAnchorA.x),
                 (m3real)(xfA->p.y + (double)aA.y - def->groundAnchorA.y),
                 (m3real)(xfA->p.z + (double)aA.z - def->groundAnchorA.z)};
    m3Vec3 u2 = {(m3real)(xfB->p.x + (double)aB.x - def->groundAnchorB.x),
                 (m3real)(xfB->p.y + (double)aB.y - def->groundAnchorB.y),
                 (m3real)(xfB->p.z + (double)aB.z - def->groundAnchorB.z)};
    bake->pulleyLen1 = m3Length3(u1);
    bake->pulleyLen2 = m3Length3(u2);
    return bake->pulleyLen1 > 1.0e-3f && bake->pulleyLen2 > 1.0e-3f;
}

// The wheel frame: x is the suspension axis (from A), z the axle
// captured from B's world image and snapped exactly perpendicular. An
// axle along the strut is not a wheel and refuses.
static bool BakeWheel(const m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB,
                      JointBake* bake)
{
    const m3Transform* xfA = &world->bodies.transforms[bodyA];
    const m3Transform* xfB = &world->bodies.transforms[bodyB];
    m3Vec3 susp = m3Normalize3(def->localAxisA);
    m3Vec3 axleWorld = m3RotateVec3(xfB->q, m3Normalize3(def->localAxisB));
    m3Quat conjA = {-xfA->q.x, -xfA->q.y, -xfA->q.z, xfA->q.w};
    m3Vec3 axleA = m3RotateVec3(conjA, axleWorld);
    m3real skew = m3Dot3(axleA, susp);
    if (skew > 0.1f || skew < -0.1f)
    {
        return false;
    }
    m3Vec3 z = m3Normalize3(m3Sub3(axleA, m3MulSV3(skew, susp)));
    m3Vec3 y = m3Cross3(z, susp); // x cross y = z, right-handed
    bake->wheelQA = QuatFromBasis(susp, y, z);
    m3Quat conjB = {-xfB->q.x, -xfB->q.y, -xfB->q.z, xfB->q.w};
    bake->wheelQB = m3NormalizeQuat(m3MulQuat(conjB, m3MulQuat(xfA->q, bake->wheelQA)));
    return true;
}

// The type whitelist and the geometry walls live here, not only in the
// public wall, because replay hands m3CreateJointInternal raw journal
// bytes: a flipped type byte must refuse instead of minting a joint no
// solver branch owns.
static bool BakeJointDef(const m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB,
                         JointBake* bake)
{
    bake->pulleyLen1 = 0.0f;
    bake->pulleyLen2 = 0.0f;
    bake->wheelQA = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    bake->wheelQB = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    switch (def->type)
    {
    case m3_gearJoint:
        return GearDefValid(def);
    case m3_pulleyJoint:
        return BakePulley(world, def, bodyA, bodyB, bake);
    case m3_wheelJoint:
        return BakeWheel(world, def, bodyA, bodyB, bake);
    default:
        return def->type >= 0 && def->type <= (int32_t)m3_pulleyJoint;
    }
}

// The joint frames. A weld or servo stores frames that coincide at the
// create pose (frameA identity, frameB = conj(qB0) * qA0), so the
// rotation lock drives the live relative rotation back to identity.
// Distance, pulley and filter joints use no frames and keep identity.
static void InitJointFrames(m3World* world, int32_t index, const m3JointDef* def,
                            const JointBake* bake)
{
    m3Joints* j = &world->joints;
    m3Quat identity = {0.0f, 0.0f, 0.0f, 1.0f};
    j->jointFrameQA[index] = identity;
    j->jointFrameQB[index] = identity;
    switch (def->type)
    {
    case m3_fixedJoint:
    case m3_motorJoint:
    {
        const m3Transform* xfA = &world->bodies.transforms[j->jointBodyA[index]];
        const m3Transform* xfB = &world->bodies.transforms[j->jointBodyB[index]];
        m3Quat conjB = {-xfB->q.x, -xfB->q.y, -xfB->q.z, xfB->q.w};
        j->jointFrameQB[index] = m3NormalizeQuat(m3MulQuat(conjB, xfA->q));
        break;
    }
    case m3_distanceJoint:
    case m3_pulleyJoint:
    case m3_filterJoint:
        break;
    case m3_wheelJoint:
        // The spin angle and the collinearity error both start at zero.
        j->jointFrameQA[index] = bake->wheelQA;
        j->jointFrameQB[index] = bake->wheelQB;
        break;
    default:
        j->jointFrameQA[index] = QuatFromAxisZ(m3Normalize3(def->localAxisA));
        j->jointFrameQB[index] = QuatFromAxisZ(m3Normalize3(def->localAxisB));
        break;
    }
}

// The servo's default aim is the create pose: the anchor gap in A's
// frame, so a fresh servo holds where it was built.
static m3Vec3 ServoCreateGap(const m3World* world, const m3JointDef* def, int32_t bodyA,
                             int32_t bodyB)
{
    const m3Transform* xfA = &world->bodies.transforms[bodyA];
    const m3Transform* xfB = &world->bodies.transforms[bodyB];
    m3Vec3 aA = m3RotateVec3(xfA->q, def->localAnchorA);
    m3Vec3 aB = m3RotateVec3(xfB->q, def->localAnchorB);
    m3Vec3 gap = {(m3real)(xfB->p.x + (double)aB.x - xfA->p.x - (double)aA.x),
                  (m3real)(xfB->p.y + (double)aB.y - xfA->p.y - (double)aA.y),
                  (m3real)(xfB->p.z + (double)aB.z - xfA->p.z - (double)aA.z)};
    m3Quat conjA = {-xfA->q.x, -xfA->q.y, -xfA->q.z, xfA->q.w};
    return m3RotateVec3(conjA, gap);
}

// The motor, limit and ground slots. The spherical keeps its cone angle
// in limits.z. The gear and the pulley reuse the motor and limit slots
// and are written after the generic values on purpose.
static void InitJointSlots(m3World* world, int32_t index, const m3JointDef* def,
                           const JointBake* bake)
{
    m3Joints* j = &world->joints;
    int32_t bodyA = j->jointBodyA[index];
    int32_t bodyB = j->jointBodyB[index];
    j->jointMotor[index] = (m3Vec3){def->motorSpeed, def->maxMotorEffort, 0.0f};
    if (def->type == (int32_t)m3_motorJoint)
    {
        j->jointMotor[index] = ServoCreateGap(world, def, bodyA, bodyB);
    }
    j->jointGroundA[index] = (m3Pos3){0.0, 0.0, 0.0};
    j->jointGroundB[index] = (m3Pos3){0.0, 0.0, 0.0};
    j->jointLimits[index] = (m3Vec3){def->lowerLimit, def->upperLimit, def->coneAngle};
    if (def->type == (int32_t)m3_gearJoint)
    {
        // Each body's spin about its own gear axis at create, and the
        // ratio: the solver holds (phiA - phiA0) + ratio * (phiB - phiB0).
        j->jointMotor[index] = (m3Vec3){
            m3GearSpin(world->bodies.transforms[bodyA].q, j->jointFrameQA[index]),
            m3GearSpin(world->bodies.transforms[bodyB].q, j->jointFrameQB[index]), def->ratio};
    }
    else if (def->type == (int32_t)m3_pulleyJoint)
    {
        // The rope law: length1 + ratio * length2 at create is the constant.
        j->jointGroundA[index] = def->groundAnchorA;
        j->jointGroundB[index] = def->groundAnchorB;
        j->jointMotor[index] = (m3Vec3){0.0f, 0.0f, def->ratio};
        j->jointLimits[index].z = bake->pulleyLen1 + def->ratio * bake->pulleyLen2;
    }
}

// Generic joint modes pack two bits per axis: linear in bits 0..5,
// angular in bits 6..11, the motor axis in bits 12..15.
static void InitGenericJoint(m3World* world, int32_t index, const m3JointDef* def)
{
    m3Joints* j = &world->joints;
    uint16_t packed = 0;
    for (int32_t k = 0; k < 3; ++k)
    {
        packed |= (uint16_t)(def->genericLinear[k] & 3) << (2 * k);
        packed |= (uint16_t)(def->genericAngular[k] & 3) << (6 + 2 * k);
    }
    packed |= (uint16_t)(def->genericMotorAxis == 255 ? 15 : def->genericMotorAxis) << 12;
    j->jointGenericModes[index] = packed;
    j->jointGenLinLower[index] = (m3Vec3){def->genericLinearLower[0], def->genericLinearLower[1],
                                          def->genericLinearLower[2]};
    j->jointGenLinUpper[index] = (m3Vec3){def->genericLinearUpper[0], def->genericLinearUpper[1],
                                          def->genericLinearUpper[2]};
    j->jointGenAngLower[index] = (m3Vec3){def->genericAngularLower[0], def->genericAngularLower[1],
                                          def->genericAngularLower[2]};
    j->jointGenAngUpper[index] = (m3Vec3){def->genericAngularUpper[0], def->genericAngularUpper[1],
                                          def->genericAngularUpper[2]};
}

int32_t m3CreateJointInternal(m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB)
{
    JointBake bake;
    if (!BakeJointDef(world, def, bodyA, bodyB, &bake))
    {
        return -1;
    }
    int32_t index = m3IdPoolAlloc(&world->joints.jointPool);
    if (index < 0)
    {
        return -1; // exhausted: loud at the caller
    }
    m3Joints* j = &world->joints;
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    j->jointType[index] = (uint8_t)def->type;
    j->jointBodyA[index] = bodyA;
    j->jointBodyB[index] = bodyB;
    j->jointLocalA[index] = def->localAnchorA;
    j->jointLocalB[index] = def->localAnchorB;
    j->jointCollide[index] = def->collideConnected ? 1 : 0;
    j->jointImpulse[index] = zero;
    j->jointPerpImpulse[index] = zero;
    j->jointLimitImpulse[index] = zero;
    j->jointAngularImpulse[index] = zero;
    j->jointFlags[index] = (uint8_t)((def->enableLimit ? M3_JOINT_LIMIT : 0u) |
                                     (def->enableMotor ? M3_JOINT_MOTOR : 0u) |
                                     (def->enableCone ? M3_JOINT_CONE : 0u));
    j->jointBreak[index] = zero; // unbreakable
    j->jointSpring[index] = zero;
    j->jointTargetScalar[index] = 0.0f;
    j->jointTargetQ[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    j->jointSpringImpulse[index] = zero;
    j->jointGenericModes[index] = 0;
    j->jointGenLinLower[index] = zero;
    j->jointGenLinUpper[index] = zero;
    j->jointGenAngLower[index] = zero;
    j->jointGenAngUpper[index] = zero;
    InitJointFrames(world, index, def, &bake);
    InitJointSlots(world, index, def, &bake);
    if (def->type == (int32_t)m3_genericJoint)
    {
        InitGenericJoint(world, index, def);
    }
    // Push onto both bodies' joint lists; replay recreates in the same
    // order. A new joint wakes both sides.
    j->jointNextA[index] = j->bodyJointHead[bodyA];
    j->bodyJointHead[bodyA] = index;
    j->jointNextB[index] = j->bodyJointHead[bodyB];
    j->bodyJointHead[bodyB] = index;
    m3WakeIfDynamic(world, bodyA);
    m3WakeIfDynamic(world, bodyB);
    return index;
}

// Unlink from one body's list (the list is threaded through nextA
// for joints where the body plays A and nextB where it plays B).
static void UnlinkJoint(m3World* world, int32_t body, int32_t index)
{
    int32_t* cursor = &world->joints.bodyJointHead[body];
    while (*cursor != -1)
    {
        int32_t j = *cursor;
        if (j == index)
        {
            *cursor = world->joints.jointBodyA[j] == body ? world->joints.jointNextA[j]
                                                          : world->joints.jointNextB[j];
            return;
        }
        cursor = world->joints.jointBodyA[j] == body ? &world->joints.jointNextA[j]
                                                     : &world->joints.jointNextB[j];
    }
}

void m3DestroyJointInternal(m3World* world, int32_t index)
{
    int32_t bodyA = world->joints.jointBodyA[index];
    int32_t bodyB = world->joints.jointBodyB[index];
    UnlinkJoint(world, bodyA, index);
    UnlinkJoint(world, bodyB, index);
    if (world->bodies.types[bodyA] == (uint8_t)m3_dynamicBody &&
        world->bodies.bodyPool.alive[bodyA] != 0)
    {
        world->bodies.awake[bodyA] = 1;
        world->bodies.sleepTimes[bodyA] = 0.0f;
    }
    if (world->bodies.types[bodyB] == (uint8_t)m3_dynamicBody &&
        world->bodies.bodyPool.alive[bodyB] != 0)
    {
        world->bodies.awake[bodyB] = 1;
        world->bodies.sleepTimes[bodyB] = 0.0f;
    }
    world->joints.jointType[index] = 0;
    world->joints.jointBodyA[index] = -1;
    world->joints.jointBodyB[index] = -1;
    world->joints.jointLocalA[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointLocalB[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointCollide[index] = 0;
    world->joints.jointImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointPerpImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointLimitImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointAngularImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointFrameQA[index] = m3MakeIdentityQuat();
    world->joints.jointFrameQB[index] = m3MakeIdentityQuat();
    world->joints.jointFlags[index] = 0;
    world->joints.jointMotor[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointBreak[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointSpring[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointTargetScalar[index] = 0.0f;
    world->joints.jointTargetQ[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    world->joints.jointSpringImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointLimits[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointGenericModes[index] = 0;
    world->joints.jointGenLinLower[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointGenLinUpper[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointGenAngLower[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointGenAngUpper[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->joints.jointGroundA[index] = (m3Pos3){0.0, 0.0, 0.0};
    world->joints.jointGroundB[index] = (m3Pos3){0.0, 0.0, 0.0};
    world->joints.jointNextA[index] = -1;
    world->joints.jointNextB[index] = -1;
    m3IdPoolFree(&world->joints.jointPool, index);
}

static bool AxesReal(const m3JointDef* def)
{
    return m3Dot3(def->localAxisA, def->localAxisA) > 0.0f &&
           m3Dot3(def->localAxisB, def->localAxisB) > 0.0f;
}

// Sane modes and finite ordered limits on every axis. At most one
// angular axis is limited, and only when the other two are both locked
// or both free.
static bool GenericAxesValid(const m3JointDef* def)
{
    int32_t limitedAngular = 0;
    int32_t lockedAngular = 0;
    int32_t freeAngular = 0;
    for (int32_t k = 0; k < 3; ++k)
    {
        if (def->genericLinear[k] > 2 || def->genericAngular[k] > 2)
        {
            return false;
        }
        if (def->genericLinear[k] == (uint8_t)m3_axisLimited &&
            !(m3FiniteF(def->genericLinearLower[k]) && m3FiniteF(def->genericLinearUpper[k]) &&
              def->genericLinearLower[k] <= def->genericLinearUpper[k]))
        {
            return false;
        }
        if (def->genericAngular[k] == (uint8_t)m3_axisLimited &&
            !(m3FiniteF(def->genericAngularLower[k]) && m3FiniteF(def->genericAngularUpper[k]) &&
              def->genericAngularLower[k] <= def->genericAngularUpper[k]))
        {
            return false;
        }
        limitedAngular += def->genericAngular[k] == (uint8_t)m3_axisLimited ? 1 : 0;
        lockedAngular += def->genericAngular[k] == (uint8_t)m3_axisLocked ? 1 : 0;
        freeAngular += def->genericAngular[k] == (uint8_t)m3_axisFree ? 1 : 0;
    }
    return limitedAngular == 0 || (limitedAngular == 1 && (lockedAngular == 2 || freeAngular == 2));
}

// The generic contract: valid axes, at most one motor and only on an
// axis that can move, and a joint frame with real axes.
static bool GenericDefValid(const m3JointDef* def)
{
    if (!GenericAxesValid(def) || !AxesReal(def))
    {
        return false;
    }
    if (def->genericMotorAxis == 255)
    {
        return true;
    }
    if (def->genericMotorAxis > 5 || !m3FiniteF(def->motorSpeed) ||
        !m3FiniteF(def->maxMotorEffort) || def->maxMotorEffort < 0.0f)
    {
        return false;
    }
    uint8_t mode = def->genericMotorAxis < 3 ? def->genericLinear[def->genericMotorAxis]
                                             : def->genericAngular[def->genericMotorAxis - 3];
    return mode != (uint8_t)m3_axisLocked;
}

// The distance contract: an explicit range (a rod has equal bounds), and
// a spring only with a real hertz and a rest length inside the range.
static bool DistanceDefValid(const m3JointDef* def)
{
    if (!def->enableLimit || def->lowerLimit < 0.0f)
    {
        return false;
    }
    return !def->enableMotor || (def->motorSpeed > 0.0f && def->maxMotorEffort >= 0.0f &&
                                 !(def->coneAngle > 0.0f && (def->coneAngle < def->lowerLimit ||
                                                             def->coneAngle > def->upperLimit)));
}

// The whole def contract in one place: known type, real axes where the
// type needs them, the generic and distance rules, and finite, ordered
// fields. m3CreateJoint refuses once when this fails.
static bool JointDefIsValid(const m3JointDef* def)
{
    if (def->internalValue != M3_JOINT_COOKIE || def->type < 0 ||
        def->type > (int32_t)m3_pulleyJoint)
    {
        return false;
    }
    bool typeValid = true;
    switch (def->type)
    {
    case m3_revoluteJoint:
    case m3_prismaticJoint:
    case m3_wheelJoint:
    case m3_parallelJoint:
        typeValid = AxesReal(def);
        break;
    case m3_genericJoint:
        typeValid = GenericDefValid(def);
        break;
    case m3_distanceJoint:
        typeValid = DistanceDefValid(def);
        break;
    default:
        break;
    }
    return typeValid && m3FiniteV3(def->localAnchorA) && m3FiniteV3(def->localAnchorB) &&
           m3FiniteV3(def->localAxisA) && m3FiniteV3(def->localAxisB) &&
           m3FiniteF(def->lowerLimit) && m3FiniteF(def->upperLimit) && m3FiniteF(def->motorSpeed) &&
           m3FiniteF(def->maxMotorEffort) && m3FiniteF(def->coneAngle) &&
           def->maxMotorEffort >= 0.0f &&
           !(def->enableLimit && def->lowerLimit > def->upperLimit) &&
           !(def->enableCone && def->coneAngle < 0.0f);
}

m3JointId m3CreateJoint(m3WorldId worldId, const m3JointDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullJointId;
    }
    if (!JointDefIsValid(def))
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullJointId;
    }
    if (def->bodyIdA.world != world->idWorld || def->bodyIdB.world != world->idWorld)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullJointId; // both bodies must live in this world
    }
    int32_t bodyA = m3BodySlot(world, def->bodyIdA);
    int32_t bodyB = m3BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullJointId;
    }
    if (world->bodies.types[bodyA] != (uint8_t)m3_dynamicBody &&
        world->bodies.types[bodyB] != (uint8_t)m3_dynamicBody)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullJointId; // a joint between immovables is inert
    }
    int32_t index = m3CreateJointInternal(world, def, bodyA, bodyB);
    if (index < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullJointId;
    }
    m3JointId id = {index + 1, world->idWorld, world->joints.jointPool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        m3CreateJointOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateJoint, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyJoint(m3JointId jointId)
{
    m3World* world = m3WorldFromTag(jointId.world);
    int32_t index = world != NULL ? m3JointSlot(world, jointId) : -1;
    if (index < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale id: a quiet no-op is the destroy contract
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyJoint, &jointId, (int32_t)sizeof(jointId));
    }
    m3DestroyJointInternal(world, index);
}

bool m3Joint_IsValid(m3JointId jointId)
{
    m3World* world = m3WorldFromTag(jointId.world);
    return world != NULL && m3JointSlot(world, jointId) >= 0;
}

// Resolves a joint id to its world and slot; for anything stale or
// foreign, refuses and returns NULL.
static m3World* ResolveJoint(m3JointId jointId, int32_t* outIndex)
{
    m3World* world = m3WorldFromTag(jointId.world);
    int32_t index = world != NULL ? m3JointSlot(world, jointId) : -1;
    if (index < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return NULL;
    }
    *outIndex = index;
    return world;
}

m3WorldId m3Joint_GetWorld(m3JointId jointId)
{
    int32_t index;
    m3World* world = ResolveJoint(jointId, &index);
    m3WorldId id = {0, 0};
    if (world != NULL)
    {
        id = (m3WorldId){(uint16_t)(world->slot + 1), world->generation};
    }
    return id;
}

m3JointType m3Joint_GetType(m3JointId jointId)
{
    int32_t index;
    m3World* world = ResolveJoint(jointId, &index);
    return world != NULL ? (m3JointType)world->joints.jointType[index] : m3_sphericalJoint;
}

static m3BodyId JointBody(m3JointId jointId, bool sideB)
{
    int32_t index;
    m3World* world = ResolveJoint(jointId, &index);
    m3BodyId id = {0, 0, 0};
    if (world != NULL)
    {
        int32_t body = sideB ? world->joints.jointBodyB[index] : world->joints.jointBodyA[index];
        id = (m3BodyId){body + 1, world->idWorld, world->bodies.bodyPool.generations[body]};
    }
    return id;
}

m3BodyId m3Joint_GetBodyA(m3JointId jointId)
{
    return JointBody(jointId, false);
}

m3BodyId m3Joint_GetBodyB(m3JointId jointId)
{
    return JointBody(jointId, true);
}
