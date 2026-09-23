// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver's stages: the setup every joint shares, dispatch to
// the kind table (one joint_<kind>.c per type), impulse storage, and the
// helpers the kinds share.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static const m3JointKind s_filterJointKind = {NULL, NULL, NULL};

static const m3JointKind* const s_kinds[] = {
    [m3_sphericalJoint] = &m3_sphericalJointKind, [m3_revoluteJoint] = &m3_revoluteJointKind,
    [m3_prismaticJoint] = &m3_prismaticJointKind, [m3_fixedJoint] = &m3_fixedJointKind,
    [m3_distanceJoint] = &m3_distanceJointKind,   [m3_genericJoint] = &m3_genericJointKind,
    [m3_wheelJoint] = &m3_wheelJointKind,         [m3_filterJoint] = &s_filterJointKind,
    [m3_parallelJoint] = &m3_parallelJointKind,   [m3_motorJoint] = &m3_motorJointKind,
    [m3_gearJoint] = &m3_gearJointKind,           [m3_pulleyJoint] = &m3_pulleyJointKind,
};

// Twist about z with the polarity guard, and the all-squared swing
// (the reference math_functions forms).
// Pseudo angular velocity from a quaternion target (the reference
// math_internal form): w = 2 * vec((target - s) * conj(s)), polarity
// corrected.
m3Vec3 m3JointDeltaQuatToRotation(m3Quat q, m3Quat target)
{
    m3Quat s = q;
    if (q.x * target.x + q.y * target.y + q.z * target.z + q.w * target.w < 0.0f)
    {
        s = (m3Quat){-q.x, -q.y, -q.z, -q.w};
    }
    m3Quat diff = {target.x - s.x, target.y - s.y, target.z - s.z, target.w - s.w};
    m3Quat conjS = {-s.x, -s.y, -s.z, s.w};
    m3Quat product = m3MulQuat(diff, conjS);
    return (m3Vec3){2.0f * product.x, 2.0f * product.y, 2.0f * product.z};
}

m3real m3JointTwistAngle(m3Quat q)
{
    m3real twist = q.w < 0.0f ? m3Atan2(-q.z, -q.w) : m3Atan2(q.z, q.w);
    return 2.0f * twist;
}

m3real m3JointSwingAngle(m3Quat q)
{
    m3real x = sqrtf(q.z * q.z + q.w * q.w);
    m3real y = sqrtf(q.x * q.x + q.y * q.y);
    return 2.0f * m3Atan2(y, x);
}

// Rotation vector of a relative quaternion (robust: exact angle via
// atan2, small angles fall back to the linear form).
m3Vec3 m3JointQuatToRotationVec(m3Quat relQ)
{
    if (relQ.w < 0.0f)
    {
        relQ = (m3Quat){-relQ.x, -relQ.y, -relQ.z, -relQ.w};
    }
    m3Vec3 v = {relQ.x, relQ.y, relQ.z};
    m3real len = sqrtf(m3Dot3(v, v));
    if (len < 1.0e-9f)
    {
        return m3MulSV3(2.0f, v);
    }
    m3real angle = 2.0f * m3Atan2(len, relQ.w);
    return m3MulSV3(angle / len, v);
}

// Half-quaternion rotation of a frame axis, the reference form for
// the collinearity Jacobian columns.
m3Vec3 m3JointPerpColumn(m3Quat qA, m3Quat relQ, m3Vec3 axis)
{
    m3Vec3 rv = {relQ.x, relQ.y, relQ.z};
    m3Vec3 inner = m3Add3(m3MulSV3(relQ.w, axis), m3Cross3(rv, axis));
    return m3MulSV3(0.5f, m3RotateVec3(qA, inner));
}

int32_t m3PrepareJoints(m3World* world, m3JointConstraint* joints, m3real h)
{
    int32_t count = 0;
    int32_t maxJoint = world->jointPool.maxIndex;
    for (int32_t j = 0; j < maxJoint; ++j)
    {
        if (world->jointPool.alive[j] == 0)
        {
            continue;
        }
        if (world->jointType[j] == (uint8_t)m3_filterJoint)
        {
            // The filter joint is rowless BY LAW: it never
            // enters the constraint array, because every type that
            // does and fails to continue inherits the shared point
            // weld below.
            continue;
        }
        int32_t bodyA = world->jointBodyA[j];
        int32_t bodyB = world->jointBodyB[j];
        int awakeDynA = world->types[bodyA] == (uint8_t)m3_dynamicBody && world->awake[bodyA];
        int awakeDynB = world->types[bodyB] == (uint8_t)m3_dynamicBody && world->awake[bodyB];
        if (!awakeDynA && !awakeDynB)
        {
            continue; // both sides frozen or immovable
        }
        m3JointConstraint* c = &joints[count];
        count += 1;
        memset(c, 0, sizeof(*c)); // every field defined for every type:
                                  // the store writes them all back
        c->joint = j;
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        const m3Transform* xfA = &world->transforms[bodyA];
        const m3Transform* xfB = &world->transforms[bodyB];
        c->rA = m3RotateVec3(xfA->q, m3Sub3(world->jointLocalA[j], world->localCenters[bodyA]));
        c->rB = m3RotateVec3(xfB->q, m3Sub3(world->jointLocalB[j], world->localCenters[bodyB]));
        m3Vec3 rlcA = m3RotateVec3(xfA->q, world->localCenters[bodyA]);
        m3Vec3 rlcB = m3RotateVec3(xfB->q, world->localCenters[bodyB]);
        c->deltaCenter = (m3Vec3){(m3real)(xfB->p.x + (double)rlcB.x - xfA->p.x - (double)rlcA.x),
                                  (m3real)(xfB->p.y + (double)rlcB.y - xfA->p.y - (double)rlcA.y),
                                  (m3real)(xfB->p.z + (double)rlcB.z - xfA->p.z - (double)rlcA.z)};
        c->invMassA = world->types[bodyA] == (uint8_t)m3_dynamicBody ? world->invMass[bodyA] : 0.0f;
        c->invMassB = world->types[bodyB] == (uint8_t)m3_dynamicBody ? world->invMass[bodyB] : 0.0f;
        c->invIA = m3WorldInvInertia(world, bodyA);
        c->invIB = m3WorldInvInertia(world, bodyB);
        c->softness = m3MakeSoft(60.0f, 2.0f, h); // the reference joint stiffness
        c->impulse = world->jointImpulse[j];
        c->type = world->jointType[j];
        c->flags = world->jointFlags[j];
        c->targetScalar = world->jointTargetScalar[j];
        c->targetQ = world->jointTargetQ[j];
        c->springImpulseV = world->jointSpringImpulse[j];
        if ((c->flags & M3_JOINT_SPRING) != 0)
        {
            // The drive spring: reference softness from the
            // runtime hertz and damping ratio. The distance joint's
            // spring reuse cannot reach here (flag 8 refuses it).
            c->springSoft = m3MakeSoft(world->jointSpring[j].x, world->jointSpring[j].y, h);
            m3Mat3 sum = c->invIA;
            sum.cx = m3Add3(sum.cx, c->invIB.cx);
            sum.cy = m3Add3(sum.cy, c->invIB.cy);
            sum.cz = m3Add3(sum.cz, c->invIB.cz);
            c->springK = sum;
        }
        const m3JointKind* kind = s_kinds[c->type];
        m3JointFrame frame = {j, h, xfA, xfB, rlcA, rlcB};
        kind->prepare(world, c, &frame);
    }
    return count;
}

void m3WarmStartJoints(m3World* world, m3JointConstraint* joints, int32_t count,
                       const m3Quat* deltaRot)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3JointConstraint* c = &joints[i];
        m3Vec3 rA = m3RotateVec3(deltaRot[c->bodyA], c->rA);
        m3Vec3 rB = m3RotateVec3(deltaRot[c->bodyB], c->rB);
        // A kind adds its extra linear and angular impulse to these.
        m3JointWarmContext w = {rA, rB, deltaRot, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
        s_kinds[c->type]->warmStart(world, c, &w);
        m3Vec3 totalLinear = m3Add3(c->impulse, w.linearExtra);
        world->linearVelocities[c->bodyA] =
            m3Sub3(world->linearVelocities[c->bodyA], m3MulSV3(c->invMassA, totalLinear));
        world->angularVelocities[c->bodyA] =
            m3Sub3(world->angularVelocities[c->bodyA],
                   m3MulMV3(c->invIA, m3Add3(m3Cross3(rA, totalLinear), w.angularImpulse)));
        world->linearVelocities[c->bodyB] =
            m3Add3(world->linearVelocities[c->bodyB], m3MulSV3(c->invMassB, totalLinear));
        world->angularVelocities[c->bodyB] =
            m3Add3(world->angularVelocities[c->bodyB],
                   m3MulMV3(c->invIB, m3Add3(m3Cross3(rB, totalLinear), w.angularImpulse)));
    }
}

void m3SolveJoints(m3World* world, m3JointConstraint* joints, int32_t count, const m3Vec3* deltaPos,
                   const m3Quat* deltaRot, m3real hSub, m3real invHSub, int useBias)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3JointConstraint* c = &joints[i];
        m3Vec3 rA = m3RotateVec3(deltaRot[c->bodyA], c->rA);
        m3Vec3 rB = m3RotateVec3(deltaRot[c->bodyB], c->rB);

        m3Vec3 vA = world->linearVelocities[c->bodyA];
        m3Vec3 wA = world->angularVelocities[c->bodyA];
        m3Vec3 vB = world->linearVelocities[c->bodyB];
        m3Vec3 wB = world->angularVelocities[c->bodyB];

        m3JointSolveContext ctx = {rA,       rB,       vA,   wA,      vB,     wB,
                                   deltaPos, deltaRot, hSub, invHSub, useBias};
        s_kinds[c->type]->solve(world, c, &ctx);
    }
}

void m3StoreJointImpulses(m3World* world, m3JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        const m3JointConstraint* c = &joints[i];
        world->jointImpulse[c->joint] = c->impulse;
        if (c->type == (uint8_t)m3_genericJoint)
        {
            // The generic slot map: linear uppers ride the
            // perp slots, the angular upper and the motor ride the
            // limit slots.
            world->jointPerpImpulse[c->joint] =
                (m3Vec3){c->perpImpulseX, c->perpImpulseY, c->swingImpulse};
            world->jointLimitImpulse[c->joint] = (m3Vec3){c->upperImpulse, c->motorImpulse, 0.0f};
        }
        else
        {
            world->jointPerpImpulse[c->joint] =
                (m3Vec3){c->perpImpulseX, c->perpImpulseY, c->motorImpulse};
            world->jointLimitImpulse[c->joint] =
                (m3Vec3){c->lowerImpulse, c->upperImpulse,
                         c->type == (uint8_t)m3_sphericalJoint ? c->swingImpulse : 0.0f};
        }
        world->jointAngularImpulse[c->joint] = c->angularImpulse;
        world->jointSpringImpulse[c->joint] = c->springImpulseV;
    }
}

// --- Shared by the kinds -----------------------------------------------------

void m3PrepareHingeFrame(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    const m3Transform* xfA = f->xfA;
    const m3Transform* xfB = f->xfB;
    c->frameQA = m3MulQuat(xfA->q, world->jointFrameQA[j]);
    c->frameQB = m3MulQuat(xfB->q, world->jointFrameQB[j]);
    m3Vec3 axis = m3RotateVec3(c->frameQA, (m3Vec3){0.0f, 0.0f, 1.0f});
    c->rotationAxis = axis;
    m3Vec3 sum = m3Add3(m3MulMV3(c->invIA, axis), m3MulMV3(c->invIB, axis));
    m3real k = m3Dot3(axis, sum);
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
    m3Quat conjA = {-c->frameQA.x, -c->frameQA.y, -c->frameQA.z, c->frameQA.w};
    m3Quat relQ = m3MulQuat(conjA, c->frameQB);
    c->perpAxisX = m3JointPerpColumn(c->frameQA, relQ, (m3Vec3){1.0f, 0.0f, 0.0f});
    c->perpAxisY = m3JointPerpColumn(c->frameQA, relQ, (m3Vec3){0.0f, 1.0f, 0.0f});
    c->perpImpulseX = world->jointPerpImpulse[j].x;
    c->perpImpulseY = world->jointPerpImpulse[j].y;
    c->motorImpulse = world->jointPerpImpulse[j].z;
    c->lowerImpulse = world->jointLimitImpulse[j].x;
    c->upperImpulse = world->jointLimitImpulse[j].y;
    c->motorSpeed = world->jointMotor[j].x;
    c->maxMotorEffort = world->jointMotor[j].y;
    c->lowerLimit = world->jointLimits[j].x;
    c->upperLimit = world->jointLimits[j].y;
    c->angularImpulse = world->jointAngularImpulse[j];
}

void m3WarmStartHinge(const m3World* world, const m3JointConstraint* c, m3JointWarmContext* w)
{
    (void)world; // the shared tail applies the impulse
    // The parallel joint rides this branch with every axial
    // term zero: only the perp locks carry warm impulse.
    m3real axial = c->springImpulseV.x + c->motorImpulse + c->lowerImpulse - c->upperImpulse;
    w->angularImpulse =
        m3Add3(m3MulSV3(c->perpImpulseX, c->perpAxisX), m3MulSV3(c->perpImpulseY, c->perpAxisY));
    w->angularImpulse = m3Add3(w->angularImpulse, m3MulSV3(axial, c->rotationAxis));
}

void m3WarmStartAngularLock(const m3World* world, const m3JointConstraint* c, m3JointWarmContext* w)
{
    (void)world; // the shared tail applies the impulse
    // The weld's lock; the servo weld rides the same
    // slots (prepare zeroed them when springless) and its
    // translation row is c->impulse in the shared tail.
    w->angularImpulse = c->angularImpulse;
}

// The shared point constraint: the anchors coincide. The spherical,
// revolute and fixed joints end their solve here, with the velocities
// their own rows left behind.
void m3SolveJointPoint(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s,
                       m3Vec3 vA, m3Vec3 wA, m3Vec3 vB, m3Vec3 wB)
{
    m3Vec3 rA = s->rA;
    m3Vec3 rB = s->rB;
    const m3Vec3* deltaPos = s->deltaPos;
    int useBias = s->useBias;
    m3Vec3 cdot = m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA)));

    m3Vec3 bias = {0.0f, 0.0f, 0.0f};
    m3real massScale = 1.0f;
    m3real impulseScale = 0.0f;
    if (useBias)
    {
        // The raw anchor violation, the reference form: current
        // anchor delta = COM drift + rotated anchors + the
        // prepare-time center offset. No baseline subtraction:
        // a satisfied joint has deltaCenter = rA0 - rB0 and the
        // sum vanishes by itself.
        m3Vec3 separation = m3Add3(
            m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), m3Sub3(rB, rA)), c->deltaCenter);
        bias = m3MulSV3(c->softness.biasRate, separation);
        massScale = c->softness.massScale;
        impulseScale = c->softness.impulseScale;
    }

    // K = (mA + mB) I - skew(rA) iA skew(rA) - skew(rB) iB skew(rB),
    // built column by column by applying the operator to the basis
    // (no matrix-matrix helpers needed).
    m3Mat3 k;
    m3Vec3 basis[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    m3Vec3* cols[3] = {&k.cx, &k.cy, &k.cz};
    for (int32_t a = 0; a < 3; ++a)
    {
        m3Vec3 e = basis[a];
        m3Vec3 tA = m3Cross3(rA, m3MulMV3(c->invIA, m3Cross3(rA, e)));
        m3Vec3 tB = m3Cross3(rB, m3MulMV3(c->invIB, m3Cross3(rB, e)));
        m3Vec3 col = m3Sub3(m3MulSV3(c->invMassA + c->invMassB, e), m3Add3(tA, tB));
        *cols[a] = col;
    }

    m3Vec3 b = m3Solve3(&k, m3Add3(cdot, bias));
    m3Vec3 impulse = m3Sub3(m3MulSV3(-massScale, b), m3MulSV3(impulseScale, c->impulse));
    c->impulse = m3Add3(c->impulse, impulse);

    world->linearVelocities[c->bodyA] = m3Sub3(vA, m3MulSV3(c->invMassA, impulse));
    world->angularVelocities[c->bodyA] = m3Sub3(wA, m3MulMV3(c->invIA, m3Cross3(rA, impulse)));
    world->linearVelocities[c->bodyB] = m3Add3(vB, m3MulSV3(c->invMassB, impulse));
    world->angularVelocities[c->bodyB] = m3Add3(wB, m3MulMV3(c->invIB, m3Cross3(rB, impulse)));
}
