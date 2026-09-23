// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The spherical joint: a shared point, with an optional cone and twist
// limit, motor and drive spring.

#include "joint_solver.h"

#include "manifold.h"
#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void PrepareSpherical(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    if (!((c->flags & (M3_JOINT_LIMIT | M3_JOINT_CONE | M3_JOINT_SPRING)) != 0))
    {
        return;
    }
    int32_t j = f->joint;
    const m3Transform* xfA = f->xfA;
    const m3Transform* xfB = f->xfB;
    // The shoulder: cone and twist limits, prepared
    // per the reference: swing axis from the two frame
    // z-axes, the flagged twist Jacobian with its tan(theta
    // over two) term, both masses frozen at prepare.
    c->frameQA = m3MulQuat(xfA->q, world->joints.jointFrameQA[j]);
    c->frameQB = m3MulQuat(xfB->q, world->joints.jointFrameQB[j]);
    m3Vec3 coneAxis = m3RotateVec3(c->frameQA, (m3Vec3){0.0f, 0.0f, 1.0f});
    m3Vec3 twistAxis = m3RotateVec3(c->frameQB, (m3Vec3){0.0f, 0.0f, 1.0f});
    m3Vec3 swing = m3Cross3(coneAxis, twistAxis);
    m3real swingLen2 = m3Dot3(swing, swing);
    if (swingLen2 > 1.0e-12f)
    {
        swing = m3MulSV3(1.0f / sqrtf(swingLen2), swing);
    }
    else
    {
        // Aligned axes: any perpendicular does; the tangent
        // basis rule keeps the pick bit-stable.
        m3Vec3 t2;
        m3MakeTangentBasis(coneAxis, &swing, &t2);
    }
    c->swingAxis = swing;
    c->rotationAxis = coneAxis;
    m3Vec3 sumSwing = m3Add3(m3MulMV3(c->invIA, swing), m3MulMV3(c->invIB, swing));
    m3real kSwing = m3Dot3(swing, sumSwing);
    c->swingMass = kSwing > 0.0f ? 1.0f / kSwing : 0.0f;

    m3Quat conjA = {-c->frameQA.x, -c->frameQA.y, -c->frameQA.z, c->frameQA.w};
    m3Quat relQ = m3MulQuat(conjA, c->frameQB);
    m3real denom = relQ.z * relQ.z + relQ.w * relQ.w;
    m3real tanHalf = denom > 1.0e-12f ? sqrtf((relQ.x * relQ.x + relQ.y * relQ.y) / denom) : 0.0f;
    m3Vec3 perp = m3Cross3(swing, coneAxis);
    c->twistJacobian = m3Add3(coneAxis, m3MulSV3(tanHalf, perp));
    m3Vec3 sumTwist =
        m3Add3(m3MulMV3(c->invIA, c->twistJacobian), m3MulMV3(c->invIB, c->twistJacobian));
    m3real kTwist = m3Dot3(c->twistJacobian, sumTwist);
    c->twistMass = kTwist > 0.0f ? 1.0f / kTwist : 0.0f;

    c->coneAngle = world->joints.jointLimits[j].z;
    c->lowerLimit = world->joints.jointLimits[j].x;
    c->upperLimit = world->joints.jointLimits[j].y;
    c->lowerImpulse = world->joints.jointLimitImpulse[j].x;
    c->upperImpulse = world->joints.jointLimitImpulse[j].y;
    c->swingImpulse = world->joints.jointLimitImpulse[j].z;
}

static void WarmStartSpherical(const m3World* world, const m3JointConstraint* c,
                               m3JointWarmContext* w)
{
    (void)world; // the shared tail applies the impulse
    if (!((c->flags & (M3_JOINT_LIMIT | M3_JOINT_CONE | M3_JOINT_SPRING)) != 0))
    {
        return;
    }
    // Reference warm form: minus swing along the swing axis,
    // twist difference along the flagged Jacobian, and the
    // drive spring's world-frame angular payload.
    w->angularImpulse = m3MulSV3(-c->swingImpulse, c->swingAxis);
    w->angularImpulse =
        m3Add3(w->angularImpulse, m3MulSV3(c->lowerImpulse - c->upperImpulse, c->twistJacobian));
    if ((c->flags & M3_JOINT_SPRING) != 0)
    {
        w->angularImpulse = m3Add3(w->angularImpulse, c->springImpulseV);
    }
}

static void SolveSpherical(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 vA = s->vA;
    m3Vec3 wA = s->wA;
    m3Vec3 vB = s->vB;
    m3Vec3 wB = s->wB;
    const m3Quat* deltaRot = s->deltaRot;
    m3real invHSub = s->invHSub;
    int useBias = s->useBias;
    if ((c->flags & (M3_JOINT_LIMIT | M3_JOINT_CONE | M3_JOINT_SPRING)) != 0)
    {
        m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
        m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
        m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
        m3Quat relQ = m3MulQuat(conjA, quatB);

        if ((c->flags & M3_JOINT_SPRING) != 0)
        {
            // Rotation drive, the reference spherical
            // spring: the error is the world-frame pseudo
            // velocity toward the target, softened, solved
            // against the summed angular mass.
            m3Vec3 delta = m3JointDeltaQuatToRotation(relQ, c->targetQ);
            m3Vec3 err = m3MulSV3(-1.0f, m3RotateVec3(quatA, delta));
            m3Vec3 rhs = m3Add3(m3Sub3(wB, wA), m3MulSV3(c->springSoft.biasRate, err));
            m3Vec3 impulse = m3Sub3(m3MulSV3(-c->springSoft.massScale, m3Solve3(&c->springK, rhs)),
                                    m3MulSV3(c->springSoft.impulseScale, c->springImpulseV));
            c->springImpulseV = m3Add3(c->springImpulseV, impulse);
            wA = m3Sub3(wA, m3MulMV3(c->invIA, impulse));
            wB = m3Add3(wB, m3MulMV3(c->invIB, impulse));
        }

        if ((c->flags & M3_JOINT_LIMIT) != 0)
        {
            // Twist range on the flagged Jacobian (FD-verified).
            m3real angle = m3JointTwistAngle(relQ);
            m3Vec3 jac = c->twistJacobian;
            // Lower twist.
            {
                m3real cc = angle - c->lowerLimit;
                m3real bias = 0.0f;
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (cc > 0.0f)
                {
                    bias = cc * invHSub;
                }
                else if (useBias)
                {
                    bias = c->softness.biasRate * cc;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3real cdot = m3Dot3(m3Sub3(wB, wA), jac);
                m3real old = c->lowerImpulse;
                m3real delta = -massScale * c->twistMass * (cdot + bias) - impulseScale * old;
                c->lowerImpulse = m3MaxF(old + delta, 0.0f);
                delta = c->lowerImpulse - old;
                wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, jac)));
                wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, jac)));
            }
            // Upper twist (signs flipped).
            {
                m3real cc = c->upperLimit - angle;
                m3real bias = 0.0f;
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (cc > 0.0f)
                {
                    bias = cc * invHSub;
                }
                else if (useBias)
                {
                    bias = c->softness.biasRate * cc;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3real cdot = m3Dot3(m3Sub3(wA, wB), jac);
                m3real old = c->upperImpulse;
                m3real delta = -massScale * c->twistMass * (cdot + bias) - impulseScale * old;
                c->upperImpulse = m3MaxF(old + delta, 0.0f);
                delta = c->upperImpulse - old;
                wA = m3Add3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, jac)));
                wB = m3Sub3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, jac)));
            }
        }

        if ((c->flags & M3_JOINT_CONE) != 0)
        {
            // The cone: keep the swing inside the shoulder angle
            // (one-sided, signs flipped, the reference form).
            m3real swingAngle = m3JointSwingAngle(relQ);
            m3Vec3 axis = c->swingAxis;
            m3real cc = c->coneAngle - swingAngle;
            m3real bias = 0.0f;
            m3real massScale = 1.0f;
            m3real impulseScale = 0.0f;
            if (cc > 0.0f)
            {
                bias = cc * invHSub;
            }
            else if (useBias)
            {
                bias = c->softness.biasRate * cc;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m3real cdot = m3Dot3(m3Sub3(wA, wB), axis);
            m3real old = c->swingImpulse;
            m3real delta = -massScale * c->swingMass * (cdot + bias) - impulseScale * old;
            c->swingImpulse = m3MaxF(old + delta, 0.0f);
            delta = c->swingImpulse - old;
            wA = m3Add3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, axis)));
            wB = m3Sub3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, axis)));
        }

        world->bodies.angularVelocities[c->bodyA] = wA;
        world->bodies.angularVelocities[c->bodyB] = wB;
        // Fall through to the shared point constraint.
    }
    m3SolveJointPoint(world, c, s, vA, wA, vB, wB);
}

const m3JointKind m3_sphericalJointKind = {PrepareSpherical, WarmStartSpherical, SolveSpherical};
