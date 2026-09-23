// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The revolute joint: a hinge about the frame z axis, with an optional
// motor, limits and drive spring.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void SolveRevolute(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 wA = s->wA;
    m3Vec3 wB = s->wB;
    const m3Quat* deltaRot = s->deltaRot;
    m3real hSub = s->hSub;
    m3real invHSub = s->invHSub;
    int useBias = s->useBias;
    // Substep frames and the relative rotation, sign-guarded
    // so the angle lives in [-pi, pi] (the reference rule).
    m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
    m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
    if (quatA.x * quatB.x + quatA.y * quatB.y + quatA.z * quatB.z + quatA.w * quatB.w < 0.0f)
    {
        quatB = (m3Quat){-quatB.x, -quatB.y, -quatB.z, -quatB.w};
    }
    m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
    m3Quat relQ = m3MulQuat(conjA, quatB);
    m3Vec3 axis = c->rotationAxis;

    if ((c->flags & M3_JOINT_SPRING) != 0)
    {
        // Angle drive, the reference revolute spring.
        m3real angle = 2.0f * m3Atan2(relQ.z, relQ.w);
        m3real cc = angle - c->targetScalar;
        m3real bias = c->springSoft.biasRate * cc;
        m3real cdot = m3Dot3(m3Sub3(wB, wA), axis);
        m3real delta = -c->springSoft.massScale * c->axialMass * (cdot + bias) -
                       c->springSoft.impulseScale * c->springImpulseV.x;
        if ((c->flags & M3_JOINT_MOTOR) != 0)
        {
            // The four-state drive: when the motor
            // rides beside the drive, both rows share ONE
            // effort budget. The spring spends what the
            // motor's accumulator has left.
            m3real budget = c->maxMotorEffort * hSub;
            m3real room = m3MaxF(budget - m3AbsF(c->motorImpulse), 0.0f);
            m3real next = c->springImpulseV.x + delta;
            next = m3MaxF(-room, m3MinF(room, next));
            delta = next - c->springImpulseV.x;
            c->springImpulseV.x = next;
        }
        else
        {
            c->springImpulseV.x += delta;
        }
        wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, axis)));
        wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, axis)));
    }

    if ((c->flags & M3_JOINT_MOTOR) != 0)
    {
        // Motor: velocity drive with the torque-rate cap.
        m3real cdot = m3Dot3(m3Sub3(wB, wA), axis) - c->motorSpeed;
        m3real delta = -c->axialMass * cdot;
        m3real newImpulse = c->motorImpulse + delta;
        m3real maxImpulse = c->maxMotorEffort * hSub;
        newImpulse = m3MaxF(-maxImpulse, m3MinF(maxImpulse, newImpulse));
        delta = newImpulse - c->motorImpulse;
        c->motorImpulse = newImpulse;
        wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, axis)));
        wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, axis)));
    }

    if ((c->flags & M3_JOINT_LIMIT) != 0)
    {
        // Twist angle about the hinge: 2 atan2(relQ.z, relQ.w).
        m3real angle = 2.0f * m3Atan2(relQ.z, relQ.w);
        // Lower limit.
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
            m3real cdot = m3Dot3(m3Sub3(wB, wA), axis);
            m3real old = c->lowerImpulse;
            m3real delta = -massScale * c->axialMass * (cdot + bias) - impulseScale * old;
            c->lowerImpulse = m3MaxF(old + delta, 0.0f);
            delta = c->lowerImpulse - old;
            wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, axis)));
            wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, axis)));
        }
        // Upper limit (signs flipped, the reference form).
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
            m3real cdot = m3Dot3(m3Sub3(wA, wB), axis);
            m3real old = c->upperImpulse;
            m3real delta = -massScale * c->axialMass * (cdot + bias) - impulseScale * old;
            c->upperImpulse = m3MaxF(old + delta, 0.0f);
            delta = c->upperImpulse - old;
            wA = m3Add3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, axis)));
            wB = m3Sub3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, axis)));
        }
    }

    // Collinearity: lock the two off-axis rotations, 2x2.
    {
        m3Vec3 perpX = m3JointPerpColumn(quatA, relQ, (m3Vec3){1.0f, 0.0f, 0.0f});
        m3Vec3 perpY = m3JointPerpColumn(quatA, relQ, (m3Vec3){0.0f, 1.0f, 0.0f});
        c->perpAxisX = perpX;
        c->perpAxisY = perpY;
        m3real biasX = 0.0f;
        m3real biasY = 0.0f;
        m3real massScale = 1.0f;
        m3real impulseScale = 0.0f;
        if (useBias)
        {
            biasX = c->softness.biasRate * relQ.x;
            biasY = c->softness.biasRate * relQ.y;
            massScale = c->softness.massScale;
            impulseScale = c->softness.impulseScale;
        }
        m3Vec3 sumX = m3Add3(m3MulMV3(c->invIA, perpX), m3MulMV3(c->invIB, perpX));
        m3Vec3 sumY = m3Add3(m3MulMV3(c->invIA, perpY), m3MulMV3(c->invIB, perpY));
        m3real kxx = m3Dot3(perpX, sumX);
        m3real kyy = m3Dot3(perpY, sumY);
        m3real kxy = m3Dot3(perpX, sumY);
        m3Vec3 wRel = m3Sub3(wB, wA);
        m3real cdotX = m3Dot3(wRel, perpX) + biasX;
        m3real cdotY = m3Dot3(wRel, perpY) + biasY;
        m3real det = kxx * kyy - kxy * kxy;
        m3real solX = 0.0f;
        m3real solY = 0.0f;
        if (det != 0.0f)
        {
            m3real inv = 1.0f / det;
            solX = inv * (kyy * cdotX - kxy * cdotY);
            solY = inv * (kxx * cdotY - kxy * cdotX);
        }
        m3real deltaX = -massScale * solX - impulseScale * c->perpImpulseX;
        m3real deltaY = -massScale * solY - impulseScale * c->perpImpulseY;
        c->perpImpulseX += deltaX;
        c->perpImpulseY += deltaY;
        m3Vec3 angular = m3Add3(m3MulSV3(deltaX, perpX), m3MulSV3(deltaY, perpY));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, angular));
        wB = m3Add3(wB, m3MulMV3(c->invIB, angular));
    }

    // The hinge rows move only the spins; the shared point constraint
    // finishes with the linear velocities as the substep found them.
    m3SolveJointPoint(world, c, s, s->vA, wA, s->vB, wB);
}

const m3JointKind m3_revoluteJointKind = {m3PrepareHingeFrame, m3WarmStartHinge, SolveRevolute};
