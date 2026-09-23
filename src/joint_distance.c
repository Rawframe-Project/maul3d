// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The distance joint: a rod or rope between two anchors, optionally
// springy, with an optional motor.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void PrepareDistance(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    m3real h = f->h;
    // One axial row: axis from the live anchor gap; a
    // degenerate gap picks a fixed axis deterministically.
    m3Vec3 s = m3Add3(c->deltaCenter, m3Sub3(c->rB, c->rA));
    m3real len2 = m3Dot3(s, s);
    m3Vec3 axis = {0.0f, 1.0f, 0.0f};
    if (len2 > 1.0e-12f)
    {
        axis = m3MulSV3(1.0f / sqrtf(len2), s);
    }
    c->rotationAxis = axis;
    m3Vec3 crossA = m3Cross3(c->rA, axis);
    m3Vec3 crossB = m3Cross3(c->rB, axis);
    m3real k = c->invMassA + c->invMassB + m3Dot3(crossA, m3MulMV3(c->invIA, crossA)) +
               m3Dot3(crossB, m3MulMV3(c->invIB, crossB));
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
    c->motorImpulse = world->joints.jointPerpImpulse[j].z; // the spring
    c->lowerImpulse = world->joints.jointLimitImpulse[j].x;
    c->upperImpulse = world->joints.jointLimitImpulse[j].y;
    c->lowerLimit = world->joints.jointLimits[j].x;
    c->upperLimit = world->joints.jointLimits[j].y;
    m3real rest = world->joints.jointLimits[j].z; // coneAngle reuse:
                                                  // the documented
                                                  // rest length
    c->restLength = rest > 0.0f ? rest : world->joints.jointLimits[j].y;
    c->springSoft = m3MakeSoft(world->joints.jointMotor[j].x, world->joints.jointMotor[j].y, h);
}

static void WarmStartDistance(const m3World* world, const m3JointConstraint* c,
                              m3JointWarmContext* w)
{
    (void)world; // the shared tail applies the impulse
    m3real axial = c->motorImpulse + c->lowerImpulse - c->upperImpulse;
    w->linearExtra = m3MulSV3(axial, c->rotationAxis);
}

static void SolveDistance(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 rA = s->rA;
    m3Vec3 rB = s->rB;
    m3Vec3 vA = s->vA;
    m3Vec3 wA = s->wA;
    m3Vec3 vB = s->vB;
    m3Vec3 wB = s->wB;
    const m3Vec3* deltaPos = s->deltaPos;
    m3real invHSub = s->invHSub;
    int useBias = s->useBias;
    // The distance rows: live gap and axis, fresh
    // axial mass per iteration (as in the prismatic), then
    // the optional spring and the two clamped bound rows in
    // the revolute limit recipe, signs mirrored.
    m3Vec3 sVec = m3Add3(m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), m3Sub3(rB, rA)),
                         c->deltaCenter);
    m3real len2 = m3Dot3(sVec, sVec);
    m3Vec3 axis = c->rotationAxis;
    m3real length = 0.0f;
    if (len2 > 1.0e-12f)
    {
        length = sqrtf(len2);
        axis = m3MulSV3(1.0f / length, sVec);
    }
    m3Vec3 crossA = m3Cross3(rA, axis);
    m3Vec3 crossB = m3Cross3(rB, axis);
    m3real kAx = c->invMassA + c->invMassB + m3Dot3(crossA, m3MulMV3(c->invIA, crossA)) +
                 m3Dot3(crossB, m3MulMV3(c->invIB, crossB));
    m3real axialMass = kAx > 0.0f ? 1.0f / kAx : 0.0f;

    if ((c->flags & M3_JOINT_MOTOR) != 0 && useBias)
    {
        // The spring: a soft unclamped pull toward the rest
        // length (hertz and damping from the def's motor
        // fields, the documented reuse). Bias pass ONLY: in
        // the relax pass a positional spring degenerates
        // into a dead full-strength damper and no zeta can
        // ring through it.
        m3real cdot =
            m3Dot3(axis, m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA))));
        m3real bias = c->springSoft.biasRate * (length - c->restLength);
        m3real delta = -c->springSoft.massScale * axialMass * (cdot + bias) -
                       c->springSoft.impulseScale * c->motorImpulse;
        c->motorImpulse += delta;
        vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, axis));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, crossA)));
        vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, axis));
        wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, crossB)));
    }

    // Lower bound: length >= lower (pushes apart).
    {
        m3real cc = length - c->lowerLimit;
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
        m3real cdot =
            m3Dot3(axis, m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA))));
        m3real old = c->lowerImpulse;
        m3real delta = -massScale * axialMass * (cdot + bias) - impulseScale * old;
        c->lowerImpulse = m3MaxF(old + delta, 0.0f);
        m3real applied = c->lowerImpulse - old;
        vA = m3Sub3(vA, m3MulSV3(c->invMassA * applied, axis));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(applied, crossA)));
        vB = m3Add3(vB, m3MulSV3(c->invMassB * applied, axis));
        wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(applied, crossB)));
    }
    // Upper bound: length <= upper (pulls together, signs
    // mirrored, the reference form).
    {
        m3real cc = c->upperLimit - length;
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
        m3real cdot =
            -m3Dot3(axis, m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA))));
        m3real old = c->upperImpulse;
        m3real delta = -massScale * axialMass * (cdot + bias) - impulseScale * old;
        c->upperImpulse = m3MaxF(old + delta, 0.0f);
        m3real applied = c->upperImpulse - old;
        vA = m3Add3(vA, m3MulSV3(c->invMassA * applied, axis));
        wA = m3Add3(wA, m3MulMV3(c->invIA, m3MulSV3(applied, crossA)));
        vB = m3Sub3(vB, m3MulSV3(c->invMassB * applied, axis));
        wB = m3Sub3(wB, m3MulMV3(c->invIB, m3MulSV3(applied, crossB)));
    }
    world->bodies.linearVelocities[c->bodyA] = vA;
    world->bodies.angularVelocities[c->bodyA] = wA;
    world->bodies.linearVelocities[c->bodyB] = vB;
    world->bodies.angularVelocities[c->bodyB] = wB;
    return;
}

const m3JointKind m3_distanceJointKind = {PrepareDistance, WarmStartDistance, SolveDistance};
