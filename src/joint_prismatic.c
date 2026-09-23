// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The prismatic joint: a slider along the frame z axis with the
// rotation locked, an optional motor, limits and drive spring.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void WarmStartPrismatic(const m3World* world, const m3JointConstraint* c,
                               m3JointWarmContext* w)
{
    (void)world; // the shared tail applies the impulse
    // Linear impulses along the prepared axis and perps; the
    // angular arms differ per body and re-derive in the solve,
    // so the warm start uses the prepared arms (the reference
    // does the same through its cached sA/sB).
    m3real axial = c->springImpulseV.x + c->motorImpulse + c->lowerImpulse - c->upperImpulse;
    w->linearExtra = m3MulSV3(axial, c->rotationAxis);
    w->linearExtra = m3Add3(w->linearExtra, m3MulSV3(c->perpImpulseX, c->perpAxisX));
    w->linearExtra = m3Add3(w->linearExtra, m3MulSV3(c->perpImpulseY, c->perpAxisY));
    w->angularImpulse = c->angularImpulse;
}

static void SolvePrismatic(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 rA = s->rA;
    m3Vec3 rB = s->rB;
    m3Vec3 vA = s->vA;
    m3Vec3 wA = s->wA;
    m3Vec3 vB = s->vB;
    m3Vec3 wB = s->wB;
    const m3Vec3* deltaPos = s->deltaPos;
    const m3Quat* deltaRot = s->deltaRot;
    m3real hSub = s->hSub;
    m3real invHSub = s->invHSub;
    int useBias = s->useBias;
    // The reference prismatic with the FULL Jacobian arms
    // sA = cross(rA + d, axis), sB = cross(rB, axis): the
    // simplified arms are the author-flagged todo, and the
    // finite-difference test in test_joints holds the full
    // form to the numerics.
    m3Vec3 d = m3Add3(m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), c->deltaCenter),
                      m3Sub3(rB, rA));
    m3Vec3 axis = m3RotateVec3(deltaRot[c->bodyA], c->rotationAxis);
    m3Vec3 sAx = m3Cross3(m3Add3(rA, d), axis);
    m3Vec3 sBx = m3Cross3(rB, axis);
    m3real translation = m3Dot3(d, axis);

    // Fresh axial mass every iteration (the reference note:
    // stale masses diverge under stress).
    m3real ka = c->invMassA + c->invMassB + m3Dot3(sAx, m3MulMV3(c->invIA, sAx)) +
                m3Dot3(sBx, m3MulMV3(c->invIB, sBx));
    m3real axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

    if ((c->flags & M3_JOINT_SPRING) != 0)
    {
        // Translation drive, the reference prismatic
        // spring: a softened linear row along the slide axis
        // with the full Jacobian arms.
        m3real cc = translation - c->targetScalar;
        m3real bias = c->springSoft.biasRate * cc;
        m3Vec3 vRel = m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
        m3real cdot = m3Dot3(vRel, axis);
        m3real delta = -c->springSoft.massScale * axialMass * (cdot + bias) -
                       c->springSoft.impulseScale * c->springImpulseV.x;
        if ((c->flags & M3_JOINT_MOTOR) != 0)
        {
            // The four-state drive, the slide twin of
            // the revolute rule: one shared effort budget.
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
        vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, axis));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sAx)));
        vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, axis));
        wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sBx)));
    }

    if ((c->flags & M3_JOINT_MOTOR) != 0)
    {
        // Motor: velocity drive along the axis, force cap.
        m3Vec3 vRel = m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
        m3real cdot = m3Dot3(vRel, axis) - c->motorSpeed;
        m3real delta = -axialMass * cdot;
        m3real newImpulse = c->motorImpulse + delta;
        m3real maxImpulse = c->maxMotorEffort * hSub;
        newImpulse = m3MaxF(-maxImpulse, m3MinF(maxImpulse, newImpulse));
        delta = newImpulse - c->motorImpulse;
        c->motorImpulse = newImpulse;
        vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, axis));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sAx)));
        vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, axis));
        wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sBx)));
    }

    if ((c->flags & M3_JOINT_LIMIT) != 0)
    {
        // Lower limit.
        {
            m3real cc = translation - c->lowerLimit;
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
            m3Vec3 vRel =
                m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
            m3real cdot = m3Dot3(vRel, axis);
            m3real old = c->lowerImpulse;
            m3real delta = -massScale * axialMass * (cdot + bias) - impulseScale * old;
            c->lowerImpulse = m3MaxF(old + delta, 0.0f);
            delta = c->lowerImpulse - old;
            vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, axis));
            wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sAx)));
            vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, axis));
            wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sBx)));
        }
        // Upper limit (signs flipped).
        {
            m3real cc = c->upperLimit - translation;
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
            m3Vec3 vRel =
                m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
            m3real cdot = -m3Dot3(vRel, axis);
            m3real old = c->upperImpulse;
            m3real delta = -massScale * axialMass * (cdot + bias) - impulseScale * old;
            c->upperImpulse = m3MaxF(old + delta, 0.0f);
            m3real applied = old - c->upperImpulse;
            vA = m3Sub3(vA, m3MulSV3(c->invMassA * applied, axis));
            wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(applied, sAx)));
            vB = m3Add3(vB, m3MulSV3(c->invMassB * applied, axis));
            wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(applied, sBx)));
        }
    }

    // Rotation lock: all three angular DOF held to the
    // prepared relative frame.
    {
        m3Vec3 bias = {0.0f, 0.0f, 0.0f};
        m3real massScale = 1.0f;
        m3real impulseScale = 0.0f;
        if (useBias)
        {
            m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
            m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
            m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
            m3Quat relQ = m3MulQuat(conjA, quatB);
            m3Vec3 rotVec = m3JointQuatToRotationVec(relQ);
            // Rev 19: the rotation error rides the bias with
            // the contact row's sign convention; the negated
            // form was a latent amplifier that prismatic
            // scenes (error ~ 0) never exposed until the
            // weld arrived carrying real tumble energy.
            m3Vec3 cErr = m3RotateVec3(quatA, rotVec);
            bias = m3MulSV3(c->softness.biasRate, cErr);
            massScale = c->softness.massScale;
            impulseScale = c->softness.impulseScale;
        }
        m3Mat3 k;
        m3Vec3 basis[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        m3Vec3* cols[3] = {&k.cx, &k.cy, &k.cz};
        for (int32_t a = 0; a < 3; ++a)
        {
            *cols[a] = m3Add3(m3MulMV3(c->invIA, basis[a]), m3MulMV3(c->invIB, basis[a]));
        }
        m3Vec3 cdot = m3Sub3(wB, wA);
        m3Vec3 sol = m3Solve3(&k, m3Add3(cdot, bias));
        m3Vec3 delta = m3Sub3(m3MulSV3(-massScale, sol), m3MulSV3(impulseScale, c->angularImpulse));
        c->angularImpulse = m3Add3(c->angularImpulse, delta);
        wA = m3Sub3(wA, m3MulMV3(c->invIA, delta));
        wB = m3Add3(wB, m3MulMV3(c->invIB, delta));
    }

    // Point-to-line: the two perpendicular translations, 2x2
    // with the full arms.
    {
        m3Vec3 perpY = m3RotateVec3(deltaRot[c->bodyA], c->perpAxisX);
        m3Vec3 perpZ = m3RotateVec3(deltaRot[c->bodyA], c->perpAxisY);
        m3real biasY = 0.0f;
        m3real biasZ = 0.0f;
        m3real massScale = 1.0f;
        m3real impulseScale = 0.0f;
        if (useBias)
        {
            biasY = c->softness.biasRate * m3Dot3(perpY, d);
            biasZ = c->softness.biasRate * m3Dot3(perpZ, d);
            massScale = c->softness.massScale;
            impulseScale = c->softness.impulseScale;
        }
        m3Vec3 sAy = m3Cross3(m3Add3(rA, d), perpY);
        m3Vec3 sBy = m3Cross3(rB, perpY);
        m3Vec3 sAz = m3Cross3(m3Add3(rA, d), perpZ);
        m3Vec3 sBz = m3Cross3(rB, perpZ);
        m3real kyy = c->invMassA + c->invMassB + m3Dot3(sAy, m3MulMV3(c->invIA, sAy)) +
                     m3Dot3(sBy, m3MulMV3(c->invIB, sBy));
        m3real kyz = m3Dot3(sAy, m3MulMV3(c->invIA, sAz)) + m3Dot3(sBy, m3MulMV3(c->invIB, sBz));
        m3real kzz = c->invMassA + c->invMassB + m3Dot3(sAz, m3MulMV3(c->invIA, sAz)) +
                     m3Dot3(sBz, m3MulMV3(c->invIB, sBz));
        m3Vec3 vRel = m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
        m3real cdotY = m3Dot3(vRel, perpY) + biasY;
        m3real cdotZ = m3Dot3(vRel, perpZ) + biasZ;
        m3real det = kyy * kzz - kyz * kyz;
        m3real solY = 0.0f;
        m3real solZ = 0.0f;
        if (det != 0.0f)
        {
            m3real inv = 1.0f / det;
            solY = inv * (kzz * cdotY - kyz * cdotZ);
            solZ = inv * (kyy * cdotZ - kyz * cdotY);
        }
        m3real deltaY = -massScale * solY - impulseScale * c->perpImpulseX;
        m3real deltaZ = -massScale * solZ - impulseScale * c->perpImpulseY;
        c->perpImpulseX += deltaY;
        c->perpImpulseY += deltaZ;
        m3Vec3 P = m3Add3(m3MulSV3(deltaY, perpY), m3MulSV3(deltaZ, perpZ));
        m3Vec3 LA = m3Add3(m3MulSV3(deltaY, sAy), m3MulSV3(deltaZ, sAz));
        m3Vec3 LB = m3Add3(m3MulSV3(deltaY, sBy), m3MulSV3(deltaZ, sBz));
        vA = m3Sub3(vA, m3MulSV3(c->invMassA, P));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, LA));
        vB = m3Add3(vB, m3MulSV3(c->invMassB, P));
        wB = m3Add3(wB, m3MulMV3(c->invIB, LB));
    }

    // The prismatic has no free point constraint: write back
    // and continue to the next joint.
    world->linearVelocities[c->bodyA] = vA;
    world->angularVelocities[c->bodyA] = wA;
    world->linearVelocities[c->bodyB] = vB;
    world->angularVelocities[c->bodyB] = wB;
    return;
}

const m3JointKind m3_prismaticJointKind = {m3PrepareHingeFrame, WarmStartPrismatic, SolvePrismatic};
