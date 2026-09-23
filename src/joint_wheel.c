// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The wheel joint: a suspension strut with a spinning, optionally steered
// wheel.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void PrepareWheel(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    m3real h = f->h;
    const m3Transform* xfA = f->xfA;
    const m3Transform* xfB = f->xfB;
    // The wheel: frame z is the axle (the revolute
    // half spins about it), frame x the suspension slide
    // (the prismatic half), frame y the fore-aft lock. The
    // spare spherical slots carry the extra axes: swingAxis
    // = suspension world, twistJacobian = fore-aft world.
    c->frameQA = m3MulQuat(xfA->q, world->jointFrameQA[j]);
    c->frameQB = m3MulQuat(xfB->q, world->jointFrameQB[j]);
    m3Vec3 axis = m3RotateVec3(c->frameQA, (m3Vec3){0.0f, 0.0f, 1.0f});
    c->rotationAxis = axis;
    m3Vec3 sum = m3Add3(m3MulMV3(c->invIA, axis), m3MulMV3(c->invIB, axis));
    m3real k = m3Dot3(axis, sum);
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
    c->swingAxis = m3RotateVec3(c->frameQA, (m3Vec3){1.0f, 0.0f, 0.0f});
    c->twistJacobian = m3RotateVec3(c->frameQA, (m3Vec3){0.0f, 1.0f, 0.0f});
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
    if ((c->flags & M3_JOINT_STEER) != 0)
    {
        // Steering: the wheel slot map. The strut
        // drive's softness rides the spherical target slots
        // (wheels never use the quat target), the target
        // angle rides jointMotor.z, and the warm impulse
        // rides jointSpringImpulse.y beside the suspension
        // spring's x.
        c->steerSoft = m3MakeSoft(world->jointTargetQ[j].x, world->jointTargetQ[j].y, h);
        c->steerTarget = world->jointMotor[j].z;
        c->steerBudget = world->jointTargetQ[j].z;
    }
}

static void WarmStartWheel(const m3World* world, const m3JointConstraint* c, m3JointWarmContext* w)
{
    (void)world; // the shared tail applies the impulse
    // Angular: the axle collinearity pair plus the spin
    // motor (the revolute half). Linear: suspension rows
    // along the slide axis plus the two point-to-line rows
    // whose scalars ride c->impulse; the shared tail adds
    // c->impulse itself, so subtract it (the generic's
    // bookkeeping trick, keeping the sum honest).
    w->angularImpulse =
        m3Add3(m3MulSV3(c->perpImpulseX, c->perpAxisX), m3MulSV3(c->perpImpulseY, c->perpAxisY));
    w->angularImpulse = m3Add3(w->angularImpulse, m3MulSV3(c->motorImpulse, c->rotationAxis));
    if ((c->flags & M3_JOINT_STEER) != 0)
    {
        // The steer drive's warm impulse rides the
        // strut axis; without the flag the slot is zero.
        w->angularImpulse = m3Add3(w->angularImpulse, m3MulSV3(c->springImpulseV.y, c->swingAxis));
    }
    m3real axial = c->springImpulseV.x + c->lowerImpulse - c->upperImpulse;
    m3Vec3 P = m3MulSV3(axial, c->swingAxis);
    P = m3Add3(P, m3MulSV3(c->impulse.x, c->twistJacobian));
    P = m3Add3(P, m3MulSV3(c->impulse.y, c->rotationAxis));
    w->linearExtra = m3Sub3(P, c->impulse);
}

static void SolveWheel(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
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
    // The wheel: each block is its parent's form
    // verbatim, only the axes differ. Fixed canonical order:
    // suspension spring, spin motor, suspension limits, axle
    // collinearity, point-to-line. The suspension rows ride
    // the live frame x (prismatic style, full arms); the
    // spin rows ride the prepared axle (revolute style).
    m3Vec3 d = m3Add3(m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), c->deltaCenter),
                      m3Sub3(rB, rA));
    m3Vec3 susp = m3RotateVec3(deltaRot[c->bodyA], c->swingAxis);
    m3Vec3 sAx = m3Cross3(m3Add3(rA, d), susp);
    m3Vec3 sBx = m3Cross3(rB, susp);
    m3real translation = m3Dot3(d, susp);
    m3real ka = c->invMassA + c->invMassB + m3Dot3(sAx, m3MulMV3(c->invIA, sAx)) +
                m3Dot3(sBx, m3MulMV3(c->invIB, sBx));
    m3real suspMass = ka > 0.0f ? 1.0f / ka : 0.0f;

    if ((c->flags & M3_JOINT_SPRING) != 0)
    {
        // The suspension spring (8-6b through the wheel).
        m3real cc = translation - c->targetScalar;
        m3real bias = c->springSoft.biasRate * cc;
        m3Vec3 vRel = m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
        m3real cdot = m3Dot3(vRel, susp);
        m3real delta = -c->springSoft.massScale * suspMass * (cdot + bias) -
                       c->springSoft.impulseScale * c->springImpulseV.x;
        c->springImpulseV.x += delta;
        vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, susp));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sAx)));
        vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, susp));
        wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sBx)));
    }

    if ((c->flags & M3_JOINT_MOTOR) != 0)
    {
        // The spin motor: the drive axle, torque-rate capped.
        m3Vec3 axis = c->rotationAxis;
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
        // Suspension travel limits, the prismatic rows.
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
            m3real cdot = m3Dot3(vRel, susp);
            m3real old = c->lowerImpulse;
            m3real delta = -massScale * suspMass * (cdot + bias) - impulseScale * old;
            c->lowerImpulse = m3MaxF(old + delta, 0.0f);
            delta = c->lowerImpulse - old;
            vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, susp));
            wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sAx)));
            vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, susp));
            wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sBx)));
        }
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
            m3real cdot = -m3Dot3(vRel, susp);
            m3real old = c->upperImpulse;
            m3real delta = -massScale * suspMass * (cdot + bias) - impulseScale * old;
            c->upperImpulse = m3MaxF(old + delta, 0.0f);
            m3real applied = old - c->upperImpulse;
            vA = m3Sub3(vA, m3MulSV3(c->invMassA * applied, susp));
            wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(applied, sAx)));
            vB = m3Add3(vB, m3MulSV3(c->invMassB * applied, susp));
            wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(applied, sBx)));
        }
    }

    // Axle collinearity: the revolute's 2x2 verbatim, locking
    // the two off-axle rotations so the wheel plane rides the
    // chassis. With steering engaged the frame-x row
    // becomes the strut drive and only frame y stays locked.
    if ((c->flags & M3_JOINT_STEER) == 0)
    {
        m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
        m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
        if (quatA.x * quatB.x + quatA.y * quatB.y + quatA.z * quatB.z + quatA.w * quatB.w < 0.0f)
        {
            quatB = (m3Quat){-quatB.x, -quatB.y, -quatB.z, -quatB.w};
        }
        m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
        m3Quat relQ = m3MulQuat(conjA, quatB);
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
    else
    {
        // Steering: frame y keeps its lock, frame x
        // becomes a soft drive toward the steer target. The
        // twist about the strut is the swing-twist about
        // frame x: 2 atan2(relQ.x, relQ.w), exact regardless
        // of the spin the axle carries.
        m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
        m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
        if (quatA.x * quatB.x + quatA.y * quatB.y + quatA.z * quatB.z + quatA.w * quatB.w < 0.0f)
        {
            quatB = (m3Quat){-quatB.x, -quatB.y, -quatB.z, -quatB.w};
        }
        m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
        m3Quat relQ = m3MulQuat(conjA, quatB);
        m3Vec3 perpY = m3JointPerpColumn(quatA, relQ, (m3Vec3){0.0f, 1.0f, 0.0f});
        c->perpAxisY = perpY;
        c->perpAxisX = m3JointPerpColumn(quatA, relQ, (m3Vec3){1.0f, 0.0f, 0.0f});
        {
            m3real biasY = 0.0f;
            m3real massScale = 1.0f;
            m3real impulseScale = 0.0f;
            if (useBias)
            {
                biasY = c->softness.biasRate * relQ.y;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m3Vec3 sumY = m3Add3(m3MulMV3(c->invIA, perpY), m3MulMV3(c->invIB, perpY));
            m3real kyy = m3Dot3(perpY, sumY);
            m3real cdotY = m3Dot3(m3Sub3(wB, wA), perpY) + biasY;
            m3real deltaY =
                kyy > 0.0f ? (-massScale * cdotY / kyy - impulseScale * c->perpImpulseY) : 0.0f;
            c->perpImpulseY += deltaY;
            m3Vec3 angular = m3MulSV3(deltaY, perpY);
            wA = m3Sub3(wA, m3MulMV3(c->invIA, angular));
            wB = m3Add3(wB, m3MulMV3(c->invIB, angular));
        }
        {
            m3Vec3 strut = m3RotateVec3(quatA, (m3Vec3){1.0f, 0.0f, 0.0f});
            m3real twist = 2.0f * m3Atan2(relQ.x, relQ.w);
            m3real cc = twist - c->steerTarget;
            m3real bias = c->steerSoft.biasRate * cc;
            m3Vec3 sum = m3Add3(m3MulMV3(c->invIA, strut), m3MulMV3(c->invIB, strut));
            m3real k = m3Dot3(strut, sum);
            m3real steerMass = k > 0.0f ? 1.0f / k : 0.0f;
            m3real cdot = m3Dot3(m3Sub3(wB, wA), strut);
            m3real delta = -c->steerSoft.massScale * steerMass * (cdot + bias) -
                           c->steerSoft.impulseScale * c->springImpulseV.y;
            if (c->steerBudget > 0.0f)
            {
                m3real budget = c->steerBudget * hSub;
                m3real next = c->springImpulseV.y + delta;
                next = m3MaxF(-budget, m3MinF(budget, next));
                delta = next - c->springImpulseV.y;
                c->springImpulseV.y = next;
            }
            else
            {
                c->springImpulseV.y += delta;
            }
            wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, strut)));
            wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, strut)));
        }
    }

    // Point-to-line: the prismatic's 2x2 verbatim along the
    // fore-aft direction and the axle, the two directions
    // perpendicular to the suspension slide.
    {
        m3Vec3 perpY = m3RotateVec3(deltaRot[c->bodyA], c->twistJacobian);
        m3Vec3 perpZ = m3RotateVec3(deltaRot[c->bodyA], c->rotationAxis);
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
        m3real deltaY = -massScale * solY - impulseScale * c->impulse.x;
        m3real deltaZ = -massScale * solZ - impulseScale * c->impulse.y;
        c->impulse.x += deltaY;
        c->impulse.y += deltaZ;
        m3Vec3 P = m3Add3(m3MulSV3(deltaY, perpY), m3MulSV3(deltaZ, perpZ));
        m3Vec3 LA = m3Add3(m3MulSV3(deltaY, sAy), m3MulSV3(deltaZ, sAz));
        m3Vec3 LB = m3Add3(m3MulSV3(deltaY, sBy), m3MulSV3(deltaZ, sBz));
        vA = m3Sub3(vA, m3MulSV3(c->invMassA, P));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, LA));
        vB = m3Add3(vB, m3MulSV3(c->invMassB, P));
        wB = m3Add3(wB, m3MulMV3(c->invIB, LB));
    }

    // The wheel has no free point constraint either: write
    // back and continue.
    world->linearVelocities[c->bodyA] = vA;
    world->angularVelocities[c->bodyA] = wA;
    world->linearVelocities[c->bodyB] = vB;
    world->angularVelocities[c->bodyB] = wB;
    return;
}

const m3JointKind m3_wheelJointKind = {PrepareWheel, WarmStartWheel, SolveWheel};
