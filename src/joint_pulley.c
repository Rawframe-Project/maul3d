// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pulley joint: two ropes over ground anchors sharing one length.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void PreparePulley(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    const m3Transform* xfA = f->xfA;
    const m3Transform* xfB = f->xfB;
    m3Vec3 rlcA = f->rlcA;
    m3Vec3 rlcB = f->rlcB;
    // The pulley: two rope segments to fixed WORLD
    // anchors, u vectors double-subtracted here; the solve
    // refreshes lengths from the substep deltas and builds
    // the axial mass fresh (as in the distance joint). Slot map:
    // jointMotor.z = ratio, jointLimits.z = the constant.
    m3Pos3 gA = world->joints.jointGroundA[j];
    m3Pos3 gB = world->joints.jointGroundB[j];
    c->perpAxisX = (m3Vec3){(m3real)(xfA->p.x + (double)rlcA.x + (double)c->rA.x - gA.x),
                            (m3real)(xfA->p.y + (double)rlcA.y + (double)c->rA.y - gA.y),
                            (m3real)(xfA->p.z + (double)rlcA.z + (double)c->rA.z - gA.z)};
    c->perpAxisY = (m3Vec3){(m3real)(xfB->p.x + (double)rlcB.x + (double)c->rB.x - gB.x),
                            (m3real)(xfB->p.y + (double)rlcB.y + (double)c->rB.y - gB.y),
                            (m3real)(xfB->p.z + (double)rlcB.z + (double)c->rB.z - gB.z)};
    m3real l1sq = m3Dot3(c->perpAxisX, c->perpAxisX);
    m3real l2sq = m3Dot3(c->perpAxisY, c->perpAxisY);
    c->rotationAxis = (m3Vec3){0.0f, 1.0f, 0.0f};
    c->swingAxis = (m3Vec3){0.0f, 1.0f, 0.0f};
    if (l1sq > 1.0e-12f)
    {
        c->rotationAxis = m3MulSV3(1.0f / sqrtf(l1sq), c->perpAxisX);
    }
    if (l2sq > 1.0e-12f)
    {
        c->swingAxis = m3MulSV3(1.0f / sqrtf(l2sq), c->perpAxisY);
    }
    c->motorSpeed = world->joints.jointMotor[j].z;
    c->restLength = world->joints.jointLimits[j].z;
    c->motorImpulse = world->joints.jointPerpImpulse[j].z;
}

static void WarmStartPulley(const m3World* world, const m3JointConstraint* c, m3JointWarmContext* w)
{
    m3Vec3 rA = w->rA;
    m3Vec3 rB = w->rB;
    // The same law linearly: the rope impulse along axis1
    // on A and ratio times along axis2 on B, self-applied.
    m3Vec3 pA = m3MulSV3(c->motorImpulse, c->rotationAxis);
    m3Vec3 pB = m3MulSV3(c->motorImpulse * c->motorSpeed, c->swingAxis);
    world->bodies.linearVelocities[c->bodyA] =
        m3Add3(world->bodies.linearVelocities[c->bodyA], m3MulSV3(c->invMassA, pA));
    world->bodies.angularVelocities[c->bodyA] =
        m3Add3(world->bodies.angularVelocities[c->bodyA], m3MulMV3(c->invIA, m3Cross3(rA, pA)));
    world->bodies.linearVelocities[c->bodyB] =
        m3Add3(world->bodies.linearVelocities[c->bodyB], m3MulSV3(c->invMassB, pB));
    world->bodies.angularVelocities[c->bodyB] =
        m3Add3(world->bodies.angularVelocities[c->bodyB], m3MulMV3(c->invIB, m3Cross3(rB, pB)));
}

static void SolvePulley(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 rA = s->rA;
    m3Vec3 rB = s->rB;
    m3Vec3 vA = s->vA;
    m3Vec3 wA = s->wA;
    m3Vec3 vB = s->vB;
    m3Vec3 wB = s->wB;
    const m3Vec3* deltaPos = s->deltaPos;
    int useBias = s->useBias;
    // The rope row: live segment vectors from the
    // prepare u's plus the substep COM and arm motion (the
    // world anchors never move), fresh axial mass per
    // iteration (as in the distance joint), rigid both ways.
    m3real ratio = c->motorSpeed;
    m3Vec3 u1 = m3Add3(c->perpAxisX, m3Add3(deltaPos[c->bodyA], m3Sub3(rA, c->rA)));
    m3Vec3 u2 = m3Add3(c->perpAxisY, m3Add3(deltaPos[c->bodyB], m3Sub3(rB, c->rB)));
    m3real l1sq = m3Dot3(u1, u1);
    m3real l2sq = m3Dot3(u2, u2);
    m3Vec3 a1 = c->rotationAxis;
    m3Vec3 a2 = c->swingAxis;
    m3real len1 = 0.0f;
    m3real len2 = 0.0f;
    if (l1sq > 1.0e-12f)
    {
        len1 = sqrtf(l1sq);
        a1 = m3MulSV3(1.0f / len1, u1);
    }
    if (l2sq > 1.0e-12f)
    {
        len2 = sqrtf(l2sq);
        a2 = m3MulSV3(1.0f / len2, u2);
    }
    m3Vec3 crossA = m3Cross3(rA, a1);
    m3Vec3 crossB = m3Cross3(rB, a2);
    m3real k = c->invMassA + m3Dot3(crossA, m3MulMV3(c->invIA, crossA)) +
               ratio * ratio * (c->invMassB + m3Dot3(crossB, m3MulMV3(c->invIB, crossB)));
    m3real mass = k > 0.0f ? 1.0f / k : 0.0f;
    m3real cdot =
        m3Dot3(a1, m3Add3(vA, m3Cross3(wA, rA))) + ratio * m3Dot3(a2, m3Add3(vB, m3Cross3(wB, rB)));
    m3real bias = 0.0f;
    m3real massScale = 1.0f;
    m3real impulseScale = 0.0f;
    if (useBias)
    {
        m3real cc = len1 + ratio * len2 - c->restLength;
        bias = c->softness.biasRate * cc;
        massScale = c->softness.massScale;
        impulseScale = c->softness.impulseScale;
    }
    m3real delta = -massScale * mass * (cdot + bias) - impulseScale * c->motorImpulse;
    c->motorImpulse += delta;
    m3Vec3 pA = m3MulSV3(delta, a1);
    m3Vec3 pB = m3MulSV3(delta * ratio, a2);
    vA = m3Add3(vA, m3MulSV3(c->invMassA, pA));
    wA = m3Add3(wA, m3MulMV3(c->invIA, m3Cross3(rA, pA)));
    vB = m3Add3(vB, m3MulSV3(c->invMassB, pB));
    wB = m3Add3(wB, m3MulMV3(c->invIB, m3Cross3(rB, pB)));
    world->bodies.linearVelocities[c->bodyA] = vA;
    world->bodies.angularVelocities[c->bodyA] = wA;
    world->bodies.linearVelocities[c->bodyB] = vB;
    world->bodies.angularVelocities[c->bodyB] = wB;
    return;
}

const m3JointKind m3_pulleyJointKind = {PreparePulley, WarmStartPulley, SolvePulley};
