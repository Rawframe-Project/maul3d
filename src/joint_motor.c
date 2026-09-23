// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motor joint: drives body B toward a pose relative to body A within
// force and torque budgets.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void PrepareMotor(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    const m3Transform* xfA = f->xfA;
    const m3Transform* xfB = f->xfB;
    // The servo weld: frames like the fixed joint,
    // targets from the slot map (jointMotor = offset in the
    // A frame, jointTargetQ = rotation), budgets from the
    // limit slots (x = maxForce, y = maxTorque, 0 =
    // uncapped). swingAxis carries the offset to the solve.
    c->frameQA = m3MulQuat(xfA->q, world->jointFrameQA[j]);
    c->frameQB = m3MulQuat(xfB->q, world->jointFrameQB[j]);
    c->swingAxis = world->jointMotor[j];
    c->lowerLimit = world->jointLimits[j].x;
    c->upperLimit = world->jointLimits[j].y;
    c->angularImpulse = world->jointAngularImpulse[j];
    if ((c->flags & M3_JOINT_SPRING) == 0)
    {
        // A springless servo idles free: no rows, and the
        // warm state forgets so a re-enabled spring starts
        // from rest instead of a stale kick.
        c->impulse = (m3Vec3){0.0f, 0.0f, 0.0f};
        c->angularImpulse = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
}

static void SolveMotor(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
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
    // The servo weld: both rows soft BY LAW, so the
    // shared point weld below must never see this type. The
    // rotation row is the spherical drive verbatim on the
    // fixed joint's frames; the translation row is the
    // shared tail's separation aimed at the rotated target
    // offset and softened by the same spring. Budgets clamp
    // the accumulated impulse magnitude (the 16-2 shape),
    // so a starved servo sags honestly instead of lying.
    if ((c->flags & M3_JOINT_SPRING) == 0)
    {
        return; // the springless servo idles free
    }
    m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
    m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
    m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
    m3Quat relQ = m3MulQuat(conjA, quatB);
    {
        m3Vec3 delta = m3JointDeltaQuatToRotation(relQ, c->targetQ);
        m3Vec3 err = m3MulSV3(-1.0f, m3RotateVec3(quatA, delta));
        m3Vec3 rhs = m3Add3(m3Sub3(wB, wA), m3MulSV3(c->springSoft.biasRate, err));
        m3Vec3 impulse = m3Sub3(m3MulSV3(-c->springSoft.massScale, m3Solve3(&c->springK, rhs)),
                                m3MulSV3(c->springSoft.impulseScale, c->angularImpulse));
        m3Vec3 total = m3Add3(c->angularImpulse, impulse);
        if (c->upperLimit > 0.0f)
        {
            m3real maxImpulse = c->upperLimit * hSub;
            m3real len2 = m3Dot3(total, total);
            if (len2 > maxImpulse * maxImpulse)
            {
                total = m3MulSV3(maxImpulse / sqrtf(len2), total);
            }
        }
        impulse = m3Sub3(total, c->angularImpulse);
        c->angularImpulse = total;
        wA = m3Sub3(wA, m3MulMV3(c->invIA, impulse));
        wB = m3Add3(wB, m3MulMV3(c->invIB, impulse));
    }
    {
        m3Vec3 separation = m3Add3(
            m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), m3Sub3(rB, rA)), c->deltaCenter);
        separation = m3Sub3(separation, m3RotateVec3(quatA, c->swingAxis));
        m3Mat3 k;
        m3Vec3 basis[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        m3Vec3* cols[3] = {&k.cx, &k.cy, &k.cz};
        for (int32_t a = 0; a < 3; ++a)
        {
            m3Vec3 e = basis[a];
            m3Vec3 tA = m3Cross3(rA, m3MulMV3(c->invIA, m3Cross3(rA, e)));
            m3Vec3 tB = m3Cross3(rB, m3MulMV3(c->invIB, m3Cross3(rB, e)));
            *cols[a] = m3Sub3(m3MulSV3(c->invMassA + c->invMassB, e), m3Add3(tA, tB));
        }
        m3Vec3 cdotLin = m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA)));
        m3Vec3 rhs = m3Add3(cdotLin, m3MulSV3(c->springSoft.biasRate, separation));
        m3Vec3 impulse = m3Sub3(m3MulSV3(-c->springSoft.massScale, m3Solve3(&k, rhs)),
                                m3MulSV3(c->springSoft.impulseScale, c->impulse));
        m3Vec3 total = m3Add3(c->impulse, impulse);
        if (c->lowerLimit > 0.0f)
        {
            m3real maxImpulse = c->lowerLimit * hSub;
            m3real len2 = m3Dot3(total, total);
            if (len2 > maxImpulse * maxImpulse)
            {
                total = m3MulSV3(maxImpulse / sqrtf(len2), total);
            }
        }
        impulse = m3Sub3(total, c->impulse);
        c->impulse = total;
        vA = m3Sub3(vA, m3MulSV3(c->invMassA, impulse));
        wA = m3Sub3(wA, m3MulMV3(c->invIA, m3Cross3(rA, impulse)));
        vB = m3Add3(vB, m3MulSV3(c->invMassB, impulse));
        wB = m3Add3(wB, m3MulMV3(c->invIB, m3Cross3(rB, impulse)));
    }
    world->linearVelocities[c->bodyA] = vA;
    world->angularVelocities[c->bodyA] = wA;
    world->linearVelocities[c->bodyB] = vB;
    world->angularVelocities[c->bodyB] = wB;
    return;
}

const m3JointKind m3_motorJointKind = {PrepareMotor, m3WarmStartAngularLock, SolveMotor};
