// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motor joint: a servo that pulls body B toward a pose relative to
// body A through its spring, within force and torque budgets. The
// budgets bound the accumulated impulse, so a starved servo visibly sags
// instead of reporting a pose it cannot hold. Without a spring it idles.
//
// Slots: the rotation block (angularImpulse) and the point block
// (impulse).

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

static void PrepareMotor(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    c->offset = world->joints.jointMotor[f->joint];
    if ((c->flags & M3_JOINT_SPRING) == 0)
    {
        // An idle servo forgets, so a spring enabled later starts from rest.
        c->impulse = (m3Vec3){0.0f, 0.0f, 0.0f};
        c->angularImpulse = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
}

static void WarmStartMotor(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    m3PushTurn(b, c->angularImpulse);
    m3PushPoint(pose->armA, pose->armB, b, c->impulse);
}

// A budget of zero means none.
static m3real Budget(m3real effort, m3real h)
{
    return effort > 0.0f ? effort * h : M3_ROW_FREE;
}

static void SolveMotor(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                       const m3JointPass* pass)
{
    if ((c->flags & M3_JOINT_SPRING) == 0)
    {
        return;
    }
    m3JointFrames f = m3PoseFrames(c, pose);
    m3RowDrive unit = m3SpringDrive(c->springSoft, 1.0f);
    m3Vec3 turnBias = m3BlockBias(unit, m3RotationError(&f, c->targetQ));
    m3SolveTurnBlock(b, turnBias, unit, &c->angularImpulse, Budget(c->upperLimit, pass->h));
    m3Vec3 error = m3Sub3(pose->gap, m3RotateVec3(f.a, c->offset));
    m3SolvePointBlock(pose->armA, pose->armB, b, m3BlockBias(unit, error), unit, &c->impulse,
                      Budget(c->lowerLimit, pass->h));
}

const m3JointKind m3_motorJointKind = {PrepareMotor, WarmStartMotor, SolveMotor};
