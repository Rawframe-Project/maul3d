// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The fixed joint: a shared point and a locked relative rotation, both
// held at the pose the joint was created in.
//
// Slots: the rotation lock (angularImpulse) and the point block
// (impulse).

#include "joint_solver.h"
#include "world_internal.h"

static const m3Quat s_identity = {0.0f, 0.0f, 0.0f, 1.0f};

static void PrepareFixed(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    (void)world; // the frames from the common setup are all it needs
    (void)c;
    (void)f;
}

static void WarmStartFixed(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    m3PushTurn(b, c->angularImpulse);
    m3PushPoint(pose->armA, pose->armB, b, c->impulse);
}

static void SolveFixed(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                       const m3JointPass* pass)
{
    m3RowDrive unit = m3HeldDrive(c->softness, 1.0f, pass->biased);
    m3Vec3 error = {0.0f, 0.0f, 0.0f};
    if (pass->biased)
    {
        m3JointFrames f = m3PoseFrames(c, pose);
        error = m3RotationError(&f, s_identity);
    }
    m3Vec3 turnBias = m3BlockBias(unit, error);
    const m3JointRow rows[3] = {m3TurnRow((m3Vec3){1.0f, 0.0f, 0.0f}),
                                m3TurnRow((m3Vec3){0.0f, 1.0f, 0.0f}),
                                m3TurnRow((m3Vec3){0.0f, 0.0f, 1.0f})};
    const m3real bias[3] = {turnBias.x, turnBias.y, turnBias.z};
    m3real* const accumulated[3] = {&c->angularImpulse.x, &c->angularImpulse.y,
                                    &c->angularImpulse.z};
    m3SolveJointPointWith(c, pose, b, pass, rows, 3, bias, accumulated);
}

const m3JointKind m3_fixedJointKind = {PrepareFixed, WarmStartFixed, SolveFixed};
