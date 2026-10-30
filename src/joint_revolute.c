// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The revolute joint: a hinge about the frame z axis, with an optional
// motor, limits and drive spring.
//
// Slots: the hinge angle row carries the drive spring (springImpulse.x),
// the motor (perpImpulse.z) and the limits (limitImpulse.x and .y); the
// alignment pair keeps the frame z axes together (perpImpulse.x and .y);
// the point block pins the anchors (impulse).

#include "joint_solver.h"
#include "world_internal.h"

static void PrepareRevolute(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    (void)world; // the common setup is all a hinge needs
    (void)c;
    (void)f;
}

static void WarmStartRevolute(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    m3JointFrames f = m3PoseFrames(c, pose);
    m3JointRow spin = m3TurnRow(m3FrameAxis(f.a, 2));
    m3PushRow(&spin, b, c->springImpulse.x + c->perpImpulse.z);
    m3WarmStartLimits(&spin, c->limitImpulse.x, c->limitImpulse.y, b);
    m3WarmStartAlignment(&f, b, c->perpImpulse.x, c->perpImpulse.y);
    m3PushPoint(pose->armA, pose->armB, b, c->impulse);
}

static void SolveRevolute(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                          const m3JointPass* pass)
{
    m3JointFrames f = m3PoseFrames(c, pose);
    m3JointRow spin = m3TurnRow(m3FrameAxis(f.a, 2));
    bool angled = (c->flags & (M3_JOINT_SPRING | M3_JOINT_LIMIT)) != 0;
    m3real angle = angled ? 2.0f * m3Atan2(f.rel.z, f.rel.w) : 0.0f;
    m3SolveDrive(&spin, angle, c, b, pass, &c->springImpulse.x, &c->perpImpulse.z);
    if ((c->flags & M3_JOINT_LIMIT) != 0)
    {
        m3SolveLimits(&spin, angle, c->lowerLimit, c->upperLimit, c->softness, &c->limitImpulse.x,
                      &c->limitImpulse.y, b, pass);
    }
    m3RowDrive unit = m3HeldDrive(c->softness, 1.0f, pass->biased);
    const m3JointRow rows[2] = {m3TurnRow(m3RelativeGradient(&f, 0)),
                                m3TurnRow(m3RelativeGradient(&f, 1))};
    const m3real bias[2] = {unit.bias * f.rel.x, unit.bias * f.rel.y};
    m3real* const accumulated[2] = {&c->perpImpulse.x, &c->perpImpulse.y};
    m3SolveJointPointWith(c, pose, b, pass, rows, 2, bias, accumulated);
}

const m3JointKind m3_revoluteJointKind = {PrepareRevolute, WarmStartRevolute, SolveRevolute};
