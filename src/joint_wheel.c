// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The wheel joint: a suspension strut with a spinning, optionally
// steered wheel. In the joint frame, z is the axle, x the suspension
// slide and y the fore-aft direction; all turn with A.
//
// Rows in order: the suspension spring and travel limits along x, the
// spin motor about the axle, the alignment that keeps the axle in the
// chassis frame (with steering on, only its y part stays locked and the
// strut twist becomes a soft drive toward the steer angle), and the rows
// that keep the wheel center on the strut line along y and z.
//
// Slots: the rows across the strut (impulse.x and .y), the alignment
// (perpImpulse.x and .y), the spin motor (perpImpulse.z), the travel
// limits (limitImpulse.x and .y), the suspension spring
// (springImpulse.x) and the steering drive (springImpulse.y).

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

static void PrepareWheel(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    if ((c->flags & M3_JOINT_STEER) != 0)
    {
        // The steering drive keeps its stiffness, damping and budget in
        // the rotation target slots, which a wheel never uses otherwise,
        // and its angle in the motor z slot.
        int32_t j = f->joint;
        c->steerSoft =
            m3MakeSoft(world->joints.jointTargetQ[j].x, world->joints.jointTargetQ[j].y, f->h);
        c->steerTarget = world->joints.jointMotor[j].z;
        c->steerBudget = world->joints.jointTargetQ[j].z;
    }
}

typedef struct WheelRows
{
    m3JointFrames frames;
    m3JointRow strut;     // along the suspension slide
    m3JointRow across[2]; // fore-aft and along the axle
    m3JointRow spin;      // about the axle
} WheelRows;

static WheelRows BuildRows(const m3JointConstraint* c, const m3JointPose* pose)
{
    WheelRows r;
    r.frames = m3PoseFrames(c, pose);
    r.strut = m3SlideRow(pose, m3FrameAxis(r.frames.a, 0));
    r.across[0] = m3SlideRow(pose, m3FrameAxis(r.frames.a, 1));
    r.across[1] = m3SlideRow(pose, m3FrameAxis(r.frames.a, 2));
    r.spin = m3TurnRow(m3FrameAxis(r.frames.a, 2));
    return r;
}

static void WarmStartWheel(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    WheelRows r = BuildRows(c, pose);
    m3PushRow(&r.strut, b, c->springImpulse.x);
    m3WarmStartLimits(&r.strut, c->limitImpulse.x, c->limitImpulse.y, b);
    m3PushRow(&r.spin, b, c->perpImpulse.z);
    if ((c->flags & M3_JOINT_STEER) == 0)
    {
        m3WarmStartAlignment(&r.frames, b, c->perpImpulse.x, c->perpImpulse.y);
    }
    else
    {
        m3JointRow lock = m3TurnRow(m3RelativeGradient(&r.frames, 1));
        m3JointRow steer = m3TurnRow(m3FrameAxis(r.frames.a, 0));
        m3PushRow(&lock, b, c->perpImpulse.y);
        m3PushRow(&steer, b, c->springImpulse.y);
    }
    m3PushRow(&r.across[0], b, c->impulse.x);
    m3PushRow(&r.across[1], b, c->impulse.y);
}

// With steering on: the alignment's y part stays locked; the twist about
// the strut, 2 atan2(x, w) of the relative rotation whatever the axle's
// spin, is driven softly toward the steer angle.
static void SolveSteering(m3JointConstraint* c, const m3JointFrames* f, m3JointBodies* b,
                          const m3JointPass* pass)
{
    m3JointRow lock = m3TurnRow(m3RelativeGradient(f, 1));
    m3SolveRow(&lock, b, m3HeldDrive(c->softness, f->rel.y, pass->biased), &c->perpImpulse.y,
               -M3_ROW_FREE, M3_ROW_FREE);
    m3JointRow steer = m3TurnRow(m3FrameAxis(f->a, 0));
    m3real twist = 2.0f * m3Atan2(f->rel.x, f->rel.w);
    m3real budget = c->steerBudget > 0.0f ? c->steerBudget * pass->h : M3_ROW_FREE;
    m3SolveRow(&steer, b, m3SpringDrive(c->steerSoft, twist - c->steerTarget), &c->springImpulse.y,
               -budget, budget);
}

static void SolveWheel(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                       const m3JointPass* pass)
{
    WheelRows r = BuildRows(c, pose);
    m3real travel = m3Dot3(pose->gap, m3FrameAxis(r.frames.a, 0));
    if ((c->flags & M3_JOINT_SPRING) != 0)
    {
        m3SolveRow(&r.strut, b, m3SpringDrive(c->springSoft, travel - c->target),
                   &c->springImpulse.x, -M3_ROW_FREE, M3_ROW_FREE);
    }
    if ((c->flags & M3_JOINT_MOTOR) != 0)
    {
        m3real budget = c->maxMotorEffort * pass->h;
        m3SolveRow(&r.spin, b, m3RigidDrive(-c->motorSpeed), &c->perpImpulse.z, -budget, budget);
    }
    if ((c->flags & M3_JOINT_LIMIT) != 0)
    {
        m3SolveLimits(&r.strut, travel, c->lowerLimit, c->upperLimit, c->softness,
                      &c->limitImpulse.x, &c->limitImpulse.y, b, pass);
    }
    if ((c->flags & M3_JOINT_STEER) == 0)
    {
        m3SolveAlignment(&r.frames, b, c->softness, pass->biased, &c->perpImpulse.x,
                         &c->perpImpulse.y);
    }
    else
    {
        SolveSteering(c, &r.frames, b, pass);
    }
    m3RowDrive unit = m3HeldDrive(c->softness, 1.0f, pass->biased);
    m3real bias[2] = {unit.bias * m3Dot3(pose->gap, m3FrameAxis(r.frames.a, 1)),
                      unit.bias * m3Dot3(pose->gap, m3FrameAxis(r.frames.a, 2))};
    m3SolveRowPair(r.across, b, bias, unit, &c->impulse.x, &c->impulse.y);
}

const m3JointKind m3_wheelJointKind = {PrepareWheel, WarmStartWheel, SolveWheel};
