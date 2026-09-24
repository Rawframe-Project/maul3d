// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The prismatic joint: a slider along the frame z axis with the rotation
// locked, an optional motor, limits and drive spring. The slide axes are
// fixed in A and turn with it, so every linear row reaches from A's
// center to B's anchor.
//
// Slots: the slide row carries the drive spring (springImpulse.x), the
// motor (perpImpulse.z) and the limits (limitImpulse.x and .y); the rows
// across the slide along the frame x and y axes (perpImpulse.x and .y);
// the rotation lock (angularImpulse).

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

static const m3Quat s_identity = {0.0f, 0.0f, 0.0f, 1.0f};

static void PreparePrismatic(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    (void)world; // the frames from the common setup are all it needs
    (void)c;
    (void)f;
}

typedef struct SlideRows
{
    m3JointRow across[2];
    m3JointRow along;
    m3JointFrames frames;
} SlideRows;

static SlideRows BuildRows(const m3JointConstraint* c, const m3JointPose* pose)
{
    SlideRows r;
    r.frames = m3PoseFrames(c, pose);
    r.across[0] = m3SlideRow(pose, m3FrameAxis(r.frames.a, 0));
    r.across[1] = m3SlideRow(pose, m3FrameAxis(r.frames.a, 1));
    r.along = m3SlideRow(pose, m3FrameAxis(r.frames.a, 2));
    return r;
}

static void WarmStartPrismatic(const m3JointConstraint* c, const m3JointPose* pose,
                               m3JointBodies* b)
{
    SlideRows r = BuildRows(c, pose);
    m3PushRow(&r.along, b, c->springImpulse.x + c->perpImpulse.z);
    m3WarmStartLimits(&r.along, c->limitImpulse.x, c->limitImpulse.y, b);
    m3PushTurn(b, c->angularImpulse);
    m3PushRow(&r.across[0], b, c->perpImpulse.x);
    m3PushRow(&r.across[1], b, c->perpImpulse.y);
}

static void SolvePrismatic(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                           const m3JointPass* pass)
{
    SlideRows r = BuildRows(c, pose);
    m3real travel = m3Dot3(pose->gap, m3FrameAxis(r.frames.a, 2));
    m3SolveDrive(&r.along, travel, c, b, pass, &c->springImpulse.x, &c->perpImpulse.z);
    if ((c->flags & M3_JOINT_LIMIT) != 0)
    {
        m3SolveLimits(&r.along, travel, c->lowerLimit, c->upperLimit, c->softness,
                      &c->limitImpulse.x, &c->limitImpulse.y, b, pass);
    }
    m3RowDrive unit = m3HeldDrive(c->softness, 1.0f, pass->biased);
    m3Vec3 error =
        pass->biased ? m3RotationError(&r.frames, s_identity) : (m3Vec3){0.0f, 0.0f, 0.0f};
    m3SolveTurnBlock(b, m3BlockBias(unit, error), unit, &c->angularImpulse, M3_ROW_FREE);
    m3real bias[2] = {unit.bias * m3Dot3(pose->gap, m3FrameAxis(r.frames.a, 0)),
                      unit.bias * m3Dot3(pose->gap, m3FrameAxis(r.frames.a, 1))};
    m3SolveRowPair(r.across, b, bias, unit, &c->perpImpulse.x, &c->perpImpulse.y);
}

const m3JointKind m3_prismaticJointKind = {PreparePrismatic, WarmStartPrismatic, SolvePrismatic};
