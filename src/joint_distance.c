// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The distance joint: a rod or rope between two anchors holding their
// distance in [minLength, maxLength], optionally with the joint spring
// pulling toward its rest length.
//
// Slots: the range rides the limit slot's x and y and the rest length its
// z. One row along the line between the anchors carries the spring
// (springImpulse.x) and the range (limitImpulse.x and .y).

#include "joint_solver.h"
#include "solver_math.h"
#include "world_internal.h"

#include <math.h>

static void PrepareDistance(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    (void)world;
    (void)f;
    m3Vec3 gap = m3Add3(c->deltaCenter, m3Sub3(c->rB, c->rA));
    float length2 = m3Dot3(gap, gap);
    c->axis =
        length2 > 1.0e-12f ? m3MulSV3(1.0f / sqrtf(length2), gap) : (m3Vec3){0.0f, 1.0f, 0.0f};
    c->restLength = c->coneAngle; // the limit slot's z, as the cone angle loads
}

// The row along the anchor line and the current length. Anchors that
// meet keep the prepare direction.
static m3JointRow RopeRow(const m3JointConstraint* c, const m3JointPose* pose, float* length)
{
    float length2 = m3Dot3(pose->gap, pose->gap);
    *length = length2 > 1.0e-12f ? sqrtf(length2) : 0.0f;
    m3Vec3 u = length2 > 1.0e-12f ? m3MulSV3(1.0f / *length, pose->gap) : c->axis;
    return m3LineRow(pose->armA, pose->armB, u);
}

static void WarmStartDistance(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    float length;
    m3JointRow row = RopeRow(c, pose, &length);
    m3PushRow(&row, b, c->springImpulse.x);
    m3WarmStartLimits(&row, c->limitImpulse.x, c->limitImpulse.y, b);
}

static void SolveDistance(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                          const m3JointPass* pass)
{
    float length;
    m3JointRow row = RopeRow(c, pose, &length);
    // The spring pulls in the solve pass only: in the relax pass a
    // position spring would act as a full-strength damper.
    if ((c->flags & M3_JOINT_SPRING) != 0 && pass->biased)
    {
        m3RowDrive drive = m3SpringDrive(c->springSoft, length - c->restLength);
        m3SolveRow(&row, b, drive, &c->springImpulse.x, -M3_ROW_FREE, M3_ROW_FREE);
    }
    m3SolveLimits(&row, length, c->lowerLimit, c->upperLimit, c->softness, &c->limitImpulse.x,
                  &c->limitImpulse.y, b, pass);
}

const m3JointKind m3_distanceJointKind = {PrepareDistance, WarmStartDistance, SolveDistance};
