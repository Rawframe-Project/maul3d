// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The spherical joint: a shared point, with an optional drive spring
// toward a target rotation, a twist range and a swing cone.
//
// The relative rotation splits into a swing, which tilts B's frame z
// axis away from A's, and a twist about B's z. The swing angle is the
// angle between the two z axes; it grows at the relative spin along
// their normalized cross product. The twist angle is 2 atan2(z, w) of
// the relative rotation q = (x, y, z, w); from the identity in
// joint_solver.c its gradient in A's frame is
//   ((w y + z x), (z y - w x), (w^2 + z^2)) / (w^2 + z^2).
//
// Slots: the drive spring (springImpulse), the twist range
// (limitImpulse.x and .y), the cone (limitImpulse.z) and the point
// block (impulse).

#include "joint_solver.h"

#include "manifold.h"
#include "solver.h"
#include "world_internal.h"

#include <math.h>

#define ROTATION_ROWS (M3_JOINT_LIMIT | M3_JOINT_CONE | M3_JOINT_SPRING)

static void PrepareSpherical(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    (void)world; // limits, cone and target come with the common setup
    (void)c;
    (void)f;
}

// The twist row and angle; at a half-turn swing the twist is undefined
// and the row falls back to B's axis.
static m3JointRow TwistRow(const m3JointFrames* f, m3real* angle)
{
    m3Quat q = f->rel;
    m3real norm = q.w * q.w + q.z * q.z;
    *angle = 2.0f * m3Atan2(q.z, q.w);
    m3Vec3 local = {0.0f, 0.0f, 1.0f};
    if (norm > 1.0e-12f)
    {
        local = (m3Vec3){(q.w * q.y + q.z * q.x) / norm, (q.z * q.y - q.w * q.x) / norm, 1.0f};
    }
    return m3TurnRow(m3RotateVec3(f->a, local));
}

// The swing row, run backward so the cone's impulse is never negative,
// and the swing angle. Aligned axes leave the swing direction open; the
// tangent basis picks one deterministically.
static m3JointRow ConeRow(const m3JointFrames* f, m3real* angle)
{
    m3Vec3 zA = m3FrameAxis(f->a, 2);
    m3Vec3 zB = m3FrameAxis(f->b, 2);
    m3Vec3 n = m3Cross3(zA, zB);
    m3real s = sqrtf(m3Dot3(n, n));
    *angle = m3Atan2(s, m3Dot3(zA, zB));
    if (s > 1.0e-6f)
    {
        n = m3MulSV3(1.0f / s, n);
    }
    else
    {
        m3Vec3 other;
        m3MakeTangentBasis(zA, &n, &other);
    }
    return m3TurnRow(m3MulSV3(-1.0f, n));
}

static void WarmStartSpherical(const m3JointConstraint* c, const m3JointPose* pose,
                               m3JointBodies* b)
{
    if ((c->flags & ROTATION_ROWS) != 0)
    {
        m3JointFrames f = m3PoseFrames(c, pose);
        m3real angle;
        if ((c->flags & M3_JOINT_SPRING) != 0)
        {
            m3PushTurn(b, c->springImpulse);
        }
        if ((c->flags & M3_JOINT_LIMIT) != 0)
        {
            m3JointRow twist = TwistRow(&f, &angle);
            m3WarmStartLimits(&twist, c->limitImpulse.x, c->limitImpulse.y, b);
        }
        if ((c->flags & M3_JOINT_CONE) != 0)
        {
            m3JointRow cone = ConeRow(&f, &angle);
            m3PushRow(&cone, b, c->limitImpulse.z);
        }
    }
    m3PushPoint(pose->armA, pose->armB, b, c->impulse);
}

static void SolveSpherical(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                           const m3JointPass* pass)
{
    if ((c->flags & ROTATION_ROWS) != 0)
    {
        m3JointFrames f = m3PoseFrames(c, pose);
        if ((c->flags & M3_JOINT_SPRING) != 0)
        {
            m3RowDrive unit = m3SpringDrive(c->springSoft, 1.0f);
            m3Vec3 bias = m3BlockBias(unit, m3RotationError(&f, c->targetQ));
            m3SolveTurnBlock(b, bias, unit, &c->springImpulse, M3_ROW_FREE);
        }
        if ((c->flags & M3_JOINT_LIMIT) != 0)
        {
            m3real angle;
            m3JointRow twist = TwistRow(&f, &angle);
            m3SolveLimits(&twist, angle, c->lowerLimit, c->upperLimit, c->softness,
                          &c->limitImpulse.x, &c->limitImpulse.y, b, pass);
        }
        if ((c->flags & M3_JOINT_CONE) != 0)
        {
            m3real angle;
            m3JointRow cone = ConeRow(&f, &angle);
            m3RowDrive drive =
                m3LimitDrive(c->softness, c->coneAngle - angle, pass->invH, pass->biased);
            m3SolveRow(&cone, b, drive, &c->limitImpulse.z, 0.0f, M3_ROW_FREE);
        }
    }
    m3SolveJointPoint(c, pose, b, pass);
}

const m3JointKind m3_sphericalJointKind = {PrepareSpherical, WarmStartSpherical, SolveSpherical};
