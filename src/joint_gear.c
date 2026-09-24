// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The gear joint: couples the spins of two bodies about their frame z
// axes, aA . wA + ratio aB . wB = 0, one row on two different axes.
//
// Each side's spin is measured about its own gear axis against its spin
// at creation. A spin is only known up to a whole turn, so the sum of
// the two drifts jumps by 2 pi, or ratio times it, whenever a gear
// crosses the seam; left alone, the correction would hammer the mesh
// back a quarter turn at each crossing. The true drift is small (the
// velocity row holds it), so the sum snaps to the nearest point of the
// lattice of whole turns and only the residual is corrected.
//
// Slots: the gear row (perpImpulse.z).

#include "joint_solver.h"

#include "joint.h"
#include "solver.h"
#include "world_internal.h"

#include <math.h>

static void PrepareGear(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    c->ratio = world->joints.jointMotor[j].z;
    m3real spinA = m3GearSpin(f->xfA->q, world->joints.jointFrameQA[j]);
    m3real spinB = m3GearSpin(f->xfB->q, world->joints.jointFrameQB[j]);
    m3real drift = m3WrapPi(spinA - world->joints.jointMotor[j].x) +
                   c->ratio * m3WrapPi(spinB - world->joints.jointMotor[j].y);
    m3real best = drift;
    for (int32_t turnsA = -1; turnsA <= 1; ++turnsA)
    {
        for (int32_t turnsB = -1; turnsB <= 1; ++turnsB)
        {
            m3real candidate =
                drift + 2.0f * M3_PI * (m3real)turnsA + 2.0f * M3_PI * c->ratio * (m3real)turnsB;
            best = fabsf(candidate) < fabsf(best) ? candidate : best;
        }
    }
    c->target = best;
}

// The gear axes stay at their prepare directions, where the drift is
// measured.
static m3JointRow GearRow(const m3JointConstraint* c)
{
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    m3JointRow row = {zero, m3FrameAxis(c->frameQA, 2), zero,
                      m3MulSV3(c->ratio, m3FrameAxis(c->frameQB, 2))};
    return row;
}

static void WarmStartGear(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    (void)pose;
    m3JointRow row = GearRow(c);
    m3PushRow(&row, b, c->perpImpulse.z);
}

static void SolveGear(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                      const m3JointPass* pass)
{
    m3JointRow row = GearRow(c);
    m3real C = 0.0f;
    if (pass->biased)
    {
        // The drift at prepare plus what each side turned since, taken
        // along its gear axis.
        C = c->target + m3Dot3(m3RotationVector(pose->turnA), row.angA) +
            m3Dot3(m3RotationVector(pose->turnB), row.angB);
    }
    m3SolveRow(&row, b, m3HeldDrive(c->softness, C, pass->biased), &c->perpImpulse.z, -M3_ROW_FREE,
               M3_ROW_FREE);
}

const m3JointKind m3_gearJointKind = {PrepareGear, WarmStartGear, SolveGear};
