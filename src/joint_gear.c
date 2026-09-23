// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The gear joint: couples the rotations of two bodies by a ratio.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void PrepareGear(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    const m3Transform* xfA = f->xfA;
    const m3Transform* xfB = f->xfB;
    // The gear: ONE angular row on two DIFFERENT
    // world axes, J = [aA, ratio * aB]. Mass frozen at
    // prepare; the drift against the create spins (slot
    // map: jointMotor = {phiA0, phiB0, ratio}) rides
    // targetScalar and the solve adds the substep deltas.
    c->frameQA = m3MulQuat(xfA->q, world->jointFrameQA[j]);
    c->frameQB = m3MulQuat(xfB->q, world->jointFrameQB[j]);
    m3Vec3 aA = m3RotateVec3(c->frameQA, (m3Vec3){0.0f, 0.0f, 1.0f});
    m3Vec3 aB = m3RotateVec3(c->frameQB, (m3Vec3){0.0f, 0.0f, 1.0f});
    c->rotationAxis = aA;
    c->swingAxis = aB;
    m3real ratio = world->jointMotor[j].z;
    c->motorSpeed = ratio;
    m3real k =
        m3Dot3(aA, m3MulMV3(c->invIA, aA)) + ratio * ratio * m3Dot3(aB, m3MulMV3(c->invIB, aB));
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
    c->motorImpulse = world->jointPerpImpulse[j].z;
    m3real phiA = m3GearSpin(xfA->q, world->jointFrameQA[j]);
    m3real phiB = m3GearSpin(xfB->q, world->jointFrameQB[j]);
    // Each side's spin is only measurable mod its wrap, so
    // the raw sum jumps by 2*pi (or ratio times it) every
    // time a gear crosses the seam, and the drift correction
    // hammered the mesh backwards a quarter turn (the tak
    // tak tak the testbed's user heard). The true drift is
    // tiny (the velocity row holds it), so snap the sum to
    // the nearest point of the wrap lattice and correct only
    // the residual. Candidates keep the exact float when no
    // wrap happened: the golden does not move.
    m3real s =
        m3WrapPi(phiA - world->jointMotor[j].x) + ratio * m3WrapPi(phiB - world->jointMotor[j].y);
    m3real best = s;
    for (int32_t wi = -1; wi <= 1; ++wi)
    {
        for (int32_t wj = -1; wj <= 1; ++wj)
        {
            m3real cand = s + 2.0f * M3_PI * (m3real)wi + 2.0f * M3_PI * ratio * (m3real)wj;
            if (fabsf(cand) < fabsf(best))
            {
                best = cand;
            }
        }
    }
    c->targetScalar = best;
}

static void WarmStartGear(const m3World* world, const m3JointConstraint* c, m3JointWarmContext* w)
{
    (void)w; // applies its own impulse, no shared-tail share
    // One scalar on two DIFFERENT axes: the shared tail
    // cannot express it, so the gear warms itself here and
    // leaves the tail zero (c->impulse stays zero for
    // gears by construction).
    world->angularVelocities[c->bodyA] =
        m3Add3(world->angularVelocities[c->bodyA],
               m3MulMV3(c->invIA, m3MulSV3(c->motorImpulse, c->rotationAxis)));
    world->angularVelocities[c->bodyB] =
        m3Add3(world->angularVelocities[c->bodyB],
               m3MulMV3(c->invIB, m3MulSV3(c->motorImpulse * c->motorSpeed, c->swingAxis)));
}

static void SolveGear(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 wA = s->wA;
    m3Vec3 wB = s->wB;
    const m3Quat* deltaRot = s->deltaRot;
    int useBias = s->useBias;
    // The gear row: one equality on two axes; the
    // angular-only joint must never see the point weld.
    m3real ratio = c->motorSpeed;
    m3Vec3 aA = c->rotationAxis;
    m3Vec3 aB = c->swingAxis;
    m3real cdot = m3Dot3(aA, wA) + ratio * m3Dot3(aB, wB);
    m3real bias = 0.0f;
    m3real massScale = 1.0f;
    m3real impulseScale = 0.0f;
    if (useBias)
    {
        // Drift = the prepare-time offset plus what the
        // substeps have rotated since, linearized on the
        // prepare axes (the TGS small-angle form).
        m3real cc = c->targetScalar + m3Dot3(m3JointQuatToRotationVec(deltaRot[c->bodyA]), aA) +
                    ratio * m3Dot3(m3JointQuatToRotationVec(deltaRot[c->bodyB]), aB);
        bias = c->softness.biasRate * cc;
        massScale = c->softness.massScale;
        impulseScale = c->softness.impulseScale;
    }
    m3real delta = -massScale * c->axialMass * (cdot + bias) - impulseScale * c->motorImpulse;
    c->motorImpulse += delta;
    wA = m3Add3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, aA)));
    wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta * ratio, aB)));
    world->angularVelocities[c->bodyA] = wA;
    world->angularVelocities[c->bodyB] = wB;
    return;
}

const m3JointKind m3_gearJointKind = {PrepareGear, WarmStartGear, SolveGear};
