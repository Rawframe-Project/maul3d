// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The parallel joint: keeps the two frame z axes parallel, nothing else.
// The twist about the shared axis and every translation stay free.
//
// Slots: the alignment pair (perpImpulse.x and .y).

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

static void PrepareParallel(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    (void)world; // the frames from the common setup are all it needs
    (void)c;
    (void)f;
}

static void WarmStartParallel(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    m3JointFrames f = m3PoseFrames(c, pose);
    m3WarmStartAlignment(&f, b, c->perpImpulse.x, c->perpImpulse.y);
}

static void SolveParallel(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                          const m3JointPass* pass)
{
    m3JointFrames f = m3PoseFrames(c, pose);
    m3SolveAlignment(&f, b, c->softness, pass->biased, &c->perpImpulse.x, &c->perpImpulse.y);
}

const m3JointKind m3_parallelJointKind = {PrepareParallel, WarmStartParallel, SolveParallel};
