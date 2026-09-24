// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pulley joint: two ropes from fixed ground points to the anchors,
// length A + ratio * length B held at the rope total (ratio in the
// motor z slot, the total in the cone slot).
//
// Slots: the rope row (perpImpulse.z).

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>

// An anchor's world position minus a ground point, the origin subtracted
// in double first.
static m3Vec3 RopeFrom(const m3Transform* xf, m3Vec3 center, m3Vec3 arm, m3Pos3 ground)
{
    return (m3Vec3){(m3real)(xf->p.x + (double)center.x + (double)arm.x - ground.x),
                    (m3real)(xf->p.y + (double)center.y + (double)arm.y - ground.y),
                    (m3real)(xf->p.z + (double)center.z + (double)arm.z - ground.z)};
}

static void PreparePulley(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    c->ropeA = RopeFrom(f->xfA, f->rlcA, c->rA, world->joints.jointGroundA[j]);
    c->ropeB = RopeFrom(f->xfB, f->rlcB, c->rB, world->joints.jointGroundB[j]);
    c->ratio = world->joints.jointMotor[j].z;
    c->restLength = world->joints.jointLimits[j].z;
}

// A rope now and its unit direction; a rope that shrank to its ground
// point keeps +y.
static m3Vec3 Rope(m3Vec3 rope0, m3Vec3 move, m3Vec3 arm, m3Vec3 arm0, m3real* length)
{
    m3Vec3 rope = m3Add3(rope0, m3Add3(move, m3Sub3(arm, arm0)));
    m3real length2 = m3Dot3(rope, rope);
    *length = length2 > 1.0e-12f ? sqrtf(length2) : 0.0f;
    return length2 > 1.0e-12f ? m3MulSV3(1.0f / *length, rope) : (m3Vec3){0.0f, 1.0f, 0.0f};
}

// The rope row, J v = uA . vpA + ratio uB . vpB, and its error.
static m3JointRow PulleyRow(const m3JointConstraint* c, const m3JointPose* pose, m3real* C)
{
    m3real lengthA;
    m3real lengthB;
    m3Vec3 uA = Rope(c->ropeA, pose->moveA, pose->armA, c->rA, &lengthA);
    m3Vec3 uB = Rope(c->ropeB, pose->moveB, pose->armB, c->rB, &lengthB);
    *C = lengthA + c->ratio * lengthB - c->restLength;
    m3JointRow row = {uA, m3Cross3(pose->armA, uA), m3MulSV3(c->ratio, uB),
                      m3MulSV3(c->ratio, m3Cross3(pose->armB, uB))};
    return row;
}

static void WarmStartPulley(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    m3real C;
    m3JointRow row = PulleyRow(c, pose, &C);
    m3PushRow(&row, b, c->perpImpulse.z);
}

static void SolvePulley(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                        const m3JointPass* pass)
{
    m3real C;
    m3JointRow row = PulleyRow(c, pose, &C);
    m3SolveRow(&row, b, m3HeldDrive(c->softness, C, pass->biased), &c->perpImpulse.z, -M3_ROW_FREE,
               M3_ROW_FREE);
}

const m3JointKind m3_pulleyJointKind = {PreparePulley, WarmStartPulley, SolvePulley};
