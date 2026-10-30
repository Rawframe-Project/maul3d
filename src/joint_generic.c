// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The generic joint: each of the six axes of the joint frame (three
// slides, three turns) is locked, limited or free, with at most one
// motor. The axes are fixed in A and turn with it.
//
// Modes pack two bits per axis, linear x, y, z then angular x, y, z,
// with the motor axis (0 to 2 linear, 3 to 5 angular, else none) in the
// top four bits.
//
// Slots: a linear axis's locked or lower row (impulse) and upper row
// (perpImpulse); an angular axis's locked or lower row
// (angularImpulse); the one angular upper row (limitImpulse.x, one
// angular axis at most is limited); the motor (limitImpulse.y).

#include "joint_solver.h"
#include "world_internal.h"

#define MODE_LOCKED  0u
#define MODE_FREE    1u
#define MODE_LIMITED 2u

static const m3Quat s_identity = {0.0f, 0.0f, 0.0f, 1.0f};

static void PrepareGeneric(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    c->genModes = world->joints.jointGenericModes[j];
    c->genLinLower = world->joints.jointGenLinLower[j];
    c->genLinUpper = world->joints.jointGenLinUpper[j];
    c->genAngLower = world->joints.jointGenAngLower[j];
    c->genAngUpper = world->joints.jointGenAngUpper[j];
}

static uint32_t LinearMode(const m3JointConstraint* c, int32_t k)
{
    return (uint32_t)(c->genModes >> (2 * k)) & 3u;
}

static uint32_t AngularMode(const m3JointConstraint* c, int32_t k)
{
    return (uint32_t)(c->genModes >> (6 + 2 * k)) & 3u;
}

static uint32_t MotorAxis(const m3JointConstraint* c)
{
    return (uint32_t)(c->genModes >> 12) & 15u;
}

// The motor's row, when there is one.
static bool MotorRow(const m3JointConstraint* c, const m3JointPose* pose, const m3JointFrames* f,
                     m3JointRow* row)
{
    uint32_t axis = MotorAxis(c);
    if (axis < 3u)
    {
        *row = m3SlideRow(pose, m3FrameAxis(f->a, (int32_t)axis));
    }
    else if (axis < 6u)
    {
        *row = m3TurnRow(m3FrameAxis(f->a, (int32_t)axis - 3));
    }
    return axis < 6u;
}

static void WarmStartGeneric(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b)
{
    m3JointFrames f = m3PoseFrames(c, pose);
    m3Vec3 linear = c->impulse;
    m3Vec3 upper = c->perpImpulse;
    m3Vec3 angular = c->angularImpulse;
    for (int32_t k = 0; k < 3; ++k)
    {
        if (LinearMode(c, k) != MODE_FREE)
        {
            m3JointRow row = m3SlideRow(pose, m3FrameAxis(f.a, k));
            m3PushRow(&row, b, *m3Component(&linear, k) - *m3Component(&upper, k));
        }
        if (AngularMode(c, k) != MODE_FREE)
        {
            m3JointRow row = m3TurnRow(m3FrameAxis(f.a, k));
            m3real push = *m3Component(&angular, k);
            push -= AngularMode(c, k) == MODE_LIMITED ? c->limitImpulse.x : 0.0f;
            m3PushRow(&row, b, push);
        }
    }
    m3JointRow motor;
    if (MotorRow(c, pose, &f, &motor))
    {
        m3PushRow(&motor, b, c->limitImpulse.y);
    }
}

// The locked axes, whatever subset of the six, solved as one block: a
// slide row per locked linear axis and a turn row per locked angular one.
// Solved one after another, rows that share a body far from its center
// undo each other's work. One locked axis is a single row.
static void SolveLockedAxes(m3JointConstraint* c, const m3JointPose* pose, const m3JointFrames* f,
                            m3JointBodies* b, const m3JointPass* pass)
{
    m3RowDrive unit = m3HeldDrive(c->softness, 1.0f, pass->biased);
    bool turns = false;
    for (int32_t k = 0; k < 3; ++k)
    {
        turns = turns || AngularMode(c, k) == MODE_LOCKED;
    }
    m3Vec3 error = turns ? m3RotationError(f, s_identity) : (m3Vec3){0.0f, 0.0f, 0.0f};
    m3JointRow rows[M3_BLOCK_ROWS];
    m3real bias[M3_BLOCK_ROWS];
    m3real* accumulated[M3_BLOCK_ROWS];
    int32_t count = 0;
    for (int32_t k = 0; k < 3; ++k)
    {
        if (LinearMode(c, k) == MODE_LOCKED)
        {
            m3Vec3 axis = m3FrameAxis(f->a, k);
            rows[count] = m3SlideRow(pose, axis);
            bias[count] = unit.bias * m3Dot3(axis, pose->gap);
            accumulated[count++] = m3Component(&c->impulse, k);
        }
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        if (AngularMode(c, k) == MODE_LOCKED)
        {
            m3Vec3 axis = m3FrameAxis(f->a, k);
            rows[count] = m3TurnRow(axis);
            bias[count] = unit.bias * m3Dot3(axis, error);
            accumulated[count++] = m3Component(&c->angularImpulse, k);
        }
    }
    if (count == 1)
    {
        m3RowDrive one = {bias[0], unit.scale, unit.leak};
        m3SolveRow(&rows[0], b, one, accumulated[0], -M3_ROW_FREE, M3_ROW_FREE);
    }
    else if (count > 1)
    {
        m3SolveRowBlock(rows, count, b, bias, unit, accumulated);
    }
}

static void SolveLinearLimits(m3JointConstraint* c, const m3JointPose* pose, const m3JointFrames* f,
                              m3JointBodies* b, const m3JointPass* pass)
{
    for (int32_t k = 0; k < 3; ++k)
    {
        uint32_t mode = LinearMode(c, k);
        m3Vec3 axis = m3FrameAxis(f->a, k);
        m3JointRow row = m3SlideRow(pose, axis);
        m3real coord = m3Dot3(axis, pose->gap);
        if (mode == MODE_LIMITED)
        {
            m3SolveLimits(&row, coord, *m3Component(&c->genLinLower, k),
                          *m3Component(&c->genLinUpper, k), c->softness,
                          m3Component(&c->impulse, k), m3Component(&c->perpImpulse, k), b, pass);
        }
    }
}

// A limited angular axis measures its angle as 2 atan2 of its part of
// the relative rotation; the joint contract keeps that well posed (the
// other two axes both locked or both free).
static void SolveAngularLimits(m3JointConstraint* c, const m3JointFrames* f, m3JointBodies* b,
                               const m3JointPass* pass)
{
    m3Quat rel = f->rel;
    m3Vec3 part = {rel.x, rel.y, rel.z};
    for (int32_t k = 0; k < 3; ++k)
    {
        uint32_t mode = AngularMode(c, k);
        m3Vec3 axis = m3FrameAxis(f->a, k);
        m3JointRow row = m3TurnRow(axis);
        if (mode == MODE_LIMITED)
        {
            m3real angle = 2.0f * m3Atan2(*m3Component(&part, k), rel.w);
            m3SolveLimits(&row, angle, *m3Component(&c->genAngLower, k),
                          *m3Component(&c->genAngUpper, k), c->softness,
                          m3Component(&c->angularImpulse, k), &c->limitImpulse.x, b, pass);
        }
    }
}

static void SolveGeneric(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                         const m3JointPass* pass)
{
    m3JointFrames f = m3PoseFrames(c, pose);
    SolveLockedAxes(c, pose, &f, b, pass);
    SolveLinearLimits(c, pose, &f, b, pass);
    SolveAngularLimits(c, &f, b, pass);
    m3JointRow motor;
    if (MotorRow(c, pose, &f, &motor))
    {
        m3real budget = c->maxMotorEffort * pass->h;
        m3SolveRow(&motor, b, m3RigidDrive(-c->motorSpeed), &c->limitImpulse.y, -budget, budget);
    }
}

const m3JointKind m3_genericJointKind = {PrepareGeneric, WarmStartGeneric, SolveGeneric};
