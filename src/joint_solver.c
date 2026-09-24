// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver's stages. Prepare fills what every joint shares and
// hands the rest to its kind. Each pass then loads the two bodies,
// works out where the substep has taken the joint, lets the kind solve
// its rows against that pose, and stores the dynamic bodies back.
// Joints run serially in slot order.
//
// The angular rows rest on one identity. With the relative rotation
// q = conj(qA) qB and the relative spin u = conj(qA) (wB - wA) qA seen in
// A's frame, dq/dt = (1/2) (0, u) q, and the product expands to
//   d(q.w)/dt = -(1/2) u . v,   d(q.v)/dt = (1/2) (w u + u x v),
// where q = (v, w). Every angle and alignment row below takes its
// gradient from this.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static const m3JointKind s_filterJointKind = {NULL, NULL, NULL};

static const m3JointKind* const s_kinds[] = {
    [m3_sphericalJoint] = &m3_sphericalJointKind, [m3_revoluteJoint] = &m3_revoluteJointKind,
    [m3_prismaticJoint] = &m3_prismaticJointKind, [m3_fixedJoint] = &m3_fixedJointKind,
    [m3_distanceJoint] = &m3_distanceJointKind,   [m3_genericJoint] = &m3_genericJointKind,
    [m3_wheelJoint] = &m3_wheelJointKind,         [m3_filterJoint] = &s_filterJointKind,
    [m3_parallelJoint] = &m3_parallelJointKind,   [m3_motorJoint] = &m3_motorJointKind,
    [m3_gearJoint] = &m3_gearJointKind,           [m3_pulleyJoint] = &m3_pulleyJointKind,
};

m3Softness m3StiffJointSoftness(m3real h)
{
    return m3MakeSoft(M3_JOINT_HERTZ, M3_JOINT_DAMPING_RATIO, h);
}

static bool AwakeDynamic(const m3World* world, int32_t body)
{
    return world->bodies.types[body] == (uint8_t)m3_dynamicBody && world->bodies.awake[body] != 0;
}

static m3real DynamicMass(const m3World* world, int32_t body)
{
    return world->bodies.types[body] == (uint8_t)m3_dynamicBody ? world->bodies.invMass[body]
                                                                : 0.0f;
}

static void PrepareCommon(const m3World* world, m3JointConstraint* c, int32_t j, m3real h,
                          m3JointFrame* frame)
{
    const m3Joints* joints = &world->joints;
    memset(c, 0, sizeof(*c));
    c->joint = j;
    c->type = joints->jointType[j];
    c->flags = joints->jointFlags[j];
    c->bodyA = joints->jointBodyA[j];
    c->bodyB = joints->jointBodyB[j];
    const m3Transform* xfA = &world->bodies.transforms[c->bodyA];
    const m3Transform* xfB = &world->bodies.transforms[c->bodyB];
    c->rA =
        m3RotateVec3(xfA->q, m3Sub3(joints->jointLocalA[j], world->bodies.localCenters[c->bodyA]));
    c->rB =
        m3RotateVec3(xfB->q, m3Sub3(joints->jointLocalB[j], world->bodies.localCenters[c->bodyB]));
    m3Vec3 rlcA = m3RotateVec3(xfA->q, world->bodies.localCenters[c->bodyA]);
    m3Vec3 rlcB = m3RotateVec3(xfB->q, world->bodies.localCenters[c->bodyB]);
    c->deltaCenter = (m3Vec3){(m3real)(xfB->p.x + (double)rlcB.x - xfA->p.x - (double)rlcA.x),
                              (m3real)(xfB->p.y + (double)rlcB.y - xfA->p.y - (double)rlcA.y),
                              (m3real)(xfB->p.z + (double)rlcB.z - xfA->p.z - (double)rlcA.z)};
    c->invMassA = DynamicMass(world, c->bodyA);
    c->invMassB = DynamicMass(world, c->bodyB);
    c->invIA = m3WorldInvInertia(world, c->bodyA);
    c->invIB = m3WorldInvInertia(world, c->bodyB);
    c->frameQA = m3MulQuat(xfA->q, joints->jointFrameQA[j]);
    c->frameQB = m3MulQuat(xfB->q, joints->jointFrameQB[j]);
    c->softness = m3StiffJointSoftness(h);
    if ((c->flags & M3_JOINT_SPRING) != 0)
    {
        c->springSoft = m3MakeSoft(joints->jointSpring[j].x, joints->jointSpring[j].y, h);
    }
    c->motorSpeed = joints->jointMotor[j].x;
    c->maxMotorEffort = joints->jointMotor[j].y;
    c->lowerLimit = joints->jointLimits[j].x;
    c->upperLimit = joints->jointLimits[j].y;
    c->coneAngle = joints->jointLimits[j].z;
    c->target = joints->jointTargetScalar[j];
    c->targetQ = joints->jointTargetQ[j];
    c->impulse = joints->jointImpulse[j];
    c->perpImpulse = joints->jointPerpImpulse[j];
    c->limitImpulse = joints->jointLimitImpulse[j];
    c->angularImpulse = joints->jointAngularImpulse[j];
    c->springImpulse = joints->jointSpringImpulse[j];
    *frame = (m3JointFrame){j, h, xfA, xfB, rlcA, rlcB};
}

int32_t m3PrepareJoints(m3World* world, m3JointConstraint* joints, m3real h)
{
    int32_t count = 0;
    for (int32_t j = 0; j < world->joints.jointPool.maxIndex; ++j)
    {
        if (world->joints.jointPool.alive[j] == 0 ||
            world->joints.jointType[j] == (uint8_t)m3_filterJoint)
        {
            continue; // gone, or the filter joint, which has no rows
        }
        if (!AwakeDynamic(world, world->joints.jointBodyA[j]) &&
            !AwakeDynamic(world, world->joints.jointBodyB[j]))
        {
            continue; // nothing awake to move
        }
        m3JointConstraint* c = &joints[count];
        count += 1;
        m3JointFrame frame;
        PrepareCommon(world, c, j, h, &frame);
        s_kinds[c->type]->prepare(world, c, &frame);
    }
    return count;
}

static m3JointBodies LoadBodies(const m3World* world, const m3JointConstraint* c)
{
    m3JointBodies b = {world->bodies.linearVelocities[c->bodyA],
                       world->bodies.angularVelocities[c->bodyA],
                       world->bodies.linearVelocities[c->bodyB],
                       world->bodies.angularVelocities[c->bodyB],
                       c->invMassA,
                       c->invMassB,
                       c->invIA,
                       c->invIB};
    return b;
}

static void StoreBodies(m3World* world, const m3JointConstraint* c, const m3JointBodies* b)
{
    if (world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody)
    {
        world->bodies.linearVelocities[c->bodyA] = b->vA;
        world->bodies.angularVelocities[c->bodyA] = b->wA;
    }
    if (world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody)
    {
        world->bodies.linearVelocities[c->bodyB] = b->vB;
        world->bodies.angularVelocities[c->bodyB] = b->wB;
    }
}

static m3JointPose CurrentPose(const m3JointConstraint* c, const m3Vec3* deltaPos,
                               const m3Quat* deltaRot)
{
    m3JointPose pose;
    pose.moveA = deltaPos[c->bodyA];
    pose.moveB = deltaPos[c->bodyB];
    pose.turnA = deltaRot[c->bodyA];
    pose.turnB = deltaRot[c->bodyB];
    pose.armA = m3RotateVec3(pose.turnA, c->rA);
    pose.armB = m3RotateVec3(pose.turnB, c->rB);
    pose.gap = m3Add3(m3Add3(m3Sub3(pose.moveB, pose.moveA), m3Sub3(pose.armB, pose.armA)),
                      c->deltaCenter);
    return pose;
}

void m3WarmStartJoints(m3World* world, m3JointConstraint* joints, int32_t count,
                       const m3Vec3* deltaPos, const m3Quat* deltaRot)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3JointConstraint* c = &joints[i];
        m3JointPose pose = CurrentPose(c, deltaPos, deltaRot);
        m3JointBodies b = LoadBodies(world, c);
        s_kinds[c->type]->warmStart(c, &pose, &b);
        StoreBodies(world, c, &b);
    }
}

void m3SolveJoints(m3World* world, m3JointConstraint* joints, int32_t count, const m3Vec3* deltaPos,
                   const m3Quat* deltaRot, m3real h, m3real invH, bool biased)
{
    m3JointPass pass = {biased, h, invH};
    for (int32_t i = 0; i < count; ++i)
    {
        m3JointConstraint* c = &joints[i];
        m3JointPose pose = CurrentPose(c, deltaPos, deltaRot);
        m3JointBodies b = LoadBodies(world, c);
        s_kinds[c->type]->solve(c, &pose, &b, &pass);
        StoreBodies(world, c, &b);
    }
}

void m3StoreJointImpulses(m3World* world, m3JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        const m3JointConstraint* c = &joints[i];
        world->joints.jointImpulse[c->joint] = c->impulse;
        world->joints.jointPerpImpulse[c->joint] = c->perpImpulse;
        world->joints.jointLimitImpulse[c->joint] = c->limitImpulse;
        world->joints.jointAngularImpulse[c->joint] = c->angularImpulse;
        world->joints.jointSpringImpulse[c->joint] = c->springImpulse;
    }
}

// --- Shared by the kinds -----------------------------------------------------

m3Vec3 m3FrameAxis(m3Quat frame, int32_t k)
{
    m3Vec3 e = {k == 0 ? 1.0f : 0.0f, k == 1 ? 1.0f : 0.0f, k == 2 ? 1.0f : 0.0f};
    return m3RotateVec3(frame, e);
}

m3JointFrames m3PoseFrames(const m3JointConstraint* c, const m3JointPose* pose)
{
    m3JointFrames f;
    f.a = m3MulQuat(pose->turnA, c->frameQA);
    f.b = m3MulQuat(pose->turnB, c->frameQB);
    m3Quat conjA = {-f.a.x, -f.a.y, -f.a.z, f.a.w};
    f.rel = m3MulQuat(conjA, f.b);
    if (f.rel.w < 0.0f)
    {
        f.b = (m3Quat){-f.b.x, -f.b.y, -f.b.z, -f.b.w};
        f.rel = (m3Quat){-f.rel.x, -f.rel.y, -f.rel.z, -f.rel.w};
    }
    return f;
}

m3JointRow m3SlideRow(const m3JointPose* pose, m3Vec3 axis)
{
    return m3LineRow(m3Add3(pose->armA, pose->gap), pose->armB, axis);
}

// From the identity above, the part of d(q.v)/dt along e is
// (1/2) u . (w e + v x e); in the world that is the gradient below.
m3Vec3 m3RelativeGradient(const m3JointFrames* f, int32_t k)
{
    m3Vec3 e = {k == 0 ? 1.0f : 0.0f, k == 1 ? 1.0f : 0.0f, k == 2 ? 1.0f : 0.0f};
    m3Vec3 v = {f->rel.x, f->rel.y, f->rel.z};
    m3Vec3 local = m3Add3(m3MulSV3(f->rel.w, e), m3Cross3(v, e));
    return m3MulSV3(0.5f, m3RotateVec3(f->a, local));
}

m3Vec3 m3RotationVector(m3Quat q)
{
    if (q.w < 0.0f)
    {
        q = (m3Quat){-q.x, -q.y, -q.z, -q.w};
    }
    m3Vec3 v = {q.x, q.y, q.z};
    m3real length = sqrtf(m3Dot3(v, v));
    return length < 1.0e-9f ? m3MulSV3(2.0f, v) : m3MulSV3(2.0f * m3Atan2(length, q.w) / length, v);
}

// E = q conj(target) obeys the same identity as q, so its rotation
// vector, turned into the world by A's frame, changes at wB - wA.
m3Vec3 m3RotationError(const m3JointFrames* f, m3Quat target)
{
    m3Quat conjT = {-target.x, -target.y, -target.z, target.w};
    return m3RotateVec3(f->a, m3RotationVector(m3MulQuat(f->rel, conjT)));
}

void m3SolveLimits(const m3JointRow* row, m3real value, m3real lower, m3real upper, m3Softness soft,
                   m3real* lowerImpulse, m3real* upperImpulse, m3JointBodies* b,
                   const m3JointPass* pass)
{
    m3RowDrive low = m3LimitDrive(soft, value - lower, pass->invH, pass->biased);
    m3SolveRow(row, b, low, lowerImpulse, 0.0f, M3_ROW_FREE);
    m3JointRow back = m3ScaleRow(*row, -1.0f);
    m3RowDrive high = m3LimitDrive(soft, upper - value, pass->invH, pass->biased);
    m3SolveRow(&back, b, high, upperImpulse, 0.0f, M3_ROW_FREE);
}

void m3WarmStartLimits(const m3JointRow* row, m3real lowerImpulse, m3real upperImpulse,
                       m3JointBodies* b)
{
    m3PushRow(row, b, lowerImpulse - upperImpulse);
}

m3Vec3 m3BlockBias(m3RowDrive unit, m3Vec3 C)
{
    return m3MulSV3(unit.bias, C);
}

m3real* m3Component(m3Vec3* v, int32_t k)
{
    return k == 0 ? &v->x : (k == 1 ? &v->y : &v->z);
}

void m3SolveDrive(const m3JointRow* row, m3real value, const m3JointConstraint* c, m3JointBodies* b,
                  const m3JointPass* pass, m3real* spring, m3real* motor)
{
    bool motorOn = (c->flags & M3_JOINT_MOTOR) != 0;
    m3real budget = c->maxMotorEffort * pass->h;
    if ((c->flags & M3_JOINT_SPRING) != 0)
    {
        m3real room = motorOn ? m3MaxF(budget - m3AbsF(*motor), 0.0f) : M3_ROW_FREE;
        m3SolveRow(row, b, m3SpringDrive(c->springSoft, value - c->target), spring, -room, room);
    }
    if (motorOn)
    {
        m3SolveRow(row, b, m3RigidDrive(-c->motorSpeed), motor, -budget, budget);
    }
}

void m3SolveAlignment(const m3JointFrames* f, m3JointBodies* b, m3Softness soft, bool biased,
                      m3real* first, m3real* second)
{
    m3JointRow rows[2] = {m3TurnRow(m3RelativeGradient(f, 0)), m3TurnRow(m3RelativeGradient(f, 1))};
    m3RowDrive unit = m3HeldDrive(soft, 1.0f, biased);
    m3real bias[2] = {unit.bias * f->rel.x, unit.bias * f->rel.y};
    m3SolveRowPair(rows, b, bias, unit, first, second);
}

void m3WarmStartAlignment(const m3JointFrames* f, m3JointBodies* b, m3real first, m3real second)
{
    m3JointRow x = m3TurnRow(m3RelativeGradient(f, 0));
    m3JointRow y = m3TurnRow(m3RelativeGradient(f, 1));
    m3PushRow(&x, b, first);
    m3PushRow(&y, b, second);
}

void m3SolveJointPoint(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                       const m3JointPass* pass)
{
    m3RowDrive unit = m3HeldDrive(c->softness, 1.0f, pass->biased);
    m3SolvePointBlock(pose->armA, pose->armB, b, m3BlockBias(unit, pose->gap), unit, &c->impulse,
                      M3_ROW_FREE);
}
