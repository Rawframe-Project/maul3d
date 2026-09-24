// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint runtime control: limits, motors, springs, targets, steering and
// breakage, and the joint readback. Public functions validate and
// journal; the internal setters mutate and are what replay drives.

#include "body.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "manifold.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

static void JointWakeBodies(m3World* world, int32_t j)
{
    int32_t bodyA = world->joints.jointBodyA[j];
    int32_t bodyB = world->joints.jointBodyB[j];
    if (bodyA >= 0 && world->bodies.types[bodyA] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, bodyA, 1);
    }
    if (bodyB >= 0 && world->bodies.types[bodyB] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, bodyB, 1);
    }
}

void m3JointSetLimitsInternal(m3World* world, int32_t j, int32_t enable, float lower, float upper)
{
    world->joints.jointFlags[j] = (uint8_t)((world->joints.jointFlags[j] & ~M3_JOINT_LIMIT) |
                                            (enable != 0 ? M3_JOINT_LIMIT : 0u));
    // z carries the spherical cone angle: never clobbered here.
    world->joints.jointLimits[j].x = lower;
    world->joints.jointLimits[j].y = upper;
    // A toggled or moved limit invalidates the stored row impulses
    // (x = lower, y = upper; the spherical's swing rides z and
    // survives only when untouched by this setter's rows).
    world->joints.jointLimitImpulse[j].x = 0.0f;
    world->joints.jointLimitImpulse[j].y = 0.0f;
    JointWakeBodies(world, j);
}

void m3JointSetMotorInternal(m3World* world, int32_t j, int32_t enable, float speed, float effort)
{
    world->joints.jointFlags[j] = (uint8_t)((world->joints.jointFlags[j] & ~M3_JOINT_MOTOR) |
                                            (enable != 0 ? M3_JOINT_MOTOR : 0u));
    world->joints.jointMotor[j].x = speed;
    world->joints.jointMotor[j].y = effort;
    if (world->joints.jointType[j] == (uint8_t)m3_genericJoint)
    {
        world->joints.jointLimitImpulse[j].y = 0.0f; // the generic slot map
    }
    else
    {
        world->joints.jointPerpImpulse[j].z = 0.0f;
    }
    JointWakeBodies(world, j);
}

void m3JointSetSteerInternal(m3World* world, int32_t j, int32_t enable, float target, float hertz,
                             float zeta, float maxEffort)
{
    // The wheel slot map: M3_JOINT_STEER (the cone bit, unused
    // on wheels), target in jointMotor.z, softness and budget in
    // the spherical target slots, warm impulse in spring slot y.
    world->joints.jointFlags[j] = (uint8_t)((world->joints.jointFlags[j] & ~M3_JOINT_STEER) |
                                            (enable != 0 ? M3_JOINT_STEER : 0u));
    world->joints.jointMotor[j].z = target;
    world->joints.jointTargetQ[j].x = hertz;
    world->joints.jointTargetQ[j].y = zeta;
    world->joints.jointTargetQ[j].z = maxEffort;
    if (enable == 0)
    {
        world->joints.jointSpringImpulse[j].y = 0.0f; // the warm slot dies too
    }
    JointWakeBodies(world, j);
}

void m3JointSetMotorPoseInternal(m3World* world, int32_t j, m3Vec3 offset, m3Quat rotation)
{
    // The motor slot map: the target offset rides jointMotor
    // (the servo has no velocity motor), the rotation rides the
    // spherical target slot.
    world->joints.jointMotor[j] = offset;
    world->joints.jointTargetQ[j] = m3NormalizeQuat(rotation);
    JointWakeBodies(world, j);
}

void m3JointSetCollideInternal(m3World* world, int32_t j, int32_t on)
{
    world->joints.jointCollide[j] = on != 0 ? 1 : 0;
    JointWakeBodies(world, j);
}

void m3JointSetBreakInternal(m3World* world, int32_t j, float maxForce, float maxTorque)
{
    world->joints.jointBreak[j] = (m3Vec3){maxForce, maxTorque, 0.0f};
    JointWakeBodies(world, j);
}

// Reaction magnitudes, the per-type assembly documented on the API:
// linear rows fold into force, angular rows into torque, and the
// generic joint reports a conservative sum of its slot map.
void m3JointReactionMagnitudes(const m3World* world, int32_t j, m3real invH, m3real* outForce,
                               m3real* outTorque)
{
    m3Vec3 lin = world->joints.jointImpulse[j];
    m3Vec3 perp = world->joints.jointPerpImpulse[j];
    m3Vec3 lim = world->joints.jointLimitImpulse[j];
    m3Vec3 ang = world->joints.jointAngularImpulse[j];
    m3real force = m3Length3(lin);
    m3real torque = 0.0f;
    switch (world->joints.jointType[j])
    {
    case (uint8_t)m3_parallelJoint:
        // Two angular locks, nothing else.
        *outForce = 0.0f;
        *outTorque = invH * m3Length3((m3Vec3){perp.x, perp.y, 0.0f});
        return;
    case (uint8_t)m3_filterJoint:
        // No rows, no reactions, ever.
        *outForce = 0.0f;
        *outTorque = 0.0f;
        return;
    case (uint8_t)m3_revoluteJoint:
        // Perp x, y are the hinge's cross-axis ANGULAR locks; the
        // motor rides perp.z and the limits ride lim.x, lim.y, all
        // about the hinge axis.
        torque =
            sqrtf(perp.x * perp.x + perp.y * perp.y) + fabsf(perp.z) + fabsf(lim.x) + fabsf(lim.y);
        break;
    case (uint8_t)m3_prismaticJoint:
        // Perp x, y are LINEAR translation locks, the motor and the
        // limits act along the slide axis: all force. The 3-DOF
        // rotation lock is the torque.
        force +=
            sqrtf(perp.x * perp.x + perp.y * perp.y) + fabsf(perp.z) + fabsf(lim.x) + fabsf(lim.y);
        torque = m3Length3(ang);
        break;
    case (uint8_t)m3_sphericalJoint:
        // Twist rows ride lim.x, lim.y; the swing row rides lim.z.
        torque = fabsf(lim.x) + fabsf(lim.y) + fabsf(lim.z);
        break;
    case (uint8_t)m3_distanceJoint:
        force += fabsf(lim.x) + fabsf(lim.y) + fabsf(perp.z);
        break;
    case (uint8_t)m3_genericJoint:
        // The slot map: linear uppers ride perp x, y; the
        // angular upper and the motor ride lim.x, lim.y. The motor
        // may be either kind: fold it into BOTH sums, conservative
        // by construction, documented.
        force += sqrtf(perp.x * perp.x + perp.y * perp.y) + fabsf(lim.x) + fabsf(lim.y);
        torque = m3Length3(ang) + fabsf(lim.x) + fabsf(lim.y);
        break;
    case (uint8_t)m3_wheelJoint:
        // The composed split: lin carries the two point-to-
        // line rows and the suspension limits are linear (force);
        // perp x, y are the axle collinearity locks and the spin
        // motor rides perp.z (torque). The BREAK CONTRACT for an
        // axle: the force cap snaps a wheel torn sideways, the
        // torque cap snaps a drive axle over-driven.
        force += fabsf(lim.x) + fabsf(lim.y);
        torque = sqrtf(perp.x * perp.x + perp.y * perp.y) + fabsf(perp.z);
        break;
    case (uint8_t)m3_gearJoint:
        // One angular row on two axes: report the LARGER
        // side of the mesh, conservative for the break law.
        *outForce = 0.0f;
        *outTorque = invH * fabsf(perp.z) * m3MaxF(1.0f, fabsf(world->joints.jointMotor[j].z));
        return;
    case (uint8_t)m3_pulleyJoint:
        // The rope impulse rides perp.z; the B side carries ratio
        // times it: again the larger side.
        *outForce = invH * fabsf(perp.z) * m3MaxF(1.0f, world->joints.jointMotor[j].z);
        *outTorque = 0.0f;
        return;
    default: // fixed and motor: weld rows
        torque = m3Length3(ang);
        break;
    }
    *outForce = force * invH;
    *outTorque = torque * invH;
}

static m3World* ResolveJoint(m3JointId jointId, int32_t* outSlot)
{
    m3World* world = m3WorldFromTag(jointId.world);
    int32_t slot = world != NULL ? m3JointSlot(world, jointId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return NULL;
    }
    *outSlot = slot;
    return world;
}

void m3Joint_SetLimits(m3JointId jointId, bool enable, float lower, float upper)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || !m3FiniteF(lower) || !m3FiniteF(upper))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->joints.jointType[slot] == (uint8_t)m3_motorJoint)
    {
        // The servo budgets: lower = maxForce, upper =
        // maxTorque, independent allowances rather than a range, so
        // the range order rule does not apply but negatives do.
        if (lower < 0.0f || upper < 0.0f)
        {
            return;
        }
    }
    else if (lower > upper)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointVector record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.enable = enable ? 1 : 0;
        record.a = lower;
        record.b = upper;
        m3JournalRecord(world, m3_opJointSetLimits, &record, (int32_t)sizeof(record));
    }
    m3JointSetLimitsInternal(world, slot, enable ? 1 : 0, lower, upper);
}

void m3Joint_SetSteer(m3JointId jointId, bool enable, float targetAngle, float hertz, float zeta,
                      float maxEffort)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->joints.jointType[slot] != (uint8_t)m3_wheelJoint ||
        !m3FiniteF(targetAngle) || m3AbsF(targetAngle) > 1.0f || !m3FiniteF(hertz) ||
        !m3FiniteF(zeta) || !m3FiniteF(maxEffort) || zeta < 0.0f || maxEffort < 0.0f ||
        (enable && !(hertz > 0.0f)))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // steering is a wheel contract, refused loudly
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetSteer record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.enable = enable ? 1 : 0;
        record.target = targetAngle;
        record.hertz = hertz;
        record.zeta = zeta;
        record.effort = maxEffort;
        m3JournalRecord(world, m3_opJointSetSteer, &record, (int32_t)sizeof(record));
    }
    m3JointSetSteerInternal(world, slot, enable ? 1 : 0, targetAngle, hertz, zeta, maxEffort);
}

float m3Joint_GetSteerAngle(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->joints.jointType[slot] != (uint8_t)m3_wheelJoint)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0.0f;
    }
    int32_t bodyA = world->joints.jointBodyA[slot];
    int32_t bodyB = world->joints.jointBodyB[slot];
    m3Quat quatA = m3MulQuat(world->bodies.transforms[bodyA].q, world->joints.jointFrameQA[slot]);
    m3Quat quatB = m3MulQuat(world->bodies.transforms[bodyB].q, world->joints.jointFrameQB[slot]);
    if (quatA.x * quatB.x + quatA.y * quatB.y + quatA.z * quatB.z + quatA.w * quatB.w < 0.0f)
    {
        quatB = (m3Quat){-quatB.x, -quatB.y, -quatB.z, -quatB.w};
    }
    m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
    m3Quat relQ = m3MulQuat(conjA, quatB);
    return 2.0f * m3Atan2(relQ.x, relQ.w);
}

void m3Joint_SetMotorPose(m3JointId jointId, m3Vec3 offset, m3Quat rotation)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    m3real q2 = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
                rotation.w * rotation.w;
    if (world == NULL || world->joints.jointType[slot] != (uint8_t)m3_motorJoint ||
        !m3FiniteV3(offset) || !m3FiniteQuat(rotation) || q2 < 0.81f || q2 > 1.21f)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // the servo aim is a motor-joint contract
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetMotorPose record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.offset = offset;
        record.rotation = rotation;
        m3JournalRecord(world, m3_opJointSetMotorPose, &record, (int32_t)sizeof(record));
    }
    m3JointSetMotorPoseInternal(world, slot, offset, rotation);
}

void m3Joint_SetMotor(m3JointId jointId, bool enable, float speed, float maxEffort)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || !m3FiniteF(speed) || !m3FiniteF(maxEffort) || maxEffort < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointVector record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.enable = enable ? 1 : 0;
        record.a = speed;
        record.b = maxEffort;
        m3JournalRecord(world, m3_opJointSetMotor, &record, (int32_t)sizeof(record));
    }
    m3JointSetMotorInternal(world, slot, enable ? 1 : 0, speed, maxEffort);
}

void m3Joint_SetCollideConnected(m3JointId jointId, bool collide)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetCollide record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.on = collide ? 1 : 0;
        m3JournalRecord(world, m3_opJointSetCollide, &record, (int32_t)sizeof(record));
    }
    m3JointSetCollideInternal(world, slot, collide ? 1 : 0);
}

bool m3Joint_GetCollideConnected(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    return world != NULL && world->joints.jointCollide[slot] != 0;
}

void m3Joint_SetBreakThresholds(m3JointId jointId, float maxForce, float maxTorque)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || !m3FiniteF(maxForce) || maxForce < 0.0f || !m3FiniteF(maxTorque) ||
        maxTorque < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetBreak record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.maxForce = maxForce;
        record.maxTorque = maxTorque;
        m3JournalRecord(world, m3_opJointSetBreak, &record, (int32_t)sizeof(record));
    }
    m3JointSetBreakInternal(world, slot, maxForce, maxTorque);
}

m3real m3Joint_GetConstraintForce(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->lastInvH == 0.0f)
    {
        return 0.0f;
    }
    m3real force;
    m3real torque;
    m3JointReactionMagnitudes(world, slot, world->lastInvH, &force, &torque);
    return force;
}

m3real m3Joint_GetConstraintTorque(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->lastInvH == 0.0f)
    {
        return 0.0f;
    }
    m3real force;
    m3real torque;
    m3JointReactionMagnitudes(world, slot, world->lastInvH, &force, &torque);
    return torque;
}

m3real m3Joint_GetAngle(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || (world->joints.jointType[slot] != (uint8_t)m3_revoluteJoint &&
                          world->joints.jointType[slot] != (uint8_t)m3_wheelJoint))
    {
        m3Refuse(world, m3_errorInvalid);
        return 0.0f; // the wheel's frame z is its axle: same twist read
    }
    m3Quat qA = m3MulQuat(world->bodies.transforms[world->joints.jointBodyA[slot]].q,
                          world->joints.jointFrameQA[slot]);
    m3Quat qB = m3MulQuat(world->bodies.transforms[world->joints.jointBodyB[slot]].q,
                          world->joints.jointFrameQB[slot]);
    m3Quat conjA = {-qA.x, -qA.y, -qA.z, qA.w};
    m3Quat relQ = m3MulQuat(conjA, qB);
    m3real twist = relQ.w < 0.0f ? m3Atan2(-relQ.z, -relQ.w) : m3Atan2(relQ.z, relQ.w);
    return 2.0f * twist;
}

m3real m3Joint_GetTranslation(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || (world->joints.jointType[slot] != (uint8_t)m3_prismaticJoint &&
                          world->joints.jointType[slot] != (uint8_t)m3_wheelJoint))
    {
        m3Refuse(world, m3_errorInvalid);
        return 0.0f;
    }
    int32_t bodyA = world->joints.jointBodyA[slot];
    int32_t bodyB = world->joints.jointBodyB[slot];
    const m3Transform* xfA = &world->bodies.transforms[bodyA];
    const m3Transform* xfB = &world->bodies.transforms[bodyB];
    m3Vec3 pA = m3RotateVec3(xfA->q, world->joints.jointLocalA[slot]);
    m3Vec3 pB = m3RotateVec3(xfB->q, world->joints.jointLocalB[slot]);
    m3Vec3 d = {(m3real)(xfB->p.x + (double)pB.x - xfA->p.x - (double)pA.x),
                (m3real)(xfB->p.y + (double)pB.y - xfA->p.y - (double)pA.y),
                (m3real)(xfB->p.z + (double)pB.z - xfA->p.z - (double)pA.z)};
    m3Quat frameQ = m3MulQuat(xfA->q, world->joints.jointFrameQA[slot]);
    // The slide axis is frame z for the prismatic and frame x for
    // the wheel (whose z is the axle).
    m3Vec3 local = world->joints.jointType[slot] == (uint8_t)m3_wheelJoint
                       ? (m3Vec3){1.0f, 0.0f, 0.0f}
                       : (m3Vec3){0.0f, 0.0f, 1.0f};
    m3Vec3 axis = m3RotateVec3(frameQ, local);
    return m3Dot3(d, axis);
}

// --- Position drive --------------------------------------------------

void m3JointSetSpringInternal(m3World* world, int32_t j, int32_t enable, float hertz, float zeta)
{
    world->joints.jointFlags[j] = (uint8_t)((world->joints.jointFlags[j] & ~M3_JOINT_SPRING) |
                                            (enable != 0 ? M3_JOINT_SPRING : 0u));
    world->joints.jointSpring[j] = (m3Vec3){hertz, zeta, 0.0f};
    // Any change invalidates the stored spring impulse: a stale
    // warm start toward an old target kicks.
    world->joints.jointSpringImpulse[j] = (m3Vec3){0.0f, 0.0f, 0.0f};
    JointWakeBodies(world, j);
}

void m3JointSetTargetInternal(m3World* world, int32_t j, float scalar, m3Quat q)
{
    world->joints.jointTargetScalar[j] = scalar;
    world->joints.jointTargetQ[j] = q;
    JointWakeBodies(world, j);
}

static int JointTypeDrives(uint8_t type)
{
    return type == (uint8_t)m3_revoluteJoint || type == (uint8_t)m3_prismaticJoint ||
           type == (uint8_t)m3_sphericalJoint || type == (uint8_t)m3_wheelJoint ||
           type == (uint8_t)m3_motorJoint;
}

void m3Joint_SetSpring(m3JointId jointId, bool enable, float hertz, float dampingRatio)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || !JointTypeDrives(world->joints.jointType[slot]) || !m3FiniteF(hertz) ||
        hertz <= 0.0f || !m3FiniteF(dampingRatio) || dampingRatio < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetSpring record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.enable = enable ? 1 : 0;
        record.hertz = hertz;
        record.zeta = dampingRatio;
        m3JournalRecord(world, m3_opJointSetSpring, &record, (int32_t)sizeof(record));
    }
    m3JointSetSpringInternal(world, slot, enable ? 1 : 0, hertz, dampingRatio);
}

static void JointTargetOp(m3World* world, m3JointId jointId, int32_t slot, float scalar, m3Quat q)
{
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetTarget record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.scalar = scalar;
        record.q = q;
        m3JournalRecord(world, m3_opJointSetTarget, &record, (int32_t)sizeof(record));
    }
    m3JointSetTargetInternal(world, slot, scalar, q);
}

void m3Joint_SetTargetAngle(m3JointId jointId, float radians)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->joints.jointType[slot] != (uint8_t)m3_revoluteJoint ||
        !m3FiniteF(radians) || radians < -M3_PI || radians > M3_PI)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    JointTargetOp(world, jointId, slot, radians, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
}

void m3Joint_SetTargetTranslation(m3JointId jointId, float meters)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL ||
        (world->joints.jointType[slot] != (uint8_t)m3_prismaticJoint &&
         world->joints.jointType[slot] != (uint8_t)m3_wheelJoint) ||
        !m3FiniteF(meters))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // the wheel's drive is its suspension spring
    }
    JointTargetOp(world, jointId, slot, meters, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
}

void m3Joint_SetTargetRotation(m3JointId jointId, m3Quat target)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->joints.jointType[slot] != (uint8_t)m3_sphericalJoint ||
        !m3FiniteQuat(target))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    m3real len2 =
        target.x * target.x + target.y * target.y + target.z * target.z + target.w * target.w;
    if (len2 < 0.99f || len2 > 1.01f)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // not a unit rotation: refuse by doing nothing
    }
    JointTargetOp(world, jointId, slot, 0.0f, target);
}
