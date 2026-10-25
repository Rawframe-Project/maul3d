// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint runtime control: limits, motors, springs, targets, steering and
// breakage, and the joint readback. Public functions validate and
// journal; the internal setters mutate and are what replay drives.

#include "body_state.h"
#include "fpenv.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "misuse.h"
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

static bool ResolveJoint(const m3World* world, m3JointId id, int32_t* slotOut)
{
    int32_t slot = m3JointSlot(world, id);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    *slotOut = slot;
    return true;
}

static M3_NOINLINE void JointSetLimitsWorker(m3World* world, m3JointId joint, bool enable,
                                             float lower, float upper)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (!m3JointLimitsValid(world->joints.jointType[slot], lower, upper))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointVector record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.enable = enable ? 1 : 0;
        record.a = lower;
        record.b = upper;
        m3JournalRecord(world, m3_opJointSetLimits, &record, (int32_t)sizeof(record));
    }
    m3JointSetLimitsInternal(world, slot, enable ? 1 : 0, lower, upper);
}

void m3Joint_SetLimits(m3World* world, m3JointId joint, bool enable, float lower, float upper)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetLimitsWorker(world, joint, enable, lower, upper);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void JointSetSteerWorker(m3World* world, m3JointId joint, bool enable,
                                            float targetAngle, float hertz, float zeta,
                                            float maxEffort)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return; // steering is a wheel contract, refused loudly
    }
    if (!m3JointSteerValid(world, slot, enable ? 1 : 0, targetAngle, hertz, zeta, maxEffort))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // steering is a wheel contract, refused loudly
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetSteer record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.enable = enable ? 1 : 0;
        record.target = targetAngle;
        record.hertz = hertz;
        record.zeta = zeta;
        record.effort = maxEffort;
        m3JournalRecord(world, m3_opJointSetSteer, &record, (int32_t)sizeof(record));
    }
    m3JointSetSteerInternal(world, slot, enable ? 1 : 0, targetAngle, hertz, zeta, maxEffort);
}

void m3Joint_SetSteer(m3World* world, m3JointId joint, bool enable, float targetAngle, float hertz,
                      float zeta, float maxEffort)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetSteerWorker(world, joint, enable, targetAngle, hertz, zeta, maxEffort);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE float JointGetSteerAngleWorker(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return 0.0f;
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_wheelJoint)
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

float m3Joint_GetSteerAngle(const m3World* world, m3JointId joint)
{
    uint64_t env = m3FloatEnvEnter();
    float result = JointGetSteerAngleWorker(world, joint);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE void JointSetMotorPoseWorker(m3World* world, m3JointId joint, m3Vec3 offset,
                                                m3Quat rotation)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return; // the servo aim is a motor-joint contract
    }
    if (!m3JointMotorPoseValid(world, slot, offset, rotation))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // the servo aim is a motor-joint contract
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetMotorPose record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.offset = offset;
        record.rotation = rotation;
        m3JournalRecord(world, m3_opJointSetMotorPose, &record, (int32_t)sizeof(record));
    }
    m3JointSetMotorPoseInternal(world, slot, offset, rotation);
}

void m3Joint_SetMotorPose(m3World* world, m3JointId joint, m3Vec3 offset, m3Quat rotation)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetMotorPoseWorker(world, joint, offset, rotation);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void JointSetMotorWorker(m3World* world, m3JointId joint, bool enable,
                                            float speed, float maxEffort)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (!m3JointMotorValid(world->joints.jointType[slot], enable ? 1 : 0, speed, maxEffort))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointVector record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.enable = enable ? 1 : 0;
        record.a = speed;
        record.b = maxEffort;
        m3JournalRecord(world, m3_opJointSetMotor, &record, (int32_t)sizeof(record));
    }
    m3JointSetMotorInternal(world, slot, enable ? 1 : 0, speed, maxEffort);
}

void m3Joint_SetMotor(m3World* world, m3JointId joint, bool enable, float speed, float maxEffort)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetMotorWorker(world, joint, enable, speed, maxEffort);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void JointSetCollideConnectedWorker(m3World* world, m3JointId joint,
                                                       bool collide)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetCollide record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.on = collide ? 1 : 0;
        m3JournalRecord(world, m3_opJointSetCollide, &record, (int32_t)sizeof(record));
    }
    m3JointSetCollideInternal(world, slot, collide ? 1 : 0);
}

void m3Joint_SetCollideConnected(m3World* world, m3JointId joint, bool collide)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetCollideConnectedWorker(world, joint, collide);
    m3FloatEnvLeave(env);
}

bool m3Joint_GetCollideConnected(const m3World* world, m3JointId joint)
{
    int32_t slot;
    return ResolveJoint(world, joint, &slot) && world->joints.jointCollide[slot] != 0;
}

static M3_NOINLINE void JointSetBreakThresholdsWorker(m3World* world, m3JointId joint,
                                                      float maxForce, float maxTorque)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (!m3FiniteF(maxForce) || maxForce < 0.0f || !m3FiniteF(maxTorque) || maxTorque < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetBreak record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.maxForce = maxForce;
        record.maxTorque = maxTorque;
        m3JournalRecord(world, m3_opJointSetBreak, &record, (int32_t)sizeof(record));
    }
    m3JointSetBreakInternal(world, slot, maxForce, maxTorque);
}

void m3Joint_SetBreakThresholds(m3World* world, m3JointId joint, float maxForce, float maxTorque)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetBreakThresholdsWorker(world, joint, maxForce, maxTorque);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE m3real JointGetConstraintForceWorker(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot) || world->lastInvH == 0.0f)
    {
        return 0.0f;
    }
    m3real force;
    m3real torque;
    m3JointReactionMagnitudes(world, slot, world->lastInvH, &force, &torque);
    return force;
}

m3real m3Joint_GetConstraintForce(const m3World* world, m3JointId joint)
{
    uint64_t env = m3FloatEnvEnter();
    m3real result = JointGetConstraintForceWorker(world, joint);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3real JointGetConstraintTorqueWorker(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot) || world->lastInvH == 0.0f)
    {
        return 0.0f;
    }
    m3real force;
    m3real torque;
    m3JointReactionMagnitudes(world, slot, world->lastInvH, &force, &torque);
    return torque;
}

m3real m3Joint_GetConstraintTorque(const m3World* world, m3JointId joint)
{
    uint64_t env = m3FloatEnvEnter();
    m3real result = JointGetConstraintTorqueWorker(world, joint);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3real JointGetAngleWorker(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return 0.0f; // the wheel's frame z is its axle: same twist read
    }
    if ((world->joints.jointType[slot] != (uint8_t)m3_revoluteJoint &&
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

m3real m3Joint_GetAngle(const m3World* world, m3JointId joint)
{
    uint64_t env = m3FloatEnvEnter();
    m3real result = JointGetAngleWorker(world, joint);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3real JointGetTranslationWorker(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return 0.0f;
    }
    if ((world->joints.jointType[slot] != (uint8_t)m3_prismaticJoint &&
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

m3real m3Joint_GetTranslation(const m3World* world, m3JointId joint)
{
    uint64_t env = m3FloatEnvEnter();
    m3real result = JointGetTranslationWorker(world, joint);
    m3FloatEnvLeave(env);
    return result;
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

bool m3JointMotorValid(int32_t type, int32_t enable, float speed, float maxEffort)
{
    if (type == m3_distanceJoint) // the spring's hertz and damping ratio
    {
        return speed >= 0.0f && speed <= M3_HERTZ_LIMIT && (enable == 0 || speed > 0.0f) &&
               maxEffort >= 0.0f && maxEffort <= M3_RATIO_LIMIT; // false for NaN
    }
    return m3WithinF(speed, M3_SPEED_LIMIT) && maxEffort >= 0.0f &&
           maxEffort <= M3_FORCE_LIMIT; // false for NaN
}

bool m3JointLimitsValid(int32_t type, float lower, float upper)
{
    if (type == m3_motorJoint)
    {
        // The servo budgets: lower = maxForce, upper = maxTorque,
        // independent allowances rather than a range.
        return lower >= 0.0f && lower <= M3_FORCE_LIMIT && upper >= 0.0f &&
               upper <= M3_FORCE_LIMIT; // false for NaN
    }
    return m3FiniteF(lower) && m3FiniteF(upper) && lower <= upper;
}

bool m3JointSpringValid(const m3World* world, int32_t slot, float hertz, float zeta)
{
    return m3JointSpringFits(world->joints.jointType[slot], hertz, zeta);
}

bool m3JointSpringFits(uint8_t type, float hertz, float zeta)
{
    return JointTypeDrives(type) && hertz > 0.0f && hertz <= M3_HERTZ_LIMIT && zeta >= 0.0f &&
           zeta <= M3_RATIO_LIMIT;
}

bool m3JointSteerValid(const m3World* world, int32_t slot, int32_t enable, float target,
                       float hertz, float zeta, float maxEffort)
{
    return m3JointSteerFits(world->joints.jointType[slot], enable, target, hertz, zeta, maxEffort);
}

// Steering is a wheel contract: a target within one radian, and a
// positive frequency while it is on.
bool m3JointSteerFits(uint8_t type, int32_t enable, float target, float hertz, float zeta,
                      float maxEffort)
{
    return type == (uint8_t)m3_wheelJoint && m3WithinF(target, 1.0f) && hertz >= 0.0f &&
           hertz <= M3_HERTZ_LIMIT && (enable == 0 || hertz > 0.0f) && zeta >= 0.0f &&
           zeta <= M3_RATIO_LIMIT && maxEffort >= 0.0f && maxEffort <= M3_FORCE_LIMIT;
}

bool m3JointMotorPoseValid(const m3World* world, int32_t slot, m3Vec3 offset, m3Quat rotation)
{
    m3real q2 = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
                rotation.w * rotation.w;
    return world->joints.jointType[slot] == (uint8_t)m3_motorJoint && m3LocalV3(offset) &&
           m3FiniteQuat(rotation) && q2 >= 0.81f && q2 <= 1.21f;
}

static M3_NOINLINE void JointSetSpringWorker(m3World* world, m3JointId joint, bool enable,
                                             float hertz, float dampingRatio)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (!m3JointSpringValid(world, slot, hertz, dampingRatio))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetSpring record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.enable = enable ? 1 : 0;
        record.hertz = hertz;
        record.zeta = dampingRatio;
        m3JournalRecord(world, m3_opJointSetSpring, &record, (int32_t)sizeof(record));
    }
    m3JointSetSpringInternal(world, slot, enable ? 1 : 0, hertz, dampingRatio);
}

void m3Joint_SetSpring(m3World* world, m3JointId joint, bool enable, float hertz,
                       float dampingRatio)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetSpringWorker(world, joint, enable, hertz, dampingRatio);
    m3FloatEnvLeave(env);
}

bool m3JointTargetValid(uint8_t type, float scalar, m3Quat q)
{
    bool identity = q.x == 0.0f && q.y == 0.0f && q.z == 0.0f && q.w == 1.0f;
    if (type == (uint8_t)m3_revoluteJoint)
    {
        return identity && m3FiniteF(scalar) && scalar >= -M3_PI && scalar <= M3_PI;
    }
    if (type == (uint8_t)m3_prismaticJoint || type == (uint8_t)m3_wheelJoint)
    {
        return identity && m3FiniteF(scalar);
    }
    if (type != (uint8_t)m3_sphericalJoint || scalar != 0.0f || !m3FiniteQuat(q))
    {
        return false;
    }
    m3real len2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return len2 >= 0.99f && len2 <= 1.01f;
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

static M3_NOINLINE void JointSetTargetAngleWorker(m3World* world, m3JointId joint, float radians)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_revoluteJoint ||
        !m3JointTargetValid(world->joints.jointType[slot], radians,
                            (m3Quat){0.0f, 0.0f, 0.0f, 1.0f}))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    JointTargetOp(world, joint, slot, radians, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
}

void m3Joint_SetTargetAngle(m3World* world, m3JointId joint, float radians)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetTargetAngleWorker(world, joint, radians);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void JointSetTargetTranslationWorker(m3World* world, m3JointId joint,
                                                        float meters)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return; // the wheel's drive is its suspension spring
    }
    if ((world->joints.jointType[slot] != (uint8_t)m3_prismaticJoint &&
         world->joints.jointType[slot] != (uint8_t)m3_wheelJoint) ||
        !m3JointTargetValid(world->joints.jointType[slot], meters,
                            (m3Quat){0.0f, 0.0f, 0.0f, 1.0f}))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // the wheel's drive is its suspension spring
    }
    JointTargetOp(world, joint, slot, meters, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
}

void m3Joint_SetTargetTranslation(m3World* world, m3JointId joint, float meters)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetTargetTranslationWorker(world, joint, meters);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void JointSetTargetRotationWorker(m3World* world, m3JointId joint, m3Quat target)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_sphericalJoint ||
        !m3JointTargetValid(world->joints.jointType[slot], 0.0f, target))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // not a unit rotation: refuse by doing nothing
    }
    JointTargetOp(world, joint, slot, 0.0f, target);
}

void m3Joint_SetTargetRotation(m3World* world, m3JointId joint, m3Quat target)
{
    uint64_t env = m3FloatEnvEnter();
    JointSetTargetRotationWorker(world, joint, target);
    m3FloatEnvLeave(env);
}
