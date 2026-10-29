// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The wheel joint's steering: its switch, its target angle, stiffness,
// damping and torque budget, and the live steering angle. Public
// functions validate and journal; the internal setter in joint_params.c
// mutates and is what replay drives.

#include "fpenv.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "misuse.h"
#include "world_internal.h"

#include <string.h>

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

// The steering shares one journal record with its four values, so the
// switch and every value setter record the whole record: the part the
// call sets and the rest as the joint holds it.
typedef enum SteerPart
{
    STEER_SWITCH,
    STEER_TARGET,
    STEER_HERTZ,
    STEER_DAMPING,
    STEER_TORQUE,
} SteerPart;

typedef struct SteerRecord
{
    int32_t enable;
    float target;
    float hertz;
    float dampingRatio;
    float maxTorque;
} SteerRecord;

// The wheel slot map: the steer bit in the flags, the target in
// jointMotor.z, stiffness, damping and budget in jointTargetQ.
static SteerRecord SteerHeld(const m3World* world, int32_t slot)
{
    return (SteerRecord){(world->joints.jointFlags[slot] & M3_JOINT_STEER) != 0 ? 1 : 0,
                         world->joints.jointMotor[slot].z, world->joints.jointTargetQ[slot].x,
                         world->joints.jointTargetQ[slot].y, world->joints.jointTargetQ[slot].z};
}

static M3_NOINLINE void WheelJointSetSteering(m3World* world, m3JointId joint, SteerPart part,
                                              bool enable, float value)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    SteerRecord r = SteerHeld(world, slot);
    r.enable = part == STEER_SWITCH ? (enable ? 1 : 0) : r.enable;
    r.target = part == STEER_TARGET ? value : r.target;
    r.hertz = part == STEER_HERTZ ? value : r.hertz;
    r.dampingRatio = part == STEER_DAMPING ? value : r.dampingRatio;
    r.maxTorque = part == STEER_TORQUE ? value : r.maxTorque;
    if (!m3JointSteerFits(world->joints.jointType[slot], r.enable, r.target, r.hertz,
                          r.dampingRatio, r.maxTorque))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // steering is a wheel contract, refused loudly
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointSetSteer record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.enable = r.enable;
        record.target = r.target;
        record.hertz = r.hertz;
        record.zeta = r.dampingRatio;
        record.effort = r.maxTorque;
        m3JournalRecord(world, m3_opJointSetSteer, &record, (int32_t)sizeof(record));
    }
    m3JointSetSteerInternal(world, slot, r.enable, r.target, r.hertz, r.dampingRatio, r.maxTorque);
}

// The steering as the joint holds it; zeros for a stale id and for a
// joint that is not a wheel, each of which counts one misuse.
static SteerRecord WheelJointGetSteering(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return (SteerRecord){0, 0.0f, 0.0f, 0.0f, 0.0f};
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_wheelJoint)
    {
        m3Refuse(world, m3_errorInvalid);
        return (SteerRecord){0, 0.0f, 0.0f, 0.0f, 0.0f};
    }
    return SteerHeld(world, slot);
}

void m3WheelJoint_EnableSteering(m3World* world, m3JointId joint, bool enable)
{
    uint64_t env = m3FloatEnvEnter();
    WheelJointSetSteering(world, joint, STEER_SWITCH, enable, 0);
    m3FloatEnvLeave(env);
}

bool m3WheelJoint_IsSteeringEnabled(const m3World* world, m3JointId joint)
{
    return WheelJointGetSteering(world, joint).enable != 0;
}

void m3WheelJoint_SetTargetSteeringAngle(m3World* world, m3JointId joint, float radians)
{
    uint64_t env = m3FloatEnvEnter();
    WheelJointSetSteering(world, joint, STEER_TARGET, false, radians);
    m3FloatEnvLeave(env);
}

float m3WheelJoint_GetTargetSteeringAngle(const m3World* world, m3JointId joint)
{
    return WheelJointGetSteering(world, joint).target;
}

void m3WheelJoint_SetSteeringHertz(m3World* world, m3JointId joint, float hertz)
{
    uint64_t env = m3FloatEnvEnter();
    WheelJointSetSteering(world, joint, STEER_HERTZ, false, hertz);
    m3FloatEnvLeave(env);
}

float m3WheelJoint_GetSteeringHertz(const m3World* world, m3JointId joint)
{
    return WheelJointGetSteering(world, joint).hertz;
}

void m3WheelJoint_SetSteeringDampingRatio(m3World* world, m3JointId joint, float dampingRatio)
{
    uint64_t env = m3FloatEnvEnter();
    WheelJointSetSteering(world, joint, STEER_DAMPING, false, dampingRatio);
    m3FloatEnvLeave(env);
}

float m3WheelJoint_GetSteeringDampingRatio(const m3World* world, m3JointId joint)
{
    return WheelJointGetSteering(world, joint).dampingRatio;
}

void m3WheelJoint_SetMaxSteeringTorque(m3World* world, m3JointId joint, float maxTorque)
{
    uint64_t env = m3FloatEnvEnter();
    WheelJointSetSteering(world, joint, STEER_TORQUE, false, maxTorque);
    m3FloatEnvLeave(env);
}

float m3WheelJoint_GetMaxSteeringTorque(const m3World* world, m3JointId joint)
{
    return WheelJointGetSteering(world, joint).maxTorque;
}

static M3_NOINLINE float WheelJointGetSteeringAngleWorker(const m3World* world, m3JointId joint)
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

float m3WheelJoint_GetSteeringAngle(const m3World* world, m3JointId joint)
{
    uint64_t env = m3FloatEnvEnter();
    float result = WheelJointGetSteeringAngleWorker(world, joint);
    m3FloatEnvLeave(env);
    return result;
}
