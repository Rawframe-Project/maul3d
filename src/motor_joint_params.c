// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motor joint's runtime calls: its aim and its force and torque
// budgets. Public functions validate and journal; the internal setters in
// joint_params.c mutate and are what replay drives.

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

static M3_NOINLINE void MotorJointSetOffsetsWorker(m3World* world, m3JointId joint,
                                                   m3Vec3 linearOffset, m3Quat angularOffset)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return; // the servo aim is a motor-joint contract
    }
    if (!m3MotorJointOffsetsValid(world, slot, linearOffset, angularOffset))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // the servo aim is a motor-joint contract
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpMotorJointSetOffsets record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.linearOffset = linearOffset;
        record.angularOffset = angularOffset;
        m3JournalRecord(world, m3_opMotorJointSetOffsets, &record, (int32_t)sizeof(record));
    }
    m3MotorJointSetOffsetsInternal(world, slot, linearOffset, angularOffset);
}

void m3MotorJoint_SetOffsets(m3World* world, m3JointId joint, m3Vec3 linearOffset,
                             m3Quat angularOffset)
{
    uint64_t env = m3FloatEnvEnter();
    MotorJointSetOffsetsWorker(world, joint, linearOffset, angularOffset);
    m3FloatEnvLeave(env);
}

// The slot of a motor joint, or -1 for a stale id and for another type,
// each of which counts one misuse.
static int32_t MotorJointSlot(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return -1;
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_motorJoint)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1;
    }
    return slot;
}

m3Vec3 m3MotorJoint_GetLinearOffset(const m3World* world, m3JointId joint)
{
    int32_t slot = MotorJointSlot(world, joint);
    return slot >= 0 ? world->joints.jointMotor[slot] : (m3Vec3){0.0f, 0.0f, 0.0f};
}

m3Quat m3MotorJoint_GetAngularOffset(const m3World* world, m3JointId joint)
{
    int32_t slot = MotorJointSlot(world, joint);
    return slot >= 0 ? world->joints.jointTargetQ[slot] : (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
}

// The budgets ride the motor joint's limit slot, force in x and torque
// in y, and its limit record on the journal.
static M3_NOINLINE void MotorJointSetBudgetWorker(m3World* world, m3JointId joint, int32_t torque,
                                                  float budget)
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
    m3Vec3 budgets = world->joints.jointLimits[slot];
    float maxForce = torque != 0 ? budgets.x : budget;
    float maxTorque = torque != 0 ? budget : budgets.y;
    int32_t enable = (world->joints.jointFlags[slot] & M3_JOINT_LIMIT) != 0 ? 1 : 0;
    if (world->joints.jointType[slot] != (uint8_t)m3_motorJoint ||
        m3JointLimitsCheck(m3_motorJoint, enable, maxForce, maxTorque) != m3_success)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointVector record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.enable = enable;
        record.a = maxForce;
        record.b = maxTorque;
        m3JournalRecord(world, m3_opJointSetLimits, &record, (int32_t)sizeof(record));
    }
    m3JointSetLimitsInternal(world, slot, enable, maxForce, maxTorque);
}

void m3MotorJoint_SetMaxForce(m3World* world, m3JointId joint, float maxForce)
{
    uint64_t env = m3FloatEnvEnter();
    MotorJointSetBudgetWorker(world, joint, 0, maxForce);
    m3FloatEnvLeave(env);
}

float m3MotorJoint_GetMaxForce(const m3World* world, m3JointId joint)
{
    int32_t slot = MotorJointSlot(world, joint);
    return slot >= 0 ? world->joints.jointLimits[slot].x : 0.0f;
}

void m3MotorJoint_SetMaxTorque(m3World* world, m3JointId joint, float maxTorque)
{
    uint64_t env = m3FloatEnvEnter();
    MotorJointSetBudgetWorker(world, joint, 1, maxTorque);
    m3FloatEnvLeave(env);
}

float m3MotorJoint_GetMaxTorque(const m3World* world, m3JointId joint)
{
    int32_t slot = MotorJointSlot(world, joint);
    return slot >= 0 ? world->joints.jointLimits[slot].y : 0.0f;
}
