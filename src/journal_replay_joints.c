// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay for joint ops: create, destroy and runtime control.

#include "body_state.h"
#include "joint.h"
#include "journal.h"
#include "journal_replay.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

bool m3ReplayCreateJoint(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateJointOp record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    m3NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableLimit));
    m3NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableMotor));
    m3NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableCone));
    m3NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableConnectedCollision));
    int32_t bodyA = m3BodySlot(world, record.def.bodyIdA);
    int32_t bodyB = m3BodySlot(world, record.def.bodyIdB);
    if (bodyA < 0 || bodyB < 0)
    {
        return false;
    }
    int32_t index = m3CreateJointInternal(world, &record.def, bodyA, bodyB);
    if (index < 0 || (uint32_t)index + 1u != record.expected.index1 ||
        world->joints.jointPool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for joints too
    }
    return true;
}

bool m3ReplayDestroyJoint(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3JointId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    int32_t index = m3JointSlot(world, id);
    if (index < 0)
    {
        return false;
    }
    m3DestroyJointInternal(world, index);
    return true;
}

bool m3ReplayMotorJointSetOffsets(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpMotorJointSetOffsets record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 ||
        !m3MotorJointOffsetsValid(world, slot, record.linearOffset, record.angularOffset))
    {
        return false; // hostile servo bytes fail loudly
    }
    m3MotorJointSetOffsetsInternal(world, slot, record.linearOffset, record.angularOffset);
    return true;
}

bool m3ReplayJointSetSteer(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpJointSetSteer record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || (record.enable != 0 && record.enable != 1) ||
        !m3JointSteerValid(world, slot, record.enable, record.target, record.hertz, record.zeta,
                           record.effort))
    {
        return false; // hostile steer bytes fail loudly
    }
    m3JointSetSteerInternal(world, slot, record.enable, record.target, record.hertz, record.zeta,
                            record.effort);
    return true;
}

bool m3ReplayJointVector(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    m3OpJointVector record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.a) || !m3FiniteF(record.b))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opJointSetLimits)
    {
        uint8_t type = world->joints.jointType[slot];
        if (!(m3JointHasLimits(type) || type == (uint8_t)m3_motorJoint) ||
            m3JointLimitsCheck(type, record.enable, record.a, record.b) != m3_success)
        {
            return false;
        }
        m3JointSetLimitsInternal(world, slot, record.enable, record.a, record.b);
    }
    else
    {
        if (!m3JointHasMotor(world->joints.jointType[slot]) ||
            m3JointMotorCheck(world->joints.jointType[slot], record.enable, record.a, record.b) !=
                m3_success)
        {
            return false;
        }
        m3JointSetMotorInternal(world, slot, record.enable, record.a, record.b);
    }
    return true;
}

bool m3ReplayJointEnableConnectedCollision(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpJointEnableConnectedCollision record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0)
    {
        return false;
    }
    m3JointEnableConnectedCollisionInternal(world, slot, record.on);
    return true;
}

bool m3ReplayJointSetBreak(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpJointSetBreak record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.maxForce) || !m3FiniteF(record.maxTorque) ||
        record.maxForce < 0.0f || record.maxTorque < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3JointSetBreakInternal(world, slot, record.maxForce, record.maxTorque);
    return true;
}

bool m3ReplayJointSetSpring(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpJointSetSpring record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3JointSpringValid(world, slot, record.enable, record.hertz, record.zeta))
    {
        return false; // hostile bytes fail loudly
    }
    m3JointSetSpringInternal(world, slot, record.enable, record.hertz, record.zeta);
    return true;
}

bool m3ReplayJointSetTarget(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpJointSetTarget record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3JointTargetValid(world->joints.jointType[slot], record.scalar, record.q))
    {
        return false; // hostile bytes fail loudly
    }
    m3JointSetTargetInternal(world, slot, record.scalar, record.q);
    return true;
}
