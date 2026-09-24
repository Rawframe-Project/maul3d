// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay for joint ops: create, destroy and runtime control.

#include "journal_replay.h"

#include "body.h"
#include "character.h"
#include "joint.h"
#include "journal.h"
#include "query.h"
#include "quickhull.h"
#include "shape.h"
#include "softbody.h"
#include "solver.h"
#include "vehicle.h"
#include "voxel.h"
#include "world.h"
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
    m3NormalizeBoolByte(&record.def, offsetof(m3JointDef, collideConnected));
    record.def.bodyIdA.world = world->idWorld;
    record.def.bodyIdB.world = world->idWorld;
    int32_t bodyA = m3BodySlot(world, record.def.bodyIdA);
    int32_t bodyB = m3BodySlot(world, record.def.bodyIdB);
    if (bodyA < 0 || bodyB < 0)
    {
        return false;
    }
    int32_t index = m3CreateJointInternal(world, &record.def, bodyA, bodyB);
    if (index < 0 || index + 1 != record.expected.index1 ||
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
    id.world = world->idWorld;
    int32_t index = m3JointSlot(world, id);
    if (index < 0)
    {
        return false;
    }
    m3DestroyJointInternal(world, index);
    return true;
}

bool m3ReplayJointSetMotorPose(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpJointSetMotorPose record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3JointSlot(world, record.id);
    float q2 = record.rotation.x * record.rotation.x + record.rotation.y * record.rotation.y +
               record.rotation.z * record.rotation.z + record.rotation.w * record.rotation.w;
    if (slot < 0 || world->joints.jointType[slot] != (uint8_t)m3_motorJoint ||
        !m3FiniteV3(record.offset) || !m3FiniteQuat(record.rotation) || q2 < 0.81f || q2 > 1.21f)
    {
        return false; // hostile servo bytes fail loudly
    }
    m3JointSetMotorPoseInternal(world, slot, record.offset, record.rotation);
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
    record.id.world = world->idWorld;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || world->joints.jointType[slot] != (uint8_t)m3_wheelJoint ||
        !m3FiniteF(record.target) || m3AbsF(record.target) > 1.0f || !m3FiniteF(record.hertz) ||
        !m3FiniteF(record.zeta) || !m3FiniteF(record.effort) || record.zeta < 0.0f ||
        record.effort < 0.0f || (record.enable != 0 && !(record.hertz > 0.0f)) ||
        (record.enable != 0 && record.enable != 1))
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
    record.id.world = world->idWorld;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.a) || !m3FiniteF(record.b))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opJointSetLimits)
    {
        // Mirror the public contract: the motor joint's
        // budgets are independent nonnegatives, every other
        // type wants an ordered range.
        if (world->joints.jointType[slot] == (uint8_t)m3_motorJoint
                ? (record.a < 0.0f || record.b < 0.0f)
                : record.a > record.b)
        {
            return false;
        }
        m3JointSetLimitsInternal(world, slot, record.enable, record.a, record.b);
    }
    else
    {
        m3JointSetMotorInternal(world, slot, record.enable, record.a, record.b);
    }
    return true;
}

bool m3ReplayJointSetCollide(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpJointSetCollide record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0)
    {
        return false;
    }
    m3JointSetCollideInternal(world, slot, record.on);
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
    record.id.world = world->idWorld;
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
    record.id.world = world->idWorld;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.hertz) || record.hertz <= 0.0f || !m3FiniteF(record.zeta) ||
        record.zeta < 0.0f)
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
    record.id.world = world->idWorld;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.scalar) || !m3FiniteQuat(record.q))
    {
        return false; // hostile bytes fail loudly
    }
    m3JointSetTargetInternal(world, slot, record.scalar, record.q);
    return true;
}
