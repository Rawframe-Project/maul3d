// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body motion: forces, impulses and velocities. The public functions
// validate and journal; the internal functions apply, and replay drives
// them directly.

#include "body.h"
#include "journal.h"
#include "solver.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

// Forces and impulses. One internal per op so replay and the
// public wrappers share exactly one application path.
static int ForceTargetValid(m3World* world, int32_t index)
{
    return world->bodies.types[index] == (uint8_t)m3_dynamicBody &&
           world->bodies.invMass[index] > 0.0f;
}

static void ForceWake(m3World* world, int32_t index, m3Vec3 a, m3Vec3 b)
{
    if (a.x != 0.0f || a.y != 0.0f || a.z != 0.0f || b.x != 0.0f || b.y != 0.0f || b.z != 0.0f)
    {
        world->bodies.awake[index] = 1;
        world->bodies.sleepTimes[index] = 0.0f;
    }
}

void m3ApplyForceInternal(m3World* world, int32_t index, m3Vec3 force)
{
    world->bodies.bodyForce[index] = m3Add3(world->bodies.bodyForce[index], force);
    ForceWake(world, index, force, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3ApplyTorqueInternal(m3World* world, int32_t index, m3Vec3 torque)
{
    world->bodies.bodyTorque[index] = m3Add3(world->bodies.bodyTorque[index], torque);
    ForceWake(world, index, torque, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3ApplyLinearImpulseInternal(m3World* world, int32_t index, m3Vec3 impulse)
{
    world->bodies.linearVelocities[index] = m3Add3(world->bodies.linearVelocities[index],
                                                   m3MulSV3(world->bodies.invMass[index], impulse));
    ForceWake(world, index, impulse, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3ApplyAngularImpulseInternal(m3World* world, int32_t index, m3Vec3 impulse)
{
    world->bodies.angularVelocities[index] = m3Add3(
        world->bodies.angularVelocities[index], m3MulMV3(m3WorldInvInertia(world, index), impulse));
    ForceWake(world, index, impulse, (m3Vec3){0.0f, 0.0f, 0.0f});
}

// The arm is measured from the CENTER OF MASS: an application at
// the COM adds no spin, wherever the body origin sits.
static m3Vec3 ForceArm(const m3World* world, int32_t index, m3Pos3 point)
{
    m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[index].q, world->bodies.localCenters[index]);
    return (m3Vec3){(m3real)(point.x - world->bodies.transforms[index].p.x) - rlc.x,
                    (m3real)(point.y - world->bodies.transforms[index].p.y) - rlc.y,
                    (m3real)(point.z - world->bodies.transforms[index].p.z) - rlc.z};
}

void m3ApplyForceAtPointInternal(m3World* world, int32_t index, m3Vec3 force, m3Pos3 point)
{
    world->bodies.bodyForce[index] = m3Add3(world->bodies.bodyForce[index], force);
    world->bodies.bodyTorque[index] =
        m3Add3(world->bodies.bodyTorque[index], m3Cross3(ForceArm(world, index, point), force));
    ForceWake(world, index, force, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3ApplyImpulseAtPointInternal(m3World* world, int32_t index, m3Vec3 impulse, m3Pos3 point)
{
    world->bodies.linearVelocities[index] = m3Add3(world->bodies.linearVelocities[index],
                                                   m3MulSV3(world->bodies.invMass[index], impulse));
    world->bodies.angularVelocities[index] =
        m3Add3(world->bodies.angularVelocities[index],
               m3MulMV3(m3WorldInvInertia(world, index),
                        m3Cross3(ForceArm(world, index, point), impulse)));
    ForceWake(world, index, impulse, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3SetLinearVelocityInternal(m3World* world, int32_t index, m3Vec3 velocity)
{
    world->bodies.awake[index] = 1; // a commanded velocity always wakes
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.linearVelocities[index] = velocity;
}

void m3SetAngularVelocityInternal(m3World* world, int32_t index, m3Vec3 velocity)
{
    world->bodies.awake[index] = 1;
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.angularVelocities[index] = velocity;
}

void m3Body_ApplyForce(m3BodyId bodyId, m3Vec3 force)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(force) || !ForceTargetValid(world, index))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVector record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = force;
        m3JournalRecord(world, m3_opApplyForce, &record, (int32_t)sizeof(record));
    }
    m3ApplyForceInternal(world, index, force);
}

void m3Body_ApplyTorque(m3BodyId bodyId, m3Vec3 torque)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(torque) || !ForceTargetValid(world, index))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVector record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = torque;
        m3JournalRecord(world, m3_opApplyTorque, &record, (int32_t)sizeof(record));
    }
    m3ApplyTorqueInternal(world, index, torque);
}

void m3Body_ApplyLinearImpulse(m3BodyId bodyId, m3Vec3 impulse)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(impulse) || !ForceTargetValid(world, index))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVector record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = impulse;
        m3JournalRecord(world, m3_opApplyLinearImpulse, &record, (int32_t)sizeof(record));
    }
    m3ApplyLinearImpulseInternal(world, index, impulse);
}

void m3Body_ApplyAngularImpulse(m3BodyId bodyId, m3Vec3 impulse)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(impulse) || !ForceTargetValid(world, index))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVector record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = impulse;
        m3JournalRecord(world, m3_opApplyAngularImpulse, &record, (int32_t)sizeof(record));
    }
    m3ApplyAngularImpulseInternal(world, index, impulse);
}

void m3Body_ApplyForceAtPoint(m3BodyId bodyId, m3Vec3 force, m3Pos3 point)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(force) || !m3FinitePos3(point) ||
        !ForceTargetValid(world, index))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVectorAtPoint record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = force;
        record.p = point;
        m3JournalRecord(world, m3_opApplyForceAtPoint, &record, (int32_t)sizeof(record));
    }
    m3ApplyForceAtPointInternal(world, index, force, point);
}

void m3Body_ApplyLinearImpulseAtPoint(m3BodyId bodyId, m3Vec3 impulse, m3Pos3 point)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(impulse) || !m3FinitePos3(point) ||
        !ForceTargetValid(world, index))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVectorAtPoint record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = impulse;
        record.p = point;
        m3JournalRecord(world, m3_opApplyImpulseAtPoint, &record, (int32_t)sizeof(record));
    }
    m3ApplyImpulseAtPointInternal(world, index, impulse, point);
}

void m3Body_SetLinearVelocity(m3BodyId bodyId, m3Vec3 velocity)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return; // stale or foreign id: contract, not invariant
    }
    if (!m3FiniteV3(velocity))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // hostile command: a documented no-op, never poison
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetLinearVelocity record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetLinearVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetLinearVelocityInternal(world, index, velocity);
}

void m3Body_SetAngularVelocity(m3BodyId bodyId, m3Vec3 velocity)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return; // stale or foreign id: contract, not invariant
    }
    if (!m3FiniteV3(velocity))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // hostile command: a documented no-op, never poison
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetAngularVelocity record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetAngularVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetAngularVelocityInternal(world, index, velocity);
}

void m3SetBodyParamInternal(m3World* world, int32_t index, int32_t param, float value)
{
    if (param == m3_bodyParamGravityScale)
    {
        world->bodies.gravityScales[index] = value;
    }
    else if (param == m3_bodyParamLinearDamping)
    {
        world->bodies.linearDamping[index] = value;
    }
    else
    {
        world->bodies.angularDamping[index] = value;
    }
}

static void SetBodyParam(m3BodyId bodyId, int32_t param, float value, bool nonNegative)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return;
    }
    if (!m3FiniteF(value) || (nonNegative && value < 0.0f))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyParam record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.param = param;
        record.value = value;
        m3JournalRecord(world, m3_opSetBodyParam, &record, (int32_t)sizeof(record));
    }
    m3SetBodyParamInternal(world, index, param, value);
}

void m3Body_SetGravityScale(m3BodyId bodyId, float scale)
{
    SetBodyParam(bodyId, m3_bodyParamGravityScale, scale, false);
}

void m3Body_SetLinearDamping(m3BodyId bodyId, float damping)
{
    SetBodyParam(bodyId, m3_bodyParamLinearDamping, damping, true);
}

void m3Body_SetAngularDamping(m3BodyId bodyId, float damping)
{
    SetBodyParam(bodyId, m3_bodyParamAngularDamping, damping, true);
}

void m3SetBulletInternal(m3World* world, int32_t index, int bullet)
{
    world->bodies.bulletFlags[index] = bullet ? 1 : 0;
}

void m3Body_SetBullet(m3BodyId bodyId, bool flag)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.value = flag ? 1 : 0;
        m3JournalRecord(world, m3_opSetBullet, &record, (int32_t)sizeof(record));
    }
    m3SetBulletInternal(world, index, flag ? 1 : 0);
}
