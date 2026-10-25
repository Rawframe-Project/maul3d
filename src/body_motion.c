// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body motion: forces, impulses and velocities. The public functions
// validate and journal; the internal functions apply, and replay drives
// them directly.

#include "body.h"
#include "body_state.h"
#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "solver_math.h"
#include "world_internal.h"

#include <string.h>

// Forces and impulses. One internal per op so replay and the
// public wrappers share exactly one application path.
static bool ForceTargetValid(const m3World* world, int32_t index)
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

// Whether a force or impulse may land: on a dynamic body with mass, the
// accumulated force and torque stay within M3_FORCE_LIMIT, and the
// velocities an impulse leaves stay within M3_SPEED_LIMIT. Each computes
// what its internal above writes, so the public calls and replay refuse
// exactly what would not fit. NaN fails every comparison.
bool m3ForceFits(const m3World* world, int32_t index, m3Vec3 force)
{
    return ForceTargetValid(world, index) &&
           m3WithinV3(m3Add3(world->bodies.bodyForce[index], force), M3_FORCE_LIMIT);
}

bool m3TorqueFits(const m3World* world, int32_t index, m3Vec3 torque)
{
    return ForceTargetValid(world, index) &&
           m3WithinV3(m3Add3(world->bodies.bodyTorque[index], torque), M3_FORCE_LIMIT);
}

bool m3ForceAtPointFits(const m3World* world, int32_t index, m3Vec3 force, m3Pos3 point)
{
    if (!m3FinitePos3(point))
    {
        return false;
    }
    m3Vec3 torque = m3Cross3(ForceArm(world, index, point), force);
    return m3ForceFits(world, index, force) && m3TorqueFits(world, index, torque);
}

bool m3LinearImpulseFits(const m3World* world, int32_t index, m3Vec3 impulse)
{
    if (!ForceTargetValid(world, index) || !m3FiniteV3(impulse))
    {
        return false;
    }
    m3Vec3 v = m3Add3(world->bodies.linearVelocities[index],
                      m3MulSV3(world->bodies.invMass[index], impulse));
    return m3WithinV3(v, M3_SPEED_LIMIT);
}

bool m3AngularImpulseFits(const m3World* world, int32_t index, m3Vec3 impulse)
{
    if (!ForceTargetValid(world, index) || !m3FiniteV3(impulse))
    {
        return false;
    }
    m3Vec3 w = m3Add3(world->bodies.angularVelocities[index],
                      m3MulMV3(m3WorldInvInertia(world, index), impulse));
    return m3WithinV3(w, M3_SPEED_LIMIT);
}

bool m3ImpulseAtPointFits(const m3World* world, int32_t index, m3Vec3 impulse, m3Pos3 point)
{
    if (!m3FinitePos3(point))
    {
        return false;
    }
    m3Vec3 arm = ForceArm(world, index, point);
    return m3LinearImpulseFits(world, index, impulse) &&
           m3AngularImpulseFits(world, index, m3Cross3(arm, impulse));
}

// A kinematic body moves at whatever speed its host or its servo gives
// it, within M3_KINEMATIC_SPEED_LIMIT; every other body within
// M3_SPEED_LIMIT. Every body spins within M3_SPEED_LIMIT.
bool m3BodyVelocityValid(uint8_t type, m3Vec3 velocity)
{
    return m3WithinV3(velocity, type == (uint8_t)m3_kinematicBody ? M3_KINEMATIC_SPEED_LIMIT
                                                                  : M3_SPEED_LIMIT);
}

bool m3BodySpinValid(m3Vec3 velocity)
{
    return m3WithinV3(velocity, M3_SPEED_LIMIT);
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

static M3_NOINLINE void BodyApplyForceWorker(m3World* world, m3BodyId body, m3Vec3 force)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (!m3ForceFits(world, index, force))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVector record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.v = force;
        m3JournalRecord(world, m3_opApplyForce, &record, (int32_t)sizeof(record));
    }
    m3ApplyForceInternal(world, index, force);
}

void m3Body_ApplyForce(m3World* world, m3BodyId body, m3Vec3 force)
{
    uint64_t env = m3FloatEnvEnter();
    BodyApplyForceWorker(world, body, force);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodyApplyTorqueWorker(m3World* world, m3BodyId body, m3Vec3 torque)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (!m3TorqueFits(world, index, torque))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVector record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.v = torque;
        m3JournalRecord(world, m3_opApplyTorque, &record, (int32_t)sizeof(record));
    }
    m3ApplyTorqueInternal(world, index, torque);
}

void m3Body_ApplyTorque(m3World* world, m3BodyId body, m3Vec3 torque)
{
    uint64_t env = m3FloatEnvEnter();
    BodyApplyTorqueWorker(world, body, torque);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodyApplyLinearImpulseWorker(m3World* world, m3BodyId body, m3Vec3 impulse)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (!m3LinearImpulseFits(world, index, impulse))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVector record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.v = impulse;
        m3JournalRecord(world, m3_opApplyLinearImpulse, &record, (int32_t)sizeof(record));
    }
    m3ApplyLinearImpulseInternal(world, index, impulse);
}

void m3Body_ApplyLinearImpulse(m3World* world, m3BodyId body, m3Vec3 impulse)
{
    uint64_t env = m3FloatEnvEnter();
    BodyApplyLinearImpulseWorker(world, body, impulse);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodyApplyAngularImpulseWorker(m3World* world, m3BodyId body, m3Vec3 impulse)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (!m3AngularImpulseFits(world, index, impulse))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVector record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.v = impulse;
        m3JournalRecord(world, m3_opApplyAngularImpulse, &record, (int32_t)sizeof(record));
    }
    m3ApplyAngularImpulseInternal(world, index, impulse);
}

void m3Body_ApplyAngularImpulse(m3World* world, m3BodyId body, m3Vec3 impulse)
{
    uint64_t env = m3FloatEnvEnter();
    BodyApplyAngularImpulseWorker(world, body, impulse);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodyApplyForceAtPointWorker(m3World* world, m3BodyId body, m3Vec3 force,
                                                    m3Pos3 point)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (!m3ForceAtPointFits(world, index, force, point))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVectorAtPoint record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.v = force;
        record.p = point;
        m3JournalRecord(world, m3_opApplyForceAtPoint, &record, (int32_t)sizeof(record));
    }
    m3ApplyForceAtPointInternal(world, index, force, point);
}

void m3Body_ApplyForceAtPoint(m3World* world, m3BodyId body, m3Vec3 force, m3Pos3 point)
{
    uint64_t env = m3FloatEnvEnter();
    BodyApplyForceAtPointWorker(world, body, force, point);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodyApplyLinearImpulseAtPointWorker(m3World* world, m3BodyId body,
                                                            m3Vec3 impulse, m3Pos3 point)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (!m3ImpulseAtPointFits(world, index, impulse, point))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyVectorAtPoint record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.v = impulse;
        record.p = point;
        m3JournalRecord(world, m3_opApplyImpulseAtPoint, &record, (int32_t)sizeof(record));
    }
    m3ApplyImpulseAtPointInternal(world, index, impulse, point);
}

void m3Body_ApplyLinearImpulseAtPoint(m3World* world, m3BodyId body, m3Vec3 impulse, m3Pos3 point)
{
    uint64_t env = m3FloatEnvEnter();
    BodyApplyLinearImpulseAtPointWorker(world, body, impulse, point);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodySetLinearVelocityWorker(m3World* world, m3BodyId body, m3Vec3 velocity)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return; // stale or foreign id: contract, not invariant
    }
    if (!m3BodyVelocityValid(world->bodies.types[index], velocity))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // hostile command: a documented no-op, never poison
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetLinearVelocity record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetLinearVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetLinearVelocityInternal(world, index, velocity);
}

void m3Body_SetLinearVelocity(m3World* world, m3BodyId body, m3Vec3 velocity)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetLinearVelocityWorker(world, body, velocity);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodySetAngularVelocityWorker(m3World* world, m3BodyId body, m3Vec3 velocity)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return; // stale or foreign id: contract, not invariant
    }
    if (!m3BodySpinValid(velocity))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // hostile command: a documented no-op, never poison
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetAngularVelocity record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetAngularVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetAngularVelocityInternal(world, index, velocity);
}

void m3Body_SetAngularVelocity(m3World* world, m3BodyId body, m3Vec3 velocity)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetAngularVelocityWorker(world, body, velocity);
    m3FloatEnvLeave(env);
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

static void SetBodyParam(m3World* world, m3BodyId bodyId, int32_t param, float value)
{
    int32_t index;
    if (!m3ResolveBody(world, bodyId, &index))
    {
        return;
    }
    if (!m3BodyParamValid(param, value))
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

static M3_NOINLINE void BodySetGravityScaleWorker(m3World* world, m3BodyId body, float scale)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    SetBodyParam(world, body, m3_bodyParamGravityScale, scale);
}

void m3Body_SetGravityScale(m3World* world, m3BodyId body, float scale)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetGravityScaleWorker(world, body, scale);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodySetLinearDampingWorker(m3World* world, m3BodyId body, float damping)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    SetBodyParam(world, body, m3_bodyParamLinearDamping, damping);
}

void m3Body_SetLinearDamping(m3World* world, m3BodyId body, float damping)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetLinearDampingWorker(world, body, damping);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodySetAngularDampingWorker(m3World* world, m3BodyId body, float damping)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    SetBodyParam(world, body, m3_bodyParamAngularDamping, damping);
}

void m3Body_SetAngularDamping(m3World* world, m3BodyId body, float damping)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetAngularDampingWorker(world, body, damping);
    m3FloatEnvLeave(env);
}

void m3SetBulletInternal(m3World* world, int32_t index, int bullet)
{
    world->bodies.bulletFlags[index] = bullet ? 1 : 0;
}

static M3_NOINLINE void BodySetBulletWorker(m3World* world, m3BodyId body, bool flag)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.value = flag ? 1 : 0;
        m3JournalRecord(world, m3_opSetBullet, &record, (int32_t)sizeof(record));
    }
    m3SetBulletInternal(world, index, flag ? 1 : 0);
}

void m3Body_SetBullet(m3World* world, m3BodyId body, bool flag)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetBulletWorker(world, body, flag);
    m3FloatEnvLeave(env);
}
