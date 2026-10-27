// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bodies: creation, destruction, and the journaled setters. The public
// functions validate and journal; the internal functions are the only
// paths that mutate state, and replay drives them directly. World
// resolution from a body id goes by slot (m3WorldFromIndex0): the body
// id carries its own generation, the world slot does not need one.

#include "body.h"

#include "body_state.h"
#include "broad_phase.h"
#include "fpenv.h"
#include "joint.h"
#include "journal.h"
#include "misuse.h"
#include "shape.h"
#include "vehicle.h"
#include "world_internal.h"

#include <string.h>

m3BodyDef m3DefaultBodyDef(void)
{
    m3BodyDef def;
    memset(&def, 0, sizeof(def));
    def.type = m3_staticBody;
    def.rotation = m3MakeIdentityQuat();
    def.gravityScale = 1.0f;
    def.enableSleep = true;
    def.isEnabled = true;
    def.internalValue = M3_BODY_COOKIE;
    return def;
}

int32_t m3CreateBodyInternal(m3World* world, const m3BodyDef* def)
{
    // Input checks live here because replay hands this function raw
    // journal bytes: nothing non-finite reaches state, a
    // rotation far from unit is a corrupted def, and a type byte
    // outside the enum must never mint a body.
    m3real qq = def->rotation.x * def->rotation.x + def->rotation.y * def->rotation.y +
                def->rotation.z * def->rotation.z + def->rotation.w * def->rotation.w;
    if ((def->type != m3_staticBody && def->type != m3_kinematicBody &&
         def->type != m3_dynamicBody) ||
        !m3BoundedPos3(def->position) || !m3FiniteQuat(def->rotation) ||
        !m3BodyVelocityValid((uint8_t)def->type, def->linearVelocity) ||
        !m3BodySpinValid(def->angularVelocity) ||
        !m3BodyParamValid(m3_bodyParamGravityScale, def->gravityScale) ||
        !m3FiniteF(def->linearDamping) || !m3FiniteF(def->angularDamping) ||
        def->linearDamping < 0.0f || def->angularDamping < 0.0f || !(qq > 0.98f) || !(qq < 1.02f) ||
        !m3FiniteF(def->sleepThreshold) || def->sleepThreshold < 0.0f)
    {
        return M3_CREATE_INVALID;
    }
    int32_t index = m3IdPoolAlloc(&world->bodies.bodyPool);
    if (index < 0)
    {
        return M3_CREATE_FULL; // pool exhausted: the caller fails loudly
    }
    world->bodies.transforms[index].p = def->position;
    world->bodies.transforms[index].q = m3NormalizeQuat(def->rotation);
    world->bodies.linearVelocities[index] = def->linearVelocity;
    world->bodies.angularVelocities[index] = def->angularVelocity;
    // A shapeless dynamic body has unit mass and zero inertia; its
    // shapes replace this when they are added.
    world->bodies.invMass[index] = def->type == m3_dynamicBody ? 1.0f : 0.0f;
    world->bodies.invInertiaLocal[index] = m3MakeZeroMat3();
    world->bodies.inertiaLocal[index] = m3MakeZeroMat3();
    world->bodies.bulletFlags[index] = 0;
    world->bodies.awake[index] = 1;
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.minExtents[index] = 1.0e30f;
    world->bodies.maxExtents[index] = 0.0f;
    world->bodies.localCenters[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodies.gravityScales[index] = def->gravityScale;
    world->bodies.linearDamping[index] = def->linearDamping;
    world->bodies.angularDamping[index] = def->angularDamping;
    world->bodies.types[index] = (uint8_t)def->type;
    world->bodies.bulletFlags[index] = def->isBullet ? 1 : 0;
    world->bodies.userData[index] = def->userData;
    world->bodies.bodyEnabled[index] = def->isEnabled ? 1 : 0;
    world->bodies.bodyLocks[index] =
        (uint8_t)(m3LockBits(def->motionLocks) |
                  (def->enableFastRotation ? M3_LOCKS_ALLOW_FAST_ROTATION : 0u));
    world->bodies.bodyIsland[index] = -1; // observer label
    memset(world->bodies.bodyNames + (size_t)index * M3_BODY_NAME_CAPACITY, 0,
           M3_BODY_NAME_CAPACITY);
    world->bodies.bodySleepThreshold[index] =
        def->sleepThreshold > 0.0f ? def->sleepThreshold : M3_SLEEP_VELOCITY_DEFAULT;
    world->bodies.bodyCanSleep[index] = def->enableSleep ? 1 : 0;
    world->bodies.bodyHasTarget[index] = 0;
    world->bodies.bodyTarget[index] = (m3Transform){{0.0, 0.0, 0.0}, {0.0f, 0.0f, 0.0f, 1.0f}};
    world->bodies.bodyShapeHead[index] = -1;
    return index;
}

void m3DestroyBodyInternal(m3World* world, int32_t index)
{
    // Destroy attached joints first (the cascade, like shapes).
    while (world->joints.bodyJointHead[index] != -1)
    {
        m3DestroyJointInternal(world, world->joints.bodyJointHead[index]);
    }
    // Cascade: a body takes its shapes with it (each destroy unlinks
    // the list head, so this drains deterministically).
    while (world->bodies.bodyShapeHead[index] != -1)
    {
        m3DestroyShapeInternal(world, world->bodies.bodyShapeHead[index]);
    }
    // Zero the slot so recycled state can never leak into a new body
    // or into the snapshot bytes.
    world->bodies.transforms[index] = (m3Transform){{0.0, 0.0, 0.0}, {0.0f, 0.0f, 0.0f, 1.0f}};
    world->bodies.linearVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodies.angularVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodies.invMass[index] = 0.0f;
    world->bodies.invInertiaLocal[index] = m3MakeZeroMat3();
    world->bodies.inertiaLocal[index] = m3MakeZeroMat3();
    world->bodies.bulletFlags[index] = 0;
    world->bodies.awake[index] = 1;
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.minExtents[index] = 1.0e30f;
    world->bodies.maxExtents[index] = 0.0f;
    world->bodies.localCenters[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodies.gravityScales[index] = 0.0f;
    world->bodies.linearDamping[index] = 0.0f;
    world->bodies.angularDamping[index] = 0.0f;
    world->bodies.types[index] = 0;
    world->bodies.userData[index] = 0;
    world->bodies.bodyForce[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodies.bodyTorque[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodies.bodyEnabled[index] = 0;
    world->bodies.bodyLocks[index] = 0;
    world->bodies.bodyIsland[index] = -1; // observer label
    memset(world->bodies.bodyNames + (size_t)index * M3_BODY_NAME_CAPACITY, 0,
           M3_BODY_NAME_CAPACITY);
    world->bodies.bodySleepThreshold[index] = 0.0f;
    world->bodies.bodyCanSleep[index] = 0;
    world->bodies.bodyHasTarget[index] = 0;
    world->bodies.bodyTarget[index] = (m3Transform){{0.0, 0.0, 0.0}, {0.0f, 0.0f, 0.0f, 1.0f}};
    // Characters standing on this body lose their ground reference
    // NOW: the generation guard would catch a recycled slot,
    // but a cleared reference never even asks.
    for (int32_t c = 0; c < world->characters.charPool.maxIndex; ++c)
    {
        if (world->characters.charPool.alive[c] != 0 &&
            world->characters.charGroundBody[c] == index)
        {
            world->characters.charGroundBody[c] = -1;
            world->characters.charGroundGen[c] = 0;
        }
    }
    // A chassis takes its vehicle with it: the cascade rule,
    // same as shapes and joints, in ascending slot order.
    for (int32_t v = 0; v < world->vehicles.vehPool.maxIndex; ++v)
    {
        if (world->vehicles.vehPool.alive[v] != 0 && world->vehicles.vehChassis[v] == index)
        {
            m3DestroyVehicleInternal(world, v);
        }
    }
    m3IdPoolFree(&world->bodies.bodyPool, index);
}

bool m3PoseValid(m3Transform pose)
{
    m3Quat q = pose.q;
    m3real qq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return m3BoundedPos3(pose.p) && m3FiniteQuat(q) && qq > 0.98f && qq < 1.02f;
}

// A servo target lands in one step at the distance over dt, so it must lie
// within M3_SERVO_REACH of where the kinematic body is now: over the
// shortest step that is M3_KINEMATIC_SPEED_LIMIT. Its turn, at most 2 / dt,
// stays within M3_SPEED_LIMIT.
bool m3TargetValid(const m3World* world, int32_t index, m3Transform pose)
{
    return m3TargetFits(world->bodies.types[index], world->bodies.transforms[index].p, pose);
}

bool m3TargetFits(uint8_t type, m3Pos3 now, m3Transform pose)
{
    if (type != (uint8_t)m3_kinematicBody || !m3PoseValid(pose))
    {
        return false;
    }
    double dx = pose.p.x - now.x;
    double dy = pose.p.y - now.y;
    double dz = pose.p.z - now.z;
    double limit = (double)M3_SERVO_REACH;
    return dx * dx + dy * dy + dz * dz <= limit * limit;
}

// A type change keeps the linear velocity within the new type's limit
// (turning static zeroes it), keeps static-only scenery (a plane, mesh,
// height field or voxel chunk) static, as its create required, holds a
// body turning dynamic to the mass bounds a dynamic body's shapes meet, and
// keeps every joint on the body within its lever ratio at the mass the
// new type gives it.
bool m3TypeChangeValid(const m3World* world, int32_t index, int32_t type)
{
    if (type < (int32_t)m3_staticBody || type > (int32_t)m3_dynamicBody ||
        m3CharacterOwnsBody(world, index) || !m3BodyTypeFits(world, index, (uint8_t)type))
    {
        return false;
    }
    if (type == (int32_t)m3_staticBody)
    {
        return true;
    }
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        uint8_t shape = world->shapes.shapeType[s];
        if (shape == (uint8_t)m3_planeShape || shape == (uint8_t)m3_meshShape ||
            shape == (uint8_t)m3_heightFieldShape || shape == (uint8_t)m3_voxelShape)
        {
            return false;
        }
    }
    if (type == (int32_t)m3_dynamicBody && world->bodies.types[index] != (uint8_t)m3_dynamicBody &&
        !m3BodyMassFits(world, index))
    {
        return false;
    }
    return m3BodyVelocityValid((uint8_t)type, world->bodies.linearVelocities[index]);
}

bool m3SleepThresholdValid(float threshold)
{
    return m3FiniteF(threshold) && threshold >= 0.0f;
}

bool m3BodyParamValid(int32_t param, float value)
{
    if (param == m3_bodyParamGravityScale)
    {
        return m3WithinF(value, M3_RATIO_LIMIT);
    }
    return value >= 0.0f && m3FiniteF(value); // damping
}

// Runtime control internals: replay and the wrappers share
// one path each.
static void BodyRegion(m3World* world, int32_t index, double lo[3], double hi[3])
{
    // A generous body region: position plus max extent plus margin.
    double r = (double)world->bodies.maxExtents[index] + 4.0 * (double)M3_AABB_MARGIN;
    lo[0] = world->bodies.transforms[index].p.x - r;
    lo[1] = world->bodies.transforms[index].p.y - r;
    lo[2] = world->bodies.transforms[index].p.z - r;
    hi[0] = world->bodies.transforms[index].p.x + r;
    hi[1] = world->bodies.transforms[index].p.y + r;
    hi[2] = world->bodies.transforms[index].p.z + r;
}

void m3SetTransformInternal(m3World* world, int32_t index, m3Transform pose)
{
    double lo[3];
    double hi[3];
    BodyRegion(world, index, lo, hi);
    m3WakeRegionAabb(world, lo, hi); // the vacated neighborhood
    world->bodies.transforms[index] = pose;
    BodyRegion(world, index, lo, hi);
    m3WakeRegionAabb(world, lo, hi); // the arrival neighborhood
    world->bodies.awake[index] = 1;
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.bodyHasTarget[index] = 0; // a teleport cancels a pending target
}

void m3SetTargetTransformInternal(m3World* world, int32_t index, m3Transform pose)
{
    world->bodies.bodyHasTarget[index] = 1;
    world->bodies.bodyTarget[index] = pose;
    world->bodies.awake[index] = 1;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m3SetTypeInternal(m3World* world, int32_t index, uint8_t type)
{
    if (world->bodies.types[index] == type)
    {
        return;
    }
    double lo[3];
    double hi[3];
    BodyRegion(world, index, lo, hi);
    m3WakeRegionAabb(world, lo, hi);
    world->bodies.types[index] = type;
    world->bodies.bodyHasTarget[index] = 0; // a type change cancels a pending target
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        if (world->broadphase.proxyIds[s] != M3_TREE_NULL)
        {
            m3TreeSetMask(&world->broadphase.tree, world->broadphase.proxyIds[s],
                          m3ProxyMask(world, s));
            world->broadphase.moved[s] = 1; // static pairs come and go
        }
    }
    if (type == (uint8_t)m3_staticBody)
    {
        world->bodies.linearVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->bodies.angularVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    // Mass, center and extents are a function of the type and the shapes:
    // a dynamic body rebuilds them, any other has none.
    m3RecomputeMass(world, index);
    world->bodies.awake[index] = 1;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m3SetEnabledInternal(m3World* world, int32_t index, int enabled)
{
    uint8_t want = enabled ? 1 : 0;
    if (world->bodies.bodyEnabled[index] == want)
    {
        return;
    }
    world->bodies.bodyEnabled[index] = want;
    double lo[3];
    double hi[3];
    BodyRegion(world, index, lo, hi);
    m3WakeRegionAabb(world, lo, hi);
    if (want)
    {
        world->bodies.awake[index] = 1;
        world->bodies.sleepTimes[index] = 0.0f;
    }
}

void m3SetMotionLocksInternal(m3World* world, int32_t index, uint8_t locks)
{
    // Bit 6 is allowFastRotation, owned by its own op; a
    // locks write must not clobber it.
    world->bodies.bodyLocks[index] =
        (uint8_t)((world->bodies.bodyLocks[index] & M3_LOCKS_ALLOW_FAST_ROTATION) |
                  (locks & 0x3Fu));
    m3Vec3* v = &world->bodies.linearVelocities[index];
    m3Vec3* w = &world->bodies.angularVelocities[index];
    if (locks & 1u)
    {
        v->x = 0.0f;
    }
    if (locks & 2u)
    {
        v->y = 0.0f;
    }
    if (locks & 4u)
    {
        v->z = 0.0f;
    }
    if (locks & 8u)
    {
        w->x = 0.0f;
    }
    if (locks & 16u)
    {
        w->y = 0.0f;
    }
    if (locks & 32u)
    {
        w->z = 0.0f;
    }
}

void m3SetSleepControlsInternal(m3World* world, int32_t index, float threshold, int canSleep)
{
    world->bodies.bodySleepThreshold[index] =
        threshold > 0.0f ? threshold : M3_SLEEP_VELOCITY_DEFAULT;
    world->bodies.bodyCanSleep[index] = canSleep ? 1 : 0;
    if (!canSleep)
    {
        world->bodies.awake[index] = 1;
        world->bodies.sleepTimes[index] = 0.0f;
    }
}

static M3_NOINLINE m3Result CreateBodyWorker(m3World* world, const m3BodyDef* def,
                                             m3BodyId* bodyOut)
{
    if (bodyOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *bodyOut = m3_nullBodyId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (world == NULL || def == NULL || def->internalValue != M3_BODY_COOKIE)
    {
        // Contract, not invariant: a bad def or stale world returns
        // the null id (see m3CreateWorld). The rest of the def is
        // checked once, inside m3CreateBodyInternal.
        return m3Refuse(world, m3_errorInvalid);
    }
    int32_t index = m3CreateBodyInternal(world, def);
    if (index < 0)
    {
        return m3CreateRefusal(world, index);
    }
    m3BodyId id = {index + 1, world->bodies.bodyPool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        m3OpCreateBody record;
        memset(&record, 0, sizeof(record));
        m3JournalBodyDef(&record.def, def);
        record.expected = id;
        m3JournalRecord(world, m3_opCreateBody, &record, (int32_t)sizeof(record));
    }
    *bodyOut = id;
    return m3_success;
}

m3Result m3CreateBody(m3World* world, const m3BodyDef* def, m3BodyId* bodyOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateBodyWorker(world, def, bodyOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result DestroyBodyWorker(m3World* world, m3BodyId body)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    int32_t index = m3BodySlot(world, body);
    if (index < 0)
    {
        return m3Refuse(world, m3MissingId(body.index1));
    }
    if (m3CharacterOwnsBody(world, index))
    {
        return m3Refuse(world, m3_errorInvalid); // destroy the character
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyBody, &body, (int32_t)sizeof(body));
    }
    m3DestroyBodyInternal(world, index);
    return m3_success;
}

m3Result m3DestroyBody(m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = DestroyBodyWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

bool m3Body_IsValid(const m3World* world, m3BodyId body)
{
    return m3BodySlot(world, body) >= 0;
}

static M3_NOINLINE m3Pos3 BodyGetPositionWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.transforms[index].p
                                              : (m3Pos3){0.0, 0.0, 0.0};
}

m3Pos3 m3Body_GetPosition(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Pos3 result = BodyGetPositionWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Quat BodyGetRotationWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.transforms[index].q
                                              : m3MakeIdentityQuat();
}

m3Quat m3Body_GetRotation(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Quat result = BodyGetRotationWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Vec3 BodyGetLinearVelocityWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.linearVelocities[index]
                                              : (m3Vec3){0.0f, 0.0f, 0.0f};
}

m3Vec3 m3Body_GetLinearVelocity(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = BodyGetLinearVelocityWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Vec3 BodyGetAngularVelocityWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.angularVelocities[index]
                                              : (m3Vec3){0.0f, 0.0f, 0.0f};
}

m3Vec3 m3Body_GetAngularVelocity(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = BodyGetAngularVelocityWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

uint64_t m3Body_GetUserData(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.userData[index] : 0;
}

m3BodyType m3Body_GetType(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? (m3BodyType)world->bodies.types[index]
                                              : m3_staticBody;
}

// The shared force and impulse wrapper: resolve, refuse hostiles and
// non-dynamic targets quietly, journal, apply through the one
// internal path replay uses.
static M3_NOINLINE void BodySetTransformWorker(m3World* world, m3BodyId body, m3Pos3 position,
                                               m3Quat rotation)
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
    m3Transform pose = {position, rotation};
    if (!m3PoseValid(pose))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyPose record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.pose = pose;
        m3JournalRecord(world, m3_opSetTransform, &record, (int32_t)sizeof(record));
    }
    m3SetTransformInternal(world, index, pose);
}

void m3Body_SetTransform(m3World* world, m3BodyId body, m3Pos3 position, m3Quat rotation)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetTransformWorker(world, body, position, rotation);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodySetTargetTransformWorker(m3World* world, m3BodyId body, m3Pos3 position,
                                                     m3Quat rotation)
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
    m3Transform pose = {position, rotation};
    if (!m3TargetValid(world, index, pose))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // kinematic bodies only, within reach: the servo contract
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyPose record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.pose = pose;
        m3JournalRecord(world, m3_opSetTargetTransform, &record, (int32_t)sizeof(record));
    }
    m3SetTargetTransformInternal(world, index, pose);
}

void m3Body_SetTargetTransform(m3World* world, m3BodyId body, m3Pos3 position, m3Quat rotation)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetTargetTransformWorker(world, body, position, rotation);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodySetTypeWorker(m3World* world, m3BodyId body, m3BodyType type)
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
    if (!m3TypeChangeValid(world, index, (int32_t)type))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.value = (int32_t)type;
        m3JournalRecord(world, m3_opSetType, &record, (int32_t)sizeof(record));
    }
    m3SetTypeInternal(world, index, (uint8_t)type);
}

void m3Body_SetType(m3World* world, m3BodyId body, m3BodyType type)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetTypeWorker(world, body, type);
    m3FloatEnvLeave(env);
}

static void SetEnabled(m3World* world, m3BodyId bodyId, bool enabled)
{
    int32_t index;
    if (!m3ResolveBody(world, bodyId, &index))
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.value = enabled ? 1 : 0;
        m3JournalRecord(world, m3_opSetEnabled, &record, (int32_t)sizeof(record));
    }
    m3SetEnabledInternal(world, index, enabled ? 1 : 0);
}

static M3_NOINLINE void BodyEnableWorker(m3World* world, m3BodyId body)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    SetEnabled(world, body, true);
}

void m3Body_Enable(m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    BodyEnableWorker(world, body);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void BodyDisableWorker(m3World* world, m3BodyId body)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    SetEnabled(world, body, false);
}

void m3Body_Disable(m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    BodyDisableWorker(world, body);
    m3FloatEnvLeave(env);
}

bool m3Body_IsEnabled(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) && world->bodies.bodyEnabled[index] != 0;
}

uint8_t m3LockBits(m3MotionLocks locks)
{
    return (uint8_t)((locks.linearX ? 0x01u : 0u) | (locks.linearY ? 0x02u : 0u) |
                     (locks.linearZ ? 0x04u : 0u) | (locks.angularX ? 0x08u : 0u) |
                     (locks.angularY ? 0x10u : 0u) | (locks.angularZ ? 0x20u : 0u));
}

static M3_NOINLINE void BodySetMotionLocksWorker(m3World* world, m3BodyId body, m3MotionLocks locks)
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
    uint8_t bits = m3LockBits(locks);
    if (world->recorder.journalActive != 0)
    {
        m3OpSetMotionLocks record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.locks = bits;
        m3JournalRecord(world, m3_opSetMotionLocks, &record, (int32_t)sizeof(record));
    }
    m3SetMotionLocksInternal(world, index, bits);
}

void m3Body_SetMotionLocks(m3World* world, m3BodyId body, m3MotionLocks locks)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetMotionLocksWorker(world, body, locks);
    m3FloatEnvLeave(env);
}

m3MotionLocks m3Body_GetMotionLocks(const m3World* world, m3BodyId body)
{
    int32_t index;
    uint8_t bits = m3ResolveBody(world, body, &index) ? world->bodies.bodyLocks[index] : 0u;
    m3MotionLocks locks = {(bits & 0x01u) != 0, (bits & 0x02u) != 0, (bits & 0x04u) != 0,
                           (bits & 0x08u) != 0, (bits & 0x10u) != 0, (bits & 0x20u) != 0};
    return locks;
}

void m3SetAllowFastRotationInternal(m3World* world, int32_t index, int32_t allow)
{
    if (allow != 0)
    {
        world->bodies.bodyLocks[index] |= (uint8_t)M3_LOCKS_ALLOW_FAST_ROTATION;
    }
    else
    {
        world->bodies.bodyLocks[index] =
            (uint8_t)(world->bodies.bodyLocks[index] & ~M3_LOCKS_ALLOW_FAST_ROTATION);
    }
}

static M3_NOINLINE void BodyEnableFastRotationWorker(m3World* world, m3BodyId body, bool flag)
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
        m3OpSetAllowFastRotation record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.allow = flag ? 1u : 0u;
        m3JournalRecord(world, m3_opSetAllowFastRotation, &record, (int32_t)sizeof(record));
    }
    m3SetAllowFastRotationInternal(world, index, flag ? 1 : 0);
}

void m3Body_EnableFastRotation(m3World* world, m3BodyId body, bool flag)
{
    uint64_t env = m3FloatEnvEnter();
    BodyEnableFastRotationWorker(world, body, flag);
    m3FloatEnvLeave(env);
}

bool m3Body_IsFastRotationEnabled(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) &&
           (world->bodies.bodyLocks[index] & M3_LOCKS_ALLOW_FAST_ROTATION) != 0;
}

void m3SetBodyNameInternal(m3World* world, int32_t index, const char* name)
{
    char* slot = world->bodies.bodyNames + (size_t)index * M3_BODY_NAME_CAPACITY;
    memset(slot, 0, M3_BODY_NAME_CAPACITY);
    if (name != NULL)
    {
        // Truncation is silent and the terminator is forced: names
        // are debug labels, not data.
        for (int32_t i = 0; i < M3_BODY_NAME_CAPACITY - 1 && name[i] != 0; ++i)
        {
            slot[i] = name[i];
        }
    }
}

void m3Body_SetName(m3World* world, m3BodyId body, const char* name)
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
        m3OpSetBodyName record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        if (name != NULL)
        {
            for (int32_t i = 0; i < M3_BODY_NAME_CAPACITY - 1 && name[i] != 0; ++i)
            {
                record.name[i] = name[i];
            }
        }
        m3JournalRecord(world, m3_opSetBodyName, &record, (int32_t)sizeof(record));
    }
    m3SetBodyNameInternal(world, index, name);
}

const char* m3Body_GetName(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index)
               ? world->bodies.bodyNames + (size_t)index * M3_BODY_NAME_CAPACITY
               : "";
}

// Both sleep settings travel in one journal op.
static void SetSleepControls(m3World* world, m3BodyId bodyId, int32_t index, float threshold,
                             bool canSleep)
{
    if (world->recorder.journalActive != 0)
    {
        m3OpSetSleepControls record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.threshold = threshold;
        record.canSleep = canSleep ? 1 : 0;
        m3JournalRecord(world, m3_opSetSleepControls, &record, (int32_t)sizeof(record));
    }
    m3SetSleepControlsInternal(world, index, threshold, canSleep ? 1 : 0);
}

static M3_NOINLINE void BodyEnableSleepWorker(m3World* world, m3BodyId body, bool flag)
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
    SetSleepControls(world, body, index, world->bodies.bodySleepThreshold[index], flag);
}

void m3Body_EnableSleep(m3World* world, m3BodyId body, bool flag)
{
    uint64_t env = m3FloatEnvEnter();
    BodyEnableSleepWorker(world, body, flag);
    m3FloatEnvLeave(env);
}

bool m3Body_IsSleepEnabled(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) && world->bodies.bodyCanSleep[index] != 0;
}

static M3_NOINLINE void BodySetSleepThresholdWorker(m3World* world, m3BodyId body, float threshold)
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
    if (!m3SleepThresholdValid(threshold))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    SetSleepControls(world, body, index, threshold, world->bodies.bodyCanSleep[index] != 0);
}

void m3Body_SetSleepThreshold(m3World* world, m3BodyId body, float threshold)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetSleepThresholdWorker(world, body, threshold);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE float BodyGetSleepThresholdWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.bodySleepThreshold[index] : 0.0f;
}

float m3Body_GetSleepThreshold(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    float result = BodyGetSleepThresholdWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

bool m3Body_IsAwake(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) && world->bodies.awake[index] != 0;
}

static M3_NOINLINE void BodySetAwakeWorker(m3World* world, m3BodyId body, bool awake)
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
    if (world->bodies.types[index] != (uint8_t)m3_dynamicBody)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.value = awake ? 1 : 0;
        m3JournalRecord(world, m3_opSetAwake, &record, (int32_t)sizeof(record));
    }
    m3SetAwakeInternal(world, index, awake ? 1 : 0);
}

void m3Body_SetAwake(m3World* world, m3BodyId body, bool awake)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetAwakeWorker(world, body, awake);
    m3FloatEnvLeave(env);
}

void m3Body_SetUserData(m3World* world, m3BodyId body, uint64_t userData)
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
        m3OpUserData record;
        memset(&record, 0, sizeof(record));
        record.userData = userData;
        record.id = body;
        m3JournalRecord(world, m3_opSetBodyUserData, &record, (int32_t)sizeof(record));
    }
    world->bodies.userData[index] = userData;
}
