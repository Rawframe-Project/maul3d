// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bodies: creation, destruction, and the journaled setters. The public
// functions validate and journal; the internal functions are the only
// paths that mutate state, and replay drives them directly. World
// resolution from a body id goes by slot (m3WorldFromIndex0): the body
// id carries its own generation, the world slot does not need one.

#include "body.h"
#include "broad_phase.h"
#include "joint.h"
#include "journal.h"
#include "shape.h"
#include "solver.h"
#include "vehicle.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

int32_t m3BodySlot(const m3World* world, m3BodyId bodyId)
{
    int32_t index = bodyId.index1 - 1;
    if (world == NULL || bodyId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->bodies.bodyPool, index, bodyId.generation))
    {
        return -1;
    }
    return index;
}

// Resolve a body id to its world and slot; NULL world or -1 index for
// anything stale or foreign.
static m3World* ResolveBody(m3BodyId bodyId, int32_t* indexOut)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t index = world != NULL ? m3BodySlot(world, bodyId) : -1;
    *indexOut = index;
    if (index < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return NULL;
    }
    return world;
}

m3BodyDef m3DefaultBodyDef(void)
{
    m3BodyDef def;
    memset(&def, 0, sizeof(def));
    def.type = m3_staticBody;
    def.rotation = m3MakeIdentityQuat();
    def.gravityScale = 1.0f;
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
        !m3FinitePos3(def->position) || !m3FiniteQuat(def->rotation) ||
        !m3FiniteV3(def->linearVelocity) || !m3FiniteV3(def->angularVelocity) ||
        !m3FiniteF(def->gravityScale) || !m3FiniteF(def->linearDamping) ||
        !m3FiniteF(def->angularDamping) || def->linearDamping < 0.0f ||
        def->angularDamping < 0.0f || !(qq > 0.98f) || !(qq < 1.02f))
    {
        return -1;
    }
    int32_t index = m3IdPoolAlloc(&world->bodies.bodyPool);
    if (index < 0)
    {
        return -1; // pool exhausted: the caller fails loudly
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
    world->bodies.bodyEnabled[index] = 1;
    world->bodies.bodyLocks[index] = 0;
    world->bodies.bodyIsland[index] = -1; // observer label
    memset(world->bodies.bodyNames + (size_t)index * M3_BODY_NAME_CAPACITY, 0,
           M3_BODY_NAME_CAPACITY);
    world->bodies.bodySleepThreshold[index] = M3_SLEEP_VELOCITY_DEFAULT;
    world->bodies.bodyCanSleep[index] = 1;
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

// Runtime control internals: replay and the wrappers share
// one path each.
void m3WakeRegionAabb(m3World* world, const double lo[3], const double hi[3])
{
    // Wake every sleeping dynamic body whose own extent box
    // overlaps the region (the voxel-edit wake idea without the
    // tree: position plus max extent is a fair, deterministic
    // over-approximation). Serial, ascending slots: canonical.
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    for (int32_t b = 0; b < maxBody; ++b)
    {
        if (world->bodies.bodyPool.alive[b] == 0 ||
            world->bodies.types[b] != (uint8_t)m3_dynamicBody || world->bodies.awake[b] != 0)
        {
            continue;
        }
        double r = (double)world->bodies.maxExtents[b] + (double)M3_AABB_MARGIN;
        const m3Pos3* pp = &world->bodies.transforms[b].p;
        if (pp->x - r <= hi[0] && pp->x + r >= lo[0] && pp->y - r <= hi[1] && pp->y + r >= lo[1] &&
            pp->z - r <= hi[2] && pp->z + r >= lo[2])
        {
            world->bodies.awake[b] = 1;
            world->bodies.sleepTimes[b] = 0.0f;
        }
    }
}

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
    if (type == (uint8_t)m3_staticBody)
    {
        world->bodies.linearVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->bodies.angularVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->bodies.invMass[index] = 0.0f;
        world->bodies.invInertiaLocal[index] = m3MakeZeroMat3();
    }
    else
    {
        // Kinematic keeps zero inverse mass; dynamic rebuilds it
        // from the shapes it carries.
        if (type == (uint8_t)m3_dynamicBody)
        {
            m3RecomputeMass(world, index);
        }
        else
        {
            world->bodies.invMass[index] = 0.0f;
            world->bodies.invInertiaLocal[index] = m3MakeZeroMat3();
        }
    }
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
        v->x = 0.0f;
    if (locks & 2u)
        v->y = 0.0f;
    if (locks & 4u)
        v->z = 0.0f;
    if (locks & 8u)
        w->x = 0.0f;
    if (locks & 16u)
        w->y = 0.0f;
    if (locks & 32u)
        w->z = 0.0f;
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

void m3SetAwakeInternal(m3World* world, int32_t index, int awake)
{
    if (awake)
    {
        world->bodies.awake[index] = 1;
        world->bodies.sleepTimes[index] = 0.0f;
    }
    else
    {
        world->bodies.awake[index] = 0;
        world->bodies.linearVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->bodies.angularVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
        // Every freeze, API-forced ones included, records its
        // cold pairs or the frozen buffer stops being a pure
        // function of the sleeping state.
        m3FreezeDiscoverPairs(world, index);
    }
}

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

m3BodyId m3CreateBody(m3WorldId worldId, const m3BodyDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_BODY_COOKIE ||
        (def->type != m3_staticBody && def->type != m3_kinematicBody &&
         def->type != m3_dynamicBody))
    {
        m3Refuse(world, m3_errorInvalid);
        // Contract, not invariant: a bad def or stale world returns
        // the null id (see m3CreateWorld).
        return m3_nullBodyId;
    }
    // Hostile-input wall: nothing non-finite reaches state,
    // and a rotation that is not close to unit is a corrupted def,
    // not a request (loud refusal beats silent renormalization).
    m3real qq = def->rotation.x * def->rotation.x + def->rotation.y * def->rotation.y +
                def->rotation.z * def->rotation.z + def->rotation.w * def->rotation.w;
    if (!m3FinitePos3(def->position) || !m3FiniteQuat(def->rotation) ||
        !m3FiniteV3(def->linearVelocity) || !m3FiniteV3(def->angularVelocity) ||
        !m3FiniteF(def->gravityScale) || !m3FiniteF(def->linearDamping) ||
        !m3FiniteF(def->angularDamping) || def->linearDamping < 0.0f ||
        def->angularDamping < 0.0f || !(qq > 0.98f) || !(qq < 1.02f))
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullBodyId;
    }
    int32_t index = m3CreateBodyInternal(world, def);
    if (index < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullBodyId;
    }
    m3BodyId id = {index + 1, world->worldIndex0, world->bodies.bodyPool.generations[index]};
    if (world->recorder.journalActive != 0)
    {
        m3OpCreateBody record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateBody, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyBody(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return; // stale or foreign id: contract, not invariant
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyBody, &bodyId, (int32_t)sizeof(bodyId));
    }
    m3DestroyBodyInternal(world, index);
}

bool m3Body_IsValid(m3BodyId bodyId)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    return world != NULL && m3BodySlot(world, bodyId) >= 0;
}

m3Pos3 m3Body_GetPosition(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.transforms[index].p : (m3Pos3){0.0, 0.0, 0.0};
}

m3Quat m3Body_GetRotation(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.transforms[index].q : m3MakeIdentityQuat();
}

m3Vec3 m3Body_GetLinearVelocity(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.linearVelocities[index] : (m3Vec3){0.0f, 0.0f, 0.0f};
}

m3Vec3 m3Body_GetAngularVelocity(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.angularVelocities[index] : (m3Vec3){0.0f, 0.0f, 0.0f};
}

uint64_t m3Body_GetUserData(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.userData[index] : 0;
}

m3BodyType m3Body_GetType(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? (m3BodyType)world->bodies.types[index] : m3_staticBody;
}

// The shared force and impulse wrapper: resolve, refuse hostiles and
// non-dynamic targets quietly, journal, apply through the one
// internal path replay uses.
void m3Body_SetTransform(m3BodyId bodyId, m3Pos3 position, m3Quat rotation)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    m3real qq = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
                rotation.w * rotation.w;
    if (world == NULL || !m3FinitePos3(position) || !m3FiniteQuat(rotation) || !(qq > 0.98f) ||
        !(qq < 1.02f))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    m3Transform pose = {position, rotation};
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyPose record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.pose = pose;
        m3JournalRecord(world, m3_opSetTransform, &record, (int32_t)sizeof(record));
    }
    m3SetTransformInternal(world, index, pose);
}

void m3Body_SetTargetTransform(m3BodyId bodyId, m3Pos3 position, m3Quat rotation)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    m3real qq = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
                rotation.w * rotation.w;
    if (world == NULL || world->bodies.types[index] != (uint8_t)m3_kinematicBody ||
        !m3FinitePos3(position) || !m3FiniteQuat(rotation) || !(qq > 0.98f) || !(qq < 1.02f))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // kinematic bodies only: the servo contract
    }
    m3Transform pose = {position, rotation};
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyPose record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.pose = pose;
        m3JournalRecord(world, m3_opSetTargetTransform, &record, (int32_t)sizeof(record));
    }
    m3SetTargetTransformInternal(world, index, pose);
}

void m3Body_SetType(m3BodyId bodyId, m3BodyType type)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL ||
        (type != m3_staticBody && type != m3_kinematicBody && type != m3_dynamicBody))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.value = (int32_t)type;
        m3JournalRecord(world, m3_opSetType, &record, (int32_t)sizeof(record));
    }
    m3SetTypeInternal(world, index, (uint8_t)type);
}

void m3Body_SetEnabled(m3BodyId bodyId, bool enabled)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL)
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

bool m3Body_IsEnabled(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL && world->bodies.bodyEnabled[index] != 0;
}

void m3Body_SetMotionLocks(m3BodyId bodyId, uint32_t locks)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || (locks & ~0x3Fu) != 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // only the six lock bits exist
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetMotionLocks record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.locks = locks;
        m3JournalRecord(world, m3_opSetMotionLocks, &record, (int32_t)sizeof(record));
    }
    m3SetMotionLocksInternal(world, index, (uint8_t)locks);
}

uint32_t m3Body_GetMotionLocks(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? (uint32_t)(world->bodies.bodyLocks[index] & 0x3Fu) : 0u;
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

void m3Body_SetAllowFastRotation(m3BodyId bodyId, bool allow)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetAllowFastRotation record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.allow = allow ? 1u : 0u;
        m3JournalRecord(world, m3_opSetAllowFastRotation, &record, (int32_t)sizeof(record));
    }
    m3SetAllowFastRotationInternal(world, index, allow ? 1 : 0);
}

bool m3Body_GetAllowFastRotation(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL && (world->bodies.bodyLocks[index] & M3_LOCKS_ALLOW_FAST_ROTATION) != 0;
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

void m3Body_SetName(m3BodyId bodyId, const char* name)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetBodyName record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
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

const char* m3Body_GetName(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.bodyNames + (size_t)index * M3_BODY_NAME_CAPACITY : "";
}

void m3Body_SetSleepControls(m3BodyId bodyId, float threshold, bool canSleep)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteF(threshold) || threshold < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
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

bool m3Body_IsAwake(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL && world->bodies.awake[index] != 0;
}

void m3Body_SetAwake(m3BodyId bodyId, bool awake)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || world->bodies.types[index] != (uint8_t)m3_dynamicBody)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.value = awake ? 1 : 0;
        m3JournalRecord(world, m3_opSetAwake, &record, (int32_t)sizeof(record));
    }
    m3SetAwakeInternal(world, index, awake ? 1 : 0);
}

void m3Body_ApplyForce(m3BodyId bodyId, m3Vec3 force)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
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
    m3World* world = ResolveBody(bodyId, &index);
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
    m3World* world = ResolveBody(bodyId, &index);
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
    m3World* world = ResolveBody(bodyId, &index);
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
    m3World* world = ResolveBody(bodyId, &index);
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
    m3World* world = ResolveBody(bodyId, &index);
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
    m3World* world = ResolveBody(bodyId, &index);
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
    m3World* world = ResolveBody(bodyId, &index);
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
