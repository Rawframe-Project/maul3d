// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Event access: contact, sensor, hit, move, joint break and fragment
// events, the hit threshold and the pre-solve hook.

#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "world.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

void m3SetHitEventThresholdInternal(m3World* world, float value)
{
    world->hitEventThreshold = value;
}

static M3_NOINLINE void WorldSetHitEventThresholdWorker(m3World* world, float value)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (world == NULL || !m3FiniteF(value) || value < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetHitEventThreshold, &value, (int32_t)sizeof(value));
    }
    m3SetHitEventThresholdInternal(world, value);
}

void m3World_SetHitEventThreshold(m3World* world, float value)
{
    uint64_t env = m3FloatEnvEnter();
    WorldSetHitEventThresholdWorker(world, value);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE m3ContactEvents WorldGetContactEventsWorker(const m3World* world)
{
    m3ContactEvents events;
    memset(&events, 0, sizeof(events));
    if (world != NULL)
    {
        events.beginEvents = world->events.beginEvents;
        events.endEvents = world->events.endEvents;
        events.hitEvents = world->events.hitEvents;
        events.beginCount = world->events.beginEventCount;
        events.endCount = world->events.endEventCount;
        events.hitCount = world->events.hitEventCount;
        events.hitsDropped = world->events.hitEventsDropped;
    }
    return events;
}

m3ContactEvents m3World_GetContactEvents(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3ContactEvents result = WorldGetContactEventsWorker(world);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3SensorEvents WorldGetSensorEventsWorker(const m3World* world)
{
    m3SensorEvents events;
    memset(&events, 0, sizeof(events));
    if (world != NULL)
    {
        events.beginEvents = world->events.sensorBeginEvents;
        events.endEvents = world->events.sensorEndEvents;
        events.beginCount = world->events.sensorBeginEventCount;
        events.endCount = world->events.sensorEndEventCount;
    }
    return events;
}

m3SensorEvents m3World_GetSensorEvents(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3SensorEvents result = WorldGetSensorEventsWorker(world);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3FragmentEvents WorldGetFragmentEventsWorker(const m3World* world)
{
    m3FragmentEvents events;
    memset(&events, 0, sizeof(events));
    if (world != NULL)
    {
        events.fragmentEvents = world->events.fragmentEvents;
        events.recipe = world->events.fragmentRecipe;
        events.fragmentCount = world->events.fragmentEventCount;
        events.recipeCount = world->events.fragmentRecipeCount;
        events.fragmentsDropped = world->events.fragmentDropped;
    }
    return events;
}

m3FragmentEvents m3World_GetFragmentEvents(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3FragmentEvents result = WorldGetFragmentEventsWorker(world);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3BodyEvents WorldGetBodyEventsWorker(const m3World* world)
{
    m3BodyEvents events;
    memset(&events, 0, sizeof(events));
    if (world != NULL)
    {
        events.moveEvents = world->events.moveEvents;
        events.moveCount = world->events.moveEventCount;
    }
    return events;
}

m3BodyEvents m3World_GetBodyEvents(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3BodyEvents result = WorldGetBodyEventsWorker(world);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3JointEvents WorldGetJointEventsWorker(const m3World* world)
{
    m3JointEvents events;
    memset(&events, 0, sizeof(events));
    if (world != NULL)
    {
        events.breakEvents = world->joints.jointBreakEvents;
        events.breakCount = world->joints.jointBreakEventCount;
    }
    return events;
}

m3JointEvents m3World_GetJointEvents(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    m3JointEvents result = WorldGetJointEventsWorker(world);
    m3FloatEnvLeave(env);
    return result;
}

void m3AppendJointBreakEvent(m3World* world, m3JointId joint)
{
    // Capacity is jointCapacity: at most every joint breaks once.
    world->joints.jointBreakEvents[world->joints.jointBreakEventCount].jointId = joint;
    world->joints.jointBreakEventCount += 1;
}

static M3_NOINLINE void WorldSetPreSolveCallbackWorker(m3World* world, m3PreSolveFn* fn,
                                                       void* context)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    world->preSolveFn = fn;
    world->preSolveContext = context;
}

void m3World_SetPreSolveCallback(m3World* world, m3PreSolveFn* fn, void* context)
{
    uint64_t env = m3FloatEnvEnter();
    WorldSetPreSolveCallbackWorker(world, fn, context);
    m3FloatEnvLeave(env);
}

void m3ResetStepEvents(m3World* world)
{
    world->events.beginEventCount = 0;
    world->events.endEventCount = 0;
    world->events.sensorBeginEventCount = 0;
    world->events.sensorEndEventCount = 0;
    world->events.fragmentEventCount = 0;
    world->events.fragmentRecipeCount = 0;
    world->events.fragmentDropped = 0;
    world->events.hitEventCount = 0;
    world->events.hitEventsDropped = 0;
    world->events.moveEventCount = 0;
    world->joints.jointBreakEventCount = 0;
}

// A pair that started or stopped touching. A pair whose shape died emits
// nothing: its id would be stale.
static void EmitPairChange(m3World* world, uint64_t key, bool began)
{
    int32_t sA = (int32_t)(key >> 32);
    int32_t sB = (int32_t)(key & 0xFFFFFFFFu);
    if (world->shapes.shapePool.alive[sA] == 0 || world->shapes.shapePool.alive[sB] == 0)
    {
        return;
    }
    m3ShapeId idA = {sA + 1, world->shapes.shapePool.generations[sA]};
    m3ShapeId idB = {sB + 1, world->shapes.shapePool.generations[sB]};
    m3ContactBeginEvent begin = {idA, idB};
    m3ContactEndEvent end = {idA, idB};
    int32_t room = world->contacts.pairCapacity;
    m3Events* ev = &world->events;
    if (world->shapes.shapeSensor[sA] != 0 || world->shapes.shapeSensor[sB] != 0)
    {
        if (began && ev->sensorBeginEventCount < room)
        {
            ev->sensorBeginEvents[ev->sensorBeginEventCount++] = begin;
        }
        else if (!began && ev->sensorEndEventCount < room)
        {
            ev->sensorEndEvents[ev->sensorEndEventCount++] = end;
        }
    }
    else if (began && ev->beginEventCount < room)
    {
        ev->beginEvents[ev->beginEventCount++] = begin;
    }
    else if (!began && ev->endEventCount < room)
    {
        ev->endEvents[ev->endEventCount++] = end;
    }
}

// Serial, after the parallel narrow phase, so events append in pair
// order.
void m3EmitContactEvents(m3World* world, const uint64_t* oldKeys, const m3Manifold* oldManifolds,
                         int32_t oldCount)
{
    int32_t iNew = 0;
    int32_t iOld = 0;
    while (iNew < world->contacts.pairCount || iOld < oldCount)
    {
        uint64_t keyNew =
            iNew < world->contacts.pairCount ? world->contacts.pairKeys[iNew] : UINT64_MAX;
        uint64_t keyOld = iOld < oldCount ? oldKeys[iOld] : UINT64_MAX;
        uint64_t key = keyNew < keyOld ? keyNew : keyOld;
        bool touchNew = false;
        bool touchOld = false;
        if (keyNew == key)
        {
            touchNew = world->contacts.manifolds[iNew].pointCount > 0;
            iNew += 1;
        }
        if (keyOld == key)
        {
            touchOld = oldManifolds[iOld].pointCount > 0;
            iOld += 1;
        }
        if (touchNew != touchOld)
        {
            EmitPairChange(world, key, touchNew);
        }
    }
}

void m3EmitMoveEvents(m3World* world, const int32_t* movers, int32_t moverCount)
{
    // Body move events: one per mover, ascending body order
    // (the mover list is built that way), post-step transform, and
    // fellAsleep on the step the island dropped off. Capacity is
    // bodyCapacity: movers cannot overflow it.
    for (int32_t m = 0; m < moverCount; ++m)
    {
        int32_t i = movers[m];
        m3BodyMoveEvent* e = &world->events.moveEvents[world->events.moveEventCount++];
        e->bodyId = (m3BodyId){i + 1, world->bodies.bodyPool.generations[i]};
        e->transform = world->bodies.transforms[i];
        e->fellAsleep =
            world->bodies.types[i] == (uint8_t)m3_dynamicBody && world->bodies.awake[i] == 0;
    }
}
