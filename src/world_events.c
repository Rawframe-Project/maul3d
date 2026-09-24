// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Event access: contact, sensor, hit, move, joint break and fragment
// events, the hit threshold and the pre-solve hook.

#include "body.h"
#include "journal.h"
#include "query.h"
#include "world.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

void m3SetHitEventThresholdInternal(m3World* world, float value)
{
    world->hitEventThreshold = value;
}

void m3World_SetHitEventThreshold(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
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

m3ContactEvents m3World_GetContactEvents(m3WorldId worldId)
{
    m3ContactEvents events;
    memset(&events, 0, sizeof(events));
    m3World* world = m3WorldFromId(worldId);
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

m3SensorEvents m3World_GetSensorEvents(m3WorldId worldId)
{
    m3SensorEvents events;
    memset(&events, 0, sizeof(events));
    m3World* world = m3WorldFromId(worldId);
    if (world != NULL)
    {
        events.beginEvents = world->events.sensorBeginEvents;
        events.endEvents = world->events.sensorEndEvents;
        events.beginCount = world->events.sensorBeginEventCount;
        events.endCount = world->events.sensorEndEventCount;
    }
    return events;
}

m3FragmentEvents m3World_GetFragmentEvents(m3WorldId worldId)
{
    m3FragmentEvents events;
    memset(&events, 0, sizeof(events));
    m3World* world = m3WorldFromId(worldId);
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

m3BodyEvents m3World_GetBodyEvents(m3WorldId worldId)
{
    m3BodyEvents events;
    memset(&events, 0, sizeof(events));
    m3World* world = m3WorldFromId(worldId);
    if (world != NULL)
    {
        events.moveEvents = world->events.moveEvents;
        events.moveCount = world->events.moveEventCount;
    }
    return events;
}

m3JointEvents m3World_GetJointEvents(m3WorldId worldId)
{
    m3JointEvents events;
    memset(&events, 0, sizeof(events));
    m3World* world = m3WorldFromId(worldId);
    if (world != NULL)
    {
        events.breakEvents = world->joints.jointBreakEvents;
        events.breakCount = world->joints.jointBreakEventCount;
    }
    return events;
}

void m3AppendJointBreakEvent(m3World* world, m3JointId joint)
{
    // Capacity is jointCapacity: at most every joint breaks once.
    world->joints.jointBreakEvents[world->joints.jointBreakEventCount].jointId = joint;
    world->joints.jointBreakEventCount += 1;
}

void m3World_SetPreSolveCallback(m3WorldId worldId, m3PreSolveFn* fn, void* context)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    world->preSolveFn = fn;
    world->preSolveContext = context;
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
    m3ShapeId idA = {sA + 1, world->idWorld, world->shapes.shapePool.generations[sA]};
    m3ShapeId idB = {sB + 1, world->idWorld, world->shapes.shapePool.generations[sB]};
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
        e->bodyId = (m3BodyId){i + 1, world->idWorld, world->bodies.bodyPool.generations[i]};
        e->transform = world->bodies.transforms[i];
        e->fellAsleep =
            world->bodies.types[i] == (uint8_t)m3_dynamicBody && world->bodies.awake[i] == 0;
    }
}
