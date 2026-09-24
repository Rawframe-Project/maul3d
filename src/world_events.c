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

const m3HitEvent* m3World_HitEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->events.hitEventCount;
    return world->events.hitEvents;
}

int32_t m3World_HitEventsDropped(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL ? world->events.hitEventsDropped : 0;
}

const m3BodyMoveEvent* m3World_BodyMoveEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->events.moveEventCount;
    return world->events.moveEvents;
}

const m3JointBreakEvent* m3World_JointBreakEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->joints.jointBreakEventCount;
    return world->joints.jointBreakEvents;
}

void m3AppendJointBreakEvent(m3World* world, m3JointId joint)
{
    // Capacity is jointCapacity: at most every joint breaks once.
    world->joints.jointBreakEvents[world->joints.jointBreakEventCount].joint = joint;
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

const m3ContactEvent* m3World_SensorBeginEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->events.sensorBeginEventCount;
    return world->events.sensorBeginEvents;
}

const m3ContactEvent* m3World_SensorEndEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->events.sensorEndEventCount;
    return world->events.sensorEndEvents;
}

const m3FragmentEvent* m3World_FragmentEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    if (count != NULL)
    {
        *count = world->events.fragmentEventCount;
    }
    return world->events.fragmentEvents;
}

const uint16_t* m3World_FragmentRecipe(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    if (count != NULL)
    {
        *count = world->events.fragmentRecipeCount;
    }
    return world->events.fragmentRecipe;
}

int32_t m3World_FragmentEventsDropped(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL ? world->events.fragmentDropped : 0;
}

const m3ContactEvent* m3World_ContactBeginEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->events.beginEventCount;
    return world->events.beginEvents;
}

const m3ContactEvent* m3World_ContactEndEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->events.endEventCount;
    return world->events.endEvents;
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
    m3ContactEvent event;
    event.shapeA = (m3ShapeId){sA + 1, world->worldIndex0, world->shapes.shapePool.generations[sA]};
    event.shapeB = (m3ShapeId){sB + 1, world->worldIndex0, world->shapes.shapePool.generations[sB]};
    int32_t room = world->contacts.pairCapacity;
    m3Events* ev = &world->events;
    if (world->shapes.shapeSensor[sA] != 0 || world->shapes.shapeSensor[sB] != 0)
    {
        if (began && ev->sensorBeginEventCount < room)
        {
            ev->sensorBeginEvents[ev->sensorBeginEventCount++] = event;
        }
        else if (!began && ev->sensorEndEventCount < room)
        {
            ev->sensorEndEvents[ev->sensorEndEventCount++] = event;
        }
    }
    else if (began && ev->beginEventCount < room)
    {
        ev->beginEvents[ev->beginEventCount++] = event;
    }
    else if (!began && ev->endEventCount < room)
    {
        ev->endEvents[ev->endEventCount++] = event;
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
        e->body = (m3BodyId){i + 1, world->worldIndex0, world->bodies.bodyPool.generations[i]};
        e->transform = world->bodies.transforms[i];
        e->fellAsleep =
            world->bodies.types[i] == (uint8_t)m3_dynamicBody && world->bodies.awake[i] == 0;
    }
}
