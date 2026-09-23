// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Islands and sleep: union-find over touching pairs and joints, the wake
// pass after contacts are built, the sleep pass at the end of the step.

#include "island.h"

#include "world_internal.h"

#include <string.h>

// ---------------------------------------------------------------
// Islands and sleep (2b-10, the Maul2D recipe): union-find over the
// touching dynamic pairs in canonical order. The wake pass runs
// right after contacts are built (a sleeping body touched by an
// awake one must join THIS step's solve); the sleep decision runs at
// the end of the step with the same island labels. Sleeping bodies
// are bit-frozen: velocities zeroed once, integration and solving
// skip them, and the hash stands still.
// ---------------------------------------------------------------

static int32_t IslandFind(int32_t* parent, int32_t i)
{
    while (parent[i] != i)
    {
        parent[i] = parent[parent[i]]; // halving, deterministic
        i = parent[i];
    }
    return i;
}

static void IslandUnion(int32_t* parent, int32_t a, int32_t b)
{
    int32_t ra = IslandFind(parent, a);
    int32_t rb = IslandFind(parent, b);
    if (ra != rb)
    {
        // Lower root wins: canonical labels independent of order.
        if (ra < rb)
        {
            parent[rb] = ra;
        }
        else
        {
            parent[ra] = rb;
        }
    }
}

// Build islands from the current touching pairs and wake every
// island that contains an awake member or a moving kinematic
// neighbor. Returns the parent array (scratch-allocated).
int32_t* m3IslandWakePass(m3World* world)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    int32_t* parent = (int32_t*)m3StackAlloc(&world->scratch,
                                             maxBody > 0 ? maxBody * (int32_t)sizeof(int32_t) : 4);
    uint8_t* forced = (uint8_t*)m3StackAlloc(&world->scratch, maxBody > 0 ? maxBody : 1);
    if (parent == NULL || forced == NULL)
    {
        return NULL;
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        parent[i] = i;
        forced[i] = 0;
    }
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->manifolds[i].pointCount == 0)
        {
            continue;
        }
        uint64_t key = world->pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if (world->shapeSensor[shapeA] != 0 || world->shapeSensor[shapeB] != 0)
        {
            continue; // sensor overlap couples nothing and wakes no one
        }
        int32_t bodyA = world->shapeBody[shapeA];
        int32_t bodyB = world->shapeBody[shapeB];
        int dynA = world->types[bodyA] == (uint8_t)m3_dynamicBody;
        int dynB = world->types[bodyB] == (uint8_t)m3_dynamicBody;
        if (dynA && dynB)
        {
            IslandUnion(parent, bodyA, bodyB);
        }
        else if (dynA || dynB)
        {
            // A moving kinematic neighbor forces its contact awake.
            int32_t kin = dynA ? bodyB : bodyA;
            int32_t dyn = dynA ? bodyA : bodyB;
            if (world->types[kin] == (uint8_t)m3_kinematicBody)
            {
                m3Vec3 v = world->linearVelocities[kin];
                m3Vec3 w = world->angularVelocities[kin];
                if (m3Dot3(v, v) > 0.0f || m3Dot3(w, w) > 0.0f)
                {
                    forced[dyn] = 1;
                }
            }
        }
    }
    // Joints couple islands exactly like touching contacts, and a
    // moving kinematic partner is a wake source through a joint too.
    int32_t maxJoint = world->jointPool.maxIndex;
    for (int32_t j = 0; j < maxJoint; ++j)
    {
        if (world->jointPool.alive[j] == 0)
        {
            continue;
        }
        int32_t bodyA = world->jointBodyA[j];
        int32_t bodyB = world->jointBodyB[j];
        int dynA = world->types[bodyA] == (uint8_t)m3_dynamicBody;
        int dynB = world->types[bodyB] == (uint8_t)m3_dynamicBody;
        if (dynA && dynB)
        {
            IslandUnion(parent, bodyA, bodyB);
        }
        else if (dynA || dynB)
        {
            int32_t kin = dynA ? bodyB : bodyA;
            int32_t dyn = dynA ? bodyA : bodyB;
            if (world->types[kin] == (uint8_t)m3_kinematicBody)
            {
                m3Vec3 v = world->linearVelocities[kin];
                m3Vec3 w = world->angularVelocities[kin];
                if (m3Dot3(v, v) > 0.0f || m3Dot3(w, w) > 0.0f)
                {
                    forced[dyn] = 1;
                }
            }
        }
    }

    // Aggregate: does any island member demand wakefulness?
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        if (world->awake[i] != 0 || forced[i] != 0)
        {
            forced[IslandFind(parent, i)] = 1;
        }
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        if (world->awake[i] == 0 && forced[IslandFind(parent, i)] != 0)
        {
            world->awake[i] = 1; // woken by the island: timers restart
            world->sleepTimes[i] = 0.0f;
        }
    }
    return parent;
}

// End-of-step sleep decision: timers advance for slow awake bodies,
// and an island sleeps only when EVERY member is ready.
void m3IslandSleepPass(m3World* world, int32_t* parent, const m3Pos3* com0, const m3Quat* rot0,
                       float dt)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    m3real invDt = dt > 0.0f ? 1.0f / dt : 0.0f;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        m3Vec3 v = world->linearVelocities[i];
        m3Vec3 w = world->angularVelocities[i];
        m3real velocity = sqrtf(m3Dot3(v, v)) + sqrtf(m3Dot3(w, w)) * world->maxExtents[i];
        // Position correction counts too (the reference lesson: bias
        // pushes move bodies that report zero velocity).
        m3Vec3 rlc = m3RotateVec3(world->transforms[i].q, world->localCenters[i]);
        m3Vec3 dc = {(m3real)(world->transforms[i].p.x + (double)rlc.x - com0[i].x),
                     (m3real)(world->transforms[i].p.y + (double)rlc.y - com0[i].y),
                     (m3real)(world->transforms[i].p.z + (double)rlc.z - com0[i].z)};
        m3Quat q0 = rot0[i];
        m3Quat dq = m3MulQuat(world->transforms[i].q, (m3Quat){-q0.x, -q0.y, -q0.z, q0.w});
        m3real motion = sqrtf(m3Dot3(dc, dc)) + 2.0f *
                                                    sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) *
                                                    world->maxExtents[i];
        m3real sleepVelocity = m3MaxF(velocity, 0.5f * invDt * motion);
        if (world->bodyCanSleep[i] == 0)
        {
            sleepVelocity = 3.4e38f; // never below any threshold
        }
        if (sleepVelocity < world->bodySleepThreshold[i])
        {
            world->sleepTimes[i] += dt;
        }
        else
        {
            world->sleepTimes[i] = 0.0f;
        }
    }
    // Island readiness: every member past the time-to-sleep bar.
    uint8_t* ready = (uint8_t*)m3StackAlloc(&world->scratch, maxBody > 0 ? maxBody : 1);
    if (ready == NULL)
    {
        return;
    }
    memset(ready, 1, (size_t)(maxBody > 0 ? maxBody : 1));
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        if (world->sleepTimes[i] < 0.5f)
        {
            ready[IslandFind(parent, i)] = 0;
        }
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        if (ready[IslandFind(parent, i)] != 0)
        {
            // The whole island crosses together: freeze bit-solid.
            world->awake[i] = 0;
            world->linearVelocities[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
            world->angularVelocities[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
            // S-3b: the freeze step may have pushed this body into
            // overlaps no pair list ever saw; discover them now so
            // the frozen buffer equals what a full query would find.
            m3FreezeDiscoverPairs(world, i);
        }
    }
}
