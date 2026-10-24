// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body lookups and wake state.

#include "body_state.h"

#include "broad_phase.h"
#include "misuse.h"

int32_t m3BodySlot(const m3World* world, m3BodyId bodyId)
{
    int32_t index = bodyId.index1 - 1;
    if (world == NULL || !m3IdPoolValid(&world->bodies.bodyPool, index, bodyId.generation))
    {
        return -1;
    }
    return index;
}

bool m3ResolveBody(const m3World* world, m3BodyId body, int32_t* indexOut)
{
    int32_t index = m3BodySlot(world, body);
    *indexOut = index;
    if (index < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    return true;
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
