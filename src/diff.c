// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The divergence finder's world comparator: a pure debug
// read over two worlds' body books. Never state, never hashed,
// never inside the step.

#include "maul3d/replay.h"

#include "world.h"
#include "world_internal.h"

#include <stdlib.h>
#include <string.h>

static m3real DiffTotal(const m3BodyDiff* d)
{
    m3real presence = (d->onlyInA || d->onlyInB) ? 1.0e30f : 0.0f;
    return presence + d->positionError + d->rotationError + d->velocityError;
}

static int DiffCompare(const void* pa, const void* pb)
{
    const m3BodyDiff* a = (const m3BodyDiff*)pa;
    const m3BodyDiff* b = (const m3BodyDiff*)pb;
    m3real ta = DiffTotal(a);
    m3real tb = DiffTotal(b);
    if (ta > tb)
    {
        return -1;
    }
    if (ta < tb)
    {
        return 1;
    }
    return a->bodyId.index1 < b->bodyId.index1 ? -1 : (a->bodyId.index1 > b->bodyId.index1 ? 1 : 0);
}

int32_t m3World_Compare(m3WorldId worldA, m3WorldId worldB, m3BodyDiff* out, int32_t capacity,
                        int32_t* outCount)
{
    m3World* a = m3WorldFromId(worldA);
    m3World* b = m3WorldFromId(worldB);
    if (a == NULL || b == NULL || out == NULL || capacity <= 0 || outCount == NULL ||
        a->bodies.bodyCapacity != b->bodies.bodyCapacity)
    {
        if (outCount != NULL)
        {
            *outCount = 0;
        }
        return -1;
    }
    int32_t maxA = a->bodies.bodyPool.maxIndex;
    int32_t maxB = b->bodies.bodyPool.maxIndex;
    int32_t maxIndex = maxA > maxB ? maxA : maxB;
    m3BodyDiff* rows = (m3BodyDiff*)m3AllocZeroed(
        maxIndex > 0 ? maxIndex * (int32_t)sizeof(m3BodyDiff) : (int32_t)sizeof(m3BodyDiff));
    if (rows == NULL)
    {
        if (outCount != NULL)
        {
            *outCount = 0;
        }
        return -1; // out of memory
    }
    int32_t found = 0;
    for (int32_t i = 0; i < maxIndex; ++i)
    {
        int aliveA = i < maxA && a->bodies.bodyPool.alive[i] != 0;
        int aliveB = i < maxB && b->bodies.bodyPool.alive[i] != 0;
        if (!aliveA && !aliveB)
        {
            continue;
        }
        m3BodyDiff d;
        memset(&d, 0, sizeof(d));
        d.bodyId = (m3BodyId){i + 1, a->idWorld,
                              aliveA ? a->bodies.bodyPool.generations[i]
                                     : b->bodies.bodyPool.generations[i]};
        if (aliveA != aliveB)
        {
            d.onlyInA = aliveA != 0;
            d.onlyInB = aliveB != 0;
            rows[found++] = d;
            continue;
        }
        m3Vec3 dp = {(m3real)(a->bodies.transforms[i].p.x - b->bodies.transforms[i].p.x),
                     (m3real)(a->bodies.transforms[i].p.y - b->bodies.transforms[i].p.y),
                     (m3real)(a->bodies.transforms[i].p.z - b->bodies.transforms[i].p.z)};
        m3Quat qa = a->bodies.transforms[i].q;
        m3Quat qb = b->bodies.transforms[i].q;
        m3real qdot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
        m3Vec3 dv = m3Sub3(a->bodies.linearVelocities[i], b->bodies.linearVelocities[i]);
        m3Vec3 dw = m3Sub3(a->bodies.angularVelocities[i], b->bodies.angularVelocities[i]);
        d.positionError = m3Length3(dp);
        d.rotationError = 1.0f - (qdot < 0.0f ? -qdot : qdot);
        d.velocityError = m3Length3(dv) + m3Length3(dw);
        if (d.positionError != 0.0f || d.rotationError != 0.0f || d.velocityError != 0.0f ||
            a->bodies.awake[i] != b->bodies.awake[i])
        {
            rows[found++] = d;
        }
    }
    qsort(rows, (size_t)found, sizeof(m3BodyDiff), DiffCompare);
    int32_t written = found < capacity ? found : capacity;
    memcpy(out, rows, (size_t)written * sizeof(m3BodyDiff));
    *outCount = written;
    m3Free(rows);
    return found;
}
