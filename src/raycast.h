// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ray casts: internal declarations.

#ifndef MAUL3D_SRC_RAYCAST_H
#define MAUL3D_SRC_RAYCAST_H

#include "world_internal.h"

// A cast's closest hit as the cast code passes it around: the public hit
// and whether there was one.
typedef struct m3CastResult
{
    m3ShapeId shapeId; // null when hit is false
    m3Pos3 point;
    m3Vec3 normal;
    float fraction;
    bool hit;
} m3CastResult;

// A closest cast clears its hit first, before any check, so a miss, a
// refused input and a NULL world all leave zeros.
static inline void m3ClearCastHit(m3CastHit* hitOut)
{
    *hitOut = (m3CastHit){0}; // the null shape id, a zero point, normal and fraction
}

// Writes a hit to hitOut and returns true, or returns false on a miss with
// hitOut as m3ClearCastHit left it.
static inline bool m3TakeCastHit(m3CastResult result, m3CastHit* hitOut)
{
    if (!result.hit)
    {
        return false;
    }
    *hitOut = (m3CastHit){result.shapeId, result.point, result.normal, result.fraction};
    return true;
}

// Monotonic milliseconds (core.c): the profile clock. Observer only.
// Ray against one shape, in world space; used by the ray queries.
m3CastResult m3RayTestOneShape(const m3World* world, int32_t shape, m3Pos3 origin,
                               m3Vec3 translation);

m3CastResult m3RayClosestInternal(m3World* world, m3Pos3 origin, m3Vec3 translation);

// The caster's float budget: a translation
// component beyond this squares past FLT_MAX inside the kernels
// and mints NaN. Every cast path refuses longer translations with
// a documented miss; the character treats them as hostile no-ops.

// The closest hit that skips one body (the caster's own), unfiltered.
m3CastResult m3RayClosestExcept(m3World* world, m3Pos3 origin, m3Vec3 translation,
                                int32_t ignoreBody);

m3CastResult m3RayClosestFiltered(const m3World* world, m3Pos3 origin, m3Vec3 translation,
                                  int32_t ignoreBody, m3QueryFilter filter);

#endif // MAUL3D_SRC_RAYCAST_H
