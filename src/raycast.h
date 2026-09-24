// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ray casts: internal declarations.

#ifndef MAUL3D_SRC_RAYCAST_H
#define MAUL3D_SRC_RAYCAST_H

#include "world_internal.h"

// Monotonic milliseconds (core.c): the profile clock. Observer only.
// Ray against one shape, in world space; used by the ray queries.
m3RayCastResult m3RayTestOneShape(m3World* world, int32_t shape, m3Pos3 origin, m3Vec3 translation);

m3RayCastResult m3RayClosestInternal(m3World* world, m3Pos3 origin, m3Vec3 translation);

// The caster's float budget: a translation
// component beyond this squares past FLT_MAX inside the kernels
// and mints NaN. Every cast path refuses longer translations with
// a documented miss; the character treats them as hostile no-ops.
#define M3_CAST_LIMIT 1.0e18f

// The closest hit that skips one body (the caster's own), unfiltered.
m3RayCastResult m3RayClosestExcept(m3World* world, m3Pos3 origin, m3Vec3 translation,
                                   int32_t ignoreBody);

m3RayCastResult m3RayClosestFiltered(m3World* world, m3Pos3 origin, m3Vec3 translation,
                                     int32_t ignoreBody, m3QueryFilter filter);

#endif // MAUL3D_SRC_RAYCAST_H
