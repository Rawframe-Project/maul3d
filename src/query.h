// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World queries: internal declarations.

#ifndef MAUL3D_SRC_QUERY_H
#define MAUL3D_SRC_QUERY_H

#include "world_internal.h"

bool m3WorldExplodeInternal(m3World* world, const m3ExplosionDef* def);

m3RayHit m3CastConvexClosestEx(m3World* world, m3Pos3 base, const m3Vec3* points,
                               int32_t pointCount, m3real radius, m3Vec3 translation,
                               int32_t ignoreBody);

#endif // MAUL3D_SRC_QUERY_H
