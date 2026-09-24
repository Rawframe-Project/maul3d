// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World queries: internal declarations.

#ifndef MAUL3D_SRC_QUERY_H
#define MAUL3D_SRC_QUERY_H

#include "world_internal.h"

bool m3WorldExplodeInternal(m3World* world, const m3ExplosionDef* def);

m3RayCastResult m3CastConvexClosestEx(m3World* world, m3Pos3 base, const m3Vec3* points,
                                      int32_t pointCount, m3real radius, m3Vec3 translation,
                                      int32_t ignoreBody);

// A bounded selection of the lowest shape ids, kept as a max-heap while
// candidates arrive in any order, then sorted ascending.
typedef struct m3ShapeSelection
{
    m3ShapeId* ids;
    int32_t capacity;
    int32_t size;
} m3ShapeSelection;

void m3SelectionOffer(m3ShapeSelection* sel, const m3World* world, int32_t shape);
// Sorts the kept ids ascending and returns how many were written.
int32_t m3SelectionFinish(m3ShapeSelection* sel);

#endif // MAUL3D_SRC_QUERY_H
