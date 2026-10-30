// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World queries: internal declarations.

#ifndef MAUL3D_SRC_QUERY_H
#define MAUL3D_SRC_QUERY_H

#include "raycast.h"
#include "world_internal.h"

bool m3WorldExplodeInternal(m3World* world, const m3ExplosionDef* def);

m3CastResult m3CastConvexClosestEx(const m3World* world, m3Pos3 base, const m3Vec3* points,
                                   int32_t pointCount, float radius, m3Vec3 translation,
                                   int32_t ignoreBody);

// A bounded selection of the lowest shape ids, kept as a max-heap while
// candidates arrive in any order, then sorted ascending, and a count of
// every candidate offered. ids may be NULL when capacity is 0: the
// selection then only counts.
typedef struct m3ShapeSelection
{
    m3ShapeId* ids;
    int32_t capacity;
    int32_t size;
    int32_t total;
} m3ShapeSelection;

void m3SelectionOffer(m3ShapeSelection* sel, const m3World* world, int32_t shape);
// Sorts the kept ids ascending and returns how many were offered, which
// may exceed the capacity.
int32_t m3SelectionFinish(m3ShapeSelection* sel);

#endif // MAUL3D_SRC_QUERY_H
