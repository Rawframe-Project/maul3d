// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex hulls: internal declarations.

#ifndef MAUL3D_SRC_HULL_H
#define MAUL3D_SRC_HULL_H

#include "world_internal.h"

// Analytic box hull (canonical vertex and face order) and the intern
// machinery. Interning dedupes by content in ascending slot order.
void m3BuildBoxHull(m3HullData* out, m3Vec3 halfExtents);

// Half-edge adjacency from the face loops (twins at 2k and 2k+1 by
// construction, the invariant the Gauss-map edge query leans on).
// One law for every hull source: the box builder and QuickHull both
// finish through this.
void m3HullBuildHalfEdges(m3HullData* hull);

int32_t m3InternHull(m3World* world, const m3HullData* data); // -1 = pool exhausted
void m3ReleaseHull(m3World* world, int32_t hullIndex);

#endif // MAUL3D_SRC_HULL_H
