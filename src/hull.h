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
// False when the face loops do not close into a hull's edges.
bool m3HullBuildHalfEdges(m3HullData* hull);

// The unit-density volume, centroid and inertia about the centroid from
// the vertices and loops. False for a hull without positive volume.
bool m3HullMassProperties(m3HullData* hull);

// Whether hull data is what the builders make: 4..64 vertices within
// M3_LOCAL_LIMIT, 4..124 faces, each a loop of three or more of the
// hull's vertices in consecutive runs of the index list, planes that hold
// those loops within the tolerances in hull.c, and the unit mass
// properties, half edges and center the builders derive from them, bit
// for bit (a box's closed form for a box). Create and restore both hold
// hull content to it.
bool m3HullDataValid(const m3HullData* hull);

int32_t m3InternHull(m3World* world, const m3HullData* data); // -1 = pool exhausted
void m3ReleaseHull(m3World* world, int32_t hullIndex);

#endif // MAUL3D_SRC_HULL_H
