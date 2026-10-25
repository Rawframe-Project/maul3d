// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// QuickHull: internal declarations.

#ifndef MAUL3D_SRC_QUICKHULL_H
#define MAUL3D_SRC_QUICKHULL_H

#include "world_internal.h"

// Builds the convex hull of up to M3_HULL_MAX_INPUT points into the
// fixed m3HullData block, coplanar faces merged, unit-density mass
// properties integrated. Returns false on degenerate input (fewer than
// four points, flat clouds, non-finite coordinates) or when the result
// exceeds the fixed caps.
bool m3ComputeHull(m3Heap* heap, const m3Vec3* points, int32_t count, m3HullData* out);

// A cloud a hull shape's create and its replay accept: 1 to
// M3_HULL_MAX_INPUT points, each within M3_LOCAL_LIMIT of the body.
bool m3HullCloudValid(const m3Vec3* points, int32_t count);

#endif // MAUL3D_SRC_QUICKHULL_H
