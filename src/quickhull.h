// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// QuickHull: internal declarations.

#ifndef MAUL3D_SRC_QUICKHULL_H
#define MAUL3D_SRC_QUICKHULL_H

#include "world_internal.h"

// QuickHull, adapted from the reference hull.c (Erin Catto,
// with portions contributed by Dirk Gregorius): builds the convex
// hull of up to M3_HULL_MAX_INPUT points into the fixed m3HullData
// block, faces merged coplanar, unit-density mass properties
// integrated. Returns false loudly on degenerate input (fewer than
// four points, coplanar clouds, non-finite coordinates) or if the
// result exceeds the fixed caps.
bool m3ComputeHull(const m3Vec3* points, int32_t count, m3HullData* out);

#endif // MAUL3D_SRC_QUICKHULL_H
