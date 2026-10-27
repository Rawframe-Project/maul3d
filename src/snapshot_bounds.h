// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restore's magnitude checks (snapshot_bounds.c).

#ifndef MAUL3D_SRC_SNAPSHOT_BOUNDS_H
#define MAUL3D_SRC_SNAPSHOT_BOUNDS_H

#include "snapshot_values.h"

#include "maul3d/softbody.h"

// The longest rest length a soft body's create gives an edge, in core.h's
// limits, which every user of these includes: a lattice edge is at most
// twice the spacing, a tet edge joins two points within M3_LOCAL_LIMIT on
// each axis, so at most 2 sqrt(3) M3_LOCAL_LIMIT apart (rounded up). Six
// times a tet's rest volume is at most the cube of it, and a lattice's
// closed volume is at most M3_SOFTBODY_MAX_PARTICLES spacing cubes.
#define M3_SOFT_EDGE_LIMIT (3.4641020f * M3_LOCAL_LIMIT)
#define M3_SOFT_TET_LIMIT  (M3_SOFT_EDGE_LIMIT * M3_SOFT_EDGE_LIMIT * M3_SOFT_EDGE_LIMIT)
#define M3_SOFT_SHAPE_LIMIT                                                                        \
    ((float)M3_SOFTBODY_MAX_PARTICLES * M3_LOCAL_LIMIT * M3_LOCAL_LIMIT * M3_LOCAL_LIMIT)

// Whether every value a live record holds is within the bound its create,
// setter and replay op keep (src/core.h). The structure and per-record
// checks run first, so every count and index read here is in range.
bool m3SnapshotBoundsValid(const m3Incoming* inc);

#endif // MAUL3D_SRC_SNAPSHOT_BOUNDS_H
