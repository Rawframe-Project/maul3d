// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase: internal declarations.

#ifndef MAUL3D_SRC_BROAD_PHASE_H
#define MAUL3D_SRC_BROAD_PHASE_H

#include "world_internal.h"

// Broadphase v1 (the swappable seam): fills pairKeys in canonical
// ascending key order from fat AABBs; the dynamic tree replaces the
// scan in 2b behind this same contract.
m3Result m3UpdatePairs(m3World* world);

void m3FreezeDiscoverPairs(m3World* world, int32_t body);

// The 2a brute-force scan, kept as the referee: on any scene the tree
// path must produce the identical pair list (a test gate).
m3Result m3UpdatePairsBruteForce(m3World* world);

// Fat world bounds of a sphere shape (double, margin included).
void m3ShapeFatAabb(const m3World* world, int32_t shape, double lo[3], double hi[3]);

#endif // MAUL3D_SRC_BROAD_PHASE_H
