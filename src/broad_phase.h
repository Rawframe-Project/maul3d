// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase: internal declarations.

#ifndef MAUL3D_SRC_BROAD_PHASE_H
#define MAUL3D_SRC_BROAD_PHASE_H

#include "world_internal.h"

// Broadphase v1 (the swappable seam): fills pairKeys in canonical
// ascending key order from fat AABBs; the dynamic tree replaces the
// scan in 2b behind this same contract. A table too small for every
// pair keeps the lowest keys and counts the rest in pairsDropped.
void m3UpdatePairs(m3World* world);
// The step scratch the pair update takes for a world of shapeCapacity
// shapes.
int64_t m3PairUpdateScratchBytes(int32_t shapeCapacity);

void m3FreezeDiscoverPairs(m3World* world, int32_t body);

// A destroyed shape's pairs die with it: the manifold of each pair it is
// in becomes a pair that never touched, and the frozen buffer drops its
// keys. Its slot's next shape then starts with no touch, no warm-start
// impulse and no end event of the dead shape's.
void m3ForgetShapePairs(m3World* world, int32_t shape);

// The 2a brute-force scan, kept as the referee: on any scene the tree
// path must produce the identical pair list (a test gate).
void m3UpdatePairsBruteForce(m3World* world);

// Fat world bounds of a sphere shape (double, margin included).
// A proxy's kind bits in the tree: its body's type, and whether it is a
// static surface (mesh or voxel chunk) that sweeps must always see.
#define M3_PROXY_STATIC    1u
#define M3_PROXY_KINEMATIC 2u
#define M3_PROXY_DYNAMIC   4u
#define M3_PROXY_SURFACE   8u
uint32_t m3ProxyMask(const m3World* world, int32_t shape);

void m3ShapeFatAabb(const m3World* world, int32_t shape, double lo[3], double hi[3]);

#endif // MAUL3D_SRC_BROAD_PHASE_H
