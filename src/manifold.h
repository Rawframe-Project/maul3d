// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact geometry kernels shared by the narrow phase (narrowphase.c):
// the pair-independent pieces of manifold building.

#ifndef MAUL3D_SRC_MANIFOLD_H
#define MAUL3D_SRC_MANIFOLD_H

#include "world_internal.h"

// Shape types run from m3_sphereShape (0) to m3_heightFieldShape.
#define M3_SHAPE_TYPE_COUNT (m3_heightFieldShape + 1)

// Contacts exist slightly before touch so the solver can catch
// approaches speculatively (the reference model).
#define M3_SPECULATIVE_DISTANCE M3_AABB_MARGIN

void m3SphereWorldCenter(const m3World* world, int32_t shape, double* cx, double* cy, double* cz);
m3Vec3 m3AnchorFromCom(const m3World* world, int32_t body, double px, double py, double pz);
void m3DeepPointInHull(const m3HullData* hull, m3Vec3 q, m3Vec3* normalOut, m3real* coreSepOut,
                       m3Vec3* onHullOut);
m3Manifold m3CollideSegmentHull(const m3HullData* hull, m3Vec3 p1, m3Vec3 p2, m3real radius);
void m3CollideMeshConvex(m3World* world, m3Manifold* fresh, int32_t meshShape, int32_t otherShape,
                         int meshIsA);
void m3CollideHeightFieldConvex(m3World* world, m3Manifold* fresh, int32_t hfShape,
                                int32_t otherShape, int hfIsA);
void m3CollideVoxelConvex(m3World* world, m3Manifold* fresh, int32_t voxelShape, int32_t otherShape,
                          int voxelIsA);

#endif // MAUL3D_SRC_MANIFOLD_H
