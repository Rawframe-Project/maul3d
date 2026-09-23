// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact geometry kernels shared by the narrow phase (narrowphase.c):
// the pair-independent pieces of manifold building.

#ifndef MAUL3D_SRC_MANIFOLD_H
#define MAUL3D_SRC_MANIFOLD_H

#include "distance.h"
#include "world_internal.h"

// Shape types run from m3_sphereShape (0) to m3_heightFieldShape.
#define M3_SHAPE_TYPE_COUNT (m3_heightFieldShape + 1)

// Contacts exist slightly before touch so the solver can catch
// approaches speculatively.
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

// Picks at most four of count contact points spanning the widest patch
// on the plane of normal: the deepest, the farthest from it, the one
// making the largest triangle with those two, and the one reaching
// farthest outside that triangle. Ties go to the lower index. Writes the
// chosen indices in ascending order and returns how many.
int32_t m3ReduceContactPoints(const m3Vec3* points, const m3real* separations, int32_t count,
                              m3Vec3 normal, int32_t out[M3_MANIFOLD_MAX_POINTS]);

// Deterministic tangent basis: ONE fixed rule (the world axis with the
// smallest absolute normal component, ties broken x before y before
// z), because the friction rows are order-sensitive downstream.
void m3MakeTangentBasis(m3Vec3 normal, m3Vec3* t1, m3Vec3* t2);

// GJK proxy for one shape in its local frame (spheres and capsules
// borrow the caller's scratch for their point storage).
m3DistanceProxy m3MakeShapeProxy(const m3World* world, int32_t shape, m3Vec3 scratch[2]);

// Hull versus hull: the separating axis test over both hulls' faces and
// the edge pairs that form Minkowski faces, then face clipping or the
// edge closest-point contact. B is given in A's frame; the manifold is
// in A's frame with the A-to-B normal, at most four points spread over
// the patch.
m3Manifold m3CollideHulls(const m3HullData* hullA, const m3HullData* hullB, m3Quat q, m3Vec3 p);

// Pure collide kernels (world-independent, tested in isolation).
// Normals point from A to B. d is the center offset B minus A in
// floats (exact enough near contact).
m3Manifold m3CollideSpheres(m3Vec3 d, m3real radiusA, m3real radiusB);

// Plane (A) versus sphere (B): dist is the signed distance of the
// sphere center above the plane, computed in double by the caller.
m3Manifold m3CollidePlaneSphere(m3Vec3 planeNormal, m3real dist, m3real radius);

#endif // MAUL3D_SRC_MANIFOLD_H
