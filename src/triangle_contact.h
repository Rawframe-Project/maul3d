// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex shapes against one triangle: the kernels the welded mesh
// pipeline (manifold_mesh.c) runs per candidate triangle.

#ifndef MAUL3D_SRC_TRIANGLE_CONTACT_H
#define MAUL3D_SRC_TRIANGLE_CONTACT_H

#include "world_internal.h"

// One triangle's local manifold: midway contact points (each anchor
// recovers as point -/+ half the separation along the normal).
// feature: vertex bitmask (1|2|4, 7 = triangle face) or 8 = hull
// face contact (the hull path's special acceptance rules).
#define M3_TRI_FEATURE_HULL_FACE 8

typedef struct m3TriManifold
{
    m3Vec3 normal;    // mesh frame, triangle toward shape
    m3Vec3 triNormal; // mesh frame (the hull-face acceptance reads it)
    m3real dist2;     // closest squared distance (the tentative sort key)
    int32_t pointCount;
    int32_t feature;
    m3Vec3 point[4];
    m3real separation[4];
    uint16_t localId[4];
} m3TriManifold;

// Each kernel takes the triangle counterclockwise seen from its front and
// culls shapes behind it. The sphere and capsule run in the triangle's
// frame; the hull kernel runs in the hull's frame.
void m3CollideSphereTriangle(m3TriManifold* out, m3Vec3 center, m3real radius, const m3Vec3 tri[3]);
void m3CollideCapsuleTriangle(m3TriManifold* out, m3Vec3 c1, m3Vec3 c2, m3real radius,
                              const m3Vec3 tri[3]);
void m3CollideHullTriangle(m3TriManifold* out, const m3HullData* hull, const m3Vec3 tri[3]);

#endif // MAUL3D_SRC_TRIANGLE_CONTACT_H
