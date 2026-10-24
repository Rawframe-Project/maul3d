// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soft body pass's cull (softbody_cull.c): which shapes no particle
// inside a box can reach this pass.

#ifndef MAUL3D_SRC_SOFTBODY_CULL_H
#define MAUL3D_SRC_SOFTBODY_CULL_H

#include "world_internal.h"

// A sphere in the shape's local frame, center *center and the returned
// radius, that holds every point a particle must come within its radius
// of to be projected; -1 for the kinds without one (planes and hulls).
m3real m3SoftShapeBound(const m3World* world, int32_t shape, uint8_t stype, m3Vec3* center);

// Whether no particle inside [lo, hi] can reach a shape with that bound
// on a body at xf.
bool m3SoftShapeOutOfReach(const m3Transform* xf, m3Vec3 center, m3real bound, m3real radius,
                           const double lo[3], const double hi[3]);

// Whether one of the hull's faces keeps every particle inside [lo, hi]
// out of its reach.
bool m3SoftHullOutOfReach(const m3World* world, int32_t shape, const m3Transform* xf, m3real radius,
                          const double lo[3], const double hi[3]);

void m3SoftGrowBox(double lo[3], double hi[3], m3Pos3 p);

#endif
