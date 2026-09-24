// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Integration: the bodies a step moves, their velocities under forces,
// water, damping and the gyroscopic term, and their poses.

#ifndef MAUL3D_SRC_INTEGRATE_H
#define MAUL3D_SRC_INTEGRATE_H

#include "water.h"
#include "world_internal.h"

// Awake dynamic and kinematic bodies in ascending slot order; kinematic
// targets become the velocities that land them this step. Returns the
// count.
int32_t m3BuildMovers(m3World* world, int32_t* movers, float dt);

void m3IntegrateVelocities(m3World* world, const int32_t* movers, int32_t moverCount,
                           const m3Buoyancy* buoy, m3real h);

// Advances the poses and the drift accumulators the solver measures
// separations with.
void m3IntegratePositions(m3World* world, const int32_t* movers, int32_t moverCount,
                          m3Vec3* deltaPos, m3Quat* deltaRot, m3real h);

#endif // MAUL3D_SRC_INTEGRATE_H
