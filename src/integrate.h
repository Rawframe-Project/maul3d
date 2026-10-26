// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Integration: the bodies a step moves, their velocities under forces,
// water, damping and the gyroscopic term, and their poses.

#ifndef MAUL3D_SRC_INTEGRATE_H
#define MAUL3D_SRC_INTEGRATE_H

#include "water.h"
#include "world_internal.h"

// One implicit gyroscopic step of a spin w in world coordinates for a
// body at rotation q, with its local inertia divided by scale. Every term
// of the step is linear in the inertia, so any scale gives the same spin
// in real arithmetic; scale 1 is the step the solver takes first. Writes
// the new spin to out and returns true, or returns false when a term of
// the step leaves float range at this scale.
bool m3GyroscopicStep(const m3Mat3* inertia, m3real scale, m3Quat q, m3Vec3 w, m3real h,
                      m3Vec3* out);

// Awake dynamic and kinematic bodies in ascending slot order; kinematic
// targets become the velocities that land them this step. Returns the
// count.
int32_t m3BuildMovers(m3World* world, int32_t* movers, float dt);

void m3IntegrateVelocities(m3World* world, const int32_t* movers, int32_t moverCount,
                           const m3Buoyancy* buoy, m3real h);

// Holds every dynamic mover's velocities within M3_SPEED_LIMIT; a
// velocity already within it is left bit for bit.
void m3CapMoverSpeeds(m3World* world, const int32_t* movers, int32_t moverCount);

// Advances the poses and the drift accumulators the solver measures
// separations with.
void m3IntegratePositions(m3World* world, const int32_t* movers, int32_t moverCount,
                          m3Vec3* deltaPos, m3Quat* deltaRot, m3real h);

#endif // MAUL3D_SRC_INTEGRATE_H
