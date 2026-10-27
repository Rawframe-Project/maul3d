// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The free rotation of a rigid body: its principal frame, computed once
// per mass update, and the splitting that turns its pose and spin through
// a substep. docs/internals/rotation.md specifies both step by step.

#ifndef MAUL3D_SRC_RIGID_FLOW_H
#define MAUL3D_SRC_RIGID_FLOW_H

#include "maul3d/core_math.h"

#include <stdbool.h>

// The principal moments of a symmetric inertia, sorted largest first, and
// the rotation from the principal frame to the body frame: inertia is
// R diag(moments) R^T for R the matrix of frame. A diagonal tensor gives
// the permutation that sorts it, kept a rotation.
void m3PrincipalAxes(const m3Mat3* inertia, m3Vec3* momentsOut, m3Quat* frameOut);

// Whether the moments allow the free rotation: each finite, the smallest
// a normal float, so every inverse below is finite.
bool m3FreeRotationApplies(m3Vec3 moments);

// Turns a body at rotation q with world spin w through time h as a free
// rigid body, by the splitting of its kinetic energy: the world angular
// momentum stays, and a body with two equal moments turns exactly. Writes
// the new rotation and world spin.
void m3FreeRotation(m3Vec3 moments, m3Quat frame, m3Quat q, m3Vec3 w, m3real h, m3Quat* qOut,
                    m3Vec3* wOut);

#endif // MAUL3D_SRC_RIGID_FLOW_H
