// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The math the contact, joint, character and vehicle solvers share: soft
// constraint coefficients, a 3x3 solve and a body's world inverse
// inertia.

#ifndef MAUL3D_SRC_SOLVER_MATH_H
#define MAUL3D_SRC_SOLVER_MATH_H

#include "world_internal.h"

// A soft constraint's bias rate and its mass and impulse scales.
// Joint rows without a user softness: stiff, and damped well past
// critical so they settle without ringing.
#define M3_JOINT_HERTZ         60.0f
#define M3_JOINT_DAMPING_RATIO 2.0f

typedef struct m3Softness
{
    m3real biasRate;
    m3real massScale;
    m3real impulseScale;
} m3Softness;

// Soft constraint coefficients for a stiffness, a damping ratio and a
// substep; zero hertz gives a rigid row.
m3Softness m3MakeSoft(m3real hertz, m3real zeta, m3real h);

// Solves the 3x3 system J x = b by Cramer's rule; a singular J gives zero.
m3Vec3 m3Solve3(const m3Mat3* J, m3Vec3 b);

m3Mat3 m3WorldInvInertia(const m3World* world, int32_t body);

#endif
