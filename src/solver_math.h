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
// Cramer works in the cube of J's scale: when the determinant is not a
// normal float, J is solved at unit scale instead (m3Solve3AtScale).
m3Vec3 m3Solve3(const m3Mat3* J, m3Vec3 b);

// The same solution taken with J divided by scale and the result divided
// by it again, which is J's own in real arithmetic for any positive
// scale. A singular or too extreme system gives zero.
m3Vec3 m3Solve3AtScale(const m3Mat3* J, m3Vec3 b, m3real scale);

// The inverse of the symmetric 2x2 [k11 k12; k12 k22], taken with the
// matrix divided by scale and the result divided by it again, into out
// as {r11, r22, r12}. False, with out untouched, for a singular pair or
// one too extreme even at that scale.
bool m3InvertPairAtScale(m3real k11, m3real k22, m3real k12, m3real scale, m3real out[3]);

m3Mat3 m3WorldInvInertia(const m3World* world, int32_t body);

#endif
