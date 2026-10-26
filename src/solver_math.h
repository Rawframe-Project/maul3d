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
// When the determinant is not a normal float, zero included, J is solved
// at its largest entry's scale instead (m3Solve3AtScale); the direct
// result stands otherwise, so every ordinary system keeps its bits.
m3Vec3 m3Solve3(const m3Mat3* J, m3Vec3 b);

// The same solution taken with J divided by scale and the result divided
// by it again, which is J's own in real arithmetic for any positive
// scale. A singular or too extreme system gives zero.
m3Vec3 m3Solve3AtScale(const m3Mat3* J, m3Vec3 b, m3real scale);

// Whether the float solve of a symmetric positive definite system keeps
// its direct result: its determinant det is a normal float and holds at
// least 2^-12 of diag, the product of its diagonal, which bounds it. Below
// that the entries have lost what tells them apart: a mass that is small
// next to inverse inertia times arm squared, mass * arm^2 / inertia past
// about 1e7, rounds away in each entry, and the float determinant comes
// out zero, negative or wrong. Such a system, and one whose determinant
// underflows, is solved in double instead (m3Solve3Wide,
// m3SolvePairWide) from its rows, where the mass survives.
bool m3DirectHolds(m3real det, m3real diag);

// The float solve of J x = b by Cramer's rule into *x, and whether it
// stands: m3DirectHolds with the product of J's diagonal. J is symmetric
// positive definite.
bool m3Solve3Direct(const m3Mat3* J, m3Vec3 b, m3Vec3* x);

// u . v and u^T w v in double.
double m3WideDot(m3Vec3 u, m3Vec3 v);
double m3WideForm(const m3Mat3* w, m3Vec3 u, m3Vec3 v);

// Solves k x = b by Cramer's rule in double, with k's columns in k[0..2],
// k[3..5] and k[6..8]. A singular k, or a solution that is not finite as
// floats, gives zero.
m3Vec3 m3Solve3Wide(const double k[9], m3Vec3 b);

// A point constraint's 3x3, mass I - [armA] invIA [armA] - [armB] invIB
// [armB] with mass the two inverse masses' sum, built in double with its
// columns as m3Solve3Wide takes them.
void m3WidePointBlock(double mass, const m3Mat3* invIA, m3Vec3 armA, const m3Mat3* invIB,
                      m3Vec3 armB, double k[9]);

// The inverse of the symmetric 2x2 [k11 k12; k12 k22], taken with the
// matrix divided by scale and the result divided by it again, into out
// as {r11, r22, r12}. False, with out untouched, for a singular pair or
// one too extreme even at that scale.
bool m3InvertPairAtScale(m3real k11, m3real k22, m3real k12, m3real scale, m3real out[3]);

// Solves the symmetric 2x2 [k11 k12; k12 k22] s = (x, y) in double, its
// couplings taken in double, into out. False, with out untouched, for a
// pair that is not positive definite in double or whose solution is not
// finite as floats.
bool m3SolvePairWide(double k11, double k22, double k12, m3real x, m3real y, m3real out[2]);

m3Mat3 m3WorldInvInertia(const m3World* world, int32_t body);

// The factor, at most 1, that holds a loaded warm start within
// M3_WARM_LIMIT and M3_WARM_MOMENT_LIMIT (core.h): its impulses sum to
// sum in magnitude and act at levers within lever (L1 norms) on bodies
// with these inverse masses and world inverse inertias. Exactly 1 when
// no hold is needed.
m3real m3WarmHold(m3real invMassA, const m3Mat3* invIA, m3real invMassB, const m3Mat3* invIB,
                  double lever, double sum);

// |x| + |y| + |z| in double.
double m3NormL1(m3Vec3 v);

// An impulse scale * dir at arm (from the center of mass) that the step
// itself lands on a dynamic body: a character's push, a soft particle's,
// a wheel's. Like the step, it leaves the body within M3_SPEED_LIMIT. When
// the float velocity or spin is not finite, the push is taken again in
// double, velocity and spin both, from the same float scale, which every
// caller's bounds keep finite; there the bounds on every factor keep each
// product finite. The result is that double push, held within the limit
// and rounded to float, so a velocity that was finite in float may move by
// an ulp. Each retake counts in m3Counters.heldPushRetakes.
void m3PushHeld(m3World* world, int32_t index, m3real scale, m3Vec3 dir, m3Vec3 arm);

#endif
