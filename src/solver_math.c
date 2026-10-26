// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Soft constraint coefficients, the 3x3 solve and world inverse inertia.

#include "solver_math.h"

#include <float.h>
#include <string.h>

// With w = 2 pi hertz, stiffness k = m w^2 and damping c = 2 m zeta w, an
// implicit Euler step of length h turns the spring into a row that solves
//   J v + (k / (h k + c)) C + (1 / (h (h k + c))) lambda = 0
// for the total impulse lambda. Let a = h w (2 zeta + h w). Then
//   biasRate     = k / (h k + c) = w / (2 zeta + h w)
//   massScale    = a / (1 + a)   (m against m + the lambda term)
//   impulseScale = 1 / (1 + a)   (the lambda term on the accumulated part)
// and a solve adds -m massScale (J v + biasRate C) - impulseScale
// accumulated. Zero hertz is a rigid row with no position feedback.
m3Softness m3MakeSoft(m3real hertz, m3real zeta, m3real h)
{
    if (hertz == 0.0f)
    {
        return (m3Softness){0.0f, 0.0f, 0.0f};
    }
    m3real omega = 2.0f * M3_PI * hertz;
    m3real a1 = 2.0f * zeta + h * omega;
    m3real a2 = h * omega * a1;
    m3real a3 = 1.0f / (1.0f + a2);
    return (m3Softness){omega / a1, a2 * a3, a3};
}

// I_w^-1 = R I_l^-1 R^T, built by applying the operator to the world
// basis vectors. Fixed at prepare like the contact anchors.
m3Mat3 m3WorldInvInertia(const m3World* world, int32_t body)
{
    if (world->bodies.types[body] != (uint8_t)m3_dynamicBody)
    {
        return m3MakeZeroMat3();
    }
    m3Quat q = world->bodies.transforms[body].q;
    m3Mat3 il = world->bodies.invInertiaLocal[body];
    m3Mat3 r;
    r.cx = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f})));
    r.cy = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 1.0f, 0.0f})));
    r.cz = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 0.0f, 1.0f})));
    return r;
}

static double MaxD(double a, double b)
{
    return a > b ? a : b;
}

double m3NormL1(m3Vec3 v)
{
    return (double)m3AbsF(v.x) + (double)m3AbsF(v.y) + (double)m3AbsF(v.z);
}

// The largest change, per component, impulses summing to sum at levers
// within lever make in a body's velocity or spin: each component of an
// inverse inertia times a vector is at most three times its largest
// entry times the vector's largest component.
static double WarmReach(m3real invMass, const m3Mat3* invI, double lever, double sum)
{
    double big = 0.0;
    const m3Vec3* cols[3] = {&invI->cx, &invI->cy, &invI->cz};
    for (int32_t k = 0; k < 3; ++k)
    {
        big = MaxD(big, MaxD((double)m3AbsF(cols[k]->x),
                             MaxD((double)m3AbsF(cols[k]->y), (double)m3AbsF(cols[k]->z))));
    }
    return ((double)invMass + 3.0 * big * (lever + 1.0)) * sum;
}

m3real m3WarmHold(m3real invMassA, const m3Mat3* invIA, m3real invMassB, const m3Mat3* invIB,
                  double lever, double sum)
{
    double reach =
        MaxD(WarmReach(invMassA, invIA, lever, sum), WarmReach(invMassB, invIB, lever, sum));
    double moment = (lever + 1.0) * sum;
    double scale = 1.0;
    if (reach > (double)M3_WARM_LIMIT)
    {
        scale = (double)M3_WARM_LIMIT / reach;
    }
    if (moment * scale > (double)M3_WARM_MOMENT_LIMIT)
    {
        scale = (double)M3_WARM_MOMENT_LIMIT / moment;
    }
    // Rounded down to a float, so a held impulse lands within the limits
    // and one past them by a hair is held too.
    m3real held = (m3real)scale;
    if ((double)held > scale)
    {
        uint32_t bits;
        memcpy(&bits, &held, sizeof(bits));
        bits -= 1u; // the next float toward zero; held is positive here
        memcpy(&held, &bits, sizeof(held));
    }
    return held;
}

// Cramer's rule, with its determinant.
static m3Vec3 Cramer(const m3Mat3* J, m3Vec3 b, m3real* detOut)
{
    m3Vec3 cxy = m3Cross3(J->cy, J->cz);
    m3real det = m3Dot3(J->cx, cxy);
    *detOut = det;
    if (det == 0.0f)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    m3real inv = 1.0f / det;
    m3Vec3 x;
    x.x = inv * m3Dot3(b, cxy);
    x.y = inv * m3Dot3(J->cx, m3Cross3(b, J->cz));
    x.z = inv * m3Dot3(J->cx, m3Cross3(J->cy, b));
    return x;
}

m3Vec3 m3Solve3AtScale(const m3Mat3* J, m3Vec3 b, m3real scale)
{
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    if (!(scale > 0.0f) || !(scale <= FLT_MAX))
    {
        return zero;
    }
    m3Mat3 unit = *J;
    m3Vec3* units[3] = {&unit.cx, &unit.cy, &unit.cz};
    for (int32_t c = 0; c < 3; ++c)
    {
        units[c]->x /= scale;
        units[c]->y /= scale;
        units[c]->z /= scale;
    }
    m3real det;
    m3Vec3 x = Cramer(&unit, b, &det);
    x = (m3Vec3){x.x / scale, x.y / scale, x.z / scale};
    return m3FiniteV3(x) ? x : zero;
}

// Solve J * x = b for a general 3x3 via Cramer's rule. A singular
// Jacobian returns zero, which leaves omega unchanged (the safe step).
// The direct result stands whenever the determinant is a normal float,
// which covers every ordinary body bit for bit; otherwise, zero included,
// J is solved at the scale of its largest entry: a determinant of tiny
// but well-conditioned entries underflows to zero, and only the scaled
// solve tells it from a singular J.
m3Vec3 m3Solve3(const m3Mat3* J, m3Vec3 b)
{
    m3real det;
    m3Vec3 x = Cramer(J, b, &det);
    m3real size = m3AbsF(det);
    if (size >= FLT_MIN && size <= FLT_MAX)
    {
        return x;
    }
    m3real scale = 0.0f;
    const m3Vec3* cols[3] = {&J->cx, &J->cy, &J->cz};
    for (int32_t c = 0; c < 3; ++c)
    {
        scale = m3MaxF(scale,
                       m3MaxF(m3AbsF(cols[c]->x), m3MaxF(m3AbsF(cols[c]->y), m3AbsF(cols[c]->z))));
    }
    return m3Solve3AtScale(J, b, scale);
}

bool m3Solve3Direct(const m3Mat3* J, m3Vec3 b, m3Vec3* x)
{
    m3real det;
    *x = Cramer(J, b, &det);
    return m3DirectHolds(det, J->cx.x * J->cy.y * J->cz.z);
}

bool m3DirectHolds(m3real det, m3real diag)
{
    return det >= FLT_MIN && det <= FLT_MAX && det >= 0x1p-12f * diag;
}

double m3WideDot(m3Vec3 u, m3Vec3 v)
{
    return (double)u.x * (double)v.x + (double)u.y * (double)v.y + (double)u.z * (double)v.z;
}

double m3WideForm(const m3Mat3* w, m3Vec3 u, m3Vec3 v)
{
    m3Vec3 rows[3] = {
        {w->cx.x, w->cy.x, w->cz.x}, {w->cx.y, w->cy.y, w->cz.y}, {w->cx.z, w->cy.z, w->cz.z}};
    return (double)u.x * m3WideDot(rows[0], v) + (double)u.y * m3WideDot(rows[1], v) +
           (double)u.z * m3WideDot(rows[2], v);
}

// a x b for double triples.
static void WideCross(const double* a, const double* b, double* out)
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static double WideDot3(const double* a, const double* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// (w t) x a, the column a body's turn adds to a point block, for
// t = a x e.
static void WideTurn(const m3Mat3* w, const double* a, const double* t, double* out)
{
    double wt[3] = {
        (double)w->cx.x * t[0] + (double)w->cy.x * t[1] + (double)w->cz.x * t[2],
        (double)w->cx.y * t[0] + (double)w->cy.y * t[1] + (double)w->cz.y * t[2],
        (double)w->cx.z * t[0] + (double)w->cy.z * t[1] + (double)w->cz.z * t[2],
    };
    WideCross(wt, a, out);
}

void m3WidePointBlock(double mass, const m3Mat3* invIA, m3Vec3 armA, const m3Mat3* invIB,
                      m3Vec3 armB, double k[9])
{
    double a[3] = {(double)armA.x, (double)armA.y, (double)armA.z};
    double b[3] = {(double)armB.x, (double)armB.y, (double)armB.z};
    for (int32_t c = 0; c < 3; ++c)
    {
        double e[3] = {c == 0 ? 1.0 : 0.0, c == 1 ? 1.0 : 0.0, c == 2 ? 1.0 : 0.0};
        double t[3];
        double turnA[3];
        double turnB[3];
        WideCross(a, e, t);
        WideTurn(invIA, a, t, turnA);
        WideCross(b, e, t);
        WideTurn(invIB, b, t, turnB);
        for (int32_t r = 0; r < 3; ++r)
        {
            k[3 * c + r] = mass * e[r] + (turnA[r] + turnB[r]);
        }
    }
}

m3Vec3 m3Solve3Wide(const double k[9], m3Vec3 b)
{
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    double rhs[3] = {(double)b.x, (double)b.y, (double)b.z};
    double yz[3];
    double ry[3];
    double xr[3];
    WideCross(k + 3, k + 6, yz);
    WideCross(rhs, k + 6, ry);
    WideCross(k + 3, rhs, xr);
    double det = WideDot3(k, yz);
    if (det == 0.0)
    {
        return zero;
    }
    m3Vec3 x = {(m3real)(WideDot3(rhs, yz) / det), (m3real)(WideDot3(k, ry) / det),
                (m3real)(WideDot3(k, xr) / det)};
    return m3FiniteV3(x) ? x : zero;
}

bool m3InvertPairAtScale(m3real k11, m3real k22, m3real k12, m3real scale, m3real out[3])
{
    if (!(scale > 0.0f) || !(scale <= FLT_MAX))
    {
        return false;
    }
    m3real a = k11 / scale;
    m3real b = k22 / scale;
    m3real d = k12 / scale;
    m3real det = a * b - d * d;
    if (!(det > 0.0f))
    {
        return false;
    }
    m3real inv = 1.0f / det;
    m3real r11 = b * inv / scale;
    m3real r22 = a * inv / scale;
    m3real r12 = -d * inv / scale;
    if (!m3FiniteF(r11) || !m3FiniteF(r22) || !m3FiniteF(r12))
    {
        return false;
    }
    out[0] = r11;
    out[1] = r22;
    out[2] = r12;
    return true;
}

bool m3SolvePairWide(double k11, double k22, double k12, m3real x, m3real y, m3real out[2])
{
    double det = k11 * k22 - k12 * k12;
    if (!(det > 0.0))
    {
        return false;
    }
    m3real s1 = (m3real)((k22 * (double)x - k12 * (double)y) / det);
    m3real s2 = (m3real)((k11 * (double)y - k12 * (double)x) / det);
    if (!m3FiniteF(s1) || !m3FiniteF(s2))
    {
        return false;
    }
    out[0] = s1;
    out[1] = s2;
    return true;
}

// v + d from double components, held within the speed limit. Past the
// limit the sum is scaled by its largest component first, so every
// product stays finite.
static m3Vec3 HeldSum(m3Vec3 v, double dx, double dy, double dz)
{
    double x = (double)v.x + dx;
    double y = (double)v.y + dy;
    double z = (double)v.z + dz;
    double cap = (double)M3_SPEED_LIMIT;
    if (x * x + y * y + z * z <= cap * cap)
    {
        return (m3Vec3){(m3real)x, (m3real)y, (m3real)z};
    }
    double ax = x < 0.0 ? -x : x;
    double ay = y < 0.0 ? -y : y;
    double az = z < 0.0 ? -z : z;
    double big = ax > ay ? ax : ay;
    big = big > az ? big : az;
    m3Vec3 unit = {(m3real)(x / big), (m3real)(y / big), (m3real)(z / big)};
    return m3CapSpeed(m3MulSV3(M3_SPEED_LIMIT, unit), M3_SPEED_LIMIT);
}

void m3PushHeld(m3World* world, int32_t index, m3real scale, m3Vec3 dir, m3Vec3 arm)
{
    m3Mat3 invI = m3WorldInvInertia(world, index);
    m3real invMass = world->bodies.invMass[index];
    m3Vec3 impulse = m3MulSV3(scale, dir);
    m3Vec3 v = m3Add3(world->bodies.linearVelocities[index], m3MulSV3(invMass, impulse));
    m3Vec3 w =
        m3Add3(world->bodies.angularVelocities[index], m3MulMV3(invI, m3Cross3(arm, impulse)));
    if (!m3FiniteV3(v) || !m3FiniteV3(w))
    {
        world->heldPushRetakes += 1;
        double jx = (double)scale * (double)dir.x;
        double jy = (double)scale * (double)dir.y;
        double jz = (double)scale * (double)dir.z;
        double tx = (double)arm.y * jz - (double)arm.z * jy;
        double ty = (double)arm.z * jx - (double)arm.x * jz;
        double tz = (double)arm.x * jy - (double)arm.y * jx;
        double m = (double)invMass;
        double i[9] = {(double)invI.cx.x, (double)invI.cx.y, (double)invI.cx.z,
                       (double)invI.cy.x, (double)invI.cy.y, (double)invI.cy.z,
                       (double)invI.cz.x, (double)invI.cz.y, (double)invI.cz.z};
        v = HeldSum(world->bodies.linearVelocities[index], m * jx, m * jy, m * jz);
        w = HeldSum(world->bodies.angularVelocities[index], i[0] * tx + i[3] * ty + i[6] * tz,
                    i[1] * tx + i[4] * ty + i[7] * tz, i[2] * tx + i[5] * ty + i[8] * tz);
    }
    world->bodies.linearVelocities[index] = m3CapSpeed(v, M3_SPEED_LIMIT);
    world->bodies.angularVelocities[index] = m3CapSpeed(w, M3_SPEED_LIMIT);
}
