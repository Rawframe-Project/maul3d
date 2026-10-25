// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Soft constraint coefficients, the 3x3 solve and world inverse inertia.

#include "solver_math.h"

#include <float.h>

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
// which covers every ordinary body bit for bit; otherwise J is solved at
// the scale of its largest entry.
m3Vec3 m3Solve3(const m3Mat3* J, m3Vec3 b)
{
    m3real det;
    m3Vec3 x = Cramer(J, b, &det);
    m3real size = m3AbsF(det);
    if (det == 0.0f || (size >= FLT_MIN && size <= FLT_MAX))
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
