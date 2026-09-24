// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Deterministic 3D math. World positions are 64-bit (m3Pos3) and
// everything local is 32-bit (m3real). Every operation is plain IEEE
// arithmetic under -ffp-contract=off, so the bits agree on every
// platform; the transcendentals are the engine's own.

#ifndef MAUL3D_CORE_MATH_H
#define MAUL3D_CORE_MATH_H

#include "maul3d/base.h"

#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef float m3real;

#define M3_PI 3.14159265358979323846f

    typedef struct m3Vec3
    {
        m3real x, y, z;
    } m3Vec3;

    /// World-space position, double precision (large-world support and
    /// the delta-position solver both lean on this split).
    typedef struct m3Pos3
    {
        double x, y, z;
    } m3Pos3;

    /// Rotation quaternion, x y z vector part, w scalar part.
    typedef struct m3Quat
    {
        m3real x, y, z, w;
    } m3Quat;

    typedef struct m3Transform
    {
        m3Pos3 p;
        m3Quat q;
    } m3Transform;

    /// Column-major 3x3 (columns cx, cy, cz).
    typedef struct m3Mat3
    {
        m3Vec3 cx, cy, cz;
    } m3Mat3;

    typedef struct m3CosSin
    {
        m3real c, s;
    } m3CosSin;

    /// Column-major matrix times vector: cx*v.x + cy*v.y + cz*v.z.
    static inline m3Vec3 m3MulMV3(m3Mat3 m, m3Vec3 v)
    {
        m3Vec3 result = {m.cx.x * v.x + m.cy.x * v.y + m.cz.x * v.z,
                         m.cx.y * v.x + m.cy.y * v.y + m.cz.y * v.z,
                         m.cx.z * v.x + m.cy.z * v.y + m.cz.z * v.z};
        return result;
    }

    static inline m3Mat3 m3MakeZeroMat3(void)
    {
        m3Mat3 m;
        m3Vec3 zero = {0.0f, 0.0f, 0.0f};
        m.cx = zero;
        m.cy = zero;
        m.cz = zero;
        return m;
    }

    /// Pinned minimum: exactly (a < b ? a : b), in this operand order,
    /// on every platform. MSVC x64 lowers the ternary through MINSS
    /// which matches. It is spelled out rather than assumed because
    /// some arm64 compilers lower min and max differently.
    static inline m3real m3MinF(m3real a, m3real b)
    {
        return a < b ? a : b;
    }

    /// Pinned maximum: exactly (a > b ? a : b), in this operand order.
    static inline m3real m3MaxF(m3real a, m3real b)
    {
        return a > b ? a : b;
    }

    static inline m3real m3ClampF(m3real a, m3real lo, m3real hi)
    {
        return m3MaxF(lo, m3MinF(a, hi));
    }

    static inline m3real m3AbsF(m3real a)
    {
        return a < 0.0f ? -a : a;
    }

    static inline m3Vec3 m3Add3(m3Vec3 a, m3Vec3 b)
    {
        m3Vec3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
        return result;
    }

    static inline m3Vec3 m3Sub3(m3Vec3 a, m3Vec3 b)
    {
        m3Vec3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
        return result;
    }

    static inline m3Vec3 m3MulSV3(m3real s, m3Vec3 v)
    {
        m3Vec3 result = {s * v.x, s * v.y, s * v.z};
        return result;
    }

    static inline m3Vec3 m3Neg3(m3Vec3 v)
    {
        m3Vec3 result = {-v.x, -v.y, -v.z};
        return result;
    }

    static inline m3real m3Dot3(m3Vec3 a, m3Vec3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static inline m3Vec3 m3Cross3(m3Vec3 a, m3Vec3 b)
    {
        m3Vec3 result = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        return result;
    }

    static inline m3real m3LengthSquared3(m3Vec3 v)
    {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    static inline m3real m3Length3(m3Vec3 v)
    {
        // sqrtf is IEEE correctly rounded, the one libm call the bit
        // law allows on the hot path.
        return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    /// Normalize with a fixed degenerate fallback of {0, 1, 0}: one
    /// rule, documented, so a zero vector never turns into NaN and
    /// never depends on the caller.
    static inline m3Vec3 m3Normalize3(m3Vec3 v)
    {
        m3real length = m3Length3(v);
        if (length < 1.19209290e-7f)
        {
            m3Vec3 result = {0.0f, 1.0f, 0.0f};
            return result;
        }
        m3real inv = 1.0f / length;
        m3Vec3 result = {inv * v.x, inv * v.y, inv * v.z};
        return result;
    }

    /// Convert any angle into the range [-pi, pi]. remainderf is IEEE
    /// exact (like sqrt), so this is deterministic.
    static inline m3real m3UnwindAngle(m3real radians)
    {
        return remainderf(radians, 2.0f * M3_PI);
    }

    static inline m3Quat m3MakeIdentityQuat(void)
    {
        m3Quat result = {0.0f, 0.0f, 0.0f, 1.0f};
        return result;
    }

    static inline m3Quat m3MulQuat(m3Quat a, m3Quat b)
    {
        m3Quat q;
        q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
        return q;
    }

    /// Normalize with a fixed degenerate fallback of identity.
    static inline m3Quat m3NormalizeQuat(m3Quat q)
    {
        m3real mag = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (mag < 1.19209290e-7f)
        {
            return m3MakeIdentityQuat();
        }
        m3real inv = 1.0f / mag;
        m3Quat result = {inv * q.x, inv * q.y, inv * q.z, inv * q.w};
        return result;
    }

    /// Rotate a vector by a unit quaternion: v' = v + w*t + q x t with
    /// t = 2 (q x v). Plain mul and add only, no fused ops.
    static inline m3Vec3 m3RotateVec3(m3Quat q, m3Vec3 v)
    {
        m3Vec3 u = {q.x, q.y, q.z};
        m3Vec3 t = m3MulSV3(2.0f, m3Cross3(u, v));
        return m3Add3(m3Add3(v, m3MulSV3(q.w, t)), m3Cross3(u, t));
    }

    /// Rotate by the conjugate (inverse for unit quaternions).
    static inline m3Vec3 m3InvRotateVec3(m3Quat q, m3Vec3 v)
    {
        m3Quat c = {-q.x, -q.y, -q.z, q.w};
        return m3RotateVec3(c, v);
    }

    /// Integrate a rotation by an angular displacement (radians, world
    /// frame): q2 = normalize(q1 + 0.5 * (dr, 0) * q1), the first-order
    /// step of the quaternion derivative. Renormalization every call is the
    /// 3D numeric contract: drift never accumulates.
    static inline m3Quat m3IntegrateRotation(m3Quat q, m3Vec3 deltaRotation)
    {
        m3real ax = 0.5f * deltaRotation.x;
        m3real ay = 0.5f * deltaRotation.y;
        m3real az = 0.5f * deltaRotation.z;
        m3Quat qd;
        qd.w = -(ax * q.x + ay * q.y + az * q.z);
        qd.x = ax * q.w + ay * q.z - az * q.y;
        qd.y = -ax * q.z + ay * q.w + az * q.x;
        qd.z = ax * q.y - ay * q.x + az * q.w;
        m3Quat q2 = {q.x + qd.x, q.y + qd.y, q.z + qd.z, q.w + qd.w};
        return m3NormalizeQuat(q2);
    }

    /// Deterministic cosine and sine (a quarter-turn reduction with a
    /// split pi/2, then Taylor series) and atan2 (a pi/6 argument shift,
    /// then the Taylor series): hand rolled because platform libm
    /// implementations disagree in the last bits.
    M3_API m3CosSin m3ComputeCosSin(m3real radians);
    M3_API m3real m3Atan2(m3real y, m3real x);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_CORE_MATH_H
