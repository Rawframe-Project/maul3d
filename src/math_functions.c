// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Deterministic angle functions. libm's transcendentals are not required
// to agree bit for bit across platforms, so the engine evaluates its own:
// only +, -, *, / and the IEEE-exact floorf, sqrtf and remainderf are
// used, and the build turns floating-point contraction off.

#include "core.h"

#include "maul3d/core_math.h"

// atan2 from the ratio a = min(|x|, |y|) / max(|x|, |y|) in [0, 1]. Above
// tan(pi/12) the identity atan(a) = pi/6 + atan((sqrt(3) a - 1) / (a +
// sqrt(3))) moves the argument into [-tan(pi/12), tan(pi/12)], where the
// odd Taylor series through t^11 is accurate to below 3e-9. The octant and
// quadrant then follow from the signs and the larger component.
m3real m3Atan2(m3real y, m3real x)
{
    // (0, 0) returns 0 instead of NaN: the sim path is NaN-free by contract.
    if (x == 0.0f && y == 0.0f)
    {
        return 0.0f;
    }
    m3real ax = m3AbsF(x);
    m3real ay = m3AbsF(y);
    m3real a = m3MinF(ay, ax) / m3MaxF(ay, ax);
    m3real base = 0.0f;
    if (a > 0.2679492f)
    {
        a = (1.7320508f * a - 1.0f) / (a + 1.7320508f);
        base = 0.5235988f;
    }
    m3real t2 = a * a;
    m3real r =
        base +
        a * (1.0f +
             t2 * (-0.33333334f +
                   t2 * (0.2f + t2 * (-0.14285715f + t2 * (0.11111111f + t2 * -0.09090909f)))));
    if (ay > ax)
    {
        r = 1.5707964f - r;
    }
    if (x < 0.0f)
    {
        r = 3.1415927f - r;
    }
    if (y < 0.0f)
    {
        r = -r;
    }
    return r;
}

// Cosine and sine. The angle, folded into [-pi, pi], is reduced to r in
// [-pi/4, pi/4] around the nearest multiple of pi/2, with pi/2 split into a
// float-exact high part and a low correction so the reduction loses no
// precision. On that interval the Taylor series of sine through r^9 and of
// cosine through r^8 are accurate to well below float resolution
// (truncation under 3e-8), and the quadrant maps the pair back. The pair
// is then normalized so c*c + s*s is one to float rounding.
m3CosSin m3ComputeCosSin(m3real radians)
{
    const m3real halfPiHigh = 1.5707964f;    // pi/2 rounded to float
    const m3real halfPiLow = -4.371139e-08f; // pi/2 - halfPiHigh
    m3real x = m3UnwindAngle(radians);
    m3real k = floorf(x * 0.63661975f + 0.5f);
    m3real r = (x - k * halfPiHigh) - k * halfPiLow;
    m3real r2 = r * r;
    m3real s =
        r * (1.0f + r2 * (-0.16666667f +
                          r2 * (0.008333334f + r2 * (-0.0001984127f + r2 * 2.7557319e-06f))));
    m3real c =
        1.0f + r2 * (-0.5f + r2 * (0.041666668f + r2 * (-0.0013888889f + r2 * 2.4801588e-05f)));
    m3real cosine;
    m3real sine;
    switch ((int32_t)k & 3)
    {
    case 0:
        cosine = c;
        sine = s;
        break;
    case 1:
        cosine = -s;
        sine = c;
        break;
    case 2:
        cosine = -c;
        sine = -s;
        break;
    default:
        cosine = s;
        sine = -c;
        break;
    }
    m3real mag = sqrtf(sine * sine + cosine * cosine);
    m3real invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
    m3CosSin cs = {cosine * invMag, sine * invMag};
    return cs;
}
