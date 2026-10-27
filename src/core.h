// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for core.c: assertions, the finite-input
// checks and the profile clock. Every engine source reaches these
// through world_internal.h or allocator.h.

#ifndef MAUL3D_SRC_CORE_H
#define MAUL3D_SRC_CORE_H

#include "maul3d/base.h"
#include "maul3d/core_math.h"

#include <float.h>

// M3_ASSERT states an internal invariant: a state that cannot occur
// unless the library itself is wrong. Caller input is never asserted; it
// is refused with a status or counted as misuse. The check traps in every
// build without NDEBUG (Debug, the sanitizer and fuzz builds), so a broken
// invariant stops the run where it happened. Release builds compile it
// out without evaluating the condition, so the solver's inner loops do not
// pay for it.
#if defined(NDEBUG)
#define M3_ASSERT(cond) ((void)sizeof((cond) ? 1 : 0))
#else
#define M3_ASSERT(cond) ((cond) ? (void)0 : __builtin_trap())
#endif

// Monotonic milliseconds for the step profile: observation only, never a
// hash input.
double m3NowMs(void);

// Hostile-input guards: every def field that reaches
// simulation state must be finite. NaN comparisons are false and
// inf minus inf is NaN, so the x - x == 0 form refuses NaN and both
// infinities in one branchless test, no libm, no macro promotion.
static inline bool m3FiniteF(m3real x)
{
    return x - x == 0.0f; // NOLINT(misc-redundant-expression): the finite test
}

static inline bool m3FiniteD(double x)
{
    return x - x == 0.0; // NOLINT(misc-redundant-expression): the finite test
}

static inline bool m3IsNanF(m3real x)
{
    return x != x; // NOLINT(misc-redundant-expression): the NaN test
}

static inline bool m3FiniteV3(m3Vec3 v)
{
    return m3FiniteF(v.x) && m3FiniteF(v.y) && m3FiniteF(v.z);
}

static inline bool m3FinitePos3(m3Pos3 p)
{
    return m3FiniteD(p.x) && m3FiniteD(p.y) && m3FiniteD(p.z);
}

static inline bool m3FiniteQuat(m3Quat q)
{
    return m3FiniteF(q.x) && m3FiniteF(q.y) && m3FiniteF(q.z) && m3FiniteF(q.w);
}

// The fixed bounds a call refuses are public, in maul3d/base.h. Finite
// is not enough for what a def places in a body's frame or feeds into a
// mass: a finite but huge value turns masses, inertias and solver terms
// infinite, which a restore then refuses. The bounds and the reasons for
// their values:
// - the local limit bounds every offset in a body's frame, and the
//   density limit a shape's density; a convex shape's unit-density mass
//   is then capped at (2 M3_LOCAL_LIMIT)^3, below, which keeps it finite
//   at any density;
// - the position limit bounds what the step turns into float offsets;
// - the speed limit sits 250 times above the default linear cap, so no
//   scene meets it, and far enough inside float range that a velocity
//   times any mass, lever or step length the engine accepts stays finite;
// - gravity is bounded at about 1000 g, where the suites use at most 20;
// - frequencies at 1e5 Hz, where the suites use at most 30;
// - ratios at 1e4, where the suites use at most 500 (a restitution test),
//   and a drivetrain's gear times final drive stays within its square;
// - the step floor keeps the servo's turn, at most 2 / dt rad/s, within
//   the speed limit and its reach over dt within the kinematic speed
//   limit; the suites step 1/60;
// - forces at 1e20 N, where the suites use at most 1e7;
// - the mass floors bound every inverse mass and inertia, and the mass
//   ceiling every effective mass in the solve; the suites' lightest shape
//   is 3.2e-5 kg with 1.1e-10 kg m^2, their heaviest body 4.2e9 kg;
// - a contact point on a kinematic body moves within the kinematic speed
//   limit plus the spin times the longest lever (about 3.5e4 m, two
//   body-frame offsets), about 1.4e10 m/s, and times the heaviest body
//   and that lever again it stays near 5e34, inside float range; the
//   suites servo 3 km;
// - drive force and wheel spin divide by the wheel radius, which the
//   suites set to 0.3 m;
// - over the largest facing area (4e8 m^2) and the lightest body, the
//   blast limit stays finite before the speed limit holds the result;
//   the suites use at most 50 N s/m^2.
#define M3_UNIT_MASS_LIMIT 8.0e12f

static inline bool m3LocalF(m3real x)
{
    return x >= -M3_LOCAL_LIMIT && x <= M3_LOCAL_LIMIT; // false for NaN
}

static inline bool m3DensityF(float x)
{
    return x > 0.0f && x <= M3_DENSITY_LIMIT; // false for NaN
}

static inline bool m3LocalV3(m3Vec3 v)
{
    return m3LocalF(v.x) && m3LocalF(v.y) && m3LocalF(v.z);
}

static inline bool m3BoundedPos3(m3Pos3 p)
{
    return p.x >= -M3_POSITION_LIMIT && p.x <= M3_POSITION_LIMIT && p.y >= -M3_POSITION_LIMIT &&
           p.y <= M3_POSITION_LIMIT && p.z >= -M3_POSITION_LIMIT &&
           p.z <= M3_POSITION_LIMIT; // false for NaN
}

// Every body and soft particle stays within M3_POSITION_LIMIT, per
// component: creates and teleports place it there, the step holds it
// there and restore refuses one past it, so the float differences of any
// two positions stay finite. p held within the bound: past it, scaled
// toward the origin onto the bound, direction kept, and each component
// then clamped so rounding cannot leave it a hair outside. A position
// within the bound comes back unchanged.
static inline m3Pos3 m3HoldPosition(m3Pos3 p)
{
    double big = p.x < 0.0 ? -p.x : p.x;
    double ay = p.y < 0.0 ? -p.y : p.y;
    double az = p.z < 0.0 ? -p.z : p.z;
    big = ay > big ? ay : big;
    big = az > big ? az : big;
    if (!(big > M3_POSITION_LIMIT))
    {
        return p;
    }
    double scale = M3_POSITION_LIMIT / big;
    double* c[3] = {&p.x, &p.y, &p.z};
    for (int32_t k = 0; k < 3; ++k)
    {
        double x = *c[k] * scale;
        *c[k] = x > M3_POSITION_LIMIT ? M3_POSITION_LIMIT
                                      : (x < -M3_POSITION_LIMIT ? -M3_POSITION_LIMIT : x);
    }
    return p;
}

// v scaled down to length cap when it is longer. A vector within the cap
// comes back unchanged, and one whose squared length overflows is scaled
// by its largest component first, so the result is finite either way. v
// may hold an infinite component but never a NaN.
static inline m3Vec3 m3CapSpeed(m3Vec3 v, m3real cap)
{
    m3real v2 = m3Dot3(v, v);
    M3_ASSERT(v2 == v2);
    if (!(v2 > cap * cap))
    {
        return v;
    }
    if (v2 <= FLT_MAX)
    {
        return m3MulSV3(cap / sqrtf(v2), v);
    }
    m3real big = m3MaxF(m3AbsF(v.x), m3MaxF(m3AbsF(v.y), m3AbsF(v.z)));
    m3Vec3 u;
    if (big <= FLT_MAX)
    {
        u = m3MulSV3(1.0f / big, v);
    }
    else
    {
        // An infinite component keeps only the infinite axes' signs.
        u.x = m3AbsF(v.x) > FLT_MAX ? (v.x > 0.0f ? 1.0f : -1.0f) : 0.0f;
        u.y = m3AbsF(v.y) > FLT_MAX ? (v.y > 0.0f ? 1.0f : -1.0f) : 0.0f;
        u.z = m3AbsF(v.z) > FLT_MAX ? (v.z > 0.0f ? 1.0f : -1.0f) : 0.0f;
    }
    return m3MulSV3(cap / sqrtf(m3Dot3(u, u)), u);
}

// What a stored contact or joint impulse may change, per component, in a
// body's velocity (m/s) or spin (rad/s) when the next step's warm start
// lands it, and the most its moment, impulse times lever, may reach.
// Prepare measures each loaded impulse at that step's own masses,
// inertias and levers, as if every component acted at the longest lever
// on every axis, and scales it, direction kept, to within both.
#define M3_WARM_LIMIT        1.0e13f
#define M3_WARM_MOMENT_LIMIT 1.0e38f

// x within [-limit, limit]; false for NaN.
static inline bool m3WithinF(m3real x, m3real limit)
{
    return x >= -limit && x <= limit;
}

static inline bool m3WithinV3(m3Vec3 v, m3real limit)
{
    return m3WithinF(v.x, limit) && m3WithinF(v.y, limit) && m3WithinF(v.z, limit);
}

// The status checks of caller input against the published bounds: a
// value outside them is m3_errorLimit, infinities included, and NaN is
// m3_errorInvalid. m3BothChecks joins two checks, an invalid value
// outranking one past a bound, so a call's status does not depend on the
// order it checks its fields in.
static inline m3Result m3BothChecks(m3Result a, m3Result b)
{
    if (a == m3_errorInvalid || b == m3_errorInvalid)
    {
        return m3_errorInvalid;
    }
    return a != m3_success ? a : b;
}

static inline m3Result m3CheckValid(bool valid)
{
    return valid ? m3_success : m3_errorInvalid;
}

// x within [lo, hi].
static inline m3Result m3CheckRange(m3real x, m3real lo, m3real hi)
{
    if (x >= lo && x <= hi)
    {
        return m3_success;
    }
    return m3IsNanF(x) ? m3_errorInvalid : m3_errorLimit;
}

static inline m3Result m3CheckWithin(m3real x, m3real limit)
{
    return m3CheckRange(x, -limit, limit);
}

static inline m3Result m3CheckWithinV3(m3Vec3 v, m3real limit)
{
    return m3BothChecks(m3BothChecks(m3CheckWithin(v.x, limit), m3CheckWithin(v.y, limit)),
                        m3CheckWithin(v.z, limit));
}

static inline m3Result m3CheckLocalV3(m3Vec3 v)
{
    return m3CheckWithinV3(v, M3_LOCAL_LIMIT);
}

// x from zero to limit: below zero is invalid, as for a speed, whose
// sign has no meaning.
static inline m3Result m3CheckNonNegative(m3real x, m3real limit)
{
    return x >= 0.0f ? m3CheckRange(x, 0.0f, limit) : m3_errorInvalid;
}

// x at least floor and finite: a value below the floor is past a bound,
// and so is an infinite one.
static inline m3Result m3CheckAtLeast(m3real x, m3real floor)
{
    return m3CheckRange(x, floor, FLT_MAX);
}

// x positive and at most limit: zero or below is invalid, as for a
// density, whose sign has no meaning.
static inline m3Result m3CheckPositive(m3real x, m3real limit)
{
    return x > 0.0f ? m3CheckRange(x, 0.0f, limit) : m3_errorInvalid;
}

static inline m3Result m3CheckPosition(m3Pos3 p)
{
    const double c[3] = {p.x, p.y, p.z};
    m3Result result = m3_success;
    for (int32_t k = 0; k < 3; ++k)
    {
        m3Result r = m3_success;
        if (!(c[k] >= -M3_POSITION_LIMIT && c[k] <= M3_POSITION_LIMIT))
        {
            // NOLINTNEXTLINE(misc-redundant-expression): the NaN test
            r = c[k] != c[k] ? m3_errorInvalid : m3_errorLimit;
        }
        result = m3BothChecks(result, r);
    }
    return result;
}

// A count from zero to max; a negative count is invalid.
static inline m3Result m3CheckCount(int32_t count, int32_t max)
{
    if (count < 0)
    {
        return m3_errorInvalid;
    }
    return count <= max ? m3_success : m3_errorLimit;
}

#endif // MAUL3D_SRC_CORE_H
