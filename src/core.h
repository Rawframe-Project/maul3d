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

// Finite is not enough for what a def places in a body's frame or feeds
// into a mass: a finite but huge value turns masses, inertias and solver
// terms infinite, which a restore then refuses. These bounds lie far past
// any real scene: an offset in a body's frame (joint anchors, a shape's
// local position, a lattice spacing, a hull vertex), a shape's density,
// a convex shape's unit-density mass, which hull vertices within the local
// limit cap at (2 M3_LOCAL_LIMIT)^3 so it stays finite at any density,
// and a world position the step turns into float offsets.
#define M3_LOCAL_LIMIT     1.0e4f
#define M3_DENSITY_LIMIT   1.0e6f
#define M3_UNIT_MASS_LIMIT 8.0e12f
#define M3_POSITION_LIMIT  1.0e15

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

// How far from the origin any body or soft particle goes, per component,
// m: restore refuses a position past it and the step holds every mover
// and particle within it. Creates and teleports keep M3_POSITION_LIMIT,
// so a body reaches this only by moving at the kinematic limit for about
// 1e8 s.
#define M3_REACH_LIMIT 1.0e18

static inline bool m3WithinReach(m3Pos3 p)
{
    return p.x >= -M3_REACH_LIMIT && p.x <= M3_REACH_LIMIT && p.y >= -M3_REACH_LIMIT &&
           p.y <= M3_REACH_LIMIT && p.z >= -M3_REACH_LIMIT &&
           p.z <= M3_REACH_LIMIT; // false for NaN
}

// p held within M3_REACH_LIMIT: past it, scaled toward the origin onto
// the bound, direction kept, and each component then clamped to the bound
// so rounding cannot leave it a hair outside. A position within the bound
// comes back unchanged.
static inline m3Pos3 m3HoldReach(m3Pos3 p)
{
    double big = p.x < 0.0 ? -p.x : p.x;
    double ay = p.y < 0.0 ? -p.y : p.y;
    double az = p.z < 0.0 ? -p.z : p.z;
    big = ay > big ? ay : big;
    big = az > big ? az : big;
    if (!(big > M3_REACH_LIMIT))
    {
        return p;
    }
    double scale = M3_REACH_LIMIT / big;
    double* c[3] = {&p.x, &p.y, &p.z};
    for (int32_t k = 0; k < 3; ++k)
    {
        double x = *c[k] * scale;
        *c[k] = x > M3_REACH_LIMIT ? M3_REACH_LIMIT : (x < -M3_REACH_LIMIT ? -M3_REACH_LIMIT : x);
    }
    return p;
}

// The step holds every body and particle velocity (m/s and rad/s) and
// every joint row's correction speed within this, whatever the world's
// own caps say: 250 times the default linear cap, so no scene meets it,
// and far enough inside float range that a velocity times any mass,
// lever or step length the engine accepts stays finite.
#define M3_SPEED_LIMIT 1.0e5f

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

// What the API, replay and restore accept, beyond finiteness. Each limit
// sits far past every value the suites, samples and bench scenes use, and
// together they keep every product the step forms finite.
// Gravity, m/s^2: about 1000 g; the suites use at most 20.
#define M3_ACCEL_LIMIT 1.0e4f
// A spring, contact, steering or suspension frequency, or the wind's
// gust frequency, Hz: the suites use at most 30.
#define M3_HERTZ_LIMIT 1.0e5f
// A dimensionless factor (a damping ratio, restitution, a gear or pulley
// ratio, a gravity scale, a gust or push scale): the suites use at most
// 500, a restitution test; a drivetrain's gear times final drive stays
// within its square.
#define M3_RATIO_LIMIT 1.0e4f
// A step length, s: the suites step 1/60. The floor keeps the servo's
// turn (at most 2 / dt rad/s) within M3_SPEED_LIMIT and its reach over dt
// within M3_KINEMATIC_SPEED_LIMIT.
#define M3_STEP_MIN   1.0e-4f
#define M3_STEP_LIMIT 10.0f
// A force (N) or torque (N m), held or accumulated, and a joint or
// vehicle effort: the suites use at most 1e7.
#define M3_FORCE_LIMIT 1.0e20f
// A mass-bearing shape's mass and smallest principal inertia, and a
// body's total mass (also a character's and a soft particle's): the
// suites' lightest shape is 3.2e-5 kg with 1.1e-10 kg m^2, their
// heaviest body 4.2e9 kg. The floors bound every inverse mass and
// inertia; the ceiling bounds every effective mass in the solve.
#define M3_MASS_FLOOR    1.0e-6f
#define M3_INERTIA_FLOOR 1.0e-12f
#define M3_MASS_LIMIT    1.0e20f
// A kinematic body's linear speed, m/s, set or servoed; it spins within
// M3_SPEED_LIMIT like any body. A contact point on a kinematic body moves
// within this plus the spin times the longest lever (about 3.5e4 m, two
// body-frame offsets), about 1.4e10 m/s, and times the heaviest body
// (1e20 kg) and that lever again it stays near 5e34, inside float range.
// A servo target lies within M3_SERVO_REACH, this over the shortest step.
// The suites servo 3 km.
#define M3_KINEMATIC_SPEED_LIMIT 1.0e10f
#define M3_SERVO_REACH           1.0e6
// What a stored contact or joint impulse may change, per component, in a
// body's velocity (m/s) or spin (rad/s) when the next step's warm start
// lands it, and the most its moment, impulse times lever, may reach.
// Prepare measures each loaded impulse at that step's own masses,
// inertias and levers, as if every component acted at the longest lever
// on every axis, and scales it, direction kept, to within both.
#define M3_WARM_LIMIT        1.0e13f
#define M3_WARM_MOMENT_LIMIT 1.0e38f
// A vehicle wheel's radius, m: drive force and wheel spin divide by it.
// The suites use 0.3.
#define M3_WHEEL_RADIUS_FLOOR 1.0e-3f
// An explosion's impulse per square meter, N s/m^2: the suites use at
// most 50. Over the largest facing area (4e8 m^2) and the lightest body
// it stays finite before the speed limit holds the result.
#define M3_BLAST_LIMIT 1.0e10f

// x within [-limit, limit]; false for NaN.
static inline bool m3WithinF(m3real x, m3real limit)
{
    return x >= -limit && x <= limit;
}

static inline bool m3WithinV3(m3Vec3 v, m3real limit)
{
    return m3WithinF(v.x, limit) && m3WithinF(v.y, limit) && m3WithinF(v.z, limit);
}

#endif // MAUL3D_SRC_CORE_H
