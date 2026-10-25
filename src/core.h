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

// The step holds every body and particle velocity (m/s and rad/s) and
// every joint row's correction speed within this, whatever the world's
// own caps say: 250 times the default linear cap, so no scene meets it,
// and far enough inside float range that a velocity times any mass,
// lever or step length the engine accepts stays finite.
#define M3_SPEED_LIMIT 1.0e5f

// v scaled down to length cap when it is longer. A vector within the cap
// comes back unchanged, and one whose squared length overflows is scaled
// by its largest component first, so the result is finite either way.
static inline m3Vec3 m3CapSpeed(m3Vec3 v, m3real cap)
{
    m3real v2 = m3Dot3(v, v);
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

#endif // MAUL3D_SRC_CORE_H
