// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for core.c: assertions, refusals and the
// monotonic clock. Every engine source reaches these through
// world_internal.h or allocator.h.

#ifndef MAUL3D_SRC_CORE_H
#define MAUL3D_SRC_CORE_H

#include "maul3d/base.h"
#include "maul3d/core_math.h"

// Floating-point contraction breaks bit-identical results. The build
// turns it off for GCC and Clang; for MSVC the pragma does it in every
// source, including on arm64, where the /fp:contract- switch is not
// available.
#if defined(_MSC_VER)
#pragma fp_contract(off)
#endif

// Internal invariants only: states that cannot happen unless the engine
// itself is wrong. Caller input is refused with m3Refuse, never asserted.
#if defined(NDEBUG)
#define M3_ASSERT(cond) ((void)0)
#else
#define M3_ASSERT(cond) ((cond) ? (void)0 : m3AssertFail(#cond, __FILE__, __LINE__))
#endif

// Reports a failed invariant to the host's assert handler, and aborts
// unless the handler returns nonzero.
void m3AssertFail(const char* condition, const char* file, int line);

// Hands a message to the host's assert handler as a line-0 report.
// Returns nonzero when the host took it.
int m3ReportToHost(const char* message, const char* where);

// Whether this CPU runs the SIMD backend the library was built for,
// checked once; a failure is reported to the host or printed.
int m3VerifyCpuBackend(void);

typedef struct m3World m3World;

// Refuses a caller's input: records the reason for m3LastResult on this
// thread and, for an invalid argument against a live world, counts it in
// m3Counters.misuse. world may be NULL.
void m3Refuse(m3World* world, m3Result reason);
uint64_t m3MisuseCount(const m3World* world);

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

#endif // MAUL3D_SRC_CORE_H
