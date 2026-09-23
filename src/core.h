// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for core.c: assertions, refusals and the
// monotonic clock. Every engine source reaches these through
// world_internal.h or allocator.h.

#ifndef MAUL3D_SRC_CORE_H
#define MAUL3D_SRC_CORE_H

#include "maul3d/base.h"

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

typedef struct m3World m3World;

// Refuses a caller's input: records the reason for m3LastResult on this
// thread and, for an invalid argument against a live world, counts it in
// m3Counters.misuse. world may be NULL.
void m3Refuse(m3World* world, m3Result reason);
uint64_t m3MisuseCount(const m3World* world);

// Monotonic milliseconds for the step profile: observation only, never a
// hash input.
double m3NowMs(void);

#endif // MAUL3D_SRC_CORE_H
