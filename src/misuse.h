// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Refusals of a caller's input against a world, counted in the world's
// misuse counter.

#ifndef MAUL3D_SRC_MISUSE_H
#define MAUL3D_SRC_MISUSE_H

#include "world_internal.h"

// Counts a caller's bug against a world in m3Counters.misuse: an invalid
// argument, a stale id, or a call the world does not allow now. A full
// pool, a refusing allocator, a CPU or an ABI mismatch are not bugs and
// count nothing. world may be NULL. Returns reason, so a status call can
// refuse with `return m3Refuse(world, reason);`.
m3Result m3Refuse(const m3World* world, m3Result reason);
uint64_t m3MisuseCount(const m3World* world);

// Step and Draw hold the world busy while they run, host code they call
// included (task hooks, the pre-solve callback, debug draw callbacks, the
// allocator). A writer called while it is busy changes nothing and is
// refused with m3_errorState. A NULL world is never busy.
void m3WorldEnter(const m3World* world);
void m3WorldLeave(const m3World* world);
bool m3WorldBusy(const m3World* world);

// The status for an id that names no live object: m3_errorInvalid for
// the null id, m3_errorStale for any other.
static inline m3Result m3MissingId(uint32_t index1)
{
    return index1 == 0 ? m3_errorInvalid : m3_errorStale;
}

// The status of a public create whose internal refused: an invalid def
// counts as misuse, a full pool does not.
static inline m3Result m3CreateRefusal(const m3World* world, int32_t refusal)
{
    return refusal == M3_CREATE_FULL ? m3_errorCapacity : m3Refuse(world, m3_errorInvalid);
}

#endif
