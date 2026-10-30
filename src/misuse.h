// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Refusals of a caller's input against a world, counted in the world's
// misuse counter.

#ifndef MAUL3D_SRC_MISUSE_H
#define MAUL3D_SRC_MISUSE_H

#include "world_internal.h"

#include <string.h>

// Counts a caller's bug against a world in m3Counters.misuseCount: an invalid
// argument, one past a published bound, a stale id, or a call the world
// does not allow now. A full pool, a refusing allocator, a CPU, an ABI
// mismatch and malformed stored bytes are not bugs and count nothing.
// world may be NULL. Returns reason, so a status call can refuse with
// `return m3Refuse(world, reason);`.
m3Result m3Refuse(const m3World* world, m3Result reason);
uint64_t m3MisuseCount(const m3World* world);

// Step and Draw hold the world busy while they run, host code they call
// included (task hooks, the pre-solve callback, debug draw callbacks, the
// allocator), and every call holds it busy while the host's allocator
// runs. A writer called while it is busy changes nothing and is refused
// with m3_errorBusy. A NULL world is never busy.
void m3WorldEnter(const m3World* world);
void m3WorldLeave(const m3World* world);
bool m3WorldBusy(const m3World* world);

// The status for an id that names no live object: m3_errorInvalid for
// the null id or a NULL world, which names no object at all,
// m3_errorStale for any other.
static inline m3Result m3MissingId(const m3World* world, uint32_t index1)
{
    return world == NULL || index1 == 0 ? m3_errorInvalid : m3_errorStale;
}

// The status of a public create whose internal refused: an invalid def,
// or one past a bound, counts as misuse; a full pool, or a bound the
// world's other shapes and joints decide, does not.
static inline m3Result m3CreateRefusal(const m3World* world, int32_t refusal)
{
    if (refusal == M3_CREATE_HELD)
    {
        return m3_errorLimit;
    }
    return m3Refuse(world, (m3Result)refusal);
}

// Whether a byte count fits this target's size_t, so a buffer of that
// length can exist: always on a 64-bit target, and up to SIZE_MAX on a
// 32-bit one. A size past it is a caller's bug.
static inline bool m3SizeFitsMemory(int64_t bytes)
{
#if SIZE_MAX < INT64_MAX
    return bytes <= (int64_t)SIZE_MAX;
#else
    (void)bytes;
    return true;
#endif
}

// A list call's array and capacity: a NULL array with capacity 0
// measures, any array takes a capacity of 0 or more, and anything else is
// refused.
static inline bool m3ListOutputValid(const void* arrayOut, int32_t capacity)
{
    return capacity >= 0 && (arrayOut != NULL || capacity == 0);
}

// Zeroes a list call's array over its whole capacity when the call fails,
// so no item an earlier call wrote is read as the answer. An array the
// check above refuses is left as it is: its length is not known.
static inline void m3ClearListOutput(void* arrayOut, int32_t capacity, size_t itemBytes)
{
    if (arrayOut != NULL && capacity > 0)
    {
        memset(arrayOut, 0, (size_t)capacity * itemBytes);
    }
}

// A list call's refusal of an argument a correct program never passes:
// its array cleared, one misuse counted, and the total 0 to return.
static inline int32_t m3RefuseList(const m3World* world, void* arrayOut, int32_t capacity,
                                   size_t itemBytes)
{
    m3ClearListOutput(arrayOut, capacity, itemBytes);
    (void)m3Refuse(world, m3_errorInvalid);
    return 0;
}

#endif
