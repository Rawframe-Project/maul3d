// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's misuse counter, which reader calls on several threads may
// bump at once.

#include "misuse.h"

#if defined(_MSC_VER)
#include <intrin.h>
#endif

void m3Refuse(m3World* world, m3Result reason)
{
    m3NoteResult(reason);
    if (world != NULL && reason == m3_errorInvalid)
    {
        // Reader-class calls refuse too, possibly on several threads at
        // once, so the counter is bumped atomically.
#if defined(_MSC_VER)
        _InterlockedIncrement64(&world->misuseCount);
#else
        __atomic_fetch_add(&world->misuseCount, 1, __ATOMIC_RELAXED);
#endif
    }
}

uint64_t m3MisuseCount(const m3World* world)
{
#if defined(_MSC_VER)
    return (uint64_t)_InterlockedCompareExchange64((volatile long long*)&world->misuseCount, 0, 0);
#else
    return (uint64_t)__atomic_load_n(&world->misuseCount, __ATOMIC_RELAXED);
#endif
}
