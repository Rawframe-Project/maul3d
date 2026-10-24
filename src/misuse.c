// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's misuse counter, which reader calls on several threads may
// bump at once, and its busy count, which Draw calls on several threads
// may hold at once.

#include "misuse.h"

#if defined(_MSC_VER)
#include <intrin.h>
#endif

m3Result m3Refuse(const m3World* world, m3Result reason)
{
    if (world == NULL || reason == m3_errorCapacity || reason == m3_errorUnsupported ||
        reason == m3_errorVersion)
    {
        return reason; // not the caller's bug against this world
    }
    // Reader calls refuse too, possibly on several threads at once, so the
    // counter is bumped atomically; it is the one field a reader writes.
    m3World* counted = (m3World*)world;
#if defined(_MSC_VER)
    _InterlockedIncrement64(&counted->misuseCount);
#else
    __atomic_fetch_add(&counted->misuseCount, 1, __ATOMIC_RELAXED);
#endif
    return reason;
}

uint64_t m3MisuseCount(const m3World* world)
{
#if defined(_MSC_VER)
    return (uint64_t)_InterlockedCompareExchange64((volatile long long*)&world->misuseCount, 0, 0);
#else
    return (uint64_t)__atomic_load_n(&world->misuseCount, __ATOMIC_RELAXED);
#endif
}

void m3WorldEnter(const m3World* world)
{
    m3World* busy = (m3World*)world;
#if defined(_MSC_VER)
    _InterlockedIncrement64(&busy->busyCount);
#else
    __atomic_fetch_add(&busy->busyCount, 1, __ATOMIC_ACQ_REL);
#endif
}

void m3WorldLeave(const m3World* world)
{
    m3World* busy = (m3World*)world;
#if defined(_MSC_VER)
    _InterlockedDecrement64(&busy->busyCount);
#else
    __atomic_fetch_sub(&busy->busyCount, 1, __ATOMIC_ACQ_REL);
#endif
}

bool m3WorldBusy(const m3World* world)
{
    if (world == NULL)
    {
        return false;
    }
#if defined(_MSC_VER)
    return _InterlockedCompareExchange64((volatile long long*)&world->busyCount, 0, 0) != 0;
#else
    return __atomic_load_n(&world->busyCount, __ATOMIC_ACQUIRE) != 0;
#endif
}
