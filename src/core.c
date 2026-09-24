// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core: version, the FNV-1a hash every determinism gate folds through,
// the assert hook, refusals and the profile clock.

#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 199309L // NOLINT(bugprone-reserved-identifier): clock_gettime
#endif

#include "core.h"

#include "world_internal.h"

#include "maul3d/base.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <intrin.h>
#include <windows.h>
#else
#include <time.h>
#endif

// Monotonic milliseconds for the step profile. Observer data
// only: never a hash input, never serialized, never fed back into
// the simulation. The bench keeps its own copy on purpose (tools do
// not reach into engine internals).
double m3NowMs(void)
{
#ifdef _WIN32
    LARGE_INTEGER frequency;
    LARGE_INTEGER counter;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    return 1000.0 * (double)counter.QuadPart / (double)frequency.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1000.0 * (double)ts.tv_sec + (double)ts.tv_nsec / 1000000.0;
#endif
}

int m3GetVersion(void)
{
    return M3_VERSION_MAJOR * 10000 + M3_VERSION_MINOR * 100 + M3_VERSION_PATCH;
}

static m3AssertFn* s_assertHandler = NULL;
static void* s_assertContext = NULL;

void m3SetAssertHandler(m3AssertFn* handler, void* context)
{
    s_assertHandler = handler;
    s_assertContext = context;
}

int m3ReportToHost(const char* message, const char* where)
{
    return s_assertHandler != NULL && s_assertHandler(message, where, 0, s_assertContext) != 0;
}

void m3AssertFail(const char* condition, const char* file, int line)
{
    if (s_assertHandler != NULL && s_assertHandler(condition, file, line, s_assertContext) != 0)
    {
        return; // handled by the host
    }
    fprintf(stderr, "maul3d assertion failed: %s (%s:%d)\n", condition, file, line);
    abort();
}

#if defined(_MSC_VER)
#define M3_THREAD_LOCAL __declspec(thread)
#else
#define M3_THREAD_LOCAL _Thread_local
#endif

// One slot per thread: a refusal on one thread never overwrites the
// reason another thread is about to read.
static M3_THREAD_LOCAL m3Result s_lastResult = m3_success;

m3Result m3LastResult(void)
{
    return s_lastResult;
}

void m3Refuse(m3World* world, m3Result reason)
{
    s_lastResult = reason;
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
