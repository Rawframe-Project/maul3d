// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core: the version, the result names and the profile clock.

#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 199309L // NOLINT(bugprone-reserved-identifier): clock_gettime
#endif

#include "core.h"

#include "maul3d/base.h"

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

m3Version m3GetVersion(void)
{
    return (m3Version){M3_VERSION_MAJOR, M3_VERSION_MINOR, M3_VERSION_PATCH};
}

const char* m3ResultName(m3Result result)
{
    switch (result)
    {
    case m3_success:
        return "m3_success";
    case m3_errorInvalid:
        return "m3_errorInvalid";
    case m3_errorCapacity:
        return "m3_errorCapacity";
    case m3_errorStale:
        return "m3_errorStale";
    case m3_errorUnsupported:
        return "m3_errorUnsupported";
    case m3_errorState:
        return "m3_errorState";
    case m3_errorVersion:
        return "m3_errorVersion";
    default:
        return "unknown m3Result";
    }
}
