// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark: times the scenes of scenes.c and prints the timings.
// Wall time is not repeatable on shared machines, so this is never a
// check; the scenes' world hashes are checked by test/test_bench.c.
//
//   maul3d_bench [steps]   steps per scene, 300 by default

#ifndef _WIN32
#define _POSIX_C_SOURCE 199309L // clock_gettime under strict C
#endif

#include "scenes.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
static double NowMs(void)
{
    LARGE_INTEGER freq;
    LARGE_INTEGER now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return 1000.0 * (double)now.QuadPart / (double)freq.QuadPart;
}
#else
#include <time.h>
static double NowMs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1000.0 * (double)ts.tv_sec + (double)ts.tv_nsec / 1.0e6;
}
#endif

static void Report(const char* name, int32_t steps, double ms)
{
    printf("M3_BENCH %-10s steps=%d totalMs=%9.2f perStepMs=%7.4f\n", name, steps, ms,
           ms / (double)steps);
}

int main(int argc, char** argv)
{
    int32_t steps = 300;
    if (argc > 1 && atoi(argv[1]) > 0)
    {
        steps = atoi(argv[1]);
    }
    static const char* const names[BENCH_SCENE_COUNT] = {"pyramid", "hulljam", "meshfield",
                                                         "meshgrand"};
    double ms = 0.0;
    for (int32_t scene = 0; scene < BENCH_SCENE_COUNT; ++scene)
    {
        BenchStepScene((BenchScene)scene, steps, NowMs, &ms);
        Report(names[scene], steps, ms);
    }
    BenchVoxfort(steps, NowMs, &ms);
    Report("voxfort", steps, ms);

    // The city block needs its three phases: twice the base step count.
    double phases[3];
    BenchCityblock(2 * steps, NowMs, phases);
    Report("cityblock", 2 * steps, phases[0] + phases[1] + phases[2]);
    printf("M3_BENCH cityblock.phases settleMs=%9.2f stormMs=%9.2f aftermathMs=%9.2f\n", phases[0],
           phases[1], phases[2]);

    BenchRollback rollback = BenchRollback5k(steps, NowMs);
    printf("M3_BENCH scale5k.rollback bytes=%d snapshotMs=%.2f restoreMs=%.2f\n", rollback.bytes,
           rollback.snapshotMs, rollback.restoreMs);

    BenchSmoke10k(steps / 2, NowMs, &ms);
    Report("smoke10k", steps / 2, ms);
    return 0;
}
