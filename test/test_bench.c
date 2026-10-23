// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark scenes as golden scenes: five and ten thousand bodies,
// destruction at scale and a 5k-body rollback, where broadphase rebuilds,
// island splits and the solver at full load are exercised as the small
// golden scenes cannot. They take about twenty seconds in Release and
// far longer in Debug and sanitizer builds, so this test runs only in
// Release builds and only in the nightly check (label bench). The values
// come from test/hashes.txt like the golden scenes', and test_golden.c
// shows how to regenerate them.

#include "test_harness.h"

#include "../bench/scenes.h"

#ifndef M3_GOLDEN_BENCH_PYRAMID
#define M3_GOLDEN_BENCH_PYRAMID 0
#endif
#ifndef M3_GOLDEN_BENCH_HULLJAM
#define M3_GOLDEN_BENCH_HULLJAM 0
#endif
#ifndef M3_GOLDEN_BENCH_MESHFIELD
#define M3_GOLDEN_BENCH_MESHFIELD 0
#endif
#ifndef M3_GOLDEN_BENCH_MESHGRAND
#define M3_GOLDEN_BENCH_MESHGRAND 0
#endif
#ifndef M3_GOLDEN_BENCH_VOXFORT
#define M3_GOLDEN_BENCH_VOXFORT 0
#endif
#ifndef M3_GOLDEN_BENCH_CITYBLOCK
#define M3_GOLDEN_BENCH_CITYBLOCK 0
#endif
#ifndef M3_GOLDEN_BENCH_SMOKE10K
#define M3_GOLDEN_BENCH_SMOKE10K 0
#endif

#define STEPS 300

static void Expect(const char* name, uint64_t hash, uint64_t committed)
{
    printf("M3_BENCH_%s_HASH=%016llx\n", name, (unsigned long long)hash);
    if (hash != committed)
    {
        printf("FAIL: the %s bench scene hashed %016llx; test/hashes.txt has %016llx\n", name,
               (unsigned long long)hash, (unsigned long long)committed);
        s_failures += 1;
    }
}

int main(void)
{
    static const char* const names[BENCH_SCENE_COUNT] = {"PYRAMID", "HULLJAM", "MESHFIELD",
                                                         "MESHGRAND"};
    static const uint64_t committed[BENCH_SCENE_COUNT] = {
        M3_GOLDEN_BENCH_PYRAMID, M3_GOLDEN_BENCH_HULLJAM, M3_GOLDEN_BENCH_MESHFIELD,
        M3_GOLDEN_BENCH_MESHGRAND};
    double unused = 0.0;
    for (int32_t scene = 0; scene < BENCH_SCENE_COUNT; ++scene)
    {
        Expect(names[scene], BenchStepScene((BenchScene)scene, STEPS, NULL, &unused),
               committed[scene]);
    }
    Expect("VOXFORT", BenchVoxfort(STEPS, NULL, &unused), M3_GOLDEN_BENCH_VOXFORT);

    double phases[3];
    uint64_t city = BenchCityblock(2 * STEPS, NULL, phases);
    Expect("CITYBLOCK", city, M3_GOLDEN_BENCH_CITYBLOCK);
    CHECK(BenchCityblock(2 * STEPS, NULL, phases) == city, "the city block matches its twin run");

    BenchRollback rollback = BenchRollback5k(STEPS, NULL);
    CHECK(rollback.bytes > 0 && rollback.restoreExact, "the 5k-body world restores exactly");
    CHECK(rollback.refightExact, "the re-fought 5k-body tail lands on the same bits");

    Expect("SMOKE10K", BenchSmoke10k(STEPS / 2, NULL, &unused), M3_GOLDEN_BENCH_SMOKE10K);
    if (s_failures == 0)
    {
        printf("test_bench: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
