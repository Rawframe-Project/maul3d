// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark scenes, shared by the benchmark and test/test_bench.c.
// Each runner builds its scene, runs it, and returns the world hash it
// ends on. With a clock, it also reports the time the steps took; without
// one (NULL), it reads no clock.

#ifndef MAUL3D_BENCH_SCENES_H
#define MAUL3D_BENCH_SCENES_H

#include <stdbool.h>
#include <stdint.h>

typedef double BenchClock(void);

typedef enum BenchScene
{
    BENCH_PYRAMID,   // a sphere pyramid
    BENCH_HULLJAM,   // boxes and hull rocks jammed in a pit
    BENCH_MESHFIELD, // mixed shapes on a height field
    BENCH_MESHGRAND, // boxes raining on a large triangle mesh
    BENCH_SCENE_COUNT,
} BenchScene;

// The four plain scenes: build, then step.
uint64_t BenchStepScene(BenchScene scene, int32_t steps, BenchClock* clock, double* ms);

// A voxel fort shelled by charges and raked by bullets, its fragments
// spawned as bodies.
uint64_t BenchVoxfort(int32_t steps, BenchClock* clock, double* ms);

// Five thousand bodies rain onto welded voxel towers, which a fracture
// wave then carves: settle, storm and aftermath, a third of the steps
// each, timed apart.
uint64_t BenchCityblock(int32_t totalSteps, BenchClock* clock, double phaseMs[3]);

typedef struct BenchRollback
{
    int32_t bytes;
    double snapshotMs;
    double restoreMs;
    bool restoreExact; // the restored world hashes as the snapshotted one
    bool refightExact; // the re-fought tail hashes as the first
} BenchRollback;

// The city block run for settleSteps, snapshotted, run on, restored and
// run on again.
BenchRollback BenchRollback5k(int32_t settleSteps, BenchClock* clock);

// Ten thousand mixed bodies rain onto a plane.
uint64_t BenchSmoke10k(int32_t steps, BenchClock* clock, double* ms);

#endif
