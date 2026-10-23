# Determinism

maul3d promises that the same world, given the same calls, reaches the same bits on every supported platform, compiler, build type, SIMD backend and worker count. This page describes how the tests hold that promise.

## Golden scenes

A golden value pins a promised output, so it is part of the specification. There are few of them: each scene covers several subsystems at once, so a deliberate change of results regenerates a handful of values. The scenes are in [`test/test_golden.c`](../../test/test_golden.c), and their values are in [`test/hashes.txt`](../../test/hashes.txt).

Five scenes pin simulation results. `STACK`, `ROLLBACK` and `JOURNAL` do the same job as the maul2d scenes of the same names, so the two engines' golden lists line up. `DESTRUCTION` and `VEHICLES` cover what only the 3D engine has.

| Name | What it runs | Extra check |
|---|---|---|
| `STACK` | A sphere pyramid with droppers, a column of every convex family, a compound with offset children, a triangle-mesh ramp, a height-field bowl, and every joint kind with its springs, motors and limits switched on. | A four-worker twin must match the serial run, and a warm world must step without allocating. |
| `DESTRUCTION` | A voxel wall with crates on its rim, carved by a box edit and by an explosion; the islands it splits into fracture away. | |
| `VEHICLES` | A car with a gearbox driving a circle, a character climbing voxel stairs, cloth and a jelly in gusting wind, and crates in moving water. | |
| `ROLLBACK` | The tour (below), snapshotted halfway. | After a restore the rerun must land on the same bits, a re-snapshot must be byte-equal, and a different continuation must diverge. |
| `JOURNAL` | The tour, then a seeded random walk of creates, joints, impulses, destroys, voxel edits and explosions, all recorded from the world's birth. | The replay into a fresh world and the same session on four workers must both land on the recorded bits. |

The tour is one world holding the terrain, the joints, the vehicles and the voxel wall of the other scenes, and a script that makes every journaled call at least once: each create and destroy, each setter, each command and each world knob, a pre-solve veto included. So `ROLLBACK` checks that every subsystem's state rides the snapshot, and `JOURNAL` checks that every op replays. Two checks keep that coverage from eroding: the journal scene requires every op code the journal defines to appear on its tape, so a new op without a tour call fails, and the tour fails when any of its calls is refused, which the world's `misuse` counter shows.

Two lines pin formats rather than results:

- `ROLLBACK_SNAPSHOT`: the bytes of the `ROLLBACK` scene's snapshot.
- `JOURNAL_TAPE`: the bytes of the `JOURNAL` session's tape.

They sit on their own lines because the snapshot and the tape record public structs as bytes. A change to those structs moves a format line while every result line stays the same.

The scenes draw every random number in a statement of its own. C leaves the evaluation order of function arguments and initializers to the compiler, and GCC and Clang choose differently, so two draws in one call give different scenes on different compilers.

## Benchmark scenes

The seven `BENCH_*` lines pin the benchmark scenes in [`bench/scenes.c`](../../bench/scenes.c), which the benchmark and [`test/test_bench.c`](../../test/test_bench.c) share:

| Name | Scene |
|---|---|
| `BENCH_PYRAMID` | A large sphere pyramid. |
| `BENCH_HULLJAM` | Boxes and hull rocks jammed in a pit. |
| `BENCH_MESHFIELD` | Mixed shapes on a height field. |
| `BENCH_MESHGRAND` | Boxes raining on a triangle mesh of about sixty thousand triangles. |
| `BENCH_VOXFORT` | A hollow voxel fort shelled by charges and raked by bullets; the host turns its fragments into bodies. |
| `BENCH_CITYBLOCK` | Five thousand bodies on welded voxel towers that a fracture wave carves. A twin run must match. |
| `BENCH_SMOKE10K` | Ten thousand mixed bodies raining onto a plane. |

`test_bench` also rolls a five-thousand-body world back and requires the restore and the re-fought tail to be exact. These scenes reach what the small ones cannot: broadphase rebuilds, island splits and the solver at full load.

They are pinned in a separate test because of their cost. In Release they take about twenty seconds, and Debug and sanitizer builds are far slower. So `test_bench` is registered only in Release builds that are not sanitized, under the ctest label `bench`. The pull request presets exclude that label. The `<cell>-bench` presets run only it, and the `bench` job of the CI workflow runs them on main and every night on Linux (x64 and arm64, GCC and Clang), macOS and Windows (x64 and arm64). The benchmark executable itself only prints timings.

## Checking and regenerating

`test/CMakeLists.txt` turns each line `M3_<NAME>_HASH=<hex>` of `test/hashes.txt` into the definition `M3_GOLDEN_<NAME>`, so every CI cell checks the values compiled in, with no file to read at run time. Configuration fails on a malformed or repeated line. A missing value reads as zero, so its scene fails.

Each scene prints its line, so a deliberate change of results is one regeneration that rewrites the whole file:

```sh
for p in linux-gcc-14 linux-gcc-14-bench; do cmake --preset $p && cmake --build --preset $p; done
(out/linux-gcc-14/test_golden; out/linux-gcc-14-bench/test_bench) | grep -E '^M3_[A-Z0-9_]+_HASH=' > test/hashes.txt
```

The pull request that regenerates values says which lines moved and why, and CI shows every cell agreeing on the new values.
