# Development

How to build, test and change maul3d as a maintainer. Building and using
it as a library is in the [README](../../README.md) and
[getting started](../user/getting-started.md).

## Toolchain

The organization's CI image, `ghcr.io/rawframe-project/ci-cpp`, has
everything below; CI runs every Linux job in it, pinned by digest. On
another machine:

- CMake and Ninja.
- GCC 14 or newer, or Clang 19 or newer, with the sanitizer runtimes:
  `cmake/CompilerFlags.cmake` refuses older ones, which cannot build
  C23, such as a system's default GCC 13. The presets use `gcc-14` and
  `clang-20`, and the checks `clang-tidy-20`.
- `clang-format` at the version the CI image pins; another version
  formats differently.
- Python 3 for the checks, and `python3-markdown` for the documentation
  site.
- 32-bit libraries for the `linux-i686-*` presets.
- Optional: Emscripten for the `wasm-node` preset.

## Build options

| Option | Effect |
|---|---|
| `MAUL3D_WERROR` | Warnings as errors, as CI builds. |
| `MAUL3D_SANITIZE` | AddressSanitizer and UBSan. |
| `MAUL3D_TSAN` | ThreadSanitizer. |
| `MAUL3D_COVERAGE` | gcov instrumentation. |
| `MAUL3D_SIMD=scalar` | The scalar kernels on any CPU; results are the same bits as AVX2 and NEON. |
| `MAUL3D_BUILD_SHARED` | A shared library instead of a static one. |
| `MAUL3D_TEST_PACKAGE` | The package test: installs the build and builds `samples/minimal` against it. |
| `MAUL3D_FUZZ` | The libFuzzer targets (Clang only). |
| `MAUL3D_BUILD_TESTBED` | The interactive testbed, which fetches raylib. |
| `MAUL3D_WEB_UNIT_LINK` | Under Emscripten, the plain unit tests link at `-O1`, and the golden and determinism suites keep the build's level ([checks](checks.md)). |

`MAUL3D_BUILD_TESTS`, `_SAMPLES`, `_BENCH`, `_TOOLS` and `MAUL3D_INSTALL`
are on when maul3d is the top-level project. `M3_ASSERT` is live in every
build without `NDEBUG` and traps with `__builtin_trap`, so a debugger or a
core dump stops at the failing check; Release builds compile it out
without evaluating the condition.

## Presets and running one test

Every CI cell is a CMake preset in `CMakePresets.json`: a configure, a
build and a test preset of the same name, built in `out/<preset>`.
`linux-clang-20-asan` is a Debug build on the scalar backend with the
sanitizers and the assertions on; `linux-clang-20-tsan` is a Debug build
with ThreadSanitizer; `linux-shared` builds the shared library with the
package test; `linux-i686-gcc-14` and `linux-i686-clang-20` build for
32-bit x86. Each suite is one executable and one CTest test, labeled with
its name:

```sh
cmake --preset linux-clang-20-asan
cmake --build --preset linux-clang-20-asan
ctest --preset linux-clang-20-asan -L joints
./out/linux-clang-20-asan/test_joints
```

On the shared development machine, run full builds through the
workspace's heavy-job wrapper. ThreadSanitizer does not start inside the
CI image there unless the container runs with
`--security-opt seccomp=unconfined`; run the TSan preset on the host.

## Tests

Each suite is `test/test_<label>.c`, registered with `maul_add_test` in
`test/CMakeLists.txt`. Most test through the public headers; a suite
marked `WHITEBOX` includes `src/` and reaches internal symbols, so the
shared build leaves it out. `test/helpers.h` wraps the calls the suites
build scenes with; a test about why a call refuses calls the public
function and checks its status. `test_guide.c` runs the C snippets of
`docs/user/` as written.

## Golden values

`test/hashes.txt` pins the golden scenes of `test/test_golden.c`, the
warm start scenes of `test/test_warmhold.c` and the benchmark scenes of
`test/test_bench.c`; CMake compiles each line in, and a malformed or
repeated line fails the configure. A scene that lands elsewhere fails
and prints the line it got. When a change moves results on purpose,
regenerate every line from a Release build of each, and say in the pull
request which lines moved and why:

```sh
(out/linux-gcc-14/test_golden; out/linux-gcc-14/test_warmhold;
 out/linux-gcc-14-bench/test_bench) | grep -E '^M3_[A-Z0-9_]+_HASH=' > test/hashes.txt
```

A moved line is a change of result, which [release](release.md) says how
to title and when it bumps `M3_SOLVER_REV`. A line that moves
unexpectedly is a determinism bug until proven otherwise: compare the
scalar and AVX2 builds first, since a value only one backend gives points
at a kernel.

## Benchmarks

`bench/scenes.c` holds the benchmark scenes. `maul3d_bench [steps]`
(300 steps by default) times them and prints one `M3_BENCH` line each:
`pyramid`, `hulljam`, `meshfield`, `meshgrand`, `jointrig`, `vehiclerig`,
`ragdolls`, `voxfort`, `cityblock` with its three phases, the snapshot
and restore of `scale5k.rollback`, `smoke10k` and `softfield`. The
timings are never a check;
`test_bench`, labeled `bench`, checks their hashes, in Release builds
without sanitizers only. A change meant to affect speed gives before and
after numbers from one machine in its pull request.

## Finding where two runs part

A hash that differs between two cells, two machines or two builds is
found with the replay tools, built in `out/<preset>/` with the tools:

- `m3replay record <out.m3j>` records the built-in demo as a replay
  container, `info` describes one, and `verify` replays one and checks
  its final hash.
- `m3replay diff <a.m3j> <b.m3j>` finds the first step where two
  sessions' hashes part, by bisection, checks that the step before
  agrees, and lists the bodies that differ there, worst first.
- `m3replay seek <file> <N>` and `play <file> <A> <B>` scrub one.
- `m3lockstep` runs the rollback recipe: a peer that predicts and rolls
  back against one that steps only on confirmed inputs. It checks their
  hashes at every checkpoint and exits nonzero when they differ.

A host's own session becomes a container with `m3EncodeReplay`
([the journal](../user/journal-and-replays.md)). Write one on each side,
then `diff` the two files.

## Reproducing a fuzz finding

A finding in CI prints only a report: the target, the kind of each
finding and the sanitizer's error. Fuzz the target locally until it finds
the same fault; `tools/fuzz_target.sh` keeps each finding in
`out/fuzz-crashes/<target>/` and the log in `out/fuzz-logs/`:

```sh
cmake -B out/fuzz -G Ninja -DCMAKE_C_COMPILER=clang-20 -DMAUL3D_FUZZ=ON -DMAUL3D_BUILD_TESTS=OFF
cmake --build out/fuzz
out/fuzz/fuzz_seeds out/seeds
tools/fuzz_target.sh out/fuzz/fuzz_snapshot -max_total_time=300 out/corpus/fuzz_snapshot out/seeds/fuzz_snapshot
```

A crash a fuzzer finds becomes a test in the suite of its module, with
the smallest input that still fails or the scene it reduces to.

## Shared family files

These files are synced from the canonical copies in the organization's
`.github` repository (`maul/family_sync.py --write maul3d`), and the
release gate checks that they match: `.clang-format`, `.clang-tidy`,
`.editorconfig`, `.gitattributes`, `.gitignore`, `bench/CMakeLists.txt`,
the modules in `cmake/`, `docs/internals/maul-conventions.md`,
`test/test_harness.h`, `test/test_fenv.h`, `test/test_fpenv.c`,
`src/cpu.c`, `src/cpu.h`, `src/hash.c`, `src/simd.h`, `src/fpenv.c`,
`src/fpenv.h`, the checks and scripts in `tools/` (`build_docs.py`,
`check_docs.py`, `check_exports.py`, `check_float_env.py`,
`check_guide.py`, `check_lengths.py`, `check_modules.py`,
`check_source.py`, `check_test_hygiene.py`, `fuzz_target.sh`,
`gen_api.py`, `run_quarantine.sh`, `tidy_filter.sh`) and the fixtures in
`tools/test_hygiene/`. Change the canonical copy first, then sync it and
move the workflow pins to the same tag.
