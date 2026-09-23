# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [Unreleased]

Work toward 0.0.1, the first release of the reworked library.

### Added

- `samples/`: the stack, rollback, car and character programs from the
  samples page are now real programs built with the library, so they
  cannot drift from the API. `samples/minimal` is the installed-
  package consumer, moved from `test/consumer`.
- `docs/api.md`: the API reference, generated from the headers by
  `tools/gen_api.py` (the same script Maul2D uses), and
  `maul3d/maul3d.h`, an umbrella header that includes the whole API.
- `m3replay` and `m3lockstep` are installed with the library.
- `test/hashes.txt` holds the expected determinism hashes, and
  `tools/check_hashes.py` compares a test run with them, so a change
  that moves a hash the same way on every platform is caught too.
- `m3LastResult`: the reason for the last refusal on the calling
  thread (invalid input, capacity, or configuration), and
  `m3Counters.misuse`, which counts invalid arguments against a live
  world. Every public function now records a reason when it refuses;
  before, refusals were silent.

### Changed

- The version history restarts at 0.0.1. Earlier numbered releases
  were withdrawn.
- The `m3SolvePlanes` documentation now states that only the first 64
  planes take part.
- `docs/manual.md` is now `docs/guide.md`, the name the family uses.
- The CMake options follow one family scheme: `MAUL3D_BUILD_TESTS`,
  `_BUILD_SAMPLES`, `_BUILD_BENCH`, `_BUILD_TOOLS`, `_BUILD_TESTBED`,
  `_BUILD_SHARED`, `_INSTALL`, `_WERROR`, `_SANITIZE`, `_TSAN` and
  `_COVERAGE`. Tests, samples, benchmarks and tools default to on only
  when the project is built on its own, not when it is added with
  `add_subdirectory`.
- Warnings are errors only with `MAUL3D_WERROR=ON` (CI turns it on),
  so a newer compiler no longer breaks a consumer's build.
- The CMake modules in `cmake/`, the package config template and the
  pkg-config template are shared with the sibling engine. The pkg-
  config file is relocatable and no longer lists a thread library the
  engine does not use.
- Every test suite uses the shared `test/test_harness.h` instead of
  its own copy of the check macro.
- `m3SetAssertHandler` takes a context pointer, like Maul2D's, and
  replaces `m3SetAssertHandlerCtx`. Refusing input never asserts; the
  `M3_ASSERT` macro and `m3AssertFail` are internal now.
- CI is the family workflow shared with Maul2D. It adds arm64 Linux
  and Windows cells, runs every suite under WebAssembly,
  ThreadSanitizer and a shared-library build, checks clang-tidy and
  the package consumer, and compares hashes with `test/hashes.txt`.
  Tagged releases now publish packages and the testbed for Linux,
  macOS and Windows.
- The public `maul3d/math.h` header is now `maul3d/core_math.h`, so it
  can never shadow the C library's `math.h` when `include/maul3d`
  lands on an include path.

### Removed

- The unused 4-wide SIMD header, its test and the `MAUL3D_SIMD`
  option. No engine code included it, so the README's claim that
  the default build used SIMD kernels was not true. The engine is
  scalar until real vector kernels land.

### Fixed

- A step that stalled on a full pair table or starved scratch leaked
  the previous step's pair and manifold stash. The stash now lives in
  the world, sized once at creation, so a step never allocates.
- World creation no longer writes through a null pointer when the
  allocator refuses. Every refusal now frees what was allocated and
  returns a null id; a failing-allocator sweep test covers each
  allocation in turn.
- Byte sizes derived from world capacities are computed in 64 bits
  and checked. Capacities whose sizes cannot be represented are
  refused instead of overflowing.
- A journal recording now fails loudly, like an overflow, when an op's
  payload cannot be allocated, instead of silently missing the op.
- `m3World_DiffReport` returns -1 instead of crashing when it cannot
  allocate its working rows.
- Overlap queries and `m3World_CollideMover` silently stopped
  collecting after 256 shapes, and because they sorted only what they
  had gathered, a busy query returned whatever the tree visited first
  instead of the lowest shape slots. They now keep the lowest slots
  that fit the caller's array, in ascending order, with no fixed
  limit. The mover's infinite planes are merged into that order as its
  documentation always promised, instead of being appended after the
  other shapes.
- `m3World_Restore` now also ranges the header's pair count and tree
  cursors before overwriting anything. A snapshot with a hostile pair
  count used to be accepted and crashed the next step.
- The voxel escape search indexed its visited set out of bounds (at -1
  in the corner cell) when a solid cell continued into a welded
  neighbor chunk. It now treats that direction as closed and looks for
  the exit inside the chunk. The cityblock benchmark pin moved because
  its destruction crosses welded borders.
- Two hull contact paths read the edge before index 0 when a face had
  no edges, which only malformed hull data can cause; they now produce
  no contact. GJK simplex caches are fully zero-initialized, and the
  time-of-impact setup no longer copies unused cache entries.
