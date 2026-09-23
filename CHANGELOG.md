# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [Unreleased]

Work toward 0.0.1, the first release of the reworked library.

### Changed

- The version history restarts at 0.0.1. Earlier numbered releases
  were withdrawn.
- The `m3SolvePlanes` documentation now states that only the first 64
  planes take part.

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
