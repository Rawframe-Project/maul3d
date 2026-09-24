# Maul3D

[![ci](https://github.com/Rawframe-Project/maul3d/actions/workflows/ci.yml/badge.svg)](https://github.com/Rawframe-Project/maul3d/actions/workflows/ci.yml)

A deterministic 3D physics engine for games, written in C17 with a
plain C API, no dependencies and an MIT license. Maul3D is the 3D
member of the Maul family; [Maul2D](https://github.com/Rawframe-Project/maul2d)
is its 2D sibling and follows the same rules.

The same inputs produce the same bits on every supported platform.
Snapshots restore a world exactly and it resumes identically, and a
command journal records a session and replays it byte for byte, with
id verification and atomic backout. That makes rollback netcode,
lockstep multiplayer, replays and server-side verification a few
calls away:

```c
int32_t size = m3World_SnapshotSize(world);
m3World_Snapshot(world, buffer, size);
/* ... mispredicted steps ... */
m3World_Restore(world, buffer, size); // bit-exact resimulation from here
```

## Features

- **Rigid bodies**: spheres, capsules, cylinders (faceted prisms
  through the hull path), convex hulls of up to 64 vertices, triangle
  meshes of up to 65k triangles with a BVH, heightfields, infinite
  planes and compound children with local transforms; static,
  kinematic and dynamic bodies; forces, impulses, motion locks,
  kinematic targets, runtime type switching, per-shape materials with
  conveyor velocities and gusting wind fields.
- **Voxel destruction inside rollback**: chunked voxel shapes with
  merged-box collision, journaled carving, fracture events with island
  recipes for host-spawned fragments, voxel continuous collision and
  seam welding.
- **Explosions in one call**: `m3World_Explode` pushes every body by
  the area it shows to the blast, carves voxel chunks and shoves soft
  particles, journaled and exact under rollback.
- **Solver**: soft-step contacts with speculative margins, warm
  starting, graph-colored SIMD solving and sub-steps; 64-bit positions
  keep worlds exact far from the origin.
- **Seven joint types**: spherical (with cone and twist limits),
  revolute, prismatic, fixed, distance, a generic six-degree-of-freedom
  joint and a wheel joint, with limits, motors, springs with position
  targets, constraint force readback and breaking with events.
- **Vehicles**: a raycast car with suspension, tire friction circles
  and a drivetrain (torque curve, gearbox with automatic shifting,
  clutch), or rigid wheel-joint carts.
- **Character controller**: a collide-and-slide capsule with step
  climbing, slope limits, ground snapping, moving platforms, pushing,
  and crouching guarded by a stand-up overlap check.
- **Soft bodies**: XPBD particle lattices (boxes, ropes, cloth) that
  collide with the rigid world and with each other, can be anchored
  to bodies or to other lattices and respond to wind.
- **Replays**: the M3J1 container seals a snapshot, a journal and the
  final hash into one file. The `m3replay` tool records, verifies,
  seeks, plays and diffs two runs down to the first diverging frame;
  `m3lockstep` runs a two-peer lockstep exchange in one process and
  checks both timelines at every checkpoint.
- **Events and queries**: contact, sensor, hit, body move and joint
  break events and pre-solve vetoes; rays, sphere, capsule, hull and
  box casts and overlap queries, all filterable and in canonical
  order.
- **Continuous collision** for fast bodies, island sleeping and tuning
  settings.
- **Integration**: 236 public functions, full state readback, debug
  draw as wireframe and solid triangle streams, counters and
  profiling, allocator and assert hooks.

## Getting started

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

A C17 compiler is required; with MSVC that means Visual Studio 2022 or
newer, the first version that can turn off floating-point
contraction. x64 builds use AVX2 and FMA (Haswell, 2013, and later);
configure with `-DMAUL3D_SIMD=scalar` for a portable build that
produces identical results. An AVX2 build started on a CPU without
AVX2 refuses to create worlds instead of crashing.

`cmake --install` installs the library, the headers, a CMake package
and a pkg-config file, so `find_package(maul3d)` and
`pkg-config maul3d` both work. Build a shared library with
`-DMAUL3D_BUILD_SHARED=ON`.

The [samples](docs/samples.md) show the API at work: a falling stack,
the rollback loop, a car that turns into a tank and a character on
the stairs. `samples/minimal` is a standalone project that finds the
installed package.

The interactive testbed (built on raylib, which the library itself
does not use) builds with `-DMAUL3D_BUILD_TESTBED=ON`. Its fourteen
scenes cover benchmarks, voxel destruction, three kinds of vehicle, a
playable character, cloth in gusting wind and a heightfield, with a
recording panel that seals the session into an M3J1 replay and
verifies it. Hold R and time runs backward.

## Determinism

- IEEE arithmetic only: no fast math and no floating-point
  contraction, enforced at configure time.
- Fixed tessellations and canonical ordering on every path.
- Journaled defs are treated as untrusted input on replay.
- CI compares the determinism hashes printed by the tests across
  eleven cells: GCC, Clang and MSVC on x64 and arm64 Linux and
  Windows, Clang on arm64 macOS, Debug builds, a sanitizer build and
  WebAssembly. They must also match the values committed in
  `test/hashes.txt`.

## Benchmarks

The scenes in `bench/` (a pyramid, hull and mesh fields, a voxel
fort, a 5000-body city block with destruction and a 10,000-body smoke
test) print their timings and their final world hashes. The hashes
are pinned in `bench/pins.txt`, and CI fails when one moves.

## Status

Current version: 0.0.1. Until 1.0.0 the API, the ABI and the snapshot
and journal formats may change in any minor release; the
[changelog](CHANGELOG.md) records every change. Defs carry a cookie,
so a def that was not initialized with its `m3Default...Def`
function is refused. Snapshots and journal tapes belong to one
library version and are refused by any other.

## Documentation

- [The documentation site](https://rawframe-project.github.io/maul3d/):
  the guide, the samples, the conventions and the changelog as web
  pages.
- [The API reference](docs/api.md): every public function with its
  documentation, generated from the headers.
- [The guide](docs/guide.md): the engine and host contract, chapter
  by chapter: rollback, the journal, destruction, vehicles, the
  character, soft bodies, replays, lockstep networking and an
  integration checklist.
- [The conventions](docs/conventions.md): the rules both engines
  follow, from naming to commits.
- [Design records](docs/adr/README.md): the decisions behind the
  rules and the architecture.
- [The changelog](CHANGELOG.md): every release and what changed.
- [Bindings](bindings/): starting points for C# (a single P/Invoke
  file) and Godot (a GDExtension skeleton).
- [CONTRIBUTING.md](CONTRIBUTING.md): how to contribute.

## Acknowledgments

The engine's algorithms follow published work: the soft step schedule
and soft constraints, the collision, distance and hull methods, the
mass properties and the particle model. [docs/references.md](docs/references.md)
lists each source. The interactive testbed is built on
[raylib](https://github.com/raysan5/raylib); see
[THIRD_PARTY.md](THIRD_PARTY.md).

## License

MIT. See [LICENSE](LICENSE).
