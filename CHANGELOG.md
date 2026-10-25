# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [0.3.0](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.3.0) (2026-10-25)

### Changed

- **Breaking: a character owns its body and capsule.** Queries report a
  character's capsule like any shape, and `m3Shape_GetBody` gives its
  body, so a program could destroy, retype or reshape either one, or add
  a shape to the body, while the character lived on, which the character
  does not support. `m3DestroyShape`, `m3DestroyBody`, `m3Body_SetType`,
  `m3Shape_SetSphere`, `m3Shape_SetCapsule` and a shape create on the
  character's body now refuse them with `m3_errorInvalid`, count a
  misuse and change nothing; replay refuses the same ops in a tape.
  Destroy the character instead: that still takes its body and capsule
  with it, and `m3Character_SetStance` remains the way to resize it.
  Replay also refuses a body type the public call refuses (outside
  static, kinematic and dynamic); it used to set any byte.
  ([#73](https://github.com/Rawframe-Project/maul3d/pull/73))

### Fixed

- **Restore refuses a snapshot whose structures this engine could not
  have written.** It checked each snapshot block on its own
  (indices in range, flags, finite floats) but not the structures the
  blocks form together. It now also checks, before any byte lands: the
  id pools, the body to shape and joint lists, the broadphase tree, the
  contact pairs, shared content and its reference counts, and the
  records of characters, vehicles, soft bodies and joints. Such a
  snapshot is refused with `m3_errorInvalid` like every other restore
  refusal, and the world is left as it was. Nothing is rebuilt, so
  restored worlds still resimulate bit for bit, and every snapshot this
  engine writes still restores, also one taken between a call and the
  next step. The snapshot format is unchanged.
  ([#70](https://github.com/Rawframe-Project/maul3d/pull/70))
- **`m3World_TestPoint` reports a point far from a voxel chunk as
  outside it.** The cell of a point's chunk-local coordinates is now
  bounded like every other grid-cell cast, and the library's other
  float-to-int conversions were checked the same way. No result changes
  for any value in range, and no golden or benchmark hash moved.
  ([#71](https://github.com/Rawframe-Project/maul3d/pull/71))

## [0.2.1](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.2.1) (2026-10-24)

### Changed

- **The soft body pass skips the shapes no particle can reach.** Each
  substep it tested every particle of every soft body against every
  shape in the world. It now first tests the box around a soft body's
  particles against each shape: a bounding sphere for spheres, capsules,
  voxel chunks, meshes and height fields, and a separating face for
  hulls. Planes are always tested. Each test keeps a stated rounding
  margin, so a skipped pair is one the projection would leave alone. A
  bench scene of four cloths and two jellies among 2000 static shapes 40
  to 90 m away runs on 3.57 billion instructions instead of 282.1
  billion, 79 times fewer. The skip changes no result: hashes,
  snapshots and replays are the same with it as without it, and there is
  no API change. ([#47](https://github.com/Rawframe-Project/maul3d/pull/47))
- **The API reference lives on the documentation site.** The README and
  the Godot binding's README link
  https://rawframe-project.github.io/maul3d/api.html, which is built from
  the headers. The committed `docs/api.md`, which showed an older API, is
  gone. `m3World_Hash`'s doc now lists what the hash covers and what it
  leaves out: mesh geometry and height field samples are not hashed.
  `m3Counters.voxelEditsRefused` is documented as a counter that is
  neither hashed nor snapshotted.
  ([#46](https://github.com/Rawframe-Project/maul3d/pull/46))

### Fixed

- **Soft body particles collide with thin mesh triangles and with
  triangles that repeat a corner; they used to pass through.** On a
  triangle well under a millimetre wide, the closest point could come
  out up to a metre from a particle a few centimetres away, so the
  particle was not pushed. A triangle whose first two corners are one
  vertex gave no closest point at all. On other triangles a result
  changes only where the point lies within rounding of an edge, and no
  golden or bench result moved.
  ([#48](https://github.com/Rawframe-Project/maul3d/pull/48))

## [0.2.0](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.2.0) (2026-10-24)

### Added

- **32-bit x86 builds.** The `linux-i686-gcc-14` and
  `linux-i686-clang-20` presets build for i686 with
  `-m32 -msse2 -mfpmath=sse`, and CI runs both. A host's own i686 build
  needs those flags too: the library refuses to build with x87 float
  arithmetic. `base.h` adds `M3_ALIGN8`, which the public headers use to
  give 64-bit members 8-byte alignment.
  ([#41](https://github.com/Rawframe-Project/maul3d/pull/41),
  [#44](https://github.com/Rawframe-Project/maul3d/pull/44))

### Changed

- **Breaking, on 32-bit x86 only: the public ABI.** i686 hosts must
  rebuild against the new headers. The doubles and 64-bit integers of
  `m3Pos3`, `m3QueryFilter` and the body, character, soft body, vehicle
  and shape defs are now 8-byte aligned there, as on every other target.
  These public structs change size on i686:
  - `m3BodyDef`, 108 to 120 bytes
  - `m3CharacterDef`, 68 to 72
  - `m3ShapeDef`, 92 to 104
  - `m3VehicleDef`, 460 to 464
  - `m3WaterVolumeDef`, 76 to 80
  - `m3JointDef`, 216 to 224
  - `m3RayCastResult`, 52 to 56
  - `m3BodyMoveEvent`, 52 to 56
  - `m3FragmentEvent`, 68 to 72

  Every other public struct that holds an aligned member keeps its size
  and changes only its alignment, from 4 to 8. An i686 tape recorded
  with 0.1.x is refused with `m3_errorInvalid` when it holds one of the
  twelve journal records whose size changed: a body, character, vehicle,
  water volume or joint create, any shape create, a filter change, or a
  force or impulse at a point. Other i686 tapes, and every i686
  snapshot, still load. On x86-64, arm64, Windows and wasm nothing
  changes but tape padding (below): existing tapes and snapshots load as
  before.
  ([#44](https://github.com/Rawframe-Project/maul3d/pull/44))

### Fixed

- **Journal tapes have one layout on every target.** Before, a tape's
  records were laid out as the recording target laid out its structs,
  so a tape recorded on i686 was refused on x86-64 and the other way
  round. Every journal record and every type a snapshot walks now has
  its x86-64 layout on every target, and the build checks each size and
  member offset, so a target that would lay one out otherwise fails to
  compile. The formats are little-endian, and a big-endian target fails
  to build.
  ([#44](https://github.com/Rawframe-Project/maul3d/pull/44))
- **A tape's padding bytes are always zero, on every target.** A create
  copied the caller's def into its record whole, so the tape's padding
  bytes held whatever the host left in the def's padding. Records now
  copy each def member by member into zeroed storage. Replay never read
  that padding, so worlds are unchanged, but the bytes of a tape can
  change for a host that left anything there, and the same calls now
  write the same tape.
  ([#44](https://github.com/Rawframe-Project/maul3d/pull/44))
- **The misuse and busy counters are atomic on 32-bit x86.** Their
  64-bit atomics were 4-byte aligned on i686: clang could not make them
  lock-free and the build did not link, and under GCC they were not
  guaranteed atomic. Both counters are now 8-byte aligned on every
  target.
  ([#41](https://github.com/Rawframe-Project/maul3d/pull/41))
- **A 32-bit build no longer gets AVX2.** The AVX2 backend was chosen
  from the host's processor name, so an `-m32` build on an x86-64 host
  was built for AVX2. It is now chosen only for 64-bit builds.
  ([#41](https://github.com/Rawframe-Project/maul3d/pull/41))

## [0.1.1](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.1.1) (2026-10-24)

### Added

- **`m3Counters.voxelEditsRefused`.** A chunk's surface holds at most
  2048 merged boxes, the checkerboard's count. An edit whose surface
  would pass that bound changes nothing, journals nothing and counts no
  misuse. It counts here instead, and `m3VoxelChunk_ClearBox` returns -1.
  An explosion skips that chunk's carve and still applies its impulses.
  A chunk create past the bound is refused with `m3_errorCapacity`, a
  snapshot chunk past it with `m3_errorInvalid`, and a replayed edit past
  it refuses the replay, which leaves the world as it was. No grid
  measured so far reaches the bound. The field is a counter, not
  state: it is not hashed, and a restore or a replay's back-out leaves
  it as it is. A program that wants to know whether an edit landed can
  compare it before and after.
  ([#38](https://github.com/Rawframe-Project/maul3d/pull/38))

### Changed

- **Voxel memory is reserved per chunk slot.** A voxel slot takes about
  84 KB for its collision tree when its first chunk is created, and
  keeps it for later chunks in that slot. Every world also takes about
  88 KB at creation: a 76 KB build scratch and a 12.5 KB edit staging
  copy. `m3CreateVoxelChunkShape` returns `m3_errorCapacity` when the
  allocator refuses the slot's tree, with the world as it was. A restore
  takes the tree for each live chunk slot that has none yet, and returns
  `m3_errorCapacity` when refused. `m3MemoryUsage.contentBytes` counts
  the slot trees. A replay's up-front headroom no longer counts voxel
  surfaces, only arrays sized by the mesh, shape and voxel capacities.
  Budget 84 KB per voxel slot a program fills, on top of the
  world's 88 KB.
  ([#38](https://github.com/Rawframe-Project/maul3d/pull/38))
- A mesh shape's create takes a build scratch sized by its triangle
  count rather than for 65535 triangles, so it needs less temporary
  memory. Mesh trees and every hash stay the same.
  ([#38](https://github.com/Rawframe-Project/maul3d/pull/38))

### Fixed

- **The same bits in any host's floating-point setting.** Every public
  function that computes now runs in the library's own setting (round to
  nearest, subnormals kept, every exception masked) whatever setting the
  calling thread has, and gives the thread its setting back on return,
  sticky exception flags included on x86. Before, a host that ran its
  threads with flush-to-zero, denormals-are-zero or another rounding
  mode got different results from the same calls. Host code runs in the
  host's setting: the allocator, the task hooks and the debug draw
  callbacks. The pre-solve callback runs in the library's setting,
  since its answer decides which contacts the solver keeps. Each range
  a task hook runs enters the library's setting on whatever thread the
  host gives it. A program changes nothing.
  ([#35](https://github.com/Rawframe-Project/maul3d/pull/35))
- **Voxel edits no longer fail for memory.** A set, a clear, a box
  clear, an explosion's carve or the fracture a clear causes used to
  rebuild the chunk's collision tree with memory from the allocator.
  When the allocator refused, the edit still landed and was journaled,
  but the chunk stopped colliding and answering queries until a later
  edit or restore. A replay with memory then diverged from the
  recording. Edits now rebuild in storage reserved ahead of them and ask
  the allocator for nothing.
  ([#38](https://github.com/Rawframe-Project/maul3d/pull/38))

## [0.1.0](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.1.0) (2026-10-24)

### Changed

- **Breaking: the family API shape.** `m3CreateWorld` returns
  `m3Result` and hands the world back through an out parameter, and
  every call takes the world first; readers take `const m3World*`.
  `m3WorldId`, `m3World_IsValid`, `m3Body_GetWorld`, `m3Shape_GetWorld`,
  `m3Joint_GetWorld`, `m3SetAllocator`, `m3SetAssertHandler` and
  `m3LastResult` are removed; the allocator is part of `m3WorldDef`,
  which begins with `internalValue, version` and loses `workerCount`.
  Every create and destroy returns `m3Result`, with the id in an out
  parameter (null on failure). A destroy of a stale id returns
  `m3_errorStale`, and a destroy of the null id returns `m3_errorInvalid`.
  Ids are `{index1, generation}`. Snapshots, restores, the journal calls
  and the replay container codec return `m3Result`, and
  `m3World_RebuildBroadphase` does too. Journal tapes recorded with
  0.0.1 no longer replay, and the replay container is version 2.
  ([#13](https://github.com/Rawframe-Project/maul3d/pull/13))
- **Breaking: hulls.** Hulls are built from welded points (points
  within 512 QuickHull tolerances of an earlier point, about 1e-3 of the
  summed extents) with planes worked in double precision, so a cloud of
  near-duplicate points now gives a valid hull. Hulls built from the
  same points can differ from 0.0.1's, so the hashes and trajectories of
  scenes with hulls change. Snapshots are format 2: `m3World_Restore`
  refuses format 1, and replay files that hold one, with
  `m3_errorInvalid`.
  ([#34](https://github.com/Rawframe-Project/maul3d/pull/34))
- **Breaking: the world hash** now covers every id pool's identity, a
  height field shape's grid, a body's gravity scale and dampings, a
  joint's limits, motor and collide-connected flag, and a mesh's painted
  materials, so state the step reads can no longer differ under equal
  hashes. Every world's hash changes.
  ([#22](https://github.com/Rawframe-Project/maul3d/pull/22))
- **Breaking: refusals.** Arguments the engine cannot simulate are now
  refused at creation with `m3_errorInvalid`, and they count as misuse.
  This covers coordinates, half extents and local positions beyond 1e4,
  densities above 1e6, non-finite joint limits and rope lengths, soft
  body pressure above `M3_SOFTBODY_MAX_PRESSURE`, and the other bounds
  the headers list. A stale id or a refused argument counts as misuse in
  every call. A writer called while `Step` or `Draw` runs changes
  nothing and counts as misuse.
  ([#14](https://github.com/Rawframe-Project/maul3d/pull/14),
  [#19](https://github.com/Rawframe-Project/maul3d/pull/19))
- **Breaking: what a restore or replay accepts.** `m3World_Restore`
  leaves the world unchanged and returns `m3_errorInvalid` for content
  the builders would not make (hull planes, mass and vertices; height
  field and mesh content), and `m3_errorCapacity` when the allocator
  refuses. Before, it could land such content or leave the world half
  restored. `m3World_ReplayJournal` refuses an op the public call would
  refuse. It returns `m3_errorCapacity` with the world as it was when
  memory runs out. To do that it reserves its memory up front, so a
  replay that fit a tight allocator before can now be refused before any
  op runs.
  ([#27](https://github.com/Rawframe-Project/maul3d/pull/27),
  [#29](https://github.com/Rawframe-Project/maul3d/pull/29),
  [#30](https://github.com/Rawframe-Project/maul3d/pull/30),
  [#33](https://github.com/Rawframe-Project/maul3d/pull/33))
- The library is built as C23 and needs GCC 14 or Clang 19 or newer.
  On Windows it is built with clang-cl. The public headers stay C17 and
  C++17, and MSVC's cl still compiles them. A shared library's
  SOVERSION is the whole version, and the CMake package accepts only its
  exact version, since before 1.0 any release may change the ABI.

### Fixed

- A mesh or voxel chunk whose tree cannot be built is refused with
  `m3_errorCapacity`, where before it was created and collided with
  nothing.
- An explosion applies to shapes in slot order, so the result no
  longer depends on the broadphase's internal order.
- The broadphase rebuild takes its scratch memory before recording
  itself, and records nothing when that fails.
- Two joints that can never both hold no longer risk turning the world
  to NaN. Every joint row holds its accumulated impulse within a bound
  no ordinary scene reaches.
- Replaying a damaged journal no longer reads a body def flag holding a
  byte other than 0 or 1, which was undefined behavior. A mesh's
  materials are recorded without an allocation.
- A damaged snapshot whose voxel chunk count disagrees with its
  occupancy bits is refused.

## [0.0.1] - 2026-09-24

The first release of the reworked library.

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
- The library target requires C11 of its consumers through CMake, and
  a test compiles the umbrella header as C++ so the API stays usable
  from C++.
- `test/hashes.txt` pins soft body, soft-to-soft, vehicle, drivetrain,
  water, character and voxel scenes, so every subsystem the world hash
  covers has a golden value.
- M3_JOINT_HASH: a golden hash over every joint kind with its limits,
  motors, springs and steering on.
- m3GetSimdBackend and m3CpuSupportsBackend report the SIMD backend
  the library was built for and whether this CPU runs it. x64 builds
  use AVX2 and FMA; -DMAUL3D_SIMD=scalar builds a portable library
  with identical results. A world is refused (m3_errorConfig) on a CPU
  that cannot run the backend.
- m3Shape_SetFilter and m3Shape_GetFilter change and read a shape's
  collision filter at runtime, as in Maul2D. The change is journaled,
  wakes the shape's body and everything it touches, and every pair is
  filtered again on the next step.
- The readers and setters Maul2D offers: m3Body_GetWorld,
  GetTransform, GetMass, GetRotationalInertia, GetLocalCenter,
  GetWorldCenterOfMass, GetWorldPoint, GetLocalPoint, GetWorldVector,
  GetLocalVector, GetWorldPointVelocity, GetLocalPointVelocity,
  GetGravityScale, GetLinearDamping, GetAngularDamping, IsBullet,
  ComputeAabb, GetShapes and GetJoints; m3Shape_GetWorld, GetType,
  GetUserData, IsSensor and GetAabb; m3Joint_GetWorld, GetType,
  GetBodyA and GetBodyB; m3World_GetBodies, GetJoints and
  GetStepCount; and the journaled m3Body_SetGravityScale,
  SetLinearDamping, SetAngularDamping, SetBullet, SetUserData and
  m3Shape_SetUserData. m3AabbResult holds world bounds.
- tools/check_bindings.py checks the C# and Godot bindings against the
  C API in CI: every imported or called function exists with that many
  parameters, and every C# struct has the size and field offsets of
  the C struct it mirrors. The C# layout check covers every struct.

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
- Every world array is described once in a state table
  (src/world_state.c) that drives allocation, release, the snapshot
  walk and the memory footprint, as in Maul2D; journal replay
  dispatches through a command table of per-op apply functions
  (src/journal_replay.c) instead of a 1,660-line switch. world.c
  shrank from 3,300 to 1,170 lines. Snapshot bytes and hashes are
  unchanged.
- Each joint type's solver rows live in src/joint_<kind>.c behind a
  kind table, as in Maul2D: the 345-line prepare, 170-line warm start
  and 1,500-line solve if-chains in solver.c became per-kind functions
  plus shared pieces (the hinge frame, the angular-lock warm start,
  the point constraint). Joint flag bits are named M3_JOINT_*.
  solver.c shrank from 4,350 to 2,200 lines. Results are bit-
  identical.
- The narrow phase picks each pair's collider from a [typeA][typeB]
  table in the new src/narrowphase.c instead of a 560-line if-chain,
  and the triangle-soup kernels (mesh, heightfield and voxel surfaces)
  moved to src/manifold_mesh.c; manifold.c keeps the convex kernels.
  Results are bit-identical.
- The contact solver, continuous collision and islands moved out of
  solver.c into contact_solver.c, continuous.c and island.c, each with
  its header; solver.c keeps the step. Results are bit-identical.
- m3World groups its 287 fields into per-subsystem blocks (bodies,
  shapes, hulls, meshes, heightfields, voxels, broadphase, contacts,
  joints, characters, vehicles, soft bodies, water, events, recorder),
  as in Maul2D, and world_internal.h keeps the layout: the internal
  declarations moved to one header per module (body.h, shape.h,
  joint.h, voxel.h, distance.h, world.h and others). Results are bit-
  identical.
- Journal payloads are declared once in src/journal.h and shared by
  the recorders and replay (they were declared twice, as anonymous
  structs on each side). The wire format is unchanged.
- The step reads as its stages: m3StepInternal (657 lines) hands its
  passes (contact events, scratch sizing, sweep capture, movers,
  buoyancy, velocity and position integration, joint breaks) to named
  functions over an m3StepScratch. A length rule now holds functions
  to 80 lines and source files to 1000: tools/check_lengths.py runs in
  CI, and the existing exceptions sit in tools/length-exceptions.txt
  with their reasons and a ceiling that may only shrink. Results are
  bit-identical.
- Comments no longer carry development-history markers (slice codes,
  revision stories, format versions, the 40-line changelog on
  M3_SOLVER_REV); tools/check_comments.py enforces the conventions.md
  rule in CI next to the length rules.
- m3ComputeCosSin and m3Atan2 are the engine's own: sine and cosine
  from a two-part pi/2 reduction and Taylor series, atan2 from an
  argument-halving identity and a Taylor series, accurate to about
  4e-7 instead of the previous 2e-3. docs/references.md lists the
  published sources behind the algorithms.
- The snapshot header holds only the magic, the format
  version, the build hash and the eight world capacities. Step count,
  gravity, the pair count, the tree cursors and every pool cursor are
  state table rows, checked before any byte lands. A vehicle or soft
  body capacity mismatch is now `m3_errorConfig`.
- The broadphase tree is rewritten: surface-area-guided insertion,
  strict AVL balance, moves that keep the node id, and a rebuild that
  leaves the old tree intact when it cannot allocate. Every golden
  hash and bench pin is unchanged.
- GJK distance, time of impact, QuickHull and the hull-versus-hull
  manifold are rewritten from published sources (docs/references.md).
  The GJK cache that no caller kept is gone from the API.
- The convex-versus-triangle kernels are rewritten: nearest point on a
  triangle by barycentric regions, capsules by direct segment-triangle
  closest points, hulls by a separating axis test whose edge pairs use
  the triangle's half-circle Gauss arcs. The mesh welding core is
  split into named steps with unchanged behavior; the kernels move to
  triangle_contact.c.
- Continuous collision is restructured into a filter and one sweep per
  target kind (convex, mesh, voxel chunk, plane).
- Contact solver reorganized: preparation (pair filter and pre-solve
  veto, surface mixing, point rows, center rows) lives in
  contact_prepare.c; the rows (normal, twist, rolling, friction) work
  on the pair's velocities loaded once per constraint and stored once,
  and one stage runner covers warm start, solve, relax, restitution
  and store. Results are bit for bit unchanged.
- Joint solver rewritten on rows: every kind builds its scalar rows (a
  Jacobian and a drive: rigid, held, spring or limit) from the pose
  the substep reached, and one row solver, a coupled pair, a point
  block and a rotation block serve all eleven kinds. Frames, slide
  axes and hinge axes now turn with the bodies within a step. The
  spherical twist row uses the exact gradient of the twist angle,
  derived from the relative rotation's rate; drive targets use the
  exact rotation error. The five stored impulse slots load and store
  whole, each kind documenting its slot map.
- The step's pieces moved to where they belong: integration (movers,
  velocities with the implicit gyroscopic step, poses) in integrate.c,
  the buoyancy field in water.c, the contact and move events in
  world_events.c; solver.c keeps the step's order. Results are bit for
  bit unchanged.
- Mover kit redesigned to match maul2d: m3SolveMover returns the
  translation closest to the wish that no plane blocks, found exactly
  (it rests on at most three planes), with a bitmask of the planes it
  rests on; m3ClipMoverVelocity strips the velocity pointing into
  those planes. m3SolvePlanes and its iteration count are gone;
  m3MoverPlane's shape field is now shapeId.
- THIRD_PARTY.md now lists only the testbed's raylib: the solver,
  joints, collision, ray casts and trigonometry once adapted from
  Box2D and Box3D have been replaced by the engine's own code, and a
  token-level similarity scan against those projects finds nothing
  beyond trivial shared idioms. The README credits the published work
  in docs/references.md; comments and docs that measured the engine
  against another engine now state its own rules.
- Planes are kept in a derived list, rebuilt when a plane is created
  or destroyed and after a restore. Continuous collision, shape casts,
  ray casts, overlaps, the mover and the broadphase walk that list
  instead of scanning every shape for planes: the 10k-body smoke bench
  steps 59% faster, the city block 18%. Results are bit for bit
  unchanged.
- Mesh edge flags bake from per-vertex triangle lists instead of a
  pass over every triangle for every edge: large meshes create in a
  fraction of the time (the bench suite, setup included, runs 25%
  faster), and every height field collision builds its window's flags
  without the quadratic scan. Results are bit for bit unchanged.
- Tree nodes carry the kind bits of the leaves below them (the body
  type, and whether the shape is a static surface). Continuous
  collision for ordinary fast bodies only meets statics and static
  surfaces, so its sweeps skip every subtree without one: the 10k-body
  smoke bench steps in 22 ms (30 before), the city block in 16.8 ms
  (18.3). Results are bit for bit unchanged.
- m3Hash64 hashes eight bytes a round (xor, multiply by an odd
  constant, fold the high half) and the leftover bytes one at a time,
  the same function as maul2d's: hashing a 5000-body world takes 0.74
  ms instead of 2.5. Every hash value changes; the golden hashes and
  bench pins are re-pinned, and gcc, clang and Debug agree.
- The broadphase keeps the overlapping tree leaves between steps: only
  shapes whose leaf moved query the tree, moved leaves are reinserted
  0.1 m beyond their bounds, and static shapes no longer query. Pair
  lists and hashes are unchanged; the city block steps in 13.6 ms
  (16.8 before).
- Contacts of each graph color are solved eight at a time by a SIMD
  kernel (AVX2, NEON, or eight-lane scalar code). The kernel does the
  scalar rows' operations in the same order, so results are bit for
  bit unchanged, and a test holds the kernel to the scalar rows every
  step. The pyramid bench steps in 0.13 ms (0.22 before), the city
  block in 11.0 ms (13.6).
- Public names follow the family rules and match Maul2D where the
  concept is shared: m3World_StartJournal, m3World_StopJournal and
  m3World_ReplayJournal (were JournalBegin, JournalEnd,
  JournalReplay), m3DescribeJournal, m3EncodeReplay, m3DecodeReplay,
  m3GetEncodedReplaySize, m3World_GetMemoryUsage, m3World_TestPoint
  (was PointInside), m3World_Compare (was DiffReport),
  m3Shape_IsHitEventsEnabled, m3Body_EnableFastRotation and
  m3Body_IsFastRotationEnabled. m3Body_SetEnabled is now m3Body_Enable
  and m3Body_Disable, and m3Body_SetSleepControls is now
  m3Body_EnableSleep, m3Body_IsSleepEnabled, m3Body_SetSleepThreshold
  and m3Body_GetSleepThreshold.
- Queries take Maul2D's form: every ray, cast and overlap takes an
  m3QueryFilter as its last argument (m3DefaultQueryFilter() sees
  everything), and the separate Ex variants are gone. Closest casts
  return an m3RayCastResult and m3World_CastRayAll fills m3RayHit
  entries without the hit flag, as in Maul2D; both name the shape
  field shapeId.
- Events take Maul2D's form: m3World_GetContactEvents returns begin,
  end and hit streams with their counts and the hit drop count
  (m3ContactBeginEvent, m3ContactEndEvent, m3ContactHitEvent), and
  m3World_GetSensorEvents, m3World_GetFragmentEvents (events, recipe
  buffer and drop count), m3World_GetBodyEvents and
  m3World_GetJointEvents do the same for their streams. They replace
  eleven pointer-and-count functions. Struct fields that hold ids end
  in Id: shapeIdA and shapeIdB in events and m3ContactData, bodyIdA
  and bodyIdB in m3JointDef, bodyId in m3BodyMoveEvent and m3BodyDiff,
  jointId, chunkShapeId and chassisId.
- The allocator hook's size argument is a size_t, as in Maul2D.
- Object ids carry their world's generation: the world field (was
  world0) holds the world slot in its low M3_WORLD_SLOT_BITS bits and
  the low bits of the world generation above them, so a world
  recycling a slot refuses the ids of the world before it. m3WorldId
  is four bytes, {uint16_t index1, uint16_t generation}, as in Maul2D.
- Debug draw is one m3DebugDraw and one m3World_Draw, as in Maul2D:
  the triangle callback and the drawSolidShapes, drawIslands,
  drawMassAxes and drawTreeBoxes flags joined the struct, and
  m3SolidDraw, m3ExtraDraw, m3World_DrawSolid and m3World_DrawExtras
  are gone. They were split only to keep an old ABI. Callbacks are
  camelCase fields (drawSegment, drawPoint, drawTriangle).
- Defs carry every setting a setter changes, as Maul2D's do: m3BodyDef
  gains motionLocks, enableSleep, sleepThreshold, isEnabled and
  enableFastRotation, m3ShapeDef gains surfaceVelocity, and m3WorldDef
  gains maximumAngularSpeed; its enableSleeping and enableContinuous
  are bools. Motion locks are an m3MotionLocks struct of six flags
  (like m2MotionLocks) instead of a bit mask. The C# binding's defs
  follow.
- m3CreateJoint takes the world id first, like every other create
  function; the joint's bodies must live in that world.
- Shared code follows ADR 0004: src/hash.c holds the hash in both
  engines, src/simd.h and src/cpu.c are the same file in both once the
  prefix is renamed, and tools/check_family.py checks the shared files
  against the sibling repository (byte for byte, or after the rename
  for tools/family-renamed.txt) in the weekly CI job.

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
- The inline math helpers in `core_math.h` returned compound literals,
  which C++ does not have, so MSVC rejected the public headers in C++
  code. They now build their results in named variables, valid in both
  languages.
- MSVC on arm64 ignores the `/fp:contract-` switch, so floating-point
  contraction was not reliably off there. Every source now turns it
  off with `#pragma fp_contract(off)` under MSVC, and the switch is
  only passed where the compiler knows it.
- Reads and writes through a stale body, shape, joint or voxel-chunk
  id now record m3_errorInvalid and count as misuse against the id's
  world, as Maul2D does; shape creation and m3CreateJoint record a
  reason on every refusal path (several returned null silently,
  including a plane on a dynamic body); a valid generic joint no
  longer leaves m3_errorInvalid behind.
- The rolling-resistance row wrote the angular velocity of static
  bodies, which are shared across graph colors, so a host running
  colors in parallel had a data race (ThreadSanitizer, test_scale). It
  now writes dynamic bodies only, like every other contact write-back;
  results are unchanged.
- Ray, shape and overlap queries refuse non-finite origins, bases,
  points, radii, centers and boxes (and inverted boxes) with a
  recorded reason, and a NULL world is refused instead of returning a
  silent miss; m3World_PointInside refuses a non-finite point;
  m3World_Step refuses an infinite dt; m3Character_SetStance records a
  reason for hostile dimensions (a veto for lack of headroom still is
  not misuse).
- World creation that runs out of memory, a replay that fails part
  way, and a replay whose safety snapshot cannot be taken now record a
  reason (m3_errorCapacity or m3_errorInvalid) instead of returning
  silently.
- m3World_Restore pre-validated the header and the variable tail but
  copied the fixed blocks in unchecked: a corrupted snapshot could
  carry out-of-range body, shape, joint, soft-body particle or tree
  indices, invalid flags or non-finite state into the world. Every
  fixed block is now checked against its state-table entry before any
  byte lands. A snapshot from another build or world shape now refuses
  with m3_errorConfig, and the tail checks record a reason instead of
  returning silently. The world hash moved to src/world_hash.c.
- The character's slope cosine, vehicle steering and the soft-body
  wind gust called libm's cosf and sinf, which are not required to
  agree bit for bit across platforms; they now use m3ComputeCosSin.
- Restore now brings back the slot cursors of the height field, water,
  vehicle and soft body pools. They were missing from snapshots, so a
  soft body destroyed after a snapshot came back on restore, and the
  next create reused its slot and overwrote it.
- Body extents for continuous collision are measured in each shape's
  own frame. A body built from offset shapes, such as a dumbbell of
  two spheres, reported the extent of one shape about its own center,
  so the rotation arc bound was far too small.
- The hull, mesh, height field and voxel constructors check the def's
  materials and compound pose first. A NaN density or a negative
  friction was refused as `m3_errorCapacity` instead of
  `m3_errorInvalid`.
- A shape create that fails after taking a content slot releases it
  without freeing the caller's staged mesh or height field arrays. The
  old rollback freed the height field samples the caller then freed
  again, and left mesh and voxel slots allocated.
- A create refused after taking a slot leaves every id pool as it was.
  A mesh with no free mesh slot, a tetrahedral soft body past its edge
  budget and a character with no free body or shape took a slot and
  freed it, bumping its generation. Refusals are not journaled, so a
  replay of the same session minted different ids for later creates
  and failed.
- A tree query deeper than its 64-entry stack silently skipped
  subtrees; the tree is now strictly balanced, so the stack always
  suffices. A broadphase rebuild that ran out of memory emptied the
  tree while proxies still pointed into it.
- Contact points on large patches (a cylinder cap on a plane, a hull
  face on a mesh) kept the four deepest points, which could all sit on
  one side; the drum stack sank 8 cm and wandered. The four points now
  spread over the patch and the stack rests exactly.
- GJK reported a tiny positive distance and a noise normal when the
  origin fell exactly on a simplex face of deeply overlapping hulls;
  it now reports the overlap.
- The fast-body test and the plane sweep bounded rotation by the chord
  a point travels, which is shorter than its arc; a fast spinning body
  could pass the test or overshoot a plane. Both now use the arc bound
  of the swept rotation.
- The prismatic joint's rows across the slide ran along half-length
  axes, so the reported constraint force for them was twice the true
  value; they now run along the unit frame axes.
- m3ComputeCosSin's documentation described a Bhaskara rational form
  and a minimax atan2 that the engine no longer uses; it now describes
  the quarter-turn reduction and the Taylor series.
- An id naming no live world records m3_errorInvalid on every path;
  some returned silently.
