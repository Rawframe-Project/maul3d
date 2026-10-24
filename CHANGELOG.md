# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [0.1.0](https://github.com/Rawframe-Project/maul3d/compare/v0.0.1...v0.1.0) (2026-10-24)


### ⚠ BREAKING CHANGES

* m3CreateHullShape and m3CreateBoxShape return m3_errorInvalid for coordinates or half extents past 1e4, and m3CreateHullShape for clouds of near-duplicate points whose hull would leave vertices outside its planes (until #32). m3World_Restore returns m3_errorInvalid for hull content whose planes, unit mass properties or vertices the builders would not make.
* m3World_ReplayJournal returns m3_errorCapacity, with the world as it was before the call, when the world's allocator refuses what the replay or one of its ops needs, where it returned m3_errorInvalid or, under memory pressure, left the tape half applied. For the whole call it now holds, besides its snapshot of the world, a second copy of the world's mesh and height field content and mesh trees (about contentBytes of m3World_GetMemoryUsage), every live voxel chunk's surface and arrays sized by the world's capacities, so a replay that succeeded on a tight allocator before can now be refused with m3_errorCapacity before any op runs.
* m3World_Restore returns m3_errorInvalid, with the world unchanged, for hull content the builders would not make and for height field content the grid create would refuse (including stored extremes the samples do not give), where it used to land it.
* m3World_Restore returns m3_errorCapacity with the world unchanged when the allocator refuses, instead of leaving it half restored or with empty mesh and voxel trees, and returns m3_errorInvalid for mesh content a mesh create would refuse. m3CreateMeshShape and m3CreateVoxelChunkShape return m3_errorCapacity when their tree cannot be built, instead of creating a shape that collides with nothing.
* m3World_Hash now covers every id pool's identity (live slots, generations, free-queue order and cursors), a heightfield shape's grid index, a body's gravity scale and dampings, a joint's limit and motor values and its collide-connected flag, and a mesh's painted materials; the setter values fold off their defaults and keyed by slot. Every world's hash changes.
* **world:** m3World_RebuildBroadphase returns m3Result instead of void, and returns m3_errorCapacity, recording nothing, when the world's allocator refuses the rebuild's scratch.
* m3VoxelChunk_ClearBox counts one misuse for a region outside the chunk or inverted on an axis (it still returns -1), and m3World_CastHullClosest counts one misuse for a point that is not finite (it still misses). This makes the behavior match what the footer of #14 already promised, that a refused argument counts misuse in both.
* m3World_Restore and m3World_ReplayJournal refuse a damaged, truncated or short buffer with m3_errorInvalid without counting misuse (a null pointer with a nonzero size still counts), and an empty tape replays as m3_success. Invalid defs in m3CreateBody, m3CreateCharacter, m3CreateJoint, m3CreateVehicle, m3CreateSoftBody, m3CreateSoftBodyTet and m3CreateWaterVolume return m3_errorInvalid and count misuse instead of m3_errorCapacity. Newly refused at creation: m3JointDef.localAnchorA and localAnchorB components beyond 1e4, any non-finite genericLinearLower, genericLinearUpper, genericAngularLower or genericAngularUpper, pulley groundAnchorA and groundAnchorB beyond 1e15 and non-finite rope lengths; m3ShapeDef.localPosition components beyond 1e4 and density above 1e6 (and m3Shape_SetDensity above 1e6, counting misuse); m3SoftBodyDef.position beyond 1e15, spacing above 1e4, pressure above M3_SOFTBODY_MAX_PRESSURE (100), and tet points beyond 1e4. A stale id now counts misuse in m3Character_Move, m3Character_IsGrounded, m3Character_GetGroundBody, m3Character_GetStance, m3SoftBody_GetParticleCount, m3Vehicle_GetGear and m3Vehicle_GetEngineRpm; a refused argument now counts in m3Character_Move, m3Joint_SetLimits, m3World_SetWind, m3World_CastHullClosest and m3VoxelChunk_ClearBox. m3World_ReplayJournal refuses a create that the public call would refuse: a joint def that fails the def contract or the body pair rule, a plane, mesh, terrain or voxel chunk on a body that is not static, a voxel chunk that is a sensor, empty or with a non-finite cell, a mesh with an index past its vertices or a non-finite vertex, and a shape density above 1e6. Every writer called while Step or Draw runs (from a task hook, the pre-solve callback or a debug draw callback) changes nothing and counts one misuse; a status call returns m3_errorState.
* `m3CreateWorld(const m3WorldDef*, m3World**)` returns `m3Result`, and `m3DestroyWorld(m3World*)` takes the pointer. `m3WorldId` and `m3World_IsValid` are removed, along with `m3Body_GetWorld`, `m3Shape_GetWorld` and `m3Joint_GetWorld`. Every call takes the world first; readers take `const m3World*`. Every create and destroy returns `m3Result`, with the id in an out parameter (null on failure). A destroy of a stale id returns `m3_errorStale`, and a destroy of the null id returns `m3_errorInvalid`. `m3World_Snapshot` (bytes written as an out parameter), `m3World_Restore`, the journal calls, the replay container codec (`m3DescribeJournal`, `m3EncodeReplay`, `m3DecodeReplay`) and `m3Character_SetStance` return `m3Result`; a stand-up veto is `m3_errorState`. `m3Shape_SetSphere`, `m3Shape_SetCapsule`, `m3VoxelChunk_SetVoxel`, `m3VoxelChunk_ClearVoxel` and `m3VoxelChunk_SetFill` return `void` and count misuse when they refuse. `m3WorldDef` begins with `internalValue, version`, ends with `allocator`, and loses `workerCount`. `m3DefaultWorldDef` is `static inline` over the new export `m3DefaultWorldDefValues`. `m3SetAllocator`, `m3SetAssertHandler` and `m3LastResult` are removed. `m3GetVersion` returns `m3Version`. Ids are `{uint32_t index1, uint32_t generation}`. `M3_ASSERT` traps in builds without `NDEBUG`, and `m3Counters.allocCalls`/`freeCalls` count the world's own heap. Journal tapes recorded before this change no longer replay: their ids are refused as stale. The replay container is version 2.

### Added

* move to the family API shape ([#13](https://github.com/Rawframe-Project/maul3d/issues/13)) ([cbd3895](https://github.com/Rawframe-Project/maul3d/commit/cbd3895a7891cc1abc624795d41a48822e0874bb))


### Fixed

* count misuse consistently and refuse what restore refuses ([#14](https://github.com/Rawframe-Project/maul3d/issues/14)) ([706b155](https://github.com/Rawframe-Project/maul3d/commit/706b1550464bb8a9f3cff9f7921649be762b3828))
* count the two refusals [#14](https://github.com/Rawframe-Project/maul3d/issues/14) left silent ([#19](https://github.com/Rawframe-Project/maul3d/issues/19)) ([20bbf1f](https://github.com/Rawframe-Project/maul3d/commit/20bbf1fbd5dc431e968b8cb09509efd802da7e8b))
* hash the state the step reads that setters change ([#22](https://github.com/Rawframe-Project/maul3d/issues/22)) ([d4d0037](https://github.com/Rawframe-Project/maul3d/commit/d4d003711604366b73624f9e6f52a7884b1ebe06))
* hold restored hull and height field content to the create walls ([#29](https://github.com/Rawframe-Project/maul3d/issues/29)) ([b8f0008](https://github.com/Rawframe-Project/maul3d/commit/b8f0008b3fdb1ba9949e4ab14fcd7db42f9fd8de))
* hold restored hull planes and mass to what the builders make ([#33](https://github.com/Rawframe-Project/maul3d/issues/33)) ([f73b003](https://github.com/Rawframe-Project/maul3d/commit/f73b003e48466cc6e18b9efbe00c9dded01d3cd7))
* **journal:** normalize body def bools and unmask sanitizer failures ([#3](https://github.com/Rawframe-Project/maul3d/issues/3)) ([58f03f6](https://github.com/Rawframe-Project/maul3d/commit/58f03f60c52331dee1be022e1d158841db8cb66d)), closes [#1](https://github.com/Rawframe-Project/maul3d/issues/1) [#2](https://github.com/Rawframe-Project/maul3d/issues/2)
* **journal:** record mesh materials without an allocation ([#20](https://github.com/Rawframe-Project/maul3d/issues/20)) ([df1c0c5](https://github.com/Rawframe-Project/maul3d/commit/df1c0c5974338a1304a52bd2024241d60f56d4b3))
* refuse a restore or create whose trees cannot be built ([#27](https://github.com/Rawframe-Project/maul3d/issues/27)) ([67f4859](https://github.com/Rawframe-Project/maul3d/commit/67f4859bf9b23d180494cc3c5e6050d58c32a30e))
* report a replay refused memory as capacity and back out without it ([#30](https://github.com/Rawframe-Project/maul3d/issues/30)) ([df795d1](https://github.com/Rawframe-Project/maul3d/commit/df795d1dcdf8c329153041770a1e6b3788f822ac))
* **snapshot:** refuse a voxel chunk whose count disagrees with its voxels ([#7](https://github.com/Rawframe-Project/maul3d/issues/7)) ([18ead18](https://github.com/Rawframe-Project/maul3d/commit/18ead181c4a5baacc16dcd298aed8ed3c0eaa81b)), closes [#6](https://github.com/Rawframe-Project/maul3d/issues/6)
* **world:** apply an explosion to the shapes in slot order ([#28](https://github.com/Rawframe-Project/maul3d/issues/28)) ([3659d1f](https://github.com/Rawframe-Project/maul3d/commit/3659d1f43f11368bd3a1a685086364b4a3222185))
* **world:** take the broadphase rebuild's scratch before recording it ([#23](https://github.com/Rawframe-Project/maul3d/issues/23)) ([1e36eae](https://github.com/Rawframe-Project/maul3d/commit/1e36eae212b48a8dfe4b660b0c9d2471bd9f0a23))

## [Unreleased]

### Changed

- The library is built as C23 and needs GCC 14 or Clang 19 or newer; on
  Windows it is built with clang-cl, which Visual Studio installs, since
  MSVC's cl cannot compile C23. The public headers stay C17 and C++17,
  and MSVC's cl still compiles them. A shared library's SOVERSION is
  now the whole version, and the CMake package accepts only its exact
  version, because before 1.0 any release may change the ABI.
- The README is rewritten. A new section introduces the Maul family:
  what each engine is built for, and the contract, vocabulary, rules
  and tools the two share; both libraries link into one program.
  THIRD_PARTY.md is gone: the library uses no outside code, and the
  testbed's raylib is credited in the README.

### Fixed

- The Godot kit's README claimed the kit was compile-checked against
  the headers; it says what CI actually checks.
- Two joints that can never both hold no longer risk turning the world
  to NaN through impulses that fight without limit: every joint row
  now holds its accumulated impulse within a bound no ordinary scene
  reaches, so the results of existing scenes are unchanged. Maul2D's
  weekly deep fuzz found the overflow; the joint solvers share the
  design.
- Replaying a damaged journal tape no longer hands the engine a body
  def whose motion locks, `enableSleep`, `isEnabled` or
  `enableFastRotation` hold a byte other than 0 or 1, which was
  undefined behavior. Any nonzero byte now reads as true, as it already
  did for `isBullet` and the shape, joint and vehicle flags.
- A damaged snapshot whose voxel chunk claims a different number of
  filled voxels than its occupancy bits hold is refused. Before, the
  restore accepted it, and fracture and fragment mass read the wrong
  count.

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
