# Overview

Maul3D simulates rigid bodies, joints, characters, vehicles, soft bodies, voxel destruction and water in a world that snapshots, restores and replays to the bit on every platform it supports. It follows the family rules in [maul-conventions.md](maul-conventions.md); this page states what the library's domain adds and how the parts fit.

## Library profile

- **Determinism:** bit-exact on every platform, compiler, build type, SIMD backend and worker count, whatever floating-point setting the host's threads run in. It covers the simulation state, snapshots, journal tapes, replay containers, `m3World_Hash`, the event streams, the counters and every reader. Only `m3World_GetProfile`'s wall-clock times are exempt. [determinism.md](determinism.md) explains how the tests hold it.
- **Threads:** the library starts none. The step hands its parallel work to the host's task system through the world def's `enqueueTask` and `finishTask`; without them the work runs on the caller's thread.
- **Owner objects:** the world, `m3World*`, created by `m3CreateWorld` from a root def that carries `M3_ABI_VERSION`. Bodies, shapes, joints, characters, vehicles, soft bodies and water volumes are objects of a world, named by typed ids.
- **Memory:** a world takes its pools and its step scratch from the allocator in its def when it is created, so a step never allocates. A shape create takes its broadphase entry and any content it holds (a hull, a mesh, a height field, a voxel chunk and its surface), and a destroy gives them back. `m3World_Restore` takes the scratch its checks need and the content it brings in, `m3World_ReplayJournal` the copy it backs out to, and `m3World_RebuildBroadphase` its scratch; each frees what the call alone used before it returns. The functions without a world (`m3SolveMover`, `m3ClipMoverVelocity`, the replay codec in `replay.h`) never allocate.
- **Data formats:** three state formats, the snapshot, the journal tape and the replay container, and no asset format. None is read across 1.y: a release of another minor version refuses one by its `M3_ABI_VERSION` with `m3_errorVersion`, and a release that changed what a step computes refuses an earlier snapshot by its solver revision. A host keeps its own game state, not snapshots, tapes or containers, across an upgrade. [state-formats.md](state-formats.md) has the layouts.
- **Platform dependencies:** none beyond the C standard library; the `<math.h>` functions it may call are the correctly rounded ones in `tools/libm-allowed.txt`. The testbed, which is not part of the library, uses raylib.

## The public API

The API is the twelve headers in `include/maul3d/`: `base.h`, `core_math.h`, `body.h`, `shape.h`, `joint.h`, `character.h`, `vehicle.h`, `softbody.h`, `world.h`, `draw.h`, `replay.h`, and `maul3d.h`, which includes the other eleven. Nothing else is: not `src/`, and not `test/helpers.h`, the samples, the testbed, the bench or the tools in `tools/`, which use the API as any program does. From 1.0.0 all twelve keep the family's promise ([maul-conventions.md](maul-conventions.md)): a program that builds against 1.y builds against any later 1.z and means the same. Every def starts from its `m3Default...Def`, so a field a minor release adds to a def takes a default that keeps what the program asked for before.

The structs the library fills for the program to read follow one more rule:

- **Which.** The diagnostics (`m3Profile`, `m3Counters`, `m3MemoryUsage`), the event streams (`m3ContactEvents`, `m3SensorEvents`, `m3FragmentEvents`, `m3BodyEvents`, `m3JointEvents`) and the events they hold, `m3ContactData`, `m3BodyDiff`, the replay codec's `m3JournalInfo` and `m3ReplayView`, and the query and mover results (`m3CastHit`, `m3MoverPlane`, `m3MoverMove`, `m3WorldAabb`).
- **Layout.** Each keeps its layout within a minor version. A minor release may add a field at the end of one, and no 1.x release moves, removes or renames a field. A program reads them by field name, never by a size or an offset of its own. A program built against another minor version's headers is refused at `m3CreateWorld` by `M3_ABI_VERSION`.
- **Values.** A field keeps its meaning across 1.x. The value it holds for a given world may change, as any result may (an event's approach speed, a counter, a hash): in a minor release as a `feat`, or in a patch release as a `fix` that corrects it to what the documentation already promised, and the changelog says what changes ([release.md](../operations/release.md)). `m3Profile`'s times are wall-clock and differ from run to run.

`m3DebugDraw` goes the other way: the program fills it and the world reads it. A minor release may add a callback or a switch at its end. A program starts from a zeroed one (`m3DebugDraw draw = {0};`), so a field it does not know stays off, as a null callback skips its primitive.

### The bindings

- **C#.** `bindings/csharp/Maul3D.cs` ships with every release and mirrors the C API: the same names in C# case, the same struct layouts, the same results. It is under the same promise as the headers. It binds a part of the API and grows only when the C API does, by the same additive rules; a name it binds is never removed or renamed in 1.x. `tools/check_bindings.py` checks it against the headers, its `AbiVersion` included.
- **Godot.** `bindings/godot/` is a starter kit for a GDExtension. It follows Godot's own versions, which this project does not control, so it is outside the promise: a release may change it in any way, and its README says what it covers.

## Ids and misuse

A correct program never uses an id after destroying its object, so a stale id is misuse: it counts one in `m3Counters.misuseCount`, a getter returns the value its header gives for one (zero, or the identity for a rotation), a setter changes nothing, and a call with a status returns `m3_errorStale`. A destroy is such a call: destroying a stale id returns `m3_errorStale` and counts one misuse. Destroying a body takes its shapes, its joints and a vehicle whose chassis it is with it, so their ids are stale after. Joints also break on their own, and every break is reported in the joint event stream, so a program that reads its events never holds a broken joint's id unknowingly.

Untrusted bytes are not misuse. A snapshot, tape or container that fails its checks, and a value a tape operation refuses while it replays, return `m3_errorFormat`, or `m3_errorVersion` for bytes another release wrote, and count nothing.

A refusal counts one misuse only for an argument a correct program never passes: a null pointer, the null id or an id of the wrong kind, a NaN, a value outside its documented domain, a value past a published bound. A refusal a correct program can meet because of the world's state counts nothing, whatever its status: a full pool, an allocator that refuses, and an `m3_errorLimit` the world's masses, levers or surfaces decide rather than the argument. [The introspection section](../user/queries-and-events.md#introspection) lists every case.

## Host code

The world calls host code in four places: the task hooks during a step, the pre-solve callback while the step prepares contacts, the debug draw callbacks during `m3World_Draw`, and the allocator. While it runs, a count on the world is raised, and every writer checks it first: it counts one misuse and changes nothing, and a writer with a status also returns `m3_errorBusy`. Readers stay open, so a draw callback can ask the world about what it draws. `m3DestroyWorld` is a writer like the rest, so host code cannot destroy the world it runs in. The count is atomic because several readers may draw on separate threads at once. The allocator, the task hooks and the debug draw callbacks run in the thread's own floating-point setting, and the pre-solve callback in the library's, since its answer decides contacts.

## A step

`m3World_Step` checks its arguments and returns a status when it refuses, records the step when a journal is running, and runs the step with the host-code count raised:

1. **Vehicles.** The suspension impulses land first, so the narrowphase and the solver see the sprung chassis the way they see gravity.
2. **Collide.** Moved shapes update the broadphase tree, the pair table is rebuilt in key order (a full table keeps the lowest keys and counts the rest in `pairsDropped`), and every pair's manifold is computed again. Begin and end events are emitted from the difference.
3. **Prepare.** Sleeping islands touched by awake ones wake; contacts and joints are prepared, contacts graph-colored into blocks for the SIMD lanes, and the water field is built for the bodies that move.
4. **Solve.** In each substep: forces in, the warm start, the biased pass that pushes overlap and joint error out (joints first, so contacts see their result), integration, then the unbiased relax pass. After the substeps: restitution, the impulses stored for the next warm start, the wind, and the break check.
5. **Continuous.** Fast bodies sweep against the shapes they can touch, and one that would tunnel stops at its time of impact.
6. **Sleep.** Each island accounts its members' rest time; one that has rested long enough sleeps whole and stays bit-frozen until something wakes it.
7. **Events, characters and soft bodies.** Body move events are emitted, characters ride the bodies they stand on, and soft bodies solve their particles against the rigid world.

## How the parts fit

`tools/modules.txt` declares which module may include which, and `tools/check_modules.py` keeps the graph one-way.

- **World and state:** `world.c` (creation, settings), `world_state.c` (the pools and the state table, which also drives the hash), `world_hash.c`, `world_stats.c` (counters, memory, profile), `world_events.c`, `misuse.c` (refusals and host code), `allocator.c` (the one file that reaches `malloc`), `step_scratch.c`.
- **Objects:** `body*.c`, `shape*.c`, `joint*.c` (one file per joint type, the creates in `joint_create.c` and the run-time parameters in `*_params.c`), `character.c`, `vehicle*.c`, `softbody*.c`, `water.c`, `voxel*.c` and `destruction.c` (edits, fracture and explosions).
- **Collision:** `dynamic_tree.c` and `broad_phase.c`, `narrowphase.c`, `manifold*.c`, `triangle_contact.c`, `distance.c`, `hull.c` and `quickhull.c`, `mesh_bvh.c`, `continuous.c`.
- **Solver:** `solver.c` (the step's order), `contact_prepare.c`, `contact_solver.c`, `contact_kernel.c` (the SIMD lanes), `joint_solver.c`, `joint_row.c`, `island.c`, `integrate.c`, `rigid_flow.c`, `parallel.c` (the task hooks).
- **Queries:** `query*.c`, `raycast.c`, `shape_cast.c`, `mover.c`, `draw.c`.
- **Recording:** `snapshot*.c` (the walk, and how an incoming snapshot's arrays fit together), `journal.c`, `journal_replay*.c`, `replay.c` (the container), `diff.c` (`m3World_Compare`), `state_prefix.c`, and `format_layout.c`, which pins every recorded struct's layout. See [state-formats.md](state-formats.md).
- **Family files:** `cpu.c`, `fpenv.c`, `hash.c` and `simd.h` in `src/`, with the shared CMake, tools and test files, synced from the organization's `.github` repository by `family_sync.py` and never edited here.

## Assertions

`M3_ASSERT` (`src/core.h`) checks internal invariants only: it traps in builds without `NDEBUG` and compiles out of Release. Caller input is validated and refused with a status in every build.
