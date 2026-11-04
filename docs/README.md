# Maul3D documentation

## Using the engine

- [Getting started](user/getting-started.md): the object model, ids, units, scale and the integration checklist.
- [Determinism and the step](user/determinism.md): what is bit for bit, the step, the task interface and derived data.
- [Bodies and shapes](user/bodies-and-shapes.md): forces, body control, materials, compounds, the overlap family, heightfields, filters and continuous collision.
- [Queries, events and introspection](user/queries-and-events.md): sensors, contact and hit events, the pre-solve veto, queries, the debug draw and the counters.
- [Joints](user/joints.md): every joint type, drives, breakage, the wheel joint, the servo weld, gears and pulleys.
- [Snapshots and rollback](user/snapshots-and-rollback.md): the snapshot, restore, and lockstep and rollback networking.
- [The journal and replay files](user/journal-and-replays.md): recording a session, replaying it and the replay container.
- [Destruction and voxels](user/destruction.md): voxel chunks, fracture and explosions.
- [Vehicles](user/vehicles.md): raycast vehicles and the drivetrain.
- [The character controller](user/character.md): the character, the rigid character recipe and stance.
- [Soft bodies](user/soft-bodies.md): cloth and soft bodies, their depth and their fields.
- [Water volumes](user/water.md): buoyancy and drag.
- [Limits and capacities](user/limits.md): every magnitude limit and capacity.
- [Samples](user/samples.md): small complete programs.
- [Bindings](user/bindings.md): the C# binding and the Godot starter kit.
- [The API reference](https://rawframe-project.github.io/maul3d/api.html): every public function, generated from the headers.

## How it works

- [Overview](internals/overview.md): the API, the objects, the formats and the promise.
- [Determinism](internals/determinism.md): how the bits are kept the same.
- [The state formats](internals/state-formats.md): the snapshot, the journal tape and the replay container.
- [The free rotation](internals/rotation.md): how a free body turns through a substep.
- [Glossary](internals/glossary.md): the terms these pages use.
- [References](internals/references.md): the published sources behind the algorithms.
- [The conventions](internals/maul-conventions.md): the rules every Maul library follows.

## Running the project

- [Development](operations/development.md): the toolchain, the presets, the tests, the golden values and finding where two runs part.
- [Checks](operations/checks.md): what verifies a change, locally and in CI.
- [Release](operations/release.md): how a release is made and checked.
- [The changelog](../CHANGELOG.md): every release and what changed.
