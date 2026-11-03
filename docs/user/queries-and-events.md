# Queries, events and introspection

## Sensors

A shape created with `isSensor` detects and never responds:

- No contact response: bodies pass through, with no impulse and no
  effect on trajectories.
- Its own event streams, read with `m3World_GetSensorEvents`,
  derived by the same canonical walk as contact events. Each event
  names its `sensorShapeId` and its `visitorShapeId`, and carries the
  `step` it belongs to: `m3World_GetStepCount` before that step.
  Contact streams stay silent for sensor pairs.
- No waking: a sleeping body inside a sensor keeps sleeping; the
  begin event stays open (narrowphase keeps watching because only
  the solver sleeps). No island coupling.
- No stopping power: continuous collision ignores sensor targets,
  and a sensor shape on a fast body sweeps nothing.
- Sensors do not sense other sensors.

## Events

Contact begin/end and sensor begin/end streams are rebuilt every
step and valid until the next `m3World_Step`, `m3World_Restore`, or
world destruction. Restore clears all streams (events are
observations, not state). Event order within a step is canonical and
deterministic. The count pointer of the accessor may be null if the
caller only wants the array (it is then unbounded and useless: pass
the pointer).

## Hit events, move events, and the pre-solve veto

Hit events are opt-in per shape (`m3Shape_EnableHitEvents`) and
fire at most once per contact per step, for the fastest
approaching manifold point, when the approach speed clears
`m3World_SetHitEventThreshold`. Body move events arrive one per
mover per step in ascending body order with the post-step
transform and a fell-asleep mark; sleeping bodies cost nothing.
The joint break stream reports joints that destroyed themselves
this step; the id inside is already stale, use it as a key. The
pre-solve veto (`m3World_SetPreSolveCallback` plus
`m3Shape_EnablePreSolveEvents`) runs serially in canonical pair order
and may disable any flagged contact for the step: the one-way
platform tool. THE CONTRACT, LOUD: the callback must be a pure
function of its arguments; the journal records inputs, never host
whims, so an impure callback breaks YOUR replay, not the
engine's. All event streams are transient observers: the next
step or a restore clears them.

## Queries

- Rays (`m3World_CastRayClosest`, `m3World_CastRayAll`) hit front
  faces only. A ray starting inside or exactly on a surface reports
  a miss for that shape; use `m3World_TestPoint` to ask about
  containment. All-hits results are sorted by fraction, ties by
  shape index; when the output array fills, the farthest hit drops.
- Shape casts (`m3World_CastSphereClosest`,
  `m3World_CastCapsuleClosest`, `m3World_CastBoxClosest`,
  `m3World_CastHullClosest`) report the first time of impact along
  a translation. A cast that starts overlapped reports fraction
  zero with a zero normal. Box casts take a center, half extents,
  and an orientation; hull casts take up to 24 base-relative cloud
  points. A one-point skinless cloud misses by contract (that is a
  ray). Any translation component beyond 1e18 misses by contract:
  past that, the kernels' float arithmetic would overflow (the
  caster's float budget).
- Every closest cast, the mover cast included, clears its
  `m3CastHit` at `hitOut` first, then returns `true` and writes the
  hit there, or returns `false` and leaves it zeroed, on a miss as
  on a refused input, so no earlier hit survives. `m3World_CastRayAll`
  fills an array of the same `m3CastHit`.
- `m3World_TestPoint` treats planes as solid half-spaces and
  meshes as open surfaces (a point is never "inside" a mesh).
- Overlap queries (`m3World_OverlapAabb`, `m3World_OverlapSphere`)
  return exact shape-level results, planes included.
- Every call that fills an array takes its filter, when it has one,
  before the array, and the array's capacity right after it. It
  writes at most that many entries and returns the total, which may
  be more; a NULL array with capacity 0 measures the total, so a
  caller can size the array first. A NULL array with a positive
  capacity, or a negative capacity, is refused.
- A refused list call (a NULL world, a stale id, a bad argument)
  returns 0 and zeroes the array over its whole capacity, so no entry
  of an earlier call is read as the answer; zeroed ids are null ids.
  A successful answer, an empty one included, writes only its entries
  and leaves the rest of the array as it was. List calls only read,
  so none is refused as busy. A refused `m3World_Snapshot` zeroes its
  buffer over its capacity in the same way, so a host that rings
  snapshot buffers sizes each with `m3World_GetSnapshotSize` first.

Queries read the world and never write it; they are safe to call at
any point between steps and have no effect on determinism.

## Debug draw

`m3World_Draw` emits the world as segments and points through host
callbacks (wireframes for every family, contact points with
impulse-scaled normal whiskers, joint anchors, broadphase fat AABBs,
a fixed color palette with sensor, sleeping, kinematic, and static
tints). It is a pure observer: a draw pass between two snapshots
leaves every byte identical, and the suite holds that promise on
every commit.

## Introspection

m3World_GetCounters reports the live census: bodies, shapes,
joints, contacts, characters, vehicles, soft bodies, voxel
chunks, interned hull and mesh slabs, awake dynamic bodies, the
last step's island and solver-color counts, the broadphase tree
height, and the process-wide persistent allocation/free call
counts. m3World_GetMemoryUsage reports the step scratch high water
against its reserve, and m3World_GetSnapshotSize the exact snapshot
size. m3World_GetProfile reports
wall-clock milliseconds per phase of the last completed step
(vehicles, broadphase, narrowphase, events, prepare, solve,
continuous, sleep, characters, soft bodies, and the whole).

Both are PURE observers: reading them never touches the
simulation, the snapshot, or the hash, and a suite holds that
promise with a twin that polls every tick against a twin that
never looks. Counter values are deterministic; profile times are
wall-clock and never will be.

m3World_Draw draws everything through one m3DebugDraw: solid
triangles (drawSolidShapes), wireframes, bounds, contacts and joints,
and the analysis layers: island tint points (colors cycled by island
root, sleeping bodies keep the tint of the island they slept in),
center-of-mass axis frames, and the broadphase tree's internal node
boxes. One purity gate covers every layer.

Bodies take debug names (m3Body_SetName: UTF-8 with its byte length,
at most M3_MAX_BODY_NAME_BYTES, journaled, carried by snapshots, NEVER
hashed; m3Body_GetName copies it out with no terminator) and
answer "who touches me" through m3Body_GetContactData and
m3Shape_GetContactData: manifold entries in canonical pair order
with world points, separations, and last-step normal impulses.

Every refusal returns its status and changes nothing. Misuse against a
live world also counts in m3Counters.misuseCount, so a host can poll once
per frame and catch bugs in release builds. Refusals never assert.

Which refusals count misuse, in one place:

- **Misuse, counted once.** What a correct program never does: a null
  pointer, the null id or an id of the wrong kind, a stale id, a NaN, a
  value outside its documented domain, an argument past a published
  bound, a writer called while the world is busy, and a journal call in
  the wrong recording state.
- **Not misuse, counted nothing.** What a correct program can meet: a
  full pool or caller buffer, an allocator that refuses, a CPU without
  the backend, bytes from another version or damaged bytes, and an
  `m3_errorLimit` that the world's state decides rather than the
  argument. Those are a voxel edit whose surface would pass
  `M3_MAX_VOXEL_SURFACE_BOXES`, an anchor past a full soft body table, a
  shape create, resize, density change or destroy that would carry the
  body's total mass past `M3_MASS_LIMIT` or a joint past its lever
  ratio, a joint create past that ratio, a type change the body's
  shapes, joints or speed refuse, and a servo target more than
  `M3_TARGET_REACH_LIMIT` from where the step left the kinematic body.
m3SetAssertHandler installs a host hook ahead of the abort for a
failed internal invariant; returning nonzero declares the failure
handled (test harnesses, crash reporters). There is no log hook:
the engine has no log stream to route.
