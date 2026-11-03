# Queries, events and introspection

- [Queries](#queries)
- [Events](#events)
- [The pre-solve veto](#the-pre-solve-veto)
- [Debug draw](#debug-draw)
- [Introspection](#introspection)

Queries ask the world questions, events report what the last step did,
and the counters and the debug draw show what a world holds. None of
them changes the world: a world read between two snapshots snapshots to
the same bytes, and queries are safe at any point between steps.

## Queries

```c
m3Pos3 from = {0.0, 10.0, 0.0};
m3Vec3 down = {0.0f, -20.0f, 0.0f}; // the ray runs to y = -10
m3CastHit hit;
bool found = m3World_CastRayClosest(world, from, down, m3DefaultQueryFilter(), &hit);
m3Pos3 ground = from;
if (found)
{
    ground = hit.point; // where the ray met the floor
}
```

- **Rays.** `m3World_CastRayClosest` returns the first hit along a
  segment, `m3World_CastRayAll` every hit, sorted by distance and then
  by shape. A ray sees only the front of a surface: one that starts
  inside a shape or exactly on its surface misses that shape. Ask
  `m3World_TestPoint` about containment instead.
- **Shape casts.** `m3World_CastSphereClosest`,
  `m3World_CastCapsuleClosest`, `m3World_CastBoxClosest` (a centre,
  half extents and a rotation) and `m3World_CastHullClosest` (the hull
  of 2 to `M3_MAX_CAST_POINTS`, 64, points around a base) sweep a shape
  along a translation and report the first contact. A cast that starts
  touching something reports a fraction of 0 and a zero normal. A
  translation past 1e18 m on an axis is refused.
- **The hit.** Every closest cast clears its `m3CastHit` first. On a
  hit it returns true and fills in the shape, the world point, the
  surface normal and the fraction of the translation travelled; on a
  miss or a refusal it returns false and leaves the hit zeroed, so no
  earlier hit survives.
- **Points.** `m3World_TestPoint` says whether a point is inside a
  shape. A plane is a solid half-space and a voxel chunk's cells are
  solid; a mesh is an open surface, with no inside.
- **Overlaps.** `m3World_OverlapAabb`, `m3World_OverlapSphere`,
  `m3World_OverlapCapsule`, `m3World_OverlapBox` and
  `m3World_OverlapHull` list the shapes that overlap a region, exactly
  per shape and not by bounding boxes, in ascending shape order.
- **Movers.** `m3World_CastMover` and `m3World_CollideMover` cast a
  capsule and list the planes around it, and `m3SolveMover` and
  `m3ClipMoverVelocity` turn those planes into a move, for hosts that
  build their own character ([the character
  controller](character.md#a-character-with-real-mass)).

Every query takes an `m3QueryFilter`, which works like a shape's filter
([collision filters](bodies-and-shapes.md#collision-filters));
`m3DefaultQueryFilter` sees everything. Disabled bodies are invisible
to every query.

A call that fills an array takes the array's capacity right after it,
writes at most that many entries, and returns the total, which may be
more; a null array with a capacity of 0 measures the total first. A
null array with a positive capacity, or a negative capacity, is
refused. A refused call returns 0 and zeroes the array over its whole
capacity, so no entry from an earlier call reads as an answer; a
successful one, an empty answer included, writes only its own entries.
List calls only read, so none is refused for a busy world.

## Events

Each step rebuilds the event streams. They stay valid until the next
step, a restore or the world's destruction, and their order is fixed:
the same step always reports the same events in the same order.

- **Contacts.** `m3World_GetContactEvents` gives the pairs of shapes
  that began and ended touching in the last step.
- **Hits.** A contact on a shape with `enableHitEvents` reports a hit
  when its fastest approaching point closes faster than the world's
  hit threshold (`m3World_SetHitEventThreshold`), at most once per
  contact per step, with the point, the normal and the approach speed. Hits past the
  stream's capacity still happen; `hitsDropped` counts the events lost.
- **Sensors.** `m3World_GetSensorEvents` gives the shapes that entered
  and left each sensor, each event marked with the step it belongs to.
  Contact events stay silent for sensors, and a sleeping body inside a
  sensor stays asleep with its begin event open.
- **Moves.** `m3World_GetBodyEvents` lists every body that moved in the
  last step, in ascending body order, with its new transform and
  whether it fell asleep. A sleeping body costs nothing here. Reading
  moves is cheaper than polling every body.
- **Joint breaks.** `m3World_GetJointEvents` lists the joints that broke
  in the last step ([joints](joints.md)). Their ids are already stale:
  use them as keys.
- **Fragments.** `m3World_GetFragmentEvents`, from voxel edits
  ([destruction](destruction.md#fragments)).

Events observe; they are not state. A restore clears every stream.

## The pre-solve veto

`m3World_SetPreSolveCallback` installs a function the step calls for
each contact about to be solved whose shapes enabled pre-solve events
(`enablePreSolveEvents` or `m3Shape_EnablePreSolveEvents`). Returning
false disables that contact for that step only: a one-way platform
lets a body through from below this way. The callback is called from
one thread, in a fixed pair order, with the deepest point and the
normal from shape A to shape B.

Each step's vetoes are recorded by the journal, so a replay applies
them without the callback, and lands on the recorded bits. Rollback is
different: resimulating from a snapshot calls the callback again. The
callback must therefore be a pure function of its arguments, and of
host state that is the same on every run. One that reads a clock, a
random source or changing host state breaks the host's own rollback,
not the engine. Installing the callback is host wiring, and neither
the journal nor a snapshot holds it.

## Debug draw

`m3World_Draw` draws the world through the callbacks of an
`m3DebugDraw`: segments, points and triangles, each with a colour. Its
switches choose the layers: shape wireframes, solid shapes, contact
points with their normals scaled by impulse, joints, the broad phase's
bounds, a grey tint for sleeping bodies, island colours, each body's
principal inertia axes, and the broad-phase tree's inner boxes. Start
from a zeroed `m3DebugDraw`, so a layer a later release adds stays off.
A draw changes nothing in the world.

## Introspection

- `m3World_GetCounters` gives a census: bodies, shapes, joints,
  contacts, characters, vehicles, soft bodies, voxel chunks, shared
  hulls and meshes, awake bodies, the last step's islands and solver
  colours, the broad-phase tree's height, the blocks the world took
  from its allocator and gave back, and the counts of misuse, refused
  voxel edits, dropped carves, dropped pairs and retaken pushes.
- `m3World_GetMemoryUsage` gives the world's persistent bytes, its
  shared content and its step scratch with the scratch's high-water
  mark.
- `m3World_GetProfile` gives the last step's wall-clock time per phase:
  vehicles, broad phase, narrow phase, events, preparation, solving,
  continuous collision, sleep, characters, soft bodies and the whole.
  The counters are deterministic; the times are not.
- `m3Body_GetContactData` and `m3Shape_GetContactData` answer "what
  touches me": each touching contact in a fixed order, with its world
  points, separations and the last step's normal impulses.
- `m3World_GetBodies` and `m3World_GetJoints` list every live body and
  joint in slot order.

Reading any of these changes nothing, and a test holds a world that
polls them every step to the bits of a twin that never looks.

Every refusal returns its result and changes nothing. A refusal also
counts in `m3Counters.misuseCount` when a correct program would never
make it, so a host can check the counter once a frame and catch bugs in
release builds. The engine has no log; the counter and the results are
how it reports.

Which refusals count misuse, in one place:

- **Misuse, counted once.** What a correct program never does: a null
  pointer, the null id or an id of the wrong kind, a stale id, a NaN, a
  value outside its documented domain, an argument past a published
  bound, a call that changes the world from host code the world is
  running, and a journal call in the wrong recording state.
- **Not misuse, counted nothing.** What a correct program can meet: a
  full pool or caller buffer, an allocator that refuses, a CPU without
  the build's SIMD backend, bytes from another version or damaged
  bytes, and an `m3_errorLimit` that the world's state decides rather
  than the argument. Those are a voxel edit whose surface would pass
  `M3_MAX_VOXEL_SURFACE_BOXES`, an anchor past a full soft body table, a
  shape create, resize, density change or destroy that would carry the
  body's total mass past `M3_MASS_LIMIT` or a joint past its lever
  ratio, a joint create past that ratio, a type change the body's
  shapes, joints or speed refuse, a servo target more than
  `M3_TARGET_REACH_LIMIT` from where the step left the kinematic body,
  and a stand-up the character's stance veto refuses.
