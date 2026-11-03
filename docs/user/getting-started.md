# Getting started

These pages are the contract between the engine and the program that
uses it, the host: what each call does, what it refuses, and what the
engine promises in return. The [README](../../README.md) says what
Maul3D is and how to build it. Where a page gives a number, it is the
number in the code, and the C on each page is built and run by the
tests exactly as it is shown.

## A first world

```c
m3WorldDef worldDef = m3DefaultWorldDef();
m3World* world = NULL;
m3Result result = m3CreateWorld(&worldDef, &world);
if (result != m3_success)
{
    return;
}

m3BodyDef groundDef = m3DefaultBodyDef(); // a static body
m3BodyId ground;
result = m3CreateBody(world, &groundDef, &ground);
m3ShapeDef shapeDef = m3DefaultShapeDef();
m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f}; // solid below y = 0
m3ShapeId floorShape;
if (result == m3_success)
{
    result = m3CreatePlaneShape(world, ground, &shapeDef, &floor, &floorShape);
}

m3BodyDef ballDef = m3DefaultBodyDef();
ballDef.type = m3_dynamicBody;
ballDef.position = (m3Pos3){0.0, 4.0, 0.0};
m3BodyId ball;
if (result == m3_success)
{
    result = m3CreateBody(world, &ballDef, &ball);
}
m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
m3ShapeId ballShape;
if (result == m3_success)
{
    result = m3CreateSphereShape(world, ball, &shapeDef, &sphere, &ballShape);
}

for (int32_t i = 0; i < 120 && result == m3_success; ++i)
{
    result = m3World_Step(world, 1.0f / 60.0f, 4); // 60 Hz, four substeps
}
m3Pos3 p = m3Body_GetPosition(world, ball); // resting on the floor: y = 0.5
m3DestroyWorld(world);
```

## The objects

A **world** owns everything else. Worlds share nothing, so two worlds
can step on two threads at once, and an id from one world means
nothing to another.

A **body** has a pose and a velocity, and is static, kinematic or
dynamic. A **shape** gives a body its geometry: a sphere, a capsule, a
box or cylinder (both built as convex hulls), a convex hull of points,
a triangle mesh, a height field, a voxel chunk, or an infinite plane on
a static body. A **joint** connects two bodies: spherical, revolute,
prismatic, weld, distance, generic, wheel, filter, parallel, motor,
gear or pulley ([joints](joints.md)). On top of these sit
[characters](character.md), [vehicles](vehicles.md),
[soft bodies](soft-bodies.md) and [water volumes](water.md).

Every object is made from a **def**, a struct that starts from its
`m3Default...Def` function and is changed field by field. A def carries
a private cookie, so one that did not come from its default function is
refused. A world holds as many of each kind as its def's capacities say:
creates succeed up to exactly that number, and the next is refused with
`m3_errorCapacity`. Size a world for its scene, with room for fragments.

## Results and refusals

A call that can fail returns an `m3Result`: `m3_success` (0) or a
negative error. A refused call changes nothing. The library never
aborts on what a host passes it: bad defs, stale ids, full pools and
NaN floats are refused through the result, and its assertions guard
only internal states no call can reach. A call that takes an argument
a correct program never passes, such as a null pointer, a NaN or a
value outside its documented domain, also counts one misuse in
`m3World_GetCounters`; a refusal a correct program can meet, such as a
full pool, counts nothing. `m3GetResultName` names any result.

## Ids

An id is a slot and a generation, `{index1, generation}`, where
`index1` is the slot plus one, so the zeroed id is the null id. Destroying
an object moves its slot to the next generation, so an old id never
names the object that reuses the slot. A stale id counts one misuse:
a getter given one returns zeros (the identity, for a rotation), a
command changes nothing, and a call with a result returns
`m3_errorStale`.

A slot's generation runs from 0 to 2^32 - 1; then the slot retires
rather than wrapping around. A pool of n slots therefore makes
n times 2^32 objects over a world's life: a thousand slots used a
thousand times a second last about 136 years. A pool whose slots have
all retired refuses its next create with `m3_errorCapacity`. Snapshots
carry each slot's generation, so a restore brings back the world's
remaining budget, not a fresh one.

Destroying a body takes its shapes, its joints and a vehicle it
carries with it. Destroying a world destroys everything in it.

## Units and conventions

- SI units: metres, kilograms, seconds, radians. Gravity defaults to
  (0, -10, 0).
- Right-handed coordinates. Quaternions are x, y, z, w; a def's
  rotation must be near unit length, and one that is not is refused,
  never quietly normalized.
- Positions are `double`; everything else is `float`.
- A plane is a half-space: points with `dot(normal, p) <= offset`
  are solid, in the body's frame.
- Mesh triangles wind counter-clockwise seen from outside; their back
  sides do not collide.
- A capsule is a segment with a radius. A capsule of zero length is
  refused: that is a sphere.

## Stepping

`m3World_Step(world, dt, substeps)` advances the world by `dt`
seconds, 1e-4 to 10, in 1 to 256 substeps. More substeps make stiff
scenes, stacks and chains steadier at the cost of time. A step never
asks the allocator: the world sets aside what a step needs when it is
created, sized by its capacities. [Determinism and the
step](determinism.md) says what a step promises.

A world steps on the calling thread by default. A host with its own
task system passes `enqueueTask` and `finishTask` in the world def, and
the step spreads its work over them. The worker count changes how
fast a step runs, never what it computes.

## What to wire first

1. Create a world, step it and read poses: `m3CreateWorld`,
   `m3World_Step`, and the move events, which list the bodies that
   moved ([queries and events](queries-and-events.md)).
2. Create and destroy at run time; a destroyed body takes its shapes
   and joints with it.
3. Collision filters on shapes, and the same filters on queries.
4. Gameplay forces and impulses ([bodies and shapes](bodies-and-shapes.md)).
5. Teleports, moving platforms and elevators: set a pose, or drive a
   kinematic body to a target.
6. Materials and world settings, which can change while the game runs.
7. Contact events, hit events and the pre-solve veto.
8. Motors, servos and breakable joints ([joints](joints.md)).
9. Rays, shape casts and overlaps.
10. Snapshots and the journal for rollback and replays
    ([snapshots and rollback](snapshots-and-rollback.md),
    [the journal](journal-and-replays.md)).

## At scale

- Give rolling debris a small `rollingResistance`, 0.01 to 0.1. A
  sphere that rolls without it never stops, and one body that never
  stops keeps everything it touches awake.
- Tile large levels as many chunks, voxel, mesh or height field, each
  on a static body. Voxel chunks side by side weld into one surface
  ([destruction](destruction.md)).
- The `cityblock` scene in `bench/`, five thousand bodies with voxel
  towers and a fracture wave, prints its timings on your machine. CI's
  bench job pins its final hash on every push to main and every night,
  on six native 64-bit cells: GCC 14 on x64 and arm64 Linux, Clang 20
  on x64 Linux, Clang on arm64 macOS, and clang-cl on x64 and arm64
  Windows.
