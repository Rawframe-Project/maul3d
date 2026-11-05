# Bodies and shapes

- [Bodies](#bodies)
- [Shapes](#shapes)
- [Forces and impulses](#forces-and-impulses)
- [Moving bodies by hand](#moving-bodies-by-hand)
- [Sleep](#sleep)
- [Materials](#materials)
- [Collision filters](#collision-filters)
- [Continuous collision](#continuous-collision)
- [World settings](#world-settings)

## Bodies

```c
m3BodyDef def = m3DefaultBodyDef();
def.type = m3_dynamicBody;
def.position = (m3Pos3){0.0, 2.0, 0.0};
def.motionLocks.angularX = true; // stays upright: no tipping about x or z
def.motionLocks.angularZ = true;
m3BodyId crate;
m3Result result = m3CreateBody(world, &def, &crate);
m3ShapeDef shapeDef = m3DefaultShapeDef();
shapeDef.density = 500.0f; // kg/m3
m3ShapeId box;
if (result == m3_success)
{
    result = m3CreateBoxShape(world, crate, &shapeDef, (m3Vec3){0.5f, 0.5f, 0.5f}, &box);
}
```

A body is **static** (never moves, and takes no velocity), **kinematic** (moves at the
velocity it is given and pushes dynamic bodies without being pushed)
or **dynamic** (moved by forces, contacts and joints). The default def
is a static body at the origin.

| Field | Default | Meaning |
|---|---|---|
| `type` | static | `m3_staticBody`, `m3_kinematicBody` or `m3_dynamicBody`. |
| `position`, `rotation` | the origin, identity | The pose. The rotation must be near unit length. |
| `linearVelocity`, `angularVelocity` | zero | Within 1e5, or 1e10 m/s for a kinematic body's linear velocity. A static body's are zero: create refuses any other, and the velocity setters refuse a static body. |
| `gravityScale` | 1 | The share of world gravity the body feels. |
| `linearDamping`, `angularDamping` | 0 | Per-second slowing. |
| `enableBullet` | false | Continuous collision against moving bodies too. |
| `motionLocks` | none | Axes held still: a locked axis's velocity is zeroed every substep. |
| `enableSleep` | true | False keeps the body awake. |
| `sleepThreshold` | 0 | The resting speed; 0 uses the world's default of 0.05 m/s. |
| `enable` | true | False creates the body outside simulation and queries. |
| `enableFastRotation` | false | Lets the body spin past the world's angular speed cap. |
| `userData` | 0 | The host's own value. |

A dynamic body's mass, centre of mass and inertia come from its
shapes, each weighted by its density: `m3Body_GetMass`,
`m3Body_GetLocalCenter` and `m3Body_GetCentralInertia` read them. No
call sets them directly. A dynamic body with no shape weighs 1 kg.
`m3Body_SetName` gives a body a UTF-8 name of up to
`M3_MAX_BODY_NAME_BYTES` (31) bytes, for tools and logs.

## Shapes

A shape gives a body geometry. A body may carry many, each placed by
the def's `localPosition` and `localRotation`, so a compound body is
several shapes on one body; its mass adds them up about their
offsets.

| Create | Shape | Notes |
|---|---|---|
| `m3CreateSphereShape` | sphere | A centre and a radius. |
| `m3CreateCapsuleShape` | capsule | A segment and a radius; a segment of zero length is refused. |
| `m3CreateBoxShape` | box | Half extents; built as a convex hull. |
| `m3CreateCylinderShape` | cylinder | A prism of `2 * segments` vertices, built as a hull; segments clamp to 3 to 32, and 24 rolls smoothly at metre scales. |
| `m3CreateHullShape` | convex hull | The hull of 4 to 256 points, at most 64 vertices, 124 faces and 372 half edges. |
| `m3CreateMeshShape` | triangle mesh | Static bodies only; up to 65535 vertices and 65535 triangles, counter-clockwise seen from outside. |
| `m3CreateHeightFieldShape` | height field | Static bodies only; a grid of 2 to 255 samples per axis, `cellSize` apart, from the body's origin, four bytes per sample. |
| `m3CreateHeightFieldMeshShape` | height field as a mesh | The same grid, 2 to 32 samples per axis, built as a mesh. |
| `m3CreatePlaneShape` | plane | Static bodies only; an infinite half-space. |
| `m3CreateVoxelChunkShape` | voxel chunk | Static bodies only ([destruction](destruction.md)). |

Hulls and meshes with the same content are stored once and shared.
`m3Shape_SetSphere` and `m3Shape_SetCapsule` change a sphere or capsule
shape's geometry in place, one into the other included; the body's
mass follows, and everything around the old and the new bounds wakes.
Other shapes cannot be reshaped. `m3DestroyShape` removes a shape and
rebuilds its body's mass.

`m3MeshShape_SetMaterials` paints a mesh: up to
`M3_MAX_MESH_MATERIALS` (8) materials, with friction, restitution,
rolling resistance and surface velocity each, and a material index per
triangle. A contact on a painted triangle uses that material in place
of the shape's own, so one level mesh can hold ice, tarmac and a
moving walkway. A contact that touches several triangles takes the
material of its first point.

## Forces and impulses

- `m3Body_ApplyForce`, `m3Body_ApplyTorque` and
  `m3Body_ApplyForceAtPoint` add to what the next step applies,
  beside gravity, over every substep; the step then clears them. They
  are part of a snapshot, so a snapshot taken between a force and its
  step replays the same motion.
- `m3Body_ApplyLinearImpulse`, `m3Body_ApplyAngularImpulse` and
  `m3Body_ApplyLinearImpulseAtPoint` change the velocity at once.
- `m3Body_SetLinearVelocity` and `m3Body_SetAngularVelocity` set it.

A point is given in world space; the arm runs from the centre of mass.
Only dynamic bodies take forces and impulses, and anything that is not
zero wakes the body. A disabled body takes no force or torque, since the
step does not move it, and a body that stops moving before the step,
turned static or kinematic, put to sleep or disabled, drops the force
and torque it held. An impulse whose result would pass 1e5 m/s or
rad/s is refused.

## Moving bodies by hand

- `m3Body_SetTransform` teleports a body: the pose lands at once,
  velocities stay, and bodies around both the old and the new place
  wake.
- `m3Body_SetTargetTransform` drives a kinematic body to a pose: the
  next step gives it the velocities that land it exactly there, then
  the target clears. It is how to move elevators, doors and platforms.
  A target more than 1e6 m away is refused with `m3_errorLimit`.
- `m3Body_SetType` turns a body static, kinematic or dynamic. Turning
  dynamic rebuilds its mass from its shapes, turning static zeroes its
  velocities, and the bodies around it wake. A body with a plane,
  mesh, height field or voxel chunk stays static.
- `m3Body_Enable` with false takes a body out of the simulation and
  out of every query without destroying it.
- `m3Body_SetMotionLocks` holds any of the six axes still, for 2.5D
  games and upright enemies.

## Sleep

A body at rest falls asleep and costs nothing until something wakes
it ([determinism](determinism.md#the-step)). `m3Body_EnableSleep`
keeps one body awake, `m3Body_SetSleepThreshold` changes its resting
speed, and `m3Body_SetAwake` wakes it or puts it to sleep at once with
its velocities zeroed. Only a dynamic body sleeps: a static or kinematic
body is always awake, and `m3Body_SetAwake` refuses it.

## Materials

Each shape has a `friction` (default 0.6), a `restitution` (default 0),
a `rollingResistance` (default 0) and a `density` (default 1 kg/m3),
and the setters change them while the game runs; `m3Shape_SetDensity`
rebuilds the body's mass. A contact reads its shapes' materials when the
step prepares it, so a change applies from the next step, and a
sleeping stack keeps its old values until something wakes it.

Rolling resistance brakes rolling at contacts, the larger of the two
shapes' values, scaled by the pair's size. Without it a sphere rolls
forever, and the bodies it touches never sleep; 0.01 to 0.1 feels like
soft ground. A shape's `surfaceVelocity` makes it a conveyor: contacts
drive the speed along the surface toward it, so crates ride a belt at
the belt's speed, and a belt on a dynamic deck pushes the deck back.

## Collision filters

Each shape has `categoryBits` (default 1), `maskBits` (default all bits)
and `groupIndex` (default 0). Two shapes collide when each one's
category shares a bit with the other's mask. A shared non-zero group
overrides that: positive always collides, negative never does.
`m3Shape_SetFilter` changes the three while the game runs. Every ray,
cast and overlap takes an `m3QueryFilter` the same way;
`m3DefaultQueryFilter` sees everything.

A **sensor** shape, `isSensor` in its def, detects overlap and fires
its own begin and end events, but never pushes: bodies pass through it,
it stops nothing, and it wakes nothing. Sensors do not sense other
sensors.

## Continuous collision

A fast dynamic body sweeps against static geometry each step, so it
cannot pass through a thin wall or a floor between steps. A body with
`enableBullet` also sweeps against dynamic and kinematic bodies. Two
bullets do not sweep against each other. `m3World_EnableContinuous`
turns the whole pass off or on; it is on by default.

## World settings

The world def sets these at creation, and each setter changes one while
the game runs:

| Setter | Default | Meaning |
|---|---|---|
| `m3World_SetGravity` | (0, -10, 0) | Sleeping bodies stay asleep until something wakes them. |
| `m3World_SetContactTuning` | 30 Hz, damping ratio 10, 3 m/s | How stiff contacts are, and how fast an overlap is pushed apart. Contacts with static bodies run at twice the frequency. |
| `m3World_SetRestitutionThreshold` | 1 m/s | Impacts slower than this do not bounce. |
| `m3World_SetMaximumLinearSpeed` | 400 m/s | A hard cap on every dynamic body, every substep. |
| `m3World_SetMaximumAngularSpeed` | 800 rad/s | The same for spin; a body with fast rotation enabled may exceed it, up to 1e5 rad/s. |
| `m3World_EnableSleep` | on | Off wakes every body. |
| `m3World_EnableContinuous` | on | The continuous pass above. |

The speed caps guard against a scene that blows up; they sit far above
anything a working scene reaches. Before them, the step adds the
increment of each force and torque, the host's and the water's, to the
velocities exactly as computed. Only an increment whose sum
would leave float range is held to `M3_SPEED_LIMIT` along its
direction, so a velocity the damping or the caps then read is always
finite, and every step whose arithmetic stays finite keeps its result. Wheels and other legal fast spinners
get `m3Body_EnableFastRotation`. Settings are world state: the journal
records them, a snapshot carries them, and the hash takes them in when
they differ from their defaults.

`m3World_RebuildBroadphase` rebuilds the broad-phase tree as a
balanced tree over the live shapes. Building a level one shape at a time
can leave the tree lopsided; one call afterwards makes queries faster.
It changes no result, since the tree is not part of the hash and pairs
are kept in a fixed order.
