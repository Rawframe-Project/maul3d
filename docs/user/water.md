# Water volumes

A water volume is a box of still or flowing water anchored in the
world. Bodies and soft particles inside it float, slow down and drift
with the current. There is no fluid simulation: the surface is the
box's top face, and the water itself never moves.

## Creating a volume

```c
m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
def.bounds.lowerBound = (m3Pos3){-10.0, -5.0, -10.0};
def.bounds.upperBound = (m3Pos3){10.0, 0.0, 10.0}; // the surface is y = 0
def.flow = (m3Vec3){1.5f, 0.0f, 0.0f};             // a current along x
m3WaterVolumeId pool;
m3Result result = m3CreateWaterVolume(world, &def, &pool);
```

The defaults are the unit box at the origin, a density of 1000 kg/m3,
a linear drag of 2 per second, an angular drag of 1 per second and no
flow. The fields:

| Field | Meaning | Accepted |
|---|---|---|
| `bounds` | The box, in world coordinates. | upper above lower on every axis, each bound within 1e15 m |
| `density` | kg/m3; 1000 is water. | above 0, at most 1e6 |
| `linearDrag` | The rate, per second, at which a body's velocity is pulled toward the flow. | finite, 0 or more |
| `angularDrag` | The rate, per second, at which a body's spin is damped. | finite, 0 or more |
| `flow` | The current's velocity, in the world frame. | each component within 1e5 m/s |

A def that did not come from `m3DefaultWaterVolumeDef`, an empty or
inverted box, a NaN, a density that is not positive, or a drag that is
negative or not finite is refused with `m3_errorInvalid`. A bound, a
density or a flow past its limit, infinities included, is refused with
`m3_errorLimit`. A world holds at most `M3_MAX_WATER_VOLUMES` (8)
volumes, and a ninth is refused with `m3_errorCapacity`.
`m3DestroyWaterVolume` removes one, and `m3WaterVolume_IsValid` says
whether an id still names a live volume.

## What a body feels

Each step, every shape of a dynamic body is measured against every
live volume by its bounding box: the box the broad phase keeps for it,
a little larger than the shape (in release 1.0.0, 2 cm on every side,
and 10 cm for a native height field). The part of that box inside the water gives:

- **Buoyancy**: a force against gravity of the submerged volume times
  the water's density, applied at the centre of the submerged part.
  A box half under water feels half its full lift, and a body that
  dips one end feels a torque that turns it.
- **Drag**: the water's linear and angular drag, each weighted by the
  fraction of the shape's box under water.
- **Flow**: the current, weighted the same way. Over several shapes
  and volumes, the flows are averaged by their submerged fractions.

The step adds the buoyant force and torque to the body's velocity,
then pulls the velocity toward the flow and damps the spin, both
implicitly, so a strong drag settles a body on the current and never
pushes it past. The body's own damping and the speed caps apply after.

The model reads bounding boxes, not shapes: a sphere floats like its
box, a long body turned diagonally holds more water than its volume,
and a small body rides a little high, since its margin is a larger
part of its box. It is approximate on purpose, and deterministic like every
other result. Static and kinematic bodies feel nothing, and an
infinite plane never floats.

## What a soft particle feels

A soft body particle inside a volume, edges included, trades gravity
for buoyancy as if the particle had a density of 1000: in water of
density 1000 it hangs still, denser water lifts it and thinner water
lets it sink. Its velocity is pulled toward the flow by the volume's
linear drag, as a body's is. Where volumes overlap, the one created in
the lowest slot acts on the particle alone.

## Waking and determinism

Creating or destroying a volume wakes every dynamic body whose bounds
touch its box: a sleeper floats when the water arrives and falls when
it leaves. Creating and destroying volumes is recorded by the journal
and replayed like any other call, and a snapshot carries every volume.

The world hash takes in the volumes only while one is alive, so a
world that never had water hashes as before, while destroying a
volume and creating it again changes the hash.
