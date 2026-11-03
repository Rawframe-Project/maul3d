# Limits and capacities

- [Magnitude limits](#magnitude-limits)
- [Capacities](#capacities)

## Magnitude limits

Every value the API accepts is finite and within a limit, so that
every product the step forms stays in float range. The limits are
public constants in `maul3d/base.h`, beside the soft body, vehicle and
drivetrain counts in their headers. A call past a limit refuses with
`m3_errorLimit` (a create returns the null id), counts one misuse
unless the world's state decides it (see [which refusals count
misuse](queries-and-events.md#introspection)), journals
nothing and changes nothing; replay refuses the same values in
a recorded op. A NaN is invalid rather than past a limit, and returns
`m3_errorInvalid`; an infinity is past every limit. Each limit sits far
past what real scenes use. Vectors are bounded per component.

| Quantity | Limit | Where |
|---|---|---|
| Position | `M3_POSITION_LIMIT`, 1e15 m | body defs, poses, anchors in the world, a plane's offset, water boxes, every soft particle a create places |
| Body-frame offset or extent | `M3_LOCAL_LIMIT`, 1e4 m | shape geometry, joint anchors, wheel anchors and lengths, a servo's aim, soft spacing and particle radius, a character's radius and half height |
| Kinematic target | `M3_TARGET_REACH_LIMIT`, 1e6 m from the kinematic body | `m3Body_SetTargetTransform` |
| Static geometry | `M3_GEOMETRY_LIMIT`, 1e8 m in its body's frame | mesh vertices, height samples |
| Cell size | `M3_CELL_LIMIT`, 1e5 m, positive | height field grids and mesh height fields, voxel chunks |
| Cast or character move | `M3_CAST_LIMIT`, 1e18 m | shape casts, `m3Character_Move` |
| Speed | `M3_SPEED_LIMIT`, 1e5 m/s and rad/s | dynamic body velocities, every body's spin, the world's speed caps, contact push speed, wind speed, water flow, surface velocity, joint motor speed |
| Kinematic speed | `M3_KINEMATIC_SPEED_LIMIT`, 1e10 m/s | kinematic body linear velocities |
| Impulse | the velocity it leaves within `M3_SPEED_LIMIT` | `m3Body_ApplyLinearImpulse`, `m3Body_ApplyAngularImpulse`, `m3Body_ApplyLinearImpulseAtPoint` |
| Force and torque | `M3_FORCE_LIMIT`, 1e20 N and N m, held or added up over a step | `m3Body_ApplyForce`, `m3Body_ApplyTorque`, `m3Body_ApplyForceAtPoint`, joint motor and steering effort, a servo's budgets, vehicle drive and brake force, engine torque |
| Acceleration | `M3_ACCEL_LIMIT`, 1e4 m/s^2 | gravity |
| Frequency | `M3_HERTZ_LIMIT`, 1e5 Hz | contact hertz, joint springs and steering, wind gusts, wheel suspension |
| Ratio | `M3_RATIO_LIMIT`, 1e4 | damping ratios, restitution, gravity scales, gear and pulley ratios, gear and final drive ratios, brake share, tire grip, lean gain, wind gust scale, a character's push ratio, an explosion's soft push |
| Differential coupling | `M3_FORCE_LIMIT / M3_SPEED_LIMIT`, 1e15 N per m/s | `m3DrivetrainDef.differentialCoupling` |
| Step length | `M3_MIN_STEP` to `M3_STEP_LIMIT`, 1e-4 to 10 s | `m3World_Step` |
| Substeps | `M3_MAX_SUBSTEPS`, 256 | `m3World_Step` |
| Density | `M3_DENSITY_LIMIT`, 1e6 kg/m^3 | shapes, water |
| Mass | a shape at least `M3_MIN_MASS`, 1e-6 kg, with every principal moment at least `M3_MIN_INERTIA`, 1e-12 kg m^2; a body at most `M3_MASS_LIMIT`, 1e20 kg | shapes on dynamic bodies, a body turning dynamic, characters, soft particles |
| Wheel radius | at least `M3_MIN_WHEEL_RADIUS`, 1e-3 m | vehicle wheels |
| Character stance | `M3_MIN_STANCE` to `M3_STANCE_LIMIT`, 0.05 to 10 m | `m3Character_SetStance` |
| Explosion impulse | `M3_EXPLOSION_IMPULSE_LIMIT`, 1e10 N s/m^2 | `m3ExplosionDef.impulsePerArea` |
| Joint lever | `M3_JOINT_LEVER_LIMIT`, 1e10 for mass * arm^2 / moment: see below | a spherical, revolute, prismatic, weld, wheel or motor joint, a shape create, destroy, resize or density change on its bodies, a body's type change |
| Contact lever | `M3_CONTACT_LEVER_LIMIT`, 1e6 for mass * arm^2 / moment: see below | a shape create, destroy, resize, density or filter change on a dynamic body, a body turning dynamic |
| Counts | `M3_MAX_HULL_POINTS` 256, `M3_MAX_CAST_POINTS` 64, `M3_MAX_MESH_VERTICES` and `M3_MAX_MESH_TRIANGLES` 65535, `M3_MAX_MESH_MATERIALS` 8, `M3_MAX_HEIGHTFIELD_SIZE` 255, `M3_MAX_HEIGHTFIELD_MESH_SIZE` 32, `M3_MAX_VOXEL_SURFACE_BOXES` 2048, `M3_MAX_BODY_NAME_BYTES` 31 | hull clouds, casts, meshes and their materials, height field grids and mesh height fields, a voxel chunk's surface, a body's name |
| Capacities | `M3_MAX_SHAPE_CAPACITY`, `M3_MAX_PAIR_CAPACITY` and `M3_MAX_VOXEL_CAPACITY` | `m3WorldDef` |

The step holds what it produces within the same limits: every
body leaves a substep within 1e5 m/s and 1e5 rad/s, a bounce asks
for at most 1e5 m/s of separation, and a push the step itself lands
on a body (a character's, a soft particle's, a wheel's, an
explosion's) leaves it within 1e5 as well. A character's, a soft
particle's or a wheel's push whose float sum overflows is taken again
in double, and m3Counters.heldPushRetakeCount counts it. A stored contact
or joint impulse is held, direction kept, so its warm start changes a
body's velocity or spin by at most 1e13 per component at that step's
own masses and levers.

One limit spans two bodies. Take a joint's bodies, the arm from each
one's center of mass to its anchor, and the moment about the axis that
arm turns it: the joint's lever ratio is their mass times the squared
arm over that moment, summed as the turn each gives over the inverse
masses together, so a static or kinematic body adds nothing. A joint
that solves a point block or a pair of held rows, the spherical,
revolute, prismatic, weld, wheel and motor joints, keeps it within
1e10. Past about 5e11 those blocks, solved one after another, undo
each other, and the joint lets its body go or flings it, though each
value is within the limits above: a 1.8e4 m rod 0.5 mm thick at
density 1e6 on a 1e4 m arm reaches about 2.5e15. A joint create past
the ratio is refused, and so is any change that would carry a joint
there: a shape created on, destroyed from, resized on or given a new
density on one of its bodies, or a body's type change. Each returns
m3_errorLimit, changes nothing and counts no misuse, since the
bodies' masses decide it, not the argument. Restore and replay refuse
it too. The ratio does not depend on
how the bodies turn, so a joint within it stays within it as they
move. The distance, generic, parallel, gear, pulley and filter joints
solve single rows and keep no such limit.

A dynamic body keeps the same ratio for its contacts, over its own mass:
its mass times the squared arm from its center of mass to any point of
a shape that can collide (not a sensor, and with a nonzero category and
mask or a positive group), over the moment that arm turns it, within
1e6. Contacts are solved in float lanes, one constraint after another,
so a contact far out on a body that a joint or another contact also
holds must be undone by it at every pass: past about 1e7 the two no
longer settle, and the contact lets its shape sink through the ground.
A core of density 1e6 and radius 1 with a colliding shell of radius 1
reaches 1e6 near 630 m. A shape create, destroy, resize or density
change, a filter change that lets a shape collide, and a body turning
dynamic are refused past it, each with m3_errorLimit, changing nothing
and counting no misuse; restore and replay refuse it too. Shapes that
collide with nothing, and static and kinematic bodies, keep no such
limit.

Restore refuses the same values past the same limits. Creates and
teleports place every body and soft particle within 1e15 m, and the
step holds it there, direction kept: a body reaches the bound only by
moving at the kinematic limit for about 1e5 s, or at the speed limit
for about 1e10 s.

Replay and restore take a shape's geometry only in the form a
create leaves it: a field its type does not use is zero, and a
plane's normal is unit length.

## Capacities

Every pool is fixed at world creation (`bodyCapacity`,
`shapeCapacity`, `meshCapacity`, `jointCapacity`, `voxelCapacity`,
`characterCapacity`, `vehicleCapacity`, `softBodyCapacity`) and never
grows. A create past a full pool is refused with `m3_errorCapacity`
and the null id, counts no misuse, and the world keeps stepping.

The broadphase pair table is fixed too: `pairCapacity` pairs, 8 per
shape of `shapeCapacity` when it is left 0. A dense pile can find
more pairs than that. The step then keeps the pairs of the lowest
shape indexes, so twins, replays and threaded runs keep the same
ones, and drops the rest for that step: the two shapes of a dropped
pair get no contact and pass through each other. The step still
runs whole for every other body. `m3Counters.pairsDropped` counts
the drops, so check it while a game is in development and raise
`pairCapacity` until it stays at zero:

    uint64_t dropped = m3World_GetCounters(world).pairsDropped;
    (void)m3World_Step(world, dt, 4);
    if (m3World_GetCounters(world).pairsDropped != dropped)
    {
        // Bodies passed through each other this step: give the world
        // a larger m3WorldDef.pairCapacity.
    }

The world also takes its step scratch at creation, sized for a step
at those capacities, so stepping never allocates.
`m3World_GetMemoryUsage` gives the exact sizes (`scratchCapacity`,
`persistentBytes`), and a world whose scratch would pass 2 GiB, or that
the allocator refuses, is not created (`m3_errorCapacity`).

As a guide for budgets, release 1.0.0 on a 64-bit target, measured
with `m3World_GetMemoryUsage`, takes about

    918 * pairCapacity + 126 * bodyCapacity + 416 * jointCapacity
    + 57 KiB

bytes of scratch, and each pair also keeps 564 bytes of persistent
tables, so the default 8 pairs cost about 11.6 KiB per shape. The
default world (1024 bodies, 2048 shapes and 16,384 pairs, 64 joints)
takes 14.5 MiB of scratch beside 21.7 MiB of pools and arrays. A later
release may change these figures.

Each shape's own ceilings, hulls of 64 vertices from up to 256 input
points, meshes of 65,535 vertices and 65,535 triangles, and height
fields of up to 255 by 255 samples, are described with the shapes in
[bodies and shapes](bodies-and-shapes.md).
