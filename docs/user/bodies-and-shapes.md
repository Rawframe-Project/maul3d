# Bodies and shapes

## Forces and impulses

`m3Body_ApplyForce`, `ApplyTorque`, and `ApplyForceAtPoint`
accumulate for exactly one step: integrated beside gravity every
substep, consumed after the last. `ApplyLinearImpulse`,
`ApplyAngularImpulse`, and `ApplyImpulseAtPoint` act immediately.
At-point arms are measured from the center of mass. Pending
accumulators are snapshot state: a snapshot taken between an
application and its step re-lands the same trajectory. Targets
must be dynamic with mass; anything nonzero wakes the body.

## Runtime body control

`m3Body_SetTransform` teleports and wakes both the departure and
arrival neighborhoods. `m3Body_SetTargetTransform` is a kinematic
servo: the body lands exactly on the target next step and the
order clears; the exit velocity stays yours by contract. A target
past `M3_TARGET_REACH_LIMIT` from the body returns `m3_errorLimit`.
`m3Body_SetType` flips dynamic/kinematic/static with a full mass
rebuild, and returns `m3_errorLimit` when the body's mass, joints or
speed do not allow the new type. `m3Body_Enable` with false removes the body from
contacts, CCD, rays, casts, overlaps, and soft-lattice collision.
`m3Body_SetMotionLocks` freezes any subset of the six axes (bits
0..2 linear xyz, 3..5 angular xyz), re-zeroed every substep so
contacts cannot bank motion on a frozen axis.
`m3Body_SetSleepThreshold` gives a per-body threshold and
`m3Body_EnableSleep` keeps a body awake; `m3Body_SetAwake` forces
either edge.

## Materials and world tuning

Shape materials are runtime-settable and journaled:
`m3Shape_SetFriction`, `SetRestitution`, `SetRollingResistance`,
and `SetDensity` (the latter rebuilds the body's mass).
Contacts read materials at prepare, so changes bind next step; a
sleeping stack keeps its old mix until something wakes it. World
knobs: `m3World_SetGravity` (sleepers stay asleep until
disturbed), `SetContactTuning` (hertz, damping ratio, max
depenetration speed; static contacts run twice the hertz),
`SetRestitutionThreshold`, `SetMaximumLinearSpeed` (a hard
per-substep clamp), `EnableSleep` (off wakes everyone), and
`EnableContinuous`. Knobs are STATE, not config: they journal,
they snapshot, and they hash only off-default. The config hash
stays version + solver revision + precision + FP policy, because
a config-hash knob would refuse the journal instead of replaying
it.

Painted mesh materials: m3MeshShape_SetMaterials gives a
mesh shape up to eight surface materials and one group byte per
triangle. The struck triangle's entry replaces the MESH side of
the contact mix (friction, restitution, rolling resistance), and
its surface velocity feeds the conveyor row per triangle: a
moving walkway, an ice patch, and tarmac can share one level
mesh. One welded manifold wears one material (the first,
id-canonical point picks it, documented v1 contract). Unpainted
meshes keep using their shape material everywhere, bit-exactly
as before. Journaled, snapshotted, hostile paint refused loudly.

The broadphase rebuild: m3World_RebuildBroadphase replaces
the incrementally grown tree with a balanced top-down build over
the live shapes, in one deterministic, journaled pass. Bulk level
construction inserts one shape at a time and can leave the tree
lopsided; call this once after building and queries walk a
balanced tree. It changes NO answer: pairs are canonical and the
tree is never hashed, so a rebuilt world steps bit-identically to
its unrebuilt twin (the suite proves it by hash equality). It
returns m3_errorCapacity, with the tree unchanged and nothing
journaled, when the world's allocator refuses its scratch.

## Compound shapes and the geometry ceilings

Every shape may carry a local transform relative to its body:
localPosition and localRotation in m3ShapeDef (identity by
default, near-unit rotations demanded loudly). One composed read
serves the whole engine, so an offset shape collides, casts,
rays, draws, and weighs exactly as it sits. Mass properties
compose per shape before the parallel-axis walk: a dumbbell built
from two offset boxes spins by the composite books. Caster sweeps
take explicit poses and CCD composes at t0 (documented v1 bound).
m3DestroyShape removes one shape at runtime and rebuilds the
owner's mass; the last shape leaves a unit-mass shapeless body.

Ceilings: hulls hold up to 64 vertices (124 faces, 372 half
edges, input clouds up to 256 points); mesh chunks hold up to
65,535 triangles and 65,535 vertices with 16-bit indices, stored
count-derived so empty capacity costs nothing; heightfields
triangulate through the same mesh path. One triangle or vertex
past a ceiling refuses at create, loudly.

## The faceted cylinder, reshaping, and the overlap family

m3CreateCylinderShape mints a cylinder as a 2N-vertex prism
through the interned hull path (N = segments, clamped to 3..32
so 2N fits the 64-vertex hull law). Everything downstream (mass,
contacts, casts, CCD, carving, the blast's projected area)
treats the prism exactly; the documented trade is that the side
is an N-gon, not a curve, and 24 segments roll visually smooth
at meter scales.

m3Shape_SetSphere and m3Shape_SetCapsule swap a shape's geometry
in place, conversions between the two included (journaled, op
69). Hulls, meshes, voxel chunks, and planes refuse: interned
slabs are immutable by law. A swap recomputes mass from the new
volume, wakes everything around BOTH the old and the new bounds
(a shrink frees what leaned on the old silhouette), and the
broadphase picks the new box up the same step machinery that
tracks every mover.

The overlap family joins the sphere: m3World_OverlapCapsule (a
center and two center-relative points, as the capsule cast takes
them), an oriented m3World_OverlapBox, and m3World_OverlapHull (a
base plus up to 64 base-relative points and an optional radius,
the cast convention). All answer EXACTLY per family: convex
candidates through the GJK core, planes by minimum point
distance, voxel chunks and meshes by BVH gather and per-box or
per-triangle distance. Ascending shape ids, query filters, and
the observer purity law throughout.

## The native heightfield

m3CreateHeightFieldShape stores an nx-by-nz grid (2..255 per
axis) as raw samples, four bytes each: the low-memory terrain path
beside meshes (the mesh-backed m3CreateHeightFieldMeshShape remains
for small chunks). Cells split into two triangles on coordinate
parity; contacts run the same welded triangle pipeline as meshes
over a 16-cell contact window with a halo (bodies wider than the
window contact within it; window warm-starts reset when it shifts
a cell; both deterministic and documented). Rays scan the
segment's cell span; the overlap family and soft particles see
terrain through the same cell gather. Shape casts and continuous
collision try only the cells within reach of their path, row by
row, so a long diagonal costs its length rather than its box;
meshes and voxel chunks gather the BVH leaves near the path the
same way. A ray exactly along a
shared triangle corner can miss by one float ulp, the same
contract as the mesh path. Static bodies only; journaled (op 76),
snapshotted count-derived (v49), never world-hashed (content is
create-derived, like meshes and hulls).

## Collision filters

Every shape carries `categoryBits` (default 1), `maskBits`
(default all), and `groupIndex` (default 0). Two shapes meet only
when each one's category intersects the other's mask, both ways; a
shared positive group forces collision, a shared negative group
forbids it. One chokepoint gates ordinary pairs, sensors, and the
continuous phase alike. Queries behave like shapes: every ray,
cast, and overlap takes an `m3QueryFilter`, and
`m3DefaultQueryFilter()` sees everything. Filters are state and
hash only off-default.

## Continuous collision

Every fast dynamic body sweeps against static geometry, so nothing
tunnels through a floor. A body flagged `enableBullet` additionally
sweeps against dynamic and kinematic targets, with both bodies'
sweeps in the time-of-impact kernel. Bullet versus bullet is not
resolved (a documented limitation).
Sensors neither stop nor are stopped.

## The angular speed cap

The linear speed cap has an angular twin: every substep,
any dynamic body spinning past the world cap is scaled back onto
it. The default (800 rad/s) is a catastrophe guard in the same
philosophy as the 400 m/s linear default: far above anything a
legal scene does, it exists to stop solver-explosion artifacts,
not gameplay. Hosts that want a tight clamp set a
low cap with m3World_SetMaximumAngularSpeed (journaled, hashed
only off-default) and flag their legal fast spinners, wheels
above all, with m3Body_EnableFastRotation (journaled). The
flag lives beside the motion locks, survives lock writes, and
reads back with m3Body_IsFastRotationEnabled.
