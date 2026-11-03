# Soft bodies

A soft body is a set of particles held together by distance
constraints: a rope, a sheet of cloth, a block of jelly or a mesh of
tetrahedra. Its particles fall, collide with the world's shapes and
with other soft bodies, hang from pins and bodies, and blow in the
wind. A world holds `softBodyCapacity` of them (2 by default).

## A rope

```c
m3SoftBodyDef def = m3DefaultSoftBodyDef();
def.position = (m3Pos3){0.0, 5.0, 0.0}; // the lattice's lowest corner
def.countX = 10;                        // ten particles along x
def.countY = 1;
def.countZ = 1;
m3SoftBodyId rope;
m3Result result = m3CreateSoftBody(world, &def, &rope);
if (result == m3_success)
{
    result = m3SoftBody_PinParticle(world, rope, 0); // hang it by one end
}
```

`m3CreateSoftBody` lays out a lattice of `countX` by `countY` by
`countZ` particles, `spacing` apart, from `position` upward along each
axis: one row is a rope, one layer a sheet, more a block. Particle
`x + countX * (y + countY * z)` sits at lattice point (x, y, z). Each
particle is joined to its neighbours along the axes and across the
diagonals of each lattice face.

The defaults are a 4 by 4 by 4 lattice at 0.25 m spacing, particles of
0.1 kg and radius 0.05 m, rigid constraints, and gravity at full scale:

| Field | Meaning |
|---|---|
| `countX`, `countY`, `countZ` | Particles along each axis, each at least 1; at most `M3_MAX_SOFTBODY_PARTICLES` (512) in all. |
| `spacing` | The rest distance between neighbours. |
| `particleMass` | Each particle's mass, kg. |
| `compliance` | How far the constraints give under load; 0 holds them rigid. |
| `radius` | Each particle's collision radius. |
| `gravityScale` | The share of world gravity the particles feel. |
| `bendCompliance` | Above 0, adds a tether between every particle and its second neighbour along each axis, which resists folding; smaller is stiffer. 0 adds none. |
| `pressure` | Above 0, holds the lattice's volume at this multiple of its volume at creation: 1 keeps it, 2 inflates it toward double, at most 100. Every count must be at least 2. |
| `maxDeviation` | Above 0, keeps every particle within this distance of where it was created. |
| `userData` | The host's own value. |

A lattice may have at most `M3_MAX_SOFTBODY_EDGES` (5120)
constraints. The create refuses a bad field with `m3_errorInvalid`,
and a count, size, mass or position past its bound with
`m3_errorLimit`; the header lists each.

## Tetrahedral bodies

`m3CreateTetSoftBody` builds a soft body from points the host gives,
relative to `def->position`, and tetrahedra, four point indices each.
Its constraints are the tetrahedra's edges, and each tetrahedron also
keeps its volume at creation, so the body squashes and springs back
like rubber rather than collapsing. It takes 4 to 512 points and 1 to
`M3_MAX_SOFTBODY_TETS` (1024) tetrahedra. A tetrahedron that is flat,
inside out, names a point twice or names one out of range is refused
with `m3_errorInvalid`. The lattice fields do not apply: the counts and
spacing are not read, and `bendCompliance` and `pressure` must be 0.

## Pins and anchors

- `m3SoftBody_PinParticle` fixes a particle where it is.
- `m3SoftBody_AnchorParticle` attaches a particle to a body, at the
  particle's current position in the body's frame. The particle then
  moves with the body, and the pull of the rest of the soft body acts
  on a dynamic body at that point, so cloth hung from a swinging beam
  slows the beam. If the body is destroyed, the anchor lets go.
- `m3SoftBody_AnchorToSoftBody` joins a particle of one soft body to a
  particle of another. It lets go when either soft body is destroyed.

A soft body holds at most `M3_MAX_SOFTBODY_ANCHORS` (32) anchors, its
anchors to bodies and its joins to other soft bodies together; a join
counts against the soft body in the lower slot. An anchor past a full
table is refused with `m3_errorLimit` and counts no misuse.
`m3DestroySoftBody` removes the soft body with its anchors and joins.

`m3SoftBody_GetParticleCount` and `m3SoftBody_GetParticlePosition`
read the particles back.

## What a step does

Each substep, for each soft body in slot order, the step:

1. moves every free particle by its velocity, gravity, any pending
   explosion kick, the wind and the water, capped at the world's speed
   limit;
2. satisfies each constraint once, in a fixed order, then the
   `maxDeviation` limit, the tetrahedra's volumes and the pressure;
3. moves each anchored particle to its anchor, passing its pull to the
   body;
4. pushes each particle out of the shapes it overlaps.

Then it separates touching particles of different soft bodies and
satisfies the joins between them. Every loop runs in a fixed order,
so a soft body is as deterministic as a rigid one.

A particle collides with every shape in the world except sensors and
shapes on disabled bodies: planes, spheres, capsules, boxes, hulls,
meshes, height fields and voxel chunks. Collision filters do not apply
to particles. Against a shape, a particle takes that shape's friction.
A particle pushed out of a dynamic body pushes the body back, so a
block of jelly has weight on a crate. Two particles of different soft
bodies collide at the sum of their radii, with a fixed friction (0.5
in release 1.0.0).
Particles of the same soft body never collide with each other: its
constraints hold it apart.

## Wind

`m3World_SetWind` sets a wind that blows soft body particles only; push
rigid bodies with forces. Each particle is drawn toward the wind's
velocity at a fixed rate (0.5 per second in release 1.0.0). The wind's speed swings by
`gustScale` of itself at `gustHertz`, and the swing's phase is part of
the world's state: a snapshot carries it, so a rollback resumes the
same gust. A speed of 0 turns the wind off.

## Things to know

- A sheet of particles is a grid of spheres, so a rigid body about the
  size of the gaps between them can push through it when the sheet
  stretches. Use more particles or a larger radius, or put a thin box
  behind the sheet, where it must hold.
- Positions carry no orientation, so a rope resists bending, with
  `bendCompliance`, but not twisting.
- A perfectly symmetric setup stays symmetric: a rope standing exactly
  on its end balances there, because nothing in a deterministic step
  breaks the symmetry. Tilt it a little in a real scene.
- Creating, destroying, pinning and anchoring soft bodies, and setting
  the wind, are journal operations. Every particle's current and
  previous position, which give its velocity, the constraints, the
  anchors and the wind are part of a snapshot and of the world hash.
