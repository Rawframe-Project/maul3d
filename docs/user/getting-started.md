# Getting started

This is the contract between the engine and the host, engineer to
engineer. The README says what Maul3D is; these pages say exactly
what it promises, what it deliberately does not promise, and the
rules a host must follow to keep the promises alive. Everything here
is enforced by a test that runs on every commit; where a rule has a
number, the number is the one in the code.

## The object model

A **world** owns everything: bodies, shapes, joints, contacts,
events. Worlds are isolated; nothing is shared between two worlds,
and ids from one world never operate on another.

A **body** is a frame with mass and velocity (static, kinematic, or
dynamic). A **shape** attaches geometry to a body: sphere, box,
capsule, convex hull (built by the internal QuickHull), static
triangle mesh, heightfield chunk, or infinite plane (static bodies
only). A **joint** constrains two bodies: spherical (optional swing
cone and twist limits), revolute (limits, motor), prismatic (limits,
motor).

Everything is created from a **def** struct obtained from its
`m3Default*Def()` function. Defs carry a private cookie; a def that
was not built from the default function is refused. Every create
writes an **id** to its last argument. Every refusal changes nothing,
and a call that can fail returns its status. The library never aborts
on user input: bad defs, stale ids, exhausted pools, and hostile
floats are refused through return values
(assertions guard internal invariants only, states that cannot be
reached from outside).

### Id lifecycle

Ids are generation-tagged: `{index1, generation}`, a 1-based slot and
the generation the slot had when the object was made. A slot's
generation bumps when the object is destroyed, so a stale id can
never alias a recycled slot. The rule for staleness is uniform: a
stale id counts one misuse, a getter returns zeros (the identity for a
rotation), a command changes nothing, and a call with a status, a
destroy or a create given a stale body included, returns
`m3_errorStale`.

A slot names objects under generations 0 to 2^32 - 1, then retires
instead of wrapping, so a pool of capacity n makes n × 2^32 objects
over a world's life. With 1000 slots at 1000 creates a second that is
about 136 years; one slot at 1000 a second lasts about 50 days. A pool
whose slots have all retired refuses with `m3_errorCapacity`, loudly.
Every pool follows the rule: bodies, shapes, joints, characters,
vehicles, soft bodies and water volumes, and the internal hull, mesh,
height field and voxel slots that shape creates take, so those
creates can run out the same way. The budget is part of the world's
state: a snapshot carries every slot's generation and the retired
slots, and a restore brings back the snapshot's budget, never a fresh
one.

One id is valid until its object or its world is destroyed. An id
means something only to the world that issued it: given to another
world, it names whatever that world holds in the same slot with the
same generation. Nothing crashes either way.

Destroying a body cascades: its shapes, its joints and a vehicle
whose chassis it is go with it. Destroying an already destroyed id
returns `m3_errorStale` and counts one misuse, as any stale id does.

## Units and conventions

SI units: meters, kilograms, seconds, radians. Gravity defaults to
(0, -10, 0). Right-handed coordinates. Quaternions are xyzw, unit
length, and a body def's rotation must be within 1 percent of unit
length (a corrupted quaternion is refused, not renormalized).
Positions are double precision; everything else is single. Mesh
triangles wind counter-clockwise seen from outside; backfaces do not
collide. Planes are half-spaces: `dot(normal, p) <= offset` is
solid, with p in the body's frame as for every shape. Capsules are
a segment plus a radius; a zero-length segment is refused (use a
sphere).

## Scale

The engine's scale posture is measured, not promised: the bench
binary's cityblock scene (nine welded voxel tower stacks, five
thousand mixed bodies, a mid-run fracture wave) and the scale
gauntlet behind it are the record; the scale suite proves the same
contracts at CI size with a real four-thread host pool.

- Worker counts are performance knobs, never state inputs: serial
  and threaded runs of the same ops land on identical bits, at
  every population.
- Rollback at scale is priced, not feared: at five thousand bodies
  a full snapshot runs tens of milliseconds and tens of megabytes,
  restore the same, both bit-exact (the gauntlet prints the
  numbers on your machine).
- The step scratch is taken whole when the world is created, at
  its capacities, so no step allocates or stalls for memory.
- Sleep is the sandbox's economy. Give ground-contact debris a
  small rollingResistance (0.01 to 0.1): a frictionless-rolling
  sphere never stops on its own, and one eternal roller keeps its
  whole contact island awake.
- Capacities are hard books, not hints: worlds create up to
  exactly their capacity and refuse the next, at 256 and at 8192
  alike. Size the def for the scene, plus fragments.
- Large levels tile as chunks (voxel chunks, mesh chunks,
  heightfield chunks) under one static body each; welding makes
  grid-aligned voxel seams contact-seamless.

## The integration checklist

What an engine evaluator wires first, and where Maul3D answers:

1. Create a world, step it, read transforms: `m3CreateWorld`,
   `m3World_Step`, body move events (no polling).
2. Spawn and destroy at runtime: create ops are journaled with id
   verification; destroys cascade shapes, joints, and anchors.
3. Layers and masks: collision filters, filtered queries.
4. Apply gameplay forces: the force/impulse family.
5. Teleports, character platforms, elevators: SetTransform, the
   kinematic servo, motion locks.
6. Tune materials live: the material setters and world knobs.
7. Contact callbacks: begin/end events, opt-in hit events, the
   pre-solve veto with its purity contract.
8. Motorized doors, servos, breakable structures: joint runtime
   control, drives, and breakage with the break stream.
9. Queries: rays, casts, overlaps, all filtered, plus the
   character controller, vehicles, voxels, and soft bodies from
   earlier arcs.
10. Determinism: every one of the above is journaled, snapshot
    round-trips bit-exactly, and CI enforces cross-platform
    equality on every commit. That last line is the moat.
