# The character controller

## The character controller

A character (`m3CreateCharacter`) is a kinematic capsule the
engine owns, driven only by `m3Character_Move`: a journaled,
deterministic, rollback-covered command. The body has zero
velocity by contract and never moves during `m3World_Step` except
to ride its floor (below). Gravity is the host's job: add it to
the move each tick, netcode style.

- Collide and slide: the move casts the capsule, advances to the
  first hit keeping `skin`, slides the remainder, up to four
  times. Steep faces (steeper than `maxSlopeAngle`) are walls and
  never mint upward motion out of horizontal intent.
- Stairs: a grounded walker blocked by a steep face lifts up to
  `stepHeight`, advances, and lands. The landing is accepted only
  when a ray probing half a radius AHEAD of the landing axis finds
  walkable floor and the net rise fits one step height (the ray,
  not the capsule normal, is the floor classifier: capsule casts
  read stair corners as steep, and an axis ray can be fooled by
  low floor behind a ramp's crease).
- Grounding: walkable contact grounds the character and records
  the body under its feet (`m3Character_GetGroundBody`). A
  descending character glues to floor within `snapDistance`; no
  floor in reach means airborne.
- Riders: each step ends by carrying grounded characters along
  their ground body's rigid motion, THROUGH the slide casts, so a
  platform can ferry a walker into a wall and the wall wins.
  Kinematic platforms, elevators, carousels, and dynamic bodies
  (fragments) all ferry riders.
- Pushing: a move blocked by a dynamic body applies impulse =
  `mass` * blocked displacement at the contact. Per tick that
  displacement is the walker's velocity in tick units, so the
  momentum flux comes out mass * speed with no dt in the contract.
  Walkable floors are standing, not pushing; bodies heavier than
  `pushMaxMassRatio` * `mass` are walls.
- Destruction interplay: carving the floor from under a character
  flips it airborne the SAME step (the voxel edit runs a grounding
  refresh over overlapping characters).
- Hostile moves (non-finite, or any component beyond the caster's
  1e18 float budget) are documented no-ops that never journal.

## The rigid character recipe

The kinematic controller is the engine path; when you want a
character with real mass that trades momentum, build the
rigid variant ON the public API (the suite ships this exact loop
as its proof): a dynamic capsule with angular locks (bits 3..5)
so it never tumbles; FRICTIONLESS by recipe, because an
externally-forced capsule loses the tug of war against static
friction and stands still while you push; grounding read from
m3World_CollideMover's plane list, skipping your own shape via
m3Shape_GetBody (a downward ray finds your own hull first); drive
and brake with one force toward the target velocity, clamped to
an actuator budget. The mover toolkit (m3World_CastMover,
m3World_CollideMover, m3SolveMover, m3ClipMoverVelocity) covers the rest of a custom
mover's needs as pure queries.

## Stance

m3Character_SetStance resizes the capsule in place, FEET
ANCHORED: the center moves so the capsule bottom stays level,
which also means a mid-air crouch lands shorter, never higher.
Shrinking always applies (removing volume cannot create
contact). Growing runs the stand-up veto: the grown capsule is
cast at its new pose through the same core every move uses, and
any contact within one skin (a pressing ceiling, a wall against
a wider radius) refuses the whole change and returns false with
stance untouched. The veto is bit-deterministic: twins and
replays grant and refuse in the same pattern.

Only APPLIED changes journal; a vetoed stand writes nothing.
Stance rides existing state (the capsule shape, the body
transform, the character config), so snapshots, hashes, and
rollback cover it with no new machinery. Read it back with
m3Character_GetStance.
