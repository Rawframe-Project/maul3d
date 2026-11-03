# Destruction and voxels

## Voxel chunks

A voxel chunk is a shape on a STATIC body: a dense 16x16x16 grid of
cells anchored at the body origin, each cell carrying an occupancy
bit, a uint16 payload (material, health, type: yours to define),
and a fill fraction (255 a whole voxel, 1 a sliver). All of it is
state: hashed, snapshotted, journaled, inside the rollback delta.

The collision surface is derived: a deterministic greedy merge
produces maximal boxes and a BVH over them; you never see or manage
it. Chunks placed with bit-exact identity rotations, equal cell
sizes, and world positions exactly one chunk extent apart WELD:
their shared border becomes interior geometry, and bodies roll
across it without seam impulses. Rotated or misaligned chunks
still collide; they just do not weld.

Edits are commands: set a voxel (with payload), clear a voxel,
clear a box region, set a fill fraction. Every edit is journaled
and replayed like any other mutation; the surface follows the grid
as a pure function; dynamic bodies touching the edited region wake.
No edit asks the allocator. A chunk's surface holds at most
`M3_MAX_VOXEL_SURFACE_BOXES` (2048) merged boxes, the checkerboard's
count, and a voxel slot reserves a
tree that size (about 84 KB) when its first chunk is created, so a
world pays only for the slots it fills; the world keeps one 76 KB build
scratch. An edit, an explosion's carve or the fracture it causes always
leaves a tree that answers. An edit or a carve whose surface would pass
the bound changes nothing and journals nothing, and counts no misuse.
The edit returns `m3_errorLimit` and counts in
m3Counters.voxelEditsRefused. The carve counts in
m3Counters.voxelCarvesDropped, and the explosion's impulses still land. Each edit
returns an `m3Result`, and `m3VoxelChunkShape_ClearBox` writes the
count it cleared to its `clearedOut`. A create or a snapshot chunk past the bound is
refused.
Fill is mass, never geometry: a worn voxel collides at full size
but its fragment weighs fill over 255. A chunk edited to empty
stays a valid shape that collides with nothing.

Everything sees chunks: contacts, rays (front faces, born-inside
misses), shape casts, continuous collision (a one-voxel wall stops
a bullet; a hole cleared last step is a real hole), point-inside
(solid voxels are CLOSED volumes), and overlaps. A body whose
center ends up inside the solid walks out through the nearest
exposed face at a bounded pace instead of exploding.

## Fracture

The anchor convention: an island of voxels is anchored if and only
if it has a path (six-connectivity) to the chunk's y = 0 base
layer. Build structures at their chunk's base; a floating platform
is a chunk whose BODY sits in the air. After every
occupancy-clearing edit, a deterministic flood fill removes every
unanchored island from the grid AS PART OF THE SAME STATE
TRANSITION (rollback and replay re-derive identical grids) and
emits one fragment event per island, in canonical order.

The event carries the chunk id, voxel count, mass (fill-weighted,
at density one), the center of mass in chunk and world frames, the
island bounds, and a recipe: chunk-local voxel indices
(v = x + 16 * (y + 16 * z)) in a transient buffer. The engine
NEVER spawns bodies; what a fragment becomes is yours. The
usual recipe feeds small islands' voxel corners to the
built-in QuickHull and gives large ones their bounds box, with
density matched to the event mass (see the voxfort scene in
bench/).

Fragment events and recipes are transient like every stream: the
next step clears them, restore clears them. When one edit strands
more islands than the event capacity (256), every island is STILL
removed from the grid; only the reporting has a ceiling, and the
dropped count says so loudly.

## The destruction cinematic

Record a storm, replay it as a cutscene: arm the testbed's
Record button (or m3World_StartJournal in your host), detonate
the blastyard, Save to .m3j, and play it back with the m3replay
studio (verify replays to the recorded hash; seek scrubs to any
step through keyframes). A cutscene in Maul3D is not a video: it
is the same simulation, bit for bit, every time. The lockstep
proof (tools/m3lockstep, run by CI) is the two-machine
version of the same law: an online peer that predicts and rolls
back lands on the offline peer's exact timeline.

## Explosions

m3World_Explode is one journaled call that does a whole
demolition. Fill an m3ExplosionDef (from m3DefaultExplosionDef,
cookie included): position, radius, falloff, impulsePerArea,
optionally voxelCarve and softPush, optionally a query filter.
Every dynamic sphere, capsule, and hull in range takes an
impulse scaled by the AREA it shows to the blast (its projected
area), directed from the center through the
closest point on the shape, which means off-center hits spin
their targets through the real lever arm. Past the radius the
impulse fades linearly to zero across the falloff band. The
blast wakes everything it reaches; a negative impulsePerArea
implodes.

voxelCarve > 0 bites a sphere of that radius out of every voxel
chunk in range through one surface rebuild and one fracture
sweep per chunk; freed islands arrive as the usual fragment
events for the host to spawn (the testbed kicks its newborn
fragments radially for the look; the engine keeps events and
bodies separate on purpose). softPush scales the same falloff
push onto soft-body particles: the kick is applied exactly once
by the next step, is real snapshot state in the window between
the call and that step, and folds into the hash only while it
exists.

Hostile defs (NaN anywhere, negative radius, dead cookie) apply
nothing and journal nothing; hostile tape bytes fail the replay
loudly. Explosions are events, not state: two worlds that
explode identically stay bit-identical through twins, replay,
and rollback.
