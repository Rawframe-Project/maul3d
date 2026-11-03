# Destruction and voxels

Destructible scenery in Maul3D is made of voxel chunks: grids of
cells on static bodies that the host edits. When an edit cuts part of
a structure loose, the engine removes that part from the grid and
reports it as a fragment, which the host turns into a falling body.
`m3World_Explode` pushes bodies and carves chunks in one call. All of
it is ordinary world state: edits and explosions are recorded by the
journal, and grids are part of every snapshot and of the world hash,
so destruction rolls back and replays exactly.

## A chunk, cut through

```c
uint8_t voxels[4096] = {0}; // index x + 16 * (y + 16 * z)
for (int32_t z = 0; z < 4; ++z)
{
    for (int32_t y = 0; y < 8; ++y)
    {
        for (int32_t x = 0; x < 4; ++x)
        {
            voxels[x + 16 * (y + 16 * z)] = 1; // a 4 by 8 by 4 tower
        }
    }
}
m3ShapeDef chunkDef = m3DefaultShapeDef();
m3ShapeId chunk;
m3Result result = m3CreateVoxelChunkShape(world, ground, &chunkDef, voxels, NULL, 0.5f, &chunk);

// Cut the tower through at y = 2: everything above it comes loose.
const int32_t lo[3] = {0, 2, 0};
const int32_t hi[3] = {3, 2, 3};
int32_t cleared = 0;
if (result == m3_success)
{
    result = m3VoxelChunkShape_ClearBox(world, chunk, lo, hi, &cleared);
}
m3FragmentEvents fragments = m3World_GetFragmentEvents(world);
```

A chunk is 16 by 16 by 16 cells of `cellSize` metres, starting at the
shape's origin and running along its +x, +y and +z. Each cell
holds an occupancy byte (non-zero is solid), a 16-bit payload that is
the host's to define (a material, a health, a type), and a fill. The
body must be static, the shape cannot be a sensor, and the grid
cannot be empty. A world holds `voxelCapacity` chunks (4 by default).

## Edits

- `m3VoxelChunkShape_SetVoxel` fills a cell and sets its payload.
- `m3VoxelChunkShape_ClearVoxel` empties a cell.
- `m3VoxelChunkShape_ClearBox` empties every cell of an inclusive box
  and reports how many it emptied.
- `m3VoxelChunkShape_SetFill` sets a solid cell's fill, 1 to 255.

Coordinates are cell indices from 0 to 15; one out of range, or a box
whose upper corner is below its lower one, is refused with
`m3_errorInvalid`. An edit happens at once, inside the call, and
wakes every dynamic body whose bounds touch the edited cells. A
grounded character above them is re-checked in the same call, so a
floor carved away leaves it airborne at once.

Fill is mass, not shape: a cell collides as a full box whatever its
fill, but a fragment's mass counts it at fill / 255. A cell that is
emptied loses its fill; setting a fill of 0 is refused, since that is
a clear.

The collision surface of a chunk is built from its grid: solid cells
merge into the fewest boxes a fixed, deterministic pass finds, with a
tree over them. The host never sees it. A chunk's surface holds at
most `M3_MAX_VOXEL_SURFACE_BOXES` (2048) boxes, which a checkerboard
reaches. An edit whose surface would pass that is refused with
`m3_errorLimit`, changes nothing and is not recorded. It counts no
misuse, only `m3Counters.voxelEditsRefused`. The memory for the
largest surface is set aside when a chunk slot is first used, so no
edit asks the allocator.

A chunk edited down to nothing stays a valid shape that collides
with nothing.

## Fragments

A chunk stands on its bottom layer: a solid cell is anchored when a
path of solid cells, face to face, links it to a cell with y = 0. After
an edit that empties cells, the engine finds every group of solid cells
with no such path, removes each group from the grid, and reports one
fragment per group. The search runs over the whole chunk in a fixed
order, so the same edit always gives the same fragments in the same
order. A group that was never anchored, even at creation, comes loose
at the first emptying edit anywhere in the chunk. To build a floating
platform, place its chunk's body in the air with the platform on the
chunk's bottom layer.

`m3World_GetFragmentEvents` gives each fragment's chunk, its number of
cells, its mass at a density of 1 (the cell volume times the summed
fills over 255), its centre of mass in the chunk's frame and in the
world, its cell bounds, and a recipe: the list of its cells, as
`x + 16 * (y + 16 * z)`. The engine never creates a body for a
fragment; that is the host's choice. A common recipe gives a small
fragment a hull from its cells' corners and a large one a box from its
bounds, at a density that matches the reported mass.

The events last until the next step or restore. The event and recipe
buffers have fixed sizes, 256 fragments and 8192 recipe entries in
release 1.0.0. A fragment past the first is still removed but not
reported, and `fragmentsDropped` counts it; one past the second is
reported with a `recipeStart` of -1 and no recipe, its count and
bounds still set.

## Chunks side by side

Two chunks weld when both bodies have exactly the identity rotation,
they have the same cell size, and one body sits exactly one chunk
width from the other along a single axis. The faces they share then
count as inside, so a ball rolls across the seam without a bump.
Chunks that are rotated or off by any amount still collide, but do
not weld.

Every query sees chunks: contacts, rays, shape casts, overlaps and
continuous collision. A ray that starts inside solid cells misses
them, as with every shape. A body whose centre ends up inside the
solid, from a bad spawn or a huge impulse, is pushed out through the
nearest open face a little each step (in release 1.0.0, at most two
cells' depth), rather than flung out at once.

## Explosions

```c
m3ExplosionDef blast = m3DefaultExplosionDef();
blast.position = (m3Pos3){1.0, 3.0, 1.0};
blast.radius = 2.0f;          // full strength out to 2 m
blast.falloff = 3.0f;         // fading to nothing at 5 m
blast.impulsePerArea = 50.0f; // N s for each square metre facing the blast
blast.voxelCarve = 1.0f;      // bites a 1 m sphere out of chunks
m3World_Explode(world, &blast);
```

`m3World_Explode` acts at once, inside the call:

- **Bodies.** Every dynamic sphere, capsule and hull, boxes included,
  within `radius + falloff` of the blast takes an impulse at its
  point nearest the blast, directed away from it. The impulse is
  `impulsePerArea` times the area the shape shows the blast, at full
  strength within `radius` and fading linearly to nothing across
  `falloff`. A shape struck off centre spins. A shape with the blast
  inside it is pushed out through its centroid. Every body the blast
  reaches wakes, and its speed stays within the speed limit. A
  negative `impulsePerArea` pulls bodies in.
- **Chunks.** With `voxelCarve` above 0, the blast empties the cells
  within that radius of it in every chunk it reaches, and the usual
  fragments follow. A carve whose surface would pass the box limit
  leaves that chunk as it was and counts in
  `m3Counters.voxelCarvesDropped`; the impulses still land.
- **Soft bodies.** Each free particle in reach is kicked the same way,
  scaled by `softPush`, by the area of its radius. The kick lands on
  the next step; until then it is part of the world's state, in its
  snapshot and its hash.
- **Filter.** `filter` chooses which shapes the blast touches. Shapes
  on disabled bodies are never touched. Soft particles take no filter.

The defaults are a 10 m radius, a 5 m falloff, no impulse, no carve, a
soft push of 1 and an open filter. A def that `m3DefaultExplosionDef`
did not make, a position past 1e15 m, a negative radius, falloff or
carve, an impulse past 1e10, a soft push past 1e4, or any NaN changes
nothing, is not recorded and counts one misuse.

## A destruction cutscene

Because every edit and explosion goes through the journal, a recorded
session plays back as the same simulation, bit for bit: record with
`m3World_StartJournal`, then replay the tape or seal it into a replay
file ([the journal and replay files](journal-and-replays.md)). A
cutscene made this way is not a video; it is the game's own physics,
the same on every machine.
