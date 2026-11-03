# Snapshots and rollback

A snapshot is the whole simulation in a byte buffer. Restoring it puts
a world back exactly where it was, and the world then steps to the same
bits it would have reached anyway. Rollback netcode, instant replays
and save states are built on that.

## Taking and restoring a snapshot

```c
int64_t size = m3World_GetSnapshotSize(world);
uint8_t* buffer = malloc((size_t)size);
int64_t written = 0;
m3Result result = m3World_Snapshot(world, buffer, size, &written);
```

The world steps on; when a correction arrives, it goes back:

```c
if (result == m3_success)
{
    result = m3World_Restore(world, buffer, written);
}
free(buffer);
```

`m3World_GetSnapshotSize` gives the exact size of a world's snapshot
now. It grows with the world's content, meshes and hulls above all, so
measure it before each snapshot rather than once per session. A buffer
smaller than that is refused with `m3_errorCapacity`; any failed
snapshot leaves `written` at 0 and zeroes the buffer, so no part of it
reads as a snapshot.

Restore works in place, into a world with the same capacities as the
one that wrote the snapshot. Worlds of different capacities are
different shapes of state, and restore refuses them.

## What a snapshot holds

Everything that decides what the next step does: every body, shape,
joint, character, vehicle, soft body, water volume and voxel chunk;
the hulls and meshes their shapes use; the id pools with their free
lists and generations, so an id taken before a snapshot names the same
object after a restore; the contact pairs with their stored impulses;
the broad-phase tree; sleep state; the world's settings; the wind's
phase; the step counter.

Three things are left out on purpose:

- **The worker count.** A snapshot taken while stepping on eight
  threads restores into a world stepping on one, and the two go on to
  the same bits. How many threads step a world is not its state.
- **Derived data.** What follows from content alone, such as each
  mesh's tree and each body's mass, centre and inertia, is rebuilt on
  restore by the same functions that built it.
- **Events.** Contact, sensor, hit and fragment events describe the
  last step. A restore clears them.

## What restore refuses

A snapshot is read by the releases of the minor version that wrote it,
built with the same settings. Its header carries the writer's
`M3_ABI_VERSION`, the format number, and a hash of the solver revision,
the float sizes and the floating-point policy. Another minor version, a
build of other settings, or a release that changed a result is
refused with `m3_errorVersion`. A world of other capacities, a
truncated buffer or a damaged one is refused with `m3_errorFormat`.

Restore treats the bytes as untrusted. Before it changes anything it
checks every block: indices, counts, flags and finite values; then the
structures as wholes, the id pools, the lists that tie bodies to
shapes and joints, the broad-phase tree, the contact pairs, the shared
hulls and meshes with their reference counts, and every character,
vehicle, soft body and joint record. Any value past the limit its
create or setter keeps is refused too ([the magnitude
limits](limits.md#magnitude-limits)). A snapshot the engine could not
have written is refused with `m3_errorFormat`, and the world is left
exactly as it was. A refusal of the bytes counts no misuse; a world
that accepts a snapshot steps, queries and snapshots like one built
through the API.

A snapshot is a save state for rollback and replay, not an archive.
There is no migration between versions: keep the game's own state, not
snapshots, across an upgrade.

## Size

Snapshot sizes are 64-bit and have no limit of their own: a world
that fits in memory can be snapshotted. Most of a snapshot is set by the
world's capacities: each shape slot costs about 1.9 KB, each voxel slot
13 KB and each soft body slot 100 KB, whether used or not. Content adds
to that: up to about 1.3 MB for a mesh at the full 65535 triangles,
5.8 KB for a hull of 64 vertices and 260 KB for a native height field
at the full 255 by 255 samples.

On a 32-bit target, `m3World_Restore`, `m3World_StartJournal`,
`m3World_ReplayJournal` and the replay container calls refuse a size
past `SIZE_MAX` with `m3_errorInvalid`, before they form a pointer.

## One layout everywhere

A snapshot, like a journal tape, is little-endian, with every value
laid out as on x86-64, eight-byte values eight-byte aligned. The build
pins the size and member offsets of every type a snapshot carries, so
a compiler that would lay one out differently fails the build rather
than write another format. A snapshot written on any supported target
restores on any other. [The state formats](../internals/state-formats.md)
describe the bytes.

## Rollback networking

The bit-exact promise makes the classic rollback recipe work exactly
as written:

1. Send only inputs, never state. Each peer sends its inputs for each
   frame.
2. Keep a snapshot of the last frame whose inputs have all arrived.
3. Step on past it, guessing the inputs that have not arrived; an
   empty input is the simplest guess.
4. When a late input arrives, restore that snapshot, apply the inputs
   now known, and step forward to the present again.
5. Compare `m3World_Hash` between peers at agreed frames. Equal hashes
   prove the peers agree; a difference shows the frame where they
   parted, and `m3replay diff` names the bodies that carried it.

`tools/m3lockstep` runs this recipe as a program: an online peer that
predicts and rolls back, against an offline peer that steps only on
inputs it has, over a link that delays each input by one to four
frames at 60 Hz. Over 300 frames it rolls back about 300 times, up to
four frames deep, and checks every 60 frames that both peers hold the
same hash. CI runs it on every change. Input delay, the rollback
window and how to guess inputs are the host's choices; the engine's
part is that the same inputs give the same bits.
