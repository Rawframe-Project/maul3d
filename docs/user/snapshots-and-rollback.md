# Snapshots and rollback

## The snapshot law

`m3World_GetSnapshotSize / m3World_Snapshot / m3World_Restore` save and
load the COMPLETE simulation state, bit for bit, in a portable,
versioned, field-by-field format. Never a raw memory image.

In the snapshot: pool identities (including free lists, generations,
and retirements), body state, shape state, interned hull and mesh
content, joint state including warm-start impulses, persistent
contact manifolds and their impulses, broadphase tree, sleep state,
step counter.

Deliberately NOT in the snapshot:

- **Worker count.** A snapshot taken with eight workers restores
  with one and continues bit-identically; parallelism is not state.
- **Derived data.** Acceleration structures that are pure functions
  of content (the per-mesh BVH) are rebuilt on restore and proven
  byte-identical by test. So is each body's mass, center, inertia and
  extents: no call sets them directly, so they are a function of the
  body's type and shapes, and restore rebuilds them with the function
  every change to those calls.
- **Event streams.** Events are transient observations; restore
  clears every stream (contact, sensor, and fragment alike).

A snapshot restores only into a release of the minor version that
wrote it, built with the same semantics, and into a world with the
same capacities: the header's state prefix guards the version, a
config hash the build. Another minor version's or another build's
snapshot is refused with
m3_errorVersion; truncated buffers, corrupted headers, and
wrong-shaped worlds with m3_errorFormat; each with the target world
untouched. So is any snapshot this engine could not
have written: broken links, lists, pools or trees, and any value past
the limit its create or setter keeps (see [the magnitude limits](limits.md#magnitude-limits)). A
world restore accepts steps, queries and snapshots like one the API
built.

**A snapshot's size is 64-bit.** Snapshot, journal and replay
container sizes are `int64_t` and have no cap of their own: a world that
fits memory snapshots. Most of a snapshot is sized by the capacities:
about 1.9 KB per shape slot, 13 KB per voxel slot and 100 KB per soft
body slot. Mesh, hull and height field content adds to it: up to about
1.3 MB per mesh, 5.8 KB per hull and 260 KB per native grid height
field. `m3World_GetSnapshotSize` gives a world's exact size. On a 32-bit
target, where `size_t` is 32 bits, `m3World_Restore`,
`m3World_StartJournal`, `m3World_ReplayJournal` and the container calls
refuse a size past `SIZE_MAX` with `m3_errorInvalid` before they form a
pointer, and a snapshot that does not fit memory fails its allocation
with `m3_errorCapacity`.

The header starts with the 16-byte state prefix maul2d shares, which
carries the writer's ABI version and the format's own number
([the state formats](../internals/state-formats.md)). There is no
migration; a snapshot is a save-state for rollback and replay, not an
archival format, and a host keeps assets, not snapshots, across an
upgrade.

**One layout on every target.** A snapshot and a journal tape are
little-endian, and every type they carry is laid out as on x86-64,
with each double and 64-bit integer 8-byte aligned (32-bit x86
aligns them to 4 unless told). The build pins each record's and each
snapshotted type's size and member offsets, so a target that would lay
one out otherwise fails to compile rather than write another format. A
tape or snapshot written on one target loads on any other.

## Lockstep and rollback networking

The engine's bit contract makes the classic rollback recipe safe
to build exactly as the books describe it, and tools/m3lockstep
is that recipe as a runnable document. The shape:

1. Inputs are the only truth on the wire. Send each player's
   inputs per frame; never send state.
2. Keep a snapshot of the last frame whose inputs are ALL
   confirmed (m3World_Snapshot: one buffer is enough).
3. Predict forward past the frontier with guessed remote inputs
   (empty is the simplest guess).
4. When a late input arrives, restore the frontier snapshot,
   re-apply confirmed inputs, re-predict to the present
   (m3World_Restore + your step loop; bit-exactness makes the
   repair invisible).
5. Hash at agreed checkpoints (m3World_Hash). Equal hashes ARE
   the sync proof; a mismatch is a desync alarm on the exact
   frame it was born, and the divergence finder (m3replay diff)
   tells you which body carried it.

The sample runs an online peer against an offline truth peer
through a jittered 1..4 frame wire at 60 Hz, absorbs hundreds of
rollbacks, and demands checkpoint equality in CI on every commit.
Input delay, rollback windows, and per-peer prediction policies
are host decisions; the engine's promise is only, and exactly,
that the same inputs produce the same bits.
