# The journal and replay files

## The journal law

The journal records every MUTATION as a discrete op: creates (with
their full recipes: hull input points, mesh vertex and index
payloads), destroys, velocity commands, and steps (dt and substep
count). Getters and queries are not ops.

`m3World_StartJournal(world, buffer, capacity)` arms recording into a
caller-owned buffer and writes the tape's 16-byte state prefix at its
start, so the buffer holds at least those 16 bytes; a session of no
op is just the prefix. `m3World_StopJournal` returns the bytes written,
or -1 if the buffer overflowed (an overflowed journal records
nothing more and can never be mistaken for a good one).
`m3World_ReplayJournal(world, data, size)` applies a recorded session
to a world and verifies that every minted id matches the recorded
one; a world that starts from the same state WILL mint the same ids,
so a mismatch means the session does not belong to this starting
state. A def a create records is copied member by member, so the tape
never holds what the caller's def held in its padding: the same calls
write the same bytes.

**Replay is atomic.** On any refusal the target world is restored to
its pre-call state. A half-applied session is impossible. A damaged
tape (truncation, corruption, an op that cannot re-mint its recorded
id) is refused with m3_errorFormat, a tape another minor version
wrote with m3_errorVersion, and a replay the world's allocator
turns down, before the first op or in the middle of one, with
m3_errorCapacity. Backing out never needs memory, because the replay
takes it up front: for the whole call it holds a snapshot of the world
(m3World_GetSnapshotSize bytes) and a second copy of the world's mesh and
height field content and mesh trees (about contentBytes of
m3World_GetMemoryUsage) and a few arrays sized by the mesh, shape and
voxel capacities. Budget that headroom on a tight allocator: without it the
replay is refused with m3_errorCapacity before any op runs.

**The composition rule:** a journal must close BEFORE a rollback
block. A restore is not an op; a journal spanning one would replay a
longer history than the world lived. Record, End, then snapshot and
restore freely; arm a new journal after.

## The replay studio

A replay file is [header | initial snapshot | journal], encoded
and decoded by pure-memory calls in maul3d/replay.h; file IO
belongs to hosts and tools (tools/m3replay is the model
consumer). The header carries step and op counts and the
recorder's final hash, so verification is self-contained: restore
the snapshot, replay the journal, compare hashes. A journal cut
at record boundaries, behind the tape's 16-byte prefix, is itself
a valid journal; the scrubber in m3replay keyframes every 30
steps and re-steps to seek anywhere, proving each landing against
a straight-run prefix hash. When two sessions disagree, m3replay
diff finds the first divergent step by prefix-hash binary search
and m3World_Compare names the worst bodies at it. Corrupt
containers refuse loudly: the codec validates the container's,
the snapshot's and the tape's headers, exact lengths, record
framing, and that the header's counts match its own stream. Like
the snapshot and the tape it holds, a container is read only by
the releases of the minor version that wrote it; another minor
version's is refused with m3_errorVersion.
