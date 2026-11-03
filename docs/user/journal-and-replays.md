# The journal and replay files

The journal records a session: every call that changes a world, in
order, as a tape of bytes. Replaying the tape into a world that starts
from the same state makes the same calls and ends on the same bits.
A replay file packs the starting snapshot, the tape and the final
hash together, so anyone can check a recorded session on their own
machine.

## Recording and replaying

```c
uint8_t tape[4096];
m3Result result = m3World_StartJournal(world, tape, sizeof tape);
for (int32_t i = 0; i < 60; ++i)
{
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
}
int64_t tapeBytes = 0;
if (result == m3_success)
{
    result = m3World_StopJournal(world, &tapeBytes);
}
```

Then, in a world built the same way:

```c
if (result == m3_success)
{
    result = m3World_ReplayJournal(twin, tape, tapeBytes);
}
```

`m3World_StartJournal` starts recording into a buffer the host owns
and writes the tape's 16-byte header there, so the buffer must hold at
least 16 bytes. While a journal runs, every call that changes the
world is appended to the tape: creates with everything they need to
create again (a hull's input points, a mesh's vertices and indices),
destroys, setters, commands, edits, explosions and steps. Getters and
queries change nothing and are not recorded. A call the world refuses
is not recorded either.

`m3World_StopJournal` ends the recording and writes the tape's
length to `bytesOut`. If the tape outgrew the buffer, it returns
`m3_errorCapacity`: the journal stopped writing at the overflow, so a
cut tape can never pass for a whole one. Starting a journal while one
runs, or stopping one when none runs, is refused with
`m3_errorState`.

A tape holds each def field by field, never the padding between
fields, and is laid out the same on every target, so the same calls
write the same bytes on any machine.

## What replay checks

`m3World_ReplayJournal` makes each recorded call again and checks that
each create gets the id it got when it was recorded. A world that
starts from the same state hands out the same ids, so a mismatch means
the tape does not belong to that starting state.

Replay is all or nothing. On any refusal, the world goes back to the
state it had before the call: a half-replayed session cannot happen.
A damaged tape, whether cut short, corrupted or holding a create that
does not get its recorded id, is refused with `m3_errorFormat`. A tape
written by another minor version is refused with `m3_errorVersion`.
Neither counts misuse.

To back out without needing memory at the worst moment, the replay
takes what it needs before the first call: a snapshot of the world, a
second copy of the world's mesh and height field content and mesh
trees (about `contentBytes` of `m3World_GetMemoryUsage`), and a few
arrays sized by the mesh, shape and voxel capacities. With an
allocator too tight for that, the replay is refused with
`m3_errorCapacity` before any call runs.

## Journals and rollback

A restore is not recorded, and neither is a replay. A journal that
runs across a restore therefore records a history longer than the one
the world lived. Stop the journal before restoring or replaying, and
start a new one afterwards.

## Replay files

`replay.h` packs a session into one buffer and reads it back. The
library does no file input or output; the host writes and reads the
bytes.

- `m3GetEncodedReplaySize` gives the size of a container for a
  snapshot and a tape of given sizes.
- `m3EncodeReplay` writes a container from the starting snapshot, the
  tape and the recorder's final hash. It checks both first, and
  refuses one that is damaged or was written by another minor version.
- `m3DecodeReplay` checks a container and gives a view of its parts:
  the snapshot, the tape, their sizes and the final hash. It checks the
  container's own header, the snapshot's and the tape's headers, the
  exact lengths, the tape's record framing, and that the header's
  counts match the tape.
- `m3DescribeJournal` counts a tape's records and the steps among
  them without replaying it.

To verify a recorded session: decode it, restore its snapshot into a
world of the same capacities, replay its tape, and compare
`m3World_Hash` with the final hash. A container is read only by the
releases of the minor version that wrote it.

## Finding a divergence

When two runs that should agree do not, `m3World_Compare` lists every
body slot that differs between two worlds, worst first, by its
position, rotation and velocity error, including bodies alive in only
one of them.

`tools/m3replay` builds on these calls: `record` writes a replay file
of a built-in demo, `info` describes one, `verify` checks one, `seek` and `play` scrub
through one, and `diff` finds the first step where two sessions part
and the bodies that differ there. Seeking restores a snapshot the tool
keeps every 30 steps and steps forward from it. A tape cut at a record
boundary is still a whole tape, so a prefix of a session replays on
its own.
