# Determinism and the step

The same calls, with the same arguments, in the same order, give the
same bits: on every supported platform and compiler, on x64 and arm64,
with any number of worker threads, whatever floating-point mode the
host's threads run in. Rollback, lockstep multiplayer, replays and
server checks all rest on that.

## Comparing two worlds

```c
uint64_t hashA = m3World_Hash(worldA);
uint64_t hashB = m3World_Hash(worldB);
m3BodyDiff rows[8];
int32_t differing = 0;
m3Result result = m3World_Compare(worldA, worldB, rows, 8, &differing);
```

`m3World_Hash` folds a world's whole simulation state into 64 bits: the
step count, gravity and any setting off its default, every id pool,
and every body, shape, voxel grid, character, soft body, vehicle,
joint, water volume and persistent contact. What follows from content
alone and what only observes is left out: trees, events, counters and
the profile. Equal hashes mean equal worlds. When two worlds should
agree and do not, `m3World_Compare` lists the body slots that differ,
worst first, by position, rotation and velocity.

## What is promised, and what is not

Promised: one build of Maul3D, given the same sequence of calls with
the same values, reaches the same state bit for bit. CI checks it on
every change across twelve cells: GCC 14 on x64 and arm64 Linux,
Clang 20 on x64 Linux, both on 32-bit x86 Linux, clang-cl on x64 and
arm64 Windows, Clang on arm64 macOS, a shared library, Debug builds
under AddressSanitizer on the scalar backend and under
ThreadSanitizer, and WebAssembly. Every cell must print the hashes
committed in `test/hashes.txt`.

Not promised:

- **The same bits across releases.** A release may change a result on
  purpose: a minor release for a new result, a patch release only to
  correct a result to what these pages promise. Its changelog entry
  says what changes.
- **The same bits from builds with other compiler flags.** The builds
  CI covers agree because the build fixes its floating-point flags; a
  build with flags of its own is a timeline of its own.
- **Wall-clock time.** `m3Profile` reports how long a step took, and
  that differs from run to run.
- **Particular id values.** Ids are deterministic for a deterministic
  sequence of calls, nothing more.

The host's part: make the same calls in the same order with the same
values. Never feed the simulation wall time, pointer values, hash-map
order or thread scheduling. Step at a fixed time step.

## Floating point

The build refuses fast math: a `-ffast-math`, `/fp:fast` or `-Ofast`
in the flags stops the configure step. It compiles with floating-point
contraction off, so no compiler fuses a multiply and an add on one
target and not on another. The AVX2, NEON and scalar kernels do the
same operations in the same order, and a test holds them to the same
bits.

Every call that computes runs in the library's own floating-point
setting, whatever the calling thread's is: round to nearest,
subnormals kept, every exception masked. It hands the thread back its
rounding mode and its flush-to-zero and denormals-are-zero bits as it
found them. The host code a world calls, the allocator, the task hooks
and the debug draw callbacks, runs in the thread's own setting. A task
range sets the library's setting on whichever thread runs it, and the
pre-solve callback runs in the library's, since its answer decides
contacts.

## The step

`m3World_Step(world, dt, substeps)` advances the world by `dt`
seconds, from 1e-4 to 10, in 1 to 256 substeps, and returns
`m3_success`. A step never asks the allocator, so it cannot fail for
lack of memory: the world sets aside what a step needs when it is
created, sized by its capacities. A step that finds more broad-phase
pairs than `pairCapacity` still runs whole, keeping the pairs of the
lowest shape indices for that step ([limits](limits.md#capacities)).

Each step runs, in order: the vehicles' wheels; the broad and narrow
phases, which update the contact pairs and their begin and end events;
the solver, whose substeps each integrate velocities, warm start the
joints and contacts from the impulses they kept, solve joints before
contacts, integrate positions and relax; restitution; continuous
collision for fast bodies; sleep; the move events; the characters
riding their floors; and last the soft bodies, in substeps of their
own.

A dynamic body sleeps when its speed stays below its sleep threshold,
0.05 m/s by default, for half a second, and all bodies of its island
sleep together. A sleeping island does not change by a single bit
until a contact, a joint change, a command or a waking neighbour wakes
it. `enableSleep` in the world def turns sleep off.

Calls that change a world are made between steps. A call made from
host code the world is running during a step, a task, a callback or
an allocator, is refused with `m3_errorBusy`.

## Threads

The library never starts a thread. A host that wants a step spread
over its own threads passes two hooks in the world def, both or
neither:

- `enqueueTask(task, itemCount, minRange, taskContext, userContext)`
  hands the host a function to run over the items `[0, itemCount)`.
  The host splits that range into pieces of at least `minRange` items,
  runs each piece exactly once on any thread, in any order, and
  returns a handle.
- `finishTask(handle, userContext)` must not return until all of that
  work is done.

Each piece writes only its own slots, so any split on any number of
threads gives the same bits, and CI compares one worker with four on
every change. A piece can run the narrow phase, whose deepest call
keeps under 48 KiB of stack, so give the threads that run pieces at
least 64 KiB.

Separate worlds share nothing: two worlds can step on two threads at
once and land on the same bits as stepped one after the other. Between
steps, any number of threads may read one world at once through its
queries, getters and snapshot; reading a world while it steps is
undefined.

## Derived data

What can be recomputed from content is never state: a mesh's tree, a
body's mass and inertia, a voxel chunk's merged boxes. It is not in a
snapshot or the hash, and it is rebuilt wherever content arrives, at
create, replay and restore, by functions that give the same bits every
time, so two worlds built the same way agree on it.
