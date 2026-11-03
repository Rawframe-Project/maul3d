# Determinism and the step

## The determinism law

**Promised:** the same build of Maul3D, given the same sequence of
API calls with the same arguments, produces bit-identical simulation
state on every supported platform, compiler, architecture and
worker count, whatever rounding mode or flush-to-zero setting the
host's threads run in. This is CI-enforced across seven cells
(Linux gcc and clang, Debug and Release, sanitizers, macOS arm64,
Windows MSVC, and wasm32 under node)
whose printed state hashes must be equal on every commit.

**Deliberately not promised:**

- Hash stability across releases. A release may move the bits on
  purpose, with a solver revision or a format bump: a minor release
  for a new result, a patch release only to correct a result to what
  these pages promised. Its changelog entry says what changes.
- Determinism across DIFFERENT builds of the same version (another
  compiler's build is another timeline; each build is internally
  deterministic and all builds we ship cells for agree, but that
  agreement is a tested property of our flag discipline, not
  something to assume for arbitrary flags).
- Wall-clock time of anything.
- Id values under different creation orders. Ids are deterministic
  for a deterministic call sequence, nothing more.

**Host obligations:** feed the same calls in the same order with the
same values. Never derive a simulation input from wall time, pointer
values, hash-map iteration order, or thread scheduling. Use the
fixed-timestep loop you would use for rollback netcode anyway.

Fast-math is refused at configure time; the build owns its float
semantics (`-ffp-contract=off`, no unsafe optimizations, `/fp:precise
/fp:contract-` on MSVC).

The host's floating-point setting never reaches a result. Every call
that computes runs in the library's own setting (round to nearest,
subnormals kept, exceptions masked) and gives the thread back its
rounding mode and flush-to-zero and denormals-are-zero bits as it found
them. The allocator, the task hooks and the debug draw callbacks run in
the thread's own setting; a task range enters the library's on whatever
thread the host runs it; the pre-solve callback runs in the library's,
since its answer decides contacts.

## The step

`m3World_Step(world, dt, substepCount)` advances the world by dt seconds
in `substepCount` solver substeps (4 is the tested default) and returns
`m3_success`. A step never asks the allocator, so it never fails for
memory: the world takes the scratch a step needs when it is created
(see Capacities). A step that finds more broadphase pairs than the
world holds still runs whole and drops the pairs of the highest shape
indexes for that step; see Capacities. Commands
(create, destroy, set velocity) may be issued between steps in any
deterministic order; issuing them from inside task callbacks is not
supported.

The solver is the Soft Step scheme: soft constraints, speculative
contacts, warm starting carried in the persistent manifolds, a relax
pass, then restitution. Joints solve before contacts inside every
substep. Sleeping is per island: a dynamic body whose motion stays
under the sleep velocity (0.05) for half a second sleeps; a sleeping
island is bit-frozen (its state does not drift by even one bit) until
contact, joint change, command, or a waking neighbor returns it.

## The task interface

The world def takes `enqueueTask`, `finishTask`, and
`userTaskContext`. Both hooks or neither. The library NEVER spawns
threads; it only offers work to the host.

The contract: `enqueueTask(task, count, context)` is called with a
task function to run over a range; the host may run it on any
thread, split it, or run it inline. `finishTask` must not return
until that work is complete. Calls are paired and never nested. The
work the library hands out is data-parallel by construction (
narrowphase pairs, same-color contact batches of at least 16), so
the worker count and the split points change wall time and nothing
else. The four-gate CI proves one worker and four workers produce
identical bits.

Reentering the API from inside a task callback is not supported.

## The derived-data law

Anything the engine can recompute from content is NEVER state: not
snapshotted, not hashed, rebuilt wherever content lands (create,
journal replay, restore), and the rebuild must be a pure function so
twin worlds agree bit for bit. The per-mesh BVH is the standing
example; future acceleration caches follow the same law.
