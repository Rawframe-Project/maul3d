# Glossary

**Anchor.** A soft body particle held to a rigid body (`m3SoftBody_AnchorParticle`) or to another soft body's particle (`m3SoftBody_AnchorToSoftBody`). A pinned particle is held where it is.

**Body.** A rigid object with a pose and a velocity: static, kinematic or dynamic. Its shapes give it collision and mass. Positions are 64-bit (`m3Pos3`); everything relative to a body is 32-bit.

**Bullet.** A dynamic body swept continuously against the shapes it can touch, so it cannot tunnel through thin walls.

**Character.** A capsule the world moves with `m3Character_Move`, which slides along surfaces, climbs steps and rides the body it stands on.

**Chunk.** A voxel chunk shape: a dense 16 by 16 by 16 grid of cells, each with a fill and a payload, whose collision surface is merged boxes.

**Cookie.** The value in a def's `internalValue` that its default function sets. A create refuses a def without it, which catches a def that was never initialized; a joint def's cookie also names its type.

**Def.** The struct a create takes, initialized by its `m3Default...Def` function.

**Fragment.** A piece of a voxel chunk that lost its path to the chunk's base layer. The edit that cut it removes it from the grid and reports it in the fragment events; the world creates no body for it, so what a fragment becomes is the host's choice.

**Golden value.** A hash in `test/hashes.txt` that pins a promised output; see [determinism.md](determinism.md).

**Graph coloring.** The planning of contacts into groups whose members touch no common body, so a group solves in SIMD lanes and in parallel without changing the result.

**Host code.** Code of the program that the world calls: the task hooks, the pre-solve callback, the debug draw callbacks and the allocator. It may read the world but not change it.

**Id.** A small value naming an object of a world: a 1-based slot and a generation. Stale when its object is gone.

**Island.** A set of bodies connected by touching contacts and joints. An island sleeps and wakes as one.

**Journal.** The record of the calls made to a world, as a tape in caller memory; see [state-formats.md](state-formats.md).

**Lever ratio.** A body's mass times the squared arm from its center of mass to a joint anchor or a colliding shape, over the moment that arm turns it. Creates and setters refuse a change that would carry it past `M3_JOINT_LEVER_LIMIT` or `M3_CONTACT_LEVER_LIMIT`, where the solver stops settling.

**Manifold.** The contact points, normal and separations between two touching shapes.

**Misuse.** A call a correct program never makes: an invalid argument, a stale id, a writer called from host code. Counted in `m3Counters.misuseCount`. Untrusted bytes, full pools and refusals the world's state decides are not misuse.

**Mover.** The character kit under the character controller: `m3World_CastMover`, `m3World_CollideMover`, `m3SolveMover` and `m3ClipMoverVelocity`, for a capsule the game moves itself.

**Pair.** Two shapes whose fattened boxes overlap, kept in a table sorted by key. A pair touches when its manifold has points.

**Reader and writer.** A reader takes a const world and may run with other readers; a writer changes the world and needs it alone.

**Replay container.** A snapshot, the tape recorded from it and the final hash, in one block of bytes (`m3EncodeReplay`, `m3DecodeReplay`).

**Sensor.** A shape that reports overlaps and never pushes back. Two sensors never sense each other.

**Snapshot.** The whole simulation state as one flat block of bytes.

**Soft body.** A lattice or a tetrahedral mesh of particles held by distance constraints (XPBD), solved after the rigid step against the rigid world.

**Soft step.** The solver's substep schedule: a biased pass with soft constraints that pushes error out, integration, and an unbiased relax pass that removes the velocity the push added.

**State prefix.** The 16 bytes every state format starts with, shared with maul2d: the magic, the writer's `M3_ABI_VERSION`, the format's number and the header's size.

**Substep.** One of the solver's subdivisions of a step.

**Tape.** A journal's bytes: the 16-byte state prefix and one record per call.

**Vehicle.** A chassis body with raycast wheels, a suspension spring per wheel and an optional drivetrain.

**Warm start.** Starting a constraint's solve from the impulse it ended the last step with, kept per contact point and joint row.
