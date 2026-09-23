# Samples

Each sample in [`samples/`](../samples) is a complete program that uses
only the public headers and prints something you can check. They are
built with the library (turn them off with `-DMAUL3D_BUILD_SAMPLES=OFF`):

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/sample_stack
```

## A falling stack, hashed

[`samples/stack.c`](../samples/stack.c) drops ten boxes on a plane and
prints the world hash after five seconds. Run it on two machines, on
two architectures: the hash is the same.

## Snapshot, diverge, restore

[`samples/rollback.c`](../samples/rollback.c) is the rollback loop. It
snapshots a world, simulates a mispredicted future, restores the
snapshot and simulates the corrected one. The restore is total: every
body, joint, vehicle, soft body, contact and sleep timer returns to the
bit, and ids stay valid.

## A raycast car and a tank

[`samples/car.c`](../samples/car.c) builds a four-wheel raycast car,
drives it for two seconds, brakes, and then pivots the same chassis in
place with tank commands. Commands are journaled state, so a recorded
drive replays bit-exact. `m3Vehicle_SetDrivetrain` adds gears on top of
the flat drive force.

## A character on the stairs

[`samples/character.c`](../samples/character.c) walks a capsule
character up five risers. The host applies gravity itself, as a
networked game would; the controller collides, slides and steps up.

## The installed package

[`samples/minimal/`](../samples/minimal) is a separate CMake project
that finds the installed package with `find_package(maul3d)` and
nothing else. CI installs the library and builds it.
