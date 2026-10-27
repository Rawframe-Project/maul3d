// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The snapshot walk: the one function that sizes, writes and reads the
// snapshot's byte stream after its header, and the measures taken with
// it. Below snapshot.c, so world create and the content creates can ask
// what a snapshot would take without the restore checks above.

#ifndef MAUL3D_SRC_SNAPSHOT_WALK_H
#define MAUL3D_SRC_SNAPSHOT_WALK_H

#include "world_internal.h"

typedef enum m3WalkMode
{
    m3_walkMeasure = 0,
    m3_walkWrite = 1,
    m3_walkRead = 2,
} m3WalkMode;

// A cursor over the snapshot byte stream. Measure only counts; write
// and read stream against the caller's buffer at the running offset.
typedef struct m3Walk
{
    uint8_t* out;
    const uint8_t* in;
    m3WalkMode mode;
    int64_t cursor;
} m3Walk;

// One slot's tail content: counts first, then what they describe. The
// read pass refuses hostile counts and sizes the slot through the alloc
// gate; measure and write only read the slot, and need no heap.
bool m3WalkMesh(m3Heap* heap, m3Walk* walk, m3MeshData* mesh);
bool m3WalkHull(m3Walk* walk, m3HullData* hull);
bool m3WalkHeightField(m3Heap* heap, m3Walk* walk, m3HeightFieldData* hf);

// The stream after the header: the state table's fixed prefix, then, with
// includeMesh, the mesh, hull and height field tails. Returns the byte
// total, or -1 when a read meets corrupt counts or runs out of memory.
int64_t m3WalkBlocks(m3World* world, uint8_t* out, const uint8_t* in, m3WalkMode mode,
                     int includeMesh);

// The walk for the readers: measure or write only.
int64_t m3WalkOut(const m3World* world, uint8_t* out, m3WalkMode mode, int includeMesh);

// The bound every snapshot is held to: M3_MAX_SNAPSHOT_BYTES. A test-only
// copy of the library lowers it (test/CMakeLists.txt), so a suite stands
// real worlds next to it without gigabytes of state.
#ifndef M3_SNAPSHOT_BOUND
#define M3_SNAPSHOT_BOUND M3_MAX_SNAPSHOT_BYTES
#endif

// The size of the world's snapshot in bytes, its own walk's measure:
// at most M3_SNAPSHOT_BOUND (the invariant in world_state.h).
int64_t m3SnapshotBytes(const m3World* world);

// Whether a snapshot of this many bytes is within M3_SNAPSHOT_BOUND.
bool m3SnapshotWithinBound(int64_t bytes);

// Whether a world of these capacities keeps its snapshot within the bound
// whatever mesh and hull content it holds: the header, the prefix, and
// every mesh and shape slot's tail at its largest. It reads the
// capacities alone, so m3CreateWorld asks before it allocates.
bool m3SnapshotFitsCapacities(const m3World* world);

// Whether the world's snapshot stays within the bound with an nx by nz
// height field grid's samples in one more slot, measured by the same walk.
bool m3SnapshotTakesHeightField(const m3World* world, int32_t nx, int32_t nz);

#endif // MAUL3D_SRC_SNAPSHOT_WALK_H
