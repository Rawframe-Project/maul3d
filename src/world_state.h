// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's state table: every array the world owns or snapshots,
// with its element size, its length and whether it is snapshot state.
// Allocation, release, the snapshot walk and the memory footprint all
// come from the one table in world_state.c, so a new array is one struct
// field and one table row.
//
// A world's snapshot is the header, the fixed prefix this table walks,
// and the variable tail of mesh, hull and height field content. Its size
// has no cap of its own: every size is computed in 64 bits, from the one
// per-array size the walk copies, and each array stays within INT32_MAX
// bytes (m3AllocArray), so a world that fits memory fits its snapshot.

#ifndef MAUL3D_SRC_WORLD_STATE_H
#define MAUL3D_SRC_WORLD_STATE_H

#include "world_internal.h"

// Allocates every owned array, sized from the capacities already set in
// the world, and adds them to its memory footprint. Returns false when
// an allocation is refused; m3StateFree releases whatever was allocated.
bool m3StateAllocate(m3World* world);
void m3StateFree(m3World* world);

// The bytes of the fixed snapshot prefix, from the capacities already set
// in the world alone, so a def is measured before anything is allocated.
int64_t m3StateSnapshotBytes(const m3World* world);

// Walks the snapshot state in byte order: direction 0 copies into out,
// 1 copies from in, and any other value only measures. Returns the
// number of bytes, m3StateSnapshotBytes' count.
int64_t m3StateWalk(m3World* world, uint8_t* out, const uint8_t* in, int direction);

// Where the snapshot prefix holds a field (its offset in m3World): the
// byte position of its block, or -1 when the field is not snapshot state.
// Restore reads incoming state through it before any of it lands.
int64_t m3StateFieldAt(const m3World* world, size_t fieldOffset);

// Checks the incoming fixed snapshot prefix against the table before any
// of it lands: every index in range, every flag a flag, every count
// within its bound, every float finite.
bool m3StateValidate(const m3World* world, const uint8_t* in);

#endif // MAUL3D_SRC_WORLD_STATE_H
