// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's state table: every array the world owns or snapshots,
// with its element size, its length and whether it is snapshot state.
// Allocation, release, the snapshot walk and the memory footprint all
// come from the one table in world_state.c, so a new array is one struct
// field and one table row.

#ifndef MAUL3D_SRC_WORLD_STATE_H
#define MAUL3D_SRC_WORLD_STATE_H

#include "world_internal.h"

// Allocates every owned array, sized from the capacities already set in
// the world, and adds them to its memory footprint. Returns false when
// an allocation is refused; m3StateFree releases whatever was allocated.
bool m3StateAllocate(m3World* world);
void m3StateFree(m3World* world);

// Walks the snapshot state in byte order: direction 0 copies into out,
// 1 copies from in, and any other value only measures. Returns the
// number of bytes.
int32_t m3StateWalk(m3World* world, uint8_t* out, const uint8_t* in, int direction);

// Checks the incoming fixed snapshot prefix against the table before any
// of it lands: every index in range, every flag a flag, every count
// within its bound, every float finite.
bool m3StateValidate(const m3World* world, const uint8_t* in);

#endif // MAUL3D_SRC_WORLD_STATE_H
