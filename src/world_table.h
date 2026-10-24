// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The table of live worlds behind m3WorldId: lookups by id and by the
// world tag object ids carry, and the slot bookkeeping of create and
// destroy.

#ifndef MAUL3D_SRC_WORLD_TABLE_H
#define MAUL3D_SRC_WORLD_TABLE_H

#include "world_internal.h"

// The first free slot, or -1 when the table is full.
int32_t m3WorldTableFreeSlot(void);

// The generation the next world in slot gets.
uint16_t m3WorldTableGeneration(int32_t slot);

// Registers world in slot.
void m3WorldTableSet(int32_t slot, m3World* world);

// Frees slot and bumps its generation, so ids of the world it held go
// stale.
void m3WorldTableRelease(int32_t slot);

// Registry lookup: NULL for a stale or null id.
m3World* m3WorldFromId(m3WorldId worldId);

// The world an object id names (its world field: slot and generation),
// or NULL when that world is gone.
m3World* m3WorldFromTag(uint16_t tag);

#endif
