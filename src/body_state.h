// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body lookups and wake state, shared by every part that names or wakes
// a body.

#ifndef MAUL3D_SRC_BODY_STATE_H
#define MAUL3D_SRC_BODY_STATE_H

#include "world_internal.h"

// Slot lookup with generation check: -1 for a stale or foreign id.
int32_t m3BodySlot(const m3World* world, m3BodyId bodyId);

// Resolves a body id to its slot; for a stale id, refuses and returns
// false with a -1 index.
bool m3ResolveBody(const m3World* world, m3BodyId body, int32_t* indexOut);

void m3SetAwakeInternal(m3World* world, int32_t index, int awake);

void m3WakeRegionAabb(m3World* world, const double lo[3], const double hi[3]);

// bodyLocks bit 6: this body bypasses the angular speed cap (fast
// rotation allowed, for wheels). Bits 0..5 stay the
// motion locks; the byte already snapshots and hashes off-default
// as one block, so the flag rides for free.
#define M3_LOCKS_ALLOW_FAST_ROTATION 0x40u

// Wakes a dynamic body; static and kinematic bodies never sleep.
static inline void m3WakeIfDynamic(m3World* world, int32_t body)
{
    if (world->bodies.types[body] == (uint8_t)m3_dynamicBody)
    {
        world->bodies.awake[body] = 1;
        world->bodies.sleepTimes[body] = 0.0f;
    }
}

#endif
