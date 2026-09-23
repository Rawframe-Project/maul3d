// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Islands and sleep.

#ifndef MAUL3D_SRC_ISLAND_H
#define MAUL3D_SRC_ISLAND_H

#include "world_internal.h"

int32_t* m3IslandWakePass(m3World* world);
void m3IslandSleepPass(m3World* world, int32_t* parent, const m3Pos3* com0, const m3Quat* rot0,
                       float dt);

#endif // MAUL3D_SRC_ISLAND_H
