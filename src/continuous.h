// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Continuous collision for fast bodies.

#ifndef MAUL3D_SRC_CONTINUOUS_H
#define MAUL3D_SRC_CONTINUOUS_H

#include "world_internal.h"

void m3SolveContinuousPhase(m3World* world, const m3Pos3* com0, const m3Quat* rot0);

#endif // MAUL3D_SRC_CONTINUOUS_H
