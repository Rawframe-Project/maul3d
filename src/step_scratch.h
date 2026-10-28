// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The size of a world's step scratch.

#ifndef MAUL3D_SRC_STEP_SCRATCH_H
#define MAUL3D_SRC_STEP_SCRATCH_H

#include "world_internal.h"

// The scratch a step takes at the world's capacities. The world holds
// that much from its creation, so a step never asks the allocator.
int64_t m3StepScratchBytes(const m3World* world);

#endif // MAUL3D_SRC_STEP_SCRATCH_H
