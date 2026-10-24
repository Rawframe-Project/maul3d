// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The task dispatch every parallel phase goes through.

#ifndef MAUL3D_SRC_PARALLEL_H
#define MAUL3D_SRC_PARALLEL_H

#include "world_internal.h"

// Runs fn over [0, itemCount) in ranges of at least minRange through the
// host's task hooks, which the caller has checked are installed. Each
// range runs in the library's floating-point environment on whatever
// thread the host gives it, and the hooks themselves in the host's.
// Results never depend on the split.
void m3RunTasks(m3World* world, m3TaskFn* fn, void* ctx, int32_t itemCount, int32_t minRange);

#endif // MAUL3D_SRC_PARALLEL_H
