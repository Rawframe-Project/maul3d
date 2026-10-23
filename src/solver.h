// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Solver pieces shared by the contact and joint solvers.

#ifndef MAUL3D_SRC_SOLVER_H
#define MAUL3D_SRC_SOLVER_H

#include "world_internal.h"

void m3StepInternal(m3World* world, float dt, int32_t substeps);

#endif // MAUL3D_SRC_SOLVER_H
