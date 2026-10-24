// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact constraints: one per touching manifold that can move. Each has
// a normal row per point, and at the manifold's center one friction row
// pair, a twist row about the normal and a rolling row. Built by
// contact_prepare.c, solved by graph color in contact_solver.c.

#ifndef MAUL3D_SRC_CONTACT_SOLVER_H
#define MAUL3D_SRC_CONTACT_SOLVER_H

#include "contact_constraint.h"
#include "solver_math.h"
#include "world_internal.h"

int32_t m3PrepareContacts(m3World* world, m3ContactConstraint* constraints, m3real h);

// Colors the prepared constraints. Returns false on a scratch stall.
bool m3ColorContacts(m3World* world, m3ContactPlan* plan);

void m3RunContactStage(m3World* world, const m3ContactPlan* plan, m3ContactStage stage);

#endif // MAUL3D_SRC_CONTACT_SOLVER_H
