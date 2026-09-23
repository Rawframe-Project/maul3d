// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The narrow phase: internal declarations.

#ifndef MAUL3D_SRC_NARROWPHASE_H
#define MAUL3D_SRC_NARROWPHASE_H

#include "world_internal.h"

// Narrowphase v1: rebuild manifolds for the current pairs in pair
// order, carrying warm-start impulses forward by feature id from the
// PREVIOUS step's stash (the caller copies keys and manifolds to step
// scratch BEFORE m3UpdatePairs overwrites them; oldKeys are sorted).
m3Result m3UpdateContacts(m3World* world, const uint64_t* oldKeys, const m3Manifold* oldManifolds,
                          int32_t oldCount);

// The narrowphase over one pair range: every pair writes only its own
// manifold slot, so any partition of [0, pairCount) is bit-identical
// to the serial run (the parallel contract).
void m3UpdateContactsRange(m3World* world, int32_t start, int32_t end, const uint64_t* oldKeys,
                           const m3Manifold* oldManifolds, int32_t oldCount);

#endif // MAUL3D_SRC_NARROWPHASE_H
