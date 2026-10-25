// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restore's magnitude checks (snapshot_bounds.c).

#ifndef MAUL3D_SRC_SNAPSHOT_BOUNDS_H
#define MAUL3D_SRC_SNAPSHOT_BOUNDS_H

#include "snapshot_values.h"

// Whether every value a live record holds is within the bound its create,
// setter and replay op keep (src/core.h). The structure and per-record
// checks run first, so every count and index read here is in range.
bool m3SnapshotBoundsValid(const m3Incoming* inc);

#endif // MAUL3D_SRC_SNAPSHOT_BOUNDS_H
