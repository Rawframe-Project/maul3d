// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restore's mass checks (src/snapshot_mass.c).

#ifndef MAUL3D_SRC_SNAPSHOT_MASS_H
#define MAUL3D_SRC_SNAPSHOT_MASS_H

#include "world_internal.h"

// Whether every incoming dynamic body's shapes keep the mass floors and
// the body within M3_MASS_LIMIT, and every incoming joint its lever ratio
// at the masses restore will compute: m3_errorInvalid when not, and
// m3_errorCapacity when the allocator refuses the scratch. raw is the
// snapshot, prefix its state blocks and hullCursor where its hull content
// starts; the links check has made every list and index safe to follow.
m3Result m3SnapshotMassesFit(m3World* world, const uint8_t* raw, const uint8_t* prefix,
                             int32_t hullCursor);

#endif // MAUL3D_SRC_SNAPSHOT_MASS_H
