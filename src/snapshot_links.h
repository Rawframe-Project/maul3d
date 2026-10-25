// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The whole-structure checks restore runs on an incoming snapshot after
// every block has passed its own (snapshot_links.c).

#ifndef MAUL3D_SRC_SNAPSHOT_LINKS_H
#define MAUL3D_SRC_SNAPSHOT_LINKS_H

#include "world_internal.h"

// Checks the links and cross-references of the incoming fixed prefix in
// (the bytes after the header), whose blocks m3StateValidate has passed:
// m3_success, m3_errorInvalid for a structure the engine could not have
// written, or m3_errorCapacity when the allocator refuses the scratch.
// Reads only; nothing lands.
m3Result m3SnapshotLinksCheck(m3World* world, const uint8_t* in);

#endif // MAUL3D_SRC_SNAPSHOT_LINKS_H
