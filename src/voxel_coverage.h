// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Voxel seams: the links between neighboring chunks and the coverage of
// their touching faces, rebuilt whenever a chunk's content or neighbors
// change.

#ifndef MAUL3D_SRC_VOXEL_COVERAGE_H
#define MAUL3D_SRC_VOXEL_COVERAGE_H

#include "world_internal.h"

// Seam welding. Links derive from EXACT transforms: two
// chunks weld when both bodies carry the bit-exact identity
// rotation, cell sizes match, and world positions differ by
// exactly one chunk extent along one axis (grid-laid level
// geometry; rotated assemblies still collide, just without seam
// suppression, documented in the public header). Coverage reads
// the box list of the slot and the GRIDS of the slot and its
// welded neighbors; both are pure functions of world state and
// rebuild wherever content lands.
void m3VoxelRebuildLinks(m3World* world);

void m3VoxelCoverageBuild(m3World* world, int32_t slot);

void m3VoxelCoverageRefreshAround(m3World* world, int32_t slot);

#endif
