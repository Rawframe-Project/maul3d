// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Voxel chunks: internal declarations.

#ifndef MAUL3D_SRC_VOXEL_H
#define MAUL3D_SRC_VOXEL_H

#include "world_internal.h"

int32_t m3VoxelCarveSphereInternal(m3World* world, int32_t shape, m3Vec3 center, m3real radius);

// Fracture: after a clearing edit, flood fill the chunk in
// canonical order; islands with no voxel in the y = 0 base layer
// are removed from the grid (part of the SAME state transition, so
// replay and rollback re-derive identical grids) and emitted as
// fragment events with their recipes.
#define M3_FRAGMENT_EVENT_CAP 256

#define M3_FRAGMENT_RECIPE_CAP 8192

void m3VoxelFractureSweep(m3World* world, int32_t shape);

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

void m3VoxelBoundsHull(m3Vec3 lo, m3Vec3 hi, m3HullData* out);

// Voxel edit internals: apply without journaling (replay
// drives these); the public entries validate, journal, then call.
bool m3VoxelSetInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z,
                        uint16_t payload);

bool m3VoxelClearInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z);

int32_t m3VoxelClearBoxInternal(m3World* world, int32_t shape, const int32_t lo[3],
                                const int32_t hi[3]);

bool m3VoxelSetFillInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z,
                            uint8_t fill);

bool m3VoxelEscape(const m3World* world, int32_t slot, m3Vec3 localPoint, m3Vec3* outNormal,
                   m3real* outPlane);

#endif // MAUL3D_SRC_VOXEL_H
