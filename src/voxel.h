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
void m3VoxelFractureSweep(m3World* world, int32_t shape);

void m3VoxelBoundsHull(m3Vec3 lo, m3Vec3 hi, m3HullData* out);

// Whether a cell lies in a chunk, 0 to M3_VOXEL_DIM - 1 on every axis: the
// check the public edits and their replay ops ask before an internal.
bool m3VoxelCoordsValid(int32_t x, int32_t y, int32_t z);

// Voxel edit internals: apply without journaling (replay
// drives these); the public entries validate, call, then journal what
// landed. The cell must be one m3VoxelCoordsValid accepts. An edit
// whose surface would pass the box limit changes nothing, counts in
// voxelEditsRefused and returns false (-1 for the box clear and the
// carve).
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
