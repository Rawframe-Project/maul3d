// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Snapshots: internal declarations. A restore taken apart, so a caller
// can hold the memory a restore needs before it needs it: the replay
// prepares the restore that backs a failed tape out before it applies
// the first op, so backing out cannot fail.

#ifndef MAUL3D_SRC_SNAPSHOT_H
#define MAUL3D_SRC_SNAPSHOT_H

#include "world_internal.h"

// What a restore takes from the allocator, all of it before any byte
// lands: the incoming mesh and height field content and the mesh BVHs
// derived from it, beside the buffer it lands from, which must outlive
// the stage. Voxel surfaces need only the tree storage of a live slot
// that has none yet: the commit rebuilds every surface into the trees
// its slots keep.
typedef struct m3RestoreStage
{
    const uint8_t* data;             // the checked snapshot
    m3MeshData* meshes;              // per mesh slot
    m3MeshBvh* meshBvhs;             // per mesh slot, built for live meshes
    m3HeightFieldData* heightFields; // per shape slot
    void** voxelTrees;               // per voxel slot, for live slots with none
    int32_t hullCursor;              // where the hull tail starts in the buffer
} m3RestoreStage;

// Checks every byte of a snapshot against the world and takes all the
// memory landing it needs, leaving the world as it was: m3_errorInvalid
// for bytes that fail a check, m3_errorCapacity when the allocator
// refuses. On success the stage holds the memory until a commit or a
// drop; on failure it holds nothing.
m3Result m3RestorePrepare(m3World* world, const void* data, int32_t size, m3RestoreStage* stage);

// Lands a prepared snapshot. Allocates nothing, so it cannot fail.
void m3RestoreCommit(m3World* world, m3RestoreStage* stage);

// Gives a prepared stage's memory back without landing it.
void m3RestoreDrop(m3World* world, m3RestoreStage* stage);

#endif // MAUL3D_SRC_SNAPSHOT_H
