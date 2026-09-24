// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay internals: the record type, the bool normalizers every
// handler uses on untrusted defs, and the handlers that live outside
// journal_replay.c, grouped by the part of the world they drive.

#ifndef MAUL3D_SRC_JOURNAL_REPLAY_H
#define MAUL3D_SRC_JOURNAL_REPLAY_H

#include "world_internal.h"

#include <stddef.h>

// One record as the loop found it: the op code, its payload and size.
typedef struct m3ReplayRecord
{
    int32_t op;
    const uint8_t* payload;
    int32_t bytes;
} m3ReplayRecord;

typedef bool m3ReplayApplyFn(m3World* world, const m3ReplayRecord* r);

// Journaled defs are untrusted bytes: a flipped bit in an embedded bool
// field is undefined even to load as _Bool, so every handler normalizes
// bool bytes through uint8_t before the def is used as its C type.
static inline void m3NormalizeBoolByte(void* base, size_t offset)
{
    uint8_t* b = (uint8_t*)base + offset;
    *b = *b != 0 ? 1 : 0;
}

static inline void m3NormalizeShapeDefBools(m3ShapeDef* def)
{
    m3NormalizeBoolByte(def, offsetof(m3ShapeDef, isSensor));
    m3NormalizeBoolByte(def, offsetof(m3ShapeDef, enableHitEvents));
    m3NormalizeBoolByte(def, offsetof(m3ShapeDef, enablePreSolveEvents));
}

// journal_replay_shapes.c: shapes, meshes, hulls, height fields and voxels.
m3ReplayApplyFn m3ReplayCreateShape;
m3ReplayApplyFn m3ReplayCreateMeshShape;
m3ReplayApplyFn m3ReplayCreateHullShape;
m3ReplayApplyFn m3ReplayCreateVoxelChunkShape;
m3ReplayApplyFn m3ReplayVoxelSet;
m3ReplayApplyFn m3ReplayVoxelClear;
m3ReplayApplyFn m3ReplayVoxelSetFill;
m3ReplayApplyFn m3ReplayVoxelClearBox;
m3ReplayApplyFn m3ReplayDestroyShape;
m3ReplayApplyFn m3ReplayShapeScalar;
m3ReplayApplyFn m3ReplaySetShapeDensity;
m3ReplayApplyFn m3ReplayCreateHeightFieldGrid;
m3ReplayApplyFn m3ReplaySetMeshMaterials;
m3ReplayApplyFn m3ReplaySetShapeGeom;
m3ReplayApplyFn m3ReplaySetSurfaceVelocity;
m3ReplayApplyFn m3ReplaySetFilter;
m3ReplayApplyFn m3ReplayShapeFlag;

// journal_replay_joints.c: joints.
m3ReplayApplyFn m3ReplayCreateJoint;
m3ReplayApplyFn m3ReplayDestroyJoint;
m3ReplayApplyFn m3ReplayJointSetMotorPose;
m3ReplayApplyFn m3ReplayJointSetSteer;
m3ReplayApplyFn m3ReplayJointVector;
m3ReplayApplyFn m3ReplayJointSetCollide;
m3ReplayApplyFn m3ReplayJointSetBreak;
m3ReplayApplyFn m3ReplayJointSetSpring;
m3ReplayApplyFn m3ReplayJointSetTarget;

// journal_replay_actors.c: characters, vehicles, soft bodies and water.
m3ReplayApplyFn m3ReplayCreateCharacter;
m3ReplayApplyFn m3ReplayDestroyCharacter;
m3ReplayApplyFn m3ReplayCharacterMove;
m3ReplayApplyFn m3ReplayCharacterStance;
m3ReplayApplyFn m3ReplayCreateVehicle;
m3ReplayApplyFn m3ReplayDestroyVehicle;
m3ReplayApplyFn m3ReplayVehicleTankCommands;
m3ReplayApplyFn m3ReplayVehicleCommands;
m3ReplayApplyFn m3ReplayVehicleDrivetrain;
m3ReplayApplyFn m3ReplayVehicleGear;
m3ReplayApplyFn m3ReplayCreateSoftBody;
m3ReplayApplyFn m3ReplayDestroySoftBody;
m3ReplayApplyFn m3ReplaySoftBodyPin;
m3ReplayApplyFn m3ReplaySoftBodyAnchor;
m3ReplayApplyFn m3ReplaySoftBodyAnchorSoft;
m3ReplayApplyFn m3ReplayCreateSoftBodyTet;
m3ReplayApplyFn m3ReplayCreateWaterVolume;
m3ReplayApplyFn m3ReplayDestroyWaterVolume;

#endif // MAUL3D_SRC_JOURNAL_REPLAY_H
