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

typedef bool m3ReplayApplyFunction(m3World* world, const m3ReplayRecord* r);

// Journaled defs are untrusted bytes: a flipped bit in an embedded bool
// field is undefined even to load as _Bool, so every handler normalizes
// bool bytes through uint8_t before the def is used as its C type.
static inline void m3NormalizeBoolByte(void* base, size_t offset)
{
    uint8_t* b = (uint8_t*)base + offset;
    *b = *b != 0 ? 1 : 0;
}

static inline void m3NormalizeBodyDefBools(m3BodyDef* def)
{
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, motionLocks.linearX));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, motionLocks.linearY));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, motionLocks.linearZ));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, motionLocks.angularX));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, motionLocks.angularY));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, motionLocks.angularZ));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, isBullet));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, enableSleep));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, isEnabled));
    m3NormalizeBoolByte(def, offsetof(m3BodyDef, enableFastRotation));
}

static inline void m3NormalizeShapeDefBools(m3ShapeDef* def)
{
    m3NormalizeBoolByte(def, offsetof(m3ShapeDef, isSensor));
    m3NormalizeBoolByte(def, offsetof(m3ShapeDef, enableHitEvents));
    m3NormalizeBoolByte(def, offsetof(m3ShapeDef, enablePreSolveEvents));
}

// journal_replay_shapes.c: shapes, meshes, hulls, height fields and voxels.
m3ReplayApplyFunction m3ReplayCreateShape;
m3ReplayApplyFunction m3ReplayCreateMeshShape;
m3ReplayApplyFunction m3ReplayCreateHullShape;
m3ReplayApplyFunction m3ReplayCreateVoxelChunkShape;
m3ReplayApplyFunction m3ReplayVoxelSet;
m3ReplayApplyFunction m3ReplayVoxelClear;
m3ReplayApplyFunction m3ReplayVoxelSetFill;
m3ReplayApplyFunction m3ReplayVoxelClearBox;
m3ReplayApplyFunction m3ReplayDestroyShape;
m3ReplayApplyFunction m3ReplayShapeScalar;
m3ReplayApplyFunction m3ReplaySetShapeDensity;
m3ReplayApplyFunction m3ReplayCreateHeightFieldGrid;
m3ReplayApplyFunction m3ReplaySetMeshMaterials;
m3ReplayApplyFunction m3ReplaySetShapeGeom;
m3ReplayApplyFunction m3ReplaySetSurfaceVelocity;
m3ReplayApplyFunction m3ReplaySetFilter;
m3ReplayApplyFunction m3ReplayShapeFlag;

// journal_replay_joints.c: joints.
m3ReplayApplyFunction m3ReplayCreateJoint;
m3ReplayApplyFunction m3ReplayDestroyJoint;
m3ReplayApplyFunction m3ReplayMotorJointSetOffsets;
m3ReplayApplyFunction m3ReplayJointSetSteer;
m3ReplayApplyFunction m3ReplayJointVector;
m3ReplayApplyFunction m3ReplayJointEnableConnectedCollision;
m3ReplayApplyFunction m3ReplayJointSetBreak;
m3ReplayApplyFunction m3ReplayJointSetSpring;
m3ReplayApplyFunction m3ReplayJointSetTarget;

// journal_replay_actors.c: characters, vehicles, soft bodies and water.
m3ReplayApplyFunction m3ReplayCreateCharacter;
m3ReplayApplyFunction m3ReplayDestroyCharacter;
m3ReplayApplyFunction m3ReplayCharacterMove;
m3ReplayApplyFunction m3ReplayCharacterStance;
m3ReplayApplyFunction m3ReplayCreateVehicle;
m3ReplayApplyFunction m3ReplayDestroyVehicle;
m3ReplayApplyFunction m3ReplayVehicleTankCommands;
m3ReplayApplyFunction m3ReplayVehicleCommands;
m3ReplayApplyFunction m3ReplayVehicleDrivetrain;
m3ReplayApplyFunction m3ReplayVehicleGear;
m3ReplayApplyFunction m3ReplayCreateSoftBody;
m3ReplayApplyFunction m3ReplayDestroySoftBody;
m3ReplayApplyFunction m3ReplaySoftBodyPin;
m3ReplayApplyFunction m3ReplaySoftBodyAnchor;
m3ReplayApplyFunction m3ReplaySoftBodyAnchorSoft;
m3ReplayApplyFunction m3ReplayCreateSoftBodyTet;
m3ReplayApplyFunction m3ReplayCreateWaterVolume;
m3ReplayApplyFunction m3ReplayDestroyWaterVolume;

#endif // MAUL3D_SRC_JOURNAL_REPLAY_H
