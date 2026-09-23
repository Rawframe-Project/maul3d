// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The command journal's vocabulary and entry points: the op codes, the
// recorder, and the replay worker behind m3World_JournalReplay.

#ifndef MAUL3D_SRC_JOURNAL_H
#define MAUL3D_SRC_JOURNAL_H

#include "world_internal.h"

// Journal ops. The stream is [i32 op][i32 size][payload], replayed
// through the same internal functions the public API uses.
typedef enum m3Op
{
    m3_opStep = 1, // reserved for task 9
    m3_opCreateBody = 2,
    m3_opDestroyBody = 3,
    m3_opSetLinearVelocity = 4,
    m3_opSetAngularVelocity = 5,
    m3_opCreateShape = 6,
    m3_opCreateHullShape = 7, // carries the input points (the recipe)
    m3_opCreateMeshShape = 8, // header + exact-size vertex/index payload
    m3_opCreateJoint = 9,
    m3_opDestroyJoint = 10,
    m3_opCreateVoxelChunkShape = 11,   // header + packed grid payload
    m3_opVoxelSet = 12,                // shape id + coords + payload
    m3_opVoxelClear = 13,              // shape id + coords
    m3_opVoxelClearBox = 14,           // shape id + inclusive region
    m3_opVoxelSetFill = 15,            // shape id + coords + fill byte
    m3_opCreateCharacter = 16,         // def + expected id
    m3_opDestroyCharacter = 17,        // id
    m3_opCharacterMove = 18,           // id + translation
    m3_opCreateVehicle = 19,           // def + expected id
    m3_opDestroyVehicle = 20,          // id
    m3_opVehicleCommands = 21,         // id + throttle, steer, brake
    m3_opCreateSoftBody = 22,          // def + expected id
    m3_opDestroySoftBody = 23,         // id
    m3_opSoftBodyPin = 24,             // id + particle index
    m3_opSoftBodyAnchor = 25,          // id + particle + body id
    m3_opApplyForce = 26,              // body + force at center
    m3_opApplyTorque = 27,             // body + torque
    m3_opApplyLinearImpulse = 28,      // body + impulse at center
    m3_opApplyAngularImpulse = 29,     // body + angular impulse
    m3_opApplyForceAtPoint = 30,       // body + force + world point
    m3_opApplyImpulseAtPoint = 31,     // body + impulse + world point
    m3_opSetTransform = 32,            // body + pose: the teleport
    m3_opSetTargetTransform = 33,      // body + pose: kinematic servo
    m3_opSetType = 34,                 // body + new type
    m3_opSetEnabled = 35,              // body + on/off
    m3_opSetMotionLocks = 36,          // body + lock bits
    m3_opSetSleepControls = 37,        // body + threshold + canSleep
    m3_opSetAwake = 38,                // body + awake flag
    m3_opSetGravity = 39,              // world gravity vector
    m3_opSetShapeFriction = 40,        // shape + coefficient
    m3_opSetShapeRestitution = 41,     // shape + coefficient
    m3_opSetShapeRolling = 42,         // shape + rolling resistance
    m3_opSetShapeDensity = 43,         // shape + density + mass rebuild flag
    m3_opSetContactTuning = 44,        // hertz + damping ratio + push speed
    m3_opSetRestitutionThreshold = 45, // world threshold
    m3_opSetMaximumLinearSpeed = 46,   // world speed cap
    m3_opEnableSleeping = 47,          // world toggle (off wakes everyone)
    m3_opEnableContinuous = 48,
    m3_opSetHitEventThreshold = 49,   // world hit speed gate
    m3_opEnableShapeHitEvents = 50,   // shape + on/off
    m3_opEnableShapePreSolve = 51,    // shape + on/off
    m3_opJointSetLimits = 52,         // joint + enable + lower + upper
    m3_opJointSetMotor = 53,          // joint + enable + speed + effort
    m3_opJointSetCollide = 54,        // joint + collide-connected flag
    m3_opJointSetBreak = 55,          // joint + force + torque caps
    m3_opJointSetSpring = 56,         // joint + enable + hertz + zeta
    m3_opJointSetTarget = 57,         // joint + scalar + quat drive target
    m3_opDestroyShape = 58,           // shape id (10-4: the red team
                                      // found bodies could shed shapes
                                      // only by dying)
    m3_opSoftBodyAnchorSoft = 59,     // lattice<->lattice pin
    m3_opSetWind = 60,                // world wind field
    m3_opSetSurfaceVelocity = 61,     // shape conveyor velocity        // world toggle
    m3_opVehicleDrivetrain = 62,      // vehicle + drivetrain def
    m3_opVehicleGear = 63,            // vehicle + gear select
    m3_opCharacterStance = 64,        // character + halfHeight + radius
    m3_opSetAllowFastRotation = 65,   // body + 0/1 spin-cap escape
    m3_opSetMaximumAngularSpeed = 66, // world spin cap
    m3_opWorldExplode = 67,           // explosion def
    m3_opSetBodyName = 68,            // body + 32 name bytes
    m3_opSetShapeGeom = 69,           // shape + type + geom swap
    m3_opJointSetSteer = 70,          // wheel strut drive
    m3_opJointSetMotorPose = 71,      // motor joint servo aim
    m3_opSetMeshMaterials = 72,       // per-triangle materials
    m3_opRebuildBroadphase = 73,      // balanced tree rebuild
    m3_opCreateWaterVolume = 74,      // water box
    m3_opDestroyWaterVolume = 75,     // the tide goes out
    m3_opCreateHeightFieldGrid = 76,  // native terrain chunk
    m3_opCreateSoftBodyTet = 77,      // the incompressible jelly
    m3_opVehicleTankCommands = 78,
    // The pre-solve vetoes a step's callback made,
    // recorded BEFORE that step's own op so a bare replay applies
    // them without the host's callback. Payload: the vetoed pair
    // keys, canonical ascending (count = bytes / 8).
    m3_opStepVetoes = 79, // skid steer
    m3_opCount
} m3Op;

// Recording: appends one op; an overflow latches and fails the recording.
void m3JournalRecord(m3World* world, int32_t op, const void* payload, int32_t bytes);
// Fails the active recording when an op cannot be encoded.
void m3JournalAbandon(m3World* world);

// Applies a tape's ops in order through the command table and reports
// the first refusal. Partial application is possible here;
// m3World_JournalReplay makes the whole call atomic.
bool m3JournalReplayApply(m3World* world, const void* data, int32_t size);

#endif // MAUL3D_SRC_JOURNAL_H
