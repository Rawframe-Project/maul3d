// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The journal and snapshot formats, held to one layout on every target. A
// tape is its records' bytes and a snapshot walks its state arrays as they
// lie in memory, so each record, each public def a record embeds and each
// element type a snapshot walks is pinned here to its layout on x86-64:
// every size and every member's offset. A target that would lay one out
// otherwise (32-bit x86 aligns a double or a 64-bit integer in a struct to
// 4, which M3_ALIGN8 undoes) fails to build instead of writing another
// format. A change to one of these types is a format change: update its
// numbers here and say so in the changelog.

#include "journal.h"
#include "world_internal.h"

#include <stddef.h>

// The formats are little-endian; a big-endian target needs byte swaps in
// the journal and the snapshot walk first.
#if defined(__BYTE_ORDER__)
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "maul3d: the journal and snapshot formats are little-endian"
#endif
#elif !defined(_WIN32)
#error "maul3d: cannot tell this target's byte order"
#endif

#define M3_PIN_SIZE(type, bytes)                                                                   \
    _Static_assert(sizeof(type) == (bytes), #type " left the format's layout")
#define M3_PIN_AT(type, member, at)                                                                \
    _Static_assert(offsetof(type, member) == (at), #type "." #member " left the format's layout")

// The journal's records, in journal.h's order.
M3_PIN_SIZE(m3OpBodyByte, 12);
M3_PIN_AT(m3OpBodyByte, id, 0);
M3_PIN_AT(m3OpBodyByte, value, 8);

M3_PIN_SIZE(m3OpBodyParam, 16);
M3_PIN_AT(m3OpBodyParam, id, 0);
M3_PIN_AT(m3OpBodyParam, param, 8);
M3_PIN_AT(m3OpBodyParam, value, 12);

M3_PIN_SIZE(m3OpUserData, 16);
M3_PIN_AT(m3OpUserData, userData, 0);
M3_PIN_AT(m3OpUserData, id, 8);

M3_PIN_SIZE(m3OpBodyPose, 48);
M3_PIN_AT(m3OpBodyPose, id, 0);
M3_PIN_AT(m3OpBodyPose, pose, 8);

M3_PIN_SIZE(m3OpBodyVector, 20);
M3_PIN_AT(m3OpBodyVector, id, 0);
M3_PIN_AT(m3OpBodyVector, v, 8);

M3_PIN_SIZE(m3OpBodyVectorAtPoint, 48);
M3_PIN_AT(m3OpBodyVectorAtPoint, id, 0);
M3_PIN_AT(m3OpBodyVectorAtPoint, v, 8);
M3_PIN_AT(m3OpBodyVectorAtPoint, p, 24);

M3_PIN_SIZE(m3OpCharacterMove, 20);
M3_PIN_AT(m3OpCharacterMove, id, 0);
M3_PIN_AT(m3OpCharacterMove, translation, 8);

M3_PIN_SIZE(m3OpCharacterStance, 16);
M3_PIN_AT(m3OpCharacterStance, id, 0);
M3_PIN_AT(m3OpCharacterStance, halfHeight, 8);
M3_PIN_AT(m3OpCharacterStance, radius, 12);

M3_PIN_SIZE(m3OpCreateBody, 128);
M3_PIN_AT(m3OpCreateBody, def, 0);
M3_PIN_AT(m3OpCreateBody, expected, 120);

M3_PIN_SIZE(m3OpCreateCharacter, 80);
M3_PIN_AT(m3OpCreateCharacter, def, 0);
M3_PIN_AT(m3OpCreateCharacter, expected, 72);

M3_PIN_SIZE(m3OpCreateSoftBody, 88);
M3_PIN_AT(m3OpCreateSoftBody, def, 0);
M3_PIN_AT(m3OpCreateSoftBody, expected, 80);

M3_PIN_SIZE(m3OpCreateVehicle, 472);
M3_PIN_AT(m3OpCreateVehicle, def, 0);
M3_PIN_AT(m3OpCreateVehicle, expected, 464);

M3_PIN_SIZE(m3OpCreateVoxelChunkShape, 128);
M3_PIN_AT(m3OpCreateVoxelChunkShape, def, 0);
M3_PIN_AT(m3OpCreateVoxelChunkShape, body, 104);
M3_PIN_AT(m3OpCreateVoxelChunkShape, expected, 112);
M3_PIN_AT(m3OpCreateVoxelChunkShape, cellSize, 120);

M3_PIN_SIZE(m3OpCreateWaterVolume, 88);
M3_PIN_AT(m3OpCreateWaterVolume, def, 0);
M3_PIN_AT(m3OpCreateWaterVolume, expected, 80);

M3_PIN_SIZE(m3OpJointSetBreak, 16);
M3_PIN_AT(m3OpJointSetBreak, id, 0);
M3_PIN_AT(m3OpJointSetBreak, maxForce, 8);
M3_PIN_AT(m3OpJointSetBreak, maxTorque, 12);

M3_PIN_SIZE(m3OpJointSetCollide, 12);
M3_PIN_AT(m3OpJointSetCollide, id, 0);
M3_PIN_AT(m3OpJointSetCollide, on, 8);

M3_PIN_SIZE(m3OpJointSetMotorPose, 36);
M3_PIN_AT(m3OpJointSetMotorPose, id, 0);
M3_PIN_AT(m3OpJointSetMotorPose, offset, 8);
M3_PIN_AT(m3OpJointSetMotorPose, rotation, 20);

M3_PIN_SIZE(m3OpJointSetSpring, 20);
M3_PIN_AT(m3OpJointSetSpring, id, 0);
M3_PIN_AT(m3OpJointSetSpring, enable, 8);
M3_PIN_AT(m3OpJointSetSpring, hertz, 12);
M3_PIN_AT(m3OpJointSetSpring, zeta, 16);

M3_PIN_SIZE(m3OpJointSetSteer, 28);
M3_PIN_AT(m3OpJointSetSteer, id, 0);
M3_PIN_AT(m3OpJointSetSteer, enable, 8);
M3_PIN_AT(m3OpJointSetSteer, target, 12);
M3_PIN_AT(m3OpJointSetSteer, hertz, 16);
M3_PIN_AT(m3OpJointSetSteer, zeta, 20);
M3_PIN_AT(m3OpJointSetSteer, effort, 24);

M3_PIN_SIZE(m3OpJointSetTarget, 28);
M3_PIN_AT(m3OpJointSetTarget, id, 0);
M3_PIN_AT(m3OpJointSetTarget, scalar, 8);
M3_PIN_AT(m3OpJointSetTarget, q, 12);

M3_PIN_SIZE(m3OpJointVector, 20);
M3_PIN_AT(m3OpJointVector, id, 0);
M3_PIN_AT(m3OpJointVector, enable, 8);
M3_PIN_AT(m3OpJointVector, a, 12);
M3_PIN_AT(m3OpJointVector, b, 16);

M3_PIN_SIZE(m3OpSetAllowFastRotation, 12);
M3_PIN_AT(m3OpSetAllowFastRotation, id, 0);
M3_PIN_AT(m3OpSetAllowFastRotation, allow, 8);

M3_PIN_SIZE(m3OpSetAngularVelocity, 20);
M3_PIN_AT(m3OpSetAngularVelocity, id, 0);
M3_PIN_AT(m3OpSetAngularVelocity, v, 8);

M3_PIN_SIZE(m3OpSetBodyName, 40);
M3_PIN_AT(m3OpSetBodyName, id, 0);
M3_PIN_AT(m3OpSetBodyName, name, 8);

M3_PIN_SIZE(m3OpSetContactTuning, 12);
M3_PIN_AT(m3OpSetContactTuning, hertz, 0);
M3_PIN_AT(m3OpSetContactTuning, dampingRatio, 4);
M3_PIN_AT(m3OpSetContactTuning, pushSpeed, 8);

M3_PIN_SIZE(m3OpSetLinearVelocity, 20);
M3_PIN_AT(m3OpSetLinearVelocity, id, 0);
M3_PIN_AT(m3OpSetLinearVelocity, v, 8);

M3_PIN_SIZE(m3OpSetMotionLocks, 12);
M3_PIN_AT(m3OpSetMotionLocks, id, 0);
M3_PIN_AT(m3OpSetMotionLocks, locks, 8);

M3_PIN_SIZE(m3OpSetShapeDensity, 12);
M3_PIN_AT(m3OpSetShapeDensity, id, 0);
M3_PIN_AT(m3OpSetShapeDensity, value, 8);

M3_PIN_SIZE(m3OpSetShapeGeom, 44);
M3_PIN_AT(m3OpSetShapeGeom, id, 0);
M3_PIN_AT(m3OpSetShapeGeom, type, 8);
M3_PIN_AT(m3OpSetShapeGeom, geom, 12);

M3_PIN_SIZE(m3OpSetSleepControls, 16);
M3_PIN_AT(m3OpSetSleepControls, id, 0);
M3_PIN_AT(m3OpSetSleepControls, threshold, 8);
M3_PIN_AT(m3OpSetSleepControls, canSleep, 12);

M3_PIN_SIZE(m3OpSetSurfaceVelocity, 20);
M3_PIN_AT(m3OpSetSurfaceVelocity, id, 0);
M3_PIN_AT(m3OpSetSurfaceVelocity, v, 8);

M3_PIN_SIZE(m3OpSetFilter, 32);
M3_PIN_AT(m3OpSetFilter, categoryBits, 0);
M3_PIN_AT(m3OpSetFilter, maskBits, 8);
M3_PIN_AT(m3OpSetFilter, id, 16);
M3_PIN_AT(m3OpSetFilter, groupIndex, 24);

M3_PIN_SIZE(m3OpSetWind, 24);
M3_PIN_AT(m3OpSetWind, dir, 0);
M3_PIN_AT(m3OpSetWind, speed, 12);
M3_PIN_AT(m3OpSetWind, gustHertz, 16);
M3_PIN_AT(m3OpSetWind, gustScale, 20);

M3_PIN_SIZE(m3OpShapeFlag, 12);
M3_PIN_AT(m3OpShapeFlag, id, 0);
M3_PIN_AT(m3OpShapeFlag, on, 8);

M3_PIN_SIZE(m3OpShapeScalar, 12);
M3_PIN_AT(m3OpShapeScalar, id, 0);
M3_PIN_AT(m3OpShapeScalar, value, 8);

M3_PIN_SIZE(m3OpSoftBodyAnchor, 20);
M3_PIN_AT(m3OpSoftBodyAnchor, id, 0);
M3_PIN_AT(m3OpSoftBodyAnchor, particle, 8);
M3_PIN_AT(m3OpSoftBodyAnchor, body, 12);

M3_PIN_SIZE(m3OpSoftBodyAnchorSoft, 24);
M3_PIN_AT(m3OpSoftBodyAnchorSoft, idA, 0);
M3_PIN_AT(m3OpSoftBodyAnchorSoft, particleA, 8);
M3_PIN_AT(m3OpSoftBodyAnchorSoft, idB, 12);
M3_PIN_AT(m3OpSoftBodyAnchorSoft, particleB, 20);

M3_PIN_SIZE(m3OpSoftBodyPin, 12);
M3_PIN_AT(m3OpSoftBodyPin, id, 0);
M3_PIN_AT(m3OpSoftBodyPin, particle, 8);

M3_PIN_SIZE(m3OpStep, 8);
M3_PIN_AT(m3OpStep, dt, 0);
M3_PIN_AT(m3OpStep, substeps, 4);

M3_PIN_SIZE(m3OpVehicleCommands, 20);
M3_PIN_AT(m3OpVehicleCommands, id, 0);
M3_PIN_AT(m3OpVehicleCommands, throttle, 8);
M3_PIN_AT(m3OpVehicleCommands, steer, 12);
M3_PIN_AT(m3OpVehicleCommands, brake, 16);

M3_PIN_SIZE(m3OpVehicleDrivetrain, 140);
M3_PIN_AT(m3OpVehicleDrivetrain, id, 0);
M3_PIN_AT(m3OpVehicleDrivetrain, def, 8);

M3_PIN_SIZE(m3OpVehicleGear, 12);
M3_PIN_AT(m3OpVehicleGear, id, 0);
M3_PIN_AT(m3OpVehicleGear, gear, 8);

M3_PIN_SIZE(m3OpVehicleTankCommands, 20);
M3_PIN_AT(m3OpVehicleTankCommands, id, 0);
M3_PIN_AT(m3OpVehicleTankCommands, left, 8);
M3_PIN_AT(m3OpVehicleTankCommands, right, 12);
M3_PIN_AT(m3OpVehicleTankCommands, brake, 16);

M3_PIN_SIZE(m3OpVoxelClear, 20);
M3_PIN_AT(m3OpVoxelClear, id, 0);
M3_PIN_AT(m3OpVoxelClear, x, 8);
M3_PIN_AT(m3OpVoxelClear, y, 12);
M3_PIN_AT(m3OpVoxelClear, z, 16);

M3_PIN_SIZE(m3OpVoxelClearBox, 32);
M3_PIN_AT(m3OpVoxelClearBox, id, 0);
M3_PIN_AT(m3OpVoxelClearBox, lo, 8);
M3_PIN_AT(m3OpVoxelClearBox, hi, 20);

M3_PIN_SIZE(m3OpVoxelSet, 24);
M3_PIN_AT(m3OpVoxelSet, id, 0);
M3_PIN_AT(m3OpVoxelSet, x, 8);
M3_PIN_AT(m3OpVoxelSet, y, 12);
M3_PIN_AT(m3OpVoxelSet, z, 16);
M3_PIN_AT(m3OpVoxelSet, payload, 20);
M3_PIN_AT(m3OpVoxelSet, pad, 22);

M3_PIN_SIZE(m3OpVoxelSetFill, 24);
M3_PIN_AT(m3OpVoxelSetFill, id, 0);
M3_PIN_AT(m3OpVoxelSetFill, x, 8);
M3_PIN_AT(m3OpVoxelSetFill, y, 12);
M3_PIN_AT(m3OpVoxelSetFill, z, 16);
M3_PIN_AT(m3OpVoxelSetFill, fill, 20);
M3_PIN_AT(m3OpVoxelSetFill, pad, 21);

M3_PIN_SIZE(m3SetMeshMaterialsOp, 208);
M3_PIN_AT(m3SetMeshMaterialsOp, id, 0);
M3_PIN_AT(m3SetMeshMaterialsOp, materialCount, 8);
M3_PIN_AT(m3SetMeshMaterialsOp, triangleCount, 12);
M3_PIN_AT(m3SetMeshMaterialsOp, materials, 16);

M3_PIN_SIZE(m3CreateSoftBodyTetOp, 96);
M3_PIN_AT(m3CreateSoftBodyTetOp, def, 0);
M3_PIN_AT(m3CreateSoftBodyTetOp, pointCount, 80);
M3_PIN_AT(m3CreateSoftBodyTetOp, tetCount, 84);
M3_PIN_AT(m3CreateSoftBodyTetOp, expected, 88);

M3_PIN_SIZE(m3CreateHeightFieldGridOp, 136);
M3_PIN_AT(m3CreateHeightFieldGridOp, body, 0);
M3_PIN_AT(m3CreateHeightFieldGridOp, def, 8);
M3_PIN_AT(m3CreateHeightFieldGridOp, nx, 112);
M3_PIN_AT(m3CreateHeightFieldGridOp, nz, 116);
M3_PIN_AT(m3CreateHeightFieldGridOp, cellSize, 120);
M3_PIN_AT(m3CreateHeightFieldGridOp, expected, 124);

M3_PIN_SIZE(m3CreateShapeOp, 160);
M3_PIN_AT(m3CreateShapeOp, def, 0);
M3_PIN_AT(m3CreateShapeOp, geom, 104);
M3_PIN_AT(m3CreateShapeOp, body, 136);
M3_PIN_AT(m3CreateShapeOp, expected, 144);
M3_PIN_AT(m3CreateShapeOp, type, 152);
M3_PIN_AT(m3CreateShapeOp, pad, 153);

M3_PIN_SIZE(m3CreateMeshShapeOp, 128);
M3_PIN_AT(m3CreateMeshShapeOp, def, 0);
M3_PIN_AT(m3CreateMeshShapeOp, body, 104);
M3_PIN_AT(m3CreateMeshShapeOp, expected, 112);
M3_PIN_AT(m3CreateMeshShapeOp, vertexCount, 120);
M3_PIN_AT(m3CreateMeshShapeOp, triangleCount, 124);

M3_PIN_SIZE(m3CreateJointOp, 232);
M3_PIN_AT(m3CreateJointOp, def, 0);
M3_PIN_AT(m3CreateJointOp, expected, 224);

M3_PIN_SIZE(m3CreateHullShapeOp, 3200);
M3_PIN_AT(m3CreateHullShapeOp, def, 0);
M3_PIN_AT(m3CreateHullShapeOp, body, 104);
M3_PIN_AT(m3CreateHullShapeOp, expected, 112);
M3_PIN_AT(m3CreateHullShapeOp, count, 120);
M3_PIN_AT(m3CreateHullShapeOp, points, 124);

// The public defs the records embed, the query filter the explosion def
// holds, and the wheel the vehicle def holds.
M3_PIN_SIZE(m3BodyDef, 120);
M3_PIN_AT(m3BodyDef, type, 0);
M3_PIN_AT(m3BodyDef, position, 8);
M3_PIN_AT(m3BodyDef, rotation, 32);
M3_PIN_AT(m3BodyDef, linearVelocity, 48);
M3_PIN_AT(m3BodyDef, angularVelocity, 60);
M3_PIN_AT(m3BodyDef, gravityScale, 72);
M3_PIN_AT(m3BodyDef, linearDamping, 76);
M3_PIN_AT(m3BodyDef, angularDamping, 80);
M3_PIN_AT(m3BodyDef, userData, 88);
M3_PIN_AT(m3BodyDef, isBullet, 96);
M3_PIN_AT(m3BodyDef, motionLocks, 97);
M3_PIN_AT(m3BodyDef, enableSleep, 103);
M3_PIN_AT(m3BodyDef, sleepThreshold, 104);
M3_PIN_AT(m3BodyDef, isEnabled, 108);
M3_PIN_AT(m3BodyDef, enableFastRotation, 109);
M3_PIN_AT(m3BodyDef, internalValue, 112);

M3_PIN_SIZE(m3CharacterDef, 72);
M3_PIN_AT(m3CharacterDef, position, 0);
M3_PIN_AT(m3CharacterDef, radius, 24);
M3_PIN_AT(m3CharacterDef, halfHeight, 28);
M3_PIN_AT(m3CharacterDef, maxSlopeAngle, 32);
M3_PIN_AT(m3CharacterDef, snapDistance, 36);
M3_PIN_AT(m3CharacterDef, skin, 40);
M3_PIN_AT(m3CharacterDef, stepHeight, 44);
M3_PIN_AT(m3CharacterDef, mass, 48);
M3_PIN_AT(m3CharacterDef, pushMaxMassRatio, 52);
M3_PIN_AT(m3CharacterDef, userData, 56);
M3_PIN_AT(m3CharacterDef, internalValue, 64);

M3_PIN_SIZE(m3SoftBodyDef, 80);
M3_PIN_AT(m3SoftBodyDef, position, 0);
M3_PIN_AT(m3SoftBodyDef, countX, 24);
M3_PIN_AT(m3SoftBodyDef, countY, 28);
M3_PIN_AT(m3SoftBodyDef, countZ, 32);
M3_PIN_AT(m3SoftBodyDef, spacing, 36);
M3_PIN_AT(m3SoftBodyDef, particleMass, 40);
M3_PIN_AT(m3SoftBodyDef, compliance, 44);
M3_PIN_AT(m3SoftBodyDef, radius, 48);
M3_PIN_AT(m3SoftBodyDef, gravityScale, 52);
M3_PIN_AT(m3SoftBodyDef, userData, 56);
M3_PIN_AT(m3SoftBodyDef, bendCompliance, 64);
M3_PIN_AT(m3SoftBodyDef, pressure, 68);
M3_PIN_AT(m3SoftBodyDef, maxDeviation, 72);
M3_PIN_AT(m3SoftBodyDef, internalValue, 76);

M3_PIN_SIZE(m3VehicleDef, 464);
M3_PIN_AT(m3VehicleDef, chassisId, 0);
M3_PIN_AT(m3VehicleDef, wheelCount, 8);
M3_PIN_AT(m3VehicleDef, wheels, 12);
M3_PIN_AT(m3VehicleDef, maxSteerAngle, 428);
M3_PIN_AT(m3VehicleDef, driveForce, 432);
M3_PIN_AT(m3VehicleDef, brakeForce, 436);
M3_PIN_AT(m3VehicleDef, tireGrip, 440);
M3_PIN_AT(m3VehicleDef, leanStabilization, 444);
M3_PIN_AT(m3VehicleDef, userData, 448);
M3_PIN_AT(m3VehicleDef, internalValue, 456);

M3_PIN_SIZE(m3WheelDef, 52);
M3_PIN_AT(m3WheelDef, anchor, 0);
M3_PIN_AT(m3WheelDef, direction, 12);
M3_PIN_AT(m3WheelDef, restLength, 24);
M3_PIN_AT(m3WheelDef, travel, 28);
M3_PIN_AT(m3WheelDef, hertz, 32);
M3_PIN_AT(m3WheelDef, zeta, 36);
M3_PIN_AT(m3WheelDef, radius, 40);
M3_PIN_AT(m3WheelDef, steerable, 44);
M3_PIN_AT(m3WheelDef, driven, 45);
M3_PIN_AT(m3WheelDef, brakeShare, 48);

M3_PIN_SIZE(m3WaterVolumeDef, 80);
M3_PIN_AT(m3WaterVolumeDef, lo, 0);
M3_PIN_AT(m3WaterVolumeDef, hi, 24);
M3_PIN_AT(m3WaterVolumeDef, density, 48);
M3_PIN_AT(m3WaterVolumeDef, linearDrag, 52);
M3_PIN_AT(m3WaterVolumeDef, angularDrag, 56);
M3_PIN_AT(m3WaterVolumeDef, flow, 60);
M3_PIN_AT(m3WaterVolumeDef, internalValue, 72);

M3_PIN_SIZE(m3ShapeDef, 104);
M3_PIN_AT(m3ShapeDef, density, 0);
M3_PIN_AT(m3ShapeDef, friction, 4);
M3_PIN_AT(m3ShapeDef, restitution, 8);
M3_PIN_AT(m3ShapeDef, categoryBits, 16);
M3_PIN_AT(m3ShapeDef, maskBits, 24);
M3_PIN_AT(m3ShapeDef, groupIndex, 32);
M3_PIN_AT(m3ShapeDef, rollingResistance, 36);
M3_PIN_AT(m3ShapeDef, surfaceVelocity, 40);
M3_PIN_AT(m3ShapeDef, userData, 56);
M3_PIN_AT(m3ShapeDef, isSensor, 64);
M3_PIN_AT(m3ShapeDef, enableHitEvents, 65);
M3_PIN_AT(m3ShapeDef, enablePreSolveEvents, 66);
M3_PIN_AT(m3ShapeDef, localPosition, 68);
M3_PIN_AT(m3ShapeDef, localRotation, 80);
M3_PIN_AT(m3ShapeDef, internalValue, 96);

M3_PIN_SIZE(m3JointDef, 224);
M3_PIN_AT(m3JointDef, type, 0);
M3_PIN_AT(m3JointDef, bodyIdA, 4);
M3_PIN_AT(m3JointDef, bodyIdB, 12);
M3_PIN_AT(m3JointDef, localAnchorA, 20);
M3_PIN_AT(m3JointDef, localAnchorB, 32);
M3_PIN_AT(m3JointDef, localAxisA, 44);
M3_PIN_AT(m3JointDef, localAxisB, 56);
M3_PIN_AT(m3JointDef, enableLimit, 68);
M3_PIN_AT(m3JointDef, lowerLimit, 72);
M3_PIN_AT(m3JointDef, upperLimit, 76);
M3_PIN_AT(m3JointDef, enableMotor, 80);
M3_PIN_AT(m3JointDef, motorSpeed, 84);
M3_PIN_AT(m3JointDef, maxMotorEffort, 88);
M3_PIN_AT(m3JointDef, enableCone, 92);
M3_PIN_AT(m3JointDef, coneAngle, 96);
M3_PIN_AT(m3JointDef, collideConnected, 100);
M3_PIN_AT(m3JointDef, genericLinear, 101);
M3_PIN_AT(m3JointDef, genericAngular, 104);
M3_PIN_AT(m3JointDef, genericMotorAxis, 107);
M3_PIN_AT(m3JointDef, genericReserved, 108);
M3_PIN_AT(m3JointDef, genericLinearLower, 112);
M3_PIN_AT(m3JointDef, genericLinearUpper, 124);
M3_PIN_AT(m3JointDef, genericAngularLower, 136);
M3_PIN_AT(m3JointDef, genericAngularUpper, 148);
M3_PIN_AT(m3JointDef, ratio, 160);
M3_PIN_AT(m3JointDef, groundAnchorA, 168);
M3_PIN_AT(m3JointDef, groundAnchorB, 192);
M3_PIN_AT(m3JointDef, internalValue, 216);

M3_PIN_SIZE(m3DrivetrainDef, 132);
M3_PIN_AT(m3DrivetrainDef, curveCount, 0);
M3_PIN_AT(m3DrivetrainDef, curveRpm, 4);
M3_PIN_AT(m3DrivetrainDef, curveTorque, 36);
M3_PIN_AT(m3DrivetrainDef, gearCount, 68);
M3_PIN_AT(m3DrivetrainDef, gearRatio, 72);
M3_PIN_AT(m3DrivetrainDef, reverseRatio, 96);
M3_PIN_AT(m3DrivetrainDef, finalDrive, 100);
M3_PIN_AT(m3DrivetrainDef, shiftUpRpm, 104);
M3_PIN_AT(m3DrivetrainDef, shiftDownRpm, 108);
M3_PIN_AT(m3DrivetrainDef, clutchSteps, 112);
M3_PIN_AT(m3DrivetrainDef, autoShift, 116);
M3_PIN_AT(m3DrivetrainDef, diffMode, 120);
M3_PIN_AT(m3DrivetrainDef, diffCouple, 124);
M3_PIN_AT(m3DrivetrainDef, internalValue, 128);

M3_PIN_SIZE(m3ExplosionDef, 64);
M3_PIN_AT(m3ExplosionDef, position, 0);
M3_PIN_AT(m3ExplosionDef, filter, 24);
M3_PIN_AT(m3ExplosionDef, radius, 40);
M3_PIN_AT(m3ExplosionDef, falloff, 44);
M3_PIN_AT(m3ExplosionDef, impulsePerArea, 48);
M3_PIN_AT(m3ExplosionDef, voxelCarve, 52);
M3_PIN_AT(m3ExplosionDef, softPush, 56);
M3_PIN_AT(m3ExplosionDef, internalValue, 60);

M3_PIN_SIZE(m3QueryFilter, 16);
M3_PIN_AT(m3QueryFilter, categoryBits, 0);
M3_PIN_AT(m3QueryFilter, maskBits, 8);

// The element types a snapshot walks as they lie in memory, beside the
// header snapshot.c pins.
M3_PIN_SIZE(m3Transform, 40);
M3_PIN_AT(m3Transform, p, 0);
M3_PIN_AT(m3Transform, q, 24);

M3_PIN_SIZE(m3TreeNode, 72);
M3_PIN_AT(m3TreeNode, lo, 0);
M3_PIN_AT(m3TreeNode, hi, 24);
M3_PIN_AT(m3TreeNode, parent, 48);
M3_PIN_AT(m3TreeNode, child1, 52);
M3_PIN_AT(m3TreeNode, child2, 56);
M3_PIN_AT(m3TreeNode, userData, 60);
M3_PIN_AT(m3TreeNode, height, 64);
M3_PIN_AT(m3TreeNode, mask, 68);

M3_PIN_SIZE(m3Pos3, 24);
M3_PIN_AT(m3Pos3, x, 0);
M3_PIN_AT(m3Pos3, y, 8);
M3_PIN_AT(m3Pos3, z, 16);

M3_PIN_SIZE(m3HullData, 5808);
M3_PIN_AT(m3HullData, vertexCount, 0);
M3_PIN_AT(m3HullData, faceCount, 4);
M3_PIN_AT(m3HullData, indexCount, 8);
M3_PIN_AT(m3HullData, unitMass, 12);
M3_PIN_AT(m3HullData, unitCom, 16);
M3_PIN_AT(m3HullData, unitInertiaCom, 28);
M3_PIN_AT(m3HullData, vertices, 64);
M3_PIN_AT(m3HullData, faceNormals, 832);
M3_PIN_AT(m3HullData, faceOffsets, 2320);
M3_PIN_AT(m3HullData, faceVertCounts, 2816);
M3_PIN_AT(m3HullData, faceVertStart, 2940);
M3_PIN_AT(m3HullData, faceIndices, 3188);
M3_PIN_AT(m3HullData, edgeCount, 3560);
M3_PIN_AT(m3HullData, edges, 3564);
M3_PIN_AT(m3HullData, center, 5796);

M3_PIN_SIZE(m3VoxelChunkData, 12808);
M3_PIN_AT(m3VoxelChunkData, cellSize, 0);
M3_PIN_AT(m3VoxelChunkData, filledCount, 4);
M3_PIN_AT(m3VoxelChunkData, occupancy, 8);
M3_PIN_AT(m3VoxelChunkData, payload, 520);
M3_PIN_AT(m3VoxelChunkData, fill, 8712);
