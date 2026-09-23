// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's state table and the walks built on it: allocate, free and
// snapshot. The table order is the snapshot byte order.

#include "world_state.h"

#include "allocator.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

// How many elements an array holds, as a function of the capacities.
typedef enum m3Extent
{
    m3_extent_one,
    m3_extent_body,
    m3_extent_bodyName,
    m3_extent_shape,
    m3_extent_joint,
    m3_extent_pair,
    m3_extent_treeNode,
    m3_extent_voxel,
    m3_extent_voxelFace,
    m3_extent_mesh,
    m3_extent_character,
    m3_extent_vehicle,
    m3_extent_vehicleWheel,
    m3_extent_vehicleCurve,
    m3_extent_vehicleGear,
    m3_extent_soft,
    m3_extent_softAnchor,
    m3_extent_softTet,
    m3_extent_softParticle,
    m3_extent_softEdge,
    m3_extent_water,
    m3_extent_fragmentEvent,
    m3_extent_fragmentRecipe,
} m3Extent;

#define M3_STATE_SNAPSHOT 0x01u // walked by snapshots and restores
#define M3_STATE_POINTER  0x02u // the field points at the array; else it is inline
#define M3_STATE_BORROWED 0x04u // allocated by its owner (an id pool, the tree), only walked

typedef struct m3StateArray
{
    uint32_t offset;      // of the field in m3World
    uint32_t elementSize; // bytes per element (inline: bytes of the field)
    uint8_t extent;       // m3Extent
    uint8_t flags;
} m3StateArray;

#define M3_STATE_ARRAY(field, type, ext, stateFlags)                                               \
    {(uint32_t)offsetof(m3World, field), (uint32_t)sizeof(type), m3_extent_##ext,                  \
     (uint8_t)((stateFlags) | M3_STATE_POINTER)}
#define M3_STATE_INLINE(field, stateFlags)                                                         \
    {(uint32_t)offsetof(m3World, field), (uint32_t)sizeof(((m3World*)0)->field), m3_extent_one,    \
     (uint8_t)(stateFlags)}

static const m3StateArray s_state[] = {
    // Snapshot state, in snapshot byte order. Identity is state: pool
    // generations, liveness and FIFO queues restore exactly, so ids
    // minted after a rollback cannot diverge; the broadphase tree is
    // state too, so a rolled-back world continues on the same tree.
    M3_STATE_ARRAY(transforms, m3Transform, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(linearVelocities, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(angularVelocities, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(invMass, m3real, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(invInertiaLocal, m3Mat3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(inertiaLocal, m3Mat3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(localCenters, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(gravityScales, m3real, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(linearDamping, m3real, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(angularDamping, m3real, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(types, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(awake, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(sleepTimes, float, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bulletFlags, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(minExtents, float, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(maxExtents, float, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(userData, uint64_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyNames, char, bodyName, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyForce, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyTorque, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyEnabled, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyLocks, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodySleepThreshold, float, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyCanSleep, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyHasTarget, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyTarget, m3Transform, body, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(contactHertz, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(contactDampingRatio, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(contactPushMaxSpeed, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(restitutionThreshold, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(maximumLinearSpeed, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(maximumAngularSpeed, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(sleepEnabled, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(continuousEnabled, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(hitEventThreshold, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(windDir, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(windSpeed, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(windGustHertz, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(windGustScale, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(windPhase, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyPool.generations, uint16_t, body, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(bodyPool.alive, uint8_t, body, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(bodyPool.freeQueue, int32_t, body, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(bodyShapeHead, int32_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeBody, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeType, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeGeom, m3ShapeGeom, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeDensity, float, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeFriction, float, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeRestitution, float, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeUserData, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeNext, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapePool.generations, uint16_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapePool.alive, uint8_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapePool.freeQueue, int32_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(pairKeys, uint64_t, pair, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(proxyIds, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(tree.nodes, m3TreeNode, treeNode, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapeHullIndex, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(hullRefCounts, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(hullPool.generations, uint16_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(hullPool.alive, uint8_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(hullPool.freeQueue, int32_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapeMeshIndex, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeSensor, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeRollingResistance, float, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeHitEvents, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapePreSolve, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeSurfaceVel, m3Vec3, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeLocalPos, m3Vec3, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeLocalRot, m3Quat, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeHasOffset, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeCategory, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeMask, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeGroup, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(voxelData, m3VoxelChunkData, voxel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(voxelRefCounts, int32_t, voxel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(voxelPool.generations, uint16_t, voxel, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(voxelPool.alive, uint8_t, voxel, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(voxelPool.freeQueue, int32_t, voxel, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapeVoxelIndex, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapeHfIndex, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(hfRefCounts, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(hfPool.generations, uint16_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(hfPool.alive, uint8_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(hfPool.freeQueue, int32_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(charBody, int32_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charRadius, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charHalfHeight, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charCosSlope, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charSnap, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charSkin, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charStepHeight, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charGrounded, uint8_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charGroundNormal, m3Vec3, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charMass, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charPushMax, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charGroundBody, int32_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charGroundGen, uint16_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(charPool.generations, uint16_t, character,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(charPool.alive, uint8_t, character, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(charPool.freeQueue, int32_t, character, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(vehChassis, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehChassisGen, uint16_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelCount, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehMaxSteer, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDriveForce, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehBrakeForce, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehUserData, uint64_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelAnchor, m3Vec3, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelDir, m3Vec3, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelRest, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelTravel, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelHertz, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelZeta, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelRadius, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelFlags, uint8_t, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelBrake, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehTrackMode, uint8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehTrackLeft, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehTrackRight, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehLeanGain, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelCompression, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelContact, uint8_t, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehTireGrip, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehThrottle, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehSteer, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehBrake, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelSpin, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtActive, uint8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtCurveCount, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtCurveRpm, m3real, vehicleCurve, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtCurveTorque, m3real, vehicleCurve, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtGearCount, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtGearRatio, m3real, vehicleGear, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtReverse, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtFinal, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtDiffMode, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtDiffCouple, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehWheelLon, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtShiftUp, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtShiftDown, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtClutchSteps, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtAutoShift, uint8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtGear, int8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtClutch, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehDtRpm, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehPool.generations, uint16_t, vehicle, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(vehPool.alive, uint8_t, vehicle, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(vehPool.freeQueue, int32_t, vehicle, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(softParticleCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softEdgeCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softCompliance, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBendStart, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBendCompliance, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softDimX, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softDimY, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softDimZ, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softRestVolume, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softPressure, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softTetCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softTetA, uint16_t, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softTetB, uint16_t, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softTetC, uint16_t, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softTetD, uint16_t, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softTetRestV6, m3real, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBindPos, m3Pos3, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softMaxDeviation, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softRadius, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softGravityScale, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softUserData, uint64_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softPos, m3Pos3, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softPrev, m3Pos3, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softInvMass, m3real, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softKick, m3Vec3, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softEdgeA, uint16_t, softEdge, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softEdgeB, uint16_t, softEdge, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softEdgeRest, m3real, softEdge, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softAnchorCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softAnchorParticle, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softAnchorBody, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softAnchorGen, uint16_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softAnchorLocal, m3Vec3, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softSoftCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softSoftParticleA, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softSoftSlotB, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softSoftGenB, uint16_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softSoftParticleB, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softPool.generations, uint16_t, soft, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(softPool.alive, uint8_t, soft, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(softPool.freeQueue, int32_t, soft, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(meshRefCounts, int32_t, mesh, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(meshPool.generations, uint16_t, mesh, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(meshPool.alive, uint8_t, mesh, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(meshPool.freeQueue, int32_t, mesh, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(jointType, uint8_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointBodyA, int32_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointBodyB, int32_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointLocalA, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointLocalB, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointCollide, uint8_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointPerpImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointLimitImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointAngularImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointFrameQA, m3Quat, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointFrameQB, m3Quat, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointFlags, uint8_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointMotor, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointBreak, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointSpring, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointTargetScalar, float, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointTargetQ, m3Quat, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointSpringImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointLimits, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointGenericModes, uint16_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointGenLinLower, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointGenLinUpper, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointGenAngLower, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointGenAngUpper, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointGroundA, m3Pos3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointGroundB, m3Pos3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(waterLo, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(waterHi, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(waterDensity, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(waterLinDrag, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(waterAngDrag, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(waterFlow, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(waterPool.generations, uint16_t, water, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(waterPool.alive, uint8_t, water, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(waterPool.freeQueue, int32_t, water, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(jointNextA, int32_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointNextB, int32_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodyJointHead, int32_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(jointPool.generations, uint16_t, joint, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(jointPool.alive, uint8_t, joint, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(jointPool.freeQueue, int32_t, joint, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(manifolds, m3Manifold, pair, M3_STATE_SNAPSHOT),

    // Owned but not snapshot state: derived data, per-slot content with
    // its own walk, event buffers and step scratch.
    M3_STATE_ARRAY(bodyIsland, int32_t, body, 0),
    M3_STATE_ARRAY(hullData, m3HullData, shape, 0),
    M3_STATE_ARRAY(hfData, m3HeightFieldData, shape, 0),
    M3_STATE_ARRAY(meshData, m3MeshData, mesh, 0),
    M3_STATE_ARRAY(meshBvh, m3MeshBvh, mesh, 0),
    M3_STATE_ARRAY(voxelSurface, m3VoxelSurface, voxel, 0),
    M3_STATE_ARRAY(voxelShape, int32_t, voxel, 0),
    M3_STATE_ARRAY(voxelNeighbors, int32_t, voxelFace, 0),
    M3_STATE_ARRAY(fragmentEvents, m3FragmentEvent, fragmentEvent, 0),
    M3_STATE_ARRAY(fragmentRecipe, uint16_t, fragmentRecipe, 0),
    M3_STATE_ARRAY(beginEvents, m3ContactEvent, pair, 0),
    M3_STATE_ARRAY(endEvents, m3ContactEvent, pair, 0),
    M3_STATE_ARRAY(sensorBeginEvents, m3ContactEvent, pair, 0),
    M3_STATE_ARRAY(sensorEndEvents, m3ContactEvent, pair, 0),
    M3_STATE_ARRAY(hitEvents, m3HitEvent, pair, 0),
    M3_STATE_ARRAY(moveEvents, m3BodyMoveEvent, body, 0),
    M3_STATE_ARRAY(jointBreakEvents, m3JointBreakEvent, joint, 0),
    M3_STATE_ARRAY(sleepingPairKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(stepVetoKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(replayVetoKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(stashPairKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(stashManifolds, m3Manifold, pair, 0),
};

static int64_t ExtentCount(const m3World* world, uint8_t extent)
{
    switch (extent)
    {
    case m3_extent_one:
        return 1;
    case m3_extent_body:
        return world->bodyCapacity;
    case m3_extent_bodyName:
        return (int64_t)world->bodyCapacity * M3_BODY_NAME_CAPACITY;
    case m3_extent_shape:
        return world->shapeCapacity;
    case m3_extent_joint:
        return world->jointCapacity;
    case m3_extent_pair:
        return world->pairCapacity;
    case m3_extent_treeNode:
        return world->tree.capacity;
    case m3_extent_voxel:
        return world->voxelCapacity;
    case m3_extent_voxelFace:
        return (int64_t)world->voxelCapacity * 6;
    case m3_extent_mesh:
        return world->meshCapacity;
    case m3_extent_character:
        return world->characterCapacity;
    case m3_extent_vehicle:
        return world->vehicleCapacity;
    case m3_extent_vehicleWheel:
        return (int64_t)world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS;
    case m3_extent_vehicleCurve:
        return (int64_t)world->vehicleCapacity * M3_DRIVETRAIN_MAX_CURVE;
    case m3_extent_vehicleGear:
        return (int64_t)world->vehicleCapacity * M3_DRIVETRAIN_MAX_GEARS;
    case m3_extent_soft:
        return world->softBodyCapacity;
    case m3_extent_softAnchor:
        return (int64_t)world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS;
    case m3_extent_softTet:
        return (int64_t)world->softBodyCapacity * M3_SOFTBODY_MAX_TETS;
    case m3_extent_softParticle:
        return (int64_t)world->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES;
    case m3_extent_softEdge:
        return (int64_t)world->softBodyCapacity * M3_SOFTBODY_MAX_EDGES;
    case m3_extent_water:
        return M3_MAX_WATER_VOLUMES;
    case m3_extent_fragmentEvent:
        return M3_FRAGMENT_EVENT_CAP;
    case m3_extent_fragmentRecipe:
        return M3_FRAGMENT_RECIPE_CAP;
    default:
        M3_ASSERT(false);
        return 0;
    }
}

static void** FieldPointer(m3World* world, const m3StateArray* entry)
{
    return (void**)((uint8_t*)world + entry->offset);
}

bool m3StateAllocate(m3World* world)
{
    int32_t count = (int32_t)(sizeof(s_state) / sizeof(s_state[0]));
    for (int32_t i = 0; i < count; ++i)
    {
        const m3StateArray* entry = &s_state[i];
        if ((entry->flags & M3_STATE_POINTER) == 0 || (entry->flags & M3_STATE_BORROWED) != 0)
        {
            continue;
        }
        int64_t elements = ExtentCount(world, entry->extent);
        void* array = m3AllocArray(elements, (int64_t)entry->elementSize);
        if (array == NULL)
        {
            return false;
        }
        *FieldPointer(world, entry) = array;
        world->memoryBytes += elements * (int64_t)entry->elementSize;
    }
    return true;
}

void m3StateFree(m3World* world)
{
    int32_t count = (int32_t)(sizeof(s_state) / sizeof(s_state[0]));
    for (int32_t i = 0; i < count; ++i)
    {
        const m3StateArray* entry = &s_state[i];
        if ((entry->flags & M3_STATE_POINTER) == 0 || (entry->flags & M3_STATE_BORROWED) != 0)
        {
            continue;
        }
        void** field = FieldPointer(world, entry);
        m3Free(*field);
        *field = NULL;
    }
}

int32_t m3StateWalk(m3World* world, uint8_t* out, const uint8_t* in, int direction)
{
    int32_t cursor = 0;
    int32_t count = (int32_t)(sizeof(s_state) / sizeof(s_state[0]));
    for (int32_t i = 0; i < count; ++i)
    {
        const m3StateArray* entry = &s_state[i];
        if ((entry->flags & M3_STATE_SNAPSHOT) == 0)
        {
            continue;
        }
        int32_t bytes = (int32_t)(ExtentCount(world, entry->extent) * (int64_t)entry->elementSize);
        uint8_t* data = (entry->flags & M3_STATE_POINTER) != 0
                            ? (uint8_t*)*FieldPointer(world, entry)
                            : (uint8_t*)world + entry->offset;
        if (direction == 0)
        {
            memcpy(out + cursor, data, (size_t)bytes);
        }
        else if (direction == 1)
        {
            memcpy(data, in + cursor, (size_t)bytes);
        }
        cursor += bytes;
    }
    return cursor;
}
