// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's state table and the walks built on it: allocate, free and
// snapshot. The table order is the snapshot byte order.

#include "world_state.h"

#include "allocator.h"
#include "voxel.h"
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
    M3_STATE_ARRAY(bodies.transforms, m3Transform, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.linearVelocities, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.angularVelocities, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.invMass, m3real, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.invInertiaLocal, m3Mat3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.inertiaLocal, m3Mat3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.localCenters, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.gravityScales, m3real, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.linearDamping, m3real, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.angularDamping, m3real, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.types, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.awake, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.sleepTimes, float, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bulletFlags, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.minExtents, float, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.maxExtents, float, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.userData, uint64_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyNames, char, bodyName, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyForce, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyTorque, m3Vec3, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyEnabled, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyLocks, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodySleepThreshold, float, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyCanSleep, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyHasTarget, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyTarget, m3Transform, body, M3_STATE_SNAPSHOT),
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
    M3_STATE_ARRAY(bodies.bodyPool.generations, uint16_t, body,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(bodies.bodyPool.alive, uint8_t, body, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(bodies.bodyPool.freeQueue, int32_t, body, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(bodies.bodyShapeHead, int32_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeBody, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeType, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeGeom, m3ShapeGeom, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeDensity, float, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeFriction, float, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeRestitution, float, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeUserData, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeNext, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapePool.generations, uint16_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapes.shapePool.alive, uint8_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapes.shapePool.freeQueue, int32_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(contacts.pairKeys, uint64_t, pair, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(broadphase.proxyIds, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(broadphase.tree.nodes, m3TreeNode, treeNode,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapes.shapeHullIndex, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(hulls.hullRefCounts, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(hulls.hullPool.generations, uint16_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(hulls.hullPool.alive, uint8_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(hulls.hullPool.freeQueue, int32_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapes.shapeMeshIndex, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeSensor, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeRollingResistance, float, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeHitEvents, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapePreSolve, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeSurfaceVel, m3Vec3, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeLocalPos, m3Vec3, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeLocalRot, m3Quat, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeHasOffset, uint8_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeCategory, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeMask, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeGroup, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(voxels.voxelData, m3VoxelChunkData, voxel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(voxels.voxelRefCounts, int32_t, voxel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(voxels.voxelPool.generations, uint16_t, voxel,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(voxels.voxelPool.alive, uint8_t, voxel, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(voxels.voxelPool.freeQueue, int32_t, voxel,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(shapes.shapeVoxelIndex, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeHfIndex, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(heightFields.hfRefCounts, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(heightFields.hfPool.generations, uint16_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(heightFields.hfPool.alive, uint8_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(heightFields.hfPool.freeQueue, int32_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(characters.charBody, int32_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charRadius, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charHalfHeight, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charCosSlope, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charSnap, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charSkin, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charStepHeight, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charGrounded, uint8_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charGroundNormal, m3Vec3, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charMass, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charPushMax, m3real, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charGroundBody, int32_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charGroundGen, uint16_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charPool.generations, uint16_t, character,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(characters.charPool.alive, uint8_t, character,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(characters.charPool.freeQueue, int32_t, character,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(vehicles.vehChassis, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehChassisGen, uint16_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelCount, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehMaxSteer, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDriveForce, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehBrakeForce, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehUserData, uint64_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelAnchor, m3Vec3, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelDir, m3Vec3, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelRest, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelTravel, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelHertz, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelZeta, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelRadius, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelFlags, uint8_t, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelBrake, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehTrackMode, uint8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehTrackLeft, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehTrackRight, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehLeanGain, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelCompression, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelContact, uint8_t, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehTireGrip, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehThrottle, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehSteer, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehBrake, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelSpin, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtActive, uint8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtCurveCount, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtCurveRpm, m3real, vehicleCurve, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtCurveTorque, m3real, vehicleCurve, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtGearCount, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtGearRatio, m3real, vehicleGear, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtReverse, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtFinal, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtDiffMode, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtDiffCouple, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehWheelLon, m3real, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtShiftUp, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtShiftDown, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtClutchSteps, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtAutoShift, uint8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtGear, int8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtClutch, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtRpm, m3real, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehPool.generations, uint16_t, vehicle,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(vehicles.vehPool.alive, uint8_t, vehicle, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(vehicles.vehPool.freeQueue, int32_t, vehicle,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(softBodies.softParticleCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softEdgeCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softCompliance, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softBendStart, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softBendCompliance, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softDimX, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softDimY, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softDimZ, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softRestVolume, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softPressure, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softTetCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softTetA, uint16_t, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softTetB, uint16_t, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softTetC, uint16_t, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softTetD, uint16_t, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softTetRestV6, m3real, softTet, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softBindPos, m3Pos3, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softMaxDeviation, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softRadius, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softGravityScale, m3real, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softUserData, uint64_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softPos, m3Pos3, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softPrev, m3Pos3, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softInvMass, m3real, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softKick, m3Vec3, softParticle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softEdgeA, uint16_t, softEdge, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softEdgeB, uint16_t, softEdge, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softEdgeRest, m3real, softEdge, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softAnchorCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softAnchorParticle, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softAnchorBody, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softAnchorGen, uint16_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softAnchorLocal, m3Vec3, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softSoftCount, int32_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softSoftParticleA, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softSoftSlotB, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softSoftGenB, uint16_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softSoftParticleB, int32_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softPool.generations, uint16_t, soft,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(softBodies.softPool.alive, uint8_t, soft, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(softBodies.softPool.freeQueue, int32_t, soft,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(meshes.meshRefCounts, int32_t, mesh, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(meshes.meshPool.generations, uint16_t, mesh,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(meshes.meshPool.alive, uint8_t, mesh, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(meshes.meshPool.freeQueue, int32_t, mesh, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(joints.jointType, uint8_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointBodyA, int32_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointBodyB, int32_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointLocalA, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointLocalB, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointCollide, uint8_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointPerpImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointLimitImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointAngularImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointFrameQA, m3Quat, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointFrameQB, m3Quat, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointFlags, uint8_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointMotor, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointBreak, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointSpring, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointTargetScalar, float, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointTargetQ, m3Quat, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointSpringImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointLimits, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointGenericModes, uint16_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointGenLinLower, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointGenLinUpper, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointGenAngLower, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointGenAngUpper, m3Vec3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointGroundA, m3Pos3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointGroundB, m3Pos3, joint, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(water.waterLo, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(water.waterHi, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(water.waterDensity, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(water.waterLinDrag, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(water.waterAngDrag, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE(water.waterFlow, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(water.waterPool.generations, uint16_t, water,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(water.waterPool.alive, uint8_t, water, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(water.waterPool.freeQueue, int32_t, water,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(joints.jointNextA, int32_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointNextB, int32_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.bodyJointHead, int32_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(joints.jointPool.generations, uint16_t, joint,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(joints.jointPool.alive, uint8_t, joint, M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(joints.jointPool.freeQueue, int32_t, joint,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_ARRAY(contacts.manifolds, m3Manifold, pair, M3_STATE_SNAPSHOT),

    // Owned but not snapshot state: derived data, per-slot content with
    // its own walk, event buffers and step scratch.
    M3_STATE_ARRAY(bodies.bodyIsland, int32_t, body, 0),
    M3_STATE_ARRAY(hulls.hullData, m3HullData, shape, 0),
    M3_STATE_ARRAY(heightFields.hfData, m3HeightFieldData, shape, 0),
    M3_STATE_ARRAY(meshes.meshData, m3MeshData, mesh, 0),
    M3_STATE_ARRAY(meshes.meshBvh, m3MeshBvh, mesh, 0),
    M3_STATE_ARRAY(voxels.voxelSurface, m3VoxelSurface, voxel, 0),
    M3_STATE_ARRAY(voxels.voxelShape, int32_t, voxel, 0),
    M3_STATE_ARRAY(voxels.voxelNeighbors, int32_t, voxelFace, 0),
    M3_STATE_ARRAY(events.fragmentEvents, m3FragmentEvent, fragmentEvent, 0),
    M3_STATE_ARRAY(events.fragmentRecipe, uint16_t, fragmentRecipe, 0),
    M3_STATE_ARRAY(events.beginEvents, m3ContactEvent, pair, 0),
    M3_STATE_ARRAY(events.endEvents, m3ContactEvent, pair, 0),
    M3_STATE_ARRAY(events.sensorBeginEvents, m3ContactEvent, pair, 0),
    M3_STATE_ARRAY(events.sensorEndEvents, m3ContactEvent, pair, 0),
    M3_STATE_ARRAY(events.hitEvents, m3HitEvent, pair, 0),
    M3_STATE_ARRAY(events.moveEvents, m3BodyMoveEvent, body, 0),
    M3_STATE_ARRAY(joints.jointBreakEvents, m3JointBreakEvent, joint, 0),
    M3_STATE_ARRAY(contacts.sleepingPairKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(contacts.stepVetoKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(contacts.replayVetoKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(contacts.stashPairKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(contacts.stashManifolds, m3Manifold, pair, 0),
};

static int64_t ExtentCount(const m3World* world, uint8_t extent)
{
    switch (extent)
    {
    case m3_extent_one:
        return 1;
    case m3_extent_body:
        return world->bodies.bodyCapacity;
    case m3_extent_bodyName:
        return (int64_t)world->bodies.bodyCapacity * M3_BODY_NAME_CAPACITY;
    case m3_extent_shape:
        return world->shapes.shapeCapacity;
    case m3_extent_joint:
        return world->joints.jointCapacity;
    case m3_extent_pair:
        return world->contacts.pairCapacity;
    case m3_extent_treeNode:
        return world->broadphase.tree.capacity;
    case m3_extent_voxel:
        return world->voxels.voxelCapacity;
    case m3_extent_voxelFace:
        return (int64_t)world->voxels.voxelCapacity * 6;
    case m3_extent_mesh:
        return world->meshes.meshCapacity;
    case m3_extent_character:
        return world->characters.characterCapacity;
    case m3_extent_vehicle:
        return world->vehicles.vehicleCapacity;
    case m3_extent_vehicleWheel:
        return (int64_t)world->vehicles.vehicleCapacity * M3_VEHICLE_MAX_WHEELS;
    case m3_extent_vehicleCurve:
        return (int64_t)world->vehicles.vehicleCapacity * M3_DRIVETRAIN_MAX_CURVE;
    case m3_extent_vehicleGear:
        return (int64_t)world->vehicles.vehicleCapacity * M3_DRIVETRAIN_MAX_GEARS;
    case m3_extent_soft:
        return world->softBodies.softBodyCapacity;
    case m3_extent_softAnchor:
        return (int64_t)world->softBodies.softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS;
    case m3_extent_softTet:
        return (int64_t)world->softBodies.softBodyCapacity * M3_SOFTBODY_MAX_TETS;
    case m3_extent_softParticle:
        return (int64_t)world->softBodies.softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES;
    case m3_extent_softEdge:
        return (int64_t)world->softBodies.softBodyCapacity * M3_SOFTBODY_MAX_EDGES;
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
