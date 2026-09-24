// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's state table and the walks built on it: allocate, free and
// snapshot. The table order is the snapshot byte order.

#include "world_state.h"

#include "allocator.h"
#include "manifold.h"
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
    // Fixed per-slot bounds, the ranges some checked values live in.
    m3_extent_wheelsPerVehicle,
    m3_extent_curvePerVehicle,
    m3_extent_gearsPerVehicle,
    m3_extent_particlesPerSoft,
    m3_extent_edgesPerSoft,
    m3_extent_tetsPerSoft,
    m3_extent_anchorsPerSoft,
} m3Extent;

// What a restore checks in an incoming block before any of it lands, so
// hostile snapshot bytes can never index out of bounds.
typedef enum m3Check
{
    m3_check_none,
    m3_check_index,       // int32 elements in [0, range)
    m3_check_indexOrNone, // int32 elements in [-1, range)
    m3_check_index16,     // uint16 elements below range
    m3_check_countEach,   // int32 elements in [0, range]
    m3_check_bool,        // uint8 elements 0 or 1
    m3_check_bodyType,    // uint8 elements below the body type count
    m3_check_shapeType,   // uint8 elements below the shape type count
    m3_check_jointType,   // uint8 elements below the joint type count
    m3_check_manifold,    // manifold point counts that fit
    m3_check_pairKey,     // both shape indices of every pair key in range
    m3_check_treeNode,    // tree node links, leaf payloads, finite bounds
    m3_check_voxel,       // voxel chunk: finite cell size, a fill count that fits
    m3_check_finite32,    // every float of every element finite
    m3_check_finite64,    // every double of every element finite
    m3_check_transform,   // a double position and a float rotation, finite
} m3Check;

#define M3_STATE_SNAPSHOT 0x01u // walked by snapshots and restores
#define M3_STATE_POINTER  0x02u // the field points at the array; else it is inline
#define M3_STATE_BORROWED 0x04u // allocated by its owner (an id pool, the tree), only walked

typedef struct m3StateArray
{
    uint32_t offset;      // of the field in m3World
    uint32_t elementSize; // bytes per element (inline: bytes of the field)
    uint8_t extent;       // m3Extent
    uint8_t flags;
    uint8_t check; // m3Check, validated on restore
    uint8_t range; // m3Extent the checked values index into
} m3StateArray;

#define M3_STATE_ARRAY(field, type, ext, stateFlags)                                               \
    {(uint32_t)offsetof(m3World, field),         (uint32_t)sizeof(type), m3_extent_##ext,          \
     (uint8_t)((stateFlags) | M3_STATE_POINTER), m3_check_none,          m3_extent_one}
#define M3_STATE_INLINE(field, stateFlags)                                                         \
    {(uint32_t)offsetof(m3World, field),                                                           \
     (uint32_t)sizeof(((m3World*)0)->field),                                                       \
     m3_extent_one,                                                                                \
     (uint8_t)(stateFlags),                                                                        \
     m3_check_none,                                                                                \
     m3_extent_one}

#define M3_STATE_CHECKED(field, type, ext, stateFlags, checkKind, rangeExtent)                     \
    {(uint32_t)offsetof(m3World, field),         (uint32_t)sizeof(type), m3_extent_##ext,          \
     (uint8_t)((stateFlags) | M3_STATE_POINTER), m3_check_##checkKind,   m3_extent_##rangeExtent}
#define M3_STATE_INLINE_CHECKED(field, stateFlags, checkKind, rangeExtent)                         \
    {(uint32_t)offsetof(m3World, field),                                                           \
     (uint32_t)sizeof(((m3World*)0)->field),                                                       \
     m3_extent_one,                                                                                \
     (uint8_t)(stateFlags),                                                                        \
     m3_check_##checkKind,                                                                         \
     m3_extent_##rangeExtent}

// The cursors of an id pool: its high-water mark, the head and length
// of its FIFO free ring and its retired count. The pool argument names a
// field path, so it cannot be parenthesized.
// NOLINTBEGIN(bugprone-macro-parentheses)
#define M3_STATE_POOL_CURSORS(pool, ext)                                                           \
    M3_STATE_INLINE_CHECKED(pool.maxIndex, M3_STATE_SNAPSHOT, countEach, ext),                     \
        M3_STATE_INLINE_CHECKED(pool.freeHead, M3_STATE_SNAPSHOT, index, ext),                     \
        M3_STATE_INLINE_CHECKED(pool.freeCount, M3_STATE_SNAPSHOT, countEach, ext),                \
        M3_STATE_INLINE_CHECKED(pool.retiredCount, M3_STATE_SNAPSHOT, countEach, ext)
// NOLINTEND(bugprone-macro-parentheses)

static const m3StateArray s_state[] = {
    // Snapshot state, in snapshot byte order. Identity is state: pool
    // generations, liveness and FIFO queues restore exactly, so ids
    // minted after a rollback cannot diverge; the pool cursors restore
    // with them. The broadphase tree is state too, so a rolled-back
    // world continues on the same tree.
    M3_STATE_INLINE(stepCount, M3_STATE_SNAPSHOT),
    M3_STATE_INLINE_CHECKED(gravity, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.transforms, m3Transform, body, M3_STATE_SNAPSHOT, transform, one),
    M3_STATE_CHECKED(bodies.linearVelocities, m3Vec3, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.angularVelocities, m3Vec3, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.invMass, m3real, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.invInertiaLocal, m3Mat3, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.inertiaLocal, m3Mat3, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.localCenters, m3Vec3, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.gravityScales, m3real, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.linearDamping, m3real, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.angularDamping, m3real, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.types, uint8_t, body, M3_STATE_SNAPSHOT, bodyType, one),
    M3_STATE_CHECKED(bodies.awake, uint8_t, body, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(bodies.sleepTimes, float, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.bulletFlags, uint8_t, body, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(bodies.minExtents, float, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.maxExtents, float, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(bodies.userData, uint64_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(bodies.bodyNames, char, bodyName, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(bodies.bodyForce, m3Vec3, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.bodyTorque, m3Vec3, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.bodyEnabled, uint8_t, body, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_ARRAY(bodies.bodyLocks, uint8_t, body, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(bodies.bodySleepThreshold, float, body, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(bodies.bodyCanSleep, uint8_t, body, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(bodies.bodyHasTarget, uint8_t, body, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(bodies.bodyTarget, m3Transform, body, M3_STATE_SNAPSHOT, transform, one),
    M3_STATE_INLINE_CHECKED(contactHertz, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(contactDampingRatio, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(contactPushMaxSpeed, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(restitutionThreshold, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(maximumLinearSpeed, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(maximumAngularSpeed, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(sleepEnabled, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_INLINE_CHECKED(continuousEnabled, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_INLINE_CHECKED(hitEventThreshold, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(windDir, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(windSpeed, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(windGustHertz, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(windGustScale, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(windPhase, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(bodies.bodyPool.generations, uint16_t, body,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(bodies.bodyPool.alive, uint8_t, body, M3_STATE_SNAPSHOT | M3_STATE_BORROWED,
                     bool, one),
    M3_STATE_CHECKED(bodies.bodyPool.freeQueue, int32_t, body,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, body),
    M3_STATE_POOL_CURSORS(bodies.bodyPool, body),
    M3_STATE_CHECKED(bodies.bodyShapeHead, int32_t, body, M3_STATE_SNAPSHOT, indexOrNone, shape),
    M3_STATE_CHECKED(shapes.shapeBody, int32_t, shape, M3_STATE_SNAPSHOT, indexOrNone, body),
    M3_STATE_CHECKED(shapes.shapeType, uint8_t, shape, M3_STATE_SNAPSHOT, shapeType, one),
    M3_STATE_ARRAY(shapes.shapeGeom, m3ShapeGeom, shape, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(shapes.shapeDensity, float, shape, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(shapes.shapeFriction, float, shape, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(shapes.shapeRestitution, float, shape, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(shapes.shapeUserData, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(shapes.shapeNext, int32_t, shape, M3_STATE_SNAPSHOT, indexOrNone, shape),
    M3_STATE_ARRAY(shapes.shapePool.generations, uint16_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(shapes.shapePool.alive, uint8_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED,
                     bool, one),
    M3_STATE_CHECKED(shapes.shapePool.freeQueue, int32_t, shape,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, shape),
    M3_STATE_POOL_CURSORS(shapes.shapePool, shape),
    M3_STATE_INLINE_CHECKED(contacts.pairCount, M3_STATE_SNAPSHOT, countEach, pair),
    M3_STATE_CHECKED(contacts.pairKeys, uint64_t, pair, M3_STATE_SNAPSHOT, pairKey, shape),
    M3_STATE_CHECKED(broadphase.proxyIds, int32_t, shape, M3_STATE_SNAPSHOT, indexOrNone, treeNode),
    M3_STATE_CHECKED(broadphase.tree.nodes, m3TreeNode, treeNode,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, treeNode, treeNode),
    M3_STATE_INLINE_CHECKED(broadphase.tree.root, M3_STATE_SNAPSHOT, indexOrNone, treeNode),
    M3_STATE_INLINE_CHECKED(broadphase.tree.freeList, M3_STATE_SNAPSHOT, indexOrNone, treeNode),
    M3_STATE_INLINE_CHECKED(broadphase.tree.nodeCount, M3_STATE_SNAPSHOT, countEach, treeNode),
    M3_STATE_CHECKED(shapes.shapeHullIndex, int32_t, shape, M3_STATE_SNAPSHOT, indexOrNone, shape),
    M3_STATE_ARRAY(hulls.hullRefCounts, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(hulls.hullPool.generations, uint16_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(hulls.hullPool.alive, uint8_t, shape, M3_STATE_SNAPSHOT | M3_STATE_BORROWED,
                     bool, one),
    M3_STATE_CHECKED(hulls.hullPool.freeQueue, int32_t, shape,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, shape),
    M3_STATE_POOL_CURSORS(hulls.hullPool, shape),
    M3_STATE_CHECKED(shapes.shapeMeshIndex, int32_t, shape, M3_STATE_SNAPSHOT, indexOrNone, mesh),
    M3_STATE_CHECKED(shapes.shapeSensor, uint8_t, shape, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(shapes.shapeRollingResistance, float, shape, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(shapes.shapeHitEvents, uint8_t, shape, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(shapes.shapePreSolve, uint8_t, shape, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(shapes.shapeSurfaceVel, m3Vec3, shape, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(shapes.shapeLocalPos, m3Vec3, shape, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(shapes.shapeLocalRot, m3Quat, shape, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(shapes.shapeHasOffset, uint8_t, shape, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_ARRAY(shapes.shapeCategory, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeMask, uint64_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(shapes.shapeGroup, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(voxels.voxelData, m3VoxelChunkData, voxel, M3_STATE_SNAPSHOT, voxel, one),
    M3_STATE_ARRAY(voxels.voxelRefCounts, int32_t, voxel, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(voxels.voxelPool.generations, uint16_t, voxel,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(voxels.voxelPool.alive, uint8_t, voxel, M3_STATE_SNAPSHOT | M3_STATE_BORROWED,
                     bool, one),
    M3_STATE_CHECKED(voxels.voxelPool.freeQueue, int32_t, voxel,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, voxel),
    M3_STATE_POOL_CURSORS(voxels.voxelPool, voxel),
    M3_STATE_CHECKED(shapes.shapeVoxelIndex, int32_t, shape, M3_STATE_SNAPSHOT, indexOrNone, voxel),
    M3_STATE_CHECKED(shapes.shapeHfIndex, int32_t, shape, M3_STATE_SNAPSHOT, indexOrNone, shape),
    M3_STATE_ARRAY(heightFields.hfRefCounts, int32_t, shape, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(heightFields.hfPool.generations, uint16_t, shape,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(heightFields.hfPool.alive, uint8_t, shape,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, bool, one),
    M3_STATE_CHECKED(heightFields.hfPool.freeQueue, int32_t, shape,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, shape),
    M3_STATE_POOL_CURSORS(heightFields.hfPool, shape),
    M3_STATE_CHECKED(characters.charBody, int32_t, character, M3_STATE_SNAPSHOT, indexOrNone, body),
    M3_STATE_CHECKED(characters.charRadius, m3real, character, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(characters.charHalfHeight, m3real, character, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(characters.charCosSlope, m3real, character, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(characters.charSnap, m3real, character, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(characters.charSkin, m3real, character, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(characters.charStepHeight, m3real, character, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(characters.charGrounded, uint8_t, character, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(characters.charGroundNormal, m3Vec3, character, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(characters.charMass, m3real, character, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(characters.charPushMax, m3real, character, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(characters.charGroundBody, int32_t, character, M3_STATE_SNAPSHOT, indexOrNone,
                     body),
    M3_STATE_ARRAY(characters.charGroundGen, uint16_t, character, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(characters.charPool.generations, uint16_t, character,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(characters.charPool.alive, uint8_t, character,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, bool, one),
    M3_STATE_CHECKED(characters.charPool.freeQueue, int32_t, character,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, character),
    M3_STATE_POOL_CURSORS(characters.charPool, character),
    M3_STATE_CHECKED(vehicles.vehChassis, int32_t, vehicle, M3_STATE_SNAPSHOT, indexOrNone, body),
    M3_STATE_ARRAY(vehicles.vehChassisGen, uint16_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(vehicles.vehWheelCount, int32_t, vehicle, M3_STATE_SNAPSHOT, countEach,
                     wheelsPerVehicle),
    M3_STATE_CHECKED(vehicles.vehMaxSteer, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehDriveForce, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehBrakeForce, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(vehicles.vehUserData, uint64_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(vehicles.vehWheelAnchor, m3Vec3, vehicleWheel, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(vehicles.vehWheelDir, m3Vec3, vehicleWheel, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehWheelRest, m3real, vehicleWheel, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehWheelTravel, m3real, vehicleWheel, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(vehicles.vehWheelHertz, m3real, vehicleWheel, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(vehicles.vehWheelZeta, m3real, vehicleWheel, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehWheelRadius, m3real, vehicleWheel, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_ARRAY(vehicles.vehWheelFlags, uint8_t, vehicleWheel, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(vehicles.vehWheelBrake, m3real, vehicleWheel, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(vehicles.vehTrackMode, uint8_t, vehicle, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(vehicles.vehTrackLeft, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehTrackRight, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehLeanGain, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehWheelCompression, m3real, vehicleWheel, M3_STATE_SNAPSHOT,
                     finite32, one),
    M3_STATE_CHECKED(vehicles.vehWheelContact, uint8_t, vehicleWheel, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(vehicles.vehTireGrip, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehThrottle, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehSteer, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehBrake, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehWheelSpin, m3real, vehicleWheel, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehDtActive, uint8_t, vehicle, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(vehicles.vehDtCurveCount, int32_t, vehicle, M3_STATE_SNAPSHOT, countEach,
                     curvePerVehicle),
    M3_STATE_CHECKED(vehicles.vehDtCurveRpm, m3real, vehicleCurve, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(vehicles.vehDtCurveTorque, m3real, vehicleCurve, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(vehicles.vehDtGearCount, int32_t, vehicle, M3_STATE_SNAPSHOT, countEach,
                     gearsPerVehicle),
    M3_STATE_CHECKED(vehicles.vehDtGearRatio, m3real, vehicleGear, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(vehicles.vehDtReverse, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehDtFinal, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(vehicles.vehDtDiffMode, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(vehicles.vehDtDiffCouple, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehWheelLon, m3real, vehicleWheel, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehDtShiftUp, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(vehicles.vehDtShiftDown, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(vehicles.vehDtClutchSteps, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(vehicles.vehDtAutoShift, uint8_t, vehicle, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_ARRAY(vehicles.vehDtGear, int8_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(vehicles.vehDtClutch, int32_t, vehicle, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(vehicles.vehDtRpm, m3real, vehicle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(vehicles.vehPool.generations, uint16_t, vehicle,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(vehicles.vehPool.alive, uint8_t, vehicle,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, bool, one),
    M3_STATE_CHECKED(vehicles.vehPool.freeQueue, int32_t, vehicle,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, vehicle),
    M3_STATE_POOL_CURSORS(vehicles.vehPool, vehicle),
    M3_STATE_CHECKED(softBodies.softParticleCount, int32_t, soft, M3_STATE_SNAPSHOT, countEach,
                     particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softEdgeCount, int32_t, soft, M3_STATE_SNAPSHOT, countEach,
                     edgesPerSoft),
    M3_STATE_CHECKED(softBodies.softCompliance, m3real, soft, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(softBodies.softBendStart, int32_t, soft, M3_STATE_SNAPSHOT, countEach,
                     edgesPerSoft),
    M3_STATE_CHECKED(softBodies.softBendCompliance, m3real, soft, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(softBodies.softDimX, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softDimY, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(softBodies.softDimZ, uint16_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(softBodies.softRestVolume, m3real, soft, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(softBodies.softPressure, m3real, soft, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(softBodies.softTetCount, int32_t, soft, M3_STATE_SNAPSHOT, countEach,
                     tetsPerSoft),
    M3_STATE_CHECKED(softBodies.softTetA, uint16_t, softTet, M3_STATE_SNAPSHOT, index16,
                     particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softTetB, uint16_t, softTet, M3_STATE_SNAPSHOT, index16,
                     particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softTetC, uint16_t, softTet, M3_STATE_SNAPSHOT, index16,
                     particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softTetD, uint16_t, softTet, M3_STATE_SNAPSHOT, index16,
                     particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softTetRestV6, m3real, softTet, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(softBodies.softBindPos, m3Pos3, softParticle, M3_STATE_SNAPSHOT, finite64,
                     one),
    M3_STATE_CHECKED(softBodies.softMaxDeviation, m3real, soft, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(softBodies.softRadius, m3real, soft, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(softBodies.softGravityScale, m3real, soft, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(softBodies.softUserData, uint64_t, soft, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(softBodies.softPos, m3Pos3, softParticle, M3_STATE_SNAPSHOT, finite64, one),
    M3_STATE_CHECKED(softBodies.softPrev, m3Pos3, softParticle, M3_STATE_SNAPSHOT, finite64, one),
    M3_STATE_CHECKED(softBodies.softInvMass, m3real, softParticle, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(softBodies.softKick, m3Vec3, softParticle, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(softBodies.softEdgeA, uint16_t, softEdge, M3_STATE_SNAPSHOT, index16,
                     particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softEdgeB, uint16_t, softEdge, M3_STATE_SNAPSHOT, index16,
                     particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softEdgeRest, m3real, softEdge, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(softBodies.softAnchorCount, int32_t, soft, M3_STATE_SNAPSHOT, countEach,
                     anchorsPerSoft),
    M3_STATE_CHECKED(softBodies.softAnchorParticle, int32_t, softAnchor, M3_STATE_SNAPSHOT,
                     indexOrNone, particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softAnchorBody, int32_t, softAnchor, M3_STATE_SNAPSHOT, indexOrNone,
                     body),
    M3_STATE_ARRAY(softBodies.softAnchorGen, uint16_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(softBodies.softAnchorLocal, m3Vec3, softAnchor, M3_STATE_SNAPSHOT, finite32,
                     one),
    M3_STATE_CHECKED(softBodies.softSoftCount, int32_t, soft, M3_STATE_SNAPSHOT, countEach,
                     anchorsPerSoft),
    M3_STATE_CHECKED(softBodies.softSoftParticleA, int32_t, softAnchor, M3_STATE_SNAPSHOT,
                     indexOrNone, particlesPerSoft),
    M3_STATE_CHECKED(softBodies.softSoftSlotB, int32_t, softAnchor, M3_STATE_SNAPSHOT, indexOrNone,
                     soft),
    M3_STATE_ARRAY(softBodies.softSoftGenB, uint16_t, softAnchor, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(softBodies.softSoftParticleB, int32_t, softAnchor, M3_STATE_SNAPSHOT,
                     indexOrNone, particlesPerSoft),
    M3_STATE_ARRAY(softBodies.softPool.generations, uint16_t, soft,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(softBodies.softPool.alive, uint8_t, soft,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, bool, one),
    M3_STATE_CHECKED(softBodies.softPool.freeQueue, int32_t, soft,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, soft),
    M3_STATE_POOL_CURSORS(softBodies.softPool, soft),
    M3_STATE_ARRAY(meshes.meshRefCounts, int32_t, mesh, M3_STATE_SNAPSHOT),
    M3_STATE_ARRAY(meshes.meshPool.generations, uint16_t, mesh,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(meshes.meshPool.alive, uint8_t, mesh, M3_STATE_SNAPSHOT | M3_STATE_BORROWED,
                     bool, one),
    M3_STATE_CHECKED(meshes.meshPool.freeQueue, int32_t, mesh,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, mesh),
    M3_STATE_POOL_CURSORS(meshes.meshPool, mesh),
    M3_STATE_CHECKED(joints.jointType, uint8_t, joint, M3_STATE_SNAPSHOT, jointType, one),
    M3_STATE_CHECKED(joints.jointBodyA, int32_t, joint, M3_STATE_SNAPSHOT, indexOrNone, body),
    M3_STATE_CHECKED(joints.jointBodyB, int32_t, joint, M3_STATE_SNAPSHOT, indexOrNone, body),
    M3_STATE_CHECKED(joints.jointLocalA, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointLocalB, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointCollide, uint8_t, joint, M3_STATE_SNAPSHOT, bool, one),
    M3_STATE_CHECKED(joints.jointImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointPerpImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointLimitImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointAngularImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointFrameQA, m3Quat, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointFrameQB, m3Quat, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(joints.jointFlags, uint8_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(joints.jointMotor, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointBreak, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointSpring, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointTargetScalar, float, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointTargetQ, m3Quat, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointSpringImpulse, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointLimits, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(joints.jointGenericModes, uint16_t, joint, M3_STATE_SNAPSHOT),
    M3_STATE_CHECKED(joints.jointGenLinLower, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointGenLinUpper, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointGenAngLower, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointGenAngUpper, m3Vec3, joint, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_CHECKED(joints.jointGroundA, m3Pos3, joint, M3_STATE_SNAPSHOT, finite64, one),
    M3_STATE_CHECKED(joints.jointGroundB, m3Pos3, joint, M3_STATE_SNAPSHOT, finite64, one),
    M3_STATE_INLINE_CHECKED(water.waterLo, M3_STATE_SNAPSHOT, finite64, one),
    M3_STATE_INLINE_CHECKED(water.waterHi, M3_STATE_SNAPSHOT, finite64, one),
    M3_STATE_INLINE_CHECKED(water.waterDensity, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(water.waterLinDrag, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(water.waterAngDrag, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_INLINE_CHECKED(water.waterFlow, M3_STATE_SNAPSHOT, finite32, one),
    M3_STATE_ARRAY(water.waterPool.generations, uint16_t, water,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(water.waterPool.alive, uint8_t, water, M3_STATE_SNAPSHOT | M3_STATE_BORROWED,
                     bool, one),
    M3_STATE_CHECKED(water.waterPool.freeQueue, int32_t, water,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, water),
    M3_STATE_POOL_CURSORS(water.waterPool, water),
    M3_STATE_CHECKED(joints.jointNextA, int32_t, joint, M3_STATE_SNAPSHOT, indexOrNone, joint),
    M3_STATE_CHECKED(joints.jointNextB, int32_t, joint, M3_STATE_SNAPSHOT, indexOrNone, joint),
    M3_STATE_CHECKED(joints.bodyJointHead, int32_t, body, M3_STATE_SNAPSHOT, indexOrNone, joint),
    M3_STATE_ARRAY(joints.jointPool.generations, uint16_t, joint,
                   M3_STATE_SNAPSHOT | M3_STATE_BORROWED),
    M3_STATE_CHECKED(joints.jointPool.alive, uint8_t, joint, M3_STATE_SNAPSHOT | M3_STATE_BORROWED,
                     bool, one),
    M3_STATE_CHECKED(joints.jointPool.freeQueue, int32_t, joint,
                     M3_STATE_SNAPSHOT | M3_STATE_BORROWED, index, joint),
    M3_STATE_POOL_CURSORS(joints.jointPool, joint),
    M3_STATE_CHECKED(contacts.manifolds, m3Manifold, pair, M3_STATE_SNAPSHOT, manifold, one),

    // Owned but not snapshot state: derived data, per-slot content with
    // its own walk, event buffers and step scratch.
    M3_STATE_ARRAY(bodies.bodyIsland, int32_t, body, 0),
    M3_STATE_ARRAY(shapes.planeShapes, int32_t, shape, 0),
    M3_STATE_ARRAY(broadphase.candidateKeys, uint64_t, pair, 0),
    M3_STATE_ARRAY(broadphase.moved, uint8_t, shape, 0),
    M3_STATE_ARRAY(hulls.hullData, m3HullData, shape, 0),
    M3_STATE_ARRAY(heightFields.hfData, m3HeightFieldData, shape, 0),
    M3_STATE_ARRAY(meshes.meshData, m3MeshData, mesh, 0),
    M3_STATE_ARRAY(meshes.meshBvh, m3MeshBvh, mesh, 0),
    M3_STATE_ARRAY(voxels.voxelSurface, m3VoxelSurface, voxel, 0),
    M3_STATE_ARRAY(voxels.voxelShape, int32_t, voxel, 0),
    M3_STATE_ARRAY(voxels.voxelNeighbors, int32_t, voxelFace, 0),
    M3_STATE_ARRAY(events.fragmentEvents, m3FragmentEvent, fragmentEvent, 0),
    M3_STATE_ARRAY(events.fragmentRecipe, uint16_t, fragmentRecipe, 0),
    M3_STATE_ARRAY(events.beginEvents, m3ContactBeginEvent, pair, 0),
    M3_STATE_ARRAY(events.endEvents, m3ContactEndEvent, pair, 0),
    M3_STATE_ARRAY(events.sensorBeginEvents, m3ContactBeginEvent, pair, 0),
    M3_STATE_ARRAY(events.sensorEndEvents, m3ContactEndEvent, pair, 0),
    M3_STATE_ARRAY(events.hitEvents, m3ContactHitEvent, pair, 0),
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
    case m3_extent_wheelsPerVehicle:
        return M3_VEHICLE_MAX_WHEELS;
    case m3_extent_curvePerVehicle:
        return M3_DRIVETRAIN_MAX_CURVE;
    case m3_extent_gearsPerVehicle:
        return M3_DRIVETRAIN_MAX_GEARS;
    case m3_extent_particlesPerSoft:
        return M3_SOFTBODY_MAX_PARTICLES;
    case m3_extent_edgesPerSoft:
        return M3_SOFTBODY_MAX_EDGES;
    case m3_extent_tetsPerSoft:
        return M3_SOFTBODY_MAX_TETS;
    case m3_extent_anchorsPerSoft:
        return M3_SOFTBODY_MAX_ANCHORS;
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

static bool InRange(int64_t value, int64_t lo, int64_t hi)
{
    return value >= lo && value <= hi;
}

static bool CheckInt32s(const uint8_t* data, size_t count, int64_t lo, int64_t hi)
{
    for (size_t i = 0; i < count; ++i)
    {
        int32_t value;
        memcpy(&value, data + 4 * i, sizeof(value));
        if (!InRange(value, lo, hi))
        {
            return false;
        }
    }
    return true;
}

static bool CheckUint16s(const uint8_t* data, size_t count, int64_t range)
{
    for (size_t i = 0; i < count; ++i)
    {
        uint16_t value;
        memcpy(&value, data + 2 * i, sizeof(value));
        if ((int64_t)value >= range)
        {
            return false;
        }
    }
    return true;
}

static bool CheckBytes(const uint8_t* data, size_t count, uint8_t max)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (data[i] > max)
        {
            return false;
        }
    }
    return true;
}

static bool CheckFinite(const uint8_t* data, size_t bytes, size_t width)
{
    for (size_t at = 0; at + width <= bytes; at += width)
    {
        bool finite;
        if (width == sizeof(float))
        {
            float f;
            memcpy(&f, data + at, sizeof(f));
            finite = m3FiniteF(f);
        }
        else
        {
            double d;
            memcpy(&d, data + at, sizeof(d));
            finite = m3FiniteD(d);
        }
        if (!finite)
        {
            return false;
        }
    }
    return true;
}

static bool CheckStructs(const m3World* world, const m3StateArray* array, const uint8_t* data,
                         size_t count, int64_t range)
{
    for (size_t i = 0; i < count; ++i)
    {
        const uint8_t* at = data + i * array->elementSize;
        bool ok = true;
        switch ((m3Check)array->check)
        {
        case m3_check_manifold:
        {
            m3Manifold m;
            memcpy(&m, at, sizeof(m));
            ok = InRange(m.pointCount, 0, M3_MANIFOLD_MAX_POINTS);
            break;
        }
        case m3_check_pairKey:
        {
            uint64_t key;
            memcpy(&key, at, sizeof(key));
            ok = (int64_t)(key >> 32) < range && (int64_t)(key & 0xFFFFFFFFu) < range;
            break;
        }
        case m3_check_treeNode:
        {
            m3TreeNode node;
            memcpy(&node, at, sizeof(node));
            ok = InRange(node.parent, -1, range - 1) && InRange(node.child1, -1, range - 1) &&
                 InRange(node.child2, -1, range - 1) &&
                 InRange(node.userData, -1, world->shapes.shapeCapacity - 1) &&
                 CheckFinite(at, 6 * sizeof(double), sizeof(double));
            break;
        }
        case m3_check_voxel:
        {
            m3VoxelChunkData chunk;
            memcpy(&chunk.cellSize, at + offsetof(m3VoxelChunkData, cellSize), sizeof(m3real));
            memcpy(&chunk.filledCount, at + offsetof(m3VoxelChunkData, filledCount),
                   sizeof(int32_t));
            ok = m3FiniteF(chunk.cellSize) && InRange(chunk.filledCount, 0, M3_VOXEL_COUNT);
            break;
        }
        case m3_check_transform:
        {
            m3Transform xf;
            memcpy(&xf, at, sizeof(xf));
            ok = m3FinitePos3(xf.p) && m3FiniteQuat(xf.q);
            break;
        }
        default:
            ok = false;
            break;
        }
        if (!ok)
        {
            return false;
        }
    }
    return true;
}

static bool CheckBlock(const m3World* world, const m3StateArray* array, const uint8_t* data,
                       size_t count)
{
    int64_t range = ExtentCount(world, array->range);
    switch ((m3Check)array->check)
    {
    case m3_check_none:
        return true;
    case m3_check_index:
        return CheckInt32s(data, count, 0, range - 1);
    case m3_check_indexOrNone:
        return CheckInt32s(data, count, -1, range - 1);
    case m3_check_index16:
        return CheckUint16s(data, count, range);
    case m3_check_countEach:
        return CheckInt32s(data, count, 0, range);
    case m3_check_bool:
        return CheckBytes(data, count, 1);
    case m3_check_bodyType:
        return CheckBytes(data, count, (uint8_t)m3_dynamicBody);
    case m3_check_shapeType:
        return CheckBytes(data, count, (uint8_t)(M3_SHAPE_TYPE_COUNT - 1));
    case m3_check_jointType:
        return CheckBytes(data, count, (uint8_t)m3_pulleyJoint);
    case m3_check_finite32:
        return CheckFinite(data, count * array->elementSize, sizeof(float));
    case m3_check_finite64:
        return CheckFinite(data, count * array->elementSize, sizeof(double));
    case m3_check_manifold:
    case m3_check_pairKey:
    case m3_check_treeNode:
    case m3_check_voxel:
    case m3_check_transform:
        return CheckStructs(world, array, data, count, range);
    }
    return false;
}

bool m3StateValidate(const m3World* world, const uint8_t* in)
{
    size_t cursor = 0;
    int32_t count = (int32_t)(sizeof(s_state) / sizeof(s_state[0]));
    for (int32_t i = 0; i < count; ++i)
    {
        const m3StateArray* entry = &s_state[i];
        if ((entry->flags & M3_STATE_SNAPSHOT) == 0)
        {
            continue;
        }
        bool pointer = (entry->flags & M3_STATE_POINTER) != 0;
        size_t elements = pointer ? (size_t)ExtentCount(world, entry->extent) : 1;
        if (!CheckBlock(world, entry, in + cursor, elements))
        {
            return false;
        }
        cursor += elements * entry->elementSize;
    }
    return true;
}
