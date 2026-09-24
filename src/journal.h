// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The command journal's vocabulary and entry points: the op codes, the
// recorder, and the replay worker behind m3World_ReplayJournal.

#ifndef MAUL3D_SRC_JOURNAL_H
#define MAUL3D_SRC_JOURNAL_H

#include "world_internal.h"

// Journal ops. The stream is [i32 op][i32 size][payload], replayed
// through the same internal functions the public API uses.
typedef enum m3Op
{
    m3_opStep = 1, // reserved
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
    m3_opDestroyShape = 58,           // shape id
    m3_opSoftBodyAnchorSoft = 59,     // lattice<->lattice pin
    m3_opSetWind = 60,                // world wind field
    m3_opSetSurfaceVelocity = 61,     // shape conveyor velocity
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
    m3_opVehicleTankCommands = 78,    // skid steer
    // The pre-solve vetoes a step's callback made,
    // recorded BEFORE that step's own op so a bare replay applies
    // them without the host's callback. Payload: the vetoed pair
    // keys, canonical ascending (count = bytes / 8).
    m3_opStepVetoes = 79,
    m3_opSetFilter = 80,        // shape + category, mask and group
    m3_opSetBodyParam = 81,     // body + parameter + value
    m3_opSetBullet = 82,        // body + 0/1
    m3_opSetBodyUserData = 83,  // body + user data
    m3_opSetShapeUserData = 84, // shape + user data
    m3_opCount
} m3Op;

// Payloads: one struct per op family, shared by the recorders and the
// replay. Recorders zero the struct before filling it, so padding bytes
// are deterministic.
typedef struct m3OpBodyByte
{
    m3BodyId id;
    int32_t value;
} m3OpBodyByte;

// The body parameters one op carries.
typedef enum m3BodyParam
{
    m3_bodyParamGravityScale = 0,
    m3_bodyParamLinearDamping = 1,
    m3_bodyParamAngularDamping = 2,
} m3BodyParam;

typedef struct m3OpBodyParam
{
    m3BodyId id;
    int32_t param; // m3BodyParam
    float value;
} m3OpBodyParam;

typedef struct m3OpUserData
{
    uint64_t userData;
    m3BodyId id; // a shape id for m3_opSetShapeUserData: same layout
} m3OpUserData;

typedef struct m3OpBodyPose
{
    m3BodyId id;
    m3Transform pose;
} m3OpBodyPose;

typedef struct m3OpBodyVector
{
    m3BodyId id;
    m3Vec3 v;
} m3OpBodyVector;

typedef struct m3OpBodyVectorAtPoint
{
    m3BodyId id;
    m3Vec3 v;
    m3Pos3 p;
} m3OpBodyVectorAtPoint;

typedef struct m3OpCharacterMove
{
    m3CharacterId id;
    m3Vec3 translation;
} m3OpCharacterMove;

typedef struct m3OpCharacterStance
{
    m3CharacterId id;
    m3real halfHeight;
    m3real radius;
} m3OpCharacterStance;

typedef struct m3OpCreateBody
{
    m3BodyDef def;
    m3BodyId expected;
} m3OpCreateBody;

typedef struct m3OpCreateCharacter
{
    m3CharacterDef def;
    m3CharacterId expected;
} m3OpCreateCharacter;

typedef struct m3OpCreateSoftBody
{
    m3SoftBodyDef def;
    m3SoftBodyId expected;
} m3OpCreateSoftBody;

typedef struct m3OpCreateVehicle
{
    m3VehicleDef def;
    m3VehicleId expected;
} m3OpCreateVehicle;

typedef struct m3OpCreateVoxelChunkShape
{
    m3ShapeDef def;
    m3BodyId body;
    m3ShapeId expected;
    m3real cellSize;
} m3OpCreateVoxelChunkShape;

typedef struct m3OpCreateWaterVolume
{
    m3WaterVolumeDef def;
    m3WaterVolumeId expected;
} m3OpCreateWaterVolume;

typedef struct m3OpJointSetBreak
{
    m3JointId id;
    float maxForce;
    float maxTorque;
} m3OpJointSetBreak;

typedef struct m3OpJointSetCollide
{
    m3JointId id;
    int32_t on;
} m3OpJointSetCollide;

typedef struct m3OpJointSetMotorPose
{
    m3JointId id;
    m3Vec3 offset;
    m3Quat rotation;
} m3OpJointSetMotorPose;

typedef struct m3OpJointSetSpring
{
    m3JointId id;
    int32_t enable;
    float hertz;
    float zeta;
} m3OpJointSetSpring;

typedef struct m3OpJointSetSteer
{
    m3JointId id;
    int32_t enable;
    float target;
    float hertz;
    float zeta;
    float effort;
} m3OpJointSetSteer;

typedef struct m3OpJointSetTarget
{
    m3JointId id;
    float scalar;
    m3Quat q;
} m3OpJointSetTarget;

typedef struct m3OpJointVector
{
    m3JointId id;
    int32_t enable;
    float a;
    float b;
} m3OpJointVector;

typedef struct m3OpSetAllowFastRotation
{
    m3BodyId id;
    uint32_t allow;
} m3OpSetAllowFastRotation;

typedef struct m3OpSetAngularVelocity
{
    m3BodyId id;
    m3Vec3 v;
} m3OpSetAngularVelocity;

typedef struct m3OpSetBodyName
{
    m3BodyId id;
    char name[M3_BODY_NAME_CAPACITY];
} m3OpSetBodyName;

typedef struct m3OpSetContactTuning
{
    float hertz;
    float dampingRatio;
    float pushSpeed;
} m3OpSetContactTuning;

typedef struct m3OpSetLinearVelocity
{
    m3BodyId id;
    m3Vec3 v;
} m3OpSetLinearVelocity;

typedef struct m3OpSetMotionLocks
{
    m3BodyId id;
    uint32_t locks;
} m3OpSetMotionLocks;

typedef struct m3OpSetShapeDensity
{
    m3ShapeId id;
    float value;
    int32_t updateMass;
} m3OpSetShapeDensity;

typedef struct m3OpSetShapeGeom
{
    m3ShapeId id;
    uint32_t type;
    m3ShapeGeom geom;
} m3OpSetShapeGeom;

typedef struct m3OpSetSleepControls
{
    m3BodyId id;
    float threshold;
    int32_t canSleep;
} m3OpSetSleepControls;

typedef struct m3OpSetSurfaceVelocity
{
    m3ShapeId id;
    m3Vec3 v;
} m3OpSetSurfaceVelocity;

typedef struct m3OpSetFilter
{
    uint64_t categoryBits;
    uint64_t maskBits;
    m3ShapeId id;
    int32_t groupIndex;
} m3OpSetFilter;

typedef struct m3OpSetWind
{
    m3Vec3 dir;
    float speed;
    float gustHertz;
    float gustScale;
} m3OpSetWind;

typedef struct m3OpShapeFlag
{
    m3ShapeId id;
    int32_t on;
} m3OpShapeFlag;

typedef struct m3OpShapeScalar
{
    m3ShapeId id;
    float value;
} m3OpShapeScalar;

typedef struct m3OpSoftBodyAnchor
{
    m3SoftBodyId id;
    int32_t particle;
    m3BodyId body;
} m3OpSoftBodyAnchor;

typedef struct m3OpSoftBodyAnchorSoft
{
    m3SoftBodyId idA;
    int32_t particleA;
    m3SoftBodyId idB;
    int32_t particleB;
} m3OpSoftBodyAnchorSoft;

typedef struct m3OpSoftBodyPin
{
    m3SoftBodyId id;
    int32_t particle;
} m3OpSoftBodyPin;

typedef struct m3OpStep
{
    float dt;
    int32_t substeps;
} m3OpStep;

typedef struct m3OpVehicleCommands
{
    m3VehicleId id;
    m3real throttle;
    m3real steer;
    m3real brake;
} m3OpVehicleCommands;

typedef struct m3OpVehicleDrivetrain
{
    m3VehicleId id;
    m3DrivetrainDef def;
} m3OpVehicleDrivetrain;

typedef struct m3OpVehicleGear
{
    m3VehicleId id;
    int32_t gear;
} m3OpVehicleGear;

typedef struct m3OpVehicleTankCommands
{
    m3VehicleId id;
    m3real left;
    m3real right;
    m3real brake;
} m3OpVehicleTankCommands;

typedef struct m3OpVoxelClear
{
    m3ShapeId id;
    int32_t x, y, z;
} m3OpVoxelClear;

typedef struct m3OpVoxelClearBox
{
    m3ShapeId id;
    int32_t lo[3];
    int32_t hi[3];
} m3OpVoxelClearBox;

typedef struct m3OpVoxelSet
{
    m3ShapeId id;
    int32_t x, y, z;
    uint16_t payload;
    uint16_t pad;
} m3OpVoxelSet;

typedef struct m3OpVoxelSetFill
{
    m3ShapeId id;
    int32_t x, y, z;
    uint8_t fill;
    uint8_t pad[3];
} m3OpVoxelSetFill;

// Journal payload for mesh materials: the fixed head below,
// followed by triangleCount group bytes.
typedef struct m3SetMeshMaterialsOp
{
    m3ShapeId id;
    int32_t materialCount;
    int32_t triangleCount;
    m3MeshSurfaceMaterial materials[M3_MESH_MAX_MATERIALS];
} m3SetMeshMaterialsOp;

// Journal payload for tet soft bodies: the fixed head,
// then pointCount m3Vec3 points, then 4 * tetCount uint16 ids.
typedef struct m3CreateSoftBodyTetOp
{
    m3SoftBodyDef def;
    int32_t pointCount;
    int32_t tetCount;
    m3SoftBodyId expected;
} m3CreateSoftBodyTetOp;

// Journal payload for the native heightfield: the fixed
// head below, followed by nx * nz float samples.
typedef struct m3CreateHeightFieldGridOp
{
    m3BodyId body;
    m3ShapeDef def;
    int32_t nx;
    int32_t nz;
    float cellSize;
    m3ShapeId expected;
} m3CreateHeightFieldGridOp;

// Journal payload for shape creation (replay re-derives mass).
typedef struct m3CreateShapeOp
{
    m3ShapeDef def;
    m3ShapeGeom geom;
    m3BodyId body;
    m3ShapeId expected;
    uint8_t type;
    uint8_t pad[7];
} m3CreateShapeOp;

// Journal payload for general hull shapes: the raw input points are
// the recipe; replay rebuilds through the same QuickHull, so the
// derived hull data never has to ride the journal.
// Journal header for mesh creation; the exact-size vertex and index
// arrays follow it in the payload (a full-cap struct would bloat the
// journal by 24 KB per mesh).
typedef struct m3CreateMeshShapeOp
{
    m3ShapeDef def;
    m3BodyId body;
    m3ShapeId expected;
    int32_t vertexCount;
    int32_t triangleCount;
} m3CreateMeshShapeOp;

typedef struct m3CreateJointOp
{
    m3JointDef def;
    m3JointId expected;
} m3CreateJointOp;

typedef struct m3CreateHullShapeOp
{
    m3ShapeDef def;
    m3BodyId body;
    m3ShapeId expected;
    int32_t count;
    m3Vec3 points[M3_HULL_MAX_INPUT];
} m3CreateHullShapeOp;

// Recording: appends one op; an overflow latches and fails the recording.
void m3JournalRecord(m3World* world, int32_t op, const void* payload, int32_t bytes);

// One piece of an op payload that is recorded from several buffers.
typedef struct m3JournalPart
{
    const void* data;
    int32_t bytes;
} m3JournalPart;

// Appends one op whose payload is the parts back to back: a fixed head,
// then the variable arrays it describes.
void m3JournalRecordParts(m3World* world, int32_t op, const m3JournalPart* parts,
                          int32_t partCount);

// Applies a tape's ops in order through the command table and reports
// the first refusal. Partial application is possible here;
// m3World_ReplayJournal makes the whole call atomic.
bool m3JournalReplayApply(m3World* world, const void* data, int32_t size);

#endif // MAUL3D_SRC_JOURNAL_H
