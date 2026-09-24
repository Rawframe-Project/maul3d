// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay: one apply function per op (or per family of ops that
// share a payload) and the command table that maps op codes to them.
// Every apply validates its payload as hostile bytes and re-enters the
// same internal functions the public API uses; created ids must match
// the ids the recording minted.

#include "journal_replay.h"
#include "body.h"
#include "character.h"
#include "joint.h"
#include "journal.h"
#include "query.h"
#include "quickhull.h"
#include "shape.h"
#include "softbody.h"
#include "solver.h"
#include "vehicle.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

static bool ApplyCreateBody(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpCreateBody record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    m3NormalizeBoolByte(&record.def, offsetof(m3BodyDef, isBullet));
    int32_t index = m3CreateBodyInternal(world, &record.def);
    // Id determinism: the replayed world must mint the exact
    // id the original minted, or the replay is invalid.
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->bodies.bodyPool.generations[index] != record.expected.generation)
    {
        return false;
    }
    return true;
}

static bool ApplyDestroyBody(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3BodyId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    // Recorded ids carry the ORIGINAL world's slot; replay
    // retargets them to this world (the Maul2D rule).
    id.world = world->idWorld;
    int32_t index = m3BodySlot(world, id);
    if (index < 0)
    {
        return false;
    }
    m3DestroyBodyInternal(world, index);
    return true;
}

static bool ApplySetLinearVelocity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetLinearVelocity record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3FiniteV3(record.v))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetLinearVelocityInternal(world, index, record.v);
    return true;
}

static bool ApplySetAngularVelocity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetAngularVelocity record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3FiniteV3(record.v))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetAngularVelocityInternal(world, index, record.v);
    return true;
}

static bool ApplyStepVetoes(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    if (bytes <= 0 || (bytes % 8) != 0 || bytes / 8 > world->contacts.pairCapacity)
    {
        return false; // hostile veto list refuses loudly
    }
    memcpy(world->contacts.replayVetoKeys, payload, (size_t)bytes);
    world->contacts.replayVetoCount = bytes / 8;
    return true;
}

static bool ApplyStep(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpStep record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (!(record.dt > 0.0f) || record.substeps < 1 || record.substeps > M3_MAX_SUBSTEPS)
    {
        // The upper bound matters: one flipped bit could turn
        // substeps 4 into a billion and a two-second replay into
        // hours. Hostile tapes refuse
        // in proportion to their crime.
        return false;
    }
    m3StepInternal(world, record.dt, record.substeps);
    return true;
}

static bool ApplyBodyVector(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    m3OpBodyVector record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3FiniteV3(record.v))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opApplyForce)
    {
        m3ApplyForceInternal(world, index, record.v);
    }
    else if (op == m3_opApplyTorque)
    {
        m3ApplyTorqueInternal(world, index, record.v);
    }
    else if (op == m3_opApplyLinearImpulse)
    {
        m3ApplyLinearImpulseInternal(world, index, record.v);
    }
    else
    {
        m3ApplyAngularImpulseInternal(world, index, record.v);
    }
    return true;
}

static bool ApplyBodyVectorAtPoint(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    m3OpBodyVectorAtPoint record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3FiniteV3(record.v) || !m3FinitePos3(record.p))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opApplyForceAtPoint)
    {
        m3ApplyForceAtPointInternal(world, index, record.v, record.p);
    }
    else
    {
        m3ApplyImpulseAtPointInternal(world, index, record.v, record.p);
    }
    return true;
}

static bool ApplyBodyPose(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    m3OpBodyPose record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    float qq = record.pose.q.x * record.pose.q.x + record.pose.q.y * record.pose.q.y +
               record.pose.q.z * record.pose.q.z + record.pose.q.w * record.pose.q.w;
    if (index < 0 || !m3FinitePos3(record.pose.p) || !m3FiniteQuat(record.pose.q) ||
        !(qq > 0.98f) || !(qq < 1.02f))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opSetTransform)
    {
        m3SetTransformInternal(world, index, record.pose);
    }
    else
    {
        m3SetTargetTransformInternal(world, index, record.pose);
    }
    return true;
}

static bool ApplyBodyParam(m3World* world, const m3ReplayRecord* r)
{
    m3OpBodyParam record;
    if (r->bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, r->payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || record.param < m3_bodyParamGravityScale ||
        record.param > m3_bodyParamAngularDamping || !m3FiniteF(record.value) ||
        (record.param != m3_bodyParamGravityScale && record.value < 0.0f))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetBodyParamInternal(world, index, record.param, record.value);
    return true;
}

static bool ApplyUserData(m3World* world, const m3ReplayRecord* r)
{
    m3OpUserData record;
    if (r->bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, r->payload, sizeof(record));
    record.id.world = world->idWorld;
    if (r->op == m3_opSetBodyUserData)
    {
        int32_t index = m3BodySlot(world, record.id);
        if (index < 0)
        {
            return false;
        }
        world->bodies.userData[index] = record.userData;
        return true;
    }
    m3ShapeId shape = {record.id.index1, record.id.world, record.id.generation};
    int32_t slot = m3ShapeSlot(world, shape);
    if (slot < 0)
    {
        return false;
    }
    world->shapes.shapeUserData[slot] = record.userData;
    return true;
}

static bool ApplyBodyByte(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    m3OpBodyByte record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    if (op == m3_opSetType)
    {
        m3SetTypeInternal(world, index, (uint8_t)record.value);
    }
    else if (op == m3_opSetEnabled)
    {
        m3SetEnabledInternal(world, index, record.value);
    }
    else if (op == m3_opSetBullet)
    {
        m3SetBulletInternal(world, index, record.value);
    }
    else
    {
        m3SetAwakeInternal(world, index, record.value);
    }
    return true;
}

static bool ApplySetMotionLocks(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetMotionLocks record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    m3SetMotionLocksInternal(world, index, (uint8_t)record.locks);
    return true;
}

static bool ApplySetSleepControls(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetSleepControls record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    m3SetSleepControlsInternal(world, index, record.threshold, record.canSleep);
    return true;
}

static bool ApplySetGravity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3Vec3 gravity;
    if (bytes != (int32_t)sizeof(gravity))
    {
        return false;
    }
    memcpy(&gravity, payload, sizeof(gravity));
    if (!m3FiniteV3(gravity))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetGravityInternal(world, gravity);
    return true;
}

static bool ApplySetContactTuning(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetContactTuning record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (!m3FiniteF(record.hertz) || record.hertz <= 0.0f || !m3FiniteF(record.dampingRatio) ||
        record.dampingRatio <= 0.0f || !m3FiniteF(record.pushSpeed) || record.pushSpeed <= 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetContactTuningInternal(world, record.hertz, record.dampingRatio, record.pushSpeed);
    return true;
}

static bool ApplyWorldScalar(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    float value;
    if (bytes != (int32_t)sizeof(value))
    {
        return false;
    }
    memcpy(&value, payload, sizeof(value));
    if (!m3FiniteF(value) || (op == m3_opSetRestitutionThreshold && value < 0.0f) ||
        (op == m3_opSetMaximumLinearSpeed && value <= 0.0f))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opSetRestitutionThreshold)
    {
        m3SetRestitutionThresholdInternal(world, value);
    }
    else
    {
        m3SetMaximumLinearSpeedInternal(world, value);
    }
    return true;
}

static bool ApplySetMaximumAngularSpeed(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    // A new op gets the strict wall: hostile caps (NaN,
    // nonpositive) fail the replay loudly instead of riding
    // into the solver.
    float value;
    if (bytes != (int32_t)sizeof(value))
    {
        return false;
    }
    memcpy(&value, payload, sizeof(value));
    if (!m3FiniteF(value) || value <= 0.0f)
    {
        return false;
    }
    m3SetMaximumAngularSpeedInternal(world, value);
    return true;
}

static bool ApplySetBodyName(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetBodyName record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    record.name[M3_BODY_NAME_CAPACITY - 1] = 0;
    m3SetBodyNameInternal(world, index, record.name);
    return true;
}

static bool ApplyRebuildBroadphase(m3World* world, const m3ReplayRecord* r)
{
    int32_t bytes = r->bytes;
    int32_t zero;
    if (bytes != (int32_t)sizeof(zero))
    {
        return false;
    }
    m3RebuildBroadphaseInternal(world);
    return true;
}

static bool ApplySetAllowFastRotation(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetAllowFastRotation record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (record.allow > 1u)
    {
        return false;
    }
    record.id.world = world->idWorld;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    m3SetAllowFastRotationInternal(world, index, (int32_t)record.allow);
    return true;
}

static bool ApplyWorldExplode(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3ExplosionDef def;
    if (bytes != (int32_t)sizeof(def))
    {
        return false;
    }
    memcpy(&def, payload, sizeof(def));
    if (!m3WorldExplodeInternal(world, &def))
    {
        return false; // hostile blast fields fail loudly
    }
    return true;
}

static bool ApplyWorldFlag(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    int32_t on;
    if (bytes != (int32_t)sizeof(on))
    {
        return false;
    }
    memcpy(&on, payload, sizeof(on));
    if (op == m3_opEnableSleeping)
    {
        m3EnableSleepingInternal(world, on);
    }
    else
    {
        m3EnableContinuousInternal(world, on);
    }
    return true;
}

static bool ApplySetWind(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSetWind record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (!m3FiniteV3(record.dir) || !m3FiniteF(record.speed) || record.speed < 0.0f ||
        !m3FiniteF(record.gustHertz) || record.gustHertz < 0.0f || !m3FiniteF(record.gustScale) ||
        record.gustScale < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetWindInternal(world, record.dir, record.speed, record.gustHertz, record.gustScale);
    return true;
}

static bool ApplySetHitEventThreshold(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    float value;
    if (bytes != (int32_t)sizeof(value))
    {
        return false;
    }
    memcpy(&value, payload, sizeof(value));
    if (!m3FiniteF(value) || value < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetHitEventThresholdInternal(world, value);
    return true;
}

static m3ReplayApplyFn* const s_commands[m3_opCount] = {
    [m3_opCreateBody] = ApplyCreateBody,
    [m3_opDestroyBody] = ApplyDestroyBody,
    [m3_opSetLinearVelocity] = ApplySetLinearVelocity,
    [m3_opSetAngularVelocity] = ApplySetAngularVelocity,
    [m3_opStepVetoes] = ApplyStepVetoes,
    [m3_opStep] = ApplyStep,
    [m3_opCreateShape] = m3ReplayCreateShape,
    [m3_opCreateMeshShape] = m3ReplayCreateMeshShape,
    [m3_opCreateJoint] = m3ReplayCreateJoint,
    [m3_opDestroyJoint] = m3ReplayDestroyJoint,
    [m3_opCreateHullShape] = m3ReplayCreateHullShape,
    [m3_opCreateVoxelChunkShape] = m3ReplayCreateVoxelChunkShape,
    [m3_opVoxelSet] = m3ReplayVoxelSet,
    [m3_opVoxelClear] = m3ReplayVoxelClear,
    [m3_opVoxelSetFill] = m3ReplayVoxelSetFill,
    [m3_opVoxelClearBox] = m3ReplayVoxelClearBox,
    [m3_opCreateCharacter] = m3ReplayCreateCharacter,
    [m3_opDestroyCharacter] = m3ReplayDestroyCharacter,
    [m3_opCharacterMove] = m3ReplayCharacterMove,
    [m3_opCharacterStance] = m3ReplayCharacterStance,
    [m3_opCreateVehicle] = m3ReplayCreateVehicle,
    [m3_opDestroyVehicle] = m3ReplayDestroyVehicle,
    [m3_opVehicleTankCommands] = m3ReplayVehicleTankCommands,
    [m3_opVehicleCommands] = m3ReplayVehicleCommands,
    [m3_opVehicleDrivetrain] = m3ReplayVehicleDrivetrain,
    [m3_opVehicleGear] = m3ReplayVehicleGear,
    [m3_opCreateSoftBody] = m3ReplayCreateSoftBody,
    [m3_opDestroySoftBody] = m3ReplayDestroySoftBody,
    [m3_opSoftBodyPin] = m3ReplaySoftBodyPin,
    [m3_opSoftBodyAnchor] = m3ReplaySoftBodyAnchor,
    [m3_opSoftBodyAnchorSoft] = m3ReplaySoftBodyAnchorSoft,
    [m3_opApplyForce] = ApplyBodyVector,
    [m3_opApplyTorque] = ApplyBodyVector,
    [m3_opApplyLinearImpulse] = ApplyBodyVector,
    [m3_opApplyAngularImpulse] = ApplyBodyVector,
    [m3_opApplyForceAtPoint] = ApplyBodyVectorAtPoint,
    [m3_opApplyImpulseAtPoint] = ApplyBodyVectorAtPoint,
    [m3_opSetTransform] = ApplyBodyPose,
    [m3_opSetTargetTransform] = ApplyBodyPose,
    [m3_opSetType] = ApplyBodyByte,
    [m3_opSetEnabled] = ApplyBodyByte,
    [m3_opSetAwake] = ApplyBodyByte,
    [m3_opSetMotionLocks] = ApplySetMotionLocks,
    [m3_opSetSleepControls] = ApplySetSleepControls,
    [m3_opDestroyShape] = m3ReplayDestroyShape,
    [m3_opSetGravity] = ApplySetGravity,
    [m3_opSetShapeFriction] = m3ReplayShapeScalar,
    [m3_opSetShapeRestitution] = m3ReplayShapeScalar,
    [m3_opSetShapeRolling] = m3ReplayShapeScalar,
    [m3_opSetShapeDensity] = m3ReplaySetShapeDensity,
    [m3_opSetContactTuning] = ApplySetContactTuning,
    [m3_opSetRestitutionThreshold] = ApplyWorldScalar,
    [m3_opSetMaximumLinearSpeed] = ApplyWorldScalar,
    [m3_opSetMaximumAngularSpeed] = ApplySetMaximumAngularSpeed,
    [m3_opSetBodyName] = ApplySetBodyName,
    [m3_opCreateSoftBodyTet] = m3ReplayCreateSoftBodyTet,
    [m3_opCreateHeightFieldGrid] = m3ReplayCreateHeightFieldGrid,
    [m3_opCreateWaterVolume] = m3ReplayCreateWaterVolume,
    [m3_opDestroyWaterVolume] = m3ReplayDestroyWaterVolume,
    [m3_opRebuildBroadphase] = ApplyRebuildBroadphase,
    [m3_opSetMeshMaterials] = m3ReplaySetMeshMaterials,
    [m3_opJointSetMotorPose] = m3ReplayJointSetMotorPose,
    [m3_opJointSetSteer] = m3ReplayJointSetSteer,
    [m3_opSetShapeGeom] = m3ReplaySetShapeGeom,
    [m3_opSetAllowFastRotation] = ApplySetAllowFastRotation,
    [m3_opWorldExplode] = ApplyWorldExplode,
    [m3_opEnableSleeping] = ApplyWorldFlag,
    [m3_opEnableContinuous] = ApplyWorldFlag,
    [m3_opSetWind] = ApplySetWind,
    [m3_opSetSurfaceVelocity] = m3ReplaySetSurfaceVelocity,
    [m3_opSetFilter] = m3ReplaySetFilter,
    [m3_opSetBodyParam] = ApplyBodyParam,
    [m3_opSetBullet] = ApplyBodyByte,
    [m3_opSetBodyUserData] = ApplyUserData,
    [m3_opSetShapeUserData] = ApplyUserData,
    [m3_opSetHitEventThreshold] = ApplySetHitEventThreshold,
    [m3_opEnableShapeHitEvents] = m3ReplayShapeFlag,
    [m3_opEnableShapePreSolve] = m3ReplayShapeFlag,
    [m3_opJointSetLimits] = m3ReplayJointVector,
    [m3_opJointSetMotor] = m3ReplayJointVector,
    [m3_opJointSetCollide] = m3ReplayJointSetCollide,
    [m3_opJointSetBreak] = m3ReplayJointSetBreak,
    [m3_opJointSetSpring] = m3ReplayJointSetSpring,
    [m3_opJointSetTarget] = m3ReplayJointSetTarget,
};

bool m3JournalReplayApply(m3World* world, const void* data, int32_t size)
{
    const uint8_t* stream = (const uint8_t*)data;
    int32_t cursor = 0;
    while (cursor < size)
    {
        if (cursor + 8 > size)
        {
            return false; // truncated header: reject loudly
        }
        m3ReplayRecord r;
        memcpy(&r.op, stream + cursor, 4);
        memcpy(&r.bytes, stream + cursor + 4, 4);
        cursor += 8;
        if (r.bytes < 0 || cursor + r.bytes > size)
        {
            return false; // truncated payload
        }
        r.payload = stream + cursor;
        cursor += r.bytes;
        if (r.op <= 0 || r.op >= m3_opCount || s_commands[r.op] == NULL)
        {
            return false; // unknown op: reject loudly, never skip
        }
        if (!s_commands[r.op](world, &r))
        {
            return false;
        }
    }
    return cursor == size;
}
