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
#include "body_state.h"
#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "query.h"
#include "shape_state.h"
#include "snapshot.h"
#include "solver.h"
#include "state_prefix.h"
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
    m3NormalizeBodyDefBools(&record.def);
    int32_t index = m3CreateBodyInternal(world, &record.def);
    // Id determinism: the replayed world must mint the exact
    // id the original minted, or the replay is invalid.
    if (index < 0 || (uint32_t)index + 1u != record.expected.index1 ||
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
    int32_t index = m3BodySlot(world, id);
    if (index < 0 || m3CharacterOwnsBody(world, index))
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
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3BodyVelocityValid(world->bodies.types[index], record.v))
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
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3BodySpinValid(record.v))
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
    if (!m3StepValid(record.dt, record.substeps))
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
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opApplyForce)
    {
        if (!m3ForceFits(world, index, record.v))
        {
            return false;
        }
        m3ApplyForceInternal(world, index, record.v);
    }
    else if (op == m3_opApplyTorque)
    {
        if (!m3TorqueFits(world, index, record.v))
        {
            return false;
        }
        m3ApplyTorqueInternal(world, index, record.v);
    }
    else if (op == m3_opApplyLinearImpulse)
    {
        if (!m3LinearImpulseFits(world, index, record.v))
        {
            return false;
        }
        m3ApplyLinearImpulseInternal(world, index, record.v);
    }
    else
    {
        if (!m3AngularImpulseFits(world, index, record.v))
        {
            return false;
        }
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
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opApplyForceAtPoint)
    {
        if (!m3ForceAtPointFits(world, index, record.v, record.p))
        {
            return false;
        }
        m3ApplyForceAtPointInternal(world, index, record.v, record.p);
    }
    else
    {
        if (!m3ImpulseAtPointFits(world, index, record.v, record.p))
        {
            return false;
        }
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
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || (op == m3_opSetTransform ? !m3PoseValid(record.pose)
                                              : !m3TargetValid(world, index, record.pose)))
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
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || record.param < m3_bodyParamGravityScale ||
        record.param > m3_bodyParamAngularDamping || !m3BodyParamValid(record.param, record.value))
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
    m3ShapeId shape = {record.id.index1, record.id.generation};
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
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    if (op == m3_opSetType)
    {
        if (!m3TypeChangeValid(world, index, record.value))
        {
            return false; // the public call refuses the same
        }
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
        if (world->bodies.types[index] != (uint8_t)m3_dynamicBody)
        {
            return false; // the public call refuses the same
        }
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
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3SleepThresholdValid(record.threshold))
    {
        return false; // the public calls refuse the same
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
    if (!m3GravityValid(gravity))
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
    if (!m3ContactTuningValid(record.hertz, record.dampingRatio, record.pushSpeed))
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
    if (op == m3_opSetRestitutionThreshold ? !(m3FiniteF(value) && value >= 0.0f)
                                           : !m3SpeedCapValid(value))
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
    if (!m3SpeedCapValid(value))
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
    return m3RebuildBroadphaseInternal(world); // no memory refuses the replay
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
    if (!m3WindValid(record.dir, record.speed, record.gustHertz, record.gustScale))
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

// One command per op code; a code no build ever wrote has no apply. The
// function pointer rides in a struct so the table is plain const data.
typedef struct ReplayCommand
{
    m3ReplayApplyFn* apply;
} ReplayCommand;

static const ReplayCommand s_commands[m3_opCount] = {
    [m3_opCreateBody] = {ApplyCreateBody},
    [m3_opDestroyBody] = {ApplyDestroyBody},
    [m3_opSetLinearVelocity] = {ApplySetLinearVelocity},
    [m3_opSetAngularVelocity] = {ApplySetAngularVelocity},
    [m3_opStepVetoes] = {ApplyStepVetoes},
    [m3_opStep] = {ApplyStep},
    [m3_opCreateShape] = {m3ReplayCreateShape},
    [m3_opCreateMeshShape] = {m3ReplayCreateMeshShape},
    [m3_opCreateJoint] = {m3ReplayCreateJoint},
    [m3_opDestroyJoint] = {m3ReplayDestroyJoint},
    [m3_opCreateHullShape] = {m3ReplayCreateHullShape},
    [m3_opCreateVoxelChunkShape] = {m3ReplayCreateVoxelChunkShape},
    [m3_opVoxelSet] = {m3ReplayVoxelSet},
    [m3_opVoxelClear] = {m3ReplayVoxelClear},
    [m3_opVoxelSetFill] = {m3ReplayVoxelSetFill},
    [m3_opVoxelClearBox] = {m3ReplayVoxelClearBox},
    [m3_opCreateCharacter] = {m3ReplayCreateCharacter},
    [m3_opDestroyCharacter] = {m3ReplayDestroyCharacter},
    [m3_opCharacterMove] = {m3ReplayCharacterMove},
    [m3_opCharacterStance] = {m3ReplayCharacterStance},
    [m3_opCreateVehicle] = {m3ReplayCreateVehicle},
    [m3_opDestroyVehicle] = {m3ReplayDestroyVehicle},
    [m3_opVehicleTankCommands] = {m3ReplayVehicleTankCommands},
    [m3_opVehicleCommands] = {m3ReplayVehicleCommands},
    [m3_opVehicleDrivetrain] = {m3ReplayVehicleDrivetrain},
    [m3_opVehicleGear] = {m3ReplayVehicleGear},
    [m3_opCreateSoftBody] = {m3ReplayCreateSoftBody},
    [m3_opDestroySoftBody] = {m3ReplayDestroySoftBody},
    [m3_opSoftBodyPin] = {m3ReplaySoftBodyPin},
    [m3_opSoftBodyAnchor] = {m3ReplaySoftBodyAnchor},
    [m3_opSoftBodyAnchorSoft] = {m3ReplaySoftBodyAnchorSoft},
    [m3_opApplyForce] = {ApplyBodyVector},
    [m3_opApplyTorque] = {ApplyBodyVector},
    [m3_opApplyLinearImpulse] = {ApplyBodyVector},
    [m3_opApplyAngularImpulse] = {ApplyBodyVector},
    [m3_opApplyForceAtPoint] = {ApplyBodyVectorAtPoint},
    [m3_opApplyImpulseAtPoint] = {ApplyBodyVectorAtPoint},
    [m3_opSetTransform] = {ApplyBodyPose},
    [m3_opSetTargetTransform] = {ApplyBodyPose},
    [m3_opSetType] = {ApplyBodyByte},
    [m3_opSetEnabled] = {ApplyBodyByte},
    [m3_opSetAwake] = {ApplyBodyByte},
    [m3_opSetMotionLocks] = {ApplySetMotionLocks},
    [m3_opSetSleepControls] = {ApplySetSleepControls},
    [m3_opDestroyShape] = {m3ReplayDestroyShape},
    [m3_opSetGravity] = {ApplySetGravity},
    [m3_opSetShapeFriction] = {m3ReplayShapeScalar},
    [m3_opSetShapeRestitution] = {m3ReplayShapeScalar},
    [m3_opSetShapeRolling] = {m3ReplayShapeScalar},
    [m3_opSetShapeDensity] = {m3ReplaySetShapeDensity},
    [m3_opSetContactTuning] = {ApplySetContactTuning},
    [m3_opSetRestitutionThreshold] = {ApplyWorldScalar},
    [m3_opSetMaximumLinearSpeed] = {ApplyWorldScalar},
    [m3_opSetMaximumAngularSpeed] = {ApplySetMaximumAngularSpeed},
    [m3_opSetBodyName] = {ApplySetBodyName},
    [m3_opCreateSoftBodyTet] = {m3ReplayCreateSoftBodyTet},
    [m3_opCreateHeightFieldGrid] = {m3ReplayCreateHeightFieldGrid},
    [m3_opCreateWaterVolume] = {m3ReplayCreateWaterVolume},
    [m3_opDestroyWaterVolume] = {m3ReplayDestroyWaterVolume},
    [m3_opRebuildBroadphase] = {ApplyRebuildBroadphase},
    [m3_opSetMeshMaterials] = {m3ReplaySetMeshMaterials},
    [m3_opJointSetMotorPose] = {m3ReplayJointSetMotorPose},
    [m3_opJointSetSteer] = {m3ReplayJointSetSteer},
    [m3_opSetShapeGeom] = {m3ReplaySetShapeGeom},
    [m3_opSetAllowFastRotation] = {ApplySetAllowFastRotation},
    [m3_opWorldExplode] = {ApplyWorldExplode},
    [m3_opEnableSleeping] = {ApplyWorldFlag},
    [m3_opEnableContinuous] = {ApplyWorldFlag},
    [m3_opSetWind] = {ApplySetWind},
    [m3_opSetSurfaceVelocity] = {m3ReplaySetSurfaceVelocity},
    [m3_opSetFilter] = {m3ReplaySetFilter},
    [m3_opSetBodyParam] = {ApplyBodyParam},
    [m3_opSetBullet] = {ApplyBodyByte},
    [m3_opSetBodyUserData] = {ApplyUserData},
    [m3_opSetShapeUserData] = {ApplyUserData},
    [m3_opSetHitEventThreshold] = {ApplySetHitEventThreshold},
    [m3_opEnableShapeHitEvents] = {m3ReplayShapeFlag},
    [m3_opEnableShapePreSolve] = {m3ReplayShapeFlag},
    [m3_opJointSetLimits] = {m3ReplayJointVector},
    [m3_opJointSetMotor] = {m3ReplayJointVector},
    [m3_opJointSetCollide] = {m3ReplayJointSetCollide},
    [m3_opJointSetBreak] = {m3ReplayJointSetBreak},
    [m3_opJointSetSpring] = {m3ReplayJointSetSpring},
    [m3_opJointSetTarget] = {m3ReplayJointSetTarget},
};

// Applies a tape's ops in order through the command table and reports
// the first refusal: m3_errorCapacity when the op that failed was
// refused memory, m3_errorInvalid otherwise. Partial application is
// possible here; m3World_ReplayJournal below makes the whole call atomic.
static m3Result ReplayApply(m3World* world, const void* data, int32_t size)
{
    const uint8_t* stream = (const uint8_t*)data;
    int32_t cursor = 0;
    while (cursor < size)
    {
        if (size - cursor < 8)
        {
            return m3_errorInvalid; // truncated header: reject loudly
        }
        m3ReplayRecord r;
        memcpy(&r.op, stream + cursor, 4);
        memcpy(&r.bytes, stream + cursor + 4, 4);
        cursor += 8;
        if (r.bytes < 0 || r.bytes > size - cursor)
        {
            return m3_errorInvalid; // truncated payload
        }
        r.payload = stream + cursor;
        cursor += r.bytes;
        if (r.op <= 0 || r.op >= m3_opCount || s_commands[r.op].apply == NULL)
        {
            return m3_errorInvalid; // unknown op: reject loudly, never skip
        }
        int64_t refused = world->heap.refusedCalls;
        if (!s_commands[r.op].apply(world, &r))
        {
            // An op the allocator turned down leaves the tape good.
            return world->heap.refusedCalls != refused ? m3_errorCapacity : m3_errorInvalid;
        }
    }
    return cursor == size ? m3_success : m3_errorInvalid;
}

static M3_NOINLINE m3Result WorldReplayJournalWorker(m3World* world, const void* data, int32_t size)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    // A tape is untrusted data like a snapshot: a damaged or truncated one
    // is refused with m3_errorInvalid, one another release wrote with
    // m3_errorVersion, and neither counts misuse.
    if (world == NULL || size < 0 || (data == NULL && size > 0))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    m3Result header = m3CheckStatePrefix(data, size, M3_JOURNAL_MAGIC, M3_JOURNAL_VERSION,
                                         M3_JOURNAL_HEADER_BYTES);
    if (header != m3_success)
    {
        return header;
    }
    if (size == M3_JOURNAL_HEADER_BYTES)
    {
        return m3_success; // a tape of no op
    }
    // Atomic replay: the world either takes the whole session or none of
    // it. A snapshot of the world before the first op, with the whole
    // restore of it prepared, backs out any partial application on
    // refusal without taking memory, so a damaged tape or an op the
    // allocator turns down can never leave a half-built world behind.
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)m3AllocZeroed(&world->heap, snapBytes);
    if (snap == NULL)
    {
        return m3_errorCapacity; // no memory for the guarantee means no replay
    }
    int32_t written = 0;
    m3RestoreStage backOut;
    if (m3World_Snapshot(world, snap, snapBytes, &written) != m3_success || written != snapBytes ||
        m3RestorePrepare(world, snap, snapBytes, &backOut) != m3_success)
    {
        // The world's own snapshot is always valid: only memory fails here.
        m3Free(&world->heap, snap);
        return m3_errorCapacity;
    }
    m3Result applied = ReplayApply(world, (const uint8_t*)data + M3_JOURNAL_HEADER_BYTES,
                                   size - M3_JOURNAL_HEADER_BYTES);
    if (applied != m3_success)
    {
        m3RestoreCommit(world, &backOut);
    }
    else
    {
        m3RestoreDrop(world, &backOut);
    }
    m3Free(&world->heap, snap);
    return applied;
}

m3Result m3World_ReplayJournal(m3World* world, const void* data, int32_t size)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = WorldReplayJournalWorker(world, data, size);
    m3FloatEnvLeave(env);
    return result;
}
