// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal recording and the atomic replay entry point. The per-op
// handlers live in journal_replay*.c.

#include "journal.h"

#include "misuse.h"
#include "state_prefix.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

void m3JournalRecordParts(m3World* world, int32_t op, const m3JournalPart* parts, int32_t partCount)
{
    if (world->recorder.journalActive == 0)
    {
        return;
    }
    int64_t bytes = 0;
    for (int32_t i = 0; i < partCount; ++i)
    {
        bytes += parts[i].bytes;
    }
    if (world->recorder.journalCursor + 8 + bytes > world->recorder.journalCapacity)
    {
        // Loud overflow: latch, stop recording, End reports -1.
        world->recorder.journalOverflow = 1;
        world->recorder.journalActive = 0;
        return;
    }
    uint8_t* out = world->recorder.journalBuffer + world->recorder.journalCursor;
    int32_t payloadBytes = (int32_t)bytes;
    memcpy(out, &op, 4);
    memcpy(out + 4, &payloadBytes, 4);
    int32_t at = 8;
    for (int32_t i = 0; i < partCount; ++i)
    {
        memcpy(out + at, parts[i].data, (size_t)parts[i].bytes);
        at += parts[i].bytes;
    }
    world->recorder.journalCursor += at;
}

void m3JournalRecord(m3World* world, int32_t op, const void* payload, int32_t bytes)
{
    m3JournalPart part = {payload, bytes};
    m3JournalRecordParts(world, op, &part, 1);
}

void m3JournalBodyDef(m3BodyDef* out, const m3BodyDef* def)
{
    memset(out, 0, sizeof(*out));
    out->type = def->type;
    out->position = def->position;
    out->rotation = def->rotation;
    out->linearVelocity = def->linearVelocity;
    out->angularVelocity = def->angularVelocity;
    out->gravityScale = def->gravityScale;
    out->linearDamping = def->linearDamping;
    out->angularDamping = def->angularDamping;
    out->userData = def->userData;
    out->isBullet = def->isBullet;
    out->motionLocks = def->motionLocks;
    out->enableSleep = def->enableSleep;
    out->sleepThreshold = def->sleepThreshold;
    out->isEnabled = def->isEnabled;
    out->enableFastRotation = def->enableFastRotation;
    out->internalValue = def->internalValue;
}

void m3JournalCharacterDef(m3CharacterDef* out, const m3CharacterDef* def)
{
    memset(out, 0, sizeof(*out));
    out->position = def->position;
    out->radius = def->radius;
    out->halfHeight = def->halfHeight;
    out->maxSlopeAngle = def->maxSlopeAngle;
    out->snapDistance = def->snapDistance;
    out->skin = def->skin;
    out->stepHeight = def->stepHeight;
    out->mass = def->mass;
    out->pushMaxMassRatio = def->pushMaxMassRatio;
    out->userData = def->userData;
    out->internalValue = def->internalValue;
}

void m3JournalSoftBodyDef(m3SoftBodyDef* out, const m3SoftBodyDef* def)
{
    memcpy(out, def, sizeof(*out)); // no padding
}

static void JournalWheelDef(m3WheelDef* out, const m3WheelDef* def)
{
    out->anchor = def->anchor;
    out->direction = def->direction;
    out->restLength = def->restLength;
    out->travel = def->travel;
    out->hertz = def->hertz;
    out->zeta = def->zeta;
    out->radius = def->radius;
    out->steerable = def->steerable;
    out->driven = def->driven;
    out->brakeShare = def->brakeShare;
}

void m3JournalVehicleDef(m3VehicleDef* out, const m3VehicleDef* def)
{
    memset(out, 0, sizeof(*out));
    out->chassisId = def->chassisId;
    out->wheelCount = def->wheelCount;
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        JournalWheelDef(&out->wheels[w], &def->wheels[w]);
    }
    out->maxSteerAngle = def->maxSteerAngle;
    out->driveForce = def->driveForce;
    out->brakeForce = def->brakeForce;
    out->tireGrip = def->tireGrip;
    out->leanStabilization = def->leanStabilization;
    out->userData = def->userData;
    out->internalValue = def->internalValue;
}

void m3JournalWaterVolumeDef(m3WaterVolumeDef* out, const m3WaterVolumeDef* def)
{
    memset(out, 0, sizeof(*out));
    out->lo = def->lo;
    out->hi = def->hi;
    out->density = def->density;
    out->linearDrag = def->linearDrag;
    out->angularDrag = def->angularDrag;
    out->flow = def->flow;
    out->internalValue = def->internalValue;
}

void m3JournalShapeDef(m3ShapeDef* out, const m3ShapeDef* def)
{
    memset(out, 0, sizeof(*out));
    out->density = def->density;
    out->friction = def->friction;
    out->restitution = def->restitution;
    out->categoryBits = def->categoryBits;
    out->maskBits = def->maskBits;
    out->groupIndex = def->groupIndex;
    out->rollingResistance = def->rollingResistance;
    out->surfaceVelocity = def->surfaceVelocity;
    out->userData = def->userData;
    out->isSensor = def->isSensor;
    out->enableHitEvents = def->enableHitEvents;
    out->enablePreSolveEvents = def->enablePreSolveEvents;
    out->localPosition = def->localPosition;
    out->localRotation = def->localRotation;
    out->internalValue = def->internalValue;
}

void m3JournalJointDef(m3JointDef* out, const m3JointDef* def)
{
    memset(out, 0, sizeof(*out));
    out->type = def->type;
    out->bodyIdA = def->bodyIdA;
    out->bodyIdB = def->bodyIdB;
    out->localAnchorA = def->localAnchorA;
    out->localAnchorB = def->localAnchorB;
    out->localAxisA = def->localAxisA;
    out->localAxisB = def->localAxisB;
    out->enableLimit = def->enableLimit;
    out->lowerLimit = def->lowerLimit;
    out->upperLimit = def->upperLimit;
    out->enableMotor = def->enableMotor;
    out->motorSpeed = def->motorSpeed;
    out->maxMotorEffort = def->maxMotorEffort;
    out->enableCone = def->enableCone;
    out->coneAngle = def->coneAngle;
    out->collideConnected = def->collideConnected;
    memcpy(out->genericLinear, def->genericLinear, sizeof(out->genericLinear));
    memcpy(out->genericAngular, def->genericAngular, sizeof(out->genericAngular));
    out->genericMotorAxis = def->genericMotorAxis;
    out->genericReserved = def->genericReserved;
    memcpy(out->genericLinearLower, def->genericLinearLower, sizeof(out->genericLinearLower));
    memcpy(out->genericLinearUpper, def->genericLinearUpper, sizeof(out->genericLinearUpper));
    memcpy(out->genericAngularLower, def->genericAngularLower, sizeof(out->genericAngularLower));
    memcpy(out->genericAngularUpper, def->genericAngularUpper, sizeof(out->genericAngularUpper));
    out->ratio = def->ratio;
    out->groundAnchorA = def->groundAnchorA;
    out->groundAnchorB = def->groundAnchorB;
    out->internalValue = def->internalValue;
}

void m3JournalDrivetrainDef(m3DrivetrainDef* out, const m3DrivetrainDef* def)
{
    memset(out, 0, sizeof(*out));
    out->curveCount = def->curveCount;
    memcpy(out->curveRpm, def->curveRpm, sizeof(out->curveRpm));
    memcpy(out->curveTorque, def->curveTorque, sizeof(out->curveTorque));
    out->gearCount = def->gearCount;
    memcpy(out->gearRatio, def->gearRatio, sizeof(out->gearRatio));
    out->reverseRatio = def->reverseRatio;
    out->finalDrive = def->finalDrive;
    out->shiftUpRpm = def->shiftUpRpm;
    out->shiftDownRpm = def->shiftDownRpm;
    out->clutchSteps = def->clutchSteps;
    out->autoShift = def->autoShift;
    out->diffMode = def->diffMode;
    out->diffCouple = def->diffCouple;
    out->internalValue = def->internalValue;
}

void m3JournalExplosionDef(m3ExplosionDef* out, const m3ExplosionDef* def)
{
    memcpy(out, def, sizeof(*out)); // no padding
}

m3Result m3World_StartJournal(m3World* world, void* buffer, int32_t capacity)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    // The buffer holds at least the tape's header, which is written now.
    if (world == NULL || buffer == NULL || capacity < M3_JOURNAL_HEADER_BYTES)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    if (world->recorder.journalActive != 0)
    {
        return m3Refuse(world, m3_errorState);
    }
    m3StatePrefix header =
        m3MakeStatePrefix(M3_JOURNAL_MAGIC, M3_JOURNAL_VERSION, M3_JOURNAL_HEADER_BYTES);
    memcpy(buffer, &header, sizeof(header));
    world->recorder.journalBuffer = (uint8_t*)buffer;
    world->recorder.journalCapacity = capacity;
    world->recorder.journalCursor = M3_JOURNAL_HEADER_BYTES;
    world->recorder.journalActive = 1;
    world->recorder.journalOverflow = 0;
    return m3_success;
}

m3Result m3World_StopJournal(m3World* world, int32_t* bytesOut)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (world == NULL || bytesOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *bytesOut = 0;
    if (world->recorder.journalActive == 0)
    {
        return m3Refuse(world, m3_errorState);
    }
    bool overflow = world->recorder.journalOverflow != 0;
    int32_t bytes = world->recorder.journalCursor;
    world->recorder.journalBuffer = NULL;
    world->recorder.journalCapacity = 0;
    world->recorder.journalCursor = 0;
    world->recorder.journalActive = 0;
    world->recorder.journalOverflow = 0;
    if (overflow)
    {
        return m3_errorCapacity; // the buffer filled up: the tape is unusable
    }
    *bytesOut = bytes;
    return m3_success;
}
