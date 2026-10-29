// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The distance joint's runtime calls: its length range and its spring's
// rest length. Public functions validate and journal; the internal setters
// in joint_params.c mutate and are what replay drives.

#include "fpenv.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "misuse.h"
#include "world_internal.h"

#include <string.h>

static bool ResolveJoint(const m3World* world, m3JointId id, int32_t* slotOut)
{
    int32_t slot = m3JointSlot(world, id);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    *slotOut = slot;
    return true;
}

// The range rides the distance joint's limit slot, always on, minimum in
// x and maximum in y, and its limit record on the journal.
static M3_NOINLINE void DistanceJointSetLengthRangeWorker(m3World* world, m3JointId joint,
                                                          float minLength, float maxLength)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_distanceJoint ||
        !m3DistanceLengthsFit(minLength, maxLength))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpJointVector record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.enable = 1;
        record.a = minLength;
        record.b = maxLength;
        m3JournalRecord(world, m3_opJointSetLimits, &record, (int32_t)sizeof(record));
    }
    m3JointSetLimitsInternal(world, slot, 1, minLength, maxLength);
}

void m3DistanceJoint_SetLengthRange(m3World* world, m3JointId joint, float minLength,
                                    float maxLength)
{
    uint64_t env = m3FloatEnvEnter();
    DistanceJointSetLengthRangeWorker(world, joint, minLength, maxLength);
    m3FloatEnvLeave(env);
}

// Zeros for a stale id and for a joint that is not a distance joint, each
// of which counts one misuse.
void m3DistanceJoint_GetLengthRange(const m3World* world, m3JointId joint, float* minLengthOut,
                                    float* maxLengthOut)
{
    m3Vec3 range = {0.0f, 0.0f, 0.0f};
    int32_t slot;
    if (ResolveJoint(world, joint, &slot))
    {
        if (world->joints.jointType[slot] == (uint8_t)m3_distanceJoint)
        {
            range = world->joints.jointLimits[slot];
        }
        else
        {
            m3Refuse(world, m3_errorInvalid);
        }
    }
    if (minLengthOut != NULL)
    {
        *minLengthOut = range.x;
    }
    if (maxLengthOut != NULL)
    {
        *maxLengthOut = range.y;
    }
}

static M3_NOINLINE void DistanceJointSetLengthWorker(m3World* world, m3JointId joint, float length)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_distanceJoint ||
        !m3DistanceRestLengthFits(length))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpDistanceJointSetLength record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.length = length;
        m3JournalRecord(world, m3_opDistanceJointSetLength, &record, (int32_t)sizeof(record));
    }
    m3DistanceJointSetLengthInternal(world, slot, length);
}

void m3DistanceJoint_SetLength(m3World* world, m3JointId joint, float length)
{
    uint64_t env = m3FloatEnvEnter();
    DistanceJointSetLengthWorker(world, joint, length);
    m3FloatEnvLeave(env);
}

// Zero for a stale id and for a joint that is not a distance joint, each of
// which counts one misuse.
float m3DistanceJoint_GetLength(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return 0.0f;
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_distanceJoint)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0.0f;
    }
    return world->joints.jointLimits[slot].z;
}
