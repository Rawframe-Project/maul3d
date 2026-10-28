// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body sleep: the per-body sleep controls (whether a body may sleep and
// its velocity threshold) and the awake flag. The public functions
// validate and journal; the internal functions apply, and replay drives
// them directly.

#include "body.h"
#include "body_state.h"
#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "world_internal.h"

#include <string.h>

bool m3SleepThresholdValid(float threshold)
{
    return m3FiniteF(threshold) && threshold >= 0.0f;
}

void m3SetSleepControlsInternal(m3World* world, int32_t index, float threshold, int canSleep)
{
    world->bodies.bodySleepThreshold[index] =
        threshold > 0.0f ? threshold : M3_SLEEP_VELOCITY_DEFAULT;
    world->bodies.bodyCanSleep[index] = canSleep ? 1 : 0;
    if (!canSleep)
    {
        world->bodies.awake[index] = 1;
        world->bodies.sleepTimes[index] = 0.0f;
    }
}

// Both sleep settings travel in one journal op.
static void SetSleepControls(m3World* world, m3BodyId bodyId, int32_t index, float threshold,
                             bool canSleep)
{
    if (world->recorder.journalActive != 0)
    {
        m3OpSetSleepControls record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.threshold = threshold;
        record.canSleep = canSleep ? 1 : 0;
        m3JournalRecord(world, m3_opSetSleepControls, &record, (int32_t)sizeof(record));
    }
    m3SetSleepControlsInternal(world, index, threshold, canSleep ? 1 : 0);
}

static M3_NOINLINE void BodyEnableSleepWorker(m3World* world, m3BodyId body, bool flag)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    SetSleepControls(world, body, index, world->bodies.bodySleepThreshold[index], flag);
}

void m3Body_EnableSleep(m3World* world, m3BodyId body, bool flag)
{
    uint64_t env = m3FloatEnvEnter();
    BodyEnableSleepWorker(world, body, flag);
    m3FloatEnvLeave(env);
}

bool m3Body_IsSleepEnabled(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) && world->bodies.bodyCanSleep[index] != 0;
}

static M3_NOINLINE void BodySetSleepThresholdWorker(m3World* world, m3BodyId body, float threshold)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (!m3SleepThresholdValid(threshold))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    SetSleepControls(world, body, index, threshold, world->bodies.bodyCanSleep[index] != 0);
}

void m3Body_SetSleepThreshold(m3World* world, m3BodyId body, float threshold)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetSleepThresholdWorker(world, body, threshold);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE float BodyGetSleepThresholdWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.bodySleepThreshold[index] : 0.0f;
}

float m3Body_GetSleepThreshold(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    float result = BodyGetSleepThresholdWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

bool m3Body_IsAwake(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) && world->bodies.awake[index] != 0;
}

static M3_NOINLINE void BodySetAwakeWorker(m3World* world, m3BodyId body, bool awake)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return;
    }
    if (world->bodies.types[index] != (uint8_t)m3_dynamicBody)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.id = body;
        record.value = awake ? 1 : 0;
        m3JournalRecord(world, m3_opSetAwake, &record, (int32_t)sizeof(record));
    }
    m3SetAwakeInternal(world, index, awake ? 1 : 0);
}

void m3Body_SetAwake(m3World* world, m3BodyId body, bool awake)
{
    uint64_t env = m3FloatEnvEnter();
    BodySetAwakeWorker(world, body, awake);
    m3FloatEnvLeave(env);
}
