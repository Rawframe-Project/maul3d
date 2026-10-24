// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal recording and the atomic replay entry point. The per-op
// handlers live in journal_replay*.c.

#include "journal.h"

#include "misuse.h"
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

m3Result m3World_StartJournal(m3World* world, void* buffer, int32_t capacity)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (world == NULL || buffer == NULL || capacity < 8)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    if (world->recorder.journalActive != 0)
    {
        return m3Refuse(world, m3_errorState);
    }
    world->recorder.journalBuffer = (uint8_t*)buffer;
    world->recorder.journalCapacity = capacity;
    world->recorder.journalCursor = 0;
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
