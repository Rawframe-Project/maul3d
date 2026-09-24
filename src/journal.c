// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal recording and the atomic replay entry point. The per-op
// handlers live in journal_replay*.c.

#include "journal.h"
#include "body.h"
#include "query.h"
#include "world.h"
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

bool m3World_StartJournal(m3WorldId worldId, void* buffer, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || buffer == NULL || capacity < 8 || world->recorder.journalActive != 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // contract, not invariant
    }
    world->recorder.journalBuffer = (uint8_t*)buffer;
    world->recorder.journalCapacity = capacity;
    world->recorder.journalCursor = 0;
    world->recorder.journalActive = 1;
    world->recorder.journalOverflow = 0;
    return true;
}

int32_t m3World_StopJournal(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1; // contract, not invariant
    }
    int32_t bytes = world->recorder.journalOverflow != 0 ? -1 : world->recorder.journalCursor;
    world->recorder.journalBuffer = NULL;
    world->recorder.journalCapacity = 0;
    world->recorder.journalCursor = 0;
    world->recorder.journalActive = 0;
    world->recorder.journalOverflow = 0;
    return bytes;
}

bool m3World_ReplayJournal(m3WorldId worldId, const void* data, int32_t size)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || data == NULL || size < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // contract, not invariant
    }
    // Atomic replay: the world either takes the whole session
    // or none of it. A pre-replay snapshot backs out any partial
    // application on refusal, so a corrupted or truncated journal
    // can never leave a half-built world behind.
    int32_t snapBytes = m3World_SnapshotSize(worldId);
    uint8_t* snap = (uint8_t*)m3AllocZeroed(snapBytes);
    if (snap == NULL)
    {
        m3Refuse(world, m3_errorCapacity);
        return false; // no memory for the guarantee means no replay
    }
    if (m3World_Snapshot(worldId, snap, snapBytes) != snapBytes)
    {
        m3Free(snap);
        m3Refuse(world, m3_errorCapacity);
        return false;
    }
    bool ok = m3JournalReplayApply(world, data, size);
    if (!ok)
    {
        bool restored = m3World_Restore(worldId, snap, snapBytes);
        M3_ASSERT(restored); // our own snapshot must restore: invariant
        (void)restored;
        m3Refuse(world, m3_errorInvalid);
    }
    m3Free(snap);
    return ok;
}
