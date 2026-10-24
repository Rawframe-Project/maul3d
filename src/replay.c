// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The M3J1 container codec: pure memory, no IO, no world
// access. Corrupt input refuses loudly and touches nothing.

#include "maul3d/replay.h"

#include <string.h>

#define M3_REPLAY_MAGIC        0x314A334Du // "M3J1" little-endian
#define M3_REPLAY_VERSION      2u
#define M3_REPLAY_HEADER_BYTES 32

m3Result m3DescribeJournal(const void* journal, int32_t bytes, m3JournalInfo* infoOut)
{
    if (journal == NULL || bytes < 0 || infoOut == NULL)
    {
        return m3_errorInvalid;
    }
    const uint8_t* p = (const uint8_t*)journal;
    int32_t cursor = 0;
    int32_t ops = 0;
    int32_t steps = 0;
    while (cursor < bytes)
    {
        if (bytes - cursor < 8)
        {
            return m3_errorInvalid; // truncated record header
        }
        int32_t op;
        int32_t payload;
        memcpy(&op, p + cursor, 4);
        memcpy(&payload, p + cursor + 4, 4);
        if (op <= 0 || payload < 0 || payload > bytes - cursor - 8)
        {
            return m3_errorInvalid; // corrupt framing
        }
        ops += 1;
        if (op == 1)
        {
            steps += 1; // m3_opStep
        }
        cursor += 8 + payload;
    }
    infoOut->opCount = ops;
    infoOut->stepCount = steps;
    return m3_success;
}

int32_t m3GetEncodedReplaySize(int32_t snapshotBytes, int32_t journalBytes)
{
    if (snapshotBytes <= 0 || journalBytes < 0)
    {
        return 0;
    }
    return M3_REPLAY_HEADER_BYTES + snapshotBytes + journalBytes;
}

m3Result m3EncodeReplay(const void* snapshot, int32_t snapshotBytes, const void* journal,
                        int32_t journalBytes, uint64_t finalHash, void* out, int32_t capacity,
                        int32_t* bytesOut)
{
    if (bytesOut == NULL)
    {
        return m3_errorInvalid;
    }
    *bytesOut = 0;
    int32_t need = m3GetEncodedReplaySize(snapshotBytes, journalBytes);
    if (need == 0 || snapshot == NULL || (journal == NULL && journalBytes > 0) || out == NULL)
    {
        return m3_errorInvalid;
    }
    if (capacity < need)
    {
        return m3_errorCapacity;
    }
    m3JournalInfo info;
    if (m3DescribeJournal(journal, journalBytes, &info) != m3_success)
    {
        return m3_errorInvalid; // a container never wraps a malformed stream
    }
    uint8_t* p = (uint8_t*)out;
    uint32_t magic = M3_REPLAY_MAGIC;
    uint32_t version = M3_REPLAY_VERSION;
    memcpy(p, &magic, 4);
    memcpy(p + 4, &version, 4);
    memcpy(p + 8, &snapshotBytes, 4);
    memcpy(p + 12, &journalBytes, 4);
    memcpy(p + 16, &info.stepCount, 4);
    memcpy(p + 20, &info.opCount, 4);
    memcpy(p + 24, &finalHash, 8);
    memcpy(p + M3_REPLAY_HEADER_BYTES, snapshot, (size_t)snapshotBytes);
    if (journalBytes > 0)
    {
        memcpy(p + M3_REPLAY_HEADER_BYTES + snapshotBytes, journal, (size_t)journalBytes);
    }
    *bytesOut = need;
    return m3_success;
}

m3Result m3DecodeReplay(const void* data, int32_t bytes, m3ReplayView* viewOut)
{
    if (data == NULL || bytes < M3_REPLAY_HEADER_BYTES || viewOut == NULL)
    {
        return m3_errorInvalid;
    }
    const uint8_t* p = (const uint8_t*)data;
    uint32_t magic;
    uint32_t version;
    int32_t snapshotBytes;
    int32_t journalBytes;
    int32_t stepCount;
    int32_t opCount;
    uint64_t finalHash;
    memcpy(&magic, p, 4);
    memcpy(&version, p + 4, 4);
    memcpy(&snapshotBytes, p + 8, 4);
    memcpy(&journalBytes, p + 12, 4);
    memcpy(&stepCount, p + 16, 4);
    memcpy(&opCount, p + 20, 4);
    memcpy(&finalHash, p + 24, 8);
    if (magic != M3_REPLAY_MAGIC || version != M3_REPLAY_VERSION || snapshotBytes <= 0 ||
        journalBytes < 0 ||
        (int64_t)M3_REPLAY_HEADER_BYTES + snapshotBytes + journalBytes != (int64_t)bytes)
    {
        return m3_errorInvalid;
    }
    m3JournalInfo info;
    const uint8_t* journal = p + M3_REPLAY_HEADER_BYTES + snapshotBytes;
    if (m3DescribeJournal(journal, journalBytes, &info) != m3_success)
    {
        return m3_errorInvalid;
    }
    if (info.stepCount != stepCount || info.opCount != opCount)
    {
        return m3_errorInvalid; // header lies about its own stream
    }
    viewOut->snapshot = p + M3_REPLAY_HEADER_BYTES;
    viewOut->snapshotBytes = snapshotBytes;
    viewOut->journal = journal;
    viewOut->journalBytes = journalBytes;
    viewOut->finalHash = finalHash;
    viewOut->info = info;
    return m3_success;
}
