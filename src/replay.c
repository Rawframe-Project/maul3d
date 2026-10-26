// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The replay container codec: pure memory, no IO, no world
// access. Corrupt input refuses loudly and touches nothing.

#include "maul3d/replay.h"

#include "state_prefix.h"

#include <string.h>

// The container's header: the state prefix, the recorder's final hash,
// then the sizes and counts of what follows.
typedef struct m3ReplayHeader
{
    m3StatePrefix prefix;
    uint64_t finalHash;
    int32_t snapshotBytes;
    int32_t journalBytes;
    int32_t stepCount;
    int32_t opCount;
} m3ReplayHeader;

_Static_assert(sizeof(m3ReplayHeader) == M3_REPLAY_HEADER_BYTES,
               "the replay header must be padding-free");

m3Result m3DescribeJournal(const void* journal, int32_t bytes, m3JournalInfo* infoOut)
{
    if (journal == NULL || bytes < 0 || infoOut == NULL)
    {
        return m3_errorInvalid;
    }
    m3Result header = m3CheckStatePrefix(journal, bytes, M3_JOURNAL_MAGIC, M3_JOURNAL_VERSION,
                                         M3_JOURNAL_HEADER_BYTES);
    if (header != m3_success)
    {
        return header;
    }
    const uint8_t* p = (const uint8_t*)journal;
    int32_t cursor = M3_JOURNAL_HEADER_BYTES;
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
    if (snapshotBytes < M3_SNAPSHOT_HEADER_BYTES || journalBytes < M3_JOURNAL_HEADER_BYTES ||
        snapshotBytes > INT32_MAX - M3_REPLAY_HEADER_BYTES - journalBytes)
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
    if (need == 0 || snapshot == NULL || journal == NULL || out == NULL)
    {
        return m3_errorInvalid;
    }
    if (capacity < need)
    {
        return m3_errorCapacity;
    }
    // A container never wraps a malformed stream, nor one another release
    // wrote.
    m3Result checked = m3CheckStatePrefix(snapshot, snapshotBytes, M3_SNAPSHOT_MAGIC,
                                          M3_SNAPSHOT_VERSION, M3_SNAPSHOT_HEADER_BYTES);
    m3JournalInfo info;
    if (checked == m3_success)
    {
        checked = m3DescribeJournal(journal, journalBytes, &info);
    }
    if (checked != m3_success)
    {
        return checked;
    }
    m3ReplayHeader header;
    memset(&header, 0, sizeof(header));
    header.prefix = m3MakeStatePrefix(M3_REPLAY_MAGIC, M3_REPLAY_VERSION, M3_REPLAY_HEADER_BYTES);
    header.finalHash = finalHash;
    header.snapshotBytes = snapshotBytes;
    header.journalBytes = journalBytes;
    header.stepCount = info.stepCount;
    header.opCount = info.opCount;
    uint8_t* p = (uint8_t*)out;
    memcpy(p, &header, sizeof(header));
    memcpy(p + M3_REPLAY_HEADER_BYTES, snapshot, (size_t)snapshotBytes);
    memcpy(p + M3_REPLAY_HEADER_BYTES + snapshotBytes, journal, (size_t)journalBytes);
    *bytesOut = need;
    return m3_success;
}

m3Result m3DecodeReplay(const void* data, int32_t bytes, m3ReplayView* viewOut)
{
    if (data == NULL || viewOut == NULL)
    {
        return m3_errorInvalid;
    }
    m3Result checked =
        m3CheckStatePrefix(data, bytes, M3_REPLAY_MAGIC, M3_REPLAY_VERSION, M3_REPLAY_HEADER_BYTES);
    if (checked != m3_success)
    {
        return checked;
    }
    const uint8_t* p = (const uint8_t*)data;
    m3ReplayHeader header;
    memcpy(&header, p, sizeof(header));
    if (header.snapshotBytes < M3_SNAPSHOT_HEADER_BYTES ||
        header.journalBytes < M3_JOURNAL_HEADER_BYTES ||
        (int64_t)M3_REPLAY_HEADER_BYTES + header.snapshotBytes + header.journalBytes !=
            (int64_t)bytes)
    {
        return m3_errorInvalid;
    }
    // The embedded headers are checked like any other: another release's
    // snapshot or tape inside this container is m3_errorVersion.
    const uint8_t* snapshot = p + M3_REPLAY_HEADER_BYTES;
    const uint8_t* journal = snapshot + header.snapshotBytes;
    checked = m3CheckStatePrefix(snapshot, header.snapshotBytes, M3_SNAPSHOT_MAGIC,
                                 M3_SNAPSHOT_VERSION, M3_SNAPSHOT_HEADER_BYTES);
    m3JournalInfo info;
    if (checked == m3_success)
    {
        checked = m3DescribeJournal(journal, header.journalBytes, &info);
    }
    if (checked != m3_success)
    {
        return checked;
    }
    if (info.stepCount != header.stepCount || info.opCount != header.opCount)
    {
        return m3_errorInvalid; // header lies about its own stream
    }
    viewOut->snapshot = snapshot;
    viewOut->snapshotBytes = header.snapshotBytes;
    viewOut->journal = journal;
    viewOut->journalBytes = header.journalBytes;
    viewOut->finalHash = header.finalHash;
    viewOut->info = info;
    return m3_success;
}
