// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Replays: the replay container that seals a snapshot, a journal and
// the final hash into one artifact, and the functions that write,
// verify, seek and compare replays.

#ifndef MAUL3D_REPLAY_H
#define MAUL3D_REPLAY_H

#include "maul3d/core_math.h"
#include "maul3d/world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// The replay container: a pure-memory codec for [initial snapshot
    /// + journal] sessions. The LIBRARY never touches files; tools own
    /// IO (the zero-dependency law). The container guards its own
    /// framing and the headers it embeds: the rest of the snapshot is
    /// checked by m3World_Restore like any other, and the journal's
    /// records by replay, each op as the API call it stands for. Only
    /// the release that wrote a container reads it, like the snapshot
    /// and the journal tape it holds.
    ///
    /// Layout, little-endian: the 16-byte state prefix (magic "M3RP",
    /// abiVersion, formatVersion 4, headerBytes 56) | finalHash u64 |
    /// snapshotBytes i64 | journalBytes i64 | stepCount i64 | opCount
    /// i64 | snapshot | journal. Sizes and counts are 64-bit; on a
    /// 32-bit target a size past SIZE_MAX is refused with
    /// m3_errorInvalid before any pointer is formed.

    /// Counts from a journal tape: total records, and how many of them
    /// are steps. Returns m3_errorVersion for a tape another release
    /// wrote and m3_errorFormat for malformed framing.
    typedef struct m3JournalInfo
    {
        int64_t opCount;
        int64_t stepCount;
    } m3JournalInfo;

    M3_NODISCARD M3_API m3Result m3DescribeJournal(const void* journal, int64_t bytes,
                                                   m3JournalInfo* infoOut);

    /// A decoded view into an encoded container: pointers alias the
    /// input buffer, nothing is copied.
    typedef struct m3ReplayView
    {
        const void* snapshot;
        int64_t snapshotBytes;
        const void* journal;
        int64_t journalBytes;
        uint64_t finalHash; // the recorder's end-of-session hash
        m3JournalInfo info;
    } m3ReplayView;

    /// Exact encoded size for the given payload sizes, or 0 when they
    /// cannot be a snapshot and a tape or the container would pass
    /// INT64_MAX bytes.
    M3_API int64_t m3GetEncodedReplaySize(int64_t snapshotBytes, int64_t journalBytes);

    /// Encode a container. finalHash is the recorder's end hash
    /// (m3World_Hash after the session); verifiers compare against
    /// it. Sets *bytesOut to the bytes written; any failure leaves it as
    /// it was, since the call has no owner. Returns m3_errorVersion
    /// for a snapshot or tape another release wrote, m3_errorInvalid for
    /// a null pointer or an out whose first m3GetEncodedReplaySize
    /// bytes overlap the snapshot or the tape, m3_errorFormat for sizes
    /// too short to hold a snapshot or a tape, a header that is not a
    /// snapshot's or a malformed journal, m3_errorLimit for a pair whose
    /// container no buffer of this target can hold, and m3_errorCapacity
    /// for a small buffer.
    M3_NODISCARD M3_API m3Result m3EncodeReplay(const void* snapshot, int64_t snapshotBytes,
                                                const void* journal, int64_t journalBytes,
                                                uint64_t finalHash, void* out, int64_t capacity,
                                                int64_t* bytesOut);

    /// Decode and validate framing (the container's header, the
    /// snapshot's and the tape's prefixes, lengths, journal record
    /// walk). Returns m3_errorVersion when another release wrote the
    /// container or what it holds, and m3_errorFormat on any
    /// corruption, with the view untouched either way.
    M3_NODISCARD M3_API m3Result m3DecodeReplay(const void* data, int64_t bytes,
                                                m3ReplayView* viewOut);

    /// One row of a world-versus-world body comparison. A
    /// DEBUG READ for the divergence finder: never state, never
    /// hashed, order is total (error magnitude descending, body
    /// index ascending on ties).
    typedef struct m3BodyDiff
    {
        m3BodyId bodyId;     /// the id under world A's numbering
        float positionError; /// |pA - pB|, meters
        float rotationError; /// 1 - |dot(qA, qB)|, unitless
        float velocityError; /// |vA - vB| + |wA - wB|
        bool onlyInA;        /// alive in A, missing in B
        bool onlyInB;        /// alive in B, missing in A
    } m3BodyDiff;

    /// Compare every body slot of two worlds (same capacities
    /// expected; mismatched capacities refuse). Writes up to
    /// capacity rows sorted worst-first, sets outCount to the
    /// number written, and returns the TOTAL number of differing
    /// slots (which may exceed capacity), or -1 on refusal. Zero
    /// means the worlds agree body-for-body. A refusal (a NULL world,
    /// a NULL or empty array, a NULL count, different capacities) sets a
    /// given outCount to 0 and zeroes a given array over its whole
    /// capacity; a success writes only its rows. It only reads, so it is
    /// never refused as busy.
    M3_API int32_t m3World_Compare(const m3World* worldA, const m3World* worldB, m3BodyDiff* out,
                                   int32_t capacity, int32_t* outCount);

#ifdef __cplusplus
}
#endif

#endif
