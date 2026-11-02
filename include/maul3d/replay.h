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

    /// Counts the records of a journal tape and the steps among them,
    /// walking its framing without replaying it.
    ///
    /// @param journal  The tape a journal wrote into its buffer, between
    ///                 m3World_StartJournal and m3World_StopJournal.
    /// @param bytes    The tape's length in bytes.
    /// @param infoOut  Receives the counts; any failure leaves it as it
    ///                 was, since the call has no owner.
    /// @return m3_success; m3_errorInvalid for a null tape or infoOut, a
    ///         negative length or one this target cannot address;
    ///         m3_errorVersion for a tape another release wrote;
    ///         m3_errorFormat for a tape too short for its header, another
    ///         format's header or a record whose framing runs past the end.
    /// @par Thread safety
    /// Safe from any thread.
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
    ///
    /// @param snapshotBytes  The snapshot's length in bytes.
    /// @param journalBytes   The journal tape's length in bytes.
    /// @return The container's length in bytes, or 0 for a length too
    ///         short to hold its own header or a pair past INT64_MAX.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API int64_t m3GetEncodedReplaySize(int64_t snapshotBytes, int64_t journalBytes);

    /// Encode a container. finalHash is the recorder's end hash
    /// (m3World_Hash after the session); verifiers compare against
    /// it. Sets *bytesOut to the bytes written; any failure leaves it as
    /// it was, since the call has no owner. Returns m3_errorVersion
    /// for a snapshot or tape another release wrote, m3_errorInvalid for
    /// a null pointer or a containerOut whose first m3GetEncodedReplaySize
    /// bytes overlap the snapshot or the tape, m3_errorFormat for sizes
    /// too short to hold a snapshot or a tape, a header that is not a
    /// snapshot's or a malformed journal, m3_errorLimit for a pair whose
    /// container no buffer of this target can hold, and m3_errorCapacity
    /// for a small buffer.
    ///
    /// @param snapshot       The session's first snapshot, as
    ///                       m3World_Snapshot wrote it.
    /// @param snapshotBytes  The snapshot's length in bytes.
    /// @param journal        The session's journal tape.
    /// @param journalBytes   The tape's length in bytes.
    /// @param finalHash      The recorder's m3World_Hash at the session's
    ///                       end.
    /// @param containerOut   Receives the container; it must not overlap
    ///                       either input.
    /// @param capacity       containerOut's length in bytes, at least
    ///                       m3GetEncodedReplaySize of the two lengths.
    /// @param bytesOut       Receives the container's length on success.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; the buffers are used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3EncodeReplay(const void* snapshot, int64_t snapshotBytes,
                                                const void* journal, int64_t journalBytes,
                                                uint64_t finalHash, void* containerOut,
                                                int64_t capacity, int64_t* bytesOut);

    /// Decode and validate framing (the container's header, the
    /// snapshot's and the tape's prefixes, lengths, journal record
    /// walk). Returns m3_errorVersion when another release wrote the
    /// container or what it holds, and m3_errorFormat on any
    /// corruption, with the view untouched either way.
    ///
    /// @param data     The container, as m3EncodeReplay wrote it.
    /// @param bytes    The container's length in bytes.
    /// @param viewOut  Receives the view, whose pointers alias data; any
    ///                 failure leaves it as it was.
    /// @return m3_success; m3_errorInvalid for a null data or viewOut, a
    ///         negative length or one this target cannot address;
    ///         m3_errorVersion or m3_errorFormat as above.
    /// @par Thread safety
    /// Safe from any thread.
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

    /// Compare every body slot of two worlds, of any capacities: a slot
    /// past one world's bodies counts as missing there. Writes up to
    /// capacity rows into rowsOut, sorted worst-first, sets *totalOut to
    /// the number of differing slots, which may exceed capacity, and
    /// returns m3_success. Zero means the worlds agree body for body.
    /// A NULL rowsOut with capacity 0 measures. Returns m3_errorInvalid
    /// for a NULL world, a NULL totalOut, a negative capacity or a NULL
    /// rowsOut with a positive one, and counts one misuse on worldA, or
    /// on worldB when worldA is NULL. A refusal sets a given totalOut
    /// to 0 and, when rowsOut and capacity pass, zeroes rowsOut over its
    /// whole capacity; a success writes only its rows. It only reads, so
    /// it is never refused as busy.
    ///
    /// @param worldA    The first world, whose ids the rows carry.
    /// @param worldB    The second world.
    /// @param rowsOut   Receives up to capacity rows, worst first; null
    ///                  with capacity 0 measures.
    /// @param capacity  rowsOut's length in rows, at least 0.
    /// @param totalOut  Receives the number of differing body slots.
    /// @return m3_success, or m3_errorInvalid as above.
    /// @par Thread safety
    /// Safe from any thread; each world is used by one thread at a time, or
    /// by several readers at once.
    M3_NODISCARD M3_API m3Result m3World_Compare(const m3World* worldA, const m3World* worldB,
                                                 m3BodyDiff* rowsOut, int32_t capacity,
                                                 int32_t* totalOut);

#ifdef __cplusplus
}
#endif

#endif
