// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The state formats' shared prefix, and each format's magic, number and
// header size: the snapshot, the journal tape and the replay container.

#ifndef MAUL3D_SRC_STATE_PREFIX_H
#define MAUL3D_SRC_STATE_PREFIX_H

#include "maul3d/base.h"

#include <stdint.h>

// Each format's magic, its own number, bumped whenever its bytes change,
// and the size of its whole header, the prefix and its own fields. A magic
// is its name's four letters in file order, so a hex dump reads the name:
// the little-endian uint32_t of "M3SN" is 0x4E53334D.
#define M3_SNAPSHOT_MAGIC        0x4E53334Du // "M3SN"
#define M3_SNAPSHOT_VERSION      5u          // 5: the state prefix
#define M3_SNAPSHOT_HEADER_BYTES 56
#define M3_JOURNAL_MAGIC         0x4E4A334Du // "M3JN"
#define M3_JOURNAL_VERSION       1u          // 1: the state prefix
#define M3_JOURNAL_HEADER_BYTES  16
#define M3_REPLAY_MAGIC          0x5052334Du // "M3RP"
#define M3_REPLAY_VERSION        3u          // 3: the state prefix
#define M3_REPLAY_HEADER_BYTES   40

// The 16 bytes every state format starts with, the snapshot's, the
// journal tape's and the replay container's alike, in the layout maul2d
// shares: the format's magic, the M3_ABI_VERSION of the release that
// wrote it, the format's own number and the size of the whole header.
// Little-endian like the rest.
typedef struct m3StatePrefix
{
    uint32_t magic;
    uint32_t abiVersion;
    uint32_t formatVersion;
    uint32_t headerBytes;
} m3StatePrefix;

// The prefix this release writes for a format.
m3StatePrefix m3MakeStatePrefix(uint32_t magic, uint32_t formatVersion, int32_t headerBytes);

// Checks the prefix of untrusted bytes, in the order every state format
// shares: too short or another magic is m3_errorInvalid, since the bytes
// are not this format; another release or format number is
// m3_errorVersion, since only the release that wrote them reads them; a
// header size that is not this format's, or bytes too short to hold it,
// is m3_errorInvalid. Counts nothing: the bytes are untrusted.
m3Result m3CheckStatePrefix(const void* bytes, int32_t size, uint32_t magic, uint32_t formatVersion,
                            int32_t headerBytes);

#endif // MAUL3D_SRC_STATE_PREFIX_H
