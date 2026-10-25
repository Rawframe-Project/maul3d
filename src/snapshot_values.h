// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The incoming snapshot as restore's whole-structure checks read it, and
// the per-record checks (snapshot_values.c). Every read goes through
// memcpy: the incoming bytes are unaligned.

#ifndef MAUL3D_SRC_SNAPSHOT_VALUES_H
#define MAUL3D_SRC_SNAPSHOT_VALUES_H

#include "world_internal.h"
#include "world_state.h"

#include <stddef.h>
#include <string.h>

// The world the snapshot restores into, the incoming fixed prefix (the
// bytes after the header), whose blocks m3StateValidate has passed, and
// scratch as long as the largest pool.
typedef struct m3Incoming
{
    const m3World* world;
    const uint8_t* in;
    int32_t* scratch;
} m3Incoming;

// The incoming block of the world field at offset (an m3World member).
static inline const uint8_t* m3InField(const m3Incoming* inc, size_t offset)
{
    int32_t at = m3StateFieldAt(inc->world, offset);
    M3_ASSERT(at >= 0); // every field read here is snapshot state
    return inc->in + at;
}

static inline int32_t m3InI32(const uint8_t* block, int32_t i)
{
    int32_t value;
    memcpy(&value, block + 4 * (size_t)i, sizeof(value));
    return value;
}

static inline uint16_t m3InU16(const uint8_t* block, int32_t i)
{
    uint16_t value;
    memcpy(&value, block + 2 * (size_t)i, sizeof(value));
    return value;
}

static inline uint32_t m3InU32(const uint8_t* block, int32_t i)
{
    uint32_t value;
    memcpy(&value, block + 4 * (size_t)i, sizeof(value));
    return value;
}

static inline uint64_t m3InU64(const uint8_t* block, int32_t i)
{
    uint64_t value;
    memcpy(&value, block + 8 * (size_t)i, sizeof(value));
    return value;
}

static inline float m3InF32(const uint8_t* block, int32_t i)
{
    float value;
    memcpy(&value, block + 4 * (size_t)i, sizeof(value));
    return value;
}

// Whether every live record's values are ones its setters accept.
bool m3SnapshotValuesValid(const m3Incoming* inc);

#endif // MAUL3D_SRC_SNAPSHOT_VALUES_H
