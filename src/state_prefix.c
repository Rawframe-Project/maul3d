// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The state formats' shared prefix: written by this release, checked on
// untrusted bytes.

#include "state_prefix.h"

#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(m3StatePrefix) == 16, "the state prefix is 16 bytes, padding-free");

m3StatePrefix m3MakeStatePrefix(uint32_t magic, uint32_t formatVersion, int32_t headerBytes)
{
    m3StatePrefix prefix;
    prefix.magic = magic;
    prefix.abiVersion = M3_ABI_VERSION;
    prefix.formatVersion = formatVersion;
    prefix.headerBytes = (uint32_t)headerBytes;
    return prefix;
}

m3Result m3CheckStatePrefix(const void* bytes, int32_t size, uint32_t magic, uint32_t formatVersion,
                            int32_t headerBytes)
{
    if (bytes == NULL || size < (int32_t)sizeof(m3StatePrefix))
    {
        return m3_errorInvalid;
    }
    m3StatePrefix prefix;
    memcpy(&prefix, bytes, sizeof(prefix));
    if (prefix.magic != magic)
    {
        return m3_errorInvalid; // not this format
    }
    if (prefix.abiVersion != M3_ABI_VERSION || prefix.formatVersion != formatVersion)
    {
        return m3_errorVersion; // written by another release
    }
    if (prefix.headerBytes != (uint32_t)headerBytes || size < headerBytes)
    {
        return m3_errorInvalid;
    }
    return m3_success;
}
