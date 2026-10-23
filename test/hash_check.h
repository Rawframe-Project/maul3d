// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Compares a suite's world hash with the value committed in
// test/hashes.txt, which test/CMakeLists.txt passes in as
// M3_GOLDEN_<NAME>. Every CI cell compares with the same file, so the
// cells agree bit for bit, and a change that moves a hash on every
// platform fails too. The suite prints the line the file would need, and
// tools/check_hashes.py --update collects them after a deliberate change.

#ifndef MAUL3D_TEST_HASH_CHECK_H
#define MAUL3D_TEST_HASH_CHECK_H

#include "test_harness.h"

#include <stdint.h>
#include <stdio.h>

static inline void CheckHash(const char* name, uint64_t hash, uint64_t committed)
{
    printf("M3_%s_HASH=%016llx\n", name, (unsigned long long)hash);
    if (hash != committed)
    {
        printf("FAIL: the %s hash is %016llx; test/hashes.txt has %016llx\n", name,
               (unsigned long long)hash, (unsigned long long)committed);
        s_failures += 1;
    }
}

#endif // MAUL3D_TEST_HASH_CHECK_H
