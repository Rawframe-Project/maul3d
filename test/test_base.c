// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The base calls: the linked version, the result names, the CPU check
// and the hash every determinism gate is built on, whose constants are
// frozen.

#include "test_harness.h"

#include "maul3d/base.h"

#include <stdio.h>
#include <string.h>

static void TestVersion(void)
{
    m3Version v = m3GetVersion();
    CHECK(v.major == M3_VERSION_MAJOR && v.minor == M3_VERSION_MINOR && v.patch == M3_VERSION_PATCH,
          "the linked version is the headers' version");
}

static void TestResultNames(void)
{
    CHECK(strcmp(m3GetResultName(m3_success), "m3_success") == 0, "success is named");
    CHECK(strcmp(m3GetResultName(m3_errorBusy), "m3_errorBusy") == 0, "busy is named");
    CHECK(m3GetResultName(-31) != NULL, "an unknown result still gets a name");
}

static void TestCpuCheck(void)
{
    // The tests run, so the CPU runs the backend they were built for.
    bool supported = m3IsCpuBackendSupported();
    CHECK(supported, "the CPU running the tests supports the built backend");
    CHECK(m3GetSimdBackend() != NULL, "the backend is named");
}

static void TestHash(void)
{
    const char text[] = "Maul3D hashes 64-bit sizes";
    int64_t bytes = (int64_t)(sizeof(text) - 1);
    CHECK(m3Hash64(M3_HASH_INIT, text, bytes) == 0x95553657EC6D1F27ull,
          "the hash of a pinned string keeps its frozen value");
    CHECK(m3Hash64(M3_HASH_INIT, NULL, 0) == M3_HASH_INIT, "no bytes leave the seed as it is");
}

int main(void)
{
    TestVersion();
    TestResultNames();
    TestCpuCheck();
    TestHash();
    if (s_failures == 0)
    {
        printf("test_base: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
