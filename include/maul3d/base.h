// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The base of the Maul3D API: the library version, the export and
// attribute macros, the result codes every fallible function returns,
// the allocator, the ids every header shares, and hashing.

#ifndef MAUL3D_BASE_H
#define MAUL3D_BASE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Marks the public API. Shared builds export it (dllexport and
/// dllimport on Windows, default visibility elsewhere); maul3d_EXPORTS
/// is defined by CMake while the library itself compiles.
#if defined(MAUL3D_SHARED) && defined(_WIN32)
#if defined(maul3d_EXPORTS)
#define M3_API __declspec(dllexport) extern
#else
#define M3_API __declspec(dllimport) extern
#endif
#elif defined(MAUL3D_SHARED) && (defined(__GNUC__) || defined(__clang__))
#define M3_API __attribute__((visibility("default"))) extern
#else
#define M3_API extern
#endif

// M3_NODISCARD marks a function whose result must be read: every
// function that returns a status. The attribute is standard in C23 and
// C++17 and left out for older dialects. MSVC keeps __cplusplus at
// 199711L unless /Zc:__cplusplus is given, so _MSVC_LANG is read too.
#if (defined(__cplusplus) && __cplusplus >= 201703L) ||                                            \
    (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)
#define M3_NODISCARD [[nodiscard]]
#elif !defined(__cplusplus) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L
#define M3_NODISCARD [[nodiscard]]
#else
#define M3_NODISCARD
#endif

// M3_ALIGN8 gives a 64-bit member (a double or a 64-bit integer) 8-byte
// alignment on every target. 32-bit x86 aligns them to 4 inside a struct,
// and the journal and snapshot formats are laid out as on x86-64, with
// every such member 8-aligned; src/format_layout.c holds them there.
#if defined(__cplusplus) || (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L)
#define M3_ALIGN8 alignas(8)
#else
#define M3_ALIGN8 _Alignas(8)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// The library version. CMake reads it from here, and release-please
// moves it between the markers.
// x-release-please-start-major
#define M3_VERSION_MAJOR 0
// x-release-please-end
// x-release-please-start-minor
#define M3_VERSION_MINOR 7
// x-release-please-end
// x-release-please-start-patch
#define M3_VERSION_PATCH 0
// x-release-please-end

// The binary interface a program is built for: the whole version before
// 1.0, when any release may break it, and the major and minor version from
// 1.0 on. The top bit is always set, so a zeroed def never matches. The
// world def carries it (m3DefaultWorldDef), and m3CreateWorld refuses a
// def built for another; every other def is used inside a world.
#define M3_ABI_VERSION                                                                             \
    (UINT32_C(0x80000000) | ((uint32_t)M3_VERSION_MAJOR << 16) |                                   \
     (M3_VERSION_MAJOR == 0 ? (((uint32_t)M3_VERSION_MINOR << 8) | (uint32_t)M3_VERSION_PATCH)     \
                            : (uint32_t)M3_VERSION_MINOR))
#if M3_VERSION_MAJOR > 0x7FFF || M3_VERSION_MINOR > 0xFFFF
#error "M3_VERSION_MAJOR or M3_VERSION_MINOR does not fit M3_ABI_VERSION"
#endif
#if M3_VERSION_MAJOR == 0 && (M3_VERSION_MINOR > 0xFF || M3_VERSION_PATCH > 0xFF)
#error "before 1.0, M3_VERSION_MINOR and M3_VERSION_PATCH must fit 8 bits each"
#endif

    // The status a fallible function returns. Zero is success, negative
    // values are errors. The type has a fixed width so that structs holding
    // a result have one layout in C and C++.
    typedef int32_t m3Result;

    enum
    {
        // The call did what was asked.
        m3_success = 0,
        // An argument or def is invalid, or a snapshot or journal tape
        // fails its checks (another build, another world shape, damaged
        // bytes).
        m3_errorInvalid = -1,
        // A fixed pool of the world, a caller buffer or the allocator is
        // too small for the result.
        m3_errorCapacity = -2,
        // An id names an object that no longer exists.
        m3_errorStale = -3,
        // The CPU cannot run the SIMD backend the library was built for.
        m3_errorUnsupported = -4,
        // The call is not allowed now: a writer called from host code the
        // world is running (a task hook, the pre-solve callback, a debug
        // draw callback, the allocator), a journal call in the wrong
        // recording state, or a stand-up the overlap check blocks.
        m3_errorState = -6,
        // A world def was built from headers of another binary interface
        // than the library's (M3_ABI_VERSION).
        m3_errorVersion = -7,
    };

    // The allocator a world takes in its def and keeps for its lifetime.
    // Alignment is a power of two. A zeroed allocator means the C library's
    // allocation functions.
    typedef struct m3Allocator
    {
        void* (*alloc)(size_t size, size_t alignment, void* context);
        void (*free)(void* memory, size_t size, size_t alignment, void* context);
        void* context;
    } m3Allocator;

    // A library version: major, minor and patch.
    typedef struct m3Version
    {
        uint16_t major;
        uint16_t minor;
        uint16_t patch;
    } m3Version;

    /// Returns the version of the library that was linked, which may differ
    /// from the M3_VERSION macros a program was compiled with.
    ///
    /// @return The library version.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3Version m3GetVersion(void);

    /// Returns the name of a result code, for diagnostics.
    ///
    /// @param result  Any value; an unknown one is named as such.
    /// @return A static, NUL-terminated string such as "m3_errorCapacity".
    /// @par Thread safety
    /// Safe from any thread.
    M3_API const char* m3ResultName(m3Result result);

    /// Returns the SIMD backend the library was compiled for: "avx2",
    /// "neon" or "scalar". Every backend produces the same bits; this only
    /// reports which kernels the binary carries.
    ///
    /// @return A static, NUL-terminated string.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API const char* m3GetSimdBackend(void);

    /// Returns whether the CPU running the call supports the compiled
    /// backend. An "avx2" binary needs AVX2 and FMA3 with the wide registers
    /// enabled by the operating system; "neon" and "scalar" run on every CPU
    /// of their architecture. m3CreateWorld refuses a CPU without the
    /// backend with m3_errorUnsupported, so a program can call this first to
    /// choose another path, or build with -DMAUL3D_SIMD=scalar.
    ///
    /// @return 1 when the CPU can run this library, 0 when it cannot.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API int32_t m3CpuSupportsBackend(void);

    // Ids of the objects a world owns: a 1-based slot (0 is the null id)
    // and the generation the slot had when the object was made, so an id
    // whose object is gone is told apart from the slot's next object. An
    // id means something only to the world that issued it: given to
    // another world, it names whatever that world holds in the same slot
    // with the same generation, which the library cannot tell apart.
    typedef struct m3BodyId
    {
        uint32_t index1;
        uint32_t generation;
    } m3BodyId;

    typedef struct m3ShapeId
    {
        uint32_t index1;
        uint32_t generation;
    } m3ShapeId;

    typedef struct m3JointId
    {
        uint32_t index1;
        uint32_t generation;
    } m3JointId;

    static const m3JointId m3_nullJointId = {0, 0};

    /// The 64-bit hash every determinism check is built on: eight bytes a
    /// round, xored in, multiplied by an odd constant and folded, the
    /// leftover bytes one at a time (FNV-1a). Its constants are frozen.
    ///
    /// @param seed       The running hash; start a chain with M3_HASH_INIT.
    /// @param data       The bytes to fold in; may be null when byteCount is 0.
    /// @param byteCount  The number of bytes, at least 0.
    /// @return The new running hash.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API uint64_t m3Hash64(uint64_t seed, const void* data, int32_t byteCount);

// The seed of an m3Hash64 chain.
#define M3_HASH_INIT 0xCBF29CE484222325ull

// The largest snapshot in bytes, INT32_MAX: snapshot sizes are int32_t
// byte counts, and maul3d's journal records ops and never embeds a
// snapshot, so a snapshot may take the whole range. (A replay container,
// which holds one beside a tape, refuses a pair past INT32_MAX: see
// m3GetEncodedReplaySize.) A world's snapshot never passes it:
// m3CreateWorld refuses a def whose world could with its mesh and hull
// content at the largest, and m3CreateHeightFieldGridShape refuses
// samples that would carry it past.
#define M3_MAX_SNAPSHOT_BYTES 2147483647

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BASE_H
