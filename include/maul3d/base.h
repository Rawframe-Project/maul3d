// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Base definitions shared by every Maul3D header: the version, the
// export macro, the result codes, the allocator and assert hooks, and
// hashing.

#ifndef MAUL3D_BASE_H
#define MAUL3D_BASE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// Export decoration. Static builds (the default) need none; shared
/// builds export with default visibility on ELF and Mach-O and with
/// dllexport/dllimport on Windows (maul3d_EXPORTS is defined by the
/// build of the library itself, never by consumers).
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

    /// Library version, encoded as major * 10000 + minor * 100 + patch.
#define M3_VERSION_MAJOR 0
#define M3_VERSION_MINOR 0
#define M3_VERSION_PATCH 1

    /// The linked library's version as major * 10000 + minor * 100 +
    /// patch (0.3.0 returns 300). Compare against the M3_VERSION
    /// macros to catch a header/library mismatch at startup.
    M3_API int m3GetVersion(void);

    /// The SIMD backend this library was COMPILED against: "avx2", "neon"
    /// or "scalar". It is a compile-time choice, and every backend produces
    /// bit-identical results by contract; this only reports which kernels
    /// the binary carries. Thread class: reader.
    M3_API const char* m3GetSimdBackend(void);

    /// Whether the CPU running this call actually supports the compiled
    /// backend: 1 if it can run, 0 if not. An "avx2" binary needs AVX2 and
    /// FMA3 with OS wide-register support; "neon" is architectural on
    /// arm64 and "scalar" runs anywhere, so both return 1. Creating a world
    /// on a CPU that returns 0 is refused (m3_errorConfig) rather than
    /// trapping on an illegal instruction; check this first for a graceful
    /// path, or build with -DMAUL3D_SIMD=scalar for a portable binary.
    /// Thread class: reader.
    M3_API int32_t m3CpuSupportsBackend(void);

    /// Routes every internal allocation through your hooks. Install them
    /// BEFORE the first world and never change them while any world
    /// lives. The alloc hook may return uninitialized memory (the engine
    /// zeroes what it needs); a NULL return is refused loudly upstream.
    /// Pass both hooks or neither: NULLs restore the C library's calloc
    /// and free. The context is handed back to both hooks, so hosts can
    /// meter or budget per arena.
    typedef void* m3AllocFn(size_t bytes, void* context);
    typedef void m3FreeFn(void* memory, void* context);
    M3_API void m3SetAllocator(m3AllocFn* allocFn, m3FreeFn* freeFn, void* context);

    /// Why the last call on this thread refused. Every API that rejects
    /// its input (a bad def, a stale or wrong-kind id, a full pool)
    /// records the reason here before it returns its null result. The
    /// slot is per thread and success paths do not clear it: read it
    /// right after the refusal you care about.
    typedef enum m3Result
    {
        m3_success = 0,
        m3_errorInvalid = 1,  // bad def or argument, stale id, wrong body or joint type
        m3_errorCapacity = 2, // a fixed pool, slot table or allocation ran out
        m3_errorConfig = 3,   // another build or world shape, or a CPU without the SIMD backend
    } m3Result;
    M3_API m3Result m3LastResult(void);

    /// Opaque generation-tagged handles: the only identity, public and
    /// internal. index1 is 1-based (0 means null) and the generation
    /// detects stale handles after slot reuse. Handles encode no
    /// address, so they survive snapshot restore unchanged.
    typedef struct m3WorldId
    {
        uint16_t index1;
        uint16_t generation;
    } m3WorldId;

    /// Ids of objects inside a world. world names the world that
    /// handed the id out: its slot in the low M3_WORLD_SLOT_BITS bits
    /// and the low bits of its generation above them, so an id from a
    /// destroyed world is refused by the next world in the same slot
    /// (until the slot has been reused 1024 times). An id is valid
    /// until its object or its world is destroyed. Using a stale id is
    /// a contract violation that never crashes: getters return zeros,
    /// commands and destroys no-op, creates refuse.
#define M3_WORLD_SLOT_BITS 6
    typedef struct m3BodyId
    {
        int32_t index1;
        uint16_t world;
        uint16_t generation;
    } m3BodyId;

    typedef struct m3ShapeId
    {
        int32_t index1;
        uint16_t world;
        uint16_t generation;
    } m3ShapeId;

    /// Joint handle, same 8-byte law as bodies and shapes.
    typedef struct m3JointId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world;
        uint16_t generation;
    } m3JointId;

    static const m3JointId m3_nullJointId = {0, 0, 0};

    /// The 64-bit hash every determinism gate is built on: eight bytes a
    /// round, xored in, multiplied by an odd constant and folded, the
    /// leftover bytes one at a time (FNV-1a). Its constants are frozen.
    /// Seed with M3_HASH_INIT, fold bytes in canonical order.
#define M3_HASH_INIT 0xCBF29CE484222325ull

    M3_API uint64_t m3Hash64(uint64_t seed, const void* data, int32_t byteCount);

    /// Host assert hook: called before the default print-and-abort for
    /// every failed internal invariant. Return nonzero to declare the
    /// failure handled and suppress the abort (crash reporters, test
    /// harnesses). NULL restores the default. The hook only observes;
    /// it never touches simulation state.
    typedef int m3AssertFn(const char* condition, const char* file, int line, void* context);
    M3_API void m3SetAssertHandler(m3AssertFn* handler, void* context);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BASE_H
