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
#define M3_VERSION_MINOR 10
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
        // An argument a correct program never passes: a null pointer, an
        // id of the wrong kind, a NaN, a value outside its parameter's
        // documented domain, a combination the call does not take.
        m3_errorInvalid = -1,
        // A fixed pool of the world, a caller buffer or the allocator is
        // too small for the result.
        m3_errorCapacity = -2,
        // An id names an object that no longer exists.
        m3_errorStale = -3,
        // The CPU cannot run the SIMD backend the library was built for.
        m3_errorUnsupported = -4,
        // The call is not allowed in the world's current state, which only
        // another call changes: a journal call in the wrong recording
        // state, or a stand-up the overlap check blocks.
        m3_errorState = -6,
        // A world def, a snapshot, a journal tape or a replay container
        // comes from another binary interface or release than the
        // library's (M3_ABI_VERSION).
        m3_errorVersion = -7,
        // Snapshot, journal tape or replay container bytes are malformed:
        // every check a reader makes on them fails with this status, a
        // bound included, unless the bytes come from another version.
        m3_errorFormat = -8,
        // An argument, or its content, passes a fixed bound the library
        // publishes as a named constant, an infinity included. More room
        // would not help. When a check finds both, an invalid value
        // outranks one past a bound.
        m3_errorLimit = -9,
        // The world is in use by work under way: a writer called from host
        // code the world is running (a task hook, the pre-solve callback,
        // a debug draw callback, the allocator). The same call succeeds
        // once that work ends.
        m3_errorBusy = -10,
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
    M3_API const char* m3GetResultName(m3Result result);

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
    /// @return Whether the CPU can run this library.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API bool m3IsCpuBackendSupported(void);

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
    M3_API uint64_t m3Hash64(uint64_t seed, const void* data, int64_t byteCount);

// The seed of an m3Hash64 chain.
#define M3_HASH_INIT 0xCBF29CE484222325ull

// Fixed bounds. A call refuses an argument, or its content, past one of
// these with m3_errorLimit; a snapshot, tape or replay container past one
// is refused with m3_errorFormat. Each lies far past any real scene, and
// together they keep every product the step forms finite.
//
// Magnitudes:
// - an offset in a body's frame (a shape's local position, a joint anchor,
//   a hull vertex, a lattice spacing, a wheel's anchor or rest length), m;
#define M3_LOCAL_LIMIT 1.0e4f
// - a shape's density, kg/m^3, which must also be positive;
#define M3_DENSITY_LIMIT 1.0e6f
// - a world position, m, per component: creates and teleports place every
//   body and soft particle within it, and the step holds it there;
#define M3_POSITION_LIMIT 1.0e15
// - a body's velocity (m/s) or spin (rad/s), a surface velocity and a
//   motor's speed; the step also holds every velocity within it;
#define M3_SPEED_LIMIT 1.0e5f
// - a kinematic body's linear speed, m/s, set or servoed, and how far a
//   kinematic target may lie from where the body is now, m;
#define M3_KINEMATIC_SPEED_LIMIT 1.0e10f
#define M3_SERVO_REACH_LIMIT     1.0e6
// - gravity, m/s^2;
#define M3_ACCEL_LIMIT 1.0e4f
// - a spring, contact, steering or suspension frequency, or the wind's
//   gust frequency, Hz;
#define M3_HERTZ_LIMIT 1.0e5f
// - a dimensionless factor: a damping ratio, restitution, a gear or pulley
//   ratio, a gravity scale, a gust or push scale;
#define M3_RATIO_LIMIT 1.0e4f
// - a step length, s;
#define M3_MIN_STEP   1.0e-4f
#define M3_STEP_LIMIT 10.0f
// - a force (N) or torque (N m), held or applied, and a joint or vehicle
//   effort;
#define M3_FORCE_LIMIT 1.0e20f
// - a mass-bearing shape's mass and smallest principal inertia, and a
//   body's total mass (a character's and a soft particle's too), kg and
//   kg m^2;
#define M3_MIN_MASS    1.0e-6f
#define M3_MIN_INERTIA 1.0e-12f
#define M3_MASS_LIMIT  1.0e20f
// - the ratio of the effective masses a joint couples;
#define M3_JOINT_LEVER_LIMIT 1.0e10
// - a character stance's half height and radius, m;
#define M3_MIN_STANCE   0.05f
#define M3_STANCE_LIMIT 10.0f
// - a vehicle wheel's radius, m;
#define M3_MIN_WHEEL_RADIUS 1.0e-3f
// - an explosion's impulse per square meter, N s/m^2;
#define M3_BLAST_LIMIT 1.0e10f
// - a cast's translation, m, per component;
#define M3_CAST_LIMIT 1.0e18f
// - a mesh vertex or height sample in its body's frame, m, per
//   component, and a height field or voxel cell's size, m, positive: a
//   grid of M3_MAX_HEIGHTFIELD_SIZE samples, or a voxel chunk, then stays
//   within M3_GEOMETRY_LIMIT;
#define M3_GEOMETRY_LIMIT 1.0e8f
#define M3_CELL_LIMIT     1.0e5f
//
// Counts:
// - a world's shape, pair and voxel capacities, which size tables of
//   eight contact pairs a shape, an eight-byte key a pair and six
//   neighbors a voxel chunk within an int32_t;
#define M3_MAX_SHAPE_CAPACITY (INT32_MAX / 8)
#define M3_MAX_PAIR_CAPACITY  (INT32_MAX / 8)
#define M3_MAX_VOXEL_CAPACITY (INT32_MAX / 6)
// - substeps per step;
#define M3_MAX_SUBSTEPS 256
// - the points a hull is built from, and the points a cast sweeps;
#define M3_MAX_HULL_POINTS 256
#define M3_MAX_CAST_POINTS 64
// - a triangle mesh's vertices, triangles and surface materials;
#define M3_MAX_MESH_VERTICES  65535
#define M3_MAX_MESH_TRIANGLES 65535
#define M3_MAX_MESH_MATERIALS 8
// - a height field grid's samples along each axis, and those of a height
//   field built as a mesh.
#define M3_MAX_HEIGHTFIELD_SIZE      255
#define M3_MAX_HEIGHTFIELD_MESH_SIZE 32
    // The soft body, vehicle and drivetrain counts are in their headers.

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BASE_H
