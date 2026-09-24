cmake_minimum_required(VERSION 3.22)

include(cmake/ReadVersion.cmake)
maul_read_version(${CMAKE_CURRENT_SOURCE_DIR}/include/maul3d/base.h M3 MAUL_VERSION)
project(maul3d
    VERSION ${MAUL_VERSION}
    DESCRIPTION "Deterministic 3D physics with bit-exact rollback, replay and voxel destruction"
    LANGUAGES C)

# The family modules read their options through this prefix.
set(MAUL_PREFIX MAUL3D)

option(MAUL3D_BUILD_TESTS "Build the test suites" ${PROJECT_IS_TOP_LEVEL})
option(MAUL3D_BUILD_SAMPLES "Build the samples" ${PROJECT_IS_TOP_LEVEL})
option(MAUL3D_BUILD_BENCH "Build the benchmarks" ${PROJECT_IS_TOP_LEVEL})
option(MAUL3D_BUILD_TOOLS "Build the command-line tools" ${PROJECT_IS_TOP_LEVEL})
option(MAUL3D_BUILD_TESTBED "Build the interactive testbed (fetches raylib)" OFF)
option(MAUL3D_BUILD_SHARED "Build a shared library instead of a static one" OFF)
option(MAUL3D_INSTALL "Generate install rules" ${PROJECT_IS_TOP_LEVEL})
option(MAUL3D_WERROR "Treat compiler warnings as errors" OFF)
option(MAUL3D_SANITIZE "Instrument with AddressSanitizer and UBSan" OFF)
option(MAUL3D_TSAN "Instrument with ThreadSanitizer" OFF)
option(MAUL3D_COVERAGE "Instrument for gcov coverage" OFF)

# Engine-specific options.
set(MAUL3D_SIMD "auto" CACHE STRING "SIMD backend: auto or scalar")
set_property(CACHE MAUL3D_SIMD PROPERTY STRINGS auto scalar)

# Every executable (tests, samples, benchmarks, tools) lands in the top
# of the build tree, so scripts and CI find them in one place.
if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif()

include(GNUInstallDirs)
include(cmake/CompilerFlags.cmake)
include(cmake/Library.cmake)
include(cmake/Testing.cmake)

# x64 builds use AVX2 and FMA unless the portable scalar backend is
# chosen; every backend produces identical results.
if(MAUL3D_SIMD STREQUAL "scalar")
    list(APPEND MAUL_EXTRA_DEFINITIONS MAUL3D_SIMD_FORCE_SCALAR=1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    if(MSVC)
        set(MAUL_ARCH_FLAGS /arch:AVX2)
    else()
        set(MAUL_ARCH_FLAGS -mavx2 -mfma)
    endif()
    list(APPEND MAUL_EXTRA_DEFINITIONS MAUL3D_BUILT_FOR_AVX2=1)
endif()

if(MAUL3D_BUILD_SHARED)
    set(MAUL_LIBRARY_KIND SHARED)
else()
    set(MAUL_LIBRARY_KIND STATIC)
endif()
add_library(maul3d ${MAUL_LIBRARY_KIND}
    src/allocator.c
    src/body.c
    src/broad_phase.c
    src/character.c
    src/contact_kernel.c
    src/contact_prepare.c
    src/contact_solver.c
    src/continuous.c
    src/core.c
    src/destruction.c
    src/diff.c
    src/distance.c
    src/draw.c
    src/dynamic_tree.c
    src/hull.c
    src/island.c
    src/integrate.c
    src/joint.c
    src/joint_distance.c
    src/joint_fixed.c
    src/joint_gear.c
    src/joint_generic.c
    src/joint_motor.c
    src/joint_parallel.c
    src/joint_params.c
    src/joint_prismatic.c
    src/joint_pulley.c
    src/joint_revolute.c
    src/joint_row.c
    src/joint_solver.c
    src/joint_spherical.c
    src/joint_wheel.c
    src/journal.c
    src/journal_replay.c
    src/journal_replay_actors.c
    src/journal_replay_joints.c
    src/journal_replay_shapes.c
    src/manifold.c
    src/manifold_mesh.c
    src/math_functions.c
    src/mesh_bvh.c
    src/mover.c
    src/narrowphase.c
    src/query.c
    src/query_proxy.c
    src/quickhull.c
    src/raycast.c
    src/replay.c
    src/shape.c
    src/shape_cast.c
    src/shape_create.c
    src/shape_data.c
    src/shape_mass.c
    src/shape_params.c
    src/snapshot.c
    src/softbody.c
    src/softbody_pairs.c
    src/softbody_solver.c
    src/solver.c
    src/triangle_contact.c
    src/vehicle.c
    src/vehicle_drivetrain.c
    src/voxel.c
    src/voxel_fracture.c
    src/water.c
    src/world.c
    src/world_events.c
    src/world_hash.c
    src/world_state.c)
maul_configure_library(maul3d)

# The CPU check must run on CPUs the backend does not support, so it is
# compiled without the architecture flags and linked into the library.
add_library(maul3d_cpu OBJECT src/cpu.c)
set(maul_saved_arch_flags "${MAUL_ARCH_FLAGS}")
set(MAUL_ARCH_FLAGS "")
maul_apply_flags(maul3d_cpu)
set(MAUL_ARCH_FLAGS "${maul_saved_arch_flags}")
target_include_directories(maul3d_cpu PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
set_target_properties(maul3d_cpu PROPERTIES C_VISIBILITY_PRESET hidden POSITION_INDEPENDENT_CODE ON)
if(MAUL3D_BUILD_SHARED)
    target_compile_definitions(maul3d_cpu PRIVATE MAUL3D_SHARED maul3d_EXPORTS)
endif()
target_sources(maul3d PRIVATE $<TARGET_OBJECTS:maul3d_cpu>)

if(MAUL3D_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
if(MAUL3D_BUILD_SAMPLES)
    add_subdirectory(samples)
endif()
if(MAUL3D_BUILD_BENCH)
    add_subdirectory(bench)
endif()
if(MAUL3D_BUILD_TOOLS AND EXISTS ${PROJECT_SOURCE_DIR}/tools/CMakeLists.txt)
    add_subdirectory(tools)
endif()
if(MAUL3D_BUILD_TESTBED)
    add_subdirectory(testbed)
endif()
if(MAUL3D_INSTALL)
    include(cmake/Install.cmake)
endif()
