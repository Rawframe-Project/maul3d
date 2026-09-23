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

# Every executable (tests, samples, benchmarks, tools) lands in the top
# of the build tree, so scripts and CI find them in one place.
if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif()

include(GNUInstallDirs)
include(cmake/CompilerFlags.cmake)
include(cmake/Library.cmake)
include(cmake/Testing.cmake)

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
    src/contact_solver.c
    src/continuous.c
    src/core.c
    src/diff.c
    src/distance.c
    src/draw.c
    src/dynamic_tree.c
    src/hull.c
    src/island.c
    src/joint.c
    src/joint_distance.c
    src/joint_fixed.c
    src/joint_gear.c
    src/joint_generic.c
    src/joint_motor.c
    src/joint_parallel.c
    src/joint_prismatic.c
    src/joint_pulley.c
    src/joint_revolute.c
    src/joint_solver.c
    src/joint_spherical.c
    src/joint_wheel.c
    src/journal_replay.c
    src/manifold.c
    src/manifold_mesh.c
    src/math_functions.c
    src/mesh_bvh.c
    src/narrowphase.c
    src/query.c
    src/quickhull.c
    src/raycast.c
    src/replay.c
    src/shape.c
    src/snapshot.c
    src/softbody.c
    src/solver.c
    src/vehicle.c
    src/voxel.c
    src/world.c
    src/world_state.c)
maul_configure_library(maul3d)

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
