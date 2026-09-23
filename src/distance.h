// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// GJK distance and time of impact: internal declarations.

#ifndef MAUL3D_SRC_DISTANCE_H
#define MAUL3D_SRC_DISTANCE_H

#include "world_internal.h"

// GJK distance, adapted from the reference distance.c: convex
// proxies, a warm-startable simplex cache, results in frame A. The
// SAT manifolds and the TOI build on this kernel.
#define M3_MAX_GJK_ITERATIONS 32

typedef struct m3DistanceProxy
{
    const m3Vec3* points;
    int32_t count;
    m3real radius;
} m3DistanceProxy;

typedef struct m3SimplexCache
{
    m3real metric;
    uint16_t count;
    uint8_t indexA[4];
    uint8_t indexB[4];
} m3SimplexCache;

typedef struct m3DistanceInput
{
    m3DistanceProxy proxyA;
    m3DistanceProxy proxyB;
    m3Quat q; // rotation of B in A's frame
    m3Vec3 p; // position of B in A's frame (caller localizes doubles)
    bool useRadii;
} m3DistanceInput;

typedef struct m3DistanceOutput
{
    m3Vec3 pointA; // frame A
    m3Vec3 pointB;
    m3Vec3 normal;   // A toward B (zero on overlap)
    m3real distance; // zero on overlap
    int32_t iterations;
} m3DistanceOutput;

m3DistanceOutput m3ShapeDistance(const m3DistanceInput* input, m3SimplexCache* cache);

// Sweep of one body's COM and rotation across a step, in a float
// frame the caller re-centered (the reference precision trick: TOI
// runs relative to the fast body's begin COM so doubles never enter
// the kernel).
typedef struct m3Sweep
{
    m3Vec3 localCenter; // COM in the body frame
    m3Vec3 c1;          // begin COM, re-centered
    m3Vec3 c2;          // end COM, re-centered
    m3Quat q1;
    m3Quat q2;
} m3Sweep;

m3Transform m3GetSweepTransform(const m3Sweep* sweep, m3real time);

typedef struct m3TOIInput
{
    m3DistanceProxy proxyA; // the target shape
    m3DistanceProxy proxyB; // the fast shape
    m3Sweep sweepA;
    m3Sweep sweepB;
    m3real maxFraction;
} m3TOIInput;

typedef enum m3TOIState
{
    m3_toiStateUnknown = 0,
    m3_toiStateFailed,
    m3_toiStateOverlapped,
    m3_toiStateHit,
    m3_toiStateSeparated,
} m3TOIState;

typedef struct m3TOIOutput
{
    m3TOIState state;
    m3real fraction;
    m3Vec3 normal; // A toward B at the hit (valid on hit only)
} m3TOIOutput;

// Time of impact by conservative advancement with root finding
//, adapted from the reference distance.c: separation
// functions built from the GJK simplex cache (vertices, edge pairs,
// faces), deepest-point push-back, mixed false-position and
// bisection roots. Both sweeps participate: dynamic versus dynamic
// is a first-class citizen.
m3TOIOutput m3TimeOfImpact(const m3TOIInput* input);

#endif // MAUL3D_SRC_DISTANCE_H
