// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// GJK distance and time of impact: internal declarations.

#ifndef MAUL3D_SRC_DISTANCE_H
#define MAUL3D_SRC_DISTANCE_H

#include "world_internal.h"

// GJK distance between convex proxies, results in frame A. The mesh
// manifolds, the queries and the time of impact build on this kernel.
#define M3_MAX_GJK_ITERATIONS 32

typedef struct m3DistanceProxy
{
    const m3Vec3* points;
    int32_t count;
    m3real radius;
} m3DistanceProxy;

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
    uint32_t featureA; // bit i set when A's point i is in the final simplex
} m3DistanceOutput;

m3DistanceOutput m3ShapeDistance(const m3DistanceInput* input);

// Sweep of one body's center of mass and rotation across a step, in a
// float frame the caller re-centered on the fast body's starting center
// of mass, so doubles never enter the kernel.
typedef struct m3Sweep
{
    m3Vec3 localCenter; // COM in the body frame
    m3Vec3 c1;          // begin COM, re-centered
    m3Vec3 c2;          // end COM, re-centered
    m3Quat q1;
    m3Quat q2;
} m3Sweep;

m3Transform m3GetSweepTransform(const m3Sweep* sweep, m3real time);

// The fastest the swept rotation turns, in radians per unit of sweep
// time.
m3real m3SweepAngularRateBound(const m3Sweep* sweep);

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

// Time of impact by conservative advancement over both sweeps. A hit
// stops where rounded surfaces overlap by a linear slop, or where sharp
// cores are a slop apart; a pair already that close at the start
// reports overlapped.
m3TOIOutput m3TimeOfImpact(const m3TOIInput* input);

#endif // MAUL3D_SRC_DISTANCE_H
