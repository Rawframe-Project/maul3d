// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soft body pass culls a shape no particle can reach, and the cull
// must change nothing: every scene here runs twice, once with the cull
// and once testing every particle against every shape, and the two
// worlds must end byte for byte the same. The scenes put particles where
// a wrong cull would show: on each culled kind's threshold and a few
// float ulps either side of it, on moving and rotating bodies, far from
// the origin, and where one shape's projection carries a particle into a
// second shape's reach within the same pass. Each threshold scene also
// checks that some of its particles are projected and some are not, so
// the threshold is really straddled.

#include "helpers.h"
#include "test_harness.h"
#include "world_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    MAX_PROBES = 400
};

typedef struct Probe
{
    m3SoftBodyId ids[MAX_PROBES];
    m3Pos3 start[MAX_PROBES];
    int32_t count;
} Probe;

typedef void BuildScene(m3World* world, Probe* probe);

static m3World* SceneWorld(bool cullOff)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.voxelCapacity = 16;
    def.softBodyCapacity = MAX_PROBES;
    m3World* world = NewWorld(&def);
    world->softBodies.softCullOff = cullOff ? 1 : 0;
    return world;
}

// One particle of radius r, at rest at p, with gravity off in the scene:
// it stays where it is put until a shape projects it.
static void AddProbe(m3World* world, Probe* probe, m3Pos3 p, m3real r)
{
    if (probe->count == MAX_PROBES)
    {
        return;
    }
    m3SoftBodyDef sd = m3DefaultSoftBodyDef();
    sd.position = p;
    sd.countX = 1;
    sd.countY = 1;
    sd.countZ = 1;
    sd.radius = r;
    probe->ids[probe->count] = NewSoftBody(world, &sd);
    probe->start[probe->count] = m3SoftBody_GetParticlePosition(world, probe->ids[probe->count], 0);
    probe->count += 1;
}

static m3BodyId StaticBody(m3World* world, m3Pos3 p, m3Quat q)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = p;
    bd.rotation = q;
    return NewBody(world, &bd);
}

static m3Quat Turn(m3Vec3 axis, float angle)
{
    float n = sqrtf(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    float s = sinf(0.5f * angle) / n;
    return (m3Quat){axis.x * s, axis.y * s, axis.z * s, cosf(0.5f * angle)};
}

static m3Pos3 Along(m3Pos3 from, m3Vec3 dir, double distance)
{
    double x = (double)dir.x;
    double y = (double)dir.y;
    double z = (double)dir.z;
    double n = sqrt(x * x + y * y + z * z);
    return (m3Pos3){from.x + distance * x / n, from.y + distance * y / n,
                    from.z + distance * z / n};
}

static m3Pos3 WorldPoint(m3Pos3 p, m3Quat q, m3Vec3 local)
{
    m3Vec3 r = m3RotateVec3(q, local);
    return (m3Pos3){p.x + (double)r.x, p.y + (double)r.y, p.z + (double)r.z};
}

// Distances on a threshold t: t itself and t moved by k float ulps of t,
// for k = -6..6, so the projection's float compare falls both ways.
static double Ulps(double t, int32_t k)
{
    return t * (1.0 + (double)k * (double)FLT_EPSILON);
}

static void SphereScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {3.0, 2.0, -1.0};
    m3Quat q = Turn((m3Vec3){1.0f, 2.0f, 0.5f}, 0.7f);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.3f, 0.0f, 0.2f}, 0.7f};
    NewSphereShape(world, StaticBody(world, p, q), &sd, &sphere);
    m3Pos3 c = WorldPoint(p, q, sphere.center);
    m3real r = 0.05f;
    // Spread over the sphere so no two probes touch: each takes its own
    // direction and an ulp offset in turn.
    enum
    {
        N = 91
    };
    for (int32_t i = 0; i < N; ++i)
    {
        double z = 1.0 - (2.0 * i + 1.0) / N;
        double ring = sqrt(1.0 - z * z);
        double angle = 2.399963229728653 * i;
        m3Vec3 dir = {(float)(ring * cos(angle)), (float)(ring * sin(angle)), (float)z};
        AddProbe(world, probe, Along(c, dir, Ulps(0.7 + 0.05, i % 13 - 6)), r);
    }
}

static void CapsuleScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {-2.0, 1.0, 4.0};
    m3Quat q = Turn((m3Vec3){0.0f, 1.0f, 1.0f}, 1.1f);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{-0.5f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, 0.3f};
    NewCapsuleShape(world, StaticBody(world, p, q), &sd, &capsule);
    m3real r = 0.04f;
    // Around the segment at three stations, each probe at its own angle,
    // and past both ends.
    for (int32_t at = 0; at < 3; ++at)
    {
        float x = -0.45f + 0.45f * (float)at;
        for (int32_t i = 0; i < 13; ++i)
        {
            float angle = 0.483f * (float)i + 0.2f * (float)at;
            float t = (float)Ulps(0.3 + 0.04, i - 6);
            AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){x, t * cosf(angle), t * sinf(angle)}),
                     r);
        }
    }
    for (int32_t i = 0; i < 5; ++i)
    {
        float t = 0.5f + (float)Ulps(0.3 + 0.04, 3 * i - 6);
        float side = (float)(i - 2) * 0.09f;
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){t, side, 0.0f}), r);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){-t, 0.0f, side}), r);
    }
}

static void VoxelScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {10.0, 0.0, -6.0};
    m3Quat q = Turn((m3Vec3){0.0f, 1.0f, 0.0f}, 0.4f);
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 4; ++z)
    {
        for (int32_t y = 0; y < 4; ++y)
        {
            for (int32_t x = 0; x < 4; ++x)
            {
                voxels[x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z)] = 1;
            }
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    NewVoxelChunkShape(world, StaticBody(world, p, q), &sd, voxels, NULL, 0.5f);
    m3real r = 0.06f;
    // Off the block's +x, +y and +z faces on a grid of spots 0.15 apart,
    // each spot its own ulp offset; and one inside a filled cell near the
    // surface, where the escape kernel acts.
    for (int32_t i = 0; i < 13; ++i)
    {
        float u = 0.2f + 0.15f * (float)(i % 4);
        float v = 0.2f + 0.45f * (float)(i / 4);
        float t = 2.0f + (float)Ulps(0.06, i - 6);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){t, u, v}), r);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){u, t, v}), r);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){u, v, t}), r);
    }
    AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){1.0f, 1.0f, 1.0f}), r);
}

static void MeshScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {-8.0, -1.0, -3.0};
    m3Quat q = Turn((m3Vec3){1.0f, 0.0f, 0.3f}, 0.5f);
    static const m3Vec3 quad[4] = {
        {-2.0f, 0.0f, -2.0f}, {2.0f, 0.0f, -2.0f}, {2.0f, 0.0f, 2.0f}, {-2.0f, 0.0f, 2.0f}};
    static const uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    m3ShapeDef sd = m3DefaultShapeDef();
    NewMeshShape(world, StaticBody(world, p, q), &sd, quad, 4, tris, 2);
    m3real r = 0.05f;
    // Above and below the faces on a grid 0.15 apart, over the diagonal
    // between the triangles, past an edge and past a corner.
    for (int32_t i = 0; i < 13; ++i)
    {
        float h = (float)Ulps(0.05, i - 6);
        float x = -1.5f + 0.25f * (float)i;
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){x, h, -1.1f}), r);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){x, -h, 0.9f}), r);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){x, h, x}), r);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){2.0f + h, 0.0f, x}), r);
    }
    for (int32_t i = 0; i < 5; ++i)
    {
        float d = (float)Ulps(0.05, 3 * i - 6) * 0.70710678f;
        float a = 0.3f * (float)(i - 2);
        AddProbe(world, probe,
                 WorldPoint(p, q, (m3Vec3){2.0f + d * cosf(a), d * sinf(a), 2.0f + d}), r);
    }
}

// Large particles over a voxel chunk of isolated voxels, each its own
// box, so a particle's reach covers dozens of boxes at once.
static void DenseVoxelScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {-20.0, 0.0, 20.0};
    m3Quat q = Turn((m3Vec3){0.0f, 1.0f, 0.0f}, 0.3f);
    static uint8_t voxels[M3_VOXEL_COUNT];
    for (int32_t z = 0; z < M3_VOXEL_DIM; ++z)
    {
        for (int32_t y = 0; y < M3_VOXEL_DIM; ++y)
        {
            for (int32_t x = 0; x < M3_VOXEL_DIM; ++x)
            {
                voxels[x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z)] =
                    (uint8_t)(((x + y + z) & 1) == 0);
            }
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    NewVoxelChunkShape(world, StaticBody(world, p, q), &sd, voxels, NULL, 0.25f);
    // Above the top face, against the +x face and inside the chunk.
    for (int32_t i = 0; i < 5; ++i)
    {
        float u = 0.5f + 0.75f * (float)i;
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){u, 4.5f, u}), 1.5f);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){4.5f, u, 2.0f}), 1.5f);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){u, 2.0f, 2.0f}), 1.5f);
    }
}

// Large particles over a fine mesh grid, 3200 triangles 0.1 m across, so
// a particle's reach covers hundreds of them at once.
static void DenseMeshScene(m3World* world, Probe* probe)
{
    enum
    {
        CELLS = 40
    };
    m3Pos3 p = {20.0, -2.0, 20.0};
    m3Quat q = Turn((m3Vec3){1.0f, 0.0f, 0.2f}, 0.2f);
    static m3Vec3 vertices[(CELLS + 1) * (CELLS + 1)];
    static uint16_t tris[CELLS * CELLS * 6];
    int32_t n = 0;
    for (int32_t z = 0; z <= CELLS; ++z)
    {
        for (int32_t x = 0; x <= CELLS; ++x)
        {
            float bump = 0.02f * (float)((x * 7 + z * 3) % 5);
            vertices[x + (CELLS + 1) * z] = (m3Vec3){0.1f * (float)x, bump, 0.1f * (float)z};
        }
    }
    for (int32_t z = 0; z < CELLS; ++z)
    {
        for (int32_t x = 0; x < CELLS; ++x)
        {
            uint16_t a = (uint16_t)(x + (CELLS + 1) * z);
            uint16_t c = (uint16_t)(a + CELLS + 1);
            const uint16_t quad[6] = {
                a, c, (uint16_t)(a + 1), (uint16_t)(a + 1), c, (uint16_t)(c + 1)};
            memcpy(&tris[n], quad, sizeof(quad));
            n += 6;
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    NewMeshShape(world, StaticBody(world, p, q), &sd, vertices, (CELLS + 1) * (CELLS + 1), tris,
                 CELLS * CELLS * 2);
    // Just above and just below the grid, and on its edge.
    for (int32_t i = 0; i < 5; ++i)
    {
        float u = 0.5f + 0.75f * (float)i;
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){u, 1.2f, u}), 1.5f);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){u, -0.4f, 2.0f}), 1.5f);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){0.0f, 0.5f, u}), 1.5f);
    }
}

static void HeightFieldScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {0.0, -4.0, 12.0};
    m3Quat q = Turn((m3Vec3){0.0f, 1.0f, 0.0f}, 0.0f);
    static const float heights[16] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f,
                                      0.5f, 0.5f, 0.9f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    m3ShapeDef sd = m3DefaultShapeDef();
    NewHeightFieldGridShape(world, StaticBody(world, p, q), &sd, heights, 4, 4, 1.0f);
    m3real r = 0.05f;
    // Over the flat first row on a line 0.15 apart, over the raised
    // sample's slopes, and off the far edge.
    for (int32_t i = 0; i < 13; ++i)
    {
        float x = 0.1f + 0.2f * (float)i;
        float h = 0.5f + (float)Ulps(0.05, i - 6);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){x, h, 0.3f}), r);
        AddProbe(world, probe,
                 WorldPoint(
                     p, q, (m3Vec3){3.0f + (float)Ulps(0.05, i - 6), 0.5f, 0.1f + 0.2f * (float)i}),
                 r);
    }
    for (int32_t i = 0; i < 5; ++i)
    {
        float x = 1.6f + 0.2f * (float)i;
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){x, 0.9f + 0.05f, 2.0f}), r);
    }
}

// The bounding sphere is tight only at a few points of each kind: a
// capsule's tips, a voxel chunk's corners, a height field's box corners.
// These scenes give each such point its own shape, spread out so no two
// probes touch, and put one probe on the sphere's radius through it at
// the bound plus the particle radius, moved by k float ulps of that
// distance. A cull that drops the radius or the margin skips a probe the
// projection moves.
enum
{
    TIGHT_PROBES = 13
};

static void CapsuleTipScene(m3World* world, Probe* probe)
{
    // Off the body's origin, so the projection rounds on the offset's
    // magnitude as a capsule placed in a body usually does.
    m3Capsule capsule = {{2.5f, -1.5f, 1.0f}, {3.5f, -1.5f, 1.0f}, 0.3f};
    m3ShapeDef sd = m3DefaultShapeDef();
    m3real r = 0.04f;
    for (int32_t i = 0; i < TIGHT_PROBES; ++i)
    {
        m3Pos3 p = {-20.0 + 6.0 * i, 6.0, -9.0};
        m3Quat q = Turn((m3Vec3){0.3f, 1.0f, 0.6f}, 0.4f + 0.37f * (float)i);
        NewCapsuleShape(world, StaticBody(world, p, q), &sd, &capsule);
        m3Pos3 center = WorldPoint(p, q, (m3Vec3){3.0f, -1.5f, 1.0f});
        m3Vec3 tip = m3RotateVec3(q, (m3Vec3){i % 2 == 0 ? 1.0f : -1.0f, 0.0f, 0.0f});
        AddProbe(world, probe, Along(center, tip, Ulps(0.5 + 0.3 + 0.04, i - 6)), r);
    }
}

static void VoxelCornerScene(m3World* world, Probe* probe)
{
    // The chunk's two corner cells filled: its far corner and its origin
    // are on the bounding sphere.
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    voxels[0] = 1;
    voxels[M3_VOXEL_COUNT - 1] = 1;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3real r = 0.06f;
    for (int32_t i = 0; i < TIGHT_PROBES; ++i)
    {
        m3Pos3 p = {-100.0 + 16.0 * i, -30.0, 20.0};
        m3Quat q = Turn((m3Vec3){1.0f, 0.4f, -0.2f}, 0.3f + 0.41f * (float)i);
        NewVoxelChunkShape(world, StaticBody(world, p, q), &sd, voxels, NULL, 0.5f);
        m3Pos3 center = WorldPoint(p, q, (m3Vec3){4.0f, 4.0f, 4.0f});
        m3Vec3 out = m3RotateVec3(q, i % 2 == 0 ? (m3Vec3){1.0f, 1.0f, 1.0f}
                                                : (m3Vec3){-1.0f, -1.0f, -1.0f});
        AddProbe(world, probe, Along(center, out, Ulps(4.0 * sqrt(3.0) + 0.06, i - 6)), r);
    }
}

static void HeightFieldCornerScene(m3World* world, Probe* probe)
{
    // Flat at 0 but for a peak of 3 at the far corner sample: the field's
    // box is [0, 3] on every axis, and the corner at the minimum height
    // and the peak at the maximum are both on the bounding sphere. Out
    // along the sphere's radius, each corner is the closest point.
    static const float heights[16] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f};
    m3ShapeDef sd = m3DefaultShapeDef();
    m3real r = 0.05f;
    for (int32_t i = 0; i < TIGHT_PROBES; ++i)
    {
        m3Pos3 p = {30.0 + 6.0 * i, -12.0, -40.0};
        m3Quat q = Turn((m3Vec3){0.0f, 1.0f, 0.0f}, 0.2f + 0.45f * (float)i);
        NewHeightFieldGridShape(world, StaticBody(world, p, q), &sd, heights, 4, 4, 1.0f);
        m3Pos3 center = WorldPoint(p, q, (m3Vec3){1.5f, 1.5f, 1.5f});
        m3Vec3 out = m3RotateVec3(q, i % 2 == 0 ? (m3Vec3){1.0f, 1.0f, 1.0f}
                                                : (m3Vec3){-1.0f, -1.0f, -1.0f});
        AddProbe(world, probe, Along(center, out, Ulps(1.5 * sqrt(3.0) + 0.05, i - 6)), r);
    }
}

// A box's projection threshold: its convex radius s plus the particle's.
static double BoxThreshold(const m3World* world, m3ShapeId box, m3real r)
{
    return (double)world->shapes.shapeGeom[box.index1 - 1].s + (double)r;
}

// Probes off a box's faces at s + r and a few ulps either side, at the
// face's middle and toward its edges and corners, each spot its own.
static void BoxProbes(m3World* world, Probe* probe, m3Pos3 p, m3Quat q, m3ShapeId box, m3Vec3 half,
                      m3real r)
{
    double t = BoxThreshold(world, box, r);
    for (int32_t i = 0; i < 13; ++i)
    {
        float a = -0.8f + 0.13f * (float)i;
        float b = 0.6f - 0.1f * (float)i;
        float out = (float)Ulps(t, i - 6);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){half.x + out, a * half.y, b * half.z}), r);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){b * half.x, -half.y - out, a * half.z}),
                 r);
        AddProbe(world, probe, WorldPoint(p, q, (m3Vec3){a * half.x, b * half.y, half.z + out}), r);
    }
}

static void BoxScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {5.0, 3.0, 5.0};
    m3Quat q = Turn((m3Vec3){1.0f, 1.0f, 0.0f}, 0.9f);
    m3Vec3 half = {1.0f, 0.6f, 0.8f};
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId box = NewBoxShape(world, StaticBody(world, p, q), &sd, half);
    BoxProbes(world, probe, p, q, box, half, 0.05f);
}

// The same box far from the origin, where a coordinate's float ulp is
// about a millimeter: the cull works relative to the body, so it must
// still keep every probe the projection touches.
static void FarBoxScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {8000.0, 50.0, -6000.0};
    m3Quat q = Turn((m3Vec3){0.0f, 1.0f, 2.0f}, 2.1f);
    m3Vec3 half = {1.0f, 0.6f, 0.8f};
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId box = NewBoxShape(world, StaticBody(world, p, q), &sd, half);
    BoxProbes(world, probe, p, q, box, half, 0.05f);
}

// A sharp tetrahedron: past its tip the face planes reach well beyond the
// hull's bounding sphere plus the radius, so a bounding-sphere cull would
// skip projections there. Probes fan out past the tip at growing reach.
static void SpikeScene(m3World* world, Probe* probe)
{
    m3Pos3 p = {-6.0, 6.0, 6.0};
    m3Quat q = Turn((m3Vec3){0.3f, 0.0f, 1.0f}, 0.6f);
    static const m3Vec3 spike[4] = {
        {0.0f, 0.0f, 0.0f}, {0.4f, 0.0f, 0.0f}, {0.0f, 0.4f, 0.0f}, {0.05f, 0.05f, 3.0f}};
    m3ShapeDef sd = m3DefaultShapeDef();
    NewHullShape(world, StaticBody(world, p, q), &sd, spike, 4);
    m3real r = 0.05f;
    for (int32_t i = 0; i < 24; ++i)
    {
        float reach = 0.02f + 0.03f * (float)i;
        float angle = 0.7f * (float)i;
        float tilt = 0.12f * (float)(i % 4);
        m3Vec3 local = {0.05f + tilt * cosf(angle) * reach, 0.05f + tilt * sinf(angle) * reach,
                        3.0f + reach};
        AddProbe(world, probe, WorldPoint(p, q, local), r);
    }
}

// A particle deep in sphere A is pushed out of it into sphere B, a later
// shape, which its box before the pass could not reach.
static void HandOffScene(m3World* world, Probe* probe)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId holder = StaticBody(world, (m3Pos3){0.0, 20.0, 0.0}, Turn((m3Vec3){0, 1, 0}, 0.0f));
    m3Sphere a = {{0.0f, 0.0f, 0.0f}, 1.0f};
    m3Sphere b = {{1.6f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, holder, &sd, &a);
    NewSphereShape(world, holder, &sd, &b);
    AddProbe(world, probe, (m3Pos3){0.2, 20.0, 0.0}, 0.1f);
    AddProbe(world, probe, (m3Pos3){0.3, 20.0, 0.05}, 0.1f);
}

// A particle deep in a sphere is pushed out of it into a box, a later
// shape, which its box before the pass could not reach.
static void HullHandOffScene(m3World* world, Probe* probe)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId holder = StaticBody(world, (m3Pos3){0.0, 30.0, 0.0}, Turn((m3Vec3){0, 1, 0}, 0.0f));
    m3Sphere a = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(world, holder, &sd, &a);
    m3BodyId boxHolder = StaticBody(world, (m3Pos3){1.6, 30.0, 0.0}, Turn((m3Vec3){0, 0, 1}, 0.3f));
    NewBoxShape(world, boxHolder, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    AddProbe(world, probe, (m3Pos3){0.2, 30.0, 0.0}, 0.1f);
    AddProbe(world, probe, (m3Pos3){0.3, 30.0, 0.05}, 0.1f);
}

// A kinematic body that moves and spins, carrying a sphere and a capsule
// through a hanging cloth, with gravity on.
static void MovingScene(m3World* world, Probe* probe)
{
    (void)probe;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.position = (m3Pos3){-1.5, 1.0, 0.0};
    bd.linearVelocity = (m3Vec3){0.8f, 0.0f, 0.1f};
    bd.angularVelocity = (m3Vec3){0.3f, 1.7f, 0.4f};
    m3BodyId mover = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.4f, 0.0f, 0.0f}, 0.3f};
    m3Capsule rod = {{-0.6f, 0.0f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.15f};
    NewSphereShape(world, mover, &sd, &ball);
    NewCapsuleShape(world, mover, &sd, &rod);
    m3BodyDef bd2 = bd;
    bd2.position = (m3Pos3){1.5, 1.0, 0.5};
    bd2.linearVelocity = (m3Vec3){-0.7f, 0.0f, 0.0f};
    bd2.angularVelocity = (m3Vec3){1.2f, 0.2f, -0.9f};
    NewBoxShape(world, NewBody(world, &bd2), &sd, (m3Vec3){0.35f, 0.2f, 0.3f});
    m3SoftBodyDef cloth = m3DefaultSoftBodyDef();
    cloth.position = (m3Pos3){-1.0, 1.4, -1.0};
    cloth.countX = 8;
    cloth.countY = 1;
    cloth.countZ = 8;
    cloth.spacing = 0.25f;
    cloth.radius = 0.05f;
    NewSoftBody(world, &cloth);
}

typedef struct Outcome
{
    uint8_t* bytes;
    int32_t size;
    int32_t moved;
    int32_t still;
} Outcome;

static Outcome RunScene(BuildScene* build, bool cullOff, bool gravity, int32_t steps)
{
    m3World* world = SceneWorld(cullOff);
    if (!gravity)
    {
        m3World_SetGravity(world, (m3Vec3){0.0f, 0.0f, 0.0f});
    }
    static Probe probe;
    memset(&probe, 0, sizeof(probe));
    build(world, &probe);
    for (int32_t s = 0; s < steps; ++s)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    Outcome out = {NULL, m3World_GetSnapshotSize(world), 0, 0};
    out.bytes = (uint8_t*)malloc((size_t)out.size);
    CHECK(Snapshot(world, out.bytes, out.size) == out.size, "the scene snapshots");
    for (int32_t i = 0; i < probe.count; ++i)
    {
        m3Pos3 now = m3SoftBody_GetParticlePosition(world, probe.ids[i], 0);
        bool same =
            now.x == probe.start[i].x && now.y == probe.start[i].y && now.z == probe.start[i].z;
        out.moved += same ? 0 : 1;
        out.still += same ? 1 : 0;
    }
    m3DestroyWorld(world);
    return out;
}

// The scene with the cull and without must end byte for byte the same.
// A threshold scene must straddle its threshold: some probes projected,
// some left where they were.
static void CheckScene(const char* name, BuildScene* build, bool threshold, bool gravity,
                       int32_t steps)
{
    Outcome culled = RunScene(build, false, gravity, steps);
    Outcome every = RunScene(build, true, gravity, steps);
    bool same =
        culled.size == every.size && memcmp(culled.bytes, every.bytes, (size_t)culled.size) == 0;
    if (!same)
    {
        printf("M3_SOFTCULL %s: the cull changed the world\n", name);
    }
    CHECK(same, "the cull leaves every scene byte for byte as testing every pair does");
    if (threshold)
    {
        printf("M3_SOFTCULL %s: %d probes projected, %d left alone\n", name, every.moved,
               every.still);
        CHECK(every.moved > 0 && every.still > 0, "the probes straddle the threshold");
    }
    free(culled.bytes);
    free(every.bytes);
}

int main(void)
{
    CheckScene("sphere", SphereScene, true, false, 1);
    CheckScene("capsule", CapsuleScene, true, false, 1);
    CheckScene("voxel", VoxelScene, true, false, 1);
    CheckScene("mesh", MeshScene, true, false, 1);
    CheckScene("height field", HeightFieldScene, true, false, 1);
    CheckScene("capsule tips", CapsuleTipScene, true, false, 1);
    CheckScene("voxel corners", VoxelCornerScene, true, false, 1);
    CheckScene("height field corners", HeightFieldCornerScene, true, false, 1);
    CheckScene("box", BoxScene, true, false, 1);
    CheckScene("far box", FarBoxScene, true, false, 1);
    CheckScene("spike", SpikeScene, true, false, 1);
    CheckScene("hand-off", HandOffScene, false, false, 1);
    CheckScene("hull hand-off", HullHandOffScene, false, false, 1);
    CheckScene("moving", MovingScene, false, true, 90);
    CheckScene("dense voxel", DenseVoxelScene, false, true, 8);
    CheckScene("dense mesh", DenseMeshScene, false, true, 8);
    if (s_failures == 0)
    {
        printf("test_softcull: all green\n");
        return 0;
    }
    printf("test_softcull: %d failure(s)\n", s_failures);
    return 1;
}
