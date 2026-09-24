// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Continuous collision. A dynamic body that moved further in a step than
// half its thinnest extent sweeps every shape against what it passed:
// static shapes always, and dynamic and kinematic ones too when it is a
// bullet, with both sweeps in the time of impact. The earliest impact
// pulls the body back to that pose; its velocity stays, and the next
// step's speculative contact turns it into an impulse. Bullets never
// sweep against bullets. Bodies go in ascending index order, so later
// bodies see earlier pull-backs.

#include "continuous.h"

#include "broad_phase.h"

#include "distance.h"
#include "manifold.h"
#include "shape.h"
#include "solver.h"
#include "world_internal.h"

#include <float.h>
#include <math.h>
#include <string.h>

typedef struct Sweeper
{
    m3World* world;
    const m3Pos3* com0;
    const m3Quat* rot0;
    int32_t body;
    int32_t shape;
    m3Pos3 base; // the fast body's start: the sweep frame's origin
    m3Sweep sweep;
    m3real fraction; // the earliest impact so far
} Sweeper;

// A body's sweep over the step, relative to base so the kernels see small
// floats.
static m3Sweep RelativeSweep(const m3World* world, int32_t body, const m3Pos3* com0,
                             const m3Quat* rot0, m3Pos3 base)
{
    m3Sweep sweep;
    sweep.localCenter = world->bodies.localCenters[body];
    sweep.c1 = (m3Vec3){(m3real)(com0[body].x - base.x), (m3real)(com0[body].y - base.y),
                        (m3real)(com0[body].z - base.z)};
    m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[body].q, world->bodies.localCenters[body]);
    sweep.c2 = (m3Vec3){(m3real)(world->bodies.transforms[body].p.x + (double)rlc.x - base.x),
                        (m3real)(world->bodies.transforms[body].p.y + (double)rlc.y - base.y),
                        (m3real)(world->bodies.transforms[body].p.z + (double)rlc.z - base.z)};
    sweep.q1 = rot0[body];
    sweep.q2 = world->bodies.transforms[body].q;
    return sweep;
}

static void TakeImpact(Sweeper* sw, const m3TOIInput* input)
{
    m3TOIOutput out = m3TimeOfImpact(input);
    if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < sw->fraction)
    {
        sw->fraction = out.fraction;
    }
}

// The box the fast body's center of mass sweeps, padded by its reach,
// in the frame of a target body (static meshes and voxel chunks keep
// their triangles in their own frame).
static void SweptBoxIn(const Sweeper* sw, int32_t target, m3Vec3* lo, m3Vec3* hi)
{
    const m3World* world = sw->world;
    const m3Transform* xfT = &world->bodies.transforms[target];
    const m3Transform* xfF = &world->bodies.transforms[sw->body];
    m3Pos3 start = sw->com0[sw->body];
    m3Vec3 rlc = m3RotateVec3(xfF->q, world->bodies.localCenters[sw->body]);
    m3Vec3 c1 =
        m3InvRotateVec3(xfT->q, (m3Vec3){(m3real)(start.x - xfT->p.x), (m3real)(start.y - xfT->p.y),
                                         (m3real)(start.z - xfT->p.z)});
    m3Vec3 c2 = m3InvRotateVec3(xfT->q, (m3Vec3){(m3real)(xfF->p.x + (double)rlc.x - xfT->p.x),
                                                 (m3real)(xfF->p.y + (double)rlc.y - xfT->p.y),
                                                 (m3real)(xfF->p.z + (double)rlc.z - xfT->p.z)});
    m3real pad = world->bodies.maxExtents[sw->body] + M3_AABB_MARGIN;
    *lo = (m3Vec3){m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
    *hi = (m3Vec3){m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};
}

// Sweeps against a voxel chunk's merged boxes, unextended: the seam
// extension is a contact device, and a bullet must not stop against
// phantom solid behind a thin welded wall. A sweep grazing a seam at
// worst stops a hair early and leaves the rest to the contacts.
static void SweepVoxels(Sweeper* sw, int32_t shape, int32_t body)
{
    m3World* world = sw->world;
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    const m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
    m3real cell = world->voxels.voxelData[slot].cellSize;
    m3Vec3 scratch[2];
    m3TOIInput input;
    input.proxyB = m3MakeShapeProxy(world, sw->shape, scratch);
    input.sweepA = RelativeSweep(world, body, sw->com0, sw->rot0, sw->base);
    input.sweepB = sw->sweep;
    m3Vec3 lo;
    m3Vec3 hi;
    SweptBoxIn(sw, body, &lo, &hi);
    uint16_t gather[M3_MESH_MAX_TRIS];
    int32_t count = m3MeshBvhGather(&surface->bvh, lo, hi, gather);
    for (int32_t g = 0; g < count && g < 64; ++g)
    {
        m3Vec3 blo;
        m3Vec3 bhi;
        m3VoxelBoxBounds(surface, cell, gather[g], &blo, &bhi);
        m3Vec3 corners[8];
        for (int32_t k = 0; k < 8; ++k)
        {
            corners[k] = (m3Vec3){(k & 1) != 0 ? bhi.x : blo.x, (k & 2) != 0 ? bhi.y : blo.y,
                                  (k & 4) != 0 ? bhi.z : blo.z};
        }
        input.proxyA = (m3DistanceProxy){corners, 8, 0.0f};
        input.maxFraction = sw->fraction;
        TakeImpact(sw, &input);
    }
}

// Sweeps against the mesh triangles the swept box overlaps, at most 64,
// in ascending triangle order.
static void SweepMesh(Sweeper* sw, int32_t shape, int32_t body)
{
    m3World* world = sw->world;
    const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
    m3Vec3 scratch[2];
    m3TOIInput input;
    input.proxyB = m3MakeShapeProxy(world, sw->shape, scratch);
    input.sweepA = RelativeSweep(world, body, sw->com0, sw->rot0, sw->base);
    input.sweepB = sw->sweep;
    m3Vec3 lo;
    m3Vec3 hi;
    SweptBoxIn(sw, body, &lo, &hi);
    uint16_t gather[M3_MESH_MAX_TRIS];
    int32_t count = m3MeshBvhGather(&world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape]], lo,
                                    hi, gather);
    int32_t budget = 64;
    for (int32_t g = 0; g < count && budget > 0; ++g)
    {
        int32_t t = gather[g];
        m3Vec3 tv[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                        mesh->vertices[mesh->indices[3 * t + 1]],
                        mesh->vertices[mesh->indices[3 * t + 2]]};
        m3Vec3 tlo = {m3MinF(tv[0].x, m3MinF(tv[1].x, tv[2].x)),
                      m3MinF(tv[0].y, m3MinF(tv[1].y, tv[2].y)),
                      m3MinF(tv[0].z, m3MinF(tv[1].z, tv[2].z))};
        m3Vec3 thi = {m3MaxF(tv[0].x, m3MaxF(tv[1].x, tv[2].x)),
                      m3MaxF(tv[0].y, m3MaxF(tv[1].y, tv[2].y)),
                      m3MaxF(tv[0].z, m3MaxF(tv[1].z, tv[2].z))};
        if (thi.x < lo.x || tlo.x > hi.x || thi.y < lo.y || tlo.y > hi.y || thi.z < lo.z ||
            tlo.z > hi.z)
        {
            continue;
        }
        budget -= 1;
        input.proxyA = (m3DistanceProxy){tv, 3, 0.0f};
        input.maxFraction = sw->fraction;
        TakeImpact(sw, &input);
    }
}

// Whether the fast shape sweeps against shape at all: never itself or
// its own body, disabled bodies, bullets or sensors; dynamic and
// kinematic bodies only for a bullet; and only pairs the collision
// filter lets meet.
static bool SweepsAgainst(const Sweeper* sw, int32_t shape)
{
    const m3World* world = sw->world;
    int32_t body = world->shapes.shapeBody[shape];
    uint8_t type = world->shapes.shapeType[shape];
    if (shape == sw->shape || body == sw->body || world->bodies.bodyEnabled[body] == 0 ||
        world->bodies.bulletFlags[body] != 0 || world->shapes.shapeSensor[shape] != 0)
    {
        return false;
    }
    if (type == (uint8_t)m3_voxelShape || type == (uint8_t)m3_meshShape)
    {
        return true; // static surfaces, filtered by their contacts
    }
    if (world->bodies.types[body] != (uint8_t)m3_staticBody &&
        world->bodies.bulletFlags[sw->body] == 0)
    {
        return false;
    }
    int32_t groupA = world->shapes.shapeGroup[shape];
    int32_t groupB = world->shapes.shapeGroup[sw->shape];
    if (groupA != 0 && groupA == groupB)
    {
        return groupA > 0;
    }
    return m3FilterPass(world->shapes.shapeCategory[shape], world->shapes.shapeMask[shape],
                        world->shapes.shapeCategory[sw->shape], world->shapes.shapeMask[sw->shape]);
}

static bool SweepCallback(int32_t shape, void* context)
{
    Sweeper* sw = (Sweeper*)context;
    m3World* world = sw->world;
    if (!SweepsAgainst(sw, shape))
    {
        return true;
    }
    int32_t body = world->shapes.shapeBody[shape];
    uint8_t type = world->shapes.shapeType[shape];
    if (type == (uint8_t)m3_voxelShape)
    {
        SweepVoxels(sw, shape, body);
        return true;
    }
    if (type == (uint8_t)m3_meshShape)
    {
        SweepMesh(sw, shape, body);
        return true;
    }
    m3Vec3 scratchA[2];
    m3Vec3 scratchB[2];
    m3TOIInput input;
    input.proxyA = m3MakeShapeProxy(world, shape, scratchA);
    input.proxyB = m3MakeShapeProxy(world, sw->shape, scratchB);
    input.sweepA = RelativeSweep(world, body, sw->com0, sw->rot0, sw->base);
    input.sweepB = sw->sweep;
    input.maxFraction = sw->fraction;
    TakeImpact(sw, &input);
    return true;
}

// Planes stay out of the tree: conservative advancement against the
// plane's analytic distance, the fast proxy's lowest point minus its
// radius target. A body already within the target at the start belongs
// to the discrete speculative contact; pulling it back to its start
// every step would erase its integration while its velocity stayed.
static void SweepPlane(const m3World* world, Sweeper* sw, int32_t planeShape)
{
    m3Vec3 n = world->shapes.shapeGeom[planeShape].v;
    m3real offset =
        world->shapes.shapeGeom[planeShape].s -
        (m3real)((double)n.x * sw->base.x + (double)n.y * sw->base.y + (double)n.z * sw->base.z);
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, sw->shape, scratch);
    const m3real linearSlop = 0.005f;
    m3real target = m3MaxF(linearSlop, proxy.radius - linearSlop);
    m3real tolerance = 0.25f * linearSlop;
    // The gap closes no faster than the travel plus the fastest turn of
    // the farthest point.
    m3real rate = m3Length3(m3Sub3(sw->sweep.c2, sw->sweep.c1)) +
                  m3SweepAngularRateBound(&sw->sweep) * world->bodies.maxExtents[sw->body];
    if (!(rate > 0.0f))
    {
        return;
    }
    m3real t = 0.0f;
    for (int32_t iteration = 0; iteration < 25; ++iteration)
    {
        m3Transform xf = m3GetSweepTransform(&sw->sweep, t);
        m3real lowest = FLT_MAX;
        for (int32_t k = 0; k < proxy.count; ++k)
        {
            m3Vec3 r = m3RotateVec3(xf.q, proxy.points[k]);
            lowest = m3MinF(lowest, n.x * ((m3real)xf.p.x + r.x) + n.y * ((m3real)xf.p.y + r.y) +
                                        n.z * ((m3real)xf.p.z + r.z));
        }
        m3real gap = lowest - offset;
        if (gap <= 0.0f)
        {
            return; // started behind or overlapped: the contacts own it
        }
        if (gap <= target + tolerance)
        {
            if (t > 0.0f && t < sw->fraction)
            {
                sw->fraction = t;
            }
            return;
        }
        t += (gap - target) / rate;
        if (t >= sw->fraction)
        {
            return;
        }
    }
}

// Whether a body moved further than half its thinnest extent: the travel
// of its center of mass plus the arc its farthest point can sweep.
static bool MovedFast(const m3World* world, int32_t body, const m3Sweep* sweep)
{
    m3real motion = m3Length3(m3Sub3(sweep->c2, sweep->c1)) +
                    m3SweepAngularRateBound(sweep) * world->bodies.maxExtents[body];
    return motion > 0.5f * world->bodies.minExtents[body];
}

// Sweeps one shape of the fast body against the tree and the planes.
static void SweepShape(Sweeper* sw, int32_t shape)
{
    m3World* world = sw->world;
    sw->shape = shape;
    double pad = (double)(world->bodies.maxExtents[sw->body] + M3_AABB_MARGIN);
    m3Pos3 a = sw->com0[sw->body];
    m3Pos3 b = {sw->base.x + (double)sw->sweep.c2.x, sw->base.y + (double)sw->sweep.c2.y,
                sw->base.z + (double)sw->sweep.c2.z};
    double lo[3] = {fmin(a.x, b.x) - pad, fmin(a.y, b.y) - pad, fmin(a.z, b.z) - pad};
    double hi[3] = {fmax(a.x, b.x) + pad, fmax(a.y, b.y) + pad, fmax(a.z, b.z) + pad};
    // Only a bullet sweeps against moving bodies; everything else meets
    // statics and static surfaces, and skips subtrees holding neither.
    uint32_t targets = world->bodies.bulletFlags[sw->body] != 0
                           ? 0xFFFFFFFFu
                           : (M3_PROXY_STATIC | M3_PROXY_SURFACE);
    m3TreeQueryMask(&world->broadphase.tree, lo, hi, targets, SweepCallback, sw);
    for (int32_t k = 0; k < world->shapes.planeCount; ++k)
    {
        SweepPlane(world, sw, world->shapes.planeShapes[k]);
    }
}

void m3SolveContinuousPhase(m3World* world, const m3Pos3* com0, const m3Quat* rot0)
{
    for (int32_t i = 0; i < world->bodies.bodyPool.maxIndex; ++i)
    {
        if (world->bodies.bodyPool.alive[i] == 0 ||
            world->bodies.types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        Sweeper sw;
        sw.world = world;
        sw.com0 = com0;
        sw.rot0 = rot0;
        sw.body = i;
        sw.base = com0[i];
        sw.fraction = 1.0f;
        sw.sweep = RelativeSweep(world, i, com0, rot0, sw.base);
        if (!MovedFast(world, i, &sw.sweep))
        {
            continue;
        }
        for (int32_t s = world->bodies.bodyShapeHead[i]; s != -1; s = world->shapes.shapeNext[s])
        {
            if (world->shapes.shapeSensor[s] == 0) // a fast sensor blocks nothing
            {
                SweepShape(&sw, s);
            }
        }
        if (sw.fraction < 1.0f)
        {
            m3Transform xf = m3GetSweepTransform(&sw.sweep, sw.fraction);
            world->bodies.transforms[i].q = xf.q;
            world->bodies.transforms[i].p.x = sw.base.x + xf.p.x;
            world->bodies.transforms[i].p.y = sw.base.y + xf.p.y;
            world->bodies.transforms[i].p.z = sw.base.z + xf.p.z;
        }
    }
}
