// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Continuous collision: fast bodies sweep and pull back to the time of
// impact.

#include "continuous.h"

#include "distance.h"
#include "manifold.h"
#include "shape.h"
#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

// ---------------------------------------------------------------
// Continuous collision, modeled on the reference
// b3SolveContinuous: any fast dynamic body sweeps against statics;
// a bullet additionally sweeps against non-bullet dynamics with the
// TARGET'S true sweep in the TOI (dynamic versus dynamic moves both
// bodies). A hit pulls the fast body back to the impact pose;
// velocity stays, and next step's speculative contact resolves the
// impact. Bullet versus bullet is not resolved (the reference
// limitation, kept and documented). Bodies are processed in
// ascending index order; later bodies see earlier pull-backs, all
// deterministic.
// ---------------------------------------------------------------

typedef struct m3ContinuousContext
{
    m3World* world;
    const m3Pos3* com0;
    const m3Quat* rot0;
    int32_t fastBody;
    int32_t fastShape;
    m3Pos3 base; // re-center: TOI floats stay small
    m3Sweep fastSweep;
    m3real fraction;
} m3ContinuousContext;

static m3Sweep MakeRelativeSweep(const m3World* world, int32_t body, const m3Pos3* com0,
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

static bool ContinuousQueryCallback(int32_t shape, void* userContext)
{
    m3ContinuousContext* ctx = (m3ContinuousContext*)userContext;
    m3World* world = ctx->world;
    if (shape == ctx->fastShape)
    {
        return true;
    }
    int32_t body = world->shapes.shapeBody[shape];
    if (body == ctx->fastBody)
    {
        return true;
    }
    if (world->bodies.bodyEnabled[body] == 0)
    {
        return true; // disabled bodies never block the fast mover
    }
    if (world->bodies.bulletFlags[body] != 0)
    {
        return true; // bullet versus bullet: skip (documented)
    }
    if (world->shapes.shapeSensor[shape] != 0)
    {
        return true; // sensors never stop anything
    }
    if (world->shapes.shapeType[shape] == (uint8_t)m3_voxelShape)
    {
        // Voxel TOI: sweep the fast shape against candidate
        // merged boxes in the CHUNK frame (the correct-frame
        // witnesses the architecture demands). Boxes are UNEXTENDED
        // here on purpose: the seam extension is a contact-only
        // device (coverage guarantees exactly one flush layer, and
        // a bullet must not stop against phantom solid a chunk
        // length behind a thin welded wall). A seam-grazing sweep
        // at worst stops a hair early and hands the rest of the
        // step to the welded contact solver.
        int32_t slot = world->shapes.shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
        m3real cell = world->voxels.voxelData[slot].cellSize;
        m3Sweep chunkSweep = MakeRelativeSweep(world, body, ctx->com0, ctx->rot0, ctx->base);
        m3Vec3 scratchFast[2];
        m3DistanceProxy fastProxy = m3MakeShapeProxy(world, ctx->fastShape, scratchFast);

        const m3Transform* xfV = &world->bodies.transforms[body];
        m3Vec3 c1 =
            m3InvRotateVec3(xfV->q, (m3Vec3){(m3real)(ctx->com0[ctx->fastBody].x - xfV->p.x),
                                             (m3real)(ctx->com0[ctx->fastBody].y - xfV->p.y),
                                             (m3real)(ctx->com0[ctx->fastBody].z - xfV->p.z)});
        m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[ctx->fastBody].q,
                                  world->bodies.localCenters[ctx->fastBody]);
        m3Vec3 c2 = m3InvRotateVec3(
            xfV->q,
            (m3Vec3){
                (m3real)(world->bodies.transforms[ctx->fastBody].p.x + (double)rlc.x - xfV->p.x),
                (m3real)(world->bodies.transforms[ctx->fastBody].p.y + (double)rlc.y - xfV->p.y),
                (m3real)(world->bodies.transforms[ctx->fastBody].p.z + (double)rlc.z - xfV->p.z)});
        m3real pad = world->bodies.maxExtents[ctx->fastBody] + M3_AABB_MARGIN;
        m3Vec3 lo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 hi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(&surface->bvh, lo, hi, gather);
        int32_t budget = 64;
        for (int32_t g = 0; g < gatherCount && budget > 0; ++g)
        {
            budget -= 1;
            m3Vec3 blo;
            m3Vec3 bhi;
            m3VoxelBoxBounds(surface, cell, gather[g], &blo, &bhi);
            m3Vec3 corners[8];
            for (int32_t k = 0; k < 8; ++k)
            {
                corners[k] = (m3Vec3){(k & 1) != 0 ? bhi.x : blo.x, (k & 2) != 0 ? bhi.y : blo.y,
                                      (k & 4) != 0 ? bhi.z : blo.z};
            }
            m3TOIInput input;
            input.proxyA.points = corners;
            input.proxyA.count = 8;
            input.proxyA.radius = 0.0f;
            input.proxyB = fastProxy;
            input.sweepA = chunkSweep;
            input.sweepB = ctx->fastSweep;
            input.maxFraction = ctx->fraction;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < ctx->fraction)
            {
                ctx->fraction = out.fraction;
            }
        }
        return true;
    }
    if (world->shapes.shapeType[shape] == (uint8_t)m3_meshShape)
    {
        // Mesh TOI: sweep the fast shape against every
        // candidate triangle. Each triangle is a three-point static
        // proxy in the mesh body's frame; the shared kernel does the
        // rest. Ascending triangle order, bounded candidates.
        const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
        m3Sweep meshSweep = MakeRelativeSweep(world, body, ctx->com0, ctx->rot0, ctx->base);
        m3Vec3 scratchFast[2];
        m3DistanceProxy fastProxy = m3MakeShapeProxy(world, ctx->fastShape, scratchFast);

        // The swept bounds of the fast body in mesh-local space, a
        // conservative box from the relative sweep endpoints.
        const m3Transform* xfM = &world->bodies.transforms[body];
        m3Vec3 c1 =
            m3InvRotateVec3(xfM->q, (m3Vec3){(m3real)(ctx->com0[ctx->fastBody].x - xfM->p.x),
                                             (m3real)(ctx->com0[ctx->fastBody].y - xfM->p.y),
                                             (m3real)(ctx->com0[ctx->fastBody].z - xfM->p.z)});
        m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[ctx->fastBody].q,
                                  world->bodies.localCenters[ctx->fastBody]);
        m3Vec3 c2 = m3InvRotateVec3(
            xfM->q,
            (m3Vec3){
                (m3real)(world->bodies.transforms[ctx->fastBody].p.x + (double)rlc.x - xfM->p.x),
                (m3real)(world->bodies.transforms[ctx->fastBody].p.y + (double)rlc.y - xfM->p.y),
                (m3real)(world->bodies.transforms[ctx->fastBody].p.z + (double)rlc.z - xfM->p.z)});
        m3real pad = world->bodies.maxExtents[ctx->fastBody] + M3_AABB_MARGIN;
        m3Vec3 lo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 hi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(
            &world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape]], lo, hi, gather);
        int32_t budget = 64;
        for (int32_t g = 0; g < gatherCount && budget > 0; ++g)
        {
            int32_t t = gather[g];
            m3Vec3 tv[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                            mesh->vertices[mesh->indices[3 * t + 1]],
                            mesh->vertices[mesh->indices[3 * t + 2]]};
            m3real tlx = m3MinF(tv[0].x, m3MinF(tv[1].x, tv[2].x));
            m3real thx = m3MaxF(tv[0].x, m3MaxF(tv[1].x, tv[2].x));
            m3real tly = m3MinF(tv[0].y, m3MinF(tv[1].y, tv[2].y));
            m3real thy = m3MaxF(tv[0].y, m3MaxF(tv[1].y, tv[2].y));
            m3real tlz = m3MinF(tv[0].z, m3MinF(tv[1].z, tv[2].z));
            m3real thz = m3MaxF(tv[0].z, m3MaxF(tv[1].z, tv[2].z));
            if (thx < lo.x || tlx > hi.x || thy < lo.y || tly > hi.y || thz < lo.z || tlz > hi.z)
            {
                continue;
            }
            budget -= 1;
            m3TOIInput input;
            input.proxyA.points = tv;
            input.proxyA.count = 3;
            input.proxyA.radius = 0.0f;
            input.proxyB = fastProxy;
            input.sweepA = meshSweep;
            input.sweepB = ctx->fastSweep;
            input.maxFraction = ctx->fraction;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < ctx->fraction)
            {
                ctx->fraction = out.fraction;
            }
        }
        return true;
    }
    if (world->bodies.types[body] != (uint8_t)m3_staticBody &&
        world->bodies.bulletFlags[ctx->fastBody] == 0)
    {
        return true; // only bullets sweep against dynamics and kinematics
    }
    {
        // Filters: the continuous phase obeys the same rule
        // as the discrete pair scan.
        int32_t gi = world->shapes.shapeGroup[shape];
        int32_t gj = world->shapes.shapeGroup[ctx->fastShape];
        if (gi != 0 && gi == gj)
        {
            if (gi < 0)
            {
                return true;
            }
        }
        else if (!m3FilterPass(world->shapes.shapeCategory[shape], world->shapes.shapeMask[shape],
                               world->shapes.shapeCategory[ctx->fastShape],
                               world->shapes.shapeMask[ctx->fastShape]))
        {
            return true;
        }
    }

    m3TOIInput input;
    m3Vec3 scratchA[2];
    m3Vec3 scratchB[2];
    input.proxyA = m3MakeShapeProxy(world, shape, scratchA);
    input.proxyB = m3MakeShapeProxy(world, ctx->fastShape, scratchB);
    input.sweepA = MakeRelativeSweep(world, body, ctx->com0, ctx->rot0, ctx->base);
    input.sweepB = ctx->fastSweep;
    input.maxFraction = ctx->fraction;
    m3TOIOutput out = m3TimeOfImpact(&input);
    if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < ctx->fraction)
    {
        ctx->fraction = out.fraction;
    }
    return true;
}

// Plane targets are not in the tree: a bounded conservative advance
// against the analytic plane distance (support of the fast proxy at
// the swept pose, minus its radius).
static void ContinuousVersusPlane(const m3World* world, m3ContinuousContext* ctx,
                                  int32_t planeShape)
{
    m3Vec3 n = world->shapes.shapeGeom[planeShape].v;
    m3real offset =
        world->shapes.shapeGeom[planeShape].s -
        (m3real)((double)n.x * ctx->base.x + (double)n.y * ctx->base.y + (double)n.z * ctx->base.z);
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, ctx->fastShape, scratch);

    const m3real linearSlop = 0.005f;
    m3real target = m3MaxF(linearSlop, proxy.radius - linearSlop);
    m3real tolerance = 0.25f * linearSlop;

    // Conservative rate: linear travel plus the rotation arc.
    m3Vec3 travel = m3Sub3(ctx->fastSweep.c2, ctx->fastSweep.c1);
    m3Quat dq = m3MulQuat(ctx->fastSweep.q2, (m3Quat){-ctx->fastSweep.q1.x, -ctx->fastSweep.q1.y,
                                                      -ctx->fastSweep.q1.z, ctx->fastSweep.q1.w});
    m3real arc = 2.0f * sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) *
                 world->bodies.maxExtents[ctx->fastBody];
    m3real rate = sqrtf(m3Dot3(travel, travel)) + arc;
    if (!(rate > 0.0f))
    {
        return;
    }

    m3real t = 0.0f;
    for (int32_t iter = 0; iter < 25; ++iter)
    {
        m3Transform xf = m3GetSweepTransform(&ctx->fastSweep, t);
        m3real minD = 3.4e38f;
        for (int32_t k = 0; k < proxy.count; ++k)
        {
            m3Vec3 r = m3RotateVec3(xf.q, proxy.points[k]);
            m3real d = n.x * ((m3real)xf.p.x + r.x) + n.y * ((m3real)xf.p.y + r.y) +
                       n.z * ((m3real)xf.p.z + r.z);
            minD = m3MinF(minD, d);
        }
        m3real sep = minD - offset;
        if (sep <= 0.0f)
        {
            return; // started behind or overlapped: discrete owns it
        }
        if (sep <= target + tolerance)
        {
            // The reference rule the other three arms already obey:
            // a body ALREADY within the target distance at t = 0
            // belongs to the discrete speculative contact, not to
            // the continuous pull-back. Without this guard the
            // pull-back at fraction zero
            // erased each step's integration while velocity stayed,
            // freezing bodies at the slop gap where the refunded
            // accumulator kept their friction at zero: eternal
            // frictionless skaters under every pile, sleep never
            // engaging, the aftermath phase priced like the storm.
            if (t > 0.0f && t < ctx->fraction)
            {
                ctx->fraction = t;
            }
            return;
        }
        t += (sep - target) / rate;
        if (t >= ctx->fraction)
        {
            return; // no earlier hit than the current best
        }
    }
}

void m3SolveContinuousPhase(m3World* world, const m3Pos3* com0, const m3Quat* rot0)
{
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    int32_t maxShape = world->shapes.shapePool.maxIndex;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodies.bodyPool.alive[i] == 0 ||
            world->bodies.types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        // Fast test: displacement plus rotation arc versus the
        // thinnest extent (the reference safety factor of one half).
        m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[i].q, world->bodies.localCenters[i]);
        double cx = world->bodies.transforms[i].p.x + (double)rlc.x;
        double cy = world->bodies.transforms[i].p.y + (double)rlc.y;
        double cz = world->bodies.transforms[i].p.z + (double)rlc.z;
        m3Vec3 dc = {(m3real)(cx - com0[i].x), (m3real)(cy - com0[i].y), (m3real)(cz - com0[i].z)};
        m3Quat q0 = rot0[i];
        m3Quat dq = m3MulQuat(world->bodies.transforms[i].q, (m3Quat){-q0.x, -q0.y, -q0.z, q0.w});
        m3real arc =
            2.0f * sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) * world->bodies.maxExtents[i];
        m3real maxMotion = sqrtf(m3Dot3(dc, dc)) + arc;
        if (!(maxMotion > 0.5f * world->bodies.minExtents[i]))
        {
            continue;
        }

        m3ContinuousContext ctx;
        ctx.world = world;
        ctx.com0 = com0;
        ctx.rot0 = rot0;
        ctx.fastBody = i;
        ctx.base = com0[i];
        ctx.fraction = 1.0f;
        ctx.fastSweep = MakeRelativeSweep(world, i, com0, rot0, ctx.base);

        for (int32_t s = world->bodies.bodyShapeHead[i]; s != -1; s = world->shapes.shapeNext[s])
        {
            if (world->shapes.shapeSensor[s] != 0)
            {
                continue; // a sensor on a fast body blocks nothing
            }
            ctx.fastShape = s;
            // Swept candidate box: both COM endpoints padded by the
            // body's max extent (a coarse superset; the TOI filters).
            double pad = (double)(world->bodies.maxExtents[i] + M3_AABB_MARGIN);
            double lo[3];
            double hi[3];
            lo[0] = (com0[i].x < cx ? com0[i].x : cx) - pad;
            lo[1] = (com0[i].y < cy ? com0[i].y : cy) - pad;
            lo[2] = (com0[i].z < cz ? com0[i].z : cz) - pad;
            hi[0] = (com0[i].x > cx ? com0[i].x : cx) + pad;
            hi[1] = (com0[i].y > cy ? com0[i].y : cy) + pad;
            hi[2] = (com0[i].z > cz ? com0[i].z : cz) + pad;
            m3TreeQuery(&world->broadphase.tree, lo, hi, ContinuousQueryCallback, &ctx);

            // Planes take the dedicated pass (never in the tree).
            for (int32_t p = 0; p < maxShape; ++p)
            {
                if (world->shapes.shapePool.alive[p] != 0 &&
                    world->shapes.shapeType[p] == (uint8_t)m3_planeShape)
                {
                    ContinuousVersusPlane(world, &ctx, p);
                }
            }
        }

        if (ctx.fraction < 1.0f)
        {
            // Pull the body back to the impact pose. Velocity stays:
            // next step's speculative contact turns it into impulse.
            m3Transform xf = m3GetSweepTransform(&ctx.fastSweep, ctx.fraction);
            world->bodies.transforms[i].q = xf.q;
            world->bodies.transforms[i].p.x = ctx.base.x + xf.p.x;
            world->bodies.transforms[i].p.y = ctx.base.y + xf.p.y;
            world->bodies.transforms[i].p.z = ctx.base.z + xf.p.z;
        }
    }
}
