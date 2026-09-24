// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape casts: spheres, capsules, boxes and hulls swept through the
// world with the shared time-of-impact kernel, closest hit first.

#include "body.h"
#include "distance.h"
#include "journal.h"
#include "manifold.h"
#include "query.h"
#include "raycast.h"
#include "shape.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <float.h>
#include <string.h>

typedef struct m3ShapeCastContext
{
    m3World* world;
    int32_t ignoreBody;                   // -1 none: the character excludes itself
    m3Pos3 base;                          // the cast start: TOI floats re-center here
    m3Vec3 castPoints[M3_HULL_MAX_VERTS]; // sphere 1, capsule 2,
                                          // box 8, hull up to 24
    int32_t castPointCount;
    m3real castRadius;
    m3Vec3 translation;
    m3RayCastResult best;
    int32_t bestShape;
    m3QueryFilter filter;
} m3ShapeCastContext;

static void ShapeCastTestShape(m3ShapeCastContext* ctx, int32_t shape)
{
    m3World* world = ctx->world;
    int32_t body = world->shapes.shapeBody[shape];
    if (body == ctx->ignoreBody)
    {
        return; // the caster's own body never blocks its cast
    }
    if (world->bodies.bodyEnabled[body] == 0)
    {
        return; // disabled bodies vanish from queries
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits,
                      ctx->world->shapes.shapeCategory[shape], ctx->world->shapes.shapeMask[shape]))
    {
        return; // filtered out
    }
    if (world->shapes.shapeType[shape] == (uint8_t)m3_voxelShape)
    {
        // Voxel targets: per-box TOI against the merged
        // surface, unextended (queries report true geometry; the
        // seam extension is a contact-only device).
        int32_t slot = world->shapes.shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
        m3real cell = world->voxels.voxelData[slot].cellSize;
        m3Transform xfVv = m3ShapeWorldTransform(world, shape);
        const m3Transform* xfV = &xfVv;
        m3Sweep chunkSweep;
        chunkSweep.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
        chunkSweep.c1 = (m3Vec3){(m3real)(xfV->p.x - ctx->base.x), (m3real)(xfV->p.y - ctx->base.y),
                                 (m3real)(xfV->p.z - ctx->base.z)};
        chunkSweep.c2 = chunkSweep.c1;
        chunkSweep.q1 = xfV->q;
        chunkSweep.q2 = xfV->q;

        m3Sweep castSweep;
        castSweep.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
        castSweep.c1 = (m3Vec3){0.0f, 0.0f, 0.0f};
        castSweep.c2 = ctx->translation;
        castSweep.q1 = m3MakeIdentityQuat();
        castSweep.q2 = m3MakeIdentityQuat();

        m3Vec3 c1 = m3InvRotateVec3(xfV->q, m3Neg3(chunkSweep.c1));
        m3Vec3 c2 = m3InvRotateVec3(xfV->q, m3Sub3(ctx->translation, chunkSweep.c1));
        m3real pad = ctx->castRadius + 0.6f + M3_AABB_MARGIN;
        m3Vec3 blo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 bhi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(&surface->bvh, blo, bhi, gather);
        int32_t budget = 64;
        for (int32_t g = 0; g < gatherCount && budget > 0; ++g)
        {
            budget -= 1;
            m3Vec3 lo;
            m3Vec3 hi;
            m3VoxelBoxBounds(surface, cell, gather[g], &lo, &hi);
            m3Vec3 corners[8];
            for (int32_t k = 0; k < 8; ++k)
            {
                corners[k] = (m3Vec3){(k & 1) != 0 ? hi.x : lo.x, (k & 2) != 0 ? hi.y : lo.y,
                                      (k & 4) != 0 ? hi.z : lo.z};
            }
            m3TOIInput input;
            input.proxyA.points = corners;
            input.proxyA.count = 8;
            input.proxyA.radius = 0.0f;
            input.proxyB.points = ctx->castPoints;
            input.proxyB.count = ctx->castPointCount;
            input.proxyB.radius = ctx->castRadius;
            input.sweepA = chunkSweep;
            input.sweepB = castSweep;
            input.maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit &&
                (!ctx->best.hit || out.fraction < ctx->best.fraction ||
                 (out.fraction == ctx->best.fraction && shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = out.fraction;
                ctx->best.normal = out.normal;
                ctx->best.shapeId = (m3ShapeId){shape + 1, world->idWorld,
                                                world->shapes.shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
            else if (out.state == m3_toiStateOverlapped &&
                     (!ctx->best.hit || 0.0f < ctx->best.fraction ||
                      (0.0f == ctx->best.fraction && shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = 0.0f; // the start-overlapped contract
                ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
                ctx->best.shapeId = (m3ShapeId){shape + 1, world->idWorld,
                                                world->shapes.shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
        }
        return;
    }
    if (world->shapes.shapeType[shape] == (uint8_t)m3_meshShape)
    {
        // Mesh targets: per-triangle TOI, ascending, bounded (the
        // same recipe as continuous collision).
        const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
        m3Transform xfMv = m3ShapeWorldTransform(world, shape);
        const m3Transform* xfM = &xfMv;
        m3Sweep meshSweep;
        meshSweep.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
        meshSweep.c1 = (m3Vec3){(m3real)(xfM->p.x - ctx->base.x), (m3real)(xfM->p.y - ctx->base.y),
                                (m3real)(xfM->p.z - ctx->base.z)};
        meshSweep.c2 = meshSweep.c1;
        meshSweep.q1 = xfM->q;
        meshSweep.q2 = xfM->q;

        m3Sweep castSweep;
        castSweep.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
        castSweep.c1 = (m3Vec3){0.0f, 0.0f, 0.0f};
        castSweep.c2 = ctx->translation;
        castSweep.q1 = m3MakeIdentityQuat();
        castSweep.q2 = m3MakeIdentityQuat();

        // Candidate box in mesh-local space.
        m3Vec3 c1 = m3InvRotateVec3(xfM->q, m3Neg3(meshSweep.c1));
        m3Vec3 c2 = m3InvRotateVec3(xfM->q, m3Sub3(ctx->translation, meshSweep.c1));
        m3real pad = ctx->castRadius + 0.6f + M3_AABB_MARGIN; // capsule half-reach bound
        m3Vec3 blo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 bhi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(
            &world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape]], blo, bhi, gather);
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
            if (thx < blo.x || tlx > bhi.x || thy < blo.y || tly > bhi.y || thz < blo.z ||
                tlz > bhi.z)
            {
                continue;
            }
            budget -= 1;
            m3TOIInput input;
            input.proxyA.points = tv;
            input.proxyA.count = 3;
            input.proxyA.radius = 0.0f;
            input.proxyB.points = ctx->castPoints;
            input.proxyB.count = ctx->castPointCount;
            input.proxyB.radius = ctx->castRadius;
            input.sweepA = meshSweep;
            input.sweepB = castSweep;
            input.maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit &&
                (!ctx->best.hit || out.fraction < ctx->best.fraction ||
                 (out.fraction == ctx->best.fraction && shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = out.fraction;
                ctx->best.normal = out.normal;
                ctx->best.shapeId = (m3ShapeId){shape + 1, world->idWorld,
                                                world->shapes.shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
            else if (out.state == m3_toiStateOverlapped &&
                     (!ctx->best.hit || 0.0f < ctx->best.fraction ||
                      (0.0f == ctx->best.fraction && shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = 0.0f;
                ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
                ctx->best.shapeId = (m3ShapeId){shape + 1, world->idWorld,
                                                world->shapes.shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
        }
        return;
    }

    // Convex targets: one TOI against the shape's proxy at rest.
    m3Vec3 scratch[2];
    m3TOIInput input;
    input.proxyA = m3MakeShapeProxy(world, shape, scratch);
    input.proxyB.points = ctx->castPoints;
    input.proxyB.count = ctx->castPointCount;
    input.proxyB.radius = ctx->castRadius;

    const m3Transform* xf = &world->bodies.transforms[body];
    m3Vec3 rlc = m3RotateVec3(xf->q, world->bodies.localCenters[body]);
    input.sweepA.localCenter = world->bodies.localCenters[body];
    input.sweepA.c1 = (m3Vec3){(m3real)(xf->p.x + (double)rlc.x - ctx->base.x),
                               (m3real)(xf->p.y + (double)rlc.y - ctx->base.y),
                               (m3real)(xf->p.z + (double)rlc.z - ctx->base.z)};
    input.sweepA.c2 = input.sweepA.c1;
    input.sweepA.q1 = xf->q;
    input.sweepA.q2 = xf->q;

    input.sweepB.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.sweepB.c1 = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.sweepB.c2 = ctx->translation;
    input.sweepB.q1 = m3MakeIdentityQuat();
    input.sweepB.q2 = m3MakeIdentityQuat();
    input.maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;

    m3TOIOutput out = m3TimeOfImpact(&input);
    if (out.state == m3_toiStateHit &&
        (!ctx->best.hit || out.fraction < ctx->best.fraction ||
         (out.fraction == ctx->best.fraction && shape < ctx->bestShape)))
    {
        ctx->best.hit = true;
        ctx->best.fraction = out.fraction;
        ctx->best.normal = out.normal;
        ctx->best.shapeId =
            (m3ShapeId){shape + 1, world->idWorld, world->shapes.shapePool.generations[shape]};
        ctx->bestShape = shape;
    }
    else if (out.state == m3_toiStateOverlapped)
    {
        if (!ctx->best.hit || 0.0f < ctx->best.fraction ||
            (0.0f == ctx->best.fraction && shape < ctx->bestShape))
        {
            ctx->best.hit = true;
            ctx->best.fraction = 0.0f; // the start-overlapped contract
            ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
            ctx->best.shapeId =
                (m3ShapeId){shape + 1, world->idWorld, world->shapes.shapePool.generations[shape]};
            ctx->bestShape = shape;
        }
    }
}

// A plane target for a convex cast: conservative advance along the
// analytic support distance (the CCD plane recipe).
static void ShapeCastTestPlane(m3ShapeCastContext* ctx, int32_t shape)
{
    m3World* world = ctx->world;
    if (world->shapes.shapeBody[shape] == ctx->ignoreBody)
    {
        return;
    }
    if (world->bodies.bodyEnabled[world->shapes.shapeBody[shape]] == 0)
    {
        return; // disabled bodies vanish from queries
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits,
                      world->shapes.shapeCategory[shape], world->shapes.shapeMask[shape]))
    {
        return; // filtered out
    }
    m3Vec3 n = world->shapes.shapeGeom[shape].v;
    m3real offset =
        world->shapes.shapeGeom[shape].s -
        (m3real)((double)n.x * ctx->base.x + (double)n.y * ctx->base.y + (double)n.z * ctx->base.z);
    const m3real linearSlop = 0.005f;
    // sep below is measured to the cast shape's SKIN (the radius is
    // already subtracted), so the stop target is one slop, full
    // stop. The old target of castRadius - slop double-counted the
    // radius and parked every cast one radius short of the plane.
    m3real target = linearSlop;
    m3real tolerance = 0.25f * linearSlop;
    m3real rate = -m3Dot3(n, ctx->translation);
    if (!(rate > 0.0f))
    {
        return; // moving away or parallel
    }
    m3real t = 0.0f;
    m3real maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;
    for (int32_t iter = 0; iter < 25; ++iter)
    {
        // Support of the cast proxy toward the plane at time t.
        m3real minD = 3.4e38f;
        for (int32_t k = 0; k < ctx->castPointCount; ++k)
        {
            m3Vec3 p = m3Add3(ctx->castPoints[k], m3MulSV3(t, ctx->translation));
            minD = m3MinF(minD, m3Dot3(n, p));
        }
        m3real sep = minD - offset - ctx->castRadius;
        if (sep <= 0.0f)
        {
            if (t == 0.0f)
            {
                // Start overlapped.
                if (!ctx->best.hit || 0.0f < ctx->best.fraction ||
                    (0.0f == ctx->best.fraction && shape < ctx->bestShape))
                {
                    ctx->best.hit = true;
                    ctx->best.fraction = 0.0f;
                    ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
                    ctx->best.shapeId = (m3ShapeId){shape + 1, world->idWorld,
                                                    world->shapes.shapePool.generations[shape]};
                    ctx->bestShape = shape;
                }
            }
            return;
        }
        if (sep <= target + tolerance)
        {
            if (t < maxFraction || (t == maxFraction && (!ctx->best.hit || shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = t;
                ctx->best.normal = n;
                ctx->best.shapeId = (m3ShapeId){shape + 1, world->idWorld,
                                                world->shapes.shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
            return;
        }
        t += (sep - target) / rate;
        if (t >= maxFraction)
        {
            return;
        }
    }
}

static bool ShapeCastCallback(int32_t shape, void* userContext)
{
    ShapeCastTestShape((m3ShapeCastContext*)userContext, shape);
    return true;
}

static m3RayCastResult CastConvexFiltered(m3World* worldPtr, m3Pos3 base, const m3Vec3* points,
                                          int32_t pointCount, m3real radius, m3Vec3 translation,
                                          int32_t ignoreBody, m3QueryFilter filter)
{
    m3ShapeCastContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.ignoreBody = ignoreBody;
    ctx.filter = filter;
    ctx.best.fraction = 1.0f;
    m3World* world = worldPtr;
    // A skinless cast (radius zero) is legal for real point clouds:
    // boxes and hulls cast their corners; spheres and capsules keep
    // their mandatory skin.
    bool valid = world != NULL && points != NULL && pointCount >= 1 &&
                 pointCount <= M3_HULL_MAX_VERTS && m3FiniteF(radius) && radius >= 0.0f &&
                 (radius > 0.0f || pointCount >= 2) && m3FinitePos3(base) &&
                 translation.x >= -M3_CAST_LIMIT && translation.x <= M3_CAST_LIMIT &&
                 translation.y >= -M3_CAST_LIMIT && translation.y <= M3_CAST_LIMIT &&
                 translation.z >= -M3_CAST_LIMIT && translation.z <= M3_CAST_LIMIT;
    for (int32_t k = 0; valid && k < pointCount; ++k)
    {
        valid = m3FiniteV3(points[k]);
    }
    if (!valid)
    {
        m3Refuse(world, m3_errorInvalid);
        return ctx.best;
    }
    ctx.world = world;
    ctx.base = base;
    for (int32_t k = 0; k < pointCount; ++k)
    {
        ctx.castPoints[k] = points[k];
    }
    ctx.castPointCount = pointCount;
    ctx.castRadius = radius;
    ctx.translation = translation;
    ctx.bestShape = INT32_MAX;

    // Candidates: the swept box padded by radius and point extent.
    m3real extent = radius;
    for (int32_t k = 0; k < pointCount; ++k)
    {
        extent = m3MaxF(extent, sqrtf(m3Dot3(points[k], points[k])) + radius);
    }
    double lo[3];
    double hi[3];
    double ex = base.x + (double)translation.x;
    double ey = base.y + (double)translation.y;
    double ez = base.z + (double)translation.z;
    lo[0] = (base.x < ex ? base.x : ex) - (double)extent;
    lo[1] = (base.y < ey ? base.y : ey) - (double)extent;
    lo[2] = (base.z < ez ? base.z : ez) - (double)extent;
    hi[0] = (base.x > ex ? base.x : ex) + (double)extent;
    hi[1] = (base.y > ey ? base.y : ey) + (double)extent;
    hi[2] = (base.z > ez ? base.z : ez) + (double)extent;
    m3TreeQuery(&world->broadphase.tree, lo, hi, ShapeCastCallback, &ctx);

    for (int32_t k = 0; k < world->shapes.planeCount; ++k)
    {
        ShapeCastTestPlane(&ctx, world->shapes.planeShapes[k]);
    }

    if (ctx.best.hit)
    {
        ctx.best.point.x = base.x + (double)(ctx.best.fraction * translation.x);
        ctx.best.point.y = base.y + (double)(ctx.best.fraction * translation.y);
        ctx.best.point.z = base.z + (double)(ctx.best.fraction * translation.z);
    }
    return ctx.best;
}

m3RayCastResult m3World_CastBoxClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents,
                                       m3Quat rotation, m3Vec3 translation, m3QueryFilter filter)
{
    m3RayCastResult miss;
    memset(&miss, 0, sizeof(miss));
    m3real qq = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
                rotation.w * rotation.w;
    if (!m3FinitePos3(center) || !m3FiniteV3(halfExtents) || !m3FiniteV3(translation) ||
        !m3FiniteQuat(rotation) || !(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) ||
        !(halfExtents.z > 0.0f) || !(qq > 0.98f) || !(qq < 1.02f))
    {
        m3Refuse(m3WorldFromId(worldId), m3_errorInvalid);
        return miss; // hostile input: the cast quietly misses, loudly
                     // documented (the query contract has no id to
                     // refuse with)
    }
    m3Vec3 corners[8];
    for (int32_t c = 0; c < 8; ++c)
    {
        m3Vec3 local = {(c & 1) != 0 ? halfExtents.x : -halfExtents.x,
                        (c & 2) != 0 ? halfExtents.y : -halfExtents.y,
                        (c & 4) != 0 ? halfExtents.z : -halfExtents.z};
        corners[c] = m3RotateVec3(rotation, local);
    }
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return miss;
    }
    return CastConvexFiltered(world, center, corners, 8, 0.0f, translation, -1, filter);
}

m3RayCastResult m3World_CastHullClosest(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                        int32_t count, m3Vec3 translation, m3QueryFilter filter)
{
    m3RayCastResult miss;
    memset(&miss, 0, sizeof(miss));
    if (points == NULL || count < 2 || count > M3_HULL_MAX_VERTS || !m3FinitePos3(base) ||
        !m3FiniteV3(translation))
    {
        m3Refuse(m3WorldFromId(worldId), m3_errorInvalid);
        return miss; // a one-point skinless cast is a ray: use rays
    }
    for (int32_t k = 0; k < count; ++k)
    {
        if (!m3FiniteV3(points[k]))
        {
            return miss;
        }
    }
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return miss;
    }
    return CastConvexFiltered(world, base, points, count, 0.0f, translation, -1, filter);
}

m3RayCastResult m3CastConvexClosestEx(m3World* worldPtr, m3Pos3 base, const m3Vec3* points,
                                      int32_t pointCount, m3real radius, m3Vec3 translation,
                                      int32_t ignoreBody)
{
    return CastConvexFiltered(worldPtr, base, points, pointCount, radius, translation, ignoreBody,
                              m3DefaultQueryFilter());
}

m3RayCastResult m3World_CastSphereClosest(m3WorldId worldId, m3Pos3 center, m3real radius,
                                          m3Vec3 translation, m3QueryFilter filter)
{
    m3Vec3 point = {0.0f, 0.0f, 0.0f};
    m3World* world = m3WorldFromId(worldId);
    return CastConvexFiltered(world, center, &point, 1, radius, translation, -1, filter);
}

m3RayCastResult m3World_CastCapsuleClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 point1,
                                           m3Vec3 point2, m3real radius, m3Vec3 translation,
                                           m3QueryFilter filter)
{
    m3Vec3 points[2] = {point1, point2};
    m3World* world = m3WorldFromId(worldId);
    return CastConvexFiltered(world, center, points, 2, radius, translation, -1, filter);
}
