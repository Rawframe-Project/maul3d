// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape casts: spheres, capsules, boxes and hulls swept through the
// world with the shared time-of-impact kernel, closest hit first.

#include "distance.h"
#include "fpenv.h"
#include "manifold.h"
#include "misuse.h"
#include "query.h"
#include "raycast.h"
#include "shape_state.h"
#include "world_internal.h"

#include <float.h>
#include <string.h>

typedef struct m3ShapeCastContext
{
    const m3World* world;
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

// A cast against a static surface's pieces: its triangles, or a voxel
// chunk's boxes. The sweeps of the surface and of the cast are relative
// to the cast's start, and the cast's path is kept in the surface's frame
// with the cast shape's reach, so only pieces near the path are gathered
// and a piece is tried only while it lies within the path up to the
// earliest hit found so far.
typedef struct SurfaceCast
{
    m3ShapeCastContext* ctx;
    int32_t shape;
    m3Sweep surface;
    m3Sweep cast;
    // The cast's base from fraction 0 to 1 in the surface's frame, its
    // reach the cast shape's farthest point from the base with skin, and
    // until the earliest hit so far.
    m3SurfacePath path;
} SurfaceCast;

static void SurfaceCastBegin(m3ShapeCastContext* ctx, int32_t shape, SurfaceCast* sc)
{
    m3Transform xf = m3ShapeWorldTransform(ctx->world, shape);
    sc->ctx = ctx;
    sc->shape = shape;
    sc->surface.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
    sc->surface.c1 = (m3Vec3){(m3real)(xf.p.x - ctx->base.x), (m3real)(xf.p.y - ctx->base.y),
                              (m3real)(xf.p.z - ctx->base.z)};
    sc->surface.c2 = sc->surface.c1;
    sc->surface.q1 = xf.q;
    sc->surface.q2 = xf.q;
    sc->cast.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
    sc->cast.c1 = (m3Vec3){0.0f, 0.0f, 0.0f};
    sc->cast.c2 = ctx->translation;
    sc->cast.q1 = m3MakeIdentityQuat();
    sc->cast.q2 = m3MakeIdentityQuat();
    sc->path.start = m3InvRotateVec3(xf.q, m3Neg3(sc->surface.c1));
    sc->path.end = m3InvRotateVec3(xf.q, m3Sub3(ctx->translation, sc->surface.c1));
    m3real farthest = 0.0f;
    for (int32_t k = 0; k < ctx->castPointCount; ++k)
    {
        farthest = m3MaxF(farthest, m3Length3(ctx->castPoints[k]));
    }
    sc->path.reach = farthest + ctx->castRadius + M3_AABB_MARGIN;
    sc->path.until = ctx->best.hit ? ctx->best.fraction : 1.0f;
}

// The box the cast covers in the surface's frame up to fraction.
static void SurfaceCastBox(const SurfaceCast* sc, m3real fraction, m3Vec3* lo, m3Vec3* hi)
{
    m3Vec3 start = sc->path.start;
    m3Vec3 end = m3Add3(start, m3MulSV3(fraction, m3Sub3(sc->path.end, start)));
    m3real r = sc->path.reach;
    *lo = (m3Vec3){m3MinF(start.x, end.x) - r, m3MinF(start.y, end.y) - r,
                   m3MinF(start.z, end.z) - r};
    *hi = (m3Vec3){m3MaxF(start.x, end.x) + r, m3MaxF(start.y, end.y) + r,
                   m3MaxF(start.z, end.z) + r};
}

// One piece's time of impact, kept when it is the earliest hit, ties to
// the lower shape index. A piece outside the cast's path up to the
// earliest hit so far cannot come earlier and is not tried.
static void CastPiece(SurfaceCast* sc, const m3Vec3* points, int32_t count, m3Vec3 lo, m3Vec3 hi)
{
    m3ShapeCastContext* ctx = sc->ctx;
    const m3World* world = ctx->world;
    int32_t shape = sc->shape;
    m3Vec3 pathLo;
    m3Vec3 pathHi;
    SurfaceCastBox(sc, ctx->best.hit ? ctx->best.fraction : 1.0f, &pathLo, &pathHi);
    if (hi.x < pathLo.x || lo.x > pathHi.x || hi.y < pathLo.y || lo.y > pathHi.y ||
        hi.z < pathLo.z || lo.z > pathHi.z)
    {
        return;
    }
    m3TOIInput input;
    input.proxyA.points = points;
    input.proxyA.count = count;
    input.proxyA.radius = 0.0f;
    input.proxyB.points = ctx->castPoints;
    input.proxyB.count = ctx->castPointCount;
    input.proxyB.radius = ctx->castRadius;
    input.sweepA = sc->surface;
    input.sweepB = sc->cast;
    input.maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;
    m3TOIOutput out = m3TimeOfImpact(&input);
    if (out.state == m3_toiStateHit &&
        (!ctx->best.hit || out.fraction < ctx->best.fraction ||
         (out.fraction == ctx->best.fraction && shape < ctx->bestShape)))
    {
        ctx->best.hit = true;
        ctx->best.fraction = out.fraction;
        ctx->best.normal = out.normal;
        ctx->best.shapeId = (m3ShapeId){shape + 1, world->shapes.shapePool.generations[shape]};
        ctx->bestShape = shape;
    }
    else if (out.state == m3_toiStateOverlapped &&
             (!ctx->best.hit || 0.0f < ctx->best.fraction ||
              (0.0f == ctx->best.fraction && shape < ctx->bestShape)))
    {
        ctx->best.hit = true;
        ctx->best.fraction = 0.0f;
        ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
        ctx->best.shapeId = (m3ShapeId){shape + 1, world->shapes.shapePool.generations[shape]};
        ctx->bestShape = shape;
    }
    sc->path.until = ctx->best.hit ? ctx->best.fraction : 1.0f;
}

static bool CastTriangle(const m3Vec3 tri[3], void* context)
{
    m3Vec3 tv[3] = {tri[0], tri[1], tri[2]};
    m3Vec3 lo = {m3MinF(tv[0].x, m3MinF(tv[1].x, tv[2].x)),
                 m3MinF(tv[0].y, m3MinF(tv[1].y, tv[2].y)),
                 m3MinF(tv[0].z, m3MinF(tv[1].z, tv[2].z))};
    m3Vec3 hi = {m3MaxF(tv[0].x, m3MaxF(tv[1].x, tv[2].x)),
                 m3MaxF(tv[0].y, m3MaxF(tv[1].y, tv[2].y)),
                 m3MaxF(tv[0].z, m3MaxF(tv[1].z, tv[2].z))};
    CastPiece((SurfaceCast*)context, tv, 3, lo, hi);
    return true;
}

// Mesh targets: per-triangle TOI over the triangles in the cast's path
// (the same recipe as continuous collision).
static void CastMesh(m3ShapeCastContext* ctx, int32_t shape)
{
    const m3World* world = ctx->world;
    const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
    SurfaceCast sc;
    SurfaceCastBegin(ctx, shape, &sc);
    m3MeshBvhHits gather;
    m3MeshBvhGatherPath(&world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape]], &sc.path,
                        &gather);
    for (int32_t t = m3MeshBvhFirstHit(&gather); t >= 0; t = m3MeshBvhNextHit(&gather, t))
    {
        m3Vec3 tv[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                        mesh->vertices[mesh->indices[3 * t + 1]],
                        mesh->vertices[mesh->indices[3 * t + 2]]};
        (void)CastTriangle(tv, &sc);
    }
}

// Height field grid targets: the same recipe over the triangles of the
// cells near the cast's path.
static void CastHeightField(m3ShapeCastContext* ctx, int32_t shape)
{
    const m3World* world = ctx->world;
    SurfaceCast sc;
    SurfaceCastBegin(ctx, shape, &sc);
    m3HeightFieldVisitPath(&world->heightFields.hfData[world->shapes.shapeHfIndex[shape]], &sc.path,
                           CastTriangle, &sc);
}

// Voxel targets: per-box TOI against the merged surface, unextended
// (queries report true geometry; the seam extension is a contact-only
// device).
static void CastVoxels(m3ShapeCastContext* ctx, int32_t shape)
{
    const m3World* world = ctx->world;
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    const m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
    m3real cell = world->voxels.voxelData[slot].cellSize;
    SurfaceCast sc;
    SurfaceCastBegin(ctx, shape, &sc);
    m3MeshBvhHits gather;
    m3MeshBvhGatherPath(&surface->bvh, &sc.path, &gather);
    for (int32_t g = m3MeshBvhFirstHit(&gather); g >= 0; g = m3MeshBvhNextHit(&gather, g))
    {
        m3Vec3 lo;
        m3Vec3 hi;
        m3VoxelBoxBounds(surface, cell, g, &lo, &hi);
        m3Vec3 corners[8];
        for (int32_t k = 0; k < 8; ++k)
        {
            corners[k] = (m3Vec3){(k & 1) != 0 ? hi.x : lo.x, (k & 2) != 0 ? hi.y : lo.y,
                                  (k & 4) != 0 ? hi.z : lo.z};
        }
        CastPiece(&sc, corners, 8, lo, hi);
    }
}

static void ShapeCastTestShape(m3ShapeCastContext* ctx, int32_t shape)
{
    const m3World* world = ctx->world;
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
        CastVoxels(ctx, shape);
        return;
    }
    if (world->shapes.shapeType[shape] == (uint8_t)m3_meshShape)
    {
        CastMesh(ctx, shape);
        return;
    }
    if (world->shapes.shapeType[shape] == (uint8_t)m3_heightFieldShape)
    {
        CastHeightField(ctx, shape);
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
        ctx->best.shapeId = (m3ShapeId){shape + 1, world->shapes.shapePool.generations[shape]};
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
            ctx->best.shapeId = (m3ShapeId){shape + 1, world->shapes.shapePool.generations[shape]};
            ctx->bestShape = shape;
        }
    }
}

// A plane target for a convex cast: conservative advance along the
// analytic support distance (the CCD plane recipe).
static void ShapeCastTestPlane(m3ShapeCastContext* ctx, int32_t shape)
{
    const m3World* world = ctx->world;
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
    // stop. A target of castRadius - slop would count the radius
    // twice and park every cast one radius short of the plane.
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
                    ctx->best.shapeId =
                        (m3ShapeId){shape + 1, world->shapes.shapePool.generations[shape]};
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
                ctx->best.shapeId =
                    (m3ShapeId){shape + 1, world->shapes.shapePool.generations[shape]};
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

static m3RayCastResult CastConvexFiltered(const m3World* worldPtr, m3Pos3 base,
                                          const m3Vec3* points, int32_t pointCount, m3real radius,
                                          m3Vec3 translation, int32_t ignoreBody,
                                          m3QueryFilter filter)
{
    m3ShapeCastContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.ignoreBody = ignoreBody;
    ctx.filter = filter;
    ctx.best.fraction = 1.0f;
    const m3World* world = worldPtr;
    // A skinless cast (radius zero) is legal for real point clouds:
    // boxes and hulls cast their corners; spheres and capsules keep
    // their mandatory skin.
    bool valid = world != NULL && points != NULL && pointCount >= 1 &&
                 pointCount <= M3_MAX_CAST_POINTS && m3FiniteF(radius) && radius >= 0.0f &&
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

static M3_NOINLINE m3RayCastResult WorldCastBoxClosestWorker(const m3World* world, m3Pos3 center,
                                                             m3Vec3 halfExtents, m3Quat rotation,
                                                             m3Vec3 translation,
                                                             m3QueryFilter filter)
{
    m3RayCastResult miss;
    memset(&miss, 0, sizeof(miss));
    m3real qq = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
                rotation.w * rotation.w;
    if (!m3FinitePos3(center) || !m3FiniteV3(halfExtents) || !m3FiniteV3(translation) ||
        !m3FiniteQuat(rotation) || !(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) ||
        !(halfExtents.z > 0.0f) || !(qq > 0.98f) || !(qq < 1.02f))
    {
        m3Refuse(world, m3_errorInvalid);
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
    if (world == NULL)
    {
        return miss;
    }
    return CastConvexFiltered(world, center, corners, 8, 0.0f, translation, -1, filter);
}

m3RayCastResult m3World_CastBoxClosest(const m3World* world, m3Pos3 center, m3Vec3 halfExtents,
                                       m3Quat rotation, m3Vec3 translation, m3QueryFilter filter)
{
    uint64_t env = m3FloatEnvEnter();
    m3RayCastResult result =
        WorldCastBoxClosestWorker(world, center, halfExtents, rotation, translation, filter);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3RayCastResult WorldCastHullClosestWorker(const m3World* world, m3Pos3 base,
                                                              const m3Vec3* points, int32_t count,
                                                              m3Vec3 translation,
                                                              m3QueryFilter filter)
{
    m3RayCastResult miss;
    memset(&miss, 0, sizeof(miss));
    if (points == NULL || count < 2 || count > M3_MAX_CAST_POINTS || !m3FinitePos3(base) ||
        !m3FiniteV3(translation))
    {
        m3Refuse(world, m3_errorInvalid);
        return miss; // a one-point skinless cast is a ray: use rays
    }
    for (int32_t k = 0; k < count; ++k)
    {
        if (!m3FiniteV3(points[k]))
        {
            m3Refuse(world, m3_errorInvalid);
            return miss;
        }
    }
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return miss;
    }
    return CastConvexFiltered(world, base, points, count, 0.0f, translation, -1, filter);
}

m3RayCastResult m3World_CastHullClosest(const m3World* world, m3Pos3 base, const m3Vec3* points,
                                        int32_t count, m3Vec3 translation, m3QueryFilter filter)
{
    uint64_t env = m3FloatEnvEnter();
    m3RayCastResult result =
        WorldCastHullClosestWorker(world, base, points, count, translation, filter);
    m3FloatEnvLeave(env);
    return result;
}

m3RayCastResult m3CastConvexClosestEx(const m3World* worldPtr, m3Pos3 base, const m3Vec3* points,
                                      int32_t pointCount, m3real radius, m3Vec3 translation,
                                      int32_t ignoreBody)
{
    return CastConvexFiltered(worldPtr, base, points, pointCount, radius, translation, ignoreBody,
                              m3DefaultQueryFilter());
}

static M3_NOINLINE m3RayCastResult WorldCastSphereClosestWorker(const m3World* world, m3Pos3 center,
                                                                m3real radius, m3Vec3 translation,
                                                                m3QueryFilter filter)
{
    m3Vec3 point = {0.0f, 0.0f, 0.0f};
    return CastConvexFiltered(world, center, &point, 1, radius, translation, -1, filter);
}

m3RayCastResult m3World_CastSphereClosest(const m3World* world, m3Pos3 center, m3real radius,
                                          m3Vec3 translation, m3QueryFilter filter)
{
    uint64_t env = m3FloatEnvEnter();
    m3RayCastResult result =
        WorldCastSphereClosestWorker(world, center, radius, translation, filter);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3RayCastResult WorldCastCapsuleClosestWorker(const m3World* world,
                                                                 m3Pos3 center, m3Vec3 point1,
                                                                 m3Vec3 point2, m3real radius,
                                                                 m3Vec3 translation,
                                                                 m3QueryFilter filter)
{
    m3Vec3 points[2] = {point1, point2};
    return CastConvexFiltered(world, center, points, 2, radius, translation, -1, filter);
}

m3RayCastResult m3World_CastCapsuleClosest(const m3World* world, m3Pos3 center, m3Vec3 point1,
                                           m3Vec3 point2, m3real radius, m3Vec3 translation,
                                           m3QueryFilter filter)
{
    uint64_t env = m3FloatEnvEnter();
    m3RayCastResult result =
        WorldCastCapsuleClosestWorker(world, center, point1, point2, radius, translation, filter);
    m3FloatEnvLeave(env);
    return result;
}
