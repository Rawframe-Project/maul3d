// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Overlaps of convex proxies (hull point clouds, capsules and boxes)
// against every shape kind, gathered in ascending shape order.

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

typedef struct m3ProxyOverlapContext
{
    m3World* world;
    m3Pos3 base;
    const m3Vec3* points; // base-relative query cloud, <= 64
    int32_t pointCount;
    m3real radius;
    double lo[3];
    double hi[3];
    m3ShapeSelection selection;
    m3QueryFilter filter;
} m3ProxyOverlapContext;

// GJK between the localized query cloud and an arbitrary point set
// (a shape core, a voxel merged box, a mesh triangle), radii applied
// analytically like the whole distance family.
static int ProxyCloudReach(const m3Vec3* cloud, int32_t cloudCount, m3real cloudRadius,
                           const m3Vec3* target, int32_t targetCount, m3real targetRadius)
{
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA.points = target;
    input.proxyA.count = targetCount;
    input.proxyA.radius = 0.0f;
    input.proxyB.points = cloud;
    input.proxyB.count = cloudCount;
    input.proxyB.radius = 0.0f;
    input.q = m3MakeIdentityQuat();
    input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.useRadii = false;
    m3DistanceOutput out = m3ShapeDistance(&input);
    return out.distance <= cloudRadius + targetRadius;
}

static int ProxyReachesShape(const m3ProxyOverlapContext* ctx, int32_t shape)
{
    m3World* world = ctx->world;
    if (ctx->pointCount < 1)
    {
        // The public walls refuse empty clouds already; this guard
        // exists so every compiler can PROVE local[0] is written
        // below (mingw's maybe-uninitialized fired on the release
        // asset build, the one gcc the CI matrix does not run).
        return 0;
    }
    uint8_t type = world->shapes.shapeType[shape];
    m3Transform xf = m3ShapeWorldTransform(world, shape);
    m3Vec3 local[64];
    for (int32_t k = 0; k < ctx->pointCount; ++k)
    {
        m3Vec3 w = {(m3real)(ctx->base.x + (double)ctx->points[k].x - xf.p.x),
                    (m3real)(ctx->base.y + (double)ctx->points[k].y - xf.p.y),
                    (m3real)(ctx->base.z + (double)ctx->points[k].z - xf.p.z)};
        local[k] = m3InvRotateVec3(xf.q, w);
    }
    if (type == (uint8_t)m3_planeShape)
    {
        m3real best = 0.0f;
        for (int32_t k = 0; k < ctx->pointCount; ++k)
        {
            m3real d = m3Dot3(world->shapes.shapeGeom[shape].v, local[k]) -
                       world->shapes.shapeGeom[shape].s;
            if (k == 0 || d < best)
            {
                best = d;
            }
        }
        return best <= ctx->radius;
    }
    if (type == (uint8_t)m3_voxelShape)
    {
        int32_t slot = world->shapes.shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
        m3real cell = world->voxels.voxelData[slot].cellSize;
        m3Vec3 blo = local[0];
        m3Vec3 bhi = local[0];
        for (int32_t k = 1; k < ctx->pointCount; ++k)
        {
            blo.x = local[k].x < blo.x ? local[k].x : blo.x;
            blo.y = local[k].y < blo.y ? local[k].y : blo.y;
            blo.z = local[k].z < blo.z ? local[k].z : blo.z;
            bhi.x = local[k].x > bhi.x ? local[k].x : bhi.x;
            bhi.y = local[k].y > bhi.y ? local[k].y : bhi.y;
            bhi.z = local[k].z > bhi.z ? local[k].z : bhi.z;
        }
        blo = (m3Vec3){blo.x - ctx->radius, blo.y - ctx->radius, blo.z - ctx->radius};
        bhi = (m3Vec3){bhi.x + ctx->radius, bhi.y + ctx->radius, bhi.z + ctx->radius};
        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(&surface->bvh, blo, bhi, gather);
        for (int32_t g = 0; g < gatherCount; ++g)
        {
            m3Vec3 lo;
            m3Vec3 hi;
            m3VoxelBoxBounds(surface, cell, gather[g], &lo, &hi);
            m3Vec3 corners[8];
            for (int32_t c = 0; c < 8; ++c)
            {
                corners[c] = (m3Vec3){(c & 1) != 0 ? hi.x : lo.x, (c & 2) != 0 ? hi.y : lo.y,
                                      (c & 4) != 0 ? hi.z : lo.z};
            }
            if (ProxyCloudReach(local, ctx->pointCount, ctx->radius, corners, 8, 0.0f))
            {
                return 1;
            }
        }
        return 0;
    }
    if (type == (uint8_t)m3_heightFieldShape)
    {
        // The overlap family sees terrain: cloud box, cell
        // gather, the same reach test per triangle.
        const m3HeightFieldData* hf =
            &world->heightFields.hfData[world->shapes.shapeHfIndex[shape]];
        m3Vec3 hlo = local[0];
        m3Vec3 hhi = local[0];
        for (int32_t k = 1; k < ctx->pointCount; ++k)
        {
            hlo.x = local[k].x < hlo.x ? local[k].x : hlo.x;
            hlo.y = local[k].y < hlo.y ? local[k].y : hlo.y;
            hlo.z = local[k].z < hlo.z ? local[k].z : hlo.z;
            hhi.x = local[k].x > hhi.x ? local[k].x : hhi.x;
            hhi.y = local[k].y > hhi.y ? local[k].y : hhi.y;
            hhi.z = local[k].z > hhi.z ? local[k].z : hhi.z;
        }
        hlo = (m3Vec3){hlo.x - ctx->radius, hlo.y - ctx->radius, hlo.z - ctx->radius};
        hhi = (m3Vec3){hhi.x + ctx->radius, hhi.y + ctx->radius, hhi.z + ctx->radius};
        m3Vec3 hfTris[512][3];
        int32_t hfCount = m3HeightFieldGather(hf, hlo, hhi, hfTris, 512);
        for (int32_t t = 0; t < hfCount; ++t)
        {
            if (ProxyCloudReach(local, ctx->pointCount, ctx->radius, hfTris[t], 3, 0.0f))
            {
                return 1;
            }
        }
        return 0;
    }
    if (type == (uint8_t)m3_meshShape)
    {
        const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
        m3Vec3 blo = local[0];
        m3Vec3 bhi = local[0];
        for (int32_t k = 1; k < ctx->pointCount; ++k)
        {
            blo.x = local[k].x < blo.x ? local[k].x : blo.x;
            blo.y = local[k].y < blo.y ? local[k].y : blo.y;
            blo.z = local[k].z < blo.z ? local[k].z : blo.z;
            bhi.x = local[k].x > bhi.x ? local[k].x : bhi.x;
            bhi.y = local[k].y > bhi.y ? local[k].y : bhi.y;
            bhi.z = local[k].z > bhi.z ? local[k].z : bhi.z;
        }
        blo = (m3Vec3){blo.x - ctx->radius, blo.y - ctx->radius, blo.z - ctx->radius};
        bhi = (m3Vec3){bhi.x + ctx->radius, bhi.y + ctx->radius, bhi.z + ctx->radius};
        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(
            &world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape]], blo, bhi, gather);
        for (int32_t g = 0; g < gatherCount; ++g)
        {
            int32_t t = gather[g];
            m3Vec3 tri[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                             mesh->vertices[mesh->indices[3 * t + 1]],
                             mesh->vertices[mesh->indices[3 * t + 2]]};
            if (ProxyCloudReach(local, ctx->pointCount, ctx->radius, tri, 3, 0.0f))
            {
                return 1;
            }
        }
        return 0;
    }
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, shape, scratch);
    return ProxyCloudReach(local, ctx->pointCount, ctx->radius, proxy.points, proxy.count,
                           proxy.radius);
}

static bool ProxyOverlapCallback(int32_t shape, void* userContext)
{
    m3ProxyOverlapContext* ctx = (m3ProxyOverlapContext*)userContext;
    if (ctx->world->bodies.bodyEnabled[ctx->world->shapes.shapeBody[shape]] == 0)
    {
        return true;
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits,
                      ctx->world->shapes.shapeCategory[shape], ctx->world->shapes.shapeMask[shape]))
    {
        return true;
    }
    if (!ProxyReachesShape(ctx, shape))
    {
        return true;
    }
    m3SelectionOffer(&ctx->selection, ctx->world, shape);
    return true;
}

static int32_t ProxyOverlapGather(m3World* world, m3ProxyOverlapContext* ctx, m3ShapeId* shapes,
                                  int32_t capacity)
{
    ctx->selection = (m3ShapeSelection){shapes, capacity, 0};
    m3TreeQuery(&world->broadphase.tree, ctx->lo, ctx->hi, ProxyOverlapCallback, ctx);
    for (int32_t k = 0; k < world->shapes.planeCount; ++k)
    {
        int32_t s = world->shapes.planeShapes[k];
        if (world->bodies.bodyEnabled[world->shapes.shapeBody[s]] != 0 &&
            m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits,
                         world->shapes.shapeCategory[s], world->shapes.shapeMask[s]) &&
            ProxyReachesShape(ctx, s))
        {
            m3SelectionOffer(&ctx->selection, world, s);
        }
    }
    return m3SelectionFinish(&ctx->selection);
}

int32_t m3World_OverlapHullPoints(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                  int32_t count, m3real radius, m3ShapeId* shapes, int32_t capacity,
                                  m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || points == NULL || count < 1 || count > 64 || shapes == NULL ||
        capacity <= 0 || !m3FinitePos3(base) || !m3FiniteF(radius) || radius < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    for (int32_t k = 0; k < count; ++k)
    {
        if (!m3FiniteV3(points[k]))
        {
            m3Refuse(world, m3_errorInvalid);
            return 0;
        }
    }
    m3ProxyOverlapContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.world = world;
    ctx.base = base;
    ctx.points = points;
    ctx.pointCount = count;
    ctx.radius = radius;
    ctx.filter = filter;
    ctx.lo[0] = base.x + (double)points[0].x;
    ctx.lo[1] = base.y + (double)points[0].y;
    ctx.lo[2] = base.z + (double)points[0].z;
    ctx.hi[0] = ctx.lo[0];
    ctx.hi[1] = ctx.lo[1];
    ctx.hi[2] = ctx.lo[2];
    for (int32_t k = 1; k < count; ++k)
    {
        double x = base.x + (double)points[k].x;
        double y = base.y + (double)points[k].y;
        double z = base.z + (double)points[k].z;
        ctx.lo[0] = x < ctx.lo[0] ? x : ctx.lo[0];
        ctx.lo[1] = y < ctx.lo[1] ? y : ctx.lo[1];
        ctx.lo[2] = z < ctx.lo[2] ? z : ctx.lo[2];
        ctx.hi[0] = x > ctx.hi[0] ? x : ctx.hi[0];
        ctx.hi[1] = y > ctx.hi[1] ? y : ctx.hi[1];
        ctx.hi[2] = z > ctx.hi[2] ? z : ctx.hi[2];
    }
    for (int32_t a = 0; a < 3; ++a)
    {
        ctx.lo[a] -= (double)radius;
        ctx.hi[a] += (double)radius;
    }
    return ProxyOverlapGather(world, &ctx, shapes, capacity);
}

int32_t m3World_OverlapCapsule(m3WorldId worldId, m3Pos3 p1, m3Pos3 p2, m3real radius,
                               m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter)
{
    if (!m3FinitePos3(p1) || !m3FinitePos3(p2))
    {
        m3Refuse(m3WorldFromId(worldId), m3_errorInvalid);
        return 0;
    }
    m3Vec3 pts[2] = {{0.0f, 0.0f, 0.0f},
                     {(m3real)(p2.x - p1.x), (m3real)(p2.y - p1.y), (m3real)(p2.z - p1.z)}};
    return m3World_OverlapHullPoints(worldId, p1, pts, 2, radius, shapes, capacity, filter);
}

int32_t m3World_OverlapBox(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents, m3Quat rotation,
                           m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter)
{
    if (!m3FiniteV3(halfExtents) || !(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) ||
        !(halfExtents.z > 0.0f) || !m3FiniteQuat(rotation))
    {
        m3Refuse(m3WorldFromId(worldId), m3_errorInvalid);
        return 0;
    }
    m3Vec3 corners[8];
    for (int32_t c = 0; c < 8; ++c)
    {
        m3Vec3 e = {(c & 1) != 0 ? halfExtents.x : -halfExtents.x,
                    (c & 2) != 0 ? halfExtents.y : -halfExtents.y,
                    (c & 4) != 0 ? halfExtents.z : -halfExtents.z};
        corners[c] = m3RotateVec3(rotation, e);
    }
    return m3World_OverlapHullPoints(worldId, center, corners, 8, 0.0f, shapes, capacity, filter);
}
