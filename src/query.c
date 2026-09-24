// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The query surface: multi-hit rays, point containment, sphere and box
// overlaps and contact readback. Shape casts live in shape_cast.c,
// convex proxy overlaps in query_proxy.c and the mover kit in mover.c.
// Pure observers: no query moves a bit of simulation state, and every
// result orders canonically (fraction then shape index, or plain
// ascending shape index), so twin worlds answer identically.

#include "query.h"
#include "body.h"
#include "distance.h"
#include "journal.h"
#include "manifold.h"
#include "raycast.h"
#include "shape.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <float.h>
#include <string.h>

// ------------------------------------------------------------------
// Multi-hit ray: every shape's entry point, sorted.
// ------------------------------------------------------------------

typedef struct m3RayAllContext
{
    m3World* world;
    m3Pos3 origin;
    m3Vec3 translation;
    m3RayHit* hits;
    int32_t capacity;
    int32_t count;
    m3QueryFilter filter;
} m3RayAllContext;

// The single-shape ray test lives in raycast.c; queries reuse it
// through this internal hook.

static void RayAllInsert(m3RayAllContext* ctx, const m3RayCastResult* hit, int32_t shapeIndex)
{
    // Insertion sort by (fraction, shape index): capacities are
    // small and the order is the contract.
    int32_t pos = ctx->count;
    while (pos > 0)
    {
        const m3RayHit* prev = &ctx->hits[pos - 1];
        int after = hit->fraction > prev->fraction ||
                    (hit->fraction == prev->fraction && shapeIndex > prev->shapeId.index1 - 1);
        if (after)
        {
            break;
        }
        pos -= 1;
    }
    if (pos >= ctx->capacity)
    {
        return; // beyond capacity: the far end drops, deterministically
    }
    int32_t last = ctx->count < ctx->capacity ? ctx->count : ctx->capacity - 1;
    for (int32_t k = last; k > pos; --k)
    {
        ctx->hits[k] = ctx->hits[k - 1];
    }
    ctx->hits[pos] = (m3RayHit){hit->shapeId, hit->point, hit->normal, hit->fraction};
    if (ctx->count < ctx->capacity)
    {
        ctx->count += 1;
    }
}

static bool RayAllCallback(int32_t shape, void* userContext)
{
    m3RayAllContext* ctx = (m3RayAllContext*)userContext;
    if (ctx->world->bodies.bodyEnabled[ctx->world->shapes.shapeBody[shape]] == 0)
    {
        return true; // disabled bodies vanish from queries
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits,
                      ctx->world->shapes.shapeCategory[shape], ctx->world->shapes.shapeMask[shape]))
    {
        return true; // filtered out
    }
    m3RayCastResult hit = m3RayTestOneShape(ctx->world, shape, ctx->origin, ctx->translation);
    if (hit.hit)
    {
        RayAllInsert(ctx, &hit, shape);
    }
    return true;
}

int32_t m3World_CastRayAll(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation, m3RayHit* hits,
                           int32_t capacity, m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || hits == NULL || capacity <= 0 || !m3FinitePos3(origin) ||
        !(m3Dot3(translation, translation) > 0.0f) ||
        !(translation.x >= -M3_CAST_LIMIT && translation.x <= M3_CAST_LIMIT) ||
        !(translation.y >= -M3_CAST_LIMIT && translation.y <= M3_CAST_LIMIT) ||
        !(translation.z >= -M3_CAST_LIMIT && translation.z <= M3_CAST_LIMIT))
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    m3RayAllContext ctx = {world, origin, translation, hits, capacity, 0, filter};
    double lo[3];
    double hi[3];
    double ex = origin.x + (double)translation.x;
    double ey = origin.y + (double)translation.y;
    double ez = origin.z + (double)translation.z;
    lo[0] = origin.x < ex ? origin.x : ex;
    lo[1] = origin.y < ey ? origin.y : ey;
    lo[2] = origin.z < ez ? origin.z : ez;
    hi[0] = origin.x > ex ? origin.x : ex;
    hi[1] = origin.y > ey ? origin.y : ey;
    hi[2] = origin.z > ez ? origin.z : ez;
    m3TreeQuery(&world->broadphase.tree, lo, hi, RayAllCallback, &ctx);
    for (int32_t k = 0; k < world->shapes.planeCount; ++k)
    {
        int32_t s = world->shapes.planeShapes[k];
        if (m3FilterPass(filter.categoryBits, filter.maskBits, world->shapes.shapeCategory[s],
                         world->shapes.shapeMask[s]))
        {
            m3RayCastResult hit = m3RayTestOneShape(world, s, origin, translation);
            if (hit.hit)
            {
                RayAllInsert(&ctx, &hit, s);
            }
        }
    }
    return ctx.count;
}

// ------------------------------------------------------------------
// Point containment and overlaps.
// ------------------------------------------------------------------

static int PointInVoxel(const m3World* world, int32_t shape, m3Pos3 point);

static int PointInShape(const m3World* world, int32_t shape, m3Pos3 point)
{
    m3Transform xfS7 = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS7;
    m3Vec3 local =
        m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(point.x - xf->p.x), (m3real)(point.y - xf->p.y),
                                        (m3real)(point.z - xf->p.z)});
    uint8_t type = world->shapes.shapeType[shape];
    if (type == (uint8_t)m3_sphereShape)
    {
        m3Vec3 d = m3Sub3(local, world->shapes.shapeGeom[shape].v);
        m3real r = world->shapes.shapeGeom[shape].s;
        return m3Dot3(d, d) <= r * r;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        m3Vec3 p1 = world->shapes.shapeGeom[shape].v;
        m3Vec3 axis = m3Sub3(world->shapes.shapeGeom[shape].v2, p1);
        m3real len2 = m3Dot3(axis, axis);
        m3real t = len2 > 0.0f ? m3Dot3(m3Sub3(local, p1), axis) / len2 : 0.0f;
        t = m3MaxF(0.0f, m3MinF(1.0f, t));
        m3Vec3 closest = m3Add3(p1, m3MulSV3(t, axis));
        m3Vec3 d = m3Sub3(local, closest);
        m3real r = world->shapes.shapeGeom[shape].s;
        return m3Dot3(d, d) <= r * r;
    }
    if (type == (uint8_t)m3_hullShape)
    {
        const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[shape]];
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            if (m3Dot3(hull->faceNormals[f], local) - hull->faceOffsets[f] > 0.0f)
            {
                return 0;
            }
        }
        return 1;
    }
    if (type == (uint8_t)m3_planeShape)
    {
        // Solid half space: at or below the surface.
        return m3Dot3(world->shapes.shapeGeom[shape].v, local) - world->shapes.shapeGeom[shape].s <=
               0.0f;
    }
    if (type == (uint8_t)m3_voxelShape)
    {
        return PointInVoxel(world, shape, point);
    }
    return 0; // meshes are open surfaces: no interior
}

// Solid voxels are CLOSED volumes: containment is a grid lookup.
static int PointInVoxel(const m3World* world, int32_t shape, m3Pos3 point)
{
    m3Transform xfS = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS;
    m3Vec3 local =
        m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(point.x - xf->p.x), (m3real)(point.y - xf->p.y),
                                        (m3real)(point.z - xf->p.z)});
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3real cell = world->voxels.voxelData[slot].cellSize;
    int32_t x = (int32_t)(local.x / cell);
    int32_t y = (int32_t)(local.y / cell);
    int32_t z = (int32_t)(local.z / cell);
    if (local.x < 0.0f || local.y < 0.0f || local.z < 0.0f || x >= M3_VOXEL_DIM ||
        y >= M3_VOXEL_DIM || z >= M3_VOXEL_DIM)
    {
        return 0;
    }
    return m3VoxelGet(&world->voxels.voxelData[slot], x, y, z) ? 1 : 0;
}

m3ShapeId m3World_TestPoint(m3WorldId worldId, m3Pos3 point)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FinitePos3(point))
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullShapeId;
    }
    int32_t maxShape = world->shapes.shapePool.maxIndex;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapes.shapePool.alive[s] != 0 && PointInShape(world, s, point))
        {
            return (m3ShapeId){s + 1, world->idWorld, world->shapes.shapePool.generations[s]};
        }
    }
    return m3_nullShapeId;
}

// Keeps the `capacity` lowest shape slots offered, ascending, in the
// caller's result array: a bounded max-heap on the slot while
// collecting, sorted in place at the end. No cap beyond the caller's
// capacity, and the kept set does not depend on tree visit order.

static void SelectionSiftDown(m3ShapeId* heap, int32_t size, int32_t i)
{
    for (;;)
    {
        int32_t largest = i;
        int32_t left = 2 * i + 1;
        int32_t right = left + 1;
        if (left < size && heap[left].index1 > heap[largest].index1)
        {
            largest = left;
        }
        if (right < size && heap[right].index1 > heap[largest].index1)
        {
            largest = right;
        }
        if (largest == i)
        {
            return;
        }
        m3ShapeId swap = heap[i];
        heap[i] = heap[largest];
        heap[largest] = swap;
        i = largest;
    }
}

void m3SelectionOffer(m3ShapeSelection* sel, const m3World* world, int32_t shape)
{
    m3ShapeId id = {shape + 1, world->idWorld, world->shapes.shapePool.generations[shape]};
    if (sel->size < sel->capacity)
    {
        int32_t i = sel->size++;
        sel->ids[i] = id;
        while (i > 0 && sel->ids[(i - 1) / 2].index1 < sel->ids[i].index1)
        {
            m3ShapeId swap = sel->ids[i];
            sel->ids[i] = sel->ids[(i - 1) / 2];
            sel->ids[(i - 1) / 2] = swap;
            i = (i - 1) / 2;
        }
    }
    else if (sel->capacity > 0 && id.index1 < sel->ids[0].index1)
    {
        sel->ids[0] = id;
        SelectionSiftDown(sel->ids, sel->size, 0);
    }
}

// Sorts the kept ids ascending and returns how many were written.
int32_t m3SelectionFinish(m3ShapeSelection* sel)
{
    for (int32_t end = sel->size - 1; end > 0; --end)
    {
        m3ShapeId swap = sel->ids[0];
        sel->ids[0] = sel->ids[end];
        sel->ids[end] = swap;
        SelectionSiftDown(sel->ids, end, 0);
    }
    return sel->size;
}

typedef struct m3OverlapContext
{
    m3World* world;
    m3Pos3 center;
    m3real radius; // < 0 = pure AABB gather
    double lo[3];
    double hi[3];
    m3ShapeSelection selection;
    m3QueryFilter filter;
} m3OverlapContext;

static int SphereReachesShape(m3World* world, int32_t shape, m3Pos3 center, m3real radius)
{
    uint8_t type = world->shapes.shapeType[shape];
    if (type == (uint8_t)m3_planeShape)
    {
        m3Transform xfS3 = m3ShapeWorldTransform(world, shape);
        const m3Transform* xf = &xfS3;
        m3Vec3 local = m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(center.x - xf->p.x),
                                                       (m3real)(center.y - xf->p.y),
                                                       (m3real)(center.z - xf->p.z)});
        m3real d =
            m3Dot3(world->shapes.shapeGeom[shape].v, local) - world->shapes.shapeGeom[shape].s;
        return d <= radius;
    }
    if (type == (uint8_t)m3_voxelShape)
    {
        // Reach against the merged boxes: exact clamp per candidate
        // (boxes are axis-aligned in the chunk frame).
        m3Transform xfS4 = m3ShapeWorldTransform(world, shape);
        const m3Transform* xf = &xfS4;
        m3Vec3 local = m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(center.x - xf->p.x),
                                                       (m3real)(center.y - xf->p.y),
                                                       (m3real)(center.z - xf->p.z)});
        int32_t slot = world->shapes.shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
        m3real cell = world->voxels.voxelData[slot].cellSize;
        uint16_t gather[M3_MESH_MAX_TRIS];
        m3Vec3 blo = {local.x - radius, local.y - radius, local.z - radius};
        m3Vec3 bhi = {local.x + radius, local.y + radius, local.z + radius};
        int32_t gatherCount = m3MeshBvhGather(&surface->bvh, blo, bhi, gather);
        for (int32_t g = 0; g < gatherCount; ++g)
        {
            m3Vec3 lo;
            m3Vec3 hi;
            m3VoxelBoxBounds(surface, cell, gather[g], &lo, &hi);
            m3Vec3 closest = {m3ClampF(local.x, lo.x, hi.x), m3ClampF(local.y, lo.y, hi.y),
                              m3ClampF(local.z, lo.z, hi.z)};
            m3Vec3 d = m3Sub3(local, closest);
            if (m3Dot3(d, d) <= radius * radius)
            {
                return 1;
            }
        }
        return 0;
    }
    if (type == (uint8_t)m3_heightFieldShape)
    {
        // Distance to any terrain triangle within reach: the
        // mesh recipe over the cell gather.
        m3Transform xfS6 = m3ShapeWorldTransform(world, shape);
        const m3Transform* xfH = &xfS6;
        m3Vec3 localH = m3InvRotateVec3(xfH->q, (m3Vec3){(m3real)(center.x - xfH->p.x),
                                                         (m3real)(center.y - xfH->p.y),
                                                         (m3real)(center.z - xfH->p.z)});
        const m3HeightFieldData* hf =
            &world->heightFields.hfData[world->shapes.shapeHfIndex[shape]];
        m3Vec3 hfTris[512][3];
        int32_t hfCount = m3HeightFieldGather(
            hf, (m3Vec3){localH.x - radius, localH.y - radius, localH.z - radius},
            (m3Vec3){localH.x + radius, localH.y + radius, localH.z + radius}, hfTris, 512);
        for (int32_t t = 0; t < hfCount; ++t)
        {
            // As for meshes: the conservative
            // vertex-distance check suffices here.
            m3Vec3 d0 = m3Sub3(localH, hfTris[t][0]);
            m3Vec3 d1 = m3Sub3(localH, hfTris[t][1]);
            m3Vec3 d2 = m3Sub3(localH, hfTris[t][2]);
            m3real r2 = radius * radius;
            if (m3Dot3(d0, d0) <= r2 || m3Dot3(d1, d1) <= r2 || m3Dot3(d2, d2) <= r2)
            {
                return 1;
            }
        }
        return 0;
    }
    if (type == (uint8_t)m3_meshShape)
    {
        // Distance to any triangle within reach (bounded scan).
        m3Transform xfS5 = m3ShapeWorldTransform(world, shape);
        const m3Transform* xf = &xfS5;
        m3Vec3 local = m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(center.x - xf->p.x),
                                                       (m3real)(center.y - xf->p.y),
                                                       (m3real)(center.z - xf->p.z)});
        const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
        uint16_t gather[M3_MESH_MAX_TRIS];
        m3Vec3 blo = {local.x - radius, local.y - radius, local.z - radius};
        m3Vec3 bhi = {local.x + radius, local.y + radius, local.z + radius};
        int32_t gatherCount = m3MeshBvhGather(
            &world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape]], blo, bhi, gather);
        for (int32_t g = 0; g < gatherCount; ++g)
        {
            int32_t t = gather[g];
            m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
            m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
            m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
            // Cheap reject on the triangle box, exact on the plane
            // projection clamped by the closest-point routine's job:
            // a conservative vertex-distance check suffices here.
            m3Vec3 d0 = m3Sub3(local, a);
            m3Vec3 d1 = m3Sub3(local, b);
            m3Vec3 d2 = m3Sub3(local, c);
            m3real r2 = (radius + 0.0f) * (radius + 0.0f);
            if (m3Dot3(d0, d0) <= r2 || m3Dot3(d1, d1) <= r2 || m3Dot3(d2, d2) <= r2)
            {
                return 1;
            }
        }
        return 0;
    }
    // Convex families: GJK distance between the sphere point and the
    // shape core, radii applied analytically.
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, shape, scratch);
    m3Transform xfS8 = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS8;
    m3Vec3 local =
        m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(center.x - xf->p.x), (m3real)(center.y - xf->p.y),
                                        (m3real)(center.z - xf->p.z)});
    m3Vec3 point = local;
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA = proxy;
    input.proxyB.points = &point;
    input.proxyB.count = 1;
    input.proxyB.radius = 0.0f;
    input.q = m3MakeIdentityQuat();
    input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.useRadii = false;
    m3DistanceOutput out = m3ShapeDistance(&input);
    return out.distance - proxy.radius <= radius;
}

static bool OverlapCallback(int32_t shape, void* userContext)
{
    {
        m3OverlapContext* fctx = (m3OverlapContext*)userContext;
        if (fctx->world->bodies.bodyEnabled[fctx->world->shapes.shapeBody[shape]] == 0)
        {
            return true; // disabled bodies vanish from queries
        }
        if (!m3FilterPass(fctx->filter.categoryBits, fctx->filter.maskBits,
                          fctx->world->shapes.shapeCategory[shape],
                          fctx->world->shapes.shapeMask[shape]))
        {
            return true; // filtered out
        }
    }
    m3OverlapContext* ctx = (m3OverlapContext*)userContext;
    if (ctx->radius >= 0.0f && !SphereReachesShape(ctx->world, shape, ctx->center, ctx->radius))
    {
        return true;
    }
    m3SelectionOffer(&ctx->selection, ctx->world, shape);
    return true;
}

static int32_t OverlapGather(m3World* world, m3OverlapContext* ctx, m3ShapeId* shapes,
                             int32_t capacity)
{
    ctx->selection = (m3ShapeSelection){shapes, capacity, 0};
    m3TreeQuery(&world->broadphase.tree, ctx->lo, ctx->hi, OverlapCallback, ctx);
    for (int32_t k = 0; k < world->shapes.planeCount; ++k)
    {
        int32_t s = world->shapes.planeShapes[k];
        {
            int include;
            if (ctx->radius >= 0.0f)
            {
                include = SphereReachesShape(world, s, ctx->center, ctx->radius);
            }
            else
            {
                // Half space versus box: the box reaches the plane
                // iff its most-negative corner along the normal does.
                m3Transform xfS6 = m3ShapeWorldTransform(world, s);
                const m3Transform* xf = &xfS6;
                m3Vec3 n = m3RotateVec3(xf->q, world->shapes.shapeGeom[s].v); // world normal
                double off = (double)world->shapes.shapeGeom[s].s + (double)n.x * xf->p.x +
                             (double)n.y * xf->p.y + (double)n.z * xf->p.z;
                double minProj = (n.x >= 0.0f ? ctx->lo[0] : ctx->hi[0]) * (double)n.x +
                                 (n.y >= 0.0f ? ctx->lo[1] : ctx->hi[1]) * (double)n.y +
                                 (n.z >= 0.0f ? ctx->lo[2] : ctx->hi[2]) * (double)n.z;
                include = minProj <= off;
            }
            if (include)
            {
                m3SelectionOffer(&ctx->selection, world, s);
            }
        }
    }
    return m3SelectionFinish(&ctx->selection);
}

int32_t m3World_OverlapAabb(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes,
                            int32_t capacity, m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || shapes == NULL || capacity <= 0 || !m3FinitePos3(lo) ||
        !m3FinitePos3(hi) || hi.x < lo.x || hi.y < lo.y || hi.z < lo.z)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    m3OverlapContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.filter = filter; // after the memset: a zeroed filter filters everything
    ctx.world = world;
    ctx.radius = -1.0f;
    ctx.lo[0] = lo.x;
    ctx.lo[1] = lo.y;
    ctx.lo[2] = lo.z;
    ctx.hi[0] = hi.x;
    ctx.hi[1] = hi.y;
    ctx.hi[2] = hi.z;
    return OverlapGather(world, &ctx, shapes, capacity);
}

int32_t m3World_OverlapSphere(m3WorldId worldId, m3Pos3 center, m3real radius, m3ShapeId* shapes,
                              int32_t capacity, m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || shapes == NULL || capacity <= 0 || !m3FiniteF(radius) ||
        !(radius > 0.0f) || !m3FinitePos3(center))
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    m3OverlapContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.filter = filter; // after the memset: a zeroed filter filters everything
    ctx.world = world;
    ctx.center = center;
    ctx.radius = radius;
    ctx.lo[0] = center.x - (double)radius;
    ctx.lo[1] = center.y - (double)radius;
    ctx.lo[2] = center.z - (double)radius;
    ctx.hi[0] = center.x + (double)radius;
    ctx.hi[1] = center.y + (double)radius;
    ctx.hi[2] = center.z + (double)radius;
    return OverlapGather(world, &ctx, shapes, capacity);
}

// --- Contact readback ------------------------------------------------

static void FillContactData(const m3World* world, int32_t pair, m3ContactData* out)
{
    const m3Manifold* manifold = &world->contacts.manifolds[pair];
    uint64_t key = world->contacts.pairKeys[pair];
    int32_t shapeA = (int32_t)(key >> 32);
    int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
    out->shapeIdA =
        (m3ShapeId){shapeA + 1, world->idWorld, world->shapes.shapePool.generations[shapeA]};
    out->shapeIdB =
        (m3ShapeId){shapeB + 1, world->idWorld, world->shapes.shapePool.generations[shapeB]};
    out->normal = manifold->normal;
    int32_t count = manifold->pointCount;
    out->pointCount = count;
    int32_t bodyA = world->shapes.shapeBody[shapeA];
    m3Vec3 rcA = m3RotateVec3(world->bodies.transforms[bodyA].q, world->bodies.localCenters[bodyA]);
    for (int32_t k = 0; k < count; ++k)
    {
        const m3ManifoldPoint* point = &world->contacts.manifolds[pair].points[k];
        // Anchors are measured from body A's center in world axes:
        // COM plus anchor is the world contact point at read time.
        out->points[k] =
            (m3Pos3){world->bodies.transforms[bodyA].p.x + (double)(rcA.x + point->anchorA.x),
                     world->bodies.transforms[bodyA].p.y + (double)(rcA.y + point->anchorA.y),
                     world->bodies.transforms[bodyA].p.z + (double)(rcA.z + point->anchorA.z)};
        out->separations[k] = point->separation;
        out->normalImpulses[k] = point->normalImpulse;
    }
    for (int32_t k = count; k < 4; ++k)
    {
        out->points[k] = (m3Pos3){0.0, 0.0, 0.0};
        out->separations[k] = 0.0f;
        out->normalImpulses[k] = 0.0f;
    }
}

int32_t m3Shape_GetContactData(m3ShapeId shapeId, m3ContactData* out, int32_t capacity)
{
    m3World* world = m3WorldFromTag(shapeId.world);
    if (world == NULL || out == NULL || capacity <= 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    int32_t shape = shapeId.index1 - 1;
    if (shape < 0 || shape >= world->shapes.shapePool.maxIndex ||
        world->shapes.shapePool.alive[shape] == 0 ||
        world->shapes.shapePool.generations[shape] != shapeId.generation)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    int32_t written = 0;
    for (int32_t i = 0; i < world->contacts.pairCount && written < capacity; ++i)
    {
        uint64_t key = world->contacts.pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if ((shapeA == shape || shapeB == shape) && world->contacts.manifolds[i].pointCount > 0)
        {
            FillContactData(world, i, &out[written]);
            written += 1;
        }
    }
    return written;
}

int32_t m3Body_GetContactData(m3BodyId bodyId, m3ContactData* out, int32_t capacity)
{
    m3World* world = m3WorldFromTag(bodyId.world);
    if (world == NULL || out == NULL || capacity <= 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    int32_t body = m3BodySlot(world, bodyId);
    if (body < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    int32_t written = 0;
    for (int32_t i = 0; i < world->contacts.pairCount && written < capacity; ++i)
    {
        uint64_t key = world->contacts.pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if ((world->shapes.shapeBody[shapeA] == body || world->shapes.shapeBody[shapeB] == body) &&
            world->contacts.manifolds[i].pointCount > 0)
        {
            FillContactData(world, i, &out[written]);
            written += 1;
        }
    }
    return written;
}
