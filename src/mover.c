// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mover kit: casting a character capsule, gathering its collision
// planes and solving a translation against them.

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

// --- The mover toolkit -----------------------------------------------
//
// Pure queries plus a pure solver: nothing here journals, mutates,
// or hashes. Hosts compose them into their own movers; the engine
// keeps its kinematic controller as the built-in path.

m3RayHit m3World_CastMover(m3WorldId worldId, m3Pos3 center, m3real halfHeight, m3real radius,
                           m3Vec3 translation)
{
    m3RayHit miss;
    memset(&miss, 0, sizeof(miss));
    miss.fraction = 1.0f;
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FinitePos3(center) || !m3FiniteF(halfHeight) || halfHeight < 0.0f ||
        !m3FiniteF(radius) || !(radius > 0.0f) || !m3FiniteV3(translation))
    {
        m3Refuse(world, m3_errorInvalid);
        return miss;
    }
    m3Vec3 points[2] = {{0.0f, halfHeight, 0.0f}, {0.0f, -halfHeight, 0.0f}};
    return m3CastConvexClosestEx(world, center, points, 2, radius, translation, -1);
}

typedef struct m3MoverCollideCtx
{
    m3World* world;
    m3Pos3 center;
    m3real halfHeight;
    m3real radius;
    m3real skin;
    m3MoverPlane* planes; // bounded max-heap on the shape slot while collecting
    int32_t capacity;
    int32_t count;
} m3MoverCollideCtx;

static void MoverPlaneSiftDown(m3MoverPlane* heap, int32_t size, int32_t i)
{
    for (;;)
    {
        int32_t largest = i;
        int32_t left = 2 * i + 1;
        int32_t right = left + 1;
        if (left < size && heap[left].shape.index1 > heap[largest].shape.index1)
        {
            largest = left;
        }
        if (right < size && heap[right].shape.index1 > heap[largest].shape.index1)
        {
            largest = right;
        }
        if (largest == i)
        {
            return;
        }
        m3MoverPlane swap = heap[i];
        heap[i] = heap[largest];
        heap[largest] = swap;
        i = largest;
    }
}

// Keeps the planes of the `capacity` lowest shape slots offered.
static void MoverOfferPlane(m3MoverCollideCtx* ctx, m3MoverPlane plane)
{
    if (ctx->count < ctx->capacity)
    {
        int32_t i = ctx->count++;
        ctx->planes[i] = plane;
        while (i > 0 && ctx->planes[(i - 1) / 2].shape.index1 < ctx->planes[i].shape.index1)
        {
            m3MoverPlane swap = ctx->planes[i];
            ctx->planes[i] = ctx->planes[(i - 1) / 2];
            ctx->planes[(i - 1) / 2] = swap;
            i = (i - 1) / 2;
        }
    }
    else if (plane.shape.index1 < ctx->planes[0].shape.index1)
    {
        ctx->planes[0] = plane;
        MoverPlaneSiftDown(ctx->planes, ctx->count, 0);
    }
}

static bool MoverGatherCallback(int32_t shape, void* userContext)
{
    m3MoverCollideCtx* ctx = (m3MoverCollideCtx*)userContext;
    m3World* world = ctx->world;
    if (world->shapes.shapeSensor[shape] != 0 ||
        world->bodies.bodyEnabled[world->shapes.shapeBody[shape]] == 0)
    {
        return true; // sensors and disabled bodies are invisible
    }
    m3Transform xfS = m3ShapeWorldTransform(world, shape);
    m3Pos3 center = ctx->center;
    m3Vec3 local =
        m3InvRotateVec3(xfS.q, (m3Vec3){(m3real)(center.x - xfS.p.x), (m3real)(center.y - xfS.p.y),
                                        (m3real)(center.z - xfS.p.z)});
    m3Vec3 axis = m3InvRotateVec3(xfS.q, (m3Vec3){0.0f, 1.0f, 0.0f});
    m3Vec3 caps[2] = {m3Add3(local, m3MulSV3(ctx->halfHeight, axis)),
                      m3Sub3(local, m3MulSV3(ctx->halfHeight, axis))};
    m3Vec3 scratch[2];
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA = m3MakeShapeProxy(world, shape, scratch);
    input.proxyB.points = caps;
    input.proxyB.count = 2;
    input.proxyB.radius = 0.0f;
    input.q = m3MakeIdentityQuat();
    input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.useRadii = false;
    m3DistanceOutput out = m3ShapeDistance(&input);
    m3real gap = out.distance - input.proxyA.radius - ctx->radius;
    if (gap > ctx->skin)
    {
        return true;
    }
    m3MoverPlane plane;
    if (out.distance > 1.0e-6f)
    {
        plane.normal = m3RotateVec3(xfS.q, out.normal); // shape toward mover
    }
    else
    {
        // Deep overlap: GJK gives no direction; push up (the
        // deterministic fallback a grounded mover wants).
        plane.normal = (m3Vec3){0.0f, 1.0f, 0.0f};
    }
    plane.separation = gap;
    plane.shape =
        (m3ShapeId){shape + 1, world->worldIndex0, world->shapes.shapePool.generations[shape]};
    MoverOfferPlane(ctx, plane);
    return true;
}

int32_t m3World_CollideMover(m3WorldId worldId, m3Pos3 center, m3real halfHeight, m3real radius,
                             m3real skin, m3MoverPlane* planes, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || planes == NULL || capacity <= 0 || !m3FinitePos3(center) ||
        !m3FiniteF(halfHeight) || halfHeight < 0.0f || !m3FiniteF(radius) || !(radius > 0.0f) ||
        !m3FiniteF(skin) || skin < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    m3MoverCollideCtx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.world = world;
    ctx.center = center;
    ctx.halfHeight = halfHeight;
    ctx.radius = radius;
    ctx.skin = skin;
    ctx.planes = planes;
    ctx.capacity = capacity;
    double reach = (double)(halfHeight + radius + skin);
    double lo[3] = {center.x - reach, center.y - reach, center.z - reach};
    double hi[3] = {center.x + reach, center.y + reach, center.z + reach};
    m3TreeQuery(&world->broadphase.tree, lo, hi, MoverGatherCallback, &ctx);
    // The infinite planes never enter the tree: test them directly.
    int32_t maxShape = world->shapes.shapePool.maxIndex;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapes.shapePool.alive[s] == 0 ||
            world->shapes.shapeType[s] != (uint8_t)m3_planeShape ||
            world->shapes.shapeSensor[s] != 0 ||
            world->bodies.bodyEnabled[world->shapes.shapeBody[s]] == 0)
        {
            continue;
        }
        m3Vec3 n = world->shapes.shapeGeom[s].v;
        m3real off = world->shapes.shapeGeom[s].s;
        // Closest capsule feature to the half-space.
        m3real dCenter =
            (m3real)((double)n.x * center.x + (double)n.y * center.y + (double)n.z * center.z) -
            off;
        m3real gap = dCenter - halfHeight * (n.y > 0.0f ? n.y : -n.y) - radius;
        if (gap > skin)
        {
            continue;
        }
        m3MoverPlane plane;
        plane.normal = n;
        plane.separation = gap;
        plane.shape =
            (m3ShapeId){s + 1, world->worldIndex0, world->shapes.shapePool.generations[s]};
        MoverOfferPlane(&ctx, plane);
    }
    // Ascending shape order keeps the plane list canonical.
    for (int32_t end = ctx.count - 1; end > 0; --end)
    {
        m3MoverPlane swap = planes[0];
        planes[0] = planes[end];
        planes[end] = swap;
        MoverPlaneSiftDown(planes, end, 0);
    }
    return ctx.count;
}

m3Vec3 m3SolvePlanes(m3Vec3 translation, const m3MoverPlane* planes, int32_t count,
                     int32_t iterations)
{
    if (planes == NULL || count <= 0 || iterations <= 0 || !m3FiniteV3(translation))
    {
        m3Refuse(NULL, m3_errorInvalid);
        return translation;
    }
    // The reference accumulator: per-plane nonnegative pushes,
    // Gauss-Seidel over a fixed order.
    m3real push[64];
    int32_t n = count < 64 ? count : 64;
    for (int32_t i = 0; i < n; ++i)
    {
        push[i] = 0.0f;
    }
    m3Vec3 d = translation;
    for (int32_t it = 0; it < iterations; ++it)
    {
        for (int32_t i = 0; i < n; ++i)
        {
            m3real violation = planes[i].separation + m3Dot3(planes[i].normal, d);
            m3real want = -violation;
            m3real old = push[i];
            m3real next = old + want > 0.0f ? old + want : 0.0f;
            m3real applied = next - old;
            push[i] = next;
            d = m3Add3(d, m3MulSV3(applied, planes[i].normal));
        }
    }
    return d;
}
