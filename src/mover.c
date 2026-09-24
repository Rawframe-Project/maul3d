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

m3RayCastResult m3World_CastMover(m3WorldId worldId, m3Pos3 center, m3real halfHeight,
                                  m3real radius, m3Vec3 translation)
{
    m3RayCastResult miss;
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
        if (left < size && heap[left].shapeId.index1 > heap[largest].shapeId.index1)
        {
            largest = left;
        }
        if (right < size && heap[right].shapeId.index1 > heap[largest].shapeId.index1)
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
        while (i > 0 && ctx->planes[(i - 1) / 2].shapeId.index1 < ctx->planes[i].shapeId.index1)
        {
            m3MoverPlane swap = ctx->planes[i];
            ctx->planes[i] = ctx->planes[(i - 1) / 2];
            ctx->planes[(i - 1) / 2] = swap;
            i = (i - 1) / 2;
        }
    }
    else if (plane.shapeId.index1 < ctx->planes[0].shapeId.index1)
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
    plane.shapeId =
        (m3ShapeId){shape + 1, world->idWorld, world->shapes.shapePool.generations[shape]};
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
    for (int32_t k = 0; k < world->shapes.planeCount; ++k)
    {
        int32_t s = world->shapes.planeShapes[k];
        if (world->shapes.shapeSensor[s] != 0 ||
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
        plane.shapeId = (m3ShapeId){s + 1, world->idWorld, world->shapes.shapePool.generations[s]};
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

// The move is the translation d closest to the wish w that keeps every
// plane clear, n_i . d + s_i >= 0: a small strictly convex problem, so
// its optimum is unique. At the optimum d = w + sum lambda_j n_j over the
// planes it rests on, each with lambda_j >= 0 (a plane pushes, never
// pulls), and in 3D it rests on at most three. The candidate rest sets
// are tried smallest first, in plane order; the first that clears every
// plane with pushes that are not negative is the optimum. Planes that
// contradict each other leave no such point, and the candidate that
// overlaps least is taken instead.

// How far a move may stand inside a plane and still count as clear.
#define M3_MOVER_TOLERANCE 1.0e-5f

typedef struct MoverCandidate
{
    m3Vec3 d;
    m3real clearance; // the smallest n . d + s over all planes
    uint32_t pressed;
} MoverCandidate;

// Solves the k by k Gram system G lambda = r (k up to 3) by Cramer's
// rule; false when it is degenerate.
static bool SolveGram(const m3real g[3][3], const m3real r[3], int32_t k, m3real lambda[3])
{
    if (k == 1)
    {
        lambda[0] = r[0] / g[0][0];
        return g[0][0] > 1.0e-10f;
    }
    if (k == 2)
    {
        m3real det = g[0][0] * g[1][1] - g[0][1] * g[1][0];
        lambda[0] = (g[1][1] * r[0] - g[0][1] * r[1]) / det;
        lambda[1] = (g[0][0] * r[1] - g[1][0] * r[0]) / det;
        return det > 1.0e-10f;
    }
    m3Vec3 c0 = {g[0][0], g[1][0], g[2][0]};
    m3Vec3 c1 = {g[0][1], g[1][1], g[2][1]};
    m3Vec3 c2 = {g[0][2], g[1][2], g[2][2]};
    m3Vec3 rv = {r[0], r[1], r[2]};
    m3real det = m3Dot3(c0, m3Cross3(c1, c2));
    if (!(det > 1.0e-10f))
    {
        return false;
    }
    lambda[0] = m3Dot3(rv, m3Cross3(c1, c2)) / det;
    lambda[1] = m3Dot3(c0, m3Cross3(rv, c2)) / det;
    lambda[2] = m3Dot3(c0, m3Cross3(c1, rv)) / det;
    return true;
}

// The point closest to the wish on the k planes of set, pushed by each.
// False when the set is degenerate or a push would pull.
static bool RestOn(m3Vec3 wish, const m3MoverPlane* planes, const int32_t* set, int32_t k,
                   m3Vec3* d, uint32_t* pressed)
{
    m3real g[3][3];
    m3real r[3];
    m3real lambda[3] = {0.0f, 0.0f, 0.0f};
    for (int32_t i = 0; i < k && i < 3; ++i)
    {
        for (int32_t j = 0; j < k && j < 3; ++j)
        {
            g[i][j] = m3Dot3(planes[set[i]].normal, planes[set[j]].normal);
        }
        r[i] = -(planes[set[i]].separation + m3Dot3(planes[set[i]].normal, wish));
    }
    if (k > 0 && !SolveGram(g, r, k, lambda))
    {
        return false;
    }
    *d = wish;
    *pressed = 0;
    for (int32_t j = 0; j < k && j < 3; ++j)
    {
        if (!(lambda[j] >= 0.0f))
        {
            return false;
        }
        *d = m3Add3(*d, m3MulSV3(lambda[j], planes[set[j]].normal));
        *pressed |= lambda[j] > 0.0f ? 1u << set[j] : 0u;
    }
    return true;
}

// Keeps the first clear candidate, or until one appears, the one that
// overlaps least.
static void Consider(m3Vec3 wish, const m3MoverPlane* planes, int32_t count, const int32_t* set,
                     int32_t k, MoverCandidate* best)
{
    MoverCandidate c;
    if (!RestOn(wish, planes, set, k, &c.d, &c.pressed))
    {
        return;
    }
    c.clearance = FLT_MAX;
    for (int32_t i = 0; i < count; ++i)
    {
        c.clearance = m3MinF(c.clearance, m3Dot3(planes[i].normal, c.d) + planes[i].separation);
    }
    if (best->clearance < -M3_MOVER_TOLERANCE && c.clearance > best->clearance)
    {
        *best = c;
    }
}

m3MoverMove m3SolveMover(m3Vec3 wish, const m3MoverPlane* planes, int32_t count)
{
    m3MoverMove move = {wish, 0};
    if (planes == NULL || count <= 0 || !m3FiniteV3(wish))
    {
        m3Refuse(NULL, m3_errorInvalid);
        return move;
    }
    count = count < M3_MOVER_PLANES ? count : M3_MOVER_PLANES;
    MoverCandidate best = {wish, -FLT_MAX, 0};
    int32_t set[3] = {0, 0, 0};
    Consider(wish, planes, count, set, 0, &best);
    for (set[0] = 0; set[0] < count; ++set[0])
    {
        Consider(wish, planes, count, set, 1, &best);
    }
    for (set[0] = 0; set[0] < count; ++set[0])
    {
        for (set[1] = set[0] + 1; set[1] < count; ++set[1])
        {
            Consider(wish, planes, count, set, 2, &best);
        }
    }
    for (set[0] = 0; set[0] < count; ++set[0])
    {
        for (set[1] = set[0] + 1; set[1] < count; ++set[1])
        {
            for (set[2] = set[1] + 1; set[2] < count; ++set[2])
            {
                Consider(wish, planes, count, set, 3, &best);
            }
        }
    }
    move.translation = best.d;
    move.pressed = best.pressed;
    return move;
}

m3Vec3 m3ClipMoverVelocity(m3Vec3 velocity, const m3MoverPlane* planes, int32_t count,
                           uint32_t pressed)
{
    if (planes == NULL || count <= 0 || pressed == 0)
    {
        return velocity;
    }
    m3MoverPlane touching[M3_MOVER_PLANES];
    int32_t n = 0;
    for (int32_t i = 0; i < count && i < M3_MOVER_PLANES; ++i)
    {
        if ((pressed & (1u << i)) != 0)
        {
            touching[n] = planes[i];
            touching[n].separation = 0.0f;
            n += 1;
        }
    }
    return m3SolveMover(velocity, touching, n).translation;
}
