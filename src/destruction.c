// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Explosions: an impulse on every body a blast reaches, scaled by the
// area a shape shows the blast and its distance from the center.

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

#define M3_EXPLOSION_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3ExplosionDef) << 8) ^ 13))

// Projected area of a convex shape onto a plane facing `direction`
// (a unit vector in the shape's local frame): a blast's impulse scales
// with the area the shape shows to the front.
static m3real ShapeProjectedArea(const m3World* world, int32_t shape, m3Vec3 direction)
{
    uint8_t type = world->shapes.shapeType[shape];
    const m3ShapeGeom* geom = &world->shapes.shapeGeom[shape];
    if (type == (uint8_t)m3_sphereShape)
    {
        return M3_PI * geom->s * geom->s;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        m3Vec3 axis = m3Sub3(geom->v2, geom->v);
        m3real projected = m3Length3(m3Cross3(axis, direction));
        return M3_PI * geom->s * geom->s + 2.0f * geom->s * projected;
    }
    if (type == (uint8_t)m3_hullShape)
    {
        // Fan every face from its first vertex and keep the facing
        // triangles; half the summed cross products is the area.
        const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[shape]];
        m3real area = 0.0f;
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            int32_t start = (int32_t)hull->faceVertStart[f];
            int32_t count = (int32_t)hull->faceVertCounts[f];
            m3Vec3 p1 = hull->vertices[hull->faceIndices[start]];
            for (int32_t k = 2; k < count; ++k)
            {
                m3Vec3 p2 = hull->vertices[hull->faceIndices[start + k - 1]];
                m3Vec3 p3 = hull->vertices[hull->faceIndices[start + k]];
                m3real a = m3Dot3(m3Cross3(m3Sub3(p2, p1), m3Sub3(p3, p1)), direction);
                area += a > 0.0f ? a : 0.0f;
            }
        }
        return 0.5f * area;
    }
    return 0.0f;
}

// The shape's own stable interior point, local frame: the fallback
// direction anchor when the blast center sits inside the shape.
static m3Vec3 ShapeLocalCentroid(const m3World* world, int32_t shape)
{
    uint8_t type = world->shapes.shapeType[shape];
    const m3ShapeGeom* geom = &world->shapes.shapeGeom[shape];
    if (type == (uint8_t)m3_capsuleShape)
    {
        return m3MulSV3(0.5f, m3Add3(geom->v, geom->v2));
    }
    if (type == (uint8_t)m3_hullShape)
    {
        return world->hulls.hullData[world->shapes.shapeHullIndex[shape]].center;
    }
    return geom->v; // sphere center
}

typedef struct m3ExplodeContext
{
    m3World* world;
    const m3ExplosionDef* def;
} m3ExplodeContext;

static bool ExplodeCallback(int32_t shape, void* userContext)
{
    m3ExplodeContext* ctx = (m3ExplodeContext*)userContext;
    m3World* world = ctx->world;
    const m3ExplosionDef* def = ctx->def;
    uint8_t type = world->shapes.shapeType[shape];
    if (type == (uint8_t)m3_voxelShape)
    {
        // The carve couples the blast to destruction: one
        // call bites the chunk, the fracture sweep frees islands,
        // and the fragment events carry the rest to the host.
        if (def->voxelCarve > 0.0f)
        {
            m3Transform vxf = m3ShapeWorldTransform(world, shape);
            m3Vec3 vlocal = m3InvRotateVec3(vxf.q, (m3Vec3){(m3real)(def->position.x - vxf.p.x),
                                                            (m3real)(def->position.y - vxf.p.y),
                                                            (m3real)(def->position.z - vxf.p.z)});
            m3VoxelCarveSphereInternal(world, shape, vlocal, def->voxelCarve);
        }
        return true;
    }
    if (type != (uint8_t)m3_sphereShape && type != (uint8_t)m3_capsuleShape &&
        type != (uint8_t)m3_hullShape)
    {
        return true; // meshes and planes are static scenery
    }
    int32_t body = world->shapes.shapeBody[shape];
    if (world->bodies.types[body] != (uint8_t)m3_dynamicBody ||
        world->bodies.bodyEnabled[body] == 0 || world->bodies.invMass[body] <= 0.0f)
    {
        return true;
    }
    if (!m3FilterPass(def->filter.categoryBits, def->filter.maskBits,
                      world->shapes.shapeCategory[shape], world->shapes.shapeMask[shape]))
    {
        return true;
    }
    // Work in the shape's local frame so distance and direction stay
    // precise far from the origin.
    m3Transform xf = m3ShapeWorldTransform(world, shape);
    m3Vec3 local = m3InvRotateVec3(xf.q, (m3Vec3){(m3real)(def->position.x - xf.p.x),
                                                  (m3real)(def->position.y - xf.p.y),
                                                  (m3real)(def->position.z - xf.p.z)});
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, shape, scratch);
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
    m3real surface = out.distance - proxy.radius;
    if (surface > def->radius + def->falloff)
    {
        return true;
    }
    // The blast wakes everything it reaches, sleepers included.
    m3SetAwakeInternal(world, body, 1);
    m3Vec3 direction;
    m3Vec3 contact;
    if (out.distance > 1e-6f && surface > 1e-6f)
    {
        direction = m3Normalize3(m3Sub3(out.pointA, point));
        contact = m3Sub3(out.pointA, m3MulSV3(proxy.radius, direction));
    }
    else
    {
        // The center sits inside the shape: push through the
        // centroid, a fixed axis when even
        // that is degenerate.
        m3Vec3 centroid = ShapeLocalCentroid(world, shape);
        m3Vec3 d = m3Sub3(centroid, point);
        direction = m3Dot3(d, d) > 1e-10f ? m3Normalize3(d) : (m3Vec3){1.0f, 0.0f, 0.0f};
        contact = centroid;
    }
    m3real scale = 1.0f;
    if (surface > def->radius && def->falloff > 0.0f)
    {
        scale = (def->radius + def->falloff - surface) / def->falloff;
        scale = scale < 0.0f ? 0.0f : (scale > 1.0f ? 1.0f : scale);
    }
    m3real magnitude = def->impulsePerArea * ShapeProjectedArea(world, shape, direction) * scale;
    if (magnitude != 0.0f)
    {
        m3Vec3 impulse = m3MulSV3(magnitude, m3RotateVec3(xf.q, direction));
        m3Vec3 arm = m3RotateVec3(xf.q, contact);
        m3Pos3 at = {xf.p.x + (double)arm.x, xf.p.y + (double)arm.y, xf.p.z + (double)arm.z};
        m3ApplyImpulseAtPointInternal(world, body, impulse, at);
    }
    return true;
}

bool m3WorldExplodeInternal(m3World* world, const m3ExplosionDef* def)
{
    if (!m3FinitePos3(def->position) || !m3FiniteF(def->radius) || def->radius < 0.0f ||
        !m3FiniteF(def->falloff) || def->falloff < 0.0f || !m3FiniteF(def->impulsePerArea) ||
        !m3FiniteF(def->voxelCarve) || def->voxelCarve < 0.0f || !m3FiniteF(def->softPush))
    {
        return false;
    }
    double reach = (double)def->radius + (double)def->falloff;
    double extent = reach > (double)def->voxelCarve ? reach : (double)def->voxelCarve;
    double lo[3] = {def->position.x - extent, def->position.y - extent, def->position.z - extent};
    double hi[3] = {def->position.x + extent, def->position.y + extent, def->position.z + extent};
    m3ExplodeContext ctx = {world, def};
    m3TreeQuery(&world->broadphase.tree, lo, hi, ExplodeCallback, &ctx);
    // Soft particles: a canonical linear pass over the pool. Verlet
    // has no velocity to poke, so the push lands as a pending kick
    // the next step integrates exactly once.
    if (def->softPush != 0.0f && def->impulsePerArea != 0.0f)
    {
        int32_t maxSoft = world->softBodies.softPool.maxIndex;
        for (int32_t slot = 0; slot < maxSoft; ++slot)
        {
            if (world->softBodies.softPool.alive[slot] == 0)
            {
                continue;
            }
            int32_t count = world->softBodies.softParticleCount[slot];
            int32_t base = slot * M3_SOFTBODY_MAX_PARTICLES;
            m3real pr = world->softBodies.softRadius[slot];
            m3real area = M3_PI * pr * pr;
            for (int32_t i = 0; i < count; ++i)
            {
                int32_t k = base + i;
                if (world->softBodies.softInvMass[k] == 0.0f)
                {
                    continue;
                }
                m3Vec3 d = {(m3real)(world->softBodies.softPos[k].x - def->position.x),
                            (m3real)(world->softBodies.softPos[k].y - def->position.y),
                            (m3real)(world->softBodies.softPos[k].z - def->position.z)};
                m3real dist = m3Length3(d);
                m3real surface = dist - pr;
                if (surface > def->radius + def->falloff)
                {
                    continue;
                }
                m3Vec3 direction =
                    dist > 1e-6f ? m3MulSV3(1.0f / dist, d) : (m3Vec3){1.0f, 0.0f, 0.0f};
                m3real scale = 1.0f;
                if (surface > def->radius && def->falloff > 0.0f)
                {
                    scale = (def->radius + def->falloff - surface) / def->falloff;
                    scale = scale < 0.0f ? 0.0f : (scale > 1.0f ? 1.0f : scale);
                }
                m3real magnitude = def->impulsePerArea * def->softPush * area * scale;
                world->softBodies.softKick[k] =
                    m3Add3(world->softBodies.softKick[k],
                           m3MulSV3(magnitude * world->softBodies.softInvMass[k], direction));
            }
        }
    }
    return true;
}

m3ExplosionDef m3DefaultExplosionDef(void)
{
    m3ExplosionDef def;
    memset(&def, 0, sizeof(def));
    def.filter = m3DefaultQueryFilter();
    def.radius = 10.0f;
    def.falloff = 5.0f;
    def.impulsePerArea = 0.0f;
    def.voxelCarve = 0.0f;
    def.softPush = 1.0f;
    def.internalValue = M3_EXPLOSION_COOKIE;
    return def;
}

void m3World_Explode(m3WorldId worldId, const m3ExplosionDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_EXPLOSION_COOKIE)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (!m3WorldExplodeInternal(world, def))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // hostile fields apply nothing and journal nothing
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opWorldExplode, def, (int32_t)sizeof(*def));
    }
}
