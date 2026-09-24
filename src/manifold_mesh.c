// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex shapes against triangle soups: the welded mesh pipeline that
// claims shared edges and vertices once, and its heightfield and
// voxel-surface front ends. The per-triangle kernels live in
// triangle_contact.c.

#include "distance.h"
#include "manifold.h"
#include "shape.h"
#include "triangle_contact.h"
#include "voxel.h"
#include "world_internal.h"

#include <float.h>
#include <string.h>

#define M3_MESH_CANDIDATE_CAP 64

// Welding: a convex resting across several triangles of one mesh must
// not catch on the edges between them. Face contacts are trusted and
// claim their triangles' edges and vertices; every other contact, on an
// edge or a vertex, waits, and is kept, nearest first, only when its
// feature is still unclaimed or is a genuinely convex edge of the mesh.

typedef struct Claims
{
    int32_t edges[3 * M3_MESH_CANDIDATE_CAP]; // lo * 65536 + hi
    int32_t edgeCount;
    int32_t verts[3 * M3_MESH_CANDIDATE_CAP];
    int32_t vertCount;
} Claims;

// Claims a key; true when it was not claimed before.
static bool Claim(int32_t* keys, int32_t* count, int32_t key)
{
    for (int32_t i = 0; i < *count; ++i)
    {
        if (keys[i] == key)
        {
            return false;
        }
    }
    if (*count < 3 * M3_MESH_CANDIDATE_CAP)
    {
        keys[(*count)++] = key;
    }
    return true;
}

// Claims a triangle's three edges and vertices and returns a bit per
// feature that was new: 1, 2 and 4 for the edges v0v1, v1v2 and v2v0,
// then 8, 16 and 32 for the vertices.
static int32_t ClaimTriangle(Claims* claims, const m3MeshData* mesh, int32_t t)
{
    const uint16_t* v = &mesh->indices[3 * t];
    int32_t fresh = 0;
    for (int32_t k = 0; k < 3; ++k)
    {
        int32_t a = v[k];
        int32_t b = v[(k + 1) % 3];
        int32_t key = a < b ? a * 65536 + b : b * 65536 + a;
        fresh |= Claim(claims->edges, &claims->edgeCount, key) ? 1 << k : 0;
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        fresh |= Claim(claims->verts, &claims->vertCount, v[k]) ? 8 << k : 0;
    }
    return fresh;
}

// Whether a waiting contact survives, given which of its triangle's
// features were still unclaimed. A convex edge is a real feature and
// always collides; a tilted hull face on a triangle with any convex edge
// is real too (a box teetering on a ridge), and on an all-flat triangle
// only when nothing of the triangle was claimed yet.
static bool WaitingSurvives(int32_t feature, uint8_t convex, int32_t fresh)
{
    switch (feature)
    {
    case 1 | 2:
        return (convex & 1) != 0 || (fresh & 1) != 0;
    case 2 | 4:
        return (convex & 2) != 0 || (fresh & 2) != 0;
    case 1 | 4:
        return (convex & 4) != 0 || (fresh & 4) != 0;
    case 1:
        return (fresh & 8) != 0;
    case 2:
        return (fresh & 16) != 0;
    case 4:
        return (fresh & 32) != 0;
    case M3_TRI_FEATURE_HULL_FACE:
        return convex != 0 || fresh == 63;
    default:
        return false;
    }
}

typedef struct m3MeshCandidate
{
    m3TriManifold local;
    int32_t triIndex;
} m3MeshCandidate;

// The convex shape in the mesh frame, and what the triangle kernels need
// of it.
typedef struct MeshProbe
{
    uint8_t type;
    m3Quat qRel; // convex frame to mesh frame
    m3Vec3 pRel;
    m3Vec3 s1; // sphere center, or capsule core ends
    m3Vec3 s2;
    m3real radius;
    const m3HullData* hull;
    m3Quat qHull; // mesh frame to hull frame
    m3Vec3 pHull;
    m3Vec3 lo; // core bounds
    m3Vec3 hi;
} MeshProbe;

static void GrowBounds(MeshProbe* probe, m3Vec3 p)
{
    probe->lo =
        (m3Vec3){m3MinF(probe->lo.x, p.x), m3MinF(probe->lo.y, p.y), m3MinF(probe->lo.z, p.z)};
    probe->hi =
        (m3Vec3){m3MaxF(probe->hi.x, p.x), m3MaxF(probe->hi.y, p.y), m3MaxF(probe->hi.z, p.z)};
}

static MeshProbe MakeMeshProbe(const m3World* world, const m3Transform* xfM, int32_t otherShape)
{
    MeshProbe probe;
    memset(&probe, 0, sizeof(probe));
    m3Transform xfO = m3ShapeWorldTransform(world, otherShape);
    m3Quat conjM = {-xfM->q.x, -xfM->q.y, -xfM->q.z, xfM->q.w};
    probe.type = world->shapes.shapeType[otherShape];
    probe.qRel = m3MulQuat(conjM, xfO.q);
    m3Vec3 dp = {(m3real)(xfO.p.x - xfM->p.x), (m3real)(xfO.p.y - xfM->p.y),
                 (m3real)(xfO.p.z - xfM->p.z)};
    probe.pRel = m3InvRotateVec3(xfM->q, dp);
    probe.radius = world->shapes.shapeGeom[otherShape].s;
    probe.lo = (m3Vec3){FLT_MAX, FLT_MAX, FLT_MAX};
    probe.hi = (m3Vec3){-FLT_MAX, -FLT_MAX, -FLT_MAX};
    const m3ShapeGeom* g = &world->shapes.shapeGeom[otherShape];
    if (probe.type == (uint8_t)m3_hullShape)
    {
        // The hull kernel runs in the hull's frame: triangles go in,
        // results come back out.
        probe.hull = &world->hulls.hullData[world->shapes.shapeHullIndex[otherShape]];
        probe.qHull = (m3Quat){-probe.qRel.x, -probe.qRel.y, -probe.qRel.z, probe.qRel.w};
        probe.pHull = m3Neg3(m3InvRotateVec3(probe.qRel, probe.pRel));
        probe.radius = 0.0f;
        for (int32_t v = 0; v < probe.hull->vertexCount; ++v)
        {
            GrowBounds(&probe,
                       m3Add3(m3RotateVec3(probe.qRel, probe.hull->vertices[v]), probe.pRel));
        }
        return probe;
    }
    probe.s1 = m3Add3(m3RotateVec3(probe.qRel, g->v), probe.pRel);
    probe.s2 = probe.type == (uint8_t)m3_capsuleShape
                   ? m3Add3(m3RotateVec3(probe.qRel, g->v2), probe.pRel)
                   : probe.s1;
    GrowBounds(&probe, probe.s1);
    GrowBounds(&probe, probe.s2);
    return probe;
}

// One triangle's contact in the mesh frame; false without points.
static bool TriangleContact(const MeshProbe* probe, const m3Vec3 tri[3], m3TriManifold* local)
{
    memset(local, 0, sizeof(*local));
    if (probe->type == (uint8_t)m3_sphereShape)
    {
        m3CollideSphereTriangle(local, probe->s1, probe->radius, tri);
    }
    else if (probe->type == (uint8_t)m3_capsuleShape)
    {
        m3CollideCapsuleTriangle(local, probe->s1, probe->s2, probe->radius, tri);
    }
    else
    {
        m3Vec3 inHull[3];
        for (int32_t k = 0; k < 3; ++k)
        {
            inHull[k] = m3Add3(m3RotateVec3(probe->qHull, tri[k]), probe->pHull);
        }
        m3CollideHullTriangle(local, probe->hull, inHull);
        local->normal = m3RotateVec3(probe->qRel, local->normal);
        local->triNormal = m3RotateVec3(probe->qRel, local->triNormal);
        for (int32_t k = 0; k < local->pointCount; ++k)
        {
            local->point[k] = m3Add3(m3RotateVec3(probe->qRel, local->point[k]), probe->pRel);
        }
    }
    return local->pointCount > 0;
}

// Face contacts are trusted. A hull face resting on a triangle is trusted
// too when its normal agrees with the triangle's or it sinks deep;
// otherwise it waits like an edge or vertex contact.
static bool TrustedContact(const m3TriManifold* local)
{
    if (local->feature == 7)
    {
        return true;
    }
    if (local->feature != M3_TRI_FEATURE_HULL_FACE)
    {
        return false;
    }
    m3real deepest = FLT_MAX;
    for (int32_t k = 0; k < local->pointCount; ++k)
    {
        deepest = m3MinF(deepest, local->separation[k]);
    }
    return m3Dot3(local->triNormal, local->normal) > 0.5f || deepest < -2.0f * 0.005f;
}

// Nearest first, the lower triangle on ties.
static void SortWaiting(m3MeshCandidate* waiting, int32_t count)
{
    for (int32_t a = 0; a < count; ++a)
    {
        int32_t best = a;
        for (int32_t b = a + 1; b < count; ++b)
        {
            const m3TriManifold* lb = &waiting[b].local;
            const m3TriManifold* lbest = &waiting[best].local;
            if (lb->dist2 < lbest->dist2 ||
                (lb->dist2 == lbest->dist2 && waiting[b].triIndex < waiting[best].triIndex))
            {
                best = b;
            }
        }
        m3MeshCandidate swap = waiting[a];
        waiting[a] = waiting[best];
        waiting[best] = swap;
    }
}

// Runs every candidate triangle and welds the result: trusted contacts
// first, then the waiting ones that survive the claims. Returns how many
// contacts were accepted.
static int32_t WeldTriangles(const m3MeshData* mesh, const uint16_t* gather, int32_t gatherCount,
                             const MeshProbe* probe, m3MeshCandidate* accepted)
{
    m3real reach = probe->radius + M3_SPECULATIVE_DISTANCE;
    m3MeshCandidate waiting[M3_MESH_CANDIDATE_CAP];
    int32_t acceptedCount = 0;
    int32_t waitingCount = 0;
    for (int32_t g = 0; g < gatherCount; ++g)
    {
        if (acceptedCount >= M3_MESH_CANDIDATE_CAP || waitingCount >= M3_MESH_CANDIDATE_CAP)
        {
            break;
        }
        int32_t t = gather[g];
        m3Vec3 tri[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                         mesh->vertices[mesh->indices[3 * t + 1]],
                         mesh->vertices[mesh->indices[3 * t + 2]]};
        m3Vec3 tlo = {m3MinF(tri[0].x, m3MinF(tri[1].x, tri[2].x)) - reach,
                      m3MinF(tri[0].y, m3MinF(tri[1].y, tri[2].y)) - reach,
                      m3MinF(tri[0].z, m3MinF(tri[1].z, tri[2].z)) - reach};
        m3Vec3 thi = {m3MaxF(tri[0].x, m3MaxF(tri[1].x, tri[2].x)) + reach,
                      m3MaxF(tri[0].y, m3MaxF(tri[1].y, tri[2].y)) + reach,
                      m3MaxF(tri[0].z, m3MaxF(tri[1].z, tri[2].z)) + reach};
        if (probe->hi.x < tlo.x || probe->lo.x > thi.x || probe->hi.y < tlo.y ||
            probe->lo.y > thi.y || probe->hi.z < tlo.z || probe->lo.z > thi.z)
        {
            continue;
        }
        m3MeshCandidate cand;
        cand.triIndex = t;
        if (!TriangleContact(probe, tri, &cand.local))
        {
            continue;
        }
        if (TrustedContact(&cand.local))
        {
            accepted[acceptedCount++] = cand;
        }
        else
        {
            waiting[waitingCount++] = cand;
        }
    }
    Claims claims;
    claims.edgeCount = 0;
    claims.vertCount = 0;
    for (int32_t k = 0; k < acceptedCount; ++k)
    {
        (void)ClaimTriangle(&claims, mesh, accepted[k].triIndex);
    }
    SortWaiting(waiting, waitingCount);
    for (int32_t k = 0; k < waitingCount && acceptedCount < M3_MESH_CANDIDATE_CAP; ++k)
    {
        int32_t t = waiting[k].triIndex;
        int32_t fresh = ClaimTriangle(&claims, mesh, t);
        if (WaitingSurvives(waiting[k].local.feature, mesh->edgeFlags[t], fresh))
        {
            accepted[acceptedCount++] = waiting[k];
        }
    }
    return acceptedCount;
}

// Emits one manifold: the accepted contacts whose normals agree with the
// deepest one, reduced to four points spread over the patch, in ascending
// id. Each midway point splits into both anchors along the normal.
static void EmitMeshManifold(m3World* world, m3Manifold* fresh, const m3MeshData* mesh,
                             const m3MeshCandidate* accepted, int32_t count, const m3Transform* xfM,
                             int32_t meshShape, int32_t otherShape, int meshIsA)
{
    int32_t rep = 0;
    m3real deepest = FLT_MAX;
    for (int32_t k = 0; k < count; ++k)
    {
        for (int32_t p = 0; p < accepted[k].local.pointCount; ++p)
        {
            m3real sep = accepted[k].local.separation[p];
            if (sep < deepest || (sep == deepest && accepted[k].triIndex < accepted[rep].triIndex))
            {
                deepest = sep;
                rep = k;
            }
        }
    }
    m3Vec3 normal = accepted[rep].local.normal;
    enum
    {
        POINT_CAP = 2 * M3_MESH_CANDIDATE_CAP
    };
    m3Vec3 point[POINT_CAP];
    m3real sep[POINT_CAP];
    uint16_t id[POINT_CAP];
    uint16_t material[POINT_CAP];
    int32_t n = 0;
    for (int32_t k = 0; k < count && n < POINT_CAP; ++k)
    {
        const m3TriManifold* local = &accepted[k].local;
        for (int32_t p = 0;
             p < local->pointCount && n < POINT_CAP && m3Dot3(local->normal, normal) >= 0.99f; ++p)
        {
            point[n] = local->point[p];
            sep[n] = local->separation[p];
            id[n] = (uint16_t)((accepted[k].triIndex << 2) | local->localId[p]);
            // The material group rides the point flags; a material-free
            // mesh writes zeros.
            material[n] = mesh->materialCount > 0
                              ? (uint16_t)(mesh->triMaterials[accepted[k].triIndex] << 12)
                              : 0;
            n += 1;
        }
    }
    int32_t kept[M3_MANIFOLD_MAX_POINTS];
    int32_t keptCount = m3ReduceContactPoints(point, sep, n, normal, kept);
    for (int32_t a = 0; a < keptCount; ++a)
    {
        for (int32_t b = a + 1; b < keptCount; ++b)
        {
            if (id[kept[b]] < id[kept[a]])
            {
                int32_t swap = kept[a];
                kept[a] = kept[b];
                kept[b] = swap;
            }
        }
    }
    int32_t meshBody = world->shapes.shapeBody[meshShape];
    int32_t otherBody = world->shapes.shapeBody[otherShape];
    m3Vec3 nWorld = m3RotateVec3(xfM->q, normal); // mesh toward shape
    fresh->normal = meshIsA ? nWorld : m3Neg3(nWorld);
    fresh->pointCount = keptCount;
    for (int32_t k = 0; k < keptCount; ++k)
    {
        int32_t c = kept[k];
        m3Vec3 pw = m3RotateVec3(xfM->q, point[c]);
        double px = xfM->p.x + (double)pw.x;
        double py = xfM->p.y + (double)pw.y;
        double pz = xfM->p.z + (double)pw.z;
        m3real half = 0.5f * sep[c];
        m3Vec3 onMesh =
            m3AnchorFromCom(world, meshBody, px - (double)(nWorld.x * half),
                            py - (double)(nWorld.y * half), pz - (double)(nWorld.z * half));
        m3Vec3 onOther =
            m3AnchorFromCom(world, otherBody, px + (double)(nWorld.x * half),
                            py + (double)(nWorld.y * half), pz + (double)(nWorld.z * half));
        fresh->points[k].anchorA = meshIsA ? onMesh : onOther;
        fresh->points[k].anchorB = meshIsA ? onOther : onMesh;
        fresh->points[k].separation = sep[c];
        fresh->points[k].id = id[c];
        fresh->points[k].flags = material[c];
    }
}

// The welded pipeline, mesh-agnostic: the mesh and its BVH arrive as
// parameters so the heightfield can feed a scratch window mesh through
// the same flow. A NULL bvh means every triangle is a candidate (the
// window is pre-clipped). The BVH prunes in ascending triangle order, so
// the accepted sequence matches a full scan.
static void CollideMeshCore(m3World* world, m3Manifold* fresh, const m3MeshData* mesh,
                            const m3MeshBvh* bvh, int32_t meshShape, int32_t otherShape,
                            int meshIsA)
{
    m3Transform xfM = m3ShapeWorldTransform(world, meshShape);
    MeshProbe probe = MakeMeshProbe(world, &xfM, otherShape);
    m3real reach = probe.radius + M3_SPECULATIVE_DISTANCE;
    uint16_t gather[M3_MESH_MAX_TRIS];
    int32_t gatherCount = mesh->triangleCount;
    if (bvh != NULL)
    {
        m3Vec3 lo = {probe.lo.x - reach, probe.lo.y - reach, probe.lo.z - reach};
        m3Vec3 hi = {probe.hi.x + reach, probe.hi.y + reach, probe.hi.z + reach};
        gatherCount = m3MeshBvhGather(bvh, lo, hi, gather);
    }
    else
    {
        for (int32_t t = 0; t < gatherCount; ++t)
        {
            gather[t] = (uint16_t)t;
        }
    }
    m3MeshCandidate accepted[M3_MESH_CANDIDATE_CAP];
    int32_t count = WeldTriangles(mesh, gather, gatherCount, &probe, accepted);
    if (count > 0)
    {
        EmitMeshManifold(world, fresh, mesh, accepted, count, &xfM, meshShape, otherShape, meshIsA);
    }
}

void m3CollideMeshConvex(m3World* world, m3Manifold* fresh, int32_t meshShape, int32_t otherShape,
                         int meshIsA)
{
    int32_t meshIndex = world->shapes.shapeMeshIndex[meshShape];
    CollideMeshCore(world, fresh, &world->meshes.meshData[meshIndex],
                    &world->meshes.meshBvh[meshIndex], meshShape, otherShape, meshIsA);
}

// Native heightfield versus convex: clip the convex's reach
// to a cell window (a one-cell halo keeps the interior edge flags
// correct at the window rim), lay the window out as a scratch mesh
// in the heightfield frame, bake its edge flags, and run the SAME
// welded pipeline. Window ids are window-local, so the warm carry
// resets when the window shifts a cell: deterministic, documented.
#define M3_HF_WINDOW 16 // cells per axis, halo included

// The height field cells [cx0, cx1] x [cz0, cz1] as a small mesh on the
// step scratch, split by the diagonal parity rule, its edge flags baked.
// False on a scratch stall.
static bool HeightFieldWindow(m3World* world, const m3HeightFieldData* hf, int32_t cx0, int32_t cz0,
                              int32_t cx1, int32_t cz1, m3MeshData* window)
{
    int32_t wx = cx1 - cx0 + 2; // window corners per axis
    int32_t wz = cz1 - cz0 + 2;
    int32_t vertCount = wx * wz;
    int32_t triCount = 2 * (wx - 1) * (wz - 1);

    m3Vec3* verts = (m3Vec3*)m3StackAlloc(&world->scratch, vertCount * (int32_t)sizeof(m3Vec3));
    uint16_t* tris =
        (uint16_t*)m3StackAlloc(&world->scratch, 3 * triCount * (int32_t)sizeof(uint16_t));
    uint8_t* flags = (uint8_t*)m3StackAlloc(&world->scratch, triCount);
    uint8_t* mats = (uint8_t*)m3StackAlloc(&world->scratch, triCount);
    if (verts == NULL || tris == NULL || flags == NULL || mats == NULL)
    {
        return false;
    }
    for (int32_t z = 0; z < wz; ++z)
    {
        for (int32_t x = 0; x < wx; ++x)
        {
            int32_t gx = cx0 + x;
            int32_t gz = cz0 + z;
            verts[z * wx + x] = (m3Vec3){(m3real)gx * hf->cellSize, hf->heights[gz * hf->nx + gx],
                                         (m3real)gz * hf->cellSize};
        }
    }
    int32_t tw = 0;
    for (int32_t z = 0; z + 1 < wz; ++z)
    {
        for (int32_t x = 0; x + 1 < wx; ++x)
        {
            uint16_t a = (uint16_t)(z * wx + x);
            uint16_t bIdx = (uint16_t)(z * wx + x + 1);
            uint16_t c = (uint16_t)((z + 1) * wx + x + 1);
            uint16_t d = (uint16_t)((z + 1) * wx + x);
            if (((cx0 + x) + (cz0 + z)) % 2 == 0)
            {
                tris[tw++] = a;
                tris[tw++] = c;
                tris[tw++] = bIdx;
                tris[tw++] = a;
                tris[tw++] = d;
                tris[tw++] = c;
            }
            else
            {
                tris[tw++] = bIdx;
                tris[tw++] = a;
                tris[tw++] = d;
                tris[tw++] = bIdx;
                tris[tw++] = d;
                tris[tw++] = c;
            }
        }
    }
    memset(window, 0, sizeof(*window));
    window->vertexCount = vertCount;
    window->triangleCount = triCount;
    window->vertices = verts;
    window->indices = tris;
    window->edgeFlags = flags;
    window->triMaterials = mats; // zeros: no painted terrain (yet)
    int32_t* bake = (int32_t*)m3StackAlloc(&world->scratch, m3MeshEdgeScratchCount(window) *
                                                                (int32_t)sizeof(int32_t));
    if (bake == NULL)
    {
        return false;
    }
    m3BakeMeshEdgeFlags(window, bake);
    return true;
}

void m3CollideHeightFieldConvex(m3World* world, m3Manifold* fresh, int32_t hfShape,
                                int32_t otherShape, int hfIsA)
{
    const m3HeightFieldData* hf = &world->heightFields.hfData[world->shapes.shapeHfIndex[hfShape]];
    m3Transform xfHv = m3ShapeWorldTransform(world, hfShape);
    m3Transform xfOv = m3ShapeWorldTransform(world, otherShape);
    m3Quat conjH = {-xfHv.q.x, -xfHv.q.y, -xfHv.q.z, xfHv.q.w};
    m3Quat qRel = m3MulQuat(conjH, xfOv.q);
    m3Vec3 dp = {(m3real)(xfOv.p.x - xfHv.p.x), (m3real)(xfOv.p.y - xfHv.p.y),
                 (m3real)(xfOv.p.z - xfHv.p.z)};
    m3Vec3 pRel = m3InvRotateVec3(xfHv.q, dp);

    // The convex's bounds in the heightfield frame (the core's own
    // recipe, repeated here only to pick the window).
    uint8_t otherType = world->shapes.shapeType[otherShape];
    m3real radius = world->shapes.shapeGeom[otherShape].s;
    m3Vec3 boundLo;
    m3Vec3 boundHi;
    if (otherType == (uint8_t)m3_hullShape)
    {
        const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[otherShape]];
        boundLo = (m3Vec3){3.4e38f, 3.4e38f, 3.4e38f};
        boundHi = (m3Vec3){-3.4e38f, -3.4e38f, -3.4e38f};
        for (int32_t v = 0; v < hull->vertexCount; ++v)
        {
            m3Vec3 pt = m3Add3(m3RotateVec3(qRel, hull->vertices[v]), pRel);
            boundLo.x = m3MinF(boundLo.x, pt.x);
            boundLo.y = m3MinF(boundLo.y, pt.y);
            boundLo.z = m3MinF(boundLo.z, pt.z);
            boundHi.x = m3MaxF(boundHi.x, pt.x);
            boundHi.y = m3MaxF(boundHi.y, pt.y);
            boundHi.z = m3MaxF(boundHi.z, pt.z);
        }
        radius = 0.0f;
    }
    else if (otherType == (uint8_t)m3_sphereShape)
    {
        m3Vec3 c = m3Add3(m3RotateVec3(qRel, world->shapes.shapeGeom[otherShape].v), pRel);
        boundLo = c;
        boundHi = c;
    }
    else
    {
        m3Vec3 c1 = m3Add3(m3RotateVec3(qRel, world->shapes.shapeGeom[otherShape].v), pRel);
        m3Vec3 c2 = m3Add3(m3RotateVec3(qRel, world->shapes.shapeGeom[otherShape].v2), pRel);
        boundLo.x = m3MinF(c1.x, c2.x);
        boundLo.y = m3MinF(c1.y, c2.y);
        boundLo.z = m3MinF(c1.z, c2.z);
        boundHi.x = m3MaxF(c1.x, c2.x);
        boundHi.y = m3MaxF(c1.y, c2.y);
        boundHi.z = m3MaxF(c1.z, c2.z);
    }
    m3real reach = radius + M3_SPECULATIVE_DISTANCE;

    m3real inv = 1.0f / hf->cellSize;
    int32_t cx0 = m3CellFromF(floorf((boundLo.x - reach) * inv), 2.0e9f) - 1; // the halo cell
    int32_t cx1 = m3CellFromF(floorf((boundHi.x + reach) * inv), -2.0e9f) + 1;
    int32_t cz0 = m3CellFromF(floorf((boundLo.z - reach) * inv), 2.0e9f) - 1;
    int32_t cz1 = m3CellFromF(floorf((boundHi.z + reach) * inv), -2.0e9f) + 1;
    cx0 = cx0 < 0 ? 0 : cx0;
    cz0 = cz0 < 0 ? 0 : cz0;
    cx1 = cx1 > hf->nx - 2 ? hf->nx - 2 : cx1;
    cz1 = cz1 > hf->nz - 2 ? hf->nz - 2 : cz1;
    if (cx1 < cx0 || cz1 < cz0)
    {
        return; // fully off the grid
    }
    if (cx1 - cx0 + 1 > M3_HF_WINDOW)
    {
        cx1 = cx0 + M3_HF_WINDOW - 1; // the documented window bound
    }
    if (cz1 - cz0 + 1 > M3_HF_WINDOW)
    {
        cz1 = cz0 + M3_HF_WINDOW - 1;
    }
    m3MeshData window;
    if (!HeightFieldWindow(world, hf, cx0, cz0, cx1, cz1, &window))
    {
        return; // transient scratch stall, grown next step
    }
    CollideMeshCore(world, fresh, &window, NULL, hfShape, otherShape, hfIsA);
}

// Voxel chunk versus convex: the surface BVH gathers merged
// boxes in ascending order; each candidate runs the family's exact
// kernel in the CHUNK frame (boxes are axis-aligned there by
// construction, so the sphere case is an exact clamp); the deepest
// candidate wins the manifold (ties to the lower box index via the
// ascending scan). Cross-box point merging and seam welding are not
// done; a flat floor merges into one box, so resting contacts
// get full manifolds today. Feature ids mix the box index so warm
// starts follow their box across rebuilds.
void m3CollideVoxelConvex(m3World* world, m3Manifold* fresh, int32_t voxelShape, int32_t otherShape,
                          int voxelIsA)
{
    int32_t slot = world->shapes.shapeVoxelIndex[voxelShape];
    const m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
    const m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
    m3real cell = chunk->cellSize;
    int32_t voxelBody = world->shapes.shapeBody[voxelShape];
    int32_t otherBody = world->shapes.shapeBody[otherShape];
    m3Transform xfVv = m3ShapeWorldTransform(world, voxelShape);
    m3Transform xfOv = m3ShapeWorldTransform(world, otherShape);
    const m3Transform* xfV = &xfVv;
    const m3Transform* xfO = &xfOv;
    uint8_t otherType = world->shapes.shapeType[otherShape];

    m3Quat conjV = {-xfV->q.x, -xfV->q.y, -xfV->q.z, xfV->q.w};
    m3Quat qRel = m3MulQuat(conjV, xfO->q);
    m3Vec3 dp = {(m3real)(xfO->p.x - xfV->p.x), (m3real)(xfO->p.y - xfV->p.y),
                 (m3real)(xfO->p.z - xfV->p.z)};
    m3Vec3 pRel = m3InvRotateVec3(xfV->q, dp);

    m3real radius = world->shapes.shapeGeom[otherShape].s;
    m3Vec3 s1 = {0.0f, 0.0f, 0.0f};
    m3Vec3 s2 = {0.0f, 0.0f, 0.0f};
    m3Vec3 boundLo;
    m3Vec3 boundHi;
    m3HullData otherHullLocal;
    const m3HullData* otherHull = NULL;
    if (otherType == (uint8_t)m3_hullShape)
    {
        otherHull = &world->hulls.hullData[world->shapes.shapeHullIndex[otherShape]];
        boundLo = (m3Vec3){3.4e38f, 3.4e38f, 3.4e38f};
        boundHi = (m3Vec3){-3.4e38f, -3.4e38f, -3.4e38f};
        for (int32_t v = 0; v < otherHull->vertexCount; ++v)
        {
            m3Vec3 p = m3Add3(m3RotateVec3(qRel, otherHull->vertices[v]), pRel);
            boundLo.x = m3MinF(boundLo.x, p.x);
            boundLo.y = m3MinF(boundLo.y, p.y);
            boundLo.z = m3MinF(boundLo.z, p.z);
            boundHi.x = m3MaxF(boundHi.x, p.x);
            boundHi.y = m3MaxF(boundHi.y, p.y);
            boundHi.z = m3MaxF(boundHi.z, p.z);
        }
        radius = 0.0f;
        (void)otherHullLocal;
    }
    else if (otherType == (uint8_t)m3_sphereShape)
    {
        s1 = m3Add3(m3RotateVec3(qRel, world->shapes.shapeGeom[otherShape].v), pRel);
        boundLo = s1;
        boundHi = s1;
    }
    else
    {
        s1 = m3Add3(m3RotateVec3(qRel, world->shapes.shapeGeom[otherShape].v), pRel);
        s2 = m3Add3(m3RotateVec3(qRel, world->shapes.shapeGeom[otherShape].v2), pRel);
        boundLo.x = m3MinF(s1.x, s2.x);
        boundLo.y = m3MinF(s1.y, s2.y);
        boundLo.z = m3MinF(s1.z, s2.z);
        boundHi.x = m3MaxF(s1.x, s2.x);
        boundHi.y = m3MaxF(s1.y, s2.y);
        boundHi.z = m3MaxF(s1.z, s2.z);
    }
    m3real reach = radius + M3_SPECULATIVE_DISTANCE;

    // Interior depenetration: when the OTHER shape's center
    // is inside the solid, the surface candidates are meaningless
    // (every nearby face is interior). A grid BFS names the nearest
    // exposed face; one synthetic contact walks the body out at a
    // depth clamped to two cells per step, so the soft solver's
    // pushout stays gentle by construction: recovery, never launch.
    {
        m3Vec3 center;
        if (otherType == (uint8_t)m3_hullShape)
        {
            center = m3Add3(m3RotateVec3(qRel, otherHull->unitCom), pRel);
        }
        else if (otherType == (uint8_t)m3_sphereShape)
        {
            center = s1;
        }
        else
        {
            center = m3MulSV3(0.5f, m3Add3(s1, s2));
        }
        m3Vec3 escapeNormal;
        m3real escapePlane;
        if (m3VoxelEscape(world, slot, center, &escapeNormal, &escapePlane))
        {
            m3real along = m3Dot3(escapeNormal, center);
            m3real plane = escapePlane *
                           (escapeNormal.x + escapeNormal.y + escapeNormal.z > 0.0f ? 1.0f : -1.0f);
            m3real depth = plane - along; // distance from center to the
                                          // exit plane along the normal
            m3real maxStep = 2.0f * cell;
            depth = m3MinF(depth, maxStep);
            m3Manifold escape;
            memset(&escape, 0, sizeof(escape));
            escape.normal = escapeNormal;
            escape.pointCount = 1;
            escape.points[0].anchorA = m3Add3(center, m3MulSV3(depth, escapeNormal));
            escape.points[0].anchorB = center;
            escape.points[0].separation = -(depth + radius);
            escape.points[0].id = 0x7FFE; // the reserved escape feature
            fresh->normal = m3RotateVec3(xfV->q, escape.normal);
            fresh->pointCount = 1;
            m3Vec3 rA = m3RotateVec3(xfV->q, escape.points[0].anchorA);
            m3Vec3 rB = m3RotateVec3(xfV->q, escape.points[0].anchorB);
            fresh->points[0] = escape.points[0];
            fresh->points[0].anchorA =
                m3AnchorFromCom(world, voxelBody, xfV->p.x + (double)rA.x, xfV->p.y + (double)rA.y,
                                xfV->p.z + (double)rA.z);
            fresh->points[0].anchorB =
                m3AnchorFromCom(world, otherBody, xfV->p.x + (double)rB.x, xfV->p.y + (double)rB.y,
                                xfV->p.z + (double)rB.z);
            if (!voxelIsA)
            {
                fresh->normal = m3Neg3(fresh->normal);
                m3Vec3 tmp = fresh->points[0].anchorA;
                fresh->points[0].anchorA = fresh->points[0].anchorB;
                fresh->points[0].anchorB = tmp;
            }
            return;
        }
    }

    uint16_t gather[M3_MESH_MAX_TRIS];
    int32_t gatherCount = m3MeshBvhGather(
        &surface->bvh, (m3Vec3){boundLo.x - reach, boundLo.y - reach, boundLo.z - reach},
        (m3Vec3){boundHi.x + reach, boundHi.y + reach, boundHi.z + reach}, gather);

    m3Manifold best;
    memset(&best, 0, sizeof(best));
    m3real bestScore = 3.4e38f;
    for (int32_t g = 0; g < gatherCount; ++g)
    {
        int32_t box = gather[g];
        m3Manifold local;
        memset(&local, 0, sizeof(local));
        // Seam welding: a covered face is interior geometry.
        // Extending it one chunk length outward models the solid
        // continuing through the seam, so the clamp and the SATs can
        // only ever answer with exposed features. No ghost normals.
        m3Vec3 boxLo;
        m3Vec3 boxHi;
        m3VoxelBoxBounds(surface, cell, box, &boxLo, &boxHi);
        {
            m3real ext = (m3real)M3_VOXEL_DIM * cell;
            uint8_t covered = surface->boxCovered[box];
            if ((covered & 1u) != 0)
            {
                boxLo.x -= ext;
            }
            if ((covered & 2u) != 0)
            {
                boxHi.x += ext;
            }
            if ((covered & 4u) != 0)
            {
                boxLo.y -= ext;
            }
            if ((covered & 8u) != 0)
            {
                boxHi.y += ext;
            }
            if ((covered & 16u) != 0)
            {
                boxLo.z -= ext;
            }
            if ((covered & 32u) != 0)
            {
                boxHi.z += ext;
            }
        }
        if (otherType == (uint8_t)m3_sphereShape)
        {
            m3Vec3 lo = boxLo;
            m3Vec3 hi = boxHi;
            m3Vec3 closest = {m3ClampF(s1.x, lo.x, hi.x), m3ClampF(s1.y, lo.y, hi.y),
                              m3ClampF(s1.z, lo.z, hi.z)};
            m3Vec3 d = m3Sub3(s1, closest);
            m3real d2 = m3Dot3(d, d);
            if (d2 > 0.0f)
            {
                m3real dist = sqrtf(d2);
                m3real sep = dist - radius;
                if (sep <= M3_SPECULATIVE_DISTANCE)
                {
                    m3Vec3 n = m3MulSV3(1.0f / dist, d);
                    local.normal = n;
                    local.pointCount = 1;
                    local.points[0].anchorA = closest;
                    local.points[0].anchorB = m3Sub3(s1, m3MulSV3(radius, n));
                    local.points[0].separation = sep;
                    local.points[0].id = 0;
                }
            }
            else
            {
                // Center inside the box: the least-deep face is the
                // exact minimum translation (axis-aligned, so it is
                // a six-way comparison, not an iteration).
                m3real depth[6] = {s1.x - lo.x, hi.x - s1.x, s1.y - lo.y,
                                   hi.y - s1.y, s1.z - lo.z, hi.z - s1.z};
                int32_t face = 0;
                for (int32_t k = 1; k < 6; ++k)
                {
                    if (depth[k] < depth[face])
                    {
                        face = k;
                    }
                }
                static const m3Vec3 outward[6] = {{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
                                                  {0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                                                  {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}};
                m3Vec3 n = outward[face];
                local.normal = n;
                local.pointCount = 1;
                local.points[0].anchorA = m3Add3(s1, m3MulSV3(depth[face], n));
                local.points[0].anchorB = m3Sub3(s1, m3MulSV3(radius, n));
                local.points[0].separation = -depth[face] - radius;
                local.points[0].id = 0;
            }
        }
        else
        {
            m3HullData boxHull;
            m3VoxelBoundsHull(boxLo, boxHi, &boxHull);
            if (otherType == (uint8_t)m3_capsuleShape)
            {
                local = m3CollideSegmentHull(&boxHull, s1, s2, radius);
            }
            else
            {
                local = m3CollideHulls(&boxHull, otherHull, qRel, pRel);
            }
        }
        if (local.pointCount == 0)
        {
            continue;
        }
        m3real score = 3.4e38f;
        for (int32_t k = 0; k < local.pointCount; ++k)
        {
            score = m3MinF(score, local.points[k].separation);
            // Mix the box index into the feature id so a warm start
            // follows its box, never a neighbor's.
            local.points[k].id = (uint16_t)(local.points[k].id ^ (uint16_t)(box * 0x9E3u));
        }
        if (score < bestScore)
        {
            bestScore = score;
            best = local;
        }
    }

    if (best.pointCount == 0)
    {
        return;
    }
    // Rebase from the chunk frame to world COM anchors (the hull-hull
    // convention: anchors arrive as chunk-frame positions).
    fresh->normal = m3RotateVec3(xfV->q, best.normal);
    fresh->pointCount = best.pointCount;
    for (int32_t k = 0; k < best.pointCount; ++k)
    {
        m3Vec3 rA = m3RotateVec3(xfV->q, best.points[k].anchorA);
        m3Vec3 rB = m3RotateVec3(xfV->q, best.points[k].anchorB);
        fresh->points[k] = best.points[k];
        fresh->points[k].anchorA =
            m3AnchorFromCom(world, voxelBody, xfV->p.x + (double)rA.x, xfV->p.y + (double)rA.y,
                            xfV->p.z + (double)rA.z);
        fresh->points[k].anchorB =
            m3AnchorFromCom(world, otherBody, xfV->p.x + (double)rB.x, xfV->p.y + (double)rB.y,
                            xfV->p.z + (double)rB.z);
    }
    if (!voxelIsA)
    {
        fresh->normal = m3Neg3(fresh->normal);
        for (int32_t k = 0; k < fresh->pointCount; ++k)
        {
            m3Vec3 tmp = fresh->points[k].anchorA;
            fresh->points[k].anchorA = fresh->points[k].anchorB;
            fresh->points[k].anchorB = tmp;
        }
    }
}
