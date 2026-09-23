// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex hulls of point clouds by QuickHull (Barber, Dobkin and
// Huhdanpaa). A tetrahedron of extreme points starts the hull; every
// other point waits on the face it lies farthest above. Each step takes
// the farthest waiting point of all, removes the faces it sees, and
// joins the horizon of that region to the point with new triangles,
// handing the removed faces' points to the new ones. At most
// M3_HULL_MAX_VERTS vertices are taken, farthest first, so a larger
// cloud gets the hull of its most extreme points.
//
// The triangles then merge into polygon faces: neighbors whose far
// vertices lie within a tolerance of each other's planes, or above them,
// become one face, and a vertex left between just two faces, on the
// line where they meet, is dropped. Mass properties at unit density come
// from the tetrahedra the faces make with one vertex, through the
// covariance of the canonical tetrahedron.

#include "quickhull.h"

#include "allocator.h"
#include "hull.h"

#include <float.h>
#include <string.h>

#define M3_QH_MAX_FACES 1024 // triangle slots, reused as faces die

typedef struct QhFace
{
    int16_t v[3];        // counterclockwise seen from outside
    int16_t adjacent[3]; // the face across edge v[i] -> v[i + 1]
    int16_t conflict;    // first waiting point, or -1
    uint8_t alive;
    uint8_t visible;
    m3Vec3 normal;
    m3real offset;
} QhFace;

typedef struct QhBuilder
{
    m3Vec3 p[M3_HULL_MAX_INPUT]; // input shifted to the first point
    int16_t nextConflict[M3_HULL_MAX_INPUT];
    uint8_t onHull[M3_HULL_MAX_INPUT];
    QhFace faces[M3_QH_MAX_FACES];
    int16_t freeFaces[M3_QH_MAX_FACES];
    int32_t freeCount;
    int32_t faceHigh; // slots ever used
    int32_t hullVertexCount;
    m3real tolerance;
    // Scratch for one step.
    int16_t stack[M3_QH_MAX_FACES];
    int16_t visible[M3_QH_MAX_FACES];
    int16_t horizonA[M3_QH_MAX_FACES];
    int16_t horizonB[M3_QH_MAX_FACES];
    int16_t horizonOut[M3_QH_MAX_FACES];
    int16_t newFaces[M3_QH_MAX_FACES];
    // Merging.
    int16_t group[M3_QH_MAX_FACES];
} QhBuilder;

static m3real Above(const QhFace* f, m3Vec3 point)
{
    return m3Dot3(f->normal, point) - f->offset;
}

static int32_t TakeFace(QhBuilder* b)
{
    if (b->freeCount > 0)
    {
        return b->freeFaces[--b->freeCount];
    }
    return b->faceHigh < M3_QH_MAX_FACES ? b->faceHigh++ : -1;
}

// A triangle face through three hull vertices; false when degenerate.
static bool SetTriangle(QhBuilder* b, int32_t f, int32_t v0, int32_t v1, int32_t v2)
{
    QhFace* face = &b->faces[f];
    face->v[0] = (int16_t)v0;
    face->v[1] = (int16_t)v1;
    face->v[2] = (int16_t)v2;
    face->adjacent[0] = face->adjacent[1] = face->adjacent[2] = -1;
    face->conflict = -1;
    face->alive = 1;
    face->visible = 0;
    m3Vec3 n = m3Cross3(m3Sub3(b->p[v1], b->p[v0]), m3Sub3(b->p[v2], b->p[v0]));
    m3real length = m3Length3(n);
    if (!(length > 0.0f))
    {
        return false;
    }
    face->normal = m3MulSV3(1.0f / length, n);
    face->offset = m3Dot3(face->normal, b->p[v0]);
    return true;
}

// Links the two faces sharing the undirected edge a-b, if both hold it.
static void LinkEdge(QhBuilder* b, int32_t f, int32_t g)
{
    QhFace* F = &b->faces[f];
    QhFace* G = &b->faces[g];
    for (int32_t i = 0; i < 3; ++i)
    {
        for (int32_t j = 0; j < 3; ++j)
        {
            if (F->v[i] == G->v[(j + 1) % 3] && F->v[(i + 1) % 3] == G->v[j])
            {
                F->adjacent[i] = (int16_t)g;
                G->adjacent[j] = (int16_t)f;
            }
        }
    }
}

// The widest-spread axis gives the first two points, the farthest from
// their line the third, the farthest from their plane the fourth.
static bool FindSimplex(const QhBuilder* b, int32_t count, int32_t out[4])
{
    int32_t lo[3] = {0, 0, 0};
    int32_t hi[3] = {0, 0, 0};
    out[2] = 0;
    out[3] = 0;
    for (int32_t i = 1; i < count; ++i)
    {
        for (int32_t k = 0; k < 3; ++k)
        {
            m3real c = k == 0 ? b->p[i].x : (k == 1 ? b->p[i].y : b->p[i].z);
            m3real cl = k == 0 ? b->p[lo[k]].x : (k == 1 ? b->p[lo[k]].y : b->p[lo[k]].z);
            m3real ch = k == 0 ? b->p[hi[k]].x : (k == 1 ? b->p[hi[k]].y : b->p[hi[k]].z);
            lo[k] = c < cl ? i : lo[k];
            hi[k] = c > ch ? i : hi[k];
        }
    }
    int32_t axis = 0;
    m3real spread = -1.0f;
    for (int32_t k = 0; k < 3; ++k)
    {
        m3real d = m3Length3(m3Sub3(b->p[hi[k]], b->p[lo[k]]));
        if (d > spread)
        {
            spread = d;
            axis = k;
        }
    }
    if (spread <= b->tolerance)
    {
        return false;
    }
    out[0] = lo[axis];
    out[1] = hi[axis];
    m3Vec3 dir = m3Sub3(b->p[out[1]], b->p[out[0]]);
    m3real best = -1.0f;
    for (int32_t i = 0; i < count; ++i)
    {
        m3real d = m3Length3(m3Cross3(m3Sub3(b->p[i], b->p[out[0]]), dir)) / spread;
        if (d > best)
        {
            best = d;
            out[2] = i;
        }
    }
    if (best <= b->tolerance)
    {
        return false;
    }
    m3Vec3 n = m3Cross3(dir, m3Sub3(b->p[out[2]], b->p[out[0]]));
    n = m3MulSV3(1.0f / m3Length3(n), n);
    best = -1.0f;
    for (int32_t i = 0; i < count; ++i)
    {
        m3real d = m3Dot3(n, m3Sub3(b->p[i], b->p[out[0]]));
        d = d < 0.0f ? -d : d;
        if (d > best)
        {
            best = d;
            out[3] = i;
        }
    }
    return best > b->tolerance;
}

// Hands a point to the face it lies farthest above, beyond the
// tolerance; a point above no face is inside and drops out.
static void AssignPoint(QhBuilder* b, int32_t point, const int16_t* faces, int32_t faceCount)
{
    int32_t bestFace = -1;
    m3real best = b->tolerance;
    for (int32_t i = 0; i < faceCount; ++i)
    {
        m3real d = Above(&b->faces[faces[i]], b->p[point]);
        if (d > best)
        {
            best = d;
            bestFace = faces[i];
        }
    }
    if (bestFace >= 0)
    {
        b->nextConflict[point] = b->faces[bestFace].conflict;
        b->faces[bestFace].conflict = (int16_t)point;
    }
}

static bool StartHull(QhBuilder* b, int32_t count)
{
    int32_t s[4];
    if (!FindSimplex(b, count, s))
    {
        return false;
    }
    // Wind the base so the apex lies below it.
    m3Vec3 n = m3Cross3(m3Sub3(b->p[s[1]], b->p[s[0]]), m3Sub3(b->p[s[2]], b->p[s[0]]));
    if (m3Dot3(n, m3Sub3(b->p[s[3]], b->p[s[0]])) > 0.0f)
    {
        int32_t t = s[1];
        s[1] = s[2];
        s[2] = t;
    }
    const int32_t tris[4][3] = {
        {s[0], s[1], s[2]}, {s[0], s[3], s[1]}, {s[1], s[3], s[2]}, {s[2], s[3], s[0]}};
    int16_t faces[4];
    for (int32_t f = 0; f < 4; ++f)
    {
        faces[f] = (int16_t)TakeFace(b);
        if (faces[f] < 0 || !SetTriangle(b, faces[f], tris[f][0], tris[f][1], tris[f][2]))
        {
            return false;
        }
    }
    for (int32_t f = 0; f < 4; ++f)
    {
        for (int32_t g = f + 1; g < 4; ++g)
        {
            LinkEdge(b, faces[f], faces[g]);
        }
    }
    for (int32_t k = 0; k < 4; ++k)
    {
        b->onHull[s[k]] = 1;
    }
    b->hullVertexCount = 4;
    for (int32_t i = 0; i < count; ++i)
    {
        if (b->onHull[i] == 0)
        {
            AssignPoint(b, i, faces, 4);
        }
    }
    return true;
}

// The waiting point farthest above its face, the lowest face and then
// the first-listed point on ties; -1 when none waits.
static int32_t FarthestWaiting(const QhBuilder* b, int32_t* faceOut)
{
    int32_t best = -1;
    m3real bestDist = -FLT_MAX;
    for (int32_t f = 0; f < b->faceHigh; ++f)
    {
        const QhFace* face = &b->faces[f];
        for (int32_t q = face->alive ? face->conflict : -1; q >= 0; q = b->nextConflict[q])
        {
            m3real d = Above(face, b->p[q]);
            if (d > bestDist)
            {
                bestDist = d;
                best = q;
                *faceOut = f;
            }
        }
    }
    return best;
}

// The faces the apex sees, grown from seed across edges, and the
// horizon: every edge from a visible face to one it does not see.
static int32_t FindHorizon(QhBuilder* b, int32_t apex, int32_t seed, int32_t* visibleCount)
{
    int32_t top = 0;
    int32_t nv = 0;
    int32_t nh = 0;
    b->faces[seed].visible = 1;
    b->stack[top++] = (int16_t)seed;
    while (top > 0)
    {
        int32_t f = b->stack[--top];
        b->visible[nv++] = (int16_t)f;
        for (int32_t i = 0; i < 3; ++i)
        {
            int32_t g = b->faces[f].adjacent[i];
            if (g < 0 || b->faces[g].visible != 0)
            {
                continue;
            }
            if (Above(&b->faces[g], b->p[apex]) > b->tolerance)
            {
                b->faces[g].visible = 1;
                b->stack[top++] = (int16_t)g;
            }
            else
            {
                b->horizonA[nh] = b->faces[f].v[i];
                b->horizonB[nh] = b->faces[f].v[(i + 1) % 3];
                b->horizonOut[nh] = (int16_t)g;
                nh += 1;
            }
        }
    }
    *visibleCount = nv;
    return nh;
}

// Adds apex to the hull: the faces it sees give way to a cone of new
// triangles from the horizon to it, and their waiting points move to the
// new faces. False when the bookkeeping cannot close the cone.
static bool AddPoint(QhBuilder* b, int32_t apex, int32_t seed)
{
    int32_t nv = 0;
    int32_t nh = FindHorizon(b, apex, seed, &nv);
    int32_t made = 0;
    for (int32_t h = 0; h < nh; ++h)
    {
        int32_t f = TakeFace(b);
        if (f < 0 || !SetTriangle(b, f, b->horizonA[h], b->horizonB[h], apex))
        {
            return false;
        }
        b->newFaces[made++] = (int16_t)f;
        LinkEdge(b, f, b->horizonOut[h]);
    }
    for (int32_t i = 0; i < made; ++i)
    {
        for (int32_t j = i + 1; j < made; ++j)
        {
            LinkEdge(b, b->newFaces[i], b->newFaces[j]);
        }
    }
    for (int32_t i = 0; i < made; ++i)
    {
        const QhFace* face = &b->faces[b->newFaces[i]];
        if (face->adjacent[0] < 0 || face->adjacent[1] < 0 || face->adjacent[2] < 0)
        {
            return false; // the horizon was not one loop
        }
    }
    b->onHull[apex] = 1;
    b->hullVertexCount += 1;
    for (int32_t i = 0; i < nv; ++i)
    {
        QhFace* face = &b->faces[b->visible[i]];
        int32_t q = face->conflict;
        face->alive = 0;
        face->conflict = -1;
        b->freeFaces[b->freeCount++] = b->visible[i];
        while (q >= 0)
        {
            int32_t next = b->nextConflict[q];
            if (q != apex)
            {
                AssignPoint(b, q, b->newFaces, made);
            }
            q = next;
        }
    }
    return true;
}

static int32_t FindGroup(QhBuilder* b, int32_t f)
{
    while (b->group[f] != f)
    {
        b->group[f] = b->group[b->group[f]];
        f = b->group[f];
    }
    return f;
}

// Merges neighbors that are not clearly convex across their shared
// edge: the far vertex of either lies above, or within the tolerance of,
// the other's plane. Groups keep their lowest face as root.
static void MergeFaces(QhBuilder* b)
{
    m3real tol = 2.0f * b->tolerance;
    for (int32_t f = 0; f < b->faceHigh; ++f)
    {
        b->group[f] = (int16_t)f;
    }
    for (int32_t f = 0; f < b->faceHigh; ++f)
    {
        const QhFace* F = &b->faces[f];
        for (int32_t i = 0; F->alive && i < 3; ++i)
        {
            int32_t g = F->adjacent[i];
            const QhFace* G = &b->faces[g];
            int32_t farF = F->v[(i + 2) % 3];
            int32_t farG = -1;
            for (int32_t k = 0; k < 3; ++k)
            {
                farG = G->v[k] != F->v[i] && G->v[k] != F->v[(i + 1) % 3] ? G->v[k] : farG;
            }
            if (farG >= 0 && (Above(F, b->p[farG]) > -tol || Above(G, b->p[farF]) > -tol))
            {
                int32_t rf = FindGroup(b, f);
                int32_t rg = FindGroup(b, g);
                b->group[rf > rg ? rf : rg] = (int16_t)(rf < rg ? rf : rg);
            }
        }
    }
}

typedef struct QhPolygons
{
    int32_t faceCount;
    int32_t indexCount;
    int16_t root[M3_HULL_MAX_FACES];
    uint16_t start[M3_HULL_MAX_FACES];
    uint8_t count[M3_HULL_MAX_FACES];
    int16_t loop[M3_HULL_MAX_FACE_INDICES]; // input point indices
} QhPolygons;

// Chains the boundary of each merged group into one counterclockwise
// loop: every triangle edge whose neighbor lies in another group, each
// followed by the boundary edge that starts where it ends.
static bool TraceLoops(QhBuilder* b, QhPolygons* out)
{
    out->faceCount = 0;
    out->indexCount = 0;
    for (int32_t r = 0; r < b->faceHigh; ++r)
    {
        if (!b->faces[r].alive || FindGroup(b, r) != r)
        {
            continue;
        }
        int16_t from[M3_HULL_MAX_FACE_INDICES];
        int16_t to[M3_HULL_MAX_FACE_INDICES];
        int32_t n = 0;
        for (int32_t f = r; f < b->faceHigh; ++f)
        {
            const QhFace* F = &b->faces[f];
            for (int32_t i = 0; F->alive && FindGroup(b, f) == r && i < 3; ++i)
            {
                if (FindGroup(b, F->adjacent[i]) != r)
                {
                    if (n == M3_HULL_MAX_FACE_INDICES)
                    {
                        return false;
                    }
                    from[n] = F->v[i];
                    to[n] = F->v[(i + 1) % 3];
                    n += 1;
                }
            }
        }
        if (out->faceCount == M3_HULL_MAX_FACES || out->indexCount + n > M3_HULL_MAX_FACE_INDICES)
        {
            return false;
        }
        int32_t face = out->faceCount++;
        out->root[face] = (int16_t)r;
        out->start[face] = (uint16_t)out->indexCount;
        out->count[face] = (uint8_t)n;
        int32_t at = 0;
        for (int32_t k = 0; k < n; ++k)
        {
            out->loop[out->indexCount++] = from[at];
            int32_t next = -1;
            for (int32_t e = 0; e < n && next < 0; ++e)
            {
                next = from[e] == to[at] ? e : -1;
            }
            if (next < 0)
            {
                return false; // the group's boundary is not one loop
            }
            at = next;
        }
        if (at != 0)
        {
            return false;
        }
    }
    return true;
}

// Drops every vertex that only two faces share: it sits on the line
// where they meet and adds nothing to either.
static void DropSeamVertices(QhPolygons* poly, int32_t pointCount)
{
    uint8_t degree[M3_HULL_MAX_INPUT];
    memset(degree, 0, sizeof(degree));
    for (int32_t i = 0; i < poly->indexCount; ++i)
    {
        degree[poly->loop[i]] += 1;
    }
    int32_t write = 0;
    for (int32_t f = 0; f < poly->faceCount; ++f)
    {
        int32_t start = write;
        for (int32_t k = 0; k < poly->count[f]; ++k)
        {
            int32_t v = poly->loop[poly->start[f] + k];
            if (v < pointCount && degree[v] > 2)
            {
                poly->loop[write++] = (int16_t)v;
            }
        }
        poly->start[f] = (uint16_t)start;
        poly->count[f] = (uint8_t)(write - start);
    }
    poly->indexCount = write;
}

// Unit-density volume, centroid and inertia about the centroid. Each
// face fans into tetrahedra with the first hull vertex; a tetrahedron
// o, a, b, c with det = a . (b x c) has volume det / 6, centroid (a + b +
// c) / 4 and covariance det / 120 (a a' + b b' + c c' + s s') for s = a +
// b + c, the covariance of the canonical tetrahedron mapped onto it.
static bool MassProperties(m3HullData* hull)
{
    m3Vec3 o = hull->vertices[0];
    m3real det6 = 0.0f;
    m3Vec3 first = {0.0f, 0.0f, 0.0f};
    m3real c[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // xx yy zz xy xz yz
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        const uint8_t* loop = &hull->faceIndices[hull->faceVertStart[f]];
        m3Vec3 a = m3Sub3(hull->vertices[loop[0]], o);
        for (int32_t k = 1; k + 1 < hull->faceVertCounts[f]; ++k)
        {
            m3Vec3 bv = m3Sub3(hull->vertices[loop[k]], o);
            m3Vec3 cv = m3Sub3(hull->vertices[loop[k + 1]], o);
            m3real det = m3Dot3(a, m3Cross3(bv, cv));
            m3Vec3 s = m3Add3(a, m3Add3(bv, cv));
            det6 += det;
            first = m3Add3(first, m3MulSV3(det, s));
            const m3Vec3 terms[4] = {a, bv, cv, s};
            for (int32_t t = 0; t < 4; ++t)
            {
                m3Vec3 u = terms[t];
                c[0] += det * u.x * u.x;
                c[1] += det * u.y * u.y;
                c[2] += det * u.z * u.z;
                c[3] += det * u.x * u.y;
                c[4] += det * u.x * u.z;
                c[5] += det * u.y * u.z;
            }
        }
    }
    if (!(det6 > 0.0f))
    {
        return false;
    }
    m3real mass = det6 / 6.0f;
    m3Vec3 com = m3MulSV3(0.25f / det6, first);
    for (int32_t k = 0; k < 6; ++k)
    {
        c[k] /= 120.0f;
    }
    // The inertia is trace(C) E - C about o, then shifted to the centroid.
    m3real cc = m3Dot3(com, com);
    m3Mat3 inertia;
    inertia.cx = (m3Vec3){c[1] + c[2] - mass * (cc - com.x * com.x), -c[3] + mass * com.x * com.y,
                          -c[4] + mass * com.x * com.z};
    inertia.cy = (m3Vec3){-c[3] + mass * com.x * com.y, c[0] + c[2] - mass * (cc - com.y * com.y),
                          -c[5] + mass * com.y * com.z};
    inertia.cz = (m3Vec3){-c[4] + mass * com.x * com.z, -c[5] + mass * com.y * com.z,
                          c[0] + c[1] - mass * (cc - com.z * com.z)};
    hull->unitMass = mass;
    hull->unitCom = m3Add3(com, o);
    hull->unitInertiaCom = inertia;
    return true;
}

// Writes the hull: vertices in input order, each face's plane from
// Newell's method over its loop, pushed out to its farthest vertex so
// every vertex lies on or below every plane.
static bool EmitHull(const QhBuilder* b, const QhPolygons* poly, int32_t count, m3Vec3 origin,
                     m3HullData* out)
{
    int16_t index[M3_HULL_MAX_INPUT];
    memset(out, 0, sizeof(*out));
    int32_t vertexCount = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        index[i] = -1;
    }
    for (int32_t i = 0; i < poly->indexCount; ++i)
    {
        index[poly->loop[i]] = 0;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        if (index[i] == 0)
        {
            if (vertexCount == M3_HULL_MAX_VERTS)
            {
                return false;
            }
            index[i] = (int16_t)vertexCount;
            out->vertices[vertexCount++] = m3Add3(b->p[i], origin);
        }
    }
    int32_t edges = 0;
    for (int32_t f = 0; f < poly->faceCount; ++f)
    {
        int32_t n = poly->count[f];
        if (n < 3)
        {
            return false;
        }
        m3Vec3 normal = {0.0f, 0.0f, 0.0f};
        for (int32_t k = 0; k < n; ++k)
        {
            m3Vec3 p = b->p[poly->loop[poly->start[f] + k]];
            m3Vec3 q = b->p[poly->loop[poly->start[f] + (k + 1) % n]];
            normal.x += (p.y - q.y) * (p.z + q.z);
            normal.y += (p.z - q.z) * (p.x + q.x);
            normal.z += (p.x - q.x) * (p.y + q.y);
        }
        m3real length = m3Length3(normal);
        if (!(length > 0.0f))
        {
            return false;
        }
        normal = m3MulSV3(1.0f / length, normal);
        m3real offset = -FLT_MAX;
        for (int32_t k = 0; k < n; ++k)
        {
            offset = m3MaxF(offset, m3Dot3(normal, b->p[poly->loop[poly->start[f] + k]]));
            out->faceIndices[poly->start[f] + k] = (uint8_t)index[poly->loop[poly->start[f] + k]];
        }
        out->faceNormals[f] = normal;
        out->faceOffsets[f] = offset + m3Dot3(normal, origin);
        out->faceVertCounts[f] = (uint8_t)n;
        out->faceVertStart[f] = poly->start[f];
        edges += n;
    }
    out->vertexCount = vertexCount;
    out->faceCount = poly->faceCount;
    out->indexCount = poly->indexCount;
    // Euler's formula for a closed convex surface.
    return vertexCount - edges / 2 + poly->faceCount == 2 && poly->faceCount >= 4;
}

// Coordinates past 1e18 or non-finite are refused, never built.
static bool InputValid(const m3Vec3* points, int32_t count)
{
    if (points == NULL || count < 4 || count > M3_HULL_MAX_INPUT)
    {
        return false;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        m3Vec3 p = points[i];
        if (!m3FiniteV3(p) || p.x > 1.0e18f || p.x < -1.0e18f || p.y > 1.0e18f || p.y < -1.0e18f ||
            p.z > 1.0e18f || p.z < -1.0e18f)
        {
            return false;
        }
    }
    return true;
}

static bool BuildHull(QhBuilder* b, const m3Vec3* points, int32_t count, m3HullData* out)
{
    // Work relative to the first point; the tolerance scales with the
    // cloud's extent.
    m3Vec3 origin = points[0];
    m3Vec3 extent = {0.0f, 0.0f, 0.0f};
    for (int32_t i = 0; i < count; ++i)
    {
        b->p[i] = m3Sub3(points[i], origin);
        b->nextConflict[i] = -1;
        extent.x = m3MaxF(extent.x, b->p[i].x < 0.0f ? -b->p[i].x : b->p[i].x);
        extent.y = m3MaxF(extent.y, b->p[i].y < 0.0f ? -b->p[i].y : b->p[i].y);
        extent.z = m3MaxF(extent.z, b->p[i].z < 0.0f ? -b->p[i].z : b->p[i].z);
    }
    b->tolerance = 16.0f * FLT_EPSILON * (extent.x + extent.y + extent.z);
    if (!StartHull(b, count))
    {
        return false;
    }
    int32_t seed = -1;
    for (int32_t apex = FarthestWaiting(b, &seed);
         apex >= 0 && b->hullVertexCount < M3_HULL_MAX_VERTS; apex = FarthestWaiting(b, &seed))
    {
        if (!AddPoint(b, apex, seed))
        {
            return false;
        }
    }
    MergeFaces(b);
    QhPolygons* poly = (QhPolygons*)m3AllocZeroed((int32_t)sizeof(QhPolygons));
    bool ok = poly != NULL && TraceLoops(b, poly);
    if (ok)
    {
        DropSeamVertices(poly, count);
        ok = EmitHull(b, poly, count, origin, out) && MassProperties(out);
    }
    m3Free(poly);
    if (ok)
    {
        m3HullBuildHalfEdges(out);
    }
    return ok;
}

bool m3ComputeHull(const m3Vec3* points, int32_t count, m3HullData* out)
{
    if (out == NULL || !InputValid(points, count))
    {
        return false;
    }
    // The work block is too large for the stack.
    QhBuilder* b = (QhBuilder*)m3AllocZeroed((int32_t)sizeof(QhBuilder));
    if (b == NULL)
    {
        return false;
    }
    bool ok = BuildHull(b, points, count, out);
    m3Free(b);
    if (!ok)
    {
        memset(out, 0, sizeof(*out));
    }
    return ok;
}
