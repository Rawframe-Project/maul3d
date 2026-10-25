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
// vertices both lie within a tolerance of the other's plane, or above
// it, become one face, and a vertex left between just two faces, on the
// line where they meet, is dropped. Mass properties at unit density come
// from the tetrahedra the faces make with one vertex, through the
// covariance of the canonical tetrahedron.
//
// Near-duplicate points make sliver faces, whose planes float cross
// products cannot hold: a tolerance-sized decision on a short edge tilts
// the face by the tolerance over that edge. Points closer than
// M3_QH_WELD tolerances to an earlier point are dropped before the
// build, which bounds that tilt, and every plane is worked in double
// and rounded to float last.

#include "quickhull.h"

#include "allocator.h"
#include "hull.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define M3_QH_MAX_FACES 1024   // triangle slots, reused as faces die
#define M3_QH_WELD      512.0f // weld radius, in tolerances

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

// The plane of a loop of points by Newell's method, in double: the unit
// normal rounded to float, and the offset of the loop's farthest point
// along it, so the whole loop lies on or below the plane. False when
// the loop is degenerate.
static bool LoopPlane(const QhBuilder* b, const int16_t* loop, int32_t n, m3Vec3* normal,
                      double* offset)
{
    double nx = 0.0;
    double ny = 0.0;
    double nz = 0.0;
    for (int32_t k = 0; k < n; ++k)
    {
        m3Vec3 p = b->p[loop[k]];
        m3Vec3 q = b->p[loop[(k + 1) % n]];
        nx += ((double)p.y - (double)q.y) * ((double)p.z + (double)q.z);
        ny += ((double)p.z - (double)q.z) * ((double)p.x + (double)q.x);
        nz += ((double)p.x - (double)q.x) * ((double)p.y + (double)q.y);
    }
    double length = sqrt(nx * nx + ny * ny + nz * nz);
    if (!(length > 0.0))
    {
        return false;
    }
    *normal = (m3Vec3){(float)(nx / length), (float)(ny / length), (float)(nz / length)};
    double far = -DBL_MAX;
    for (int32_t k = 0; k < n; ++k)
    {
        m3Vec3 p = b->p[loop[k]];
        double d = (double)normal->x * (double)p.x + (double)normal->y * (double)p.y +
                   (double)normal->z * (double)p.z;
        far = d > far ? d : far;
    }
    *offset = far;
    return true;
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
    double offset = 0.0;
    if (!LoopPlane(b, face->v, 3, &face->normal, &offset))
    {
        return false;
    }
    face->offset = (m3real)offset;
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
// edge: the far vertex of each lies above, or within the tolerance of,
// the other's plane. Asking it of either alone would glue a tiny face,
// whose far vertex is always near its neighbor's plane, to every
// neighbor around it, whatever their directions. Groups keep their
// lowest face as root.
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
            if (farG >= 0 && Above(F, b->p[farG]) > -tol && Above(G, b->p[farF]) > -tol)
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
// where they meet and adds nothing to either. A face left with fewer
// than three vertices was a sliver wedged into one neighbor around the
// dropped vertex; that neighbor's loop now runs along the sliver's last
// edge, so the sliver goes too, and its vertices are counted again.
static void DropSeamVertices(QhPolygons* poly, int32_t pointCount)
{
    bool dropped = true;
    while (dropped)
    {
        uint8_t degree[M3_HULL_MAX_INPUT];
        memset(degree, 0, sizeof(degree));
        for (int32_t i = 0; i < poly->indexCount; ++i)
        {
            degree[poly->loop[i]] += 1;
        }
        dropped = false;
        int32_t write = 0;
        int32_t faces = 0;
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
            if (write - start < 3)
            {
                write = start;
                dropped = true;
                continue;
            }
            poly->start[faces] = (uint16_t)start;
            poly->count[faces] = (uint8_t)(write - start);
            faces += 1;
        }
        poly->faceCount = faces;
        poly->indexCount = write;
    }
}

// Writes the hull: vertices in input order, each face's plane from its
// loop, moved back from the first point in double.
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
        m3Vec3 normal;
        double offset = 0.0;
        if (!LoopPlane(b, &poly->loop[poly->start[f]], n, &normal, &offset))
        {
            return false;
        }
        for (int32_t k = 0; k < n; ++k)
        {
            out->faceIndices[poly->start[f] + k] = (uint8_t)index[poly->loop[poly->start[f] + k]];
        }
        offset += (double)normal.x * (double)origin.x + (double)normal.y * (double)origin.y +
                  (double)normal.z * (double)origin.z;
        out->faceNormals[f] = normal;
        out->faceOffsets[f] = (m3real)offset;
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

// Drops every point within M3_QH_WELD tolerances of an earlier kept
// point, keeping input order. Returns the kept count.
static int32_t WeldPoints(QhBuilder* b, int32_t count)
{
    m3real radius = M3_QH_WELD * b->tolerance;
    int32_t kept = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        bool near = false;
        for (int32_t k = 0; k < kept && !near; ++k)
        {
            m3Vec3 d = m3Sub3(b->p[i], b->p[k]);
            near = m3Dot3(d, d) <= radius * radius;
        }
        if (!near)
        {
            b->p[kept++] = b->p[i];
        }
    }
    return kept;
}

static bool BuildHull(m3Heap* heap, QhBuilder* b, const m3Vec3* points, int32_t count,
                      m3HullData* out)
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
    count = WeldPoints(b, count);
    if (count < 4)
    {
        return false;
    }
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
    QhPolygons* poly = (QhPolygons*)m3AllocZeroed(heap, (int32_t)sizeof(QhPolygons));
    bool ok = poly != NULL && TraceLoops(b, poly);
    if (ok)
    {
        DropSeamVertices(poly, count);
        ok = EmitHull(b, poly, count, origin, out) && m3HullMassProperties(out);
    }
    m3Free(heap, poly);
    return ok && m3HullBuildHalfEdges(out);
}

bool m3HullCloudValid(const m3Vec3* points, int32_t count)
{
    if (points == NULL || count <= 0 || count > M3_HULL_MAX_INPUT)
    {
        return false;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        if (!m3LocalV3(points[i]))
        {
            return false;
        }
    }
    return true;
}

bool m3ComputeHull(m3Heap* heap, const m3Vec3* points, int32_t count, m3HullData* out)
{
    if (out == NULL || !InputValid(points, count))
    {
        return false;
    }
    // The work block is too large for the stack.
    QhBuilder* b = (QhBuilder*)m3AllocZeroed(heap, (int32_t)sizeof(QhBuilder));
    if (b == NULL)
    {
        return false;
    }
    bool ok = BuildHull(heap, b, points, count, out);
    m3Free(heap, b);
    if (!ok)
    {
        memset(out, 0, sizeof(*out));
    }
    return ok;
}
