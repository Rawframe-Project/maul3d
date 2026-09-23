// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex shapes against triangle soups: the per-triangle kernels for
// spheres, capsules and hulls, the welded mesh pipeline that claims
// shared edges and vertices once, and its heightfield and voxel-surface
// front ends.

#include "distance.h"
#include "manifold.h"
#include "shape.h"
#include "voxel.h"
#include "world_internal.h"

#include <float.h>
#include <string.h>

typedef struct m3TriPoint
{
    m3Vec3 point;
    int32_t feature; // vertex bitmask, 7 = face interior
} m3TriPoint;

// Closest point on a triangle with its voronoi feature (the
// reference math_functions.c routine, byte for byte in structure).
static m3TriPoint ClosestPointOnTriangle(m3Vec3 a, m3Vec3 b, m3Vec3 c, m3Vec3 q)
{
    m3Vec3 ab = m3Sub3(b, a);
    m3Vec3 ac = m3Sub3(c, a);
    m3Vec3 aq = m3Sub3(q, a);
    m3real d1 = m3Dot3(ab, aq);
    m3real d2 = m3Dot3(ac, aq);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        return (m3TriPoint){a, 1};
    }
    m3Vec3 bq = m3Sub3(q, b);
    m3real d3 = m3Dot3(ab, bq);
    m3real d4 = m3Dot3(ac, bq);
    if (d3 > 0.0f && d4 <= d3)
    {
        return (m3TriPoint){b, 2};
    }
    m3real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        m3real t = d1 / (d1 - d3);
        return (m3TriPoint){m3Add3(a, m3MulSV3(t, ab)), 1 | 2};
    }
    m3Vec3 cq = m3Sub3(q, c);
    m3real d5 = m3Dot3(ab, cq);
    m3real d6 = m3Dot3(ac, cq);
    if (d6 >= 0.0f && d5 <= d6)
    {
        return (m3TriPoint){c, 4};
    }
    m3real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        m3real t = d2 / (d2 - d6);
        return (m3TriPoint){m3Add3(a, m3MulSV3(t, ac)), 1 | 4};
    }
    m3real va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && d4 >= d3 && d5 >= d6)
    {
        m3real t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (m3TriPoint){m3Add3(b, m3MulSV3(t, m3Sub3(c, b))), 2 | 4};
    }
    m3real t1 = vb / (va + vb + vc);
    m3real t2 = vc / (va + vb + vc);
    m3Vec3 p = m3Add3(a, m3Add3(m3MulSV3(t1, ab), m3MulSV3(t2, ac)));
    return (m3TriPoint){p, 7};
}

// One triangle's local manifold: midway contact points (each anchor
// recovers as point -/+ half the separation along the normal).
// feature: vertex bitmask (1|2|4, 7 = triangle face) or 8 = hull
// face contact (the hull path's special acceptance rules).
#define M3_TRI_FEATURE_HULL_FACE 8

typedef struct m3TriManifold
{
    m3Vec3 normal;    // mesh frame, triangle toward shape
    m3Vec3 triNormal; // mesh frame (the hull-face acceptance reads it)
    m3real dist2;     // closest squared distance (the tentative sort key)
    int32_t pointCount;
    int32_t feature;
    m3Vec3 point[4];
    m3real separation[4];
    uint16_t localId[4];
} m3TriManifold;

static void CollideSphereTriangle(m3TriManifold* out, m3Vec3 center, m3real radius,
                                  const m3Vec3 tri[3])
{
    out->pointCount = 0;
    m3Vec3 triN = m3Cross3(m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[0]));
    if (m3Dot3(triN, m3Sub3(center, tri[0])) < 0.0f)
    {
        return; // back side cull (CCW winding, outward normals)
    }
    m3TriPoint closest = ClosestPointOnTriangle(tri[0], tri[1], tri[2], center);
    m3Vec3 d = m3Sub3(center, closest.point);
    m3real dist2 = m3Dot3(d, d);
    m3real reach = radius + M3_SPECULATIVE_DISTANCE;
    if (dist2 > reach * reach)
    {
        return;
    }
    m3real dist = sqrtf(dist2);
    m3Vec3 normal = dist2 > 1000.0f * FLT_MIN ? m3MulSV3(1.0f / dist, d) : m3Normalize3(triN);
    out->normal = normal;
    out->triNormal = m3Normalize3(triN);
    out->dist2 = dist2;
    out->feature = closest.feature;
    out->pointCount = 1;
    out->separation[0] = dist - radius;
    out->point[0] = m3MulSV3(0.5f, m3Add3(m3Sub3(center, m3MulSV3(radius, normal)), closest.point));
    out->localId[0] = 0;
}

// Clip the capsule segment to the triangle's side planes (reference
// b3ClipSegmentToTriangleFace).
static int ClipSegmentToTriFace(m3Vec3 segment[2], const m3Vec3 tri[3], m3Vec3 faceNormal)
{
    m3Vec3 vertex1 = tri[2];
    for (int32_t i = 0; i < 3; ++i)
    {
        m3Vec3 vertex2 = tri[i];
        m3Vec3 tangent = m3Normalize3(m3Sub3(vertex2, vertex1));
        m3Vec3 binormal = m3Cross3(tangent, faceNormal);
        m3real planeOff = m3Dot3(binormal, vertex1);

        m3Vec3 p1 = segment[0];
        m3Vec3 p2 = segment[1];
        m3real distance1 = m3Dot3(binormal, p1) - planeOff;
        m3real distance2 = m3Dot3(binormal, p2) - planeOff;
        int32_t vertexCount = 0;
        if (distance1 <= 0.0f)
        {
            segment[vertexCount++] = p1;
        }
        if (distance2 <= 0.0f)
        {
            segment[vertexCount++] = p2;
        }
        if (vertexCount < 2 && distance1 * distance2 < 0.0f)
        {
            m3real t = distance1 / (distance1 - distance2);
            segment[vertexCount] = m3Add3(p1, m3MulSV3(t, m3Sub3(p2, p1)));
            vertexCount++;
        }
        if (vertexCount != 2)
        {
            return 0;
        }
        vertex1 = vertex2;
    }
    return 1;
}

// Closest points between two infinite lines, with fractions
// (reference b3LineDistance).
static void LineClosest(m3Vec3 p1, m3Vec3 d1, m3Vec3 p2, m3Vec3 d2, m3real* f1, m3real* f2)
{
    m3real a11 = m3Dot3(d1, d1);
    m3real a12 = -m3Dot3(d1, d2);
    m3real a21 = m3Dot3(d2, d1);
    m3real a22 = -m3Dot3(d2, d2);
    m3Vec3 w = m3Sub3(p1, p2);
    m3real b1 = -m3Dot3(d1, w);
    m3real b2 = -m3Dot3(d2, w);
    m3real det = a11 * a22 - a12 * a21;
    if (det * det < 1000.0f * FLT_MIN)
    {
        *f1 = a11 > 0.0f ? m3Dot3(m3Sub3(p2, p1), d1) / a11 : 0.0f;
        *f2 = 0.0f;
        return;
    }
    *f1 = (a22 * b1 - a12 * b2) / det;
    *f2 = (a11 * b2 - a21 * b1) / det;
}

// Edge-edge separation with the volume-sign orientation guard
// (reference manifold.c b3EdgeEdgeSeparation).
static m3real EdgeEdgeSep(m3Vec3 p1, m3Vec3 e1, m3Vec3 c1, m3Vec3 p2, m3Vec3 e2, m3Vec3 c2)
{
    m3Vec3 u = m3Cross3(e1, e2);
    m3real length2 = m3Dot3(u, u);
    m3real scale2 = m3Dot3(e1, e1) * m3Dot3(e2, e2);
    if (length2 < 0.005f * 0.005f * scale2 || length2 < 1000.0f * FLT_MIN)
    {
        return -3.4e38f; // near parallel: a face axis covers it
    }
    m3Vec3 n = m3MulSV3(1.0f / sqrtf(length2), u);
    m3real sign1 = m3Dot3(n, m3Sub3(p1, c1));
    m3real sign2 = m3Dot3(n, m3Sub3(p2, c2));
    m3real a1 = sign1 < 0.0f ? -sign1 : sign1;
    m3real a2 = sign2 < 0.0f ? -sign2 : sign2;
    if (a1 > a2)
    {
        if (sign1 < 0.0f)
        {
            n = m3Neg3(n);
        }
    }
    else
    {
        if (sign2 > 0.0f)
        {
            n = m3Neg3(n);
        }
    }
    return m3Dot3(n, m3Sub3(p2, p1));
}

// Capsule versus one triangle (reference b3CollideCapsuleAndTriangle):
// GJK shallow path with the two-point face clip when the closest axis
// is near the face normal, single closest point otherwise; deep path
// by face and edge queries with the reference tolerance rule.
static void CollideCapsuleTriangle(m3TriManifold* out, m3Vec3 c1, m3Vec3 c2, m3real radius,
                                   const m3Vec3 tri[3])
{
    out->pointCount = 0;
    m3Vec3 triN = m3Normalize3(m3Cross3(m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[0])));
    m3real triOff = m3Dot3(triN, tri[0]);
    m3Vec3 mid = m3MulSV3(0.5f, m3Add3(c1, c2));
    if (m3Dot3(triN, mid) - triOff < 0.0f)
    {
        return; // back side cull
    }
    out->triNormal = triN;

    m3Vec3 segPts[2] = {c1, c2};
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA.points = tri;
    input.proxyA.count = 3;
    input.proxyA.radius = 0.0f;
    input.proxyB.points = segPts;
    input.proxyB.count = 2;
    input.proxyB.radius = 0.0f;
    input.q = m3MakeIdentityQuat();
    input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.useRadii = false;
    m3DistanceOutput dOut = m3ShapeDistance(&input);

    if (dOut.distance > radius + M3_SPECULATIVE_DISTANCE)
    {
        return;
    }

    if (dOut.distance > 100.0f * FLT_EPSILON)
    {
        // Shallow: prefer the two-point face clip when the closest
        // axis is not grazing the face.
        m3Vec3 delta = m3Normalize3(m3Sub3(dOut.pointB, dOut.pointA));
        m3real cosAngle = m3Dot3(triN, delta);
        cosAngle = cosAngle < 0.0f ? -cosAngle : cosAngle;
        if (cosAngle > 0.2f)
        {
            m3Vec3 segment[2] = {c1, c2};
            if (ClipSegmentToTriFace(segment, tri, triN))
            {
                out->normal = triN;
                out->dist2 = dOut.distance * dOut.distance;
                out->feature = 7;
                out->pointCount = 2;
                for (int32_t k = 0; k < 2; ++k)
                {
                    m3real d = m3Dot3(triN, segment[k]) - triOff;
                    out->separation[k] = d - radius;
                    out->point[k] = m3Sub3(segment[k], m3MulSV3(0.5f * (radius + d), triN));
                    out->localId[k] = (uint16_t)k;
                }
                return;
            }
        }
        // Single closest point; the feature comes from the simplex
        // cache exactly like the sphere path (triangle side = A).
        int32_t mask = (int32_t)(dOut.featureA & 7u);
        out->normal = delta;
        out->dist2 = dOut.distance * dOut.distance;
        out->feature = mask == 0 ? 7 : mask;
        out->pointCount = 1;
        out->separation[0] = dOut.distance - radius;
        out->point[0] =
            m3MulSV3(0.5f, m3Add3(dOut.pointA, m3Sub3(dOut.pointB, m3MulSV3(radius, delta))));
        out->localId[0] = 0;
        return;
    }

    // Deep: face query (min cap-center distance) versus edge query.
    m3real sep1 = m3Dot3(triN, c1) - triOff;
    m3real sep2 = m3Dot3(triN, c2) - triOff;
    m3real faceQuerySep = m3MinF(sep1, sep2);
    if (faceQuerySep > radius)
    {
        return;
    }
    m3Vec3 capDir = m3Sub3(c2, c1);
    m3Vec3 triCenter = m3MulSV3(1.0f / 3.0f, m3Add3(tri[0], m3Add3(tri[1], tri[2])));
    m3real bestEdgeSep = -3.4e38f;
    int32_t bestEdge = 0;
    for (int32_t k = 0; k < 3; ++k)
    {
        m3Vec3 v1 = tri[k];
        m3Vec3 e = m3Sub3(tri[(k + 1) % 3], v1);
        m3real sep = EdgeEdgeSep(v1, e, triCenter, c1, capDir, mid);
        if (sep > bestEdgeSep)
        {
            bestEdgeSep = sep;
            bestEdge = k;
        }
    }
    if (bestEdgeSep > radius)
    {
        return;
    }

    // Face contact: clip the segment, both ends against the plane.
    m3real faceSeparation = faceQuerySep - radius;
    m3Vec3 segment[2] = {c1, c2};
    if (ClipSegmentToTriFace(segment, tri, triN))
    {
        out->normal = triN;
        out->dist2 = 0.0f;
        out->feature = 7;
        out->pointCount = 2;
        m3real minSep = 3.4e38f;
        for (int32_t k = 0; k < 2; ++k)
        {
            m3real d = m3Dot3(triN, segment[k]) - triOff;
            out->separation[k] = d - radius;
            minSep = m3MinF(minSep, out->separation[k]);
            out->point[k] = m3Sub3(segment[k], m3MulSV3(0.5f * (radius + d), triN));
            out->localId[k] = (uint16_t)k;
        }
        faceSeparation = minSep;
    }

    // Edge contact only when the face clip failed or the edge axis is
    // significantly better (the reference tolerance rule).
    m3real edgeSeparation = bestEdgeSep - radius;
    if (out->pointCount == 0 || edgeSeparation > 0.5f * faceSeparation + 0.005f)
    {
        m3Vec3 v1 = tri[bestEdge];
        m3Vec3 triEdge = m3Sub3(tri[(bestEdge + 1) % 3], v1);
        m3Vec3 normal = m3Normalize3(m3Cross3(capDir, triEdge));
        if (m3Dot3(normal, m3Sub3(v1, triCenter)) < 0.0f)
        {
            normal = m3Neg3(normal);
        }
        m3real f1;
        m3real f2;
        LineClosest(v1, triEdge, c1, capDir, &f1, &f2);
        if (f1 < 0.0f || f1 > 1.0f || f2 < 0.0f || f2 > 1.0f)
        {
            return; // closest point beyond the segment ends
        }
        m3Vec3 onTriEdge = m3Add3(v1, m3MulSV3(f1, triEdge));
        m3Vec3 onCapCore = m3Add3(c1, m3MulSV3(f2, capDir));
        m3real separation = m3Dot3(normal, m3Sub3(onCapCore, onTriEdge));
        out->normal = normal;
        out->dist2 = separation > 0.0f ? separation * separation : 0.0f;
        out->feature = (1 << bestEdge) | (1 << ((bestEdge + 1) % 3));
        out->pointCount = 1;
        out->separation[0] = separation - radius;
        out->point[0] =
            m3MulSV3(0.5f, m3Add3(m3Sub3(onCapCore, m3MulSV3(radius, normal)), onTriEdge));
        out->localId[0] = 0;
    }
}

#define M3_MESH_CLIP_CAP 16

// Clip a polygon against one plane, carrying separations against the
// reference plane (a generic Sutherland-Hodgman step for the hull
// versus triangle kernel).
typedef struct m3MeshClipVertex
{
    m3Vec3 position;
    m3real separation;
} m3MeshClipVertex;

static int32_t MeshClipPolygon(m3MeshClipVertex* out, const m3MeshClipVertex* in, int32_t count,
                               m3Vec3 clipNormal, m3real clipOffset, m3Vec3 refNormal,
                               m3real refOffset)
{
    int32_t outCount = 0;
    m3MeshClipVertex prev = in[count - 1];
    m3real prevDist = m3Dot3(clipNormal, prev.position) - clipOffset;
    for (int32_t i = 0; i < count; ++i)
    {
        m3MeshClipVertex curr = in[i];
        m3real currDist = m3Dot3(clipNormal, curr.position) - clipOffset;
        if (prevDist <= 0.0f)
        {
            out[outCount++] = prev;
        }
        if (prevDist * currDist < 0.0f)
        {
            m3real t = prevDist / (prevDist - currDist);
            m3Vec3 p = m3Add3(prev.position, m3MulSV3(t, m3Sub3(curr.position, prev.position)));
            out[outCount].position = p;
            out[outCount].separation = m3Dot3(refNormal, p) - refOffset;
            outCount += 1;
        }
        prev = curr;
        prevDist = currDist;
    }
    return outCount;
}

// The clip points within the speculative distance, reduced to four
// spread over the patch in ascending slot order.
static void KeepClipPoints(m3TriManifold* out, const m3MeshClipVertex* points, int32_t count)
{
    m3Vec3 position[M3_MESH_CLIP_CAP];
    m3real separation[M3_MESH_CLIP_CAP];
    int32_t n = 0;
    for (int32_t c = 0; c < count; ++c)
    {
        if (points[c].separation <= M3_SPECULATIVE_DISTANCE)
        {
            position[n] = points[c].position;
            separation[n] = points[c].separation;
            n += 1;
        }
    }
    int32_t kept[M3_MANIFOLD_MAX_POINTS];
    out->pointCount = m3ReduceContactPoints(position, separation, n, out->normal, kept);
    for (int32_t k = 0; k < out->pointCount; ++k)
    {
        int32_t c = kept[k];
        out->point[k] = m3Sub3(position[c], m3MulSV3(0.5f * separation[c], out->normal));
        out->separation[k] = separation[c];
        out->localId[k] = (uint16_t)k;
    }
}

// Hull versus one triangle (reference b3CollideHullAndTriangle, run
// cacheless: the SAT cache is a per-triangle performance memo, the
// axis selection below is identical without it and joins the BVH
// slice). Everything in the HULL's local frame; the caller converts
// results back to the mesh frame.
static void CollideHullTriangle(m3TriManifold* out, const m3HullData* hull, const m3Vec3 tri[3])
{
    out->pointCount = 0;
    const m3real linearSlop = 0.005f;

    m3Vec3 triN = m3Normalize3(m3Cross3(m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[0])));
    m3real triOff = m3Dot3(triN, tri[0]);
    out->triNormal = triN;
    if (m3Dot3(triN, hull->center) - triOff < -linearSlop)
    {
        return; // back side cull
    }
    m3Vec3 triCenter = m3MulSV3(1.0f / 3.0f, m3Add3(tri[0], m3Add3(tri[1], tri[2])));
    m3Vec3 triEdge[3] = {m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[1]), m3Sub3(tri[0], tri[2])};

    // Face query A: the triangle face against the hull support.
    m3real sepA = 3.4e38f;
    for (int32_t v = 0; v < hull->vertexCount; ++v)
    {
        m3real d = m3Dot3(triN, hull->vertices[v]) - triOff;
        sepA = m3MinF(sepA, d);
    }
    if (sepA > M3_SPECULATIVE_DISTANCE)
    {
        return;
    }

    // Face query B: every hull face against the triangle support.
    int32_t faceB = 0;
    m3real sepB = -3.4e38f;
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3Vec3 n = hull->faceNormals[f];
        m3real best = 3.4e38f;
        for (int32_t k = 0; k < 3; ++k)
        {
            best = m3MinF(best, m3Dot3(n, tri[k]) - hull->faceOffsets[f]);
        }
        if (best > sepB)
        {
            sepB = best;
            faceB = f;
        }
    }
    if (sepB > M3_SPECULATIVE_DISTANCE)
    {
        return;
    }

    // Edge query, Minkowski-gated (the duality-transform test).
    m3real edgeSep = -3.4e38f;
    int32_t edgeTri = -1;
    int32_t edgeHull = -1;
    for (int32_t e = 0; e < hull->edgeCount; e += 2)
    {
        m3Vec3 hp = hull->vertices[hull->edges[e].origin];
        m3Vec3 he = m3Sub3(hull->vertices[hull->edges[e + 1].origin], hp);
        m3Vec3 hn1 = hull->faceNormals[hull->edges[e].face];
        m3Vec3 hn2 = hull->faceNormals[hull->edges[e + 1].face];
        for (int32_t j = 0; j < 3; ++j)
        {
            m3real cab = m3Dot3(hn1, triEdge[j]);
            m3real dab = m3Dot3(hn2, triEdge[j]);
            m3real bcd = m3Dot3(triN, he);
            if (cab * dab >= 0.0f || cab * bcd <= 0.0f)
            {
                continue;
            }
            m3real sep = EdgeEdgeSep(tri[j], triEdge[j], triCenter, hp, he, hull->center);
            if (sep > edgeSep)
            {
                edgeSep = sep;
                edgeTri = j;
                edgeHull = e;
            }
        }
    }
    if (edgeSep > M3_SPECULATIVE_DISTANCE)
    {
        return;
    }

    // Face contact: reference rule, the hull face wins only when it
    // is meaningfully better AND pushes against the triangle normal.
    m3real clippedSep = 3.4e38f;
    m3Vec3 hullFaceN = hull->faceNormals[faceB];
    int pushingUp = m3Dot3(hullFaceN, triN) < 0.0f;
    if (sepB > sepA + linearSlop && pushingUp)
    {
        // Reference face = the hull face; clip the TRIANGLE against
        // its side planes.
        m3MeshClipVertex buf1[M3_MESH_CLIP_CAP];
        m3MeshClipVertex buf2[M3_MESH_CLIP_CAP];
        m3real refOff = hull->faceOffsets[faceB];
        for (int32_t k = 0; k < 3; ++k)
        {
            buf1[k].position = tri[k];
            buf1[k].separation = m3Dot3(hullFaceN, tri[k]) - refOff;
        }
        int32_t count = 3;
        m3MeshClipVertex* input = buf1;
        m3MeshClipVertex* output = buf2;
        int32_t startEdge = -1;
        for (int32_t e = 0; e < hull->edgeCount && startEdge < 0; ++e)
        {
            if (hull->edges[e].face == (uint8_t)faceB)
            {
                startEdge = e;
            }
        }
        if (startEdge < 0)
        {
            return; // a face without edges: malformed hull data, no contact
        }
        int32_t edgeIndex = startEdge;
        do
        {
            int32_t nextIndex = hull->edges[edgeIndex].next;
            m3Vec3 vertex1 = hull->vertices[hull->edges[edgeIndex].origin];
            m3Vec3 vertex2 = hull->vertices[hull->edges[nextIndex].origin];
            m3Vec3 tangent = m3Normalize3(m3Sub3(vertex2, vertex1));
            m3Vec3 binormal = m3Cross3(tangent, hullFaceN);
            count = MeshClipPolygon(output, input, count, binormal, m3Dot3(binormal, vertex1),
                                    hullFaceN, refOff);
            if (count < 3 || count > M3_MESH_CLIP_CAP - 2)
            {
                count = 0;
                break;
            }
            m3MeshClipVertex* tmp = input;
            input = output;
            output = tmp;
            edgeIndex = nextIndex;
        } while (edgeIndex != startEdge);

        if (count > 0)
        {
            out->normal = m3Neg3(hullFaceN); // triangle toward hull
            out->feature = M3_TRI_FEATURE_HULL_FACE;
            KeepClipPoints(out, input, count);
            clippedSep = 3.4e38f;
            for (int32_t k = 0; k < out->pointCount; ++k)
            {
                clippedSep = m3MinF(clippedSep, out->separation[k]);
            }
        }
    }
    else
    {
        // Reference face = the triangle; clip the hull's most
        // anti-parallel (incident) face against the triangle sides.
        int32_t incFace = 0;
        m3real minDot = 3.4e38f;
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            m3real d = m3Dot3(triN, hull->faceNormals[f]);
            if (d < minDot)
            {
                minDot = d;
                incFace = f;
            }
        }
        m3MeshClipVertex buf1[M3_MESH_CLIP_CAP];
        m3MeshClipVertex buf2[M3_MESH_CLIP_CAP];
        int32_t count = 0;
        int32_t startEdge = -1;
        for (int32_t e = 0; e < hull->edgeCount && startEdge < 0; ++e)
        {
            if (hull->edges[e].face == (uint8_t)incFace)
            {
                startEdge = e;
            }
        }
        if (startEdge < 0)
        {
            return; // a face without edges: malformed hull data, no contact
        }
        int32_t edgeIndex = startEdge;
        do
        {
            m3Vec3 p = hull->vertices[hull->edges[edgeIndex].origin];
            buf1[count].position = p;
            buf1[count].separation = m3Dot3(triN, p) - triOff;
            count += 1;
            edgeIndex = hull->edges[edgeIndex].next;
        } while (edgeIndex != startEdge && count < M3_MESH_CLIP_CAP - 2);

        m3MeshClipVertex* input = buf1;
        m3MeshClipVertex* output = buf2;
        for (int32_t j = 0; j < 3 && count > 0; ++j)
        {
            m3Vec3 sideN = m3Normalize3(m3Cross3(triEdge[j], triN));
            count =
                MeshClipPolygon(output, input, count, sideN, m3Dot3(sideN, tri[j]), triN, triOff);
            if (count > M3_MESH_CLIP_CAP - 2)
            {
                count = 0;
                break;
            }
            m3MeshClipVertex* tmp = input;
            input = output;
            output = tmp;
        }
        if (count > 0)
        {
            out->normal = triN;
            out->feature = 7;
            KeepClipPoints(out, input, count);
            clippedSep = 3.4e38f;
            for (int32_t k = 0; k < out->pointCount; ++k)
            {
                clippedSep = m3MinF(clippedSep, out->separation[k]);
            }
        }
    }

    // The edge axis overrides when it is genuinely better (the two
    // reference conditions).
    if (edgeTri >= 0)
    {
        m3real maxFaceSep = m3MaxF(sepA, sepB);
        if ((out->pointCount == 0 && edgeSep > maxFaceSep) ||
            (out->pointCount == 1 && edgeSep > clippedSep + linearSlop))
        {
            m3Vec3 hp = hull->vertices[hull->edges[edgeHull].origin];
            m3Vec3 he = m3Sub3(hull->vertices[hull->edges[edgeHull + 1].origin], hp);
            m3Vec3 normal = m3Normalize3(m3Cross3(triEdge[edgeTri], he));
            m3real outwardA = m3Dot3(normal, m3Sub3(tri[edgeTri], triCenter));
            m3real outwardB = m3Dot3(normal, m3Sub3(hull->center, hp));
            m3real aA = outwardA < 0.0f ? -outwardA : outwardA;
            m3real aB = outwardB < 0.0f ? -outwardB : outwardB;
            if (aA > aB ? outwardA < 0.0f : outwardB < 0.0f)
            {
                normal = m3Neg3(normal);
            }
            m3real f1;
            m3real f2;
            LineClosest(tri[edgeTri], triEdge[edgeTri], hp, he, &f1, &f2);
            if (f1 >= 0.0f && f1 <= 1.0f && f2 >= 0.0f && f2 <= 1.0f)
            {
                m3Vec3 onTri = m3Add3(tri[edgeTri], m3MulSV3(f1, triEdge[edgeTri]));
                m3Vec3 onHull = m3Add3(hp, m3MulSV3(f2, he));
                m3real separation = m3Dot3(normal, m3Sub3(onHull, onTri));
                out->pointCount = 1;
                out->normal = normal;
                out->feature = (1 << edgeTri) | (1 << ((edgeTri + 1) % 3));
                out->dist2 = separation > 0.0f ? separation * separation : 0.0f;
                out->point[0] = m3MulSV3(0.5f, m3Add3(onTri, onHull));
                out->separation[0] = separation;
                out->localId[0] = 0;
            }
        }
    }

    // GJK fallback: speculative SAT can strand a nearby pair with no
    // points; a single witness prevents rare tunneling (reference).
    if (out->pointCount == 0)
    {
        m3DistanceInput input;
        memset(&input, 0, sizeof(input));
        input.proxyA.points = tri;
        input.proxyA.count = 3;
        input.proxyA.radius = 0.0f;
        input.proxyB.points = hull->vertices;
        input.proxyB.count = hull->vertexCount;
        input.proxyB.radius = 0.0f;
        input.q = m3MakeIdentityQuat();
        input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
        input.useRadii = false;
        m3DistanceOutput dOut = m3ShapeDistance(&input);
        if (dOut.distance > 0.0f && dOut.distance <= M3_SPECULATIVE_DISTANCE)
        {
            int32_t mask = (int32_t)(dOut.featureA & 7u);
            out->pointCount = 1;
            out->normal = dOut.normal;
            out->feature = mask == 0 ? 7 : mask;
            out->dist2 = dOut.distance * dOut.distance;
            out->point[0] = m3MulSV3(0.5f, m3Add3(dOut.pointA, dOut.pointB));
            out->separation[0] = dOut.distance;
            out->localId[0] = 0;
        }
        return;
    }
    out->dist2 = clippedSep > 0.0f ? clippedSep * clippedSep : 0.0f;
}

#define M3_MESH_CANDIDATE_CAP 64

typedef struct m3FeatureSet
{
    int32_t edges[3 * M3_MESH_CANDIDATE_CAP]; // packed lo*65536+hi
    int32_t edgeCount;
    int32_t verts[3 * M3_MESH_CANDIDATE_CAP];
    int32_t vertCount;
} m3FeatureSet;

// Returns 1 when the edge or vertex was NEW (unclaimed until now).
static int ClaimEdge(m3FeatureSet* set, int32_t v1, int32_t v2)
{
    int32_t lo = v1 < v2 ? v1 : v2;
    int32_t hi = v1 < v2 ? v2 : v1;
    int32_t key = lo * 65536 + hi;
    for (int32_t i = 0; i < set->edgeCount; ++i)
    {
        if (set->edges[i] == key)
        {
            return 0;
        }
    }
    if (set->edgeCount < 3 * M3_MESH_CANDIDATE_CAP)
    {
        set->edges[set->edgeCount++] = key;
    }
    return 1;
}

static int ClaimVertex(m3FeatureSet* set, int32_t v)
{
    for (int32_t i = 0; i < set->vertCount; ++i)
    {
        if (set->verts[i] == v)
        {
            return 0;
        }
    }
    if (set->vertCount < 3 * M3_MESH_CANDIDATE_CAP)
    {
        set->verts[set->vertCount++] = v;
    }
    return 1;
}

typedef struct m3MeshCandidate
{
    m3TriManifold local;
    int32_t triIndex;
} m3MeshCandidate;

// The welded triangle pipeline, mesh-agnostic: the mesh
// and its BVH arrive as parameters so the native heightfield can
// feed a scratch window mesh through the SAME flow. A NULL bvh
// means every triangle is a candidate (the window is pre-clipped).
static void CollideMeshCore(m3World* world, m3Manifold* fresh, const m3MeshData* mesh,
                            const m3MeshBvh* bvh, int32_t meshShape, int32_t otherShape,
                            int meshIsA)
{
    int32_t meshBody = world->shapes.shapeBody[meshShape];
    int32_t otherBody = world->shapes.shapeBody[otherShape];
    m3Transform xfMv = m3ShapeWorldTransform(world, meshShape);
    m3Transform xfOv = m3ShapeWorldTransform(world, otherShape);
    const m3Transform* xfM = &xfMv;
    const m3Transform* xfO = &xfOv;
    uint8_t otherType = world->shapes.shapeType[otherShape];

    // Localize the convex shape into the mesh frame (doubles here).
    m3Quat conjM = {-xfM->q.x, -xfM->q.y, -xfM->q.z, xfM->q.w};
    m3Quat qRel = m3MulQuat(conjM, xfO->q);
    m3Vec3 dp = {(m3real)(xfO->p.x - xfM->p.x), (m3real)(xfO->p.y - xfM->p.y),
                 (m3real)(xfO->p.z - xfM->p.z)};
    m3Vec3 pRel = m3InvRotateVec3(xfM->q, dp);

    m3real radius = world->shapes.shapeGeom[otherShape].s;
    m3Vec3 s1 = {0.0f, 0.0f, 0.0f};
    m3Vec3 s2 = {0.0f, 0.0f, 0.0f};
    m3Vec3 boundLo;
    m3Vec3 boundHi;
    const m3HullData* hull = NULL;
    m3Quat qHull = m3MakeIdentityQuat(); // mesh frame -> hull frame
    m3Vec3 pHull = {0.0f, 0.0f, 0.0f};
    if (otherType == (uint8_t)m3_hullShape)
    {
        // The hull kernel runs in the HULL frame: triangles transform
        // in, results transform back out with qRel/pRel.
        hull = &world->hulls.hullData[world->shapes.shapeHullIndex[otherShape]];
        qHull = (m3Quat){-qRel.x, -qRel.y, -qRel.z, qRel.w};
        pHull = m3Neg3(m3InvRotateVec3(qRel, pRel));
        boundLo = (m3Vec3){3.4e38f, 3.4e38f, 3.4e38f};
        boundHi = (m3Vec3){-3.4e38f, -3.4e38f, -3.4e38f};
        for (int32_t v = 0; v < hull->vertexCount; ++v)
        {
            m3Vec3 p = m3Add3(m3RotateVec3(qRel, hull->vertices[v]), pRel);
            boundLo.x = m3MinF(boundLo.x, p.x);
            boundLo.y = m3MinF(boundLo.y, p.y);
            boundLo.z = m3MinF(boundLo.z, p.z);
            boundHi.x = m3MaxF(boundHi.x, p.x);
            boundHi.y = m3MaxF(boundHi.y, p.y);
            boundHi.z = m3MaxF(boundHi.z, p.z);
        }
        radius = 0.0f;
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

    // Midphase: the static BVH prunes, then the exact
    // per-triangle reject below runs unchanged, so the accepted
    // sequence is bit-identical to the full scan this replaced
    // (gather returns ascending order; the cap break fires at the
    // same processing point).
    uint16_t gather[M3_MESH_MAX_TRIS];
    int32_t gatherCount;
    if (bvh != NULL)
    {
        gatherCount = m3MeshBvhGather(
            bvh, (m3Vec3){boundLo.x - reach, boundLo.y - reach, boundLo.z - reach},
            (m3Vec3){boundHi.x + reach, boundHi.y + reach, boundHi.z + reach}, gather);
    }
    else
    {
        // The window mesh is pre-clipped: full scan, ascending.
        gatherCount = mesh->triangleCount;
        for (int32_t t = 0; t < gatherCount; ++t)
        {
            gather[t] = (uint16_t)t;
        }
    }

    m3MeshCandidate faceAccepted[M3_MESH_CANDIDATE_CAP];
    int32_t faceCount = 0;
    m3MeshCandidate tentative[M3_MESH_CANDIDATE_CAP];
    int32_t tentativeCount = 0;

    for (int32_t g = 0; g < gatherCount; ++g)
    {
        int32_t t = gather[g];
        if (faceCount >= M3_MESH_CANDIDATE_CAP || tentativeCount >= M3_MESH_CANDIDATE_CAP)
        {
            break;
        }
        m3Vec3 tri[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                         mesh->vertices[mesh->indices[3 * t + 1]],
                         mesh->vertices[mesh->indices[3 * t + 2]]};
        m3real lox = m3MinF(tri[0].x, m3MinF(tri[1].x, tri[2].x)) - reach;
        m3real hix = m3MaxF(tri[0].x, m3MaxF(tri[1].x, tri[2].x)) + reach;
        m3real loy = m3MinF(tri[0].y, m3MinF(tri[1].y, tri[2].y)) - reach;
        m3real hiy = m3MaxF(tri[0].y, m3MaxF(tri[1].y, tri[2].y)) + reach;
        m3real loz = m3MinF(tri[0].z, m3MinF(tri[1].z, tri[2].z)) - reach;
        m3real hiz = m3MaxF(tri[0].z, m3MaxF(tri[1].z, tri[2].z)) + reach;
        if (boundHi.x < lox || boundLo.x > hix || boundHi.y < loy || boundLo.y > hiy ||
            boundHi.z < loz || boundLo.z > hiz)
        {
            continue;
        }

        m3TriManifold local;
        memset(&local, 0, sizeof(local)); // the whole struct is copied
                                          // below; keep O3 flow checks
                                          // and MSVC C4701 quiet
        if (otherType == (uint8_t)m3_sphereShape)
        {
            CollideSphereTriangle(&local, s1, radius, tri);
        }
        else if (otherType == (uint8_t)m3_capsuleShape)
        {
            CollideCapsuleTriangle(&local, s1, s2, radius, tri);
        }
        else
        {
            // The hull kernel wants the triangle in the hull frame;
            // its results come back into the mesh frame here.
            m3Vec3 triH[3];
            for (int32_t k = 0; k < 3; ++k)
            {
                triH[k] = m3Add3(m3RotateVec3(qHull, tri[k]), pHull);
            }
            CollideHullTriangle(&local, hull, triH);
            if (local.pointCount > 0)
            {
                local.normal = m3RotateVec3(qRel, local.normal);
                local.triNormal = m3RotateVec3(qRel, local.triNormal);
                for (int32_t k = 0; k < local.pointCount; ++k)
                {
                    local.point[k] = m3Add3(m3RotateVec3(qRel, local.point[k]), pRel);
                }
            }
        }
        if (local.pointCount == 0)
        {
            continue;
        }
        m3MeshCandidate cand;
        cand.local = local;
        cand.triIndex = t;
        if (local.feature == 7)
        {
            faceAccepted[faceCount++] = cand;
        }
        else if (local.feature == M3_TRI_FEATURE_HULL_FACE)
        {
            // The reference acceptance for hull-face contacts: accept
            // when the contact normal agrees with the triangle normal
            // or the overlap is deep; otherwise tentative.
            m3real cosAngle = m3Dot3(local.triNormal, local.normal);
            m3real minSep = 3.4e38f;
            for (int32_t k = 0; k < local.pointCount; ++k)
            {
                minSep = m3MinF(minSep, local.separation[k]);
            }
            if (cosAngle > 0.5f || minSep < -2.0f * 0.005f)
            {
                faceAccepted[faceCount++] = cand;
            }
            else
            {
                tentative[tentativeCount++] = cand;
            }
        }
        else
        {
            tentative[tentativeCount++] = cand;
        }
    }

    // Face contacts claim their features first (triangle order).
    m3FeatureSet set;
    set.edgeCount = 0;
    set.vertCount = 0;
    for (int32_t k = 0; k < faceCount; ++k)
    {
        int32_t t = faceAccepted[k].triIndex;
        int32_t i1 = mesh->indices[3 * t + 0];
        int32_t i2 = mesh->indices[3 * t + 1];
        int32_t i3 = mesh->indices[3 * t + 2];
        (void)ClaimEdge(&set, i1, i2);
        (void)ClaimEdge(&set, i2, i3);
        (void)ClaimEdge(&set, i3, i1);
        (void)ClaimVertex(&set, i1);
        (void)ClaimVertex(&set, i2);
        (void)ClaimVertex(&set, i3);
    }

    // Tentatives in ascending distance (ties to the lower triangle).
    for (int32_t a = 0; a < tentativeCount; ++a)
    {
        int32_t best = a;
        for (int32_t b = a + 1; b < tentativeCount; ++b)
        {
            if (tentative[b].local.dist2 < tentative[best].local.dist2 ||
                (tentative[b].local.dist2 == tentative[best].local.dist2 &&
                 tentative[b].triIndex < tentative[best].triIndex))
            {
                best = b;
            }
        }
        m3MeshCandidate tmp = tentative[a];
        tentative[a] = tentative[best];
        tentative[best] = tmp;
    }

    // Accept the surviving tentatives into the face list.
    for (int32_t k = 0; k < tentativeCount && faceCount < M3_MESH_CANDIDATE_CAP; ++k)
    {
        int32_t t = tentative[k].triIndex;
        int32_t i1 = mesh->indices[3 * t + 0];
        int32_t i2 = mesh->indices[3 * t + 1];
        int32_t i3 = mesh->indices[3 * t + 2];
        int newEdge1 = ClaimEdge(&set, i1, i2);
        int newEdge2 = ClaimEdge(&set, i2, i3);
        int newEdge3 = ClaimEdge(&set, i3, i1);
        int newVert1 = ClaimVertex(&set, i1);
        int newVert2 = ClaimVertex(&set, i2);
        int newVert3 = ClaimVertex(&set, i3);
        // Baked convexity: a genuinely convex ridge or a
        // boundary is a REAL feature and overrides the claim filter;
        // only flat and concave edges can be ghosts.
        uint8_t convex = mesh->edgeFlags[t];
        int shouldCollide = 0;
        switch (tentative[k].local.feature)
        {
        case 1 | 2:
            shouldCollide = (convex & 1) != 0 || newEdge1;
            break;
        case 2 | 4:
            shouldCollide = (convex & 2) != 0 || newEdge2;
            break;
        case 1 | 4:
            shouldCollide = (convex & 4) != 0 || newEdge3;
            break;
        case 1:
            shouldCollide = newVert1;
            break;
        case 2:
            shouldCollide = newVert2;
            break;
        case 4:
            shouldCollide = newVert3;
            break;
        case M3_TRI_FEATURE_HULL_FACE:
            // A tilted hull-face contact on a triangle with a real
            // convex edge is legitimate (a box teetering on a roof
            // ridge); on all-flat triangles it survives only when
            // the whole triangle is unclaimed (the reference's
            // only-ignore-flat-edges rule).
            shouldCollide = convex != 0 ||
                            (newEdge1 && newEdge2 && newEdge3 && newVert1 && newVert2 && newVert3);
            break;
        default:
            break;
        }
        if (shouldCollide)
        {
            faceAccepted[faceCount++] = tentative[k];
        }
    }

    if (faceCount == 0)
    {
        return;
    }

    // Cluster around the deepest accepted contact: manifolds whose
    // normals agree merge their points; the rest wait for 2b-9c.
    int32_t repIndex = 0;
    m3real repSep = 3.4e38f;
    for (int32_t k = 0; k < faceCount; ++k)
    {
        for (int32_t p = 0; p < faceAccepted[k].local.pointCount; ++p)
        {
            m3real sep = faceAccepted[k].local.separation[p];
            if (sep < repSep ||
                (sep == repSep && faceAccepted[k].triIndex < faceAccepted[repIndex].triIndex))
            {
                repSep = sep;
                repIndex = k;
            }
        }
    }
    m3Vec3 repNormal = faceAccepted[repIndex].local.normal;

    // Gather the cluster's points.
    enum
    {
        GATHER_CAP = 2 * M3_MESH_CANDIDATE_CAP
    };
    m3Vec3 gPoint[GATHER_CAP];
    m3real gSep[GATHER_CAP];
    uint16_t gId[GATHER_CAP];
    uint16_t gMat[GATHER_CAP];
    int32_t gCount = 0;
    for (int32_t k = 0; k < faceCount && gCount < GATHER_CAP; ++k)
    {
        if (m3Dot3(faceAccepted[k].local.normal, repNormal) < 0.99f)
        {
            continue;
        }
        for (int32_t p = 0; p < faceAccepted[k].local.pointCount && gCount < GATHER_CAP; ++p)
        {
            gPoint[gCount] = faceAccepted[k].local.point[p];
            gSep[gCount] = faceAccepted[k].local.separation[p];
            gId[gCount] =
                (uint16_t)((faceAccepted[k].triIndex << 2) | faceAccepted[k].local.localId[p]);
            // The material group rides the point flags: a
            // material-free mesh writes zeros, so its manifolds
            // hash exactly as before.
            gMat[gCount] = mesh->materialCount > 0
                               ? (uint16_t)(mesh->triMaterials[faceAccepted[k].triIndex] << 12)
                               : 0;
            gCount += 1;
        }
    }
    // Four points spread over the cluster, then in ascending id, the
    // canonical point order.
    int32_t kept[M3_MANIFOLD_MAX_POINTS];
    int32_t keptCount = m3ReduceContactPoints(gPoint, gSep, gCount, repNormal, kept);
    for (int32_t a = 0; a < keptCount; ++a)
    {
        for (int32_t b = a + 1; b < keptCount; ++b)
        {
            if (gId[kept[b]] < gId[kept[a]])
            {
                int32_t tmp = kept[a];
                kept[a] = kept[b];
                kept[b] = tmp;
            }
        }
    }

    // Emit: midway points split into both anchors along the normal.
    m3Vec3 nWorld = m3RotateVec3(xfM->q, repNormal); // mesh toward shape
    fresh->normal = meshIsA ? nWorld : m3Neg3(nWorld);
    fresh->pointCount = keptCount;
    for (int32_t k = 0; k < keptCount; ++k)
    {
        int32_t c = kept[k];
        m3Vec3 pw = m3RotateVec3(xfM->q, gPoint[c]);
        double px = xfM->p.x + (double)pw.x;
        double py = xfM->p.y + (double)pw.y;
        double pz = xfM->p.z + (double)pw.z;
        m3real half = 0.5f * gSep[c];
        m3Vec3 anchorMesh =
            m3AnchorFromCom(world, meshBody, px - (double)(nWorld.x * half),
                            py - (double)(nWorld.y * half), pz - (double)(nWorld.z * half));
        m3Vec3 anchorOther =
            m3AnchorFromCom(world, otherBody, px + (double)(nWorld.x * half),
                            py + (double)(nWorld.y * half), pz + (double)(nWorld.z * half));
        fresh->points[k].anchorA = meshIsA ? anchorMesh : anchorOther;
        fresh->points[k].anchorB = meshIsA ? anchorOther : anchorMesh;
        fresh->points[k].separation = gSep[c];
        fresh->points[k].id = gId[c];
        fresh->points[k].flags = gMat[c];
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
        return; // transient scratch stall, grown next step
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
    m3MeshData window;
    memset(&window, 0, sizeof(window));
    window.vertexCount = vertCount;
    window.triangleCount = triCount;
    window.vertices = verts;
    window.indices = tris;
    window.edgeFlags = flags;
    window.triMaterials = mats; // zeros: no painted terrain (yet)
    m3BakeMeshEdgeFlags(&window);
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
