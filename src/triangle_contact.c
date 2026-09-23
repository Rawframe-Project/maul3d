// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex shapes against one triangle. Spheres take the nearest point of
// the triangle. Capsules take the closest points of their core segment
// and the triangle when apart, two points clipped to the triangle's prism
// when lying on its face, and a separating axis test when piercing it.
// Hulls take the separating axis test over the triangle's face, the
// hull's faces and the edge pairs that form Minkowski faces, then clip or
// take the edge contact.

#include "triangle_contact.h"

#include "distance.h"
#include "manifold.h"

#include <float.h>
#include <string.h>

typedef struct m3TriPoint
{
    m3Vec3 point;
    int32_t feature; // vertex bitmask, 7 = face interior
} m3TriPoint;

// The point of triangle a, b, c nearest q, with the feature it lies on
// as a vertex bitmask (7 for the face). Inside the face's prism the
// projection wins; otherwise the nearest of the three edges, each
// clamped to its segment, the first edge on ties.
static m3TriPoint ClosestPointOnTriangle(m3Vec3 a, m3Vec3 b, m3Vec3 c, m3Vec3 q)
{
    const m3Vec3 v[3] = {a, b, c};
    m3Vec3 n = m3Cross3(m3Sub3(b, a), m3Sub3(c, a));
    m3real nn = m3Dot3(n, n);
    if (nn > 0.0f)
    {
        // Barycentric weights of q's projection, times |n|^2.
        m3real w[3];
        for (int32_t i = 0; i < 3; ++i)
        {
            w[i] = m3Dot3(n, m3Cross3(m3Sub3(v[(i + 1) % 3], q), m3Sub3(v[(i + 2) % 3], q)));
        }
        if (w[0] >= 0.0f && w[1] >= 0.0f && w[2] >= 0.0f)
        {
            m3Vec3 p = m3Add3(m3MulSV3(w[0] / nn, a),
                              m3Add3(m3MulSV3(w[1] / nn, b), m3MulSV3(w[2] / nn, c)));
            return (m3TriPoint){p, 7};
        }
    }
    m3TriPoint best = {a, 1};
    m3real bestD = FLT_MAX;
    for (int32_t i = 0; i < 3; ++i)
    {
        int32_t j = (i + 1) % 3;
        m3Vec3 e = m3Sub3(v[j], v[i]);
        m3real ee = m3Dot3(e, e);
        m3real t = ee > 0.0f ? m3ClampF(m3Dot3(m3Sub3(q, v[i]), e) / ee, 0.0f, 1.0f) : 0.0f;
        m3Vec3 p = m3Add3(v[i], m3MulSV3(t, e));
        m3Vec3 d = m3Sub3(q, p);
        if (m3Dot3(d, d) < bestD)
        {
            bestD = m3Dot3(d, d);
            int32_t feature = t <= 0.0f ? 1 << i : (t >= 1.0f ? 1 << j : (1 << i) | (1 << j));
            best = (m3TriPoint){p, feature};
        }
    }
    return best;
}

void m3CollideSphereTriangle(m3TriManifold* out, m3Vec3 center, m3real radius, const m3Vec3 tri[3])
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

// Clips segment to the prism over the triangle: the three side planes
// through its edges, perpendicular to its face. False when nothing is
// left.
static bool ClipSegmentToPrism(m3Vec3 segment[2], const m3Vec3 tri[3], m3Vec3 faceNormal)
{
    for (int32_t i = 0; i < 3; ++i)
    {
        m3Vec3 v1 = tri[i];
        m3Vec3 side = m3Cross3(m3Normalize3(m3Sub3(tri[(i + 1) % 3], v1)), faceNormal); // outward
        m3real d1 = m3Dot3(side, m3Sub3(segment[0], v1));
        m3real d2 = m3Dot3(side, m3Sub3(segment[1], v1));
        if (d1 > 0.0f && d2 > 0.0f)
        {
            return false;
        }
        if (d1 > 0.0f || d2 > 0.0f)
        {
            m3Vec3 cut =
                m3Add3(segment[0], m3MulSV3(d1 / (d1 - d2), m3Sub3(segment[1], segment[0])));
            segment[d1 > 0.0f ? 0 : 1] = cut;
        }
    }
    return true;
}

// Closest points of segments p + s d and q + t e, s and t in [0, 1].
static void SegmentFractions(m3Vec3 p, m3Vec3 d, m3Vec3 q, m3Vec3 e, m3real* s, m3real* t)
{
    m3Vec3 r = m3Sub3(p, q);
    m3real dd = m3Dot3(d, d);
    m3real ee = m3Dot3(e, e);
    m3real de = m3Dot3(d, e);
    m3real dr = m3Dot3(d, r);
    m3real er = m3Dot3(e, r);
    m3real denom = dd * ee - de * de;
    *s = denom > 1.0e-9f ? m3ClampF((de * er - dr * ee) / denom, 0.0f, 1.0f) : 0.0f;
    *t = ee > 1.0e-9f ? m3ClampF((de * *s + er) / ee, 0.0f, 1.0f) : 0.0f;
    *s = dd > 1.0e-9f ? m3ClampF((de * *t - dr) / dd, 0.0f, 1.0f) : 0.0f;
}

typedef struct SegTriClosest
{
    m3Vec3 onSegment;
    m3TriPoint onTriangle;
    m3real distance; // zero when the segment pierces the triangle
} SegTriClosest;

// The closest points of segment c1-c2 and the triangle: the segment's
// crossing of the face when it pierces it, else the best of its two ends
// against the triangle and the segment against each edge.
static SegTriClosest ClosestSegmentTriangle(m3Vec3 c1, m3Vec3 c2, const m3Vec3 tri[3], m3Vec3 triN,
                                            m3real triOff)
{
    SegTriClosest best;
    m3real h1 = m3Dot3(triN, c1) - triOff;
    m3real h2 = m3Dot3(triN, c2) - triOff;
    if (h1 * h2 < 0.0f)
    {
        m3Vec3 cross = m3Add3(c1, m3MulSV3(h1 / (h1 - h2), m3Sub3(c2, c1)));
        m3TriPoint on = ClosestPointOnTriangle(tri[0], tri[1], tri[2], cross);
        if (on.feature == 7)
        {
            return (SegTriClosest){cross, on, 0.0f};
        }
    }
    best.distance = FLT_MAX;
    const m3Vec3 ends[2] = {c1, c2};
    for (int32_t k = 0; k < 2; ++k)
    {
        m3TriPoint on = ClosestPointOnTriangle(tri[0], tri[1], tri[2], ends[k]);
        m3real d = m3Length3(m3Sub3(ends[k], on.point));
        if (d < best.distance)
        {
            best = (SegTriClosest){ends[k], on, d};
        }
    }
    for (int32_t i = 0; i < 3; ++i)
    {
        int32_t j = (i + 1) % 3;
        m3Vec3 e = m3Sub3(tri[j], tri[i]);
        m3real s;
        m3real t;
        SegmentFractions(c1, m3Sub3(c2, c1), tri[i], e, &s, &t);
        m3Vec3 onSeg = m3Add3(c1, m3MulSV3(s, m3Sub3(c2, c1)));
        m3Vec3 onTri = m3Add3(tri[i], m3MulSV3(t, e));
        m3real d = m3Length3(m3Sub3(onSeg, onTri));
        if (d < best.distance)
        {
            int32_t feature = t <= 0.0f ? 1 << i : (t >= 1.0f ? 1 << j : (1 << i) | (1 << j));
            best = (SegTriClosest){onSeg, {onTri, feature}, d};
        }
    }
    return best;
}

// Two contact points at the ends of the capsule core clipped to the
// triangle's prism, each resting on the face. False when the clip leaves
// nothing.
static bool CapsuleFaceContact(m3TriManifold* out, m3Vec3 c1, m3Vec3 c2, m3real radius,
                               const m3Vec3 tri[3], m3Vec3 triN, m3real triOff)
{
    m3Vec3 segment[2] = {c1, c2};
    if (!ClipSegmentToPrism(segment, tri, triN))
    {
        return false;
    }
    out->normal = triN;
    out->feature = 7;
    out->pointCount = 2;
    m3real least = FLT_MAX;
    for (int32_t k = 0; k < 2; ++k)
    {
        m3real height = m3Dot3(triN, segment[k]) - triOff;
        out->separation[k] = height - radius;
        out->point[k] = m3Sub3(segment[k], m3MulSV3(0.5f * (radius + height), triN));
        out->localId[k] = (uint16_t)k;
        least = m3MinF(least, height);
    }
    out->dist2 = least > 0.0f ? least * least : 0.0f;
    return true;
}

// One point between the capsule core and a triangle feature along axis,
// the direction from the triangle toward the core.
static void CapsulePointContact(m3TriManifold* out, m3Vec3 onCore, m3Vec3 onTri, int32_t feature,
                                m3Vec3 axis, m3real radius)
{
    m3real separation = m3Dot3(axis, m3Sub3(onCore, onTri));
    out->normal = axis;
    out->feature = feature;
    out->pointCount = 1;
    out->dist2 = separation > 0.0f ? separation * separation : 0.0f;
    out->separation[0] = separation - radius;
    out->point[0] = m3MulSV3(0.5f, m3Add3(onTri, m3Sub3(onCore, m3MulSV3(radius, axis))));
    out->localId[0] = 0;
}

// Capsule versus one triangle. Apart, the closest points decide: a core
// facing the face (the closest direction within 60 degrees of the
// normal, or the face itself nearest) rests on two clipped points, any
// other approach takes one point. Piercing, the separating axis test
// between the face normal and the triangle edges crossed with the core
// picks a face contact unless an edge axis is clearly shallower.
void m3CollideCapsuleTriangle(m3TriManifold* out, m3Vec3 c1, m3Vec3 c2, m3real radius,
                              const m3Vec3 tri[3])
{
    out->pointCount = 0;
    m3Vec3 triN = m3Normalize3(m3Cross3(m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[0])));
    m3real triOff = m3Dot3(triN, tri[0]);
    out->triNormal = triN;
    if (m3Dot3(triN, m3MulSV3(0.5f, m3Add3(c1, c2))) - triOff < 0.0f)
    {
        return; // back side cull
    }
    SegTriClosest closest = ClosestSegmentTriangle(c1, c2, tri, triN, triOff);
    if (closest.distance > radius + M3_SPECULATIVE_DISTANCE)
    {
        return;
    }
    if (closest.distance > 100.0f * FLT_EPSILON)
    {
        m3Vec3 axis =
            m3MulSV3(1.0f / closest.distance, m3Sub3(closest.onSegment, closest.onTriangle.point));
        bool facing = closest.onTriangle.feature == 7 || m3Dot3(axis, triN) > 0.5f;
        if (!facing || !CapsuleFaceContact(out, c1, c2, radius, tri, triN, triOff))
        {
            CapsulePointContact(out, closest.onSegment, closest.onTriangle.point,
                                closest.onTriangle.feature, axis, radius);
        }
        return;
    }
    // Piercing: the face axis is the shallower core end's height; each
    // edge axis runs from the triangle's center outward.
    m3real faceSep = m3MinF(m3Dot3(triN, c1), m3Dot3(triN, c2)) - triOff;
    m3Vec3 center = m3MulSV3(1.0f / 3.0f, m3Add3(tri[0], m3Add3(tri[1], tri[2])));
    m3Vec3 dir = m3Sub3(c2, c1);
    int32_t bestEdge = -1;
    m3real bestSep = -FLT_MAX;
    m3Vec3 bestAxis = triN;
    for (int32_t i = 0; i < 3; ++i)
    {
        m3Vec3 e = m3Sub3(tri[(i + 1) % 3], tri[i]);
        m3Vec3 axis = m3Cross3(e, dir);
        m3real length = m3Length3(axis);
        if (length < 0.005f * m3Length3(e) * m3Length3(dir))
        {
            continue; // nearly parallel: the face axis covers it
        }
        axis = m3MulSV3(1.0f / length, axis);
        axis = m3Dot3(axis, m3Sub3(tri[i], center)) < 0.0f ? m3Neg3(axis) : axis;
        m3real sep = m3MinF(m3Dot3(axis, c1), m3Dot3(axis, c2)) - m3Dot3(axis, tri[i]);
        if (sep > bestSep)
        {
            bestSep = sep;
            bestEdge = i;
            bestAxis = axis;
        }
    }
    if (faceSep > radius || bestSep > radius)
    {
        return;
    }
    const m3real tolerance = 0.005f;
    if ((bestEdge < 0 || bestSep <= faceSep + tolerance) &&
        CapsuleFaceContact(out, c1, c2, radius, tri, triN, triOff))
    {
        return;
    }
    if (bestEdge >= 0)
    {
        m3Vec3 e = m3Sub3(tri[(bestEdge + 1) % 3], tri[bestEdge]);
        m3real s;
        m3real t;
        SegmentFractions(c1, dir, tri[bestEdge], e, &s, &t);
        int32_t feature = (1 << bestEdge) | (1 << ((bestEdge + 1) % 3));
        CapsulePointContact(out, m3Add3(c1, m3MulSV3(s, dir)),
                            m3Add3(tri[bestEdge], m3MulSV3(t, e)), feature, bestAxis, radius);
    }
}

#define M3_MESH_CLIP_CAP 16

typedef struct m3MeshClipVertex
{
    m3Vec3 position;
    m3real separation; // height over the reference plane
} m3MeshClipVertex;

typedef struct ClipPlane
{
    m3Vec3 normal; // outward: points with positive height are cut away
    m3real offset;
} ClipPlane;

// Clips a polygon to one plane (Sutherland and Hodgman), keeping each
// point's height over the reference plane. Returns the new count, or 0
// when the polygon would outgrow the buffer.
static int32_t ClipToPlane(m3MeshClipVertex* out, const m3MeshClipVertex* in, int32_t count,
                           ClipPlane plane, ClipPlane ref)
{
    int32_t n = 0;
    for (int32_t k = 0; k < count; ++k)
    {
        m3MeshClipVertex cur = in[k];
        m3MeshClipVertex nxt = in[(k + 1) % count];
        m3real dc = m3Dot3(plane.normal, cur.position) - plane.offset;
        m3real dn = m3Dot3(plane.normal, nxt.position) - plane.offset;
        if (n + 2 > M3_MESH_CLIP_CAP)
        {
            return 0;
        }
        if (dc <= 0.0f)
        {
            out[n++] = cur;
        }
        if (dc * dn < 0.0f)
        {
            m3Vec3 p =
                m3Add3(cur.position, m3MulSV3(dc / (dc - dn), m3Sub3(nxt.position, cur.position)));
            out[n++] = (m3MeshClipVertex){p, m3Dot3(ref.normal, p) - ref.offset};
        }
    }
    return n;
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

// The triangle as seen by the hull kernel: its plane, center, edges and
// the outward in-plane direction of each edge.
typedef struct TriFrame
{
    const m3Vec3* v;
    m3Vec3 normal;
    m3real offset;
    m3Vec3 center;
    m3Vec3 edge[3];
    m3Vec3 out[3];
} TriFrame;

static TriFrame MakeTriFrame(const m3Vec3 tri[3])
{
    TriFrame t;
    t.v = tri;
    t.normal = m3Normalize3(m3Cross3(m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[0])));
    t.offset = m3Dot3(t.normal, tri[0]);
    t.center = m3MulSV3(1.0f / 3.0f, m3Add3(tri[0], m3Add3(tri[1], tri[2])));
    for (int32_t i = 0; i < 3; ++i)
    {
        t.edge[i] = m3Sub3(tri[(i + 1) % 3], tri[i]);
        t.out[i] = m3Normalize3(m3Cross3(t.edge[i], t.normal));
    }
    return t;
}

typedef struct HullTriEdge
{
    m3real separation;
    int32_t triEdge;
    int32_t hullEdge; // even half-edge slot
    m3Vec3 axis;      // from the triangle toward the hull
} HullTriEdge;

// On the Gauss map a triangle edge is the half circle from the face
// normal to its reverse through the edge's outward direction: the
// directions perpendicular to the edge on its outer side. The hull
// edge's arc, from the reversed normals of its two faces, crosses it
// where the arc passes the plane perpendicular to the triangle edge, if
// that crossing lies on the outer side. The crossing is the direction of
// the Minkowski face the two edges make, so it also orients the axis.
static bool TriEdgeArcCrossing(const TriFrame* t, int32_t i, m3Vec3 a, m3Vec3 b, m3Vec3* crossing)
{
    m3real sa = m3Dot3(a, t->edge[i]);
    m3real sb = m3Dot3(b, t->edge[i]);
    if (!(sa * sb < 0.0f))
    {
        return false;
    }
    *crossing = m3Add3(a, m3MulSV3(sa / (sa - sb), m3Sub3(b, a)));
    return m3Dot3(*crossing, t->out[i]) > 0.0f;
}

static HullTriEdge BestHullTriEdge(const TriFrame* t, const m3HullData* hull)
{
    HullTriEdge best = {-FLT_MAX, -1, -1, {0.0f, 0.0f, 0.0f}};
    for (int32_t e = 0; e < hull->edgeCount; e += 2)
    {
        m3Vec3 hp = hull->vertices[hull->edges[e].origin];
        m3Vec3 he = m3Sub3(hull->vertices[hull->edges[e + 1].origin], hp);
        m3Vec3 a = m3Neg3(hull->faceNormals[hull->edges[e].face]);
        m3Vec3 b = m3Neg3(hull->faceNormals[hull->edges[e + 1].face]);
        for (int32_t i = 0; i < 3; ++i)
        {
            m3Vec3 crossing;
            if (!TriEdgeArcCrossing(t, i, a, b, &crossing))
            {
                continue;
            }
            m3Vec3 axis = m3Cross3(t->edge[i], he);
            m3real length = m3Length3(axis);
            if (length < 0.005f * m3Length3(t->edge[i]) * m3Length3(he))
            {
                continue; // nearly parallel: a face axis covers it
            }
            axis = m3MulSV3(1.0f / length, axis);
            axis = m3Dot3(axis, crossing) < 0.0f ? m3Neg3(axis) : axis;
            m3real separation = m3Dot3(axis, m3Sub3(hp, t->v[i]));
            if (separation > best.separation)
            {
                best = (HullTriEdge){separation, i, e, axis};
            }
        }
    }
    return best;
}

// The hull face is the reference: the triangle clipped to that face's
// side planes, heights over the face.
static void HullFaceContact(m3TriManifold* out, const m3HullData* hull, int32_t face,
                            const TriFrame* t)
{
    m3MeshClipVertex bufA[M3_MESH_CLIP_CAP];
    m3MeshClipVertex bufB[M3_MESH_CLIP_CAP];
    ClipPlane ref = {hull->faceNormals[face], hull->faceOffsets[face]};
    for (int32_t k = 0; k < 3; ++k)
    {
        bufA[k] = (m3MeshClipVertex){t->v[k], m3Dot3(ref.normal, t->v[k]) - ref.offset};
    }
    int32_t count = 3;
    m3MeshClipVertex* in = bufA;
    m3MeshClipVertex* outBuf = bufB;
    int32_t n = hull->faceVertCounts[face];
    const uint8_t* loop = &hull->faceIndices[hull->faceVertStart[face]];
    for (int32_t e = 0; e < n && count > 0; ++e)
    {
        m3Vec3 v1 = hull->vertices[loop[e]];
        m3Vec3 v2 = hull->vertices[loop[(e + 1) % n]];
        m3Vec3 side = m3Cross3(m3Normalize3(m3Sub3(v2, v1)), ref.normal);
        count = ClipToPlane(outBuf, in, count, (ClipPlane){side, m3Dot3(side, v1)}, ref);
        m3MeshClipVertex* swap = in;
        in = outBuf;
        outBuf = swap;
    }
    if (count > 0)
    {
        out->normal = m3Neg3(ref.normal); // triangle toward hull
        out->feature = M3_TRI_FEATURE_HULL_FACE;
        KeepClipPoints(out, in, count);
    }
}

// The triangle is the reference: the hull face most opposed to it,
// clipped to the triangle's side planes, heights over the triangle.
static void TriangleFaceContact(m3TriManifold* out, const m3HullData* hull, const TriFrame* t)
{
    int32_t face = 0;
    for (int32_t f = 1; f < hull->faceCount; ++f)
    {
        face = m3Dot3(hull->faceNormals[f], t->normal) < m3Dot3(hull->faceNormals[face], t->normal)
                   ? f
                   : face;
    }
    m3MeshClipVertex bufA[M3_MESH_CLIP_CAP];
    m3MeshClipVertex bufB[M3_MESH_CLIP_CAP];
    ClipPlane ref = {t->normal, t->offset};
    int32_t count = hull->faceVertCounts[face] < M3_MESH_CLIP_CAP - 2 ? hull->faceVertCounts[face]
                                                                      : M3_MESH_CLIP_CAP - 2;
    const uint8_t* loop = &hull->faceIndices[hull->faceVertStart[face]];
    for (int32_t k = 0; k < count; ++k)
    {
        m3Vec3 p = hull->vertices[loop[k]];
        bufA[k] = (m3MeshClipVertex){p, m3Dot3(ref.normal, p) - ref.offset};
    }
    m3MeshClipVertex* in = bufA;
    m3MeshClipVertex* outBuf = bufB;
    for (int32_t i = 0; i < 3 && count > 0; ++i)
    {
        count =
            ClipToPlane(outBuf, in, count, (ClipPlane){t->out[i], m3Dot3(t->out[i], t->v[i])}, ref);
        m3MeshClipVertex* swap = in;
        in = outBuf;
        outBuf = swap;
    }
    if (count > 0)
    {
        out->normal = t->normal;
        out->feature = 7;
        KeepClipPoints(out, in, count);
    }
}

static void HullEdgeContact(m3TriManifold* out, const m3HullData* hull, const TriFrame* t,
                            const HullTriEdge* edge)
{
    m3Vec3 hp = hull->vertices[hull->edges[edge->hullEdge].origin];
    m3Vec3 he = m3Sub3(hull->vertices[hull->edges[edge->hullEdge + 1].origin], hp);
    int32_t i = edge->triEdge;
    m3real s;
    m3real u;
    SegmentFractions(t->v[i], t->edge[i], hp, he, &s, &u);
    m3Vec3 onTri = m3Add3(t->v[i], m3MulSV3(s, t->edge[i]));
    m3Vec3 onHull = m3Add3(hp, m3MulSV3(u, he));
    m3real separation = m3Dot3(edge->axis, m3Sub3(onHull, onTri));
    out->pointCount = 1;
    out->normal = edge->axis;
    out->feature = (1 << i) | (1 << ((i + 1) % 3));
    out->dist2 = separation > 0.0f ? separation * separation : 0.0f;
    out->point[0] = m3MulSV3(0.5f, m3Add3(onTri, onHull));
    out->separation[0] = separation;
    out->localId[0] = 0;
}

// Near pairs the axis tests leave without points get one witness from
// GJK, so a speculative approach is never missed.
static void HullWitnessContact(m3TriManifold* out, const m3HullData* hull, const m3Vec3 tri[3])
{
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA = (m3DistanceProxy){tri, 3, 0.0f};
    input.proxyB = (m3DistanceProxy){hull->vertices, hull->vertexCount, 0.0f};
    input.q = m3MakeIdentityQuat();
    m3DistanceOutput d = m3ShapeDistance(&input);
    if (d.distance > 0.0f && d.distance <= M3_SPECULATIVE_DISTANCE)
    {
        int32_t mask = (int32_t)(d.featureA & 7u);
        out->pointCount = 1;
        out->normal = d.normal;
        out->feature = mask == 0 ? 7 : mask;
        out->dist2 = d.distance * d.distance;
        out->point[0] = m3MulSV3(0.5f, m3Add3(d.pointA, d.pointB));
        out->separation[0] = d.distance;
        out->localId[0] = 0;
    }
}

// Hull versus one triangle, in the hull's frame. The separating axis test
// covers the triangle's face, the hull's faces and the edge pairs that
// form Minkowski faces. An edge axis clearly shallower than both faces
// makes an edge contact; the hull face is the reference only when clearly
// shallower than the triangle's and pressing against it; otherwise the
// triangle is.
void m3CollideHullTriangle(m3TriManifold* out, const m3HullData* hull, const m3Vec3 tri[3])
{
    out->pointCount = 0;
    const m3real tolerance = 0.005f;
    TriFrame t = MakeTriFrame(tri);
    out->triNormal = t.normal;
    if (m3Dot3(t.normal, hull->center) - t.offset < -tolerance)
    {
        return; // back side cull
    }
    m3real triSep = FLT_MAX;
    for (int32_t v = 0; v < hull->vertexCount; ++v)
    {
        triSep = m3MinF(triSep, m3Dot3(t.normal, hull->vertices[v]) - t.offset);
    }
    int32_t hullFace = 0;
    m3real hullSep = -FLT_MAX;
    for (int32_t f = 0; f < hull->faceCount && triSep <= M3_SPECULATIVE_DISTANCE; ++f)
    {
        m3real least = FLT_MAX;
        for (int32_t k = 0; k < 3; ++k)
        {
            least = m3MinF(least, m3Dot3(hull->faceNormals[f], tri[k]) - hull->faceOffsets[f]);
        }
        if (least > hullSep)
        {
            hullSep = least;
            hullFace = f;
        }
    }
    if (triSep > M3_SPECULATIVE_DISTANCE || hullSep > M3_SPECULATIVE_DISTANCE)
    {
        return;
    }
    HullTriEdge edge = BestHullTriEdge(&t, hull);
    if (edge.separation > M3_SPECULATIVE_DISTANCE)
    {
        return;
    }
    if (edge.triEdge >= 0 && edge.separation > m3MaxF(triSep, hullSep) + tolerance)
    {
        HullEdgeContact(out, hull, &t, &edge);
    }
    else if (hullSep > triSep + tolerance && m3Dot3(hull->faceNormals[hullFace], t.normal) < 0.0f)
    {
        HullFaceContact(out, hull, hullFace, &t);
    }
    else
    {
        TriangleFaceContact(out, hull, &t);
    }
    if (out->pointCount == 0)
    {
        HullWitnessContact(out, hull, tri);
        return;
    }
    m3real least = FLT_MAX;
    for (int32_t k = 0; k < out->pointCount; ++k)
    {
        least = m3MinF(least, out->separation[k]);
    }
    out->dist2 = least > 0.0f ? least * least : 0.0f;
}
