// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact kernels for the convex shapes: sphere-sphere, plane-sphere,
// the hull-hull SAT over faces and edges, capsule segment versus hull,
// and the shape proxies and anchor helpers the narrow phase shares
// (narrowphase.c). Every kernel is a pure function of its inputs.

#include "manifold.h"
#include "distance.h"
#include "shape.h"
#include "world_internal.h"

#include <float.h>
#include <string.h>

void m3MakeTangentBasis(m3Vec3 normal, m3Vec3* t1, m3Vec3* t2)
{
    m3real ax = m3AbsF(normal.x);
    m3real ay = m3AbsF(normal.y);
    m3real az = m3AbsF(normal.z);
    m3Vec3 axis;
    if (ax <= ay && ax <= az)
    {
        axis = (m3Vec3){1.0f, 0.0f, 0.0f};
    }
    else if (ay <= az)
    {
        axis = (m3Vec3){0.0f, 1.0f, 0.0f};
    }
    else
    {
        axis = (m3Vec3){0.0f, 0.0f, 1.0f};
    }
    *t1 = m3Normalize3(m3Cross3(normal, axis));
    *t2 = m3Cross3(normal, *t1);
}

m3Manifold m3CollideSpheres(m3Vec3 d, m3real radiusA, m3real radiusB)
{
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m3real distance = m3Length3(d);
    m3real separation = distance - radiusA - radiusB;
    if (separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    // Concentric centers take the fixed +y fallback (one rule, never
    // NaN, never caller-dependent).
    m3Vec3 normal = m3Normalize3(d);
    manifold.normal = normal;
    manifold.pointCount = 1;
    manifold.points[0].anchorA = m3MulSV3(radiusA, normal);
    manifold.points[0].anchorB = m3MulSV3(-radiusB, normal);
    manifold.points[0].separation = separation;
    manifold.points[0].id = 0;
    return manifold;
}

m3Manifold m3CollidePlaneSphere(m3Vec3 planeNormal, m3real dist, m3real radius)
{
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m3real separation = dist - radius;
    if (separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    manifold.normal = planeNormal; // A (plane) to B (sphere)
    manifold.pointCount = 1;
    // The sphere's deepest point toward the plane; anchorA is filled
    // by the contact update, which knows body A's center.
    manifold.points[0].anchorB = m3MulSV3(-radius, planeNormal);
    manifold.points[0].separation = separation;
    manifold.points[0].id = 0;
    return manifold;
}

// --- Hull-versus-hull SAT, adapted from the reference
// convex_manifold.c (Gauss-map edge pruning by Dirk Gregorius). All
// work happens in A's frame; B arrives via the float relative pose.

typedef struct m3FaceQuery
{
    m3real separation;
    int32_t faceIndex;
} m3FaceQuery;

// Deepest support of `hull` (in its own frame, transformed by q,p into
// the query frame) against each face plane of `ref`.
static m3FaceQuery QueryFaces(const m3HullData* ref, const m3HullData* other, m3Quat q, m3Vec3 p,
                              int refIsA)
{
    m3FaceQuery query;
    query.separation = -3.4e38f;
    query.faceIndex = 0;
    // The other hull's vertices depend on the frame, not the face:
    // transform once (2c-11, the profile's first ask). Same inputs,
    // same operations, so every dot below sees bit-identical values.
    m3Vec3 w[M3_HULL_MAX_VERTS];
    for (int32_t v = 0; v < other->vertexCount; ++v)
    {
        w[v] = refIsA ? m3Add3(m3RotateVec3(q, other->vertices[v]), p)
                      : m3InvRotateVec3(q, m3Sub3(other->vertices[v], p));
    }
    for (int32_t f = 0; f < ref->faceCount; ++f)
    {
        m3Vec3 n = ref->faceNormals[f];
        m3real off = ref->faceOffsets[f];
        m3real best = 3.4e38f;
        for (int32_t v = 0; v < other->vertexCount; ++v)
        {
            m3real d = m3Dot3(n, w[v]) - off;
            best = m3MinF(best, d);
            if (best <= query.separation)
            {
                // This face cannot win the max: the update below
                // would not fire either way, so the break is
                // bit-invisible.
                break;
            }
        }
        if (best > query.separation)
        {
            query.separation = best;
            query.faceIndex = f;
            if (best > M3_SPECULATIVE_DISTANCE)
            {
                // A separating face is a verdict, not a candidate:
                // every caller returns the empty manifold on it, so
                // the rest of the loop can only refine a number
                // nobody reads.
                return query;
            }
        }
    }
    return query;
}

typedef struct m3EdgeQuery
{
    m3real separation;
    int32_t indexA; // half-edge slots (even = one per undirected edge)
    int32_t indexB;
    m3Vec3 axis; // A to B, A's frame
} m3EdgeQuery;

static m3EdgeQuery QueryEdges(const m3HullData* hullA, const m3HullData* hullB, m3Quat q, m3Vec3 p)
{
    m3EdgeQuery query;
    query.separation = -3.4e38f;
    query.indexA = -1;
    query.indexB = -1;
    query.axis = (m3Vec3){0.0f, 1.0f, 0.0f};

    m3Vec3 centerB = m3Add3(m3RotateVec3(q, hullB->center), p);
    for (int32_t ib = 0; ib < hullB->edgeCount; ib += 2)
    {
        const m3HullHalfEdge* edgeB = &hullB->edges[ib];
        const m3HullHalfEdge* twinB = &hullB->edges[ib + 1];
        m3Vec3 qB = m3Add3(m3RotateVec3(q, hullB->vertices[twinB->origin]), p);
        m3Vec3 pB = m3Add3(m3RotateVec3(q, hullB->vertices[edgeB->origin]), p);
        m3Vec3 eB = m3Sub3(qB, pB);
        m3Vec3 uB = m3RotateVec3(q, hullB->faceNormals[edgeB->face]);
        m3Vec3 vB = m3RotateVec3(q, hullB->faceNormals[twinB->face]);

        for (int32_t ia = 0; ia < hullA->edgeCount; ia += 2)
        {
            const m3HullHalfEdge* edgeA = &hullA->edges[ia];
            const m3HullHalfEdge* twinA = &hullA->edges[ia + 1];
            m3Vec3 pA = hullA->vertices[edgeA->origin];
            m3Vec3 qA = hullA->vertices[twinA->origin];
            m3Vec3 eA = m3Sub3(qA, pA);
            m3Vec3 uA = hullA->faceNormals[edgeA->face];
            m3Vec3 vA = hullA->faceNormals[twinA->face];

            // Gauss map: the two edges build a Minkowski face only if
            // the arcs cross (the reference formulation).
            m3real cba = m3Dot3(uB, eA);
            m3real dba = m3Dot3(vB, eA);
            m3real adc = -m3Dot3(uA, eB);
            m3real bdc = -m3Dot3(vA, eB);
            if (!(cba * dba < 0.0f && adc * bdc < 0.0f && cba * bdc > 0.0f))
            {
                continue;
            }

            m3Vec3 axis = m3Cross3(eA, eB);
            m3real len = m3Length3(axis);
            if (len < 1.0e-6f)
            {
                continue; // parallel edges never make the axis
            }
            axis = m3MulSV3(1.0f / len, axis);
            // Orient away from A's center.
            if (m3Dot3(axis, m3Sub3(pA, hullA->center)) < 0.0f)
            {
                axis = m3Neg3(axis);
            }
            m3real separation = m3Dot3(axis, m3Sub3(pB, pA));
            (void)centerB;
            if (separation > query.separation)
            {
                query.separation = separation;
                query.indexA = ia;
                query.indexB = ib;
                query.axis = axis;
                if (separation > M3_SPECULATIVE_DISTANCE)
                {
                    // Same verdict rule as the face query: any
                    // separating edge axis means the caller returns
                    // empty, so stop refining.
                    return query;
                }
            }
        }
    }
    return query;
}

// Closest points between two segments (edge contact).
static void SegmentClosest(m3Vec3 p1, m3Vec3 d1, m3Vec3 p2, m3Vec3 d2, m3Vec3* c1, m3Vec3* c2)
{
    m3Vec3 r = m3Sub3(p1, p2);
    m3real a = m3Dot3(d1, d1);
    m3real e = m3Dot3(d2, d2);
    m3real f = m3Dot3(d2, r);
    m3real c = m3Dot3(d1, r);
    m3real b = m3Dot3(d1, d2);
    m3real denom = a * e - b * b;
    m3real s = denom > 1.0e-9f ? m3ClampF((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
    m3real t = e > 1.0e-9f ? m3ClampF((b * s + f) / e, 0.0f, 1.0f) : 0.0f;
    s = a > 1.0e-9f ? m3ClampF((b * t - c) / a, 0.0f, 1.0f) : 0.0f;
    *c1 = m3Add3(p1, m3MulSV3(s, d1));
    *c2 = m3Add3(p2, m3MulSV3(t, d2));
}

m3Manifold m3CollideHulls(const m3HullData* hullA, const m3HullData* hullB, m3Quat q, m3Vec3 p)
{
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m3FaceQuery faceA = QueryFaces(hullA, hullB, q, p, 1);
    if (faceA.separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    m3FaceQuery faceB = QueryFaces(hullB, hullA, q, p, 0);
    if (faceB.separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    m3EdgeQuery edge = QueryEdges(hullA, hullB, q, p);
    if (edge.indexA >= 0 && edge.separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }

    const m3real linearSlop = 0.005f;
    m3real maxFace = m3MaxF(faceA.separation, faceB.separation);
    if (edge.indexA >= 0 && edge.separation > maxFace + 0.1f * linearSlop)
    {
        // Edge contact: the crossing edges' closest points.
        const m3HullHalfEdge* eA = &hullA->edges[edge.indexA];
        const m3HullHalfEdge* tA = &hullA->edges[edge.indexA + 1];
        const m3HullHalfEdge* eB = &hullB->edges[edge.indexB];
        const m3HullHalfEdge* tB = &hullB->edges[edge.indexB + 1];
        m3Vec3 pA = hullA->vertices[eA->origin];
        m3Vec3 dA = m3Sub3(hullA->vertices[tA->origin], pA);
        m3Vec3 pB = m3Add3(m3RotateVec3(q, hullB->vertices[eB->origin]), p);
        m3Vec3 dB = m3Sub3(m3Add3(m3RotateVec3(q, hullB->vertices[tB->origin]), p), pB);
        m3Vec3 cA;
        m3Vec3 cB;
        SegmentClosest(pA, dA, pB, dB, &cA, &cB);
        manifold.normal = edge.axis;
        manifold.pointCount = 1;
        manifold.points[0].anchorA = cA; // frame A anchors; caller re-bases
        manifold.points[0].anchorB = cB;
        manifold.points[0].separation = edge.separation;
        manifold.points[0].id = (uint16_t)(0x8000u | (((uint32_t)edge.indexA >> 1) << 7) |
                                           ((uint32_t)edge.indexB >> 1));
        return manifold;
    }

    // Face contact: the reference face is the larger separation side
    // (B wins ties by the slop margin, the reference rule).
    int refIsA = faceB.separation <= faceA.separation + 0.1f * linearSlop ? 1 : 0;
    const m3HullData* ref = refIsA ? hullA : hullB;
    const m3HullData* inc = refIsA ? hullB : hullA;
    int32_t refFace = refIsA ? faceA.faceIndex : faceB.faceIndex;
    m3Vec3 refN = ref->faceNormals[refFace];
    m3real refOff = ref->faceOffsets[refFace];

    // The incident face: most anti-parallel on the incident hull, in
    // the REF frame.
    m3Vec3 refNInc = refIsA ? m3InvRotateVec3(q, refN) : m3RotateVec3(q, refN);
    int32_t incFace = 0;
    m3real minDot = 3.4e38f;
    for (int32_t f = 0; f < inc->faceCount; ++f)
    {
        m3real d = m3Dot3(refNInc, inc->faceNormals[f]);
        if (d < minDot)
        {
            minDot = d;
            incFace = f;
        }
    }

    // Incident polygon into the ref frame.
    m3Vec3 poly[M3_HULL_MAX_FACE_INDICES];
    uint16_t polyId[M3_HULL_MAX_FACE_INDICES];
    int32_t polyCount = inc->faceVertCounts[incFace];
    for (int32_t k = 0; k < polyCount; ++k)
    {
        uint8_t vi = inc->faceIndices[inc->faceVertStart[incFace] + k];
        m3Vec3 v = inc->vertices[vi];
        poly[k] = refIsA ? m3Add3(m3RotateVec3(q, v), p) : m3InvRotateVec3(q, m3Sub3(v, p));
        polyId[k] = vi;
    }

    // Sutherland-Hodgman against the reference side planes (one per
    // reference-face edge, normal = tangent x refN pointing outward).
    int32_t n = ref->faceVertCounts[refFace];
    int32_t start = ref->faceVertStart[refFace];
    for (int32_t e = 0; e < n && polyCount > 0; ++e)
    {
        m3Vec3 v1 = ref->vertices[ref->faceIndices[start + e]];
        m3Vec3 v2 = ref->vertices[ref->faceIndices[start + (e + 1) % n]];
        m3Vec3 tangent = m3Normalize3(m3Sub3(v2, v1));
        m3Vec3 sideN = m3Cross3(tangent, refN); // points outward for CCW
        m3real sideOff = m3Dot3(sideN, v1);

        m3Vec3 outPoly[M3_HULL_MAX_FACE_INDICES];
        uint16_t outId[M3_HULL_MAX_FACE_INDICES];
        int32_t outCount = 0;
        for (int32_t k = 0; k < polyCount; ++k)
        {
            m3Vec3 cur = poly[k];
            m3Vec3 nxt = poly[(k + 1) % polyCount];
            m3real dc = m3Dot3(sideN, cur) - sideOff;
            m3real dn = m3Dot3(sideN, nxt) - sideOff;
            if (dc <= 0.0f)
            {
                outPoly[outCount] = cur;
                outId[outCount] = polyId[k];
                outCount += 1;
            }
            if (dc * dn < 0.0f)
            {
                m3real t = dc / (dc - dn);
                outPoly[outCount] = m3Add3(m3MulSV3(1.0f - t, cur), m3MulSV3(t, nxt));
                // A clipped vertex takes a synthetic id from the side
                // plane and the segment, stable per configuration.
                outId[outCount] = (uint16_t)(0x4000u | ((uint32_t)e << 8) | polyId[k]);
                outCount += 1;
            }
        }
        memcpy(poly, outPoly, (size_t)outCount * sizeof(m3Vec3));
        memcpy(polyId, outId, (size_t)outCount * sizeof(uint16_t));
        polyCount = outCount;
    }

    // Keep points below the margin, deepest four, ascending id.
    int32_t candIdx[M3_HULL_MAX_FACE_INDICES];
    m3real candSep[M3_HULL_MAX_FACE_INDICES];
    int32_t candCount = 0;
    for (int32_t k = 0; k < polyCount; ++k)
    {
        m3real sep = m3Dot3(refN, poly[k]) - refOff;
        if (sep < M3_SPECULATIVE_DISTANCE)
        {
            candIdx[candCount] = k;
            candSep[candCount] = sep;
            candCount += 1;
        }
    }
    int32_t want = candCount < M3_MANIFOLD_MAX_POINTS ? candCount : M3_MANIFOLD_MAX_POINTS;
    uint8_t used[M3_HULL_MAX_FACE_INDICES];
    memset(used, 0, sizeof(used));
    int32_t kept[M3_MANIFOLD_MAX_POINTS];
    for (int32_t k = 0; k < want; ++k)
    {
        int32_t best = -1;
        for (int32_t c = 0; c < candCount; ++c)
        {
            if (used[c])
            {
                continue;
            }
            if (best < 0 || candSep[c] < candSep[best] ||
                (candSep[c] == candSep[best] && polyId[candIdx[c]] < polyId[candIdx[best]]))
            {
                best = c;
            }
        }
        used[best] = 1;
        kept[k] = best;
    }
    for (int32_t a = 0; a < want; ++a)
    {
        for (int32_t b = a + 1; b < want; ++b)
        {
            if (polyId[candIdx[kept[b]]] < polyId[candIdx[kept[a]]])
            {
                int32_t tmp = kept[a];
                kept[a] = kept[b];
                kept[b] = tmp;
            }
        }
    }

    // Emit in A's frame with the A-to-B normal.
    m3Vec3 outN = refIsA ? refN : m3Neg3(m3RotateVec3(q, refN));
    manifold.normal = outN;
    manifold.pointCount = want;
    for (int32_t k = 0; k < want; ++k)
    {
        int32_t c = kept[k];
        m3Vec3 onInc = poly[candIdx[c]]; // ref frame, incident side
        m3real sep = candSep[c];
        m3Vec3 onRef = m3Sub3(onInc, m3MulSV3(sep, refN));
        // Into A's frame.
        m3Vec3 wInc = refIsA ? onInc : m3Add3(m3RotateVec3(q, onInc), p);
        m3Vec3 wRef = refIsA ? onRef : m3Add3(m3RotateVec3(q, onRef), p);
        manifold.points[k].anchorA = refIsA ? wRef : wInc;
        manifold.points[k].anchorB = refIsA ? wInc : wRef;
        manifold.points[k].separation = sep;
        manifold.points[k].id = (uint16_t)(((uint32_t)refIsA << 15) | ((uint32_t)refFace << 8) |
                                           (polyId[candIdx[c]] & 0xFFu));
    }
    return manifold;
}

// World center of a shape's sphere (double positions, float offsets).
void m3SphereWorldCenter(const m3World* world, int32_t shape, double* cx, double* cy, double* cz)
{
    m3Transform xf = m3ShapeWorldTransform(world, shape);
    m3Vec3 r = m3RotateVec3(xf.q, world->shapes.shapeGeom[shape].v);
    *cx = xf.p.x + (double)r.x;
    *cy = xf.p.y + (double)r.y;
    *cz = xf.p.z + (double)r.z;
}

// Offset from a body's world center of mass to a world point (float
// is exact enough near contact). Anchors are COM-relative because
// impulses and rotation act about the COM.
m3Vec3 m3AnchorFromCom(const m3World* world, int32_t body, double px, double py, double pz)
{
    const m3Transform* xf = &world->bodies.transforms[body];
    m3Vec3 rlc = m3RotateVec3(xf->q, world->bodies.localCenters[body]);
    return (m3Vec3){(m3real)(px - xf->p.x - (double)rlc.x), (m3real)(py - xf->p.y - (double)rlc.y),
                    (m3real)(pz - xf->p.z - (double)rlc.z)};
}

// Build the GJK proxy for one shape in its own local frame. Spheres
// and capsules park their point(s) in the caller's scratch (the proxy
// only borrows the pointer); hulls point straight at the interned
// vertex array.
m3DistanceProxy m3MakeShapeProxy(const m3World* world, int32_t shape, m3Vec3 scratch[2])
{
    m3DistanceProxy proxy;
    uint8_t type = world->shapes.shapeType[shape];
    if (type == (uint8_t)m3_hullShape)
    {
        const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[shape]];
        proxy.points = hull->vertices;
        proxy.count = hull->vertexCount;
        proxy.radius = 0.0f;
        return proxy;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        scratch[0] = world->shapes.shapeGeom[shape].v;
        scratch[1] = world->shapes.shapeGeom[shape].v2;
        proxy.points = scratch;
        proxy.count = 2;
        proxy.radius = world->shapes.shapeGeom[shape].s;
        return proxy;
    }
    // Sphere (planes never reach the GJK path).
    scratch[0] = world->shapes.shapeGeom[shape].v;
    proxy.points = scratch;
    proxy.count = 1;
    proxy.radius = world->shapes.shapeGeom[shape].s;
    return proxy;
}

// Exact deep recovery, part one: a point core strictly inside a hull.
// The least-deep face (max signed distance, ties to the lower face
// index) IS the minimum translation: moving by -d along its normal
// reaches the supporting plane, so the point leaves the hull, and any
// smaller move keeps every face constraint strictly negative. No
// iteration, no polytope, nothing to make deterministic after the
// fact. The projected witness can land off the face polygon in
// obtuse corners; the normal and depth stay exact and the anchor
// error is bounded by one face span (the reference accepts the same).
void m3DeepPointInHull(const m3HullData* hull, m3Vec3 q, m3Vec3* normalOut, m3real* coreSepOut,
                       m3Vec3* onHullOut)
{
    int32_t best = 0;
    m3real bestD = -3.4e38f;
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3real d = m3Dot3(hull->faceNormals[f], q) - hull->faceOffsets[f];
        if (d > bestD)
        {
            bestD = d;
            best = f;
        }
    }
    *normalOut = hull->faceNormals[best];
    *coreSepOut = bestD;
    *onHullOut = m3Sub3(q, m3MulSV3(bestD, hull->faceNormals[best]));
}

// Capsule versus hull, one path for every depth: the segment SAT.
// Axes are the hull faces plus every hull edge crossed with the
// segment direction (the complete set for a convex against a
// segment). A winning face clips the segment's parameter interval
// against the face side planes and contacts BOTH clipped ends, which
// is what lets a lying capsule rest instead of wobbling on GJK's one
// witness; a winning edge takes the closest-point contact. In
// vertex-region approaches these axes underestimate the true
// distance, so a speculative point can appear a touch early; that
// only pre-arms the solver's speculative band and cannot snag.
// Results in the hull frame: anchorA on the hull, anchorB on the
// capsule skin, normal hull toward capsule.
m3Manifold m3CollideSegmentHull(const m3HullData* hull, m3Vec3 p1, m3Vec3 p2, m3real radius)
{
    const m3real linearSlop = 0.005f;
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    int32_t bestFace = 0;
    m3real bestFaceSep = -3.4e38f;
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3real d1 = m3Dot3(hull->faceNormals[f], p1) - hull->faceOffsets[f];
        m3real d2 = m3Dot3(hull->faceNormals[f], p2) - hull->faceOffsets[f];
        m3real sep = m3MinF(d1, d2);
        if (sep > bestFaceSep)
        {
            bestFaceSep = sep;
            bestFace = f;
        }
    }

    m3Vec3 segDir = m3Sub3(p2, p1);
    m3Vec3 bestEdgeAxis = {0.0f, 1.0f, 0.0f};
    m3real bestEdgeSep = -3.4e38f;
    int32_t bestEdge = -1;
    for (int32_t e = 0; e < hull->edgeCount; e += 2)
    {
        m3Vec3 a = hull->vertices[hull->edges[e].origin];
        m3Vec3 b = hull->vertices[hull->edges[e + 1].origin];
        m3Vec3 axis = m3Cross3(m3Sub3(b, a), segDir);
        m3real len2 = m3Dot3(axis, axis);
        if (len2 < 1.0e-10f)
        {
            continue; // parallel: a face axis covers this direction
        }
        axis = m3MulSV3(1.0f / sqrtf(len2), axis);
        if (m3Dot3(axis, m3Sub3(a, hull->center)) < 0.0f)
        {
            axis = m3Neg3(axis); // outward from the hull
        }
        m3real hullMax = -3.4e38f;
        for (int32_t v = 0; v < hull->vertexCount; ++v)
        {
            m3real proj = m3Dot3(axis, hull->vertices[v]);
            if (proj > hullMax)
            {
                hullMax = proj;
            }
        }
        m3real sep = m3MinF(m3Dot3(axis, p1), m3Dot3(axis, p2)) - hullMax;
        if (sep > bestEdgeSep)
        {
            bestEdgeSep = sep;
            bestEdgeAxis = axis;
            bestEdge = e;
        }
    }

    int edgeWins = bestEdge >= 0 && bestEdgeSep > bestFaceSep + 0.1f * linearSlop;
    m3real coreSep = edgeWins ? bestEdgeSep : bestFaceSep;
    if (coreSep - radius > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }

    if (edgeWins)
    {
        m3Vec3 a = hull->vertices[hull->edges[bestEdge].origin];
        m3Vec3 b = hull->vertices[hull->edges[bestEdge + 1].origin];
        m3Vec3 cSeg;
        m3Vec3 cEdge;
        SegmentClosest(p1, segDir, a, m3Sub3(b, a), &cSeg, &cEdge);
        manifold.normal = bestEdgeAxis;
        manifold.pointCount = 1;
        manifold.points[0].anchorA = cEdge;
        manifold.points[0].anchorB = m3Sub3(cSeg, m3MulSV3(radius, bestEdgeAxis));
        manifold.points[0].separation = bestEdgeSep - radius;
        manifold.points[0].id = (uint16_t)(0x8000u | (uint32_t)bestEdge);
        return manifold;
    }

    m3Vec3 n = hull->faceNormals[bestFace];
    m3real off = hull->faceOffsets[bestFace];
    int32_t count = hull->faceVertCounts[bestFace];
    int32_t startIdx = hull->faceVertStart[bestFace];
    m3Vec3 centroid = {0.0f, 0.0f, 0.0f};
    for (int32_t k = 0; k < count; ++k)
    {
        centroid = m3Add3(centroid, hull->vertices[hull->faceIndices[startIdx + k]]);
    }
    centroid = m3MulSV3(1.0f / (m3real)count, centroid);

    m3real t0 = 0.0f;
    m3real t1 = 1.0f;
    int emptySlab = 0;
    for (int32_t k = 0; k < count; ++k)
    {
        m3Vec3 a = hull->vertices[hull->faceIndices[startIdx + k]];
        m3Vec3 b = hull->vertices[hull->faceIndices[startIdx + (k + 1) % count]];
        m3Vec3 sideN = m3Cross3(m3Sub3(b, a), n);
        if (m3Dot3(sideN, m3Sub3(centroid, a)) < 0.0f)
        {
            sideN = m3Neg3(sideN); // inward: keep the face side
        }
        m3real c0 = m3Dot3(sideN, m3Sub3(p1, a));
        m3real cd = m3Dot3(sideN, segDir);
        if (cd > -1.0e-9f && cd < 1.0e-9f)
        {
            if (c0 < 0.0f)
            {
                emptySlab = 1;
                break;
            }
            continue;
        }
        m3real tc = -c0 / cd;
        if (cd > 0.0f)
        {
            t0 = m3MaxF(t0, tc);
        }
        else
        {
            t1 = m3MinF(t1, tc);
        }
    }
    if (emptySlab || t0 > t1)
    {
        // Grazing a corner outside the face slab: one clamped point,
        // the deterministic middle of the crossed-over interval.
        m3real tm = 0.5f * (t0 + t1);
        tm = m3MaxF(0.0f, m3MinF(1.0f, tm));
        t0 = tm;
        t1 = tm;
    }

    int32_t emitted = 0;
    for (int32_t k = 0; k < 2; ++k)
    {
        if (k == 1 && !(t1 > t0))
        {
            break; // degenerate interval: one point only
        }
        m3real t = k == 0 ? t0 : t1;
        m3Vec3 pt = m3Add3(p1, m3MulSV3(t, segDir));
        m3real d = m3Dot3(n, pt) - off;
        m3real sepK = d - radius;
        if (sepK > M3_SPECULATIVE_DISTANCE)
        {
            continue;
        }
        manifold.points[emitted].anchorA = m3Sub3(pt, m3MulSV3(d, n));
        manifold.points[emitted].anchorB = m3Sub3(pt, m3MulSV3(radius, n));
        manifold.points[emitted].separation = sepK;
        manifold.points[emitted].id = (uint16_t)k;
        emitted += 1;
    }
    manifold.normal = n;
    manifold.pointCount = emitted;
    return manifold;
}

// ---------------------------------------------------------------
// Mesh versus convex (2b-9a sphere, 2b-9b capsule), the reference
// mesh_contact.c recipe. Feature = the closest voronoi region as a
// vertex bitmask (1|2|4; 7 = face). Face contacts are accepted
// immediately and CLAIM their triangle's edges and vertices; edge
// and vertex contacts are tentative, sorted by distance, accepted
// only while their feature is unclaimed. That filter is the
// internal-edge mitigation. Accepted manifolds then merge into ONE
// pair manifold by normal cluster around the deepest contact (the
// same-normal flat-floor case merges perfectly; the multi-normal
// valley keeps only its dominant cluster until per-manifold solver
// normals arrive with 2b-9c).
// ---------------------------------------------------------------
