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

// Twice the signed area of the triangle a, b, c seen along n.
static m3real TurnAlong(m3Vec3 a, m3Vec3 b, m3Vec3 c, m3Vec3 n)
{
    return m3Dot3(m3Cross3(m3Sub3(b, a), m3Sub3(c, a)), n);
}

// Keeping the deepest four points of a large contact patch can leave
// them all on one side, a support too narrow to rest on; spreading the
// four out keeps the widest patch the points can span.
int32_t m3ReduceContactPoints(const m3Vec3* points, const m3real* separations, int32_t count,
                              m3Vec3 normal, int32_t out[M3_MANIFOLD_MAX_POINTS])
{
    if (count <= M3_MANIFOLD_MAX_POINTS)
    {
        for (int32_t i = 0; i < count; ++i)
        {
            out[i] = i;
        }
        return count;
    }
    int32_t a = 0;
    for (int32_t i = 1; i < count; ++i)
    {
        a = separations[i] < separations[a] ? i : a;
    }
    int32_t b = -1;
    m3real far = -1.0f;
    for (int32_t i = 0; i < count; ++i)
    {
        m3Vec3 d = m3Sub3(points[i], points[a]);
        m3real along = m3Dot3(d, normal);
        m3real flat = m3Dot3(d, d) - along * along;
        if (flat > far)
        {
            far = flat;
            b = i;
        }
    }
    int32_t c = -1;
    m3real area = -1.0f;
    for (int32_t i = 0; i < count; ++i)
    {
        m3real turn = TurnAlong(points[a], points[b], points[i], normal);
        turn = turn < 0.0f ? -turn : turn;
        if (turn > area)
        {
            area = turn;
            c = i;
        }
    }
    // Wind the triangle counterclockwise along the normal; the fourth
    // point lies farthest outside one of its edges.
    if (TurnAlong(points[a], points[b], points[c], normal) < 0.0f)
    {
        int32_t t = b;
        b = c;
        c = t;
    }
    int32_t d = -1;
    m3real outside = 0.0f;
    for (int32_t i = 0; i < count; ++i)
    {
        m3real reach = TurnAlong(points[a], points[b], points[i], normal);
        reach = m3MinF(reach, TurnAlong(points[b], points[c], points[i], normal));
        reach = m3MinF(reach, TurnAlong(points[c], points[a], points[i], normal));
        if (reach < outside)
        {
            outside = reach;
            d = i;
        }
    }
    int32_t n = 0;
    int32_t chosen[4] = {a, b, c, d};
    for (int32_t i = 0; i < count; ++i)
    {
        for (int32_t k = 0; k < 4; ++k)
        {
            if (chosen[k] == i)
            {
                out[n++] = i;
                break;
            }
        }
    }
    return n;
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

// --- Hull versus hull --------------------------------------------------
//
// The separating axis test between convex polyhedra: the face normals of
// both hulls, and the cross products of edge pairs that form a face of
// the Minkowski difference. An edge pair qualifies when the arcs its two
// adjacent face normals trace on the unit sphere cross (Gregorius, "The
// Separating Axis Test between Convex Polyhedra", GDC 2013), which leaves
// only the pairs that can touch. All work happens in A's frame.

// Hull B placed in A's frame once, so every query reads plain arrays.
typedef struct PosedHull
{
    const m3HullData* hull;
    m3Vec3 vertices[M3_HULL_MAX_VERTS];
    m3Vec3 normals[M3_HULL_MAX_FACES];
    m3real offsets[M3_HULL_MAX_FACES];
    m3Vec3 center;
} PosedHull;

static void PoseHull(PosedHull* out, const m3HullData* hull, m3Quat q, m3Vec3 p)
{
    out->hull = hull;
    for (int32_t v = 0; v < hull->vertexCount; ++v)
    {
        out->vertices[v] = m3Add3(m3RotateVec3(q, hull->vertices[v]), p);
    }
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        out->normals[f] = m3RotateVec3(q, hull->faceNormals[f]);
        out->offsets[f] = hull->faceOffsets[f] + m3Dot3(out->normals[f], p);
    }
    out->center = m3Add3(m3RotateVec3(q, hull->center), p);
}

static void KeepHull(PosedHull* out, const m3HullData* hull)
{
    m3Quat identity = {0.0f, 0.0f, 0.0f, 1.0f};
    PoseHull(out, hull, identity, (m3Vec3){0.0f, 0.0f, 0.0f});
}

typedef struct FaceAxis
{
    m3real separation;
    int32_t face;
} FaceAxis;

// The face of ref whose plane the other hull lies farthest above: for
// each face, the least height of the other's vertices over it. A face
// the other hull clears by the speculative distance settles the test.
static FaceAxis BestFace(const PosedHull* ref, const PosedHull* other)
{
    FaceAxis best = {-FLT_MAX, 0};
    for (int32_t f = 0; f < ref->hull->faceCount; ++f)
    {
        m3real least = FLT_MAX;
        for (int32_t v = 0; v < other->hull->vertexCount && least > best.separation; ++v)
        {
            least = m3MinF(least, m3Dot3(ref->normals[f], other->vertices[v]) - ref->offsets[f]);
        }
        if (least > best.separation)
        {
            best = (FaceAxis){least, f};
            if (least > M3_SPECULATIVE_DISTANCE)
            {
                break;
            }
        }
    }
    return best;
}

typedef struct EdgeAxis
{
    m3real separation;
    int32_t edgeA; // even half-edge slots, one per undirected edge
    int32_t edgeB;
    m3Vec3 axis; // from A toward B
} EdgeAxis;

// Arcs a1 to a2 and b1 to b2 on the unit sphere cross when each arc's
// endpoints lie on opposite sides of the other's great circle and the two
// arcs share a hemisphere.
static bool ArcsCross(m3Vec3 a1, m3Vec3 a2, m3Vec3 b1, m3Vec3 b2)
{
    m3Vec3 na = m3Cross3(a1, a2);
    m3Vec3 nb = m3Cross3(b1, b2);
    m3real b1Side = m3Dot3(b1, na);
    m3real b2Side = m3Dot3(b2, na);
    m3real a1Side = m3Dot3(a1, nb);
    m3real a2Side = m3Dot3(a2, nb);
    return b1Side * b2Side < 0.0f && a1Side * a2Side < 0.0f && b1Side * a2Side > 0.0f;
}

// The deepest separation over the edge pairs that form faces of the
// Minkowski difference A - B, whose arcs are A's (u, v) and B's negated
// (-u, -v). Each axis is the edges' cross product, turned outward.
static EdgeAxis BestEdges(const PosedHull* a, const PosedHull* b)
{
    EdgeAxis best = {-FLT_MAX, -1, -1, {0.0f, 1.0f, 0.0f}};
    const m3HullData* ha = a->hull;
    const m3HullData* hb = b->hull;
    for (int32_t ib = 0; ib < hb->edgeCount; ib += 2)
    {
        m3Vec3 pB = b->vertices[hb->edges[ib].origin];
        m3Vec3 eB = m3Sub3(b->vertices[hb->edges[ib + 1].origin], pB);
        m3Vec3 uB = m3Neg3(b->normals[hb->edges[ib].face]);
        m3Vec3 vB = m3Neg3(b->normals[hb->edges[ib + 1].face]);
        for (int32_t ia = 0; ia < ha->edgeCount; ia += 2)
        {
            m3Vec3 uA = a->normals[ha->edges[ia].face];
            m3Vec3 vA = a->normals[ha->edges[ia + 1].face];
            if (!ArcsCross(uA, vA, uB, vB))
            {
                continue;
            }
            m3Vec3 pA = a->vertices[ha->edges[ia].origin];
            m3Vec3 eA = m3Sub3(a->vertices[ha->edges[ia + 1].origin], pA);
            m3Vec3 axis = m3Cross3(eA, eB);
            m3real length = m3Length3(axis);
            if (length < 1.0e-6f)
            {
                continue; // parallel edges give no axis
            }
            // The Minkowski face normal lies on A's arc, which spans less
            // than a half circle, so it agrees with the arc's middle.
            axis = m3MulSV3(1.0f / length, axis);
            if (m3Dot3(axis, m3Add3(uA, vA)) < 0.0f)
            {
                axis = m3Neg3(axis);
            }
            m3real separation = m3Dot3(axis, m3Sub3(pB, pA));
            if (separation > best.separation)
            {
                best = (EdgeAxis){separation, ia, ib, axis};
                if (separation > M3_SPECULATIVE_DISTANCE)
                {
                    return best;
                }
            }
        }
    }
    return best;
}

// Closest points of segments p + s d and q + t e, s and t in [0, 1].
static void ClosestOnSegments(m3Vec3 p, m3Vec3 d, m3Vec3 q, m3Vec3 e, m3Vec3* onP, m3Vec3* onQ)
{
    m3Vec3 r = m3Sub3(p, q);
    m3real dd = m3Dot3(d, d);
    m3real ee = m3Dot3(e, e);
    m3real de = m3Dot3(d, e);
    m3real dr = m3Dot3(d, r);
    m3real er = m3Dot3(e, r);
    m3real denom = dd * ee - de * de;
    m3real s = denom > 1.0e-9f ? m3ClampF((de * er - dr * ee) / denom, 0.0f, 1.0f) : 0.0f;
    m3real t = ee > 1.0e-9f ? m3ClampF((de * s + er) / ee, 0.0f, 1.0f) : 0.0f;
    s = dd > 1.0e-9f ? m3ClampF((de * t - dr) / dd, 0.0f, 1.0f) : 0.0f;
    *onP = m3Add3(p, m3MulSV3(s, d));
    *onQ = m3Add3(q, m3MulSV3(t, e));
}

// Feature ids. A face contact point names the reference side, the
// reference face and the incident vertex it came from, or, for a point
// cut by a side plane, that side and the incident vertex; an edge
// contact names both edges.
#define M3_ID_REF_B   0x8000u
#define M3_ID_CLIPPED 0x4000u
#define M3_ID_EDGES   0xE000u

static m3Manifold EdgeContact(const PosedHull* a, const PosedHull* b, const EdgeAxis* edge)
{
    const m3HullData* ha = a->hull;
    const m3HullData* hb = b->hull;
    m3Vec3 pA = a->vertices[ha->edges[edge->edgeA].origin];
    m3Vec3 pB = b->vertices[hb->edges[edge->edgeB].origin];
    m3Vec3 dA = m3Sub3(a->vertices[ha->edges[edge->edgeA + 1].origin], pA);
    m3Vec3 dB = m3Sub3(b->vertices[hb->edges[edge->edgeB + 1].origin], pB);
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));
    manifold.normal = edge->axis;
    manifold.pointCount = 1;
    ClosestOnSegments(pA, dA, pB, dB, &manifold.points[0].anchorA, &manifold.points[0].anchorB);
    manifold.points[0].separation = edge->separation;
    manifold.points[0].id = (uint16_t)(M3_ID_EDGES | ((((uint32_t)edge->edgeA >> 1) & 0x3Fu) << 7) |
                                       (((uint32_t)edge->edgeB >> 1) & 0x7Fu));
    return manifold;
}

typedef struct ClipPoint
{
    m3Vec3 p;
    uint16_t id;
} ClipPoint;

// Clips the incident face's polygon to the reference face's side planes
// (Sutherland and Hodgman), one plane per reference edge. Returns the
// point count.
static int32_t ClipIncident(const PosedHull* ref, int32_t refFace, const PosedHull* inc,
                            int32_t incFace, uint16_t refFlag, ClipPoint* poly)
{
    const m3HullData* ri = ref->hull;
    const m3HullData* ii = inc->hull;
    int32_t count = ii->faceVertCounts[incFace];
    for (int32_t k = 0; k < count; ++k)
    {
        uint8_t v = ii->faceIndices[ii->faceVertStart[incFace] + k];
        poly[k] = (ClipPoint){inc->vertices[v], (uint16_t)(refFlag | ((uint32_t)refFace << 7) | v)};
    }
    int32_t n = ri->faceVertCounts[refFace];
    const uint8_t* loop = &ri->faceIndices[ri->faceVertStart[refFace]];
    m3Vec3 refN = ref->normals[refFace];
    for (int32_t e = 0; e < n && count > 0; ++e)
    {
        m3Vec3 v1 = ref->vertices[loop[e]];
        m3Vec3 v2 = ref->vertices[loop[(e + 1) % n]];
        m3Vec3 side = m3Cross3(m3Normalize3(m3Sub3(v2, v1)), refN); // outward
        m3real sideOffset = m3Dot3(side, v1);
        ClipPoint out[M3_HULL_MAX_FACE_INDICES];
        int32_t kept = 0;
        for (int32_t k = 0; k < count; ++k)
        {
            ClipPoint cur = poly[k];
            ClipPoint nxt = poly[(k + 1) % count];
            m3real dc = m3Dot3(side, cur.p) - sideOffset;
            m3real dn = m3Dot3(side, nxt.p) - sideOffset;
            if (dc <= 0.0f)
            {
                out[kept++] = cur;
            }
            if (dc * dn < 0.0f)
            {
                m3real t = dc / (dc - dn);
                out[kept].p = m3Add3(m3MulSV3(1.0f - t, cur.p), m3MulSV3(t, nxt.p));
                out[kept].id = (uint16_t)(refFlag | M3_ID_CLIPPED | ((uint32_t)(e & 0x3F) << 7) |
                                          (cur.id & 0x7Fu));
                kept += 1;
            }
        }
        memcpy(poly, out, (size_t)kept * sizeof(ClipPoint));
        count = kept;
    }
    return count;
}

// A face contact: the incident face is the other hull's face most
// opposed to the reference normal; its clipped points within the
// speculative distance, spread to four, land on both surfaces.
static m3Manifold FaceContact(const PosedHull* a, const PosedHull* b, bool refIsA, int32_t refFace)
{
    const PosedHull* ref = refIsA ? a : b;
    const PosedHull* inc = refIsA ? b : a;
    m3Vec3 refN = ref->normals[refFace];
    int32_t incFace = 0;
    for (int32_t f = 1; f < inc->hull->faceCount; ++f)
    {
        incFace = m3Dot3(inc->normals[f], refN) < m3Dot3(inc->normals[incFace], refN) ? f : incFace;
    }
    ClipPoint poly[M3_HULL_MAX_FACE_INDICES];
    int32_t count =
        ClipIncident(ref, refFace, inc, incFace, refIsA ? 0u : (uint16_t)M3_ID_REF_B, poly);
    m3Vec3 points[M3_HULL_MAX_FACE_INDICES];
    m3real depth[M3_HULL_MAX_FACE_INDICES];
    int32_t which[M3_HULL_MAX_FACE_INDICES];
    int32_t n = 0;
    for (int32_t k = 0; k < count; ++k)
    {
        m3real sep = m3Dot3(refN, poly[k].p) - ref->offsets[refFace];
        if (sep < M3_SPECULATIVE_DISTANCE)
        {
            points[n] = poly[k].p;
            depth[n] = sep;
            which[n++] = k;
        }
    }
    int32_t kept[M3_MANIFOLD_MAX_POINTS];
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));
    manifold.pointCount = m3ReduceContactPoints(points, depth, n, refN, kept);
    manifold.normal = refIsA ? refN : m3Neg3(refN);
    for (int32_t k = 0; k < manifold.pointCount; ++k)
    {
        int32_t c = kept[k];
        m3Vec3 onInc = points[c];
        m3Vec3 onRef = m3Sub3(onInc, m3MulSV3(depth[c], refN));
        manifold.points[k].anchorA = refIsA ? onRef : onInc;
        manifold.points[k].anchorB = refIsA ? onInc : onRef;
        manifold.points[k].separation = depth[c];
        manifold.points[k].id = poly[which[c]].id;
    }
    return manifold;
}

m3Manifold m3CollideHulls(const m3HullData* hullA, const m3HullData* hullB, m3Quat q, m3Vec3 p)
{
    m3Manifold empty;
    memset(&empty, 0, sizeof(empty));
    // Zeroed, so slots past the counts read as defined values.
    PosedHull a;
    PosedHull b;
    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    KeepHull(&a, hullA);
    PoseHull(&b, hullB, q, p);
    FaceAxis faceA = BestFace(&a, &b);
    if (faceA.separation > M3_SPECULATIVE_DISTANCE)
    {
        return empty;
    }
    FaceAxis faceB = BestFace(&b, &a);
    if (faceB.separation > M3_SPECULATIVE_DISTANCE)
    {
        return empty;
    }
    EdgeAxis edge = BestEdges(&a, &b);
    if (edge.edgeA >= 0 && edge.separation > M3_SPECULATIVE_DISTANCE)
    {
        return empty;
    }
    // An edge axis wins only when clearly deeper than both faces, and B's
    // face only when clearly deeper than A's, so a resting pair keeps its
    // features from step to step.
    const m3real tolerance = 0.1f * 0.005f;
    m3real faceBest = m3MaxF(faceA.separation, faceB.separation);
    if (edge.edgeA >= 0 && edge.separation > faceBest + tolerance)
    {
        return EdgeContact(&a, &b, &edge);
    }
    bool refIsA = faceB.separation <= faceA.separation + tolerance;
    return FaceContact(&a, &b, refIsA, refIsA ? faceA.face : faceB.face);
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
// error is bounded by one face span.
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
        ClosestOnSegments(p1, segDir, a, m3Sub3(b, a), &cSeg, &cEdge);
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
// Mesh versus convex (sphere, capsule). Feature = the closest voronoi region as a
// vertex bitmask (1|2|4; 7 = face). Face contacts are accepted
// immediately and CLAIM their triangle's edges and vertices; edge
// and vertex contacts are tentative, sorted by distance, accepted
// only while their feature is unclaimed. That filter is the
// internal-edge mitigation. Accepted manifolds then merge into ONE
// pair manifold by normal cluster around the deepest contact (the
// same-normal flat-floor case merges perfectly; the multi-normal
// valley keeps only its dominant cluster).
// ---------------------------------------------------------------
