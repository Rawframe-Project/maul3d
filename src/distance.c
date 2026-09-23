// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex distance and time of impact. Every convex shape is a point set
// plus a radius, so one distance kernel serves spheres, capsules, hulls
// and triangles, and one time of impact serves every pair.
//
// Distance is Gilbert, Johnson and Keerthi's algorithm over the core
// point sets, in A's frame with B placed by a relative pose. A simplex of
// up to four points of the Minkowski difference is reduced to its
// feature nearest the origin, a point, an edge, a face or the enclosing
// tetrahedron, and grown by the support point opposite the nearest
// point, until the gain falls under a relative tolerance (van den
// Bergen's stopping rule) or a support point repeats.
//
// Time of impact is conservative advancement (Mirtich): the gap between
// the cores shrinks no faster than the relative speed of the centers
// along the normal plus each body's angular speed times its reach, so
// advancing by the gap over that bound never passes the first contact.

#include "distance.h"

#include <float.h>
#include <string.h>

typedef struct Vertex
{
    m3Vec3 a;      // support point of A, frame A
    m3Vec3 b;      // support point of B, frame A
    m3Vec3 w;      // b - a
    m3real weight; // barycentric weight in the nearest point
    int32_t ia;
    int32_t ib;
} Vertex;

typedef struct Simplex
{
    Vertex v[4];
    int32_t count;
} Simplex;

static int32_t Support(const m3DistanceProxy* proxy, m3Vec3 d)
{
    int32_t best = 0;
    m3real bestDot = m3Dot3(proxy->points[0], d);
    for (int32_t i = 1; i < proxy->count; ++i)
    {
        m3real dot = m3Dot3(proxy->points[i], d);
        if (dot > bestDot)
        {
            best = i;
            bestDot = dot;
        }
    }
    return best;
}

static Vertex MakeVertex(const m3DistanceInput* in, int32_t ia, int32_t ib)
{
    Vertex v;
    v.a = in->proxyA.points[ia];
    v.b = m3Add3(m3RotateVec3(in->q, in->proxyB.points[ib]), in->p);
    v.w = m3Sub3(v.b, v.a);
    v.weight = 1.0f;
    v.ia = ia;
    v.ib = ib;
    return v;
}

static m3real Keep1(Vertex p, Simplex* out)
{
    p.weight = 1.0f;
    out->v[0] = p;
    out->count = 1;
    return m3Dot3(p.w, p.w);
}

// The feature of segment p, q nearest the origin. Returns the squared
// distance.
static m3real ReduceSegment(Vertex p, Vertex q, Simplex* out)
{
    m3Vec3 e = m3Sub3(q.w, p.w);
    m3real ee = m3Dot3(e, e);
    m3real t = ee > 0.0f ? -m3Dot3(p.w, e) / ee : 0.0f;
    if (t <= 0.0f)
    {
        return Keep1(p, out);
    }
    if (t >= 1.0f)
    {
        return Keep1(q, out);
    }
    p.weight = 1.0f - t;
    q.weight = t;
    out->v[0] = p;
    out->v[1] = q;
    out->count = 2;
    m3Vec3 c = m3Add3(p.w, m3MulSV3(t, e));
    return m3Dot3(c, c);
}

// The feature of triangle a, b, c nearest the origin: the face when the
// origin projects inside it, else the nearest of the edges the origin
// lies beyond, those opposite a negative barycentric weight, the first
// on ties. A degenerate triangle tries all three edges.
static m3real ReduceTriangle(Vertex a, Vertex b, Vertex c, Simplex* out)
{
    m3Vec3 n = m3Cross3(m3Sub3(b.w, a.w), m3Sub3(c.w, a.w));
    m3real nn = m3Dot3(n, n);
    // Barycentric weights of the origin's projection, times |n|^2, from
    // the sub-triangles it forms with each edge.
    m3real wa = nn > 0.0f ? m3Dot3(n, m3Cross3(b.w, c.w)) : -1.0f;
    m3real wb = nn > 0.0f ? m3Dot3(n, m3Cross3(c.w, a.w)) : -1.0f;
    m3real wc = nn > 0.0f ? m3Dot3(n, m3Cross3(a.w, b.w)) : -1.0f;
    if (wa >= 0.0f && wb >= 0.0f && wc >= 0.0f)
    {
        a.weight = wa / nn;
        b.weight = wb / nn;
        c.weight = wc / nn;
        out->v[0] = a;
        out->v[1] = b;
        out->v[2] = c;
        out->count = 3;
        m3real h = m3Dot3(n, a.w);
        return h * h / nn;
    }
    // Start from vertex a, so even non-finite input leaves a valid
    // simplex behind.
    Simplex edge;
    m3real best = Keep1(a, out);
    if (wc < 0.0f)
    {
        m3real d = ReduceSegment(a, b, &edge);
        if (d <= best)
        {
            *out = edge;
            best = d;
        }
    }
    if (wa < 0.0f)
    {
        m3real d = ReduceSegment(b, c, &edge);
        if (d < best)
        {
            *out = edge;
            best = d;
        }
    }
    if (wb < 0.0f)
    {
        m3real d = ReduceSegment(c, a, &edge);
        if (d < best)
        {
            *out = edge;
            best = d;
        }
    }
    return best;
}

// Six times the signed volume of the tetrahedron o, a, b, c.
static m3real Volume6(m3Vec3 o, m3Vec3 a, m3Vec3 b, m3Vec3 c)
{
    return m3Dot3(m3Sub3(a, o), m3Cross3(m3Sub3(b, o), m3Sub3(c, o)));
}

// A tetrahedron holding the origin stays whole with its barycentric
// weights, the signed volumes the origin forms with each face; otherwise
// the nearest of its faces wins, the first on ties. Returns the squared
// distance, zero when enclosed.
static m3real ReduceTetrahedron(Simplex* s)
{
    Vertex* v = s->v;
    m3Vec3 o = {0.0f, 0.0f, 0.0f};
    m3real total = Volume6(v[0].w, v[1].w, v[2].w, v[3].w);
    m3real part[4] = {Volume6(o, v[1].w, v[2].w, v[3].w), Volume6(v[0].w, o, v[2].w, v[3].w),
                      Volume6(v[0].w, v[1].w, o, v[3].w), Volume6(v[0].w, v[1].w, v[2].w, o)};
    bool inside = total != 0.0f;
    for (int32_t i = 0; i < 4 && inside; ++i)
    {
        inside = total > 0.0f ? part[i] >= 0.0f : part[i] <= 0.0f;
    }
    if (inside)
    {
        for (int32_t i = 0; i < 4; ++i)
        {
            v[i].weight = part[i] / total;
        }
        return 0.0f;
    }
    // Only faces the origin lies beyond, those opposite a vertex whose
    // volume has the wrong sign, can hold the nearest point.
    static const int32_t faces[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
    // Start from vertex 0, so even non-finite input leaves a valid
    // simplex behind; the first face tried replaces it.
    Simplex best;
    m3real bestDist = Keep1(v[0], &best);
    bool tried = false;
    for (int32_t f = 0; f < 4; ++f)
    {
        if (total > 0.0f ? part[f] >= 0.0f : (total < 0.0f ? part[f] <= 0.0f : false))
        {
            continue;
        }
        Simplex face;
        m3real d = ReduceTriangle(v[faces[f][0]], v[faces[f][1]], v[faces[f][2]], &face);
        if (!tried || d < bestDist)
        {
            tried = true;
            best = face;
            bestDist = d;
        }
    }
    *s = best;
    return bestDist;
}

static m3real Reduce(Simplex* s)
{
    switch (s->count)
    {
    case 1:
        return m3Dot3(s->v[0].w, s->v[0].w);
    case 2:
        return ReduceSegment(s->v[0], s->v[1], s);
    case 3:
        return ReduceTriangle(s->v[0], s->v[1], s->v[2], s);
    default:
        return ReduceTetrahedron(s);
    }
}

static m3Vec3 Blend(const Simplex* s, int32_t which)
{
    m3Vec3 p = {0.0f, 0.0f, 0.0f};
    for (int32_t i = 0; i < s->count; ++i)
    {
        m3Vec3 q = which == 0 ? s->v[i].a : (which == 1 ? s->v[i].b : s->v[i].w);
        p = m3Add3(p, m3MulSV3(s->v[i].weight, q));
    }
    return p;
}

static bool Repeats(const Simplex* s, int32_t ia, int32_t ib)
{
    for (int32_t i = 0; i < s->count; ++i)
    {
        if (s->v[i].ia == ia && s->v[i].ib == ib)
        {
            return true;
        }
    }
    return false;
}

m3DistanceOutput m3ShapeDistance(const m3DistanceInput* input)
{
    m3DistanceOutput output;
    memset(&output, 0, sizeof(output));
    Simplex s;
    s.v[0] = MakeVertex(input, 0, 0);
    s.count = 1;
    bool enclosed = false;
    int32_t iteration = 0;
    for (; iteration < M3_MAX_GJK_ITERATIONS; ++iteration)
    {
        Simplex before = s;
        m3real vv = Reduce(&s);
        m3real scale = 0.0f;
        for (int32_t i = 0; i < s.count; ++i)
        {
            scale = m3MaxF(scale, m3Dot3(s.v[i].w, s.v[i].w));
        }
        // The origin on the simplex, up to rounding against its own size.
        if (s.count == 4 || vv <= 1.0e-12f * scale)
        {
            // The origin is enclosed or lies on the simplex: the cores
            // overlap or touch, and either way their distance is zero.
            enclosed = true;
            break;
        }
        // The support point opposite the nearest point v: A's farthest
        // along v, B's farthest against it.
        m3Vec3 v = Blend(&s, 2);
        int32_t ia = Support(&input->proxyA, v);
        m3Vec3 backInB = m3InvRotateVec3(input->q, m3Neg3(v));
        int32_t ib = Support(&input->proxyB, backInB);
        Vertex w = MakeVertex(input, ia, ib);
        if (Repeats(&before, ia, ib) || vv - m3Dot3(v, w.w) <= 1.0e-6f * vv)
        {
            break; // no further gain
        }
        s.v[s.count++] = w;
    }
    output.iterations = iteration;
    for (int32_t i = 0; i < s.count; ++i)
    {
        output.featureA |= s.v[i].ia < 32 ? 1u << s.v[i].ia : 0u;
    }
    output.pointA = Blend(&s, 0);
    output.pointB = enclosed ? output.pointA : Blend(&s, 1);
    m3Vec3 gap = m3Sub3(output.pointB, output.pointA);
    m3real distance = m3Length3(gap);
    if (enclosed || !(distance > 0.0f))
    {
        return output; // overlapping cores: zero distance, no normal
    }
    output.normal = m3MulSV3(1.0f / distance, gap);
    output.distance = distance;
    if (input->useRadii)
    {
        m3real rA = input->proxyA.radius;
        m3real rB = input->proxyB.radius;
        output.distance = m3MaxF(0.0f, distance - rA - rB);
        output.pointA = m3Add3(output.pointA, m3MulSV3(rA, output.normal));
        output.pointB = m3Sub3(output.pointB, m3MulSV3(rB, output.normal));
    }
    return output;
}

// --- Time of impact --------------------------------------------------

m3Transform m3GetSweepTransform(const m3Sweep* sweep, m3real time)
{
    // Normalized linear interpolation of the rotation, on the short arc.
    m3Quat q1 = sweep->q1;
    m3Quat q2 = sweep->q2;
    if (q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w < 0.0f)
    {
        q2 = (m3Quat){-q2.x, -q2.y, -q2.z, -q2.w};
    }
    m3Quat q = {q1.x + time * (q2.x - q1.x), q1.y + time * (q2.y - q1.y),
                q1.z + time * (q2.z - q1.z), q1.w + time * (q2.w - q1.w)};
    m3Transform transform;
    transform.q = m3NormalizeQuat(q);
    m3Vec3 c = m3Add3(sweep->c1, m3MulSV3(time, m3Sub3(sweep->c2, sweep->c1)));
    m3Vec3 r = m3RotateVec3(transform.q, sweep->localCenter);
    transform.p.x = (double)(c.x - r.x);
    transform.p.y = (double)(c.y - r.y);
    transform.p.z = (double)(c.z - r.z);
    return transform;
}

// The fastest the swept rotation turns, in radians per unit of sweep
// time. Interpolating unit quaternions half an angle a apart turns the
// quaternion at most 2 tan(a / 2) per unit time, at the middle of the
// sweep, and the body by twice that; tan(a / 2) = sin a / (1 + cos a).
m3real m3SweepAngularRateBound(const m3Sweep* sweep)
{
    m3Quat q1 = sweep->q1;
    m3Quat q2 = sweep->q2;
    m3real c = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
    c = c < 0.0f ? -c : c;
    c = c > 1.0f ? 1.0f : c;
    m3real s = sqrtf(1.0f - c * c);
    return 4.0f * s / (1.0f + c);
}

// How far any core point of the proxy lies from the body's center of
// mass.
static m3real Reach(const m3DistanceProxy* proxy, m3Vec3 localCenter)
{
    m3real reach = 0.0f;
    for (int32_t i = 0; i < proxy->count; ++i)
    {
        reach = m3MaxF(reach, m3Length3(m3Sub3(proxy->points[i], localCenter)));
    }
    return reach;
}

// The core distance of the two proxies at sweep time t, with the normal
// turned into the sweep frame.
static m3DistanceOutput DistanceAt(const m3TOIInput* input, const m3Sweep* sweepA,
                                   const m3Sweep* sweepB, m3real t)
{
    m3Transform xfA = m3GetSweepTransform(sweepA, t);
    m3Transform xfB = m3GetSweepTransform(sweepB, t);
    m3DistanceInput in;
    in.proxyA = input->proxyA;
    in.proxyB = input->proxyB;
    m3Quat conjA = {-xfA.q.x, -xfA.q.y, -xfA.q.z, xfA.q.w};
    in.q = m3MulQuat(conjA, xfB.q);
    m3Vec3 d = {(m3real)(xfB.p.x - xfA.p.x), (m3real)(xfB.p.y - xfA.p.y),
                (m3real)(xfB.p.z - xfA.p.z)};
    in.p = m3InvRotateVec3(xfA.q, d);
    in.useRadii = false;
    m3DistanceOutput out = m3ShapeDistance(&in);
    out.normal = m3RotateVec3(xfA.q, out.normal);
    return out;
}

m3TOIOutput m3TimeOfImpact(const m3TOIInput* input)
{
    m3TOIOutput output;
    output.state = m3_toiStateUnknown;
    output.fraction = input->maxFraction;
    output.normal = (m3Vec3){0.0f, 0.0f, 0.0f};

    // Both sweeps relative to A's start, so the kernel sees small numbers.
    m3Sweep sweepA = input->sweepA;
    m3Sweep sweepB = input->sweepB;
    m3Vec3 origin = sweepA.c1;
    sweepA.c1 = m3Sub3(sweepA.c1, origin);
    sweepA.c2 = m3Sub3(sweepA.c2, origin);
    sweepB.c1 = m3Sub3(sweepB.c1, origin);
    sweepB.c2 = m3Sub3(sweepB.c2, origin);

    // The core distance a hit stops at. Rounded pairs stop a linear slop
    // into each other, so the contact that follows is a firm one; sharp
    // pairs stop a slop apart, which keeps the normal defined.
    const m3real linearSlop = 0.005f;
    m3real target = m3MaxF(linearSlop, input->proxyA.radius + input->proxyB.radius - linearSlop);
    m3real tolerance = 0.25f * linearSlop;
    m3Vec3 velocity = m3Sub3(m3Sub3(sweepB.c2, sweepB.c1), m3Sub3(sweepA.c2, sweepA.c1));
    m3real spin = m3SweepAngularRateBound(&sweepA) * Reach(&input->proxyA, sweepA.localCenter) +
                  m3SweepAngularRateBound(&sweepB) * Reach(&input->proxyB, sweepB.localCenter);
    m3real t = 0.0f;
    for (int32_t iteration = 0; iteration < 64; ++iteration)
    {
        m3DistanceOutput d = DistanceAt(input, &sweepA, &sweepB, t);
        if (d.distance < target + tolerance)
        {
            output.state = iteration == 0 ? m3_toiStateOverlapped : m3_toiStateHit;
            output.fraction = t;
            output.normal = d.normal;
            return output;
        }
        m3real bound = spin - m3Dot3(velocity, d.normal);
        if (!(bound > 0.0f))
        {
            output.state = m3_toiStateSeparated; // the gap cannot shrink
            return output;
        }
        t += (d.distance - target) / bound;
        if (t >= input->maxFraction)
        {
            output.state = m3_toiStateSeparated;
            return output;
        }
    }
    output.state = m3_toiStateFailed;
    output.fraction = t;
    return output;
}
