// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hull infrastructure: the analytic box builder and the interned,
// refcounted, immutable hull pool (lifetime 3 in the memory model).
// The QuickHull point-cloud builder and the general mass integrator
// arrive together in a later slice (fracture fragments need them);
// boxes need neither, their mass properties are closed form.

#include "hull.h"

#include "world_internal.h"

#include <float.h>
#include <string.h>

void m3BuildBoxHull(m3HullData* out, m3Vec3 halfExtents)
{
    memset(out, 0, sizeof(*out));
    m3real hx = halfExtents.x;
    m3real hy = halfExtents.y;
    m3real hz = halfExtents.z;

    // Canonical vertex order (fixed forever: the snapshot and the SAT
    // feature ids will lean on it).
    const m3Vec3 verts[8] = {
        {-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz}, {-hx, hy, -hz},
        {-hx, -hy, hz},  {hx, -hy, hz},  {hx, hy, hz},  {-hx, hy, hz},
    };
    // Six faces, outward normals, counter-clockwise seen from outside.
    const m3Vec3 normals[6] = {
        {1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f},
    };
    const m3real offsets[6] = {hx, hx, hy, hy, hz, hz};
    const uint8_t loops[6][4] = {
        {1, 2, 6, 5}, // +x
        {0, 4, 7, 3}, // -x
        {2, 3, 7, 6}, // +y
        {0, 1, 5, 4}, // -y
        {4, 5, 6, 7}, // +z
        {0, 3, 2, 1}, // -z
    };

    out->vertexCount = 8;
    out->faceCount = 6;
    out->indexCount = 24;
    memcpy(out->vertices, verts, sizeof(verts));
    memcpy(out->faceNormals, normals, sizeof(normals));
    memcpy(out->faceOffsets, offsets, sizeof(offsets));
    for (int32_t f = 0; f < 6; ++f)
    {
        out->faceVertCounts[f] = 4;
        out->faceVertStart[f] = (uint16_t)(4 * f);
        for (int32_t k = 0; k < 4; ++k)
        {
            out->faceIndices[4 * f + k] = loops[f][k];
        }
    }

    // Closed-form box mass at unit density: m = 8 hx hy hz,
    // I = m/3 diag(hy^2+hz^2, hx^2+hz^2, hx^2+hy^2), centroid at the
    // origin.
    float m = 8.0f * hx * hy * hz;
    out->unitMass = m;
    out->unitCom = (m3Vec3){0.0f, 0.0f, 0.0f};
    out->unitInertiaCom = m3MakeZeroMat3();
    out->unitInertiaCom.cx.x = (m / 3.0f) * (hy * hy + hz * hz);
    out->unitInertiaCom.cy.y = (m / 3.0f) * (hx * hx + hz * hz);
    out->unitInertiaCom.cz.z = (m / 3.0f) * (hx * hx + hy * hy);

    bool built = m3HullBuildHalfEdges(out);
    M3_ASSERT(built); // a box is a closed hull of twelve edges
    (void)built;
}

// Build the half-edge adjacency and the centroid from the face loops.
// Generic over any loop-consistent convex hull (QuickHull finishes
// through this too). Twins are paired at 2k and 2k+1 by construction.
// False, with the edges partly built, when the loops would need more
// half edges than a hull holds: loops that do not close into a hull.
bool m3HullBuildHalfEdges(m3HullData* hull)
{
    // Collect one half edge per (face, loop position), paired with its
    // twin as it is discovered. Directed edge key = origin * 256 + end.
    int32_t count = 0;
    int32_t dirKey[M3_HULL_MAX_HALF_EDGES];
    int32_t dirSlot[M3_HULL_MAX_HALF_EDGES];
    int32_t dirCount = 0;

    // First pass: create half edges face by face, twins interleaved.
    // For each directed edge (a -> b): if the reverse (b -> a) was
    // already created, this one becomes its twin at slot twin+1 XOR.
    int32_t faceEdgeSlot[M3_HULL_MAX_FACE_INDICES];
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        int32_t n = hull->faceVertCounts[f];
        int32_t start = hull->faceVertStart[f];
        for (int32_t k = 0; k < n; ++k)
        {
            int32_t a = hull->faceIndices[start + k];
            int32_t b = hull->faceIndices[start + (k + 1) % n];
            int32_t key = b * 256 + a; // the reverse direction's key
            int32_t slot = -1;
            for (int32_t d = 0; d < dirCount; ++d)
            {
                if (dirKey[d] == key)
                {
                    slot = dirSlot[d] ^ 1; // twin sits beside it
                    break;
                }
            }
            if (slot < 0)
            {
                // A fresh undirected edge claims the next twin pair.
                if (count + 2 > M3_HULL_MAX_HALF_EDGES)
                {
                    return false;
                }
                slot = count;
                count += 2;
                dirKey[dirCount] = a * 256 + b;
                dirSlot[dirCount] = slot;
                dirCount += 1;
            }
            hull->edges[slot].origin = (uint8_t)a;
            hull->edges[slot].twin = (uint16_t)(slot ^ 1);
            hull->edges[slot].face = (uint8_t)f;
            faceEdgeSlot[start + k] = slot;
        }
    }
    // Second pass: next pointers follow each face loop.
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        int32_t n = hull->faceVertCounts[f];
        int32_t start = hull->faceVertStart[f];
        for (int32_t k = 0; k < n; ++k)
        {
            hull->edges[faceEdgeSlot[start + k]].next = (uint16_t)faceEdgeSlot[start + (k + 1) % n];
        }
    }
    hull->edgeCount = count;

    m3Vec3 c = {0.0f, 0.0f, 0.0f};
    for (int32_t v = 0; v < hull->vertexCount; ++v)
    {
        c = m3Add3(c, hull->vertices[v]);
    }
    hull->center = m3MulSV3(1.0f / (m3real)hull->vertexCount, c);
    return true;
}

// Unit-density volume, centroid and inertia about the centroid. Each
// face fans into tetrahedra with the first hull vertex; a tetrahedron
// o, a, b, c with det = a . (b x c) has volume det / 6, centroid (a + b +
// c) / 4 and covariance det / 120 (a a' + b b' + c c' + s s') for s = a +
// b + c, the covariance of the canonical tetrahedron mapped onto it.
bool m3HullMassProperties(m3HullData* hull)
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

static bool Vec3Same(m3Vec3 a, m3Vec3 b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

// Whether the hull is the box m3BuildBoxHull makes of its own extents.
static bool IsItsOwnBox(const m3HullData* hull)
{
    m3Vec3 half = hull->vertices[6];
    if (hull->vertexCount != 8 || hull->faceCount != 6 || !(half.x > 0.0f) || !(half.y > 0.0f) ||
        !(half.z > 0.0f))
    {
        return false; // a box has positive extents
    }
    m3HullData box;
    m3BuildBoxHull(&box, half);
    // Hull data is padding-free, so equal bytes mean an equal hull.
    // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison)
    return memcmp(&box, hull, sizeof(m3HullData)) == 0;
}

// How far a hull's planes may sit from the planes its vertices define.
// QuickHull works its planes in double about a point the hull does not
// store, so a restore cannot rebuild them bit for bit, and it merges
// faces whose vertices lie within its tolerance of one plane: measured,
// a vertex sits up to 8e-6 of the extent off a plane. Far from the
// body's origin the floats themselves are coarse: rounding the
// vertices, the offsets and the check's own dot products moves a
// distance by up to 2.5 ulps of the largest coordinate. The tolerances
// take those with a margin and refuse a plane that misplaces its face
// or leaves a vertex outside the hull.
#define M3_HULL_NORMAL_TOLERANCE   1.0e-3f // | |n| - 1 |
#define M3_HULL_PLANE_TOLERANCE    1.0e-4f // distances, times the hull's extent
#define M3_HULL_ROUNDING_TOLERANCE 16.0f   // distances, in FLT_EPSILON of the largest coordinate

// The distance a vertex may sit off a plane: a share of the hull's
// extent plus the rounding of its largest coordinate.
static m3real PlaneSlack(const m3HullData* hull)
{
    m3Vec3 lo = hull->vertices[0];
    m3Vec3 hi = hull->vertices[0];
    m3real far = 0.0f;
    for (int32_t v = 0; v < hull->vertexCount; ++v)
    {
        m3Vec3 p = hull->vertices[v];
        lo = (m3Vec3){m3MinF(lo.x, p.x), m3MinF(lo.y, p.y), m3MinF(lo.z, p.z)};
        hi = (m3Vec3){m3MaxF(hi.x, p.x), m3MaxF(hi.y, p.y), m3MaxF(hi.z, p.z)};
        far = m3MaxF(far, m3MaxF(m3AbsF(p.x), m3MaxF(m3AbsF(p.y), m3AbsF(p.z))));
    }
    m3real extent = m3MaxF(hi.x - lo.x, m3MaxF(hi.y - lo.y, hi.z - lo.z));
    return M3_HULL_PLANE_TOLERANCE * extent + M3_HULL_ROUNDING_TOLERANCE * FLT_EPSILON * far;
}

// Each face's plane has a unit normal, holds the face's own vertices and
// has every vertex of the hull on or below it, within the tolerances.
static bool PlanesHold(const m3HullData* hull)
{
    m3real slack = PlaneSlack(hull);
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3Vec3 n = hull->faceNormals[f];
        m3real offset = hull->faceOffsets[f];
        m3real length = m3Length3(n);
        if (!(length >= 1.0f - M3_HULL_NORMAL_TOLERANCE &&
              length <= 1.0f + M3_HULL_NORMAL_TOLERANCE) ||
            !m3FiniteF(offset))
        {
            return false;
        }
        for (int32_t v = 0; v < hull->vertexCount; ++v)
        {
            if (!(m3Dot3(n, hull->vertices[v]) - offset <= slack))
            {
                return false; // a vertex above the plane
            }
        }
        const uint8_t* loop = &hull->faceIndices[hull->faceVertStart[f]];
        for (int32_t k = 0; k < hull->faceVertCounts[f]; ++k)
        {
            if (!(m3Dot3(n, hull->vertices[loop[k]]) - offset >= -slack))
            {
                return false; // the face's own vertex off its plane
            }
        }
    }
    return true;
}

// The unit mass properties are derived from the vertices and loops: they
// must be what the builders make of them, bit for bit, and the planes
// must hold.
static bool DerivedMatch(const m3HullData* hull)
{
    if (!(hull->unitMass > 0.0f) || hull->unitMass > M3_UNIT_MASS_LIMIT)
    {
        return false; // a box's closed form and the integrator alike
    }
    if (IsItsOwnBox(hull))
    {
        return true; // a box's planes and mass are closed form
    }
    m3HullData rebuilt = *hull;
    if (!PlanesHold(hull) || !m3HullMassProperties(&rebuilt))
    {
        return false;
    }
    return rebuilt.unitMass == hull->unitMass && Vec3Same(rebuilt.unitCom, hull->unitCom) &&
           Vec3Same(rebuilt.unitInertiaCom.cx, hull->unitInertiaCom.cx) &&
           Vec3Same(rebuilt.unitInertiaCom.cy, hull->unitInertiaCom.cy) &&
           Vec3Same(rebuilt.unitInertiaCom.cz, hull->unitInertiaCom.cz);
}

bool m3HullDataValid(const m3HullData* hull)
{
    int32_t vc = hull->vertexCount;
    int32_t fc = hull->faceCount;
    int32_t ic = hull->indexCount;
    if (vc < 4 || vc > M3_HULL_MAX_VERTS || fc < 4 || fc > M3_HULL_MAX_FACES || ic < 3 * fc ||
        ic > M3_HULL_MAX_FACE_INDICES || hull->edgeCount != ic)
    {
        return false;
    }
    for (int32_t v = 0; v < vc; ++v)
    {
        if (!m3LocalV3(hull->vertices[v]))
        {
            return false; // a body-frame offset, like every local position
        }
    }
    // Faces own consecutive runs of the index list, each a loop of at
    // least three vertices of this hull.
    int32_t start = 0;
    for (int32_t f = 0; f < fc; ++f)
    {
        if (hull->faceVertCounts[f] < 3 || hull->faceVertStart[f] != start)
        {
            return false;
        }
        start += hull->faceVertCounts[f];
    }
    if (start != ic)
    {
        return false;
    }
    for (int32_t i = 0; i < ic; ++i)
    {
        if (hull->faceIndices[i] >= vc)
        {
            return false;
        }
    }
    if (!DerivedMatch(hull))
    {
        return false;
    }
    // The adjacency and the center are derived from the loops: they must
    // be what the builder makes of them. The rebuilt edges start as 0xFF,
    // an origin no hull has, so a slot the builder leaves unwritten (loops
    // that repeat a directed edge) cannot match whatever the input holds.
    m3HullData rebuilt = *hull;
    memset(rebuilt.edges, 0xFF, sizeof(rebuilt.edges));
    if (!m3HullBuildHalfEdges(&rebuilt) || rebuilt.edgeCount != ic)
    {
        return false;
    }
    for (int32_t e = 0; e < ic; ++e)
    {
        if (rebuilt.edges[e].origin >= vc)
        {
            return false;
        }
    }
    if (memcmp(rebuilt.edges, hull->edges, (size_t)ic * sizeof(m3HullHalfEdge)) != 0)
    {
        return false;
    }
    return rebuilt.center.x == hull->center.x && rebuilt.center.y == hull->center.y &&
           rebuilt.center.z == hull->center.z;
}

int32_t m3InternHull(m3World* world, const m3HullData* data)
{
    // Content dedupe in ascending slot order: identical hulls share
    // one slot, deterministically.
    int32_t maxIndex = world->hulls.hullPool.maxIndex;
    for (int32_t i = 0; i < maxIndex; ++i)
    {
        if (world->hulls.hullPool.alive[i] == 0)
        {
            continue;
        }
        // Hull data is padding-free, so equal bytes mean an equal hull.
        // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison)
        if (memcmp(&world->hulls.hullData[i], data, sizeof(m3HullData)) == 0)
        {
            world->hulls.hullRefCounts[i] += 1;
            return i;
        }
    }
    int32_t index = m3IdPoolAlloc(&world->hulls.hullPool);
    if (index < 0)
    {
        return -1; // exhausted: loud at the caller
    }
    world->hulls.hullData[index] = *data;
    world->hulls.hullRefCounts[index] = 1;
    return index;
}

void m3ReleaseHull(m3World* world, int32_t hullIndex)
{
    if (hullIndex < 0)
    {
        return;
    }
    world->hulls.hullRefCounts[hullIndex] -= 1;
    if (world->hulls.hullRefCounts[hullIndex] == 0)
    {
        memset(&world->hulls.hullData[hullIndex], 0, sizeof(m3HullData));
        m3IdPoolFree(&world->hulls.hullPool, hullIndex);
    }
}
