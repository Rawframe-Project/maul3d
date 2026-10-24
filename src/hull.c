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

static bool Mat3Finite(const m3Mat3* m)
{
    return m3FiniteV3(m->cx) && m3FiniteV3(m->cy) && m3FiniteV3(m->cz);
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
        if (!m3FiniteV3(hull->vertices[v]))
        {
            return false;
        }
    }
    // Faces own consecutive runs of the index list, each a loop of at
    // least three vertices of this hull.
    int32_t start = 0;
    for (int32_t f = 0; f < fc; ++f)
    {
        if (!m3FiniteV3(hull->faceNormals[f]) || !m3FiniteF(hull->faceOffsets[f]) ||
            hull->faceVertCounts[f] < 3 || hull->faceVertStart[f] != start)
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
    if (!m3FiniteF(hull->unitMass) || !m3FiniteV3(hull->unitCom) ||
        !Mat3Finite(&hull->unitInertiaCom))
    {
        return false;
    }
    // The adjacency and the center are derived from the loops: they must
    // be what the builder makes of them.
    m3HullData rebuilt = *hull;
    if (!m3HullBuildHalfEdges(&rebuilt) || rebuilt.edgeCount != ic ||
        memcmp(rebuilt.edges, hull->edges, (size_t)ic * sizeof(m3HullHalfEdge)) != 0)
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
