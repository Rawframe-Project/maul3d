// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mesh and height field content: allocation, release, edge flags and
// the height field triangle gather.

#include "world_internal.h"

#include <string.h>

int32_t m3MeshEdgeScratchCount(const m3MeshData* mesh)
{
    return 2 * mesh->vertexCount + 1 + 3 * mesh->triangleCount;
}

// Edge e is triangle e / 3's edge e % 3, from its vertex e % 3 to the
// next; its lower and higher vertex.
static int32_t EdgeLo(const m3MeshData* mesh, int32_t e)
{
    int32_t a = mesh->indices[e];
    int32_t b = mesh->indices[3 * (e / 3) + (e % 3 + 1) % 3];
    return a < b ? a : b;
}

static int32_t EdgeHi(const m3MeshData* mesh, int32_t e)
{
    int32_t a = mesh->indices[e];
    int32_t b = mesh->indices[3 * (e / 3) + (e % 3 + 1) % 3];
    return a < b ? b : a;
}

// Sets edge e's flag bit when it is a boundary (far < 0) or a convex
// ridge: the neighbor's far vertex lies below its triangle's plane.
static void FlagEdge(m3MeshData* mesh, int32_t e, int32_t far)
{
    const float tol = 0.005f;
    int32_t t = e / 3;
    m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
    m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
    m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
    m3Vec3 n = m3Normalize3(m3Cross3(m3Sub3(b, a), m3Sub3(c, a)));
    float off = m3Dot3(n, a);
    if (far < 0 || m3Dot3(n, mesh->vertices[far]) - off < -tol)
    {
        mesh->edgeFlags[t] |= (uint8_t)(1 << (e % 3));
    }
}

// An edge is a real contact feature when it is a boundary or a convex
// ridge: the neighbor's far vertex lies below this triangle's plane.
// Flat and concave edges stay clear, ghost candidates for the welding.
// The neighbor across an edge is the lowest other triangle that has the
// same two vertices, at its first such edge, so the result never depends
// on how many triangles meet at a vertex. The edges are counted into
// buckets by their lower vertex, in edge order; within a bucket, first
// and second hold, per higher vertex, the first edge and the first edge
// of another triangle. Linear in the vertices and triangles: the scratch
// is the bucket counts, reused as first once the edges are placed, the
// edge order and second.
void m3BakeMeshEdgeFlags(m3MeshData* mesh, int32_t* scratch)
{
    int32_t vertexCount = mesh->vertexCount;
    int32_t edgeCount = 3 * mesh->triangleCount;
    int32_t* count = scratch;
    int32_t* order = scratch + vertexCount + 1;
    int32_t* second = order + edgeCount;
    memset(count, 0, (size_t)(vertexCount + 1) * sizeof(int32_t));
    for (int32_t e = 0; e < edgeCount; ++e)
    {
        count[EdgeLo(mesh, e) + 1] += 1;
    }
    for (int32_t v = 0; v < vertexCount; ++v)
    {
        count[v + 1] += count[v];
    }
    for (int32_t e = 0; e < edgeCount; ++e)
    {
        order[count[EdgeLo(mesh, e)]++] = e;
    }
    int32_t* first = count;
    memset(first, 0xFF, (size_t)vertexCount * sizeof(int32_t));
    memset(second, 0xFF, (size_t)vertexCount * sizeof(int32_t));
    memset(mesh->edgeFlags, 0, (size_t)mesh->triangleCount);
    for (int32_t begin = 0; begin < edgeCount;)
    {
        int32_t lo = EdgeLo(mesh, order[begin]);
        int32_t end = begin + 1;
        while (end < edgeCount && EdgeLo(mesh, order[end]) == lo)
        {
            end += 1;
        }
        for (int32_t i = begin; i < end; ++i)
        {
            int32_t e = order[i];
            int32_t hi = EdgeHi(mesh, e);
            if (first[hi] < 0)
            {
                first[hi] = e;
            }
            else if (second[hi] < 0 && e / 3 != first[hi] / 3)
            {
                second[hi] = e;
            }
        }
        for (int32_t i = begin; i < end; ++i)
        {
            int32_t e = order[i];
            int32_t hi = EdgeHi(mesh, e);
            int32_t other = e / 3 != first[hi] / 3 ? first[hi] : second[hi];
            int32_t far = other < 0 ? -1 : mesh->indices[3 * (other / 3) + (other % 3 + 2) % 3];
            FlagEdge(mesh, e, far);
        }
        for (int32_t i = begin; i < end; ++i)
        {
            int32_t hi = EdgeHi(mesh, order[i]);
            first[hi] = -1;
            second[hi] = -1;
        }
        begin = end;
    }
}

m3Result m3HeightFieldSamplesCheck(int32_t nx, int32_t nz, float cellSize, const void* samples,
                                   float* minOut, float* maxOut)
{
    if (nx < 2 || nz < 2)
    {
        return m3_errorInvalid;
    }
    m3Result size = m3BothChecks(m3BothChecks(m3CheckCount(nx, M3_MAX_HEIGHTFIELD_SIZE),
                                              m3CheckCount(nz, M3_MAX_HEIGHTFIELD_SIZE)),
                                 m3CheckPositive(cellSize, M3_CELL_LIMIT));
    if (size != m3_success)
    {
        return size;
    }
    // Read by memcpy: journal and snapshot bytes are packed.
    const uint8_t* bytes = (const uint8_t*)samples;
    float lo = 0.0f;
    float hi = 0.0f;
    for (int32_t i = 0; i < nx * nz; ++i)
    {
        float v;
        memcpy(&v, bytes + (size_t)i * sizeof(float), sizeof(v));
        m3Result sample = m3CheckWithin(v, M3_GEOMETRY_LIMIT);
        if (sample != m3_success)
        {
            return sample;
        }
        lo = i == 0 ? v : (v < lo ? v : lo);
        hi = i == 0 ? v : (v > hi ? v : hi);
    }
    *minOut = lo;
    *maxOut = hi;
    return m3_success;
}

bool m3HeightFieldDataAlloc(m3Heap* heap, m3HeightFieldData* hf)
{
    m3Free(heap, hf->heights);
    hf->heights = NULL;
    if (hf->nx <= 0 || hf->nz <= 0)
    {
        return hf->nx == 0 && hf->nz == 0; // an empty slot is legal
    }
    hf->heights = (float*)m3AllocZeroed(heap, hf->nx * hf->nz * (int32_t)sizeof(float));
    if (hf->heights == NULL)
    {
        m3HeightFieldDataFree(heap, hf);
        return false;
    }
    return true;
}

void m3HeightFieldDataFree(m3Heap* heap, m3HeightFieldData* hf)
{
    m3Free(heap, hf->heights);
    memset(hf, 0, sizeof(*hf));
}

// The cells a local-frame box covers in x and z, clamped to the grid:
// cells[0] to cells[1] in x, cells[2] to cells[3] in z. A NaN corner
// gives an empty range.
static void CellRange(const m3HeightFieldData* hf, m3Vec3 lo, m3Vec3 hi, int32_t cells[4])
{
    float inv = 1.0f / hf->cellSize;
    int32_t cx0 = m3CellFromF(floorf(lo.x * inv), 2.0e9f);
    int32_t cx1 = m3CellFromF(floorf(hi.x * inv), -2.0e9f);
    int32_t cz0 = m3CellFromF(floorf(lo.z * inv), 2.0e9f);
    int32_t cz1 = m3CellFromF(floorf(hi.z * inv), -2.0e9f);
    cells[0] = cx0 < 0 ? 0 : cx0;
    cells[1] = cx1 > hf->nx - 2 ? hf->nx - 2 : cx1;
    cells[2] = cz0 < 0 ? 0 : cz0;
    cells[3] = cz1 > hf->nz - 2 ? hf->nz - 2 : cz1;
}

void m3HeightFieldVisit(const m3HeightFieldData* hf, m3Vec3 lo, m3Vec3 hi, m3TriangleVisit* visit,
                        void* context)
{
    int32_t cells[4];
    CellRange(hf, lo, hi, cells);
    for (int32_t cz = cells[2]; cz <= cells[3]; ++cz)
    {
        for (int32_t cx = cells[0]; cx <= cells[1]; ++cx)
        {
            m3Vec3 cell[2][3];
            m3HeightFieldCellTris(hf, cx, cz, cell);
            for (int32_t t = 0; t < 2; ++t)
            {
                if (m3TriangleTouchesBox(cell[t], lo, hi) && !visit(cell[t], context))
                {
                    return;
                }
            }
        }
    }
}

// The box of a path's points from fraction t0 to t1, padded by its reach.
static void PathStretchBox(const m3SurfacePath* path, double t0, double t1, m3Vec3* lo, m3Vec3* hi)
{
    const double s[3] = {(double)path->start.x, (double)path->start.y, (double)path->start.z};
    const double e[3] = {(double)path->end.x, (double)path->end.y, (double)path->end.z};
    double r = (double)path->reach;
    float l[3];
    float h[3];
    for (int32_t k = 0; k < 3; ++k)
    {
        double a = s[k] + t0 * (e[k] - s[k]);
        double b = s[k] + t1 * (e[k] - s[k]);
        l[k] = (float)((a < b ? a : b) - r);
        h[k] = (float)((a < b ? b : a) + r);
    }
    *lo = (m3Vec3){l[0], l[1], l[2]};
    *hi = (m3Vec3){h[0], h[1], h[2]};
}

void m3HeightFieldVisitPath(const m3HeightFieldData* hf, const m3SurfacePath* path,
                            m3TriangleVisit* visit, void* context)
{
    m3Vec3 lo;
    m3Vec3 hi;
    PathStretchBox(path, 0.0, (double)path->until, &lo, &hi);
    int32_t rows[4];
    CellRange(hf, lo, hi, rows);
    double s = (double)path->start.z;
    double d = (double)path->end.z - s;
    double r = (double)path->reach;
    double cs = (double)hf->cellSize;
    for (int32_t cz = rows[2]; cz <= rows[3]; ++cz)
    {
        // The stretch of the path within reach of this row of cells: a
        // triangle in the row is met, if at all, while the path is there.
        double t0 = 0.0;
        double t1 = (double)path->until;
        if (!m3ClipSlab(s, d, (double)cz * cs - r, (double)(cz + 1) * cs + r, &t0, &t1))
        {
            continue;
        }
        m3Vec3 rowLo;
        m3Vec3 rowHi;
        PathStretchBox(path, t0, t1, &rowLo, &rowHi);
        int32_t cells[4];
        CellRange(hf, rowLo, rowHi, cells);
        for (int32_t cx = cells[0]; cx <= cells[1]; ++cx)
        {
            m3Vec3 cell[2][3];
            m3HeightFieldCellTris(hf, cx, cz, cell);
            for (int32_t t = 0; t < 2; ++t)
            {
                if (m3TriangleTouchesBox(cell[t], rowLo, rowHi) && !visit(cell[t], context))
                {
                    return;
                }
            }
        }
    }
}

int32_t m3HeightFieldGather(const m3HeightFieldData* hf, m3Vec3 lo, m3Vec3 hi, m3Vec3 (*tris)[3],
                            int32_t cap)
{
    int32_t cells[4];
    CellRange(hf, lo, hi, cells);
    int32_t cx0 = cells[0];
    int32_t cx1 = cells[1];
    int32_t cz0 = cells[2];
    int32_t cz1 = cells[3];
    int32_t count = 0;
    for (int32_t cz = cz0; cz <= cz1; ++cz)
    {
        for (int32_t cx = cx0; cx <= cx1; ++cx)
        {
            if (count + 2 > cap)
            {
                return count; // bounded by contract
            }
            m3Vec3 cell[2][3];
            m3HeightFieldCellTris(hf, cx, cz, cell);
            for (int32_t t = 0; t < 2; ++t)
            {
                tris[count][0] = cell[t][0];
                tris[count][1] = cell[t][1];
                tris[count][2] = cell[t][2];
                count += 1;
            }
        }
    }
    return count;
}

bool m3MeshDataAlloc(m3Heap* heap, m3MeshData* mesh)
{
    m3Free(heap, mesh->vertices);
    m3Free(heap, mesh->indices);
    m3Free(heap, mesh->edgeFlags);
    m3Free(heap, mesh->triMaterials);
    mesh->vertices = NULL;
    mesh->indices = NULL;
    mesh->edgeFlags = NULL;
    mesh->triMaterials = NULL;
    if (mesh->vertexCount <= 0 || mesh->triangleCount <= 0)
    {
        return mesh->vertexCount == 0 && mesh->triangleCount == 0; // an empty slot is legal
    }
    mesh->vertices = (m3Vec3*)m3AllocZeroed(heap, mesh->vertexCount * (int32_t)sizeof(m3Vec3));
    mesh->indices =
        (uint16_t*)m3AllocZeroed(heap, 3 * mesh->triangleCount * (int32_t)sizeof(uint16_t));
    mesh->edgeFlags = (uint8_t*)m3AllocZeroed(heap, mesh->triangleCount);
    // Material groups ride beside the content: all-zero
    // bytes and count 0 ARE the "shape material everywhere" state.
    mesh->triMaterials = (uint8_t*)m3AllocZeroed(heap, mesh->triangleCount);
    if (mesh->vertices == NULL || mesh->indices == NULL || mesh->edgeFlags == NULL ||
        mesh->triMaterials == NULL)
    {
        m3MeshDataFree(heap, mesh);
        return false;
    }
    return true;
}

void m3MeshDataFree(m3Heap* heap, m3MeshData* mesh)
{
    m3Free(heap, mesh->vertices);
    m3Free(heap, mesh->indices);
    m3Free(heap, mesh->edgeFlags);
    m3Free(heap, mesh->triMaterials);
    memset(mesh, 0, sizeof(*mesh));
}
