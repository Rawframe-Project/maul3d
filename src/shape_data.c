// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mesh and height field content: allocation, release, edge flags and
// the height field triangle gather.

#include "body.h"
#include "broad_phase.h"
#include "hull.h"
#include "journal.h"
#include "manifold.h"
#include "quickhull.h"
#include "shape.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

int32_t m3MeshEdgeScratchCount(const m3MeshData* mesh)
{
    return 2 * (mesh->vertexCount + 1) + 3 * mesh->triangleCount;
}

// The first other triangle, by index, that shares the edge v1 v2 in
// either direction, from v1's list of triangles; its vertex opposite the
// edge, or -1 when the edge is a boundary.
static int32_t OppositeVertex(const m3MeshData* mesh, const int32_t* start, const int32_t* list,
                              int32_t t, int32_t v1, int32_t v2)
{
    for (int32_t i = start[v1]; i < start[v1 + 1]; ++i)
    {
        int32_t u = list[i];
        if (u == t)
        {
            continue;
        }
        for (int32_t j = 0; j < 3; ++j)
        {
            int32_t w1 = mesh->indices[3 * u + j];
            int32_t w2 = mesh->indices[3 * u + (j + 1) % 3];
            if ((w1 == v2 && w2 == v1) || (w1 == v1 && w2 == v2))
            {
                return mesh->indices[3 * u + (j + 2) % 3];
            }
        }
    }
    return -1;
}

// An edge is a real contact feature when it is a boundary or a convex
// ridge: the neighbor's far vertex lies below this triangle's plane.
// Flat and concave edges stay clear, ghost candidates for the welding.
// Each vertex lists the triangles that use it in ascending order (a
// counting sort into scratch), so the neighbor across an edge is found
// among the few triangles around its first vertex.
void m3BakeMeshEdgeFlags(m3MeshData* mesh, int32_t* scratch)
{
    const m3real tol = 0.005f;
    int32_t vertexCount = mesh->vertexCount;
    int32_t triCount = mesh->triangleCount;
    int32_t* start = scratch;
    int32_t* cursor = scratch + vertexCount + 1;
    int32_t* list = cursor + vertexCount + 1;
    memset(start, 0, (size_t)(vertexCount + 1) * sizeof(int32_t));
    for (int32_t i = 0; i < 3 * triCount; ++i)
    {
        start[mesh->indices[i] + 1] += 1;
    }
    for (int32_t v = 0; v < vertexCount; ++v)
    {
        start[v + 1] += start[v];
        cursor[v] = start[v];
    }
    for (int32_t i = 0; i < 3 * triCount; ++i)
    {
        list[cursor[mesh->indices[i]]++] = i / 3;
    }
    for (int32_t t = 0; t < triCount; ++t)
    {
        m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
        m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
        m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
        m3Vec3 n = m3Normalize3(m3Cross3(m3Sub3(b, a), m3Sub3(c, a)));
        m3real off = m3Dot3(n, a);
        mesh->edgeFlags[t] = 0;
        for (int32_t k = 0; k < 3; ++k)
        {
            int32_t far = OppositeVertex(mesh, start, list, t, mesh->indices[3 * t + k],
                                         mesh->indices[3 * t + (k + 1) % 3]);
            if (far < 0 || m3Dot3(n, mesh->vertices[far]) - off < -tol)
            {
                mesh->edgeFlags[t] |= (uint8_t)(1 << k);
            }
        }
    }
}

bool m3HeightFieldDataAlloc(m3HeightFieldData* hf)
{
    m3Free(hf->heights);
    hf->heights = NULL;
    if (hf->nx <= 0 || hf->nz <= 0)
    {
        return hf->nx == 0 && hf->nz == 0; // an empty slot is legal
    }
    hf->heights = (float*)m3AllocZeroed(hf->nx * hf->nz * (int32_t)sizeof(float));
    if (hf->heights == NULL)
    {
        m3HeightFieldDataFree(hf);
        return false;
    }
    return true;
}

void m3HeightFieldDataFree(m3HeightFieldData* hf)
{
    m3Free(hf->heights);
    memset(hf, 0, sizeof(*hf));
}

int32_t m3HeightFieldGather(const m3HeightFieldData* hf, m3Vec3 lo, m3Vec3 hi, m3Vec3 (*tris)[3],
                            int32_t cap)
{
    m3real inv = 1.0f / hf->cellSize;
    int32_t cx0 = m3CellFromF(floorf(lo.x * inv), 2.0e9f);
    int32_t cx1 = m3CellFromF(floorf(hi.x * inv), -2.0e9f);
    int32_t cz0 = m3CellFromF(floorf(lo.z * inv), 2.0e9f);
    int32_t cz1 = m3CellFromF(floorf(hi.z * inv), -2.0e9f);
    cx0 = cx0 < 0 ? 0 : cx0;
    cz0 = cz0 < 0 ? 0 : cz0;
    cx1 = cx1 > hf->nx - 2 ? hf->nx - 2 : cx1;
    cz1 = cz1 > hf->nz - 2 ? hf->nz - 2 : cz1;
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

bool m3MeshDataAlloc(m3MeshData* mesh)
{
    m3Free(mesh->vertices);
    m3Free(mesh->indices);
    m3Free(mesh->edgeFlags);
    m3Free(mesh->triMaterials);
    mesh->vertices = NULL;
    mesh->indices = NULL;
    mesh->edgeFlags = NULL;
    mesh->triMaterials = NULL;
    if (mesh->vertexCount <= 0 || mesh->triangleCount <= 0)
    {
        return mesh->vertexCount == 0 && mesh->triangleCount == 0; // an empty slot is legal
    }
    mesh->vertices = (m3Vec3*)m3AllocZeroed(mesh->vertexCount * (int32_t)sizeof(m3Vec3));
    mesh->indices = (uint16_t*)m3AllocZeroed(3 * mesh->triangleCount * (int32_t)sizeof(uint16_t));
    mesh->edgeFlags = (uint8_t*)m3AllocZeroed(mesh->triangleCount);
    // Material groups ride beside the content: all-zero
    // bytes and count 0 ARE the "shape material everywhere" state.
    mesh->triMaterials = (uint8_t*)m3AllocZeroed(mesh->triangleCount);
    if (mesh->vertices == NULL || mesh->indices == NULL || mesh->edgeFlags == NULL ||
        mesh->triMaterials == NULL)
    {
        m3MeshDataFree(mesh);
        return false;
    }
    return true;
}

void m3MeshDataFree(m3MeshData* mesh)
{
    m3Free(mesh->vertices);
    m3Free(mesh->indices);
    m3Free(mesh->edgeFlags);
    m3Free(mesh->triMaterials);
    memset(mesh, 0, sizeof(*mesh));
}
