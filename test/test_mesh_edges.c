// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mesh edge flag bake against a reference: the scan the bake used to
// run, which looks for the triangle across each edge among every
// triangle at the edge's first vertex. The library's bake buckets the
// edges instead, so its work does not grow with a vertex's valence; its
// flags must be the scan's, byte for byte, on fans, ridged grids and
// random soups with repeated vertices, duplicated triangles and
// degenerate ones. Its scratch is never larger than the scan's was.

#include "test_harness.h"
#include "world_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    MAX_VERTICES = 8200,
    MAX_TRIANGLES = 8192
};

static m3Vec3 s_vertices[MAX_VERTICES];
static uint16_t s_indices[3 * MAX_TRIANGLES];
static uint8_t s_flags[MAX_TRIANGLES];
static uint8_t s_reference[MAX_TRIANGLES];
static int32_t s_scratch[2 * (MAX_VERTICES + 1) + 3 * MAX_TRIANGLES];

static uint32_t s_seed = 0x9E3779B9u;

static uint32_t Next(void)
{
    s_seed ^= s_seed << 13;
    s_seed ^= s_seed >> 17;
    s_seed ^= s_seed << 5;
    return s_seed;
}

static float Unit(void)
{
    return (float)(Next() & 0xFFFF) / 65535.0f;
}

// The reference: the first other triangle, by index, that shares the
// edge v1 v2 in either direction, from v1's list of triangles; its vertex
// opposite the edge, or -1 when the edge is a boundary.
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

static void ReferenceBake(const m3MeshData* mesh, uint8_t* flags, int32_t* scratch)
{
    const float tol = 0.005f;
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
        float off = m3Dot3(n, a);
        flags[t] = 0;
        for (int32_t k = 0; k < 3; ++k)
        {
            int32_t far = OppositeVertex(mesh, start, list, t, mesh->indices[3 * t + k],
                                         mesh->indices[3 * t + (k + 1) % 3]);
            if (far < 0 || m3Dot3(n, mesh->vertices[far]) - off < -tol)
            {
                flags[t] |= (uint8_t)(1 << k);
            }
        }
    }
}

// Both bakes of the mesh in the shared arrays; whether they agree.
static bool BakesAgree(int32_t vertexCount, int32_t triangleCount)
{
    m3MeshData mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.vertexCount = vertexCount;
    mesh.triangleCount = triangleCount;
    mesh.vertices = s_vertices;
    mesh.indices = s_indices;
    mesh.edgeFlags = s_flags;
    int32_t scanScratch = 2 * (vertexCount + 1) + 3 * triangleCount;
    CHECK(m3MeshEdgeScratchCount(&mesh) <= scanScratch, "the bake's scratch is no larger");
    memset(s_flags, 0xAA, sizeof(s_flags));
    m3BakeMeshEdgeFlags(&mesh, s_scratch);
    ReferenceBake(&mesh, s_reference, s_scratch);
    return memcmp(s_flags, s_reference, (size_t)triangleCount) == 0;
}

// A fan of count triangles around vertex 0, its rim rising and falling
// so some edges are ridges and some are valleys.
static int32_t Fan(int32_t count)
{
    s_vertices[0] = (m3Vec3){0.0f, 0.0f, 0.0f};
    for (int32_t i = 1; i < count + 2; ++i)
    {
        float x = (float)(i % 64) - 32.0f;
        float z = (float)(i / 64) + 1.0f;
        s_vertices[i] = (m3Vec3){x, (float)(i % 3) - 1.0f, z};
    }
    for (int32_t t = 0; t < count; ++t)
    {
        s_indices[3 * t + 0] = 0;
        s_indices[3 * t + 1] = (uint16_t)(t + 2);
        s_indices[3 * t + 2] = (uint16_t)(t + 1);
    }
    return count + 2;
}

// An n by n vertex grid with ridges, two triangles per cell, its
// diagonal alternating.
static int32_t Grid(int32_t n)
{
    for (int32_t z = 0; z < n; ++z)
    {
        for (int32_t x = 0; x < n; ++x)
        {
            float y = (float)((x * 7 + z * 3) % 5) * 0.3f;
            s_vertices[z * n + x] = (m3Vec3){(float)x, y, (float)z};
        }
    }
    int32_t t = 0;
    for (int32_t z = 0; z + 1 < n; ++z)
    {
        for (int32_t x = 0; x + 1 < n; ++x)
        {
            uint16_t a = (uint16_t)(z * n + x);
            uint16_t b = (uint16_t)(a + 1);
            uint16_t c = (uint16_t)(a + n);
            uint16_t d = (uint16_t)(a + n + 1);
            uint16_t tri[6] = {a, c, b, b, c, d};
            if ((x + z) % 2 != 0)
            {
                uint16_t flipped[6] = {a, c, d, a, d, b};
                memcpy(tri, flipped, sizeof(tri));
            }
            memcpy(&s_indices[3 * t], tri, sizeof(tri));
            t += 2;
        }
    }
    return t;
}

// count random triangles over vertexCount random vertices: with so few
// vertices, edges are shared by many triangles, some triangles repeat a
// vertex and some repeat whole.
static void Soup(int32_t vertexCount, int32_t count)
{
    for (int32_t v = 0; v < vertexCount; ++v)
    {
        s_vertices[v] = (m3Vec3){Unit() * 4.0f, Unit() * 4.0f, Unit() * 4.0f};
    }
    for (int32_t i = 0; i < 3 * count; ++i)
    {
        s_indices[i] = (uint16_t)(Next() % (uint32_t)vertexCount);
    }
    for (int32_t t = 1; t < count; t += 7)
    {
        memcpy(&s_indices[3 * t], &s_indices[3 * (t - 1)], 3 * sizeof(uint16_t));
    }
}

static void TestFans(void)
{
    const int32_t counts[] = {1, 2, 3, 17, 500, MAX_TRIANGLES};
    for (size_t k = 0; k < sizeof(counts) / sizeof(counts[0]); ++k)
    {
        int32_t vertices = Fan(counts[k]);
        char what[96];
        snprintf(what, sizeof(what), "a fan of %d triangles bakes as the scan", (int)counts[k]);
        CHECK(BakesAgree(vertices, counts[k]), what);
    }
}

static void TestGrids(void)
{
    const int32_t sizes[] = {2, 3, 9, 64};
    for (size_t k = 0; k < sizeof(sizes) / sizeof(sizes[0]); ++k)
    {
        int32_t triangles = Grid(sizes[k]);
        char what[96];
        snprintf(what, sizeof(what), "a %d by %d grid bakes as the scan", (int)sizes[k],
                 (int)sizes[k]);
        CHECK(BakesAgree(sizes[k] * sizes[k], triangles), what);
    }
}

static void TestSoups(void)
{
    bool agree = true;
    for (int32_t round = 0; round < 200 && agree; ++round)
    {
        int32_t vertexCount = 3 + (int32_t)(Next() % 40u);
        int32_t count = 1 + (int32_t)(Next() % 400u);
        Soup(vertexCount, count);
        agree = BakesAgree(vertexCount, count);
    }
    CHECK(agree, "200 random soups bake as the scan");
}

int main(void)
{
    TestFans();
    TestGrids();
    TestSoups();
    if (s_failures == 0)
    {
        printf("test_mesh_edges: all passed\n");
        return 0;
    }
    printf("test_mesh_edges: %d FAILURES\n", s_failures);
    return 1;
}
