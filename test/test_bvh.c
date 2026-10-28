// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mesh BVH gate: the brute-force referee. The tree is a pruning
// layer, nothing more: for any query box the gathered set, filtered by
// the exact per-triangle test, must equal the full scan filtered by the
// same test. Plus the degenerate boxes. The path gathers of the tree and
// of a height field grid keep every triangle whose box a path comes
// within reach of, and leave most of a long diagonal's box out.

#include "helpers.h"
#include "test_harness.h"
#include "world.h"
#include "world_internal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Deterministic LCG (never libc rand: implementation-defined).
static uint32_t s_rng = 0x12345678u;
static uint32_t NextRand(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return s_rng;
}
static float RandRange(float lo, float hi)
{
    return lo + (hi - lo) * ((float)(NextRand() >> 8) / 16777216.0f);
}

// A bumpy heightfield: 1922 triangles, the realistic midphase load.
static m3World* MakeMeshWorld(m3ShapeId* meshShapeOut)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.meshCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    float heights[32 * 32];
    for (int32_t z = 0; z < 32; ++z)
    {
        for (int32_t x = 0; x < 32; ++x)
        {
            // A deterministic bumpscape (no libm beyond the allowed).
            heights[z * 32 + x] = 0.25f * (float)((x * 7 + z * 13) % 5) - 0.5f;
        }
    }
    m3ShapeId meshShape = NewHeightFieldShape(world, ground, &sd, heights, 32, 32, 1.0f);
    if (meshShapeOut != NULL)
    {
        *meshShapeOut = meshShape;
    }
    return world;
}

static const m3MeshData* MeshOf(m3World* worldId, m3ShapeId shape)
{
    m3World* world = worldId;
    return &world->meshes.meshData[world->shapes.shapeMeshIndex[shape.index1 - 1]];
}

static const m3MeshBvh* BvhOf(m3World* worldId, m3ShapeId shape)
{
    m3World* world = worldId;
    return &world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape.index1 - 1]];
}

static void TriBox(const m3MeshData* mesh, int32_t t, m3Vec3* lo, m3Vec3* hi)
{
    m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
    m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
    m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
    *lo = (m3Vec3){m3MinF(a.x, m3MinF(b.x, c.x)), m3MinF(a.y, m3MinF(b.y, c.y)),
                   m3MinF(a.z, m3MinF(b.z, c.z))};
    *hi = (m3Vec3){m3MaxF(a.x, m3MaxF(b.x, c.x)), m3MaxF(a.y, m3MaxF(b.y, c.y)),
                   m3MaxF(a.z, m3MaxF(b.z, c.z))};
}

static bool BoxesOverlap(m3Vec3 alo, m3Vec3 ahi, m3Vec3 blo, m3Vec3 bhi)
{
    return !(ahi.x < blo.x || alo.x > bhi.x || ahi.y < blo.y || alo.y > bhi.y || ahi.z < blo.z ||
             alo.z > bhi.z);
}

// A gather's hits read out in ascending order, as the callers walk them.
static int32_t ReadHits(const m3MeshBvhHits* hits, uint16_t* out)
{
    int32_t count = 0;
    for (int32_t t = m3MeshBvhFirstHit(hits); t >= 0; t = m3MeshBvhNextHit(hits, t))
    {
        out[count++] = (uint16_t)t;
    }
    CHECK(count == hits->count, "the hits read out as many items as were found");
    return count;
}

static int32_t Gather(const m3MeshBvh* bvh, m3Vec3 lo, m3Vec3 hi, uint16_t* out)
{
    m3MeshBvhHits hits;
    m3MeshBvhGather(bvh, lo, hi, &hits);
    return ReadHits(&hits, out);
}

static int32_t GatherPath(const m3MeshBvh* bvh, const m3SurfacePath* path, uint16_t* out)
{
    m3MeshBvhHits hits;
    m3MeshBvhGatherPath(bvh, path, &hits);
    return ReadHits(&hits, out);
}

static void TestRefereeGather(void)
{
    m3ShapeId meshShape;
    m3World* world = MakeMeshWorld(&meshShape);
    const m3MeshData* mesh = MeshOf(world, meshShape);
    const m3MeshBvh* bvh = BvhOf(world, meshShape);
    CHECK(mesh->triangleCount == 2 * 31 * 31, "heightfield triangulates fully");
    CHECK(bvh->nodeCount > 100, "the tree actually built");

    uint16_t gathered[M3_MAX_MESH_TRIANGLES];
    uint16_t brute[M3_MAX_MESH_TRIANGLES];
    for (int32_t round = 0; round < 300; ++round)
    {
        m3Vec3 center = {RandRange(-4.0f, 36.0f), RandRange(-2.0f, 2.0f), RandRange(-4.0f, 36.0f)};
        m3Vec3 half = {RandRange(0.05f, 3.0f), RandRange(0.05f, 3.0f), RandRange(0.05f, 3.0f)};
        m3Vec3 qlo = m3Sub3(center, half);
        m3Vec3 qhi = m3Add3(center, half);

        int32_t gatherCount = Gather(bvh, qlo, qhi, gathered);

        // The referee: full scan with the exact per-triangle test.
        int32_t bruteCount = 0;
        for (int32_t t = 0; t < mesh->triangleCount; ++t)
        {
            m3Vec3 tlo;
            m3Vec3 thi;
            TriBox(mesh, t, &tlo, &thi);
            if (BoxesOverlap(tlo, thi, qlo, qhi))
            {
                brute[bruteCount++] = (uint16_t)t;
            }
        }

        // Gather is ascending and duplicate-free.
        bool ascending = true;
        for (int32_t i = 1; i < gatherCount; ++i)
        {
            if (gathered[i] <= gathered[i - 1])
            {
                ascending = false;
            }
        }
        CHECK(ascending, "gather is strictly ascending");

        // Filtered by the exact test, gather equals brute exactly.
        int32_t filtered = 0;
        bool equal = true;
        for (int32_t i = 0; i < gatherCount; ++i)
        {
            m3Vec3 tlo;
            m3Vec3 thi;
            TriBox(mesh, gathered[i], &tlo, &thi);
            if (BoxesOverlap(tlo, thi, qlo, qhi))
            {
                if (filtered >= bruteCount || brute[filtered] != gathered[i])
                {
                    equal = false;
                    break;
                }
                filtered += 1;
            }
        }
        if (filtered != bruteCount)
        {
            equal = false;
        }
        CHECK(equal, "exact-filtered gather equals the brute-force scan");
        if (!equal)
        {
            break; // one detailed failure is enough
        }
    }

    // Degenerates: a box that misses everything, a box that covers it.
    int32_t none =
        Gather(bvh, (m3Vec3){100.0f, 100.0f, 100.0f}, (m3Vec3){101.0f, 101.0f, 101.0f}, gathered);
    CHECK(none == 0, "a far box gathers nothing");
    int32_t all = Gather(bvh, (m3Vec3){-100.0f, -100.0f, -100.0f}, (m3Vec3){100.0f, 100.0f, 100.0f},
                         gathered);
    CHECK(all == mesh->triangleCount, "a covering box gathers every triangle");

    // The hit words at the edges: every item below a count reads out once,
    // in order, across word boundaries and up to the largest item id.
    static const int32_t counts[] = {0, 1, 63, 64, 65, 129, M3_MAX_MESH_TRIANGLES};
    bool whole = true;
    for (int32_t c = 0; c < (int32_t)(sizeof(counts) / sizeof(counts[0])); ++c)
    {
        static m3MeshBvhHits hits;
        m3MeshBvhHitAll(&hits, counts[c]);
        int32_t read = ReadHits(&hits, gathered);
        for (int32_t i = 0; i < read; ++i)
        {
            whole = whole && gathered[i] == (uint16_t)i;
        }
        whole = whole && read == counts[c];
    }
    CHECK(whole, "every item below a count reads out once, in order");
    m3DestroyWorld(world);
}

// A random path over the 32 m mesh or grid: up to 40 m long, with a
// reach from 5 cm to 2 m, counted up to a random fraction or all of it.
static m3SurfacePath RandomPath(m3real span)
{
    m3Vec3 start = {RandRange(-4.0f, span + 4.0f), RandRange(-2.0f, 2.0f),
                    RandRange(-4.0f, span + 4.0f)};
    m3Vec3 step = {RandRange(-40.0f, 40.0f), RandRange(-3.0f, 3.0f), RandRange(-40.0f, 40.0f)};
    m3real until = (NextRand() & 1u) != 0 ? 1.0f : RandRange(0.0f, 1.0f);
    return (m3SurfacePath){start, m3Add3(start, step), RandRange(0.05f, 2.0f), until};
}

// The referee for a path: whether it comes within reach of the box lo..hi
// on every axis at once, up to until. Slabs in double, written out here
// apart from the library's.
static bool PathNearBox(const m3SurfacePath* path, m3Vec3 lo, m3Vec3 hi)
{
    const double s[3] = {(double)path->start.x, (double)path->start.y, (double)path->start.z};
    const double e[3] = {(double)path->end.x, (double)path->end.y, (double)path->end.z};
    const double l[3] = {(double)lo.x, (double)lo.y, (double)lo.z};
    const double h[3] = {(double)hi.x, (double)hi.y, (double)hi.z};
    double r = (double)path->reach;
    double t0 = 0.0;
    double t1 = (double)path->until;
    for (int32_t k = 0; k < 3; ++k)
    {
        double d = e[k] - s[k];
        double a = l[k] - r;
        double b = h[k] + r;
        if (d == 0.0)
        {
            if (s[k] < a || s[k] > b)
            {
                return false;
            }
            continue;
        }
        double ta = (a - s[k]) / d;
        double tb = (b - s[k]) / d;
        t0 = fmax(t0, fmin(ta, tb));
        t1 = fmin(t1, fmax(ta, tb));
    }
    return t0 <= t1;
}

static void TestRefereePathGather(void)
{
    m3ShapeId meshShape;
    m3World* world = MakeMeshWorld(&meshShape);
    const m3MeshData* mesh = MeshOf(world, meshShape);
    const m3MeshBvh* bvh = BvhOf(world, meshShape);
    uint16_t gathered[M3_MAX_MESH_TRIANGLES];
    bool ascending = true;
    bool complete = true;
    for (int32_t round = 0; round < 300 && complete; ++round)
    {
        m3SurfacePath path = RandomPath(32.0f);
        int32_t count = GatherPath(bvh, &path, gathered);
        for (int32_t i = 1; i < count; ++i)
        {
            ascending = ascending && gathered[i] > gathered[i - 1];
        }
        int32_t at = 0;
        for (int32_t t = 0; t < mesh->triangleCount; ++t)
        {
            m3Vec3 tlo;
            m3Vec3 thi;
            TriBox(mesh, t, &tlo, &thi);
            while (at < count && gathered[at] < t)
            {
                at += 1;
            }
            if (PathNearBox(&path, tlo, thi) && !(at < count && gathered[at] == t))
            {
                complete = false;
            }
        }
    }
    CHECK(ascending, "a path gather is strictly ascending");
    CHECK(complete, "a path gather holds every triangle the path comes within reach of");

    // A diagonal across the mesh, half a meter thick, against its box.
    m3SurfacePath diagonal = {{0.0f, 0.5f, 0.0f}, {31.0f, 0.5f, 31.0f}, 0.5f, 1.0f};
    int32_t band = GatherPath(bvh, &diagonal, gathered);
    int32_t box = Gather(bvh, (m3Vec3){-0.5f, 0.0f, -0.5f}, (m3Vec3){31.5f, 1.0f, 31.5f}, gathered);
    printf("a diagonal path gathers %d triangles, its box %d\n", band, box);
    CHECK(band > 0 && 4 * band < box, "a diagonal path gathers a band, not its box");
    m3DestroyWorld(world);
}

// What a path visit saw: each triangle as 2 * cell + its half, in order.
typedef struct Seen
{
    const m3HeightFieldData* hf;
    int32_t ids[2 * 64 * 64];
    int32_t count;
} Seen;

static bool SeeTriangle(const m3Vec3 tri[3], void* context)
{
    Seen* seen = (Seen*)context;
    int32_t cx = (int32_t)m3MinF(tri[0].x, m3MinF(tri[1].x, tri[2].x));
    int32_t cz = (int32_t)m3MinF(tri[0].z, m3MinF(tri[1].z, tri[2].z));
    m3Vec3 cell[2][3];
    m3HeightFieldCellTris(seen->hf, cx, cz, cell);
    int32_t half = memcmp(cell[0], tri, sizeof(cell[0])) == 0 ? 0 : 1;
    seen->ids[seen->count++] = 2 * (cz * (seen->hf->nx - 1) + cx) + half;
    return true;
}

static void TestRefereeHeightFieldPath(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float heights[33 * 33];
    for (int32_t i = 0; i < 33 * 33; ++i)
    {
        heights[i] = 0.25f * (float)((i * 7 + (i / 33) * 13) % 5) - 0.5f;
    }
    m3ShapeId grid = NewHeightFieldGridShape(world, ground, &sd, heights, 33, 33, 1.0f);
    const m3HeightFieldData* hf =
        &world->heightFields.hfData[world->shapes.shapeHfIndex[grid.index1 - 1]];
    static Seen seen;
    seen.hf = hf;
    bool ascending = true;
    bool complete = true;
    for (int32_t round = 0; round < 300 && complete; ++round)
    {
        m3SurfacePath path = RandomPath(32.0f);
        seen.count = 0;
        m3HeightFieldVisitPath(hf, &path, SeeTriangle, &seen);
        for (int32_t i = 1; i < seen.count; ++i)
        {
            ascending = ascending && seen.ids[i] > seen.ids[i - 1];
        }
        int32_t at = 0;
        for (int32_t id = 0; id < 2 * 32 * 32; ++id)
        {
            m3Vec3 cell[2][3];
            m3HeightFieldCellTris(hf, (id / 2) % 32, (id / 2) / 32, cell);
            m3Vec3 tlo = {m3MinF(cell[id & 1][0].x, m3MinF(cell[id & 1][1].x, cell[id & 1][2].x)),
                          m3MinF(cell[id & 1][0].y, m3MinF(cell[id & 1][1].y, cell[id & 1][2].y)),
                          m3MinF(cell[id & 1][0].z, m3MinF(cell[id & 1][1].z, cell[id & 1][2].z))};
            m3Vec3 thi = {m3MaxF(cell[id & 1][0].x, m3MaxF(cell[id & 1][1].x, cell[id & 1][2].x)),
                          m3MaxF(cell[id & 1][0].y, m3MaxF(cell[id & 1][1].y, cell[id & 1][2].y)),
                          m3MaxF(cell[id & 1][0].z, m3MaxF(cell[id & 1][1].z, cell[id & 1][2].z))};
            while (at < seen.count && seen.ids[at] < id)
            {
                at += 1;
            }
            if (PathNearBox(&path, tlo, thi) && !(at < seen.count && seen.ids[at] == id))
            {
                complete = false;
            }
        }
    }
    CHECK(ascending, "a grid path visit goes in ascending cell order");
    CHECK(complete, "a grid path visit sees every triangle the path comes within reach of");

    m3SurfacePath diagonal = {{0.0f, 0.5f, 0.0f}, {32.0f, 0.5f, 32.0f}, 0.5f, 1.0f};
    seen.count = 0;
    m3HeightFieldVisitPath(hf, &diagonal, SeeTriangle, &seen);
    int32_t band = seen.count;
    seen.count = 0;
    m3HeightFieldVisit(hf, (m3Vec3){-0.5f, 0.0f, -0.5f}, (m3Vec3){32.5f, 1.0f, 32.5f}, SeeTriangle,
                       &seen);
    printf("a diagonal path visits %d grid triangles, its box %d\n", band, seen.count);
    CHECK(band > 0 && 4 * band < seen.count, "a diagonal path visits a band, not its box");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRefereeGather();
    TestRefereePathGather();
    TestRefereeHeightFieldPath();
    if (s_failures == 0)
    {
        printf("test_bvh: all green\n");
        return 0;
    }
    printf("test_bvh: %d failure(s)\n", s_failures);
    return 1;
}
