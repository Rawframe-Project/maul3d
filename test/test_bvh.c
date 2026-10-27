// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mesh BVH gate: the brute-force referee. The tree is a pruning
// layer, nothing more: for any query box the gathered set, filtered by
// the exact per-triangle test, must equal the full scan filtered by the
// same test. Plus the degenerate boxes.

#include "helpers.h"
#include "test_harness.h"
#include "world.h"
#include "world_internal.h"

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

static void TestRefereeGather(void)
{
    m3ShapeId meshShape;
    m3World* world = MakeMeshWorld(&meshShape);
    const m3MeshData* mesh = MeshOf(world, meshShape);
    const m3MeshBvh* bvh = BvhOf(world, meshShape);
    CHECK(mesh->triangleCount == 2 * 31 * 31, "heightfield triangulates fully");
    CHECK(bvh->nodeCount > 100, "the tree actually built");

    uint16_t gathered[M3_MESH_MAX_TRIS];
    uint16_t brute[M3_MESH_MAX_TRIS];
    for (int32_t round = 0; round < 300; ++round)
    {
        m3Vec3 center = {RandRange(-4.0f, 36.0f), RandRange(-2.0f, 2.0f), RandRange(-4.0f, 36.0f)};
        m3Vec3 half = {RandRange(0.05f, 3.0f), RandRange(0.05f, 3.0f), RandRange(0.05f, 3.0f)};
        m3Vec3 qlo = m3Sub3(center, half);
        m3Vec3 qhi = m3Add3(center, half);

        int32_t gatherCount = m3MeshBvhGather(bvh, qlo, qhi, gathered, M3_COUNT_OF(gathered));

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
        m3MeshBvhGather(bvh, (m3Vec3){100.0f, 100.0f, 100.0f}, (m3Vec3){101.0f, 101.0f, 101.0f},
                        gathered, M3_COUNT_OF(gathered));
    CHECK(none == 0, "a far box gathers nothing");
    int32_t all =
        m3MeshBvhGather(bvh, (m3Vec3){-100.0f, -100.0f, -100.0f}, (m3Vec3){100.0f, 100.0f, 100.0f},
                        gathered, M3_COUNT_OF(gathered));
    CHECK(all == mesh->triangleCount, "a covering box gathers every triangle");

    // A gather fills at most the capacity it is given and leaves the rest
    // of the array alone.
    enum
    {
        CAPACITY = 8
    };
    for (int32_t i = 0; i < 2 * CAPACITY; ++i)
    {
        gathered[i] = 0xBEEF;
    }
    int32_t few = m3MeshBvhGather(bvh, (m3Vec3){-100.0f, -100.0f, -100.0f},
                                  (m3Vec3){100.0f, 100.0f, 100.0f}, gathered, CAPACITY);
    bool kept = true;
    for (int32_t i = CAPACITY; i < 2 * CAPACITY; ++i)
    {
        kept = kept && gathered[i] == 0xBEEF;
    }
    CHECK(few == CAPACITY && kept, "a gather fills at most its capacity");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRefereeGather();
    if (s_failures == 0)
    {
        printf("test_bvh: all green\n");
        return 0;
    }
    printf("test_bvh: %d failure(s)\n", s_failures);
    return 1;
}
