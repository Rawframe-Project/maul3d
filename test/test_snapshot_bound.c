// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A world's snapshot never passes M3_MAX_SNAPSHOT_BYTES. Create counts the
// header, the fixed prefix and every mesh and hull slot's tail at its
// largest, so a def one slot past the bound is refused before anything is
// allocated and a def at the bound is created. A height field grid's
// samples are counted as they arrive: a grid that would carry the
// snapshot past the bound is refused with the world as it was.

#include "helpers.h"
#include "test_harness.h"
#include "world_internal.h"

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The most bytes a mesh slot's and a hull slot's content take in the
// tail, counted from the format by hand: the vertices, the indices, an
// edge flag byte, the material count, table and group byte per triangle;
// the unit mass, center, inertia and hull center, then the vertices and
// the per-face, index and half-edge arrays.
static int64_t MeshContentMost(void)
{
    int64_t vc = M3_MESH_MAX_VERTS;
    int64_t tc = M3_MESH_MAX_TRIS;
    return vc * (int64_t)sizeof(m3Vec3) + 6 * tc + tc + 4 +
           M3_MESH_MAX_MATERIALS * (int64_t)sizeof(m3MeshSurfaceMaterial) + tc;
}

static int64_t HullContentMost(void)
{
    return 4 + 2 * (int64_t)sizeof(m3Vec3) + (int64_t)sizeof(m3Mat3) +
           M3_HULL_MAX_VERTS * (int64_t)sizeof(m3Vec3) +
           M3_HULL_MAX_FACES * ((int64_t)sizeof(m3Vec3) + 4 + 1 + 2) + M3_HULL_MAX_FACE_INDICES +
           M3_HULL_MAX_HALF_EDGES * (int64_t)sizeof(m3HullHalfEdge);
}

static m3WorldDef SmallDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 1;
    def.shapeCapacity = 1;
    def.meshCapacity = 1;
    def.jointCapacity = 1;
    def.voxelCapacity = 1;
    def.characterCapacity = 1;
    def.vehicleCapacity = 1;
    def.softBodyCapacity = 1;
    return def;
}

// The snapshot bytes of an empty world of the small def with one capacity
// changed.
static int64_t EmptySnapshot(int32_t* capacity, const m3WorldDef* def, int32_t value)
{
    *capacity = value;
    m3World* world = NewWorld(def);
    int64_t size = m3World_SnapshotSize(world);
    m3DestroyWorld(world);
    return size;
}

// An allocator that counts every call and refuses each one.
static int32_t s_asked = 0;

static void* RefuseAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    (void)context;
    s_asked += 1;
    return NULL;
}

static void RefuseFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)memory;
    (void)bytes;
    (void)alignment;
    (void)context;
}

// One capacity's slots grow an empty world's snapshot by the same bytes
// each; with its content at the largest a slot takes perSlot + content.
// Returns the largest count of those slots the bound allows, the other
// slots of the small def at their largest beside them.
static int32_t MostSlots(int32_t* capacity, const m3WorldDef* def, int64_t content,
                         int64_t otherContent)
{
    int64_t a = EmptySnapshot(capacity, def, 100);
    int64_t b = EmptySnapshot(capacity, def, 200);
    int64_t perSlot = (b - a) / 100;
    int64_t fixed = a - 100 * perSlot;
    CHECK(perSlot > 0 && EmptySnapshot(capacity, def, 300) == fixed + 300 * perSlot,
          "an empty world's snapshot grows by the same bytes per slot");
    return (int32_t)((M3_MAX_SNAPSHOT_BYTES - fixed - otherContent) / (perSlot + content));
}

static void TestCreateAtTheBound(void)
{
    // Mesh slots weigh little in memory but up to 1.3 MB each in the
    // snapshot, so the bound counted on them is a world small enough to
    // create.
    m3WorldDef def = SmallDef();
    int32_t most = MostSlots(&def.meshCapacity, &def, MeshContentMost(), HullContentMost());
    def.meshCapacity = most;
    m3World* world = NULL;
    CHECK(m3CreateWorld(&def, &world) == m3_success && world != NULL,
          "a world whose snapshot can reach M3_MAX_SNAPSHOT_BYTES is created");
    m3DestroyWorld(world);

    def.meshCapacity = most + 1;
    def.allocator = (m3Allocator){RefuseAlloc, RefuseFree, NULL};
    s_asked = 0;
    world = (m3World*)&def;
    CHECK(m3CreateWorld(&def, &world) == m3_errorCapacity && world == NULL,
          "one mesh slot more could pass the bound and is refused");
    CHECK(s_asked == 0, "before anything is allocated");
}

static void TestShapeSlotsAtTheBound(void)
{
    // A world of shape slots at the bound takes gigabytes, so an allocator
    // that refuses everything shows how far creation got.
    m3WorldDef def = SmallDef();
    int32_t most = MostSlots(&def.shapeCapacity, &def, HullContentMost(), MeshContentMost());
    def.allocator = (m3Allocator){RefuseAlloc, RefuseFree, NULL};
    def.shapeCapacity = most + 1;
    s_asked = 0;
    m3World* world = (m3World*)&def;
    CHECK(m3CreateWorld(&def, &world) == m3_errorCapacity && world == NULL && s_asked == 0,
          "one shape slot past the bound is refused before anything is allocated");
    def.shapeCapacity = most;
    CHECK(m3CreateWorld(&def, &world) == m3_errorCapacity && world == NULL && s_asked > 0,
          "at the bound the def passes the size check and only the allocator refuses it");
}

static void TestHeightFieldAtTheBound(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    enum
    {
        side = M3_HEIGHTFIELD_MAX_DIM
    };
    float* heights = (float*)calloc((size_t)side * side, sizeof(float));
    int64_t growth = 8 + (int64_t)side * side * (int64_t)sizeof(float); // extremes and samples

    // The measure reads capacities alone, so a pair capacity raised in
    // place stands the world's snapshot next to the bound without memory.
    int32_t pairs = world->contacts.pairCapacity;
    int64_t base = m3World_SnapshotSize(world);
    world->contacts.pairCapacity = pairs + 1;
    int64_t perPair = m3World_SnapshotSize(world) - base;
    int32_t fits = pairs + (int32_t)((M3_MAX_SNAPSHOT_BYTES - growth - base) / perPair);
    int64_t near = base + (int64_t)(fits - pairs) * perPair;

    world->contacts.pairCapacity = fits + 1;
    uint64_t hash = m3World_Hash(world);
    int32_t size = m3World_SnapshotSize(world);
    uint64_t misuse = m3World_GetCounters(world).misuse;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId shape = {1, 1};
    CHECK(m3CreateHeightFieldGridShape(world, ground, &sd, heights, side, side, 1.0f, &shape) ==
                  m3_errorCapacity &&
              shape.index1 == 0,
          "a grid that would carry the snapshot past the bound is refused");
    CHECK(m3World_Hash(world) == hash && m3World_SnapshotSize(world) == size &&
              m3World_GetCounters(world).misuse == misuse,
          "with the world as it was, and no misuse counted");

    world->contacts.pairCapacity = fits;
    CHECK(m3CreateHeightFieldGridShape(world, ground, &sd, heights, side, side, 1.0f, &shape) ==
                  m3_success &&
              m3World_SnapshotSize(world) == near + growth,
          "a grid that keeps the snapshot within the bound is taken");
    world->contacts.pairCapacity = pairs;
    free(heights);
    m3DestroyWorld(world);
}

int main(void)
{
    TestCreateAtTheBound();
    TestShapeSlotsAtTheBound();
    TestHeightFieldAtTheBound();
    if (s_failures == 0)
    {
        printf("test_snapshot_bound: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
