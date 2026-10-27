// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the snapshot bound suites measure with: the largest mesh and hull
// content a slot takes, counted from the format by hand, and the slots a
// bound allows, measured from empty worlds.

#ifndef MAUL3D_TEST_SNAPSHOT_BOUND_H
#define MAUL3D_TEST_SNAPSHOT_BOUND_H

#include "helpers.h"
#include "test_harness.h"
#include "world_internal.h"

#include <stdint.h>

// The most bytes a mesh slot's and a hull slot's content take in the
// tail: the vertices, the indices, an edge flag byte, the material count,
// table and group byte per triangle; the unit mass, center, inertia and
// hull center, then the vertices and the per-face, index and half-edge
// arrays.
static inline int64_t MeshContentMost(void)
{
    int64_t vc = M3_MAX_MESH_VERTICES;
    int64_t tc = M3_MAX_MESH_TRIANGLES;
    return vc * (int64_t)sizeof(m3Vec3) + 6 * tc + tc + 4 +
           M3_MAX_MESH_MATERIALS * (int64_t)sizeof(m3MeshSurfaceMaterial) + tc;
}

static inline int64_t HullContentMost(void)
{
    return 4 + 2 * (int64_t)sizeof(m3Vec3) + (int64_t)sizeof(m3Mat3) +
           M3_HULL_MAX_VERTS * (int64_t)sizeof(m3Vec3) +
           M3_HULL_MAX_FACES * ((int64_t)sizeof(m3Vec3) + 4 + 1 + 2) + M3_HULL_MAX_FACE_INDICES +
           M3_HULL_MAX_HALF_EDGES * (int64_t)sizeof(m3HullHalfEdge);
}

// One slot of every kind.
static inline m3WorldDef SmallDef(void)
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

// The snapshot bytes of an empty world of def with one capacity changed.
static inline int64_t EmptySnapshot(int32_t* capacity, const m3WorldDef* def, int32_t value)
{
    *capacity = value;
    m3World* world = NewWorld(def);
    int64_t size = m3World_SnapshotSize(world);
    m3DestroyWorld(world);
    return size;
}

// An allocator that counts every call and refuses each one.
static int32_t s_asked = 0;

static inline void* RefuseAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    (void)context;
    s_asked += 1;
    return NULL;
}

static inline void RefuseFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)memory;
    (void)bytes;
    (void)alignment;
    (void)context;
}

// One capacity's slots grow an empty world's snapshot by the same bytes
// each; with its content at the largest a slot takes perSlot + content.
// Returns the largest count of those slots bound allows, the other slots
// of def at their largest beside them. It measures worlds of one to three
// slots, which any bound the suites use admits.
static inline int32_t MostSlots(int32_t* capacity, const m3WorldDef* def, int64_t content,
                                int64_t otherContent, int64_t bound)
{
    int64_t one = EmptySnapshot(capacity, def, 1);
    int64_t perSlot = EmptySnapshot(capacity, def, 2) - one;
    int64_t fixed = one - perSlot;
    CHECK(perSlot > 0 && EmptySnapshot(capacity, def, 3) == fixed + 3 * perSlot,
          "an empty world's snapshot grows by the same bytes per slot");
    return (int32_t)((bound - fixed - otherContent) / (perSlot + content));
}

#endif // MAUL3D_TEST_SNAPSHOT_BOUND_H
