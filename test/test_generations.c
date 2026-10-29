// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Generation retirement, pool by pool. A slot names objects under
// generations 0 to M3_LAST_GENERATION, then retires: every slot here
// starts one generation short of the last, so the first round of creates
// takes the last generation and the second retires each slot. No
// destroyed id ever names anything again, the spent pool then refuses
// with m3_errorCapacity, and a restore keeps the spent budget. The
// internal hull pool spends its slots the same way through hull shapes.

#include "helpers.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    CAPACITY = 2,
    HEADER_BYTES = 48,
};

// One pool driven through its public create and destroy: a made object
// is reported as its slot and generation.
typedef struct Pool
{
    const char* name;
    m3IdPool* (*pool)(m3World* world);
    bool (*create)(m3World* world, uint32_t* index1, uint32_t* generation);
    void (*destroy)(m3World* world, uint32_t index1, uint32_t generation);
    bool (*valid)(const m3World* world, uint32_t index1, uint32_t generation);
} Pool;

static m3WorldDef SpentDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = CAPACITY;
    def.shapeCapacity = CAPACITY;
    return def;
}

// ---- Bodies ----

static m3IdPool* BodyPool(m3World* world)
{
    return &world->bodies.bodyPool;
}

static bool CreateBody(m3World* world, uint32_t* index1, uint32_t* generation)
{
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId id = m3_nullBodyId;
    if (m3CreateBody(world, &bd, &id) != m3_success)
    {
        return false;
    }
    *index1 = id.index1;
    *generation = id.generation;
    return true;
}

static void DestroyBodyAt(m3World* world, uint32_t index1, uint32_t generation)
{
    DestroyBody(world, (m3BodyId){index1, generation});
}

static bool BodyValid(const m3World* world, uint32_t index1, uint32_t generation)
{
    return m3Body_IsValid(world, (m3BodyId){index1, generation});
}

// ---- Shapes, and the hull pool behind hull shapes ----

static m3BodyId s_ground;

static m3IdPool* ShapePool(m3World* world)
{
    return &world->shapes.shapePool;
}

static m3IdPool* HullPool(m3World* world)
{
    return &world->hulls.hullPool;
}

static m3BodyId Ground(m3World* world)
{
    if (!m3Body_IsValid(world, s_ground))
    {
        m3BodyDef bd = m3DefaultBodyDef();
        s_ground = NewBody(world, &bd);
    }
    return s_ground;
}

static bool CreateSphere(m3World* world, uint32_t* index1, uint32_t* generation)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId id = m3_nullShapeId;
    if (m3CreateSphereShape(world, Ground(world), &sd, &sphere, &id) != m3_success)
    {
        return false;
    }
    *index1 = id.index1;
    *generation = id.generation;
    return true;
}

// A hull shape takes a shape slot and a hull slot; the shape pool here
// never runs short, so the hull pool is the one that retires.
static bool CreateHull(m3World* world, uint32_t* index1, uint32_t* generation)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    const m3Vec3 cloud[4] = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    int32_t hulls = world->hulls.hullPool.maxIndex - world->hulls.hullPool.freeCount -
                    world->hulls.hullPool.retiredCount;
    m3ShapeId id = m3_nullShapeId;
    if (m3CreateHullShape(world, Ground(world), &sd, cloud, 4, &id) != m3_success)
    {
        return false;
    }
    // The hull slot the shape took: the only live one.
    int32_t slot = -1;
    for (int32_t i = 0; i < world->hulls.hullPool.maxIndex; ++i)
    {
        slot = world->hulls.hullPool.alive[i] != 0 ? i : slot;
    }
    CHECK(hulls == 0 && slot >= 0, "a hull shape takes one hull slot");
    *index1 = (uint32_t)slot + 1u;
    *generation = world->hulls.hullPool.generations[slot];
    return true;
}

static void DestroyShapeAt(m3World* world, uint32_t index1, uint32_t generation)
{
    DestroyShape(world, (m3ShapeId){index1, generation});
}

static void DestroyHullAt(m3World* world, uint32_t index1, uint32_t generation)
{
    (void)index1;
    (void)generation;
    for (int32_t s = 0; s < world->shapes.shapePool.maxIndex; ++s)
    {
        if (world->shapes.shapePool.alive[s] != 0 && world->shapes.shapeHullIndex[s] >= 0)
        {
            DestroyShape(world,
                         (m3ShapeId){(uint32_t)s + 1u, world->shapes.shapePool.generations[s]});
        }
    }
}

static bool ShapeValid(const m3World* world, uint32_t index1, uint32_t generation)
{
    return m3Shape_IsValid(world, (m3ShapeId){index1, generation});
}

static bool HullValid(const m3World* world, uint32_t index1, uint32_t generation)
{
    return m3IdPoolValid(&world->hulls.hullPool, (int32_t)index1 - 1, generation) != 0;
}

// ---- Water volumes ----

static m3IdPool* WaterPool(m3World* world)
{
    return &world->water.waterPool;
}

static bool CreateWater(m3World* world, uint32_t* index1, uint32_t* generation)
{
    m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
    m3WaterVolumeId id = m3_nullWaterVolumeId;
    if (m3CreateWaterVolume(world, &def, &id) != m3_success)
    {
        return false;
    }
    *index1 = id.index1;
    *generation = id.generation;
    return true;
}

static void DestroyWaterAt(m3World* world, uint32_t index1, uint32_t generation)
{
    (void)m3DestroyWaterVolume(world, (m3WaterVolumeId){index1, generation});
}

static bool WaterValid(const m3World* world, uint32_t index1, uint32_t generation)
{
    return m3WaterVolume_IsValid(world, (m3WaterVolumeId){index1, generation});
}

// ---- The driver ----

// Spends every slot of the pool: returns the snapshot of the spent
// world, its size in *bytes.
static uint8_t* Spend(const Pool* p, m3World* world, int32_t* bytes)
{
    char what[160];
    m3IdPool* pool = p->pool(world);
    int32_t capacity = pool->capacity;
    for (int32_t i = 0; i < capacity; ++i)
    {
        pool->generations[i] = M3_LAST_GENERATION - 1u;
    }
    uint32_t index1[2 * M3_MAX_WATER_VOLUMES];
    uint32_t generation[2 * M3_MAX_WATER_VOLUMES];
    int32_t made = 0;
    for (int32_t k = 0; k < 2 * capacity; ++k)
    {
        if (!p->create(world, &index1[k], &generation[k]))
        {
            break;
        }
        p->destroy(world, index1[k], generation[k]);
        made += 1;
    }
    int32_t lastRound = 0;
    int32_t revived = 0;
    for (int32_t k = 0; k < made; ++k)
    {
        lastRound += generation[k] == M3_LAST_GENERATION ? 1 : 0;
        revived += p->valid(world, index1[k], generation[k]) ? 1 : 0;
    }
    snprintf(what, sizeof(what), "%s: each slot names its last generation once", p->name);
    CHECK(made == 2 * capacity && lastRound == capacity, what);
    snprintf(what, sizeof(what), "%s: no destroyed id names anything", p->name);
    CHECK(revived == 0, what);
    snprintf(what, sizeof(what), "%s: every slot retired", p->name);
    CHECK(pool->retiredCount == capacity && pool->freeCount == 0, what);
    uint32_t spareIndex = 0;
    uint32_t spareGeneration = 0;
    snprintf(what, sizeof(what), "%s: the spent pool refuses", p->name);
    CHECK(!p->create(world, &spareIndex, &spareGeneration), what);
    *bytes = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)*bytes);
    CHECK(snap != NULL && Snapshot(world, snap, *bytes) == *bytes, "the spent world snapshots");
    return snap;
}

static void CheckPool(const Pool* p, int32_t capacity)
{
    char what[160];
    m3WorldDef def = SpentDef();
    m3World* world = NewWorld(&def);
    s_ground = m3_nullBodyId;
    CHECK(p->pool(world)->capacity == capacity, "the pool has the capacity the test sets");
    int32_t bytes = 0;
    uint8_t* snap = Spend(p, world, &bytes);
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);

    // A fresh world takes the spent one's budget with its state.
    world = NewWorld(&def);
    snprintf(what, sizeof(what), "%s: the spent world restores to its hash", p->name);
    CHECK(Restore(world, snap, bytes) && m3World_Hash(world) == hash, what);
    uint32_t index1 = 0;
    uint32_t generation = 0;
    snprintf(what, sizeof(what), "%s: a restore keeps the spent budget", p->name);
    CHECK(!p->create(world, &index1, &generation), what);
    m3DestroyWorld(world);

    // A retired slot is dead, not queued and at the last generation; a
    // snapshot that gives the pool's last slot an earlier generation hides
    // it and is refused, the world unchanged.
    world = NewWorld(&def);
    uint64_t freshHash = m3World_Hash(world);
    m3IdPool* pool = p->pool(world);
    size_t field = (size_t)((uint8_t*)&pool->generations - (uint8_t*)world);
    int64_t at = m3StateFieldAt(world, field);
    CHECK(at >= 0, "the generations are snapshot state");
    uint32_t earlier = M3_LAST_GENERATION - 1u;
    memcpy(snap + HEADER_BYTES + (at < 0 ? 0 : at) + sizeof(uint32_t) * (size_t)(capacity - 1),
           &earlier, sizeof(earlier));
    snprintf(what, sizeof(what), "%s: a hidden retired slot is refused", p->name);
    CHECK(!Restore(world, snap, bytes) && m3World_Hash(world) == freshHash, what);
    m3DestroyWorld(world);
    free(snap);
}

static void TestSpentSlotsRetire(void)
{
    const Pool bodies = {"bodies", BodyPool, CreateBody, DestroyBodyAt, BodyValid};
    const Pool shapes = {"shapes", ShapePool, CreateSphere, DestroyShapeAt, ShapeValid};
    const Pool hulls = {"hulls", HullPool, CreateHull, DestroyHullAt, HullValid};
    const Pool water = {"water", WaterPool, CreateWater, DestroyWaterAt, WaterValid};
    CheckPool(&bodies, CAPACITY);
    CheckPool(&shapes, CAPACITY);
    CheckPool(&hulls, CAPACITY);
    CheckPool(&water, M3_MAX_WATER_VOLUMES);
}

int main(void)
{
    TestSpentSlotsRetire();
    if (s_failures == 0)
    {
        printf("test_generations: all green\n");
        return 0;
    }
    printf("test_generations: %d failure(s)\n", s_failures);
    return 1;
}
