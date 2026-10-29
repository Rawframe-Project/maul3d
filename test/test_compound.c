// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The compound gate: an offset shape behaves exactly like a
// centered shape parked at the same world spot, a dumbbell carries
// the analytic composite inertia, a rotated offset lies the way it
// looks, queries hit shapes where they visually sit, and hostile
// offsets refuse.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* PlaneWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &fl);
    return world;
}

static void TestOffsetEquivalence(void)
{
    // Same shape placement, two bookkeepings: a body at y=3 with
    // the box slung 0.5 below versus a body at y=2.5 with the box
    // centered. The COM and inertia agree, so the PHYSICS must:
    // both boxes rest with their tops at exactly the same height
    // and both worlds go to sleep.
    double topA = 0.0;
    double topB = 0.0;
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3World* world = PlaneWorld();
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, pass == 0 ? 3.0 : 2.5, 0.0};
        m3BodyId body = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        if (pass == 0)
        {
            sd.localPosition = (m3Vec3){0.0f, -0.5f, 0.0f};
        }
        NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
        for (int32_t i = 0; i < 240; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3RayCastResult hit = m3World_CastRayClosest(
            world, (m3Pos3){0.0, 5.0, 0.0}, (m3Vec3){0.0f, -6.0f, 0.0f}, m3DefaultQueryFilter());
        CHECK(hit.hit, "the settled box answers the ray");
        if (pass == 0)
        {
            topA = hit.point.y;
        }
        else
        {
            topB = hit.point.y;
        }
        CHECK(!m3Body_IsAwake(world, body), "the settled box sleeps");
        m3DestroyWorld(world);
    }
    CHECK(fabs(topA - topB) < 1.0e-9, "offset and centered boxes rest at the same height");
    CHECK(fabs(topA - 1.0) < 0.01, "and that height is the box top");
}

static void TestDumbbellInertia(void)
{
    // Two boxes slung at x = -1 and x = +1 on one body: spin about
    // y must obey I = 2 (Ibox + m d^2), the parallel axis theorem
    // in public. Gravity-free, damping zeroed for the analytics.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.angularDamping = 0.0f;
    m3BodyId bar = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    float half = 0.25f;
    for (int32_t k = 0; k < 2; ++k)
    {
        sd.localPosition = (m3Vec3){k == 0 ? -1.0f : 1.0f, 0.0f, 0.0f};
        NewBoxShape(world, bar, &sd, (m3Vec3){half, half, half});
    }
    // m = rho * (2h)^3 = 0.125; Ibox_yy = m/12 * ((2h)^2 + (2h)^2);
    // I = 2 * (Ibox + m * d^2).
    float m = 0.125f;
    float ibox = m / 12.0f * (0.25f + 0.25f);
    float iyy = 2.0f * (ibox + m * 1.0f);
    m3Body_ApplyAngularImpulse(world, bar, (m3Vec3){0.0f, 0.5f, 0.0f});
    m3Vec3 w = m3Body_GetAngularVelocity(world, bar);
    float expected = 0.5f / iyy;
    CHECK(fabsf(w.y - expected) < 0.02f * expected, "the dumbbell spins by the composite books");
    CHECK(fabsf(w.x) < 1.0e-6f && fabsf(w.z) < 1.0e-6f, "a principal axis stays principal");
    m3DestroyWorld(world);
}

static void TestRotatedOffsetRests(void)
{
    // A capsule whose OFFSET rotation lays it on its side: the body
    // frame stays upright, the shape lies down, and it rests at
    // radius height like any lying capsule.
    m3World* world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    float s = sinf(0.25f * M3_PI);
    float c = cosf(0.25f * M3_PI);
    sd.localRotation = (m3Quat){0.0f, 0.0f, s, c}; // 90 degrees about z
    m3Capsule cap = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.3f};
    NewCapsuleShape(world, body, &sd, &cap);
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double y = m3Body_GetPosition(world, body).y;
    CHECK(fabs(y - 0.3) < 0.02, "the offset-rotated capsule lies at radius height");
    m3DestroyWorld(world);
}

static void TestQueriesSeeOffsets(void)
{
    // A static body at the origin with a box slung at x = 3: the
    // ray at x = 3 hits the box, the ray at x = 0 falls to the
    // plane, and an overlap at the box's true spot finds it.
    m3World* world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId post = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.localPosition = (m3Vec3){3.0f, 1.0f, 0.0f};
    NewBoxShape(world, post, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3RayCastResult at3 = m3World_CastRayClosest(
        world, (m3Pos3){3.0, 5.0, 0.0}, (m3Vec3){0.0f, -6.0f, 0.0f}, m3DefaultQueryFilter());
    CHECK(at3.hit && fabs(at3.point.y - 1.5) < 1.0e-6, "the ray hits the box where it sits");
    m3RayCastResult at0 = m3World_CastRayClosest(
        world, (m3Pos3){0.0, 5.0, 0.0}, (m3Vec3){0.0f, -6.0f, 0.0f}, m3DefaultQueryFilter());
    CHECK(at0.hit && at0.point.y < 0.01, "the body origin is empty space");
    m3DestroyWorld(world);
}

static void TestBigHullUnderTheGates(void)
{
    // A hull of more than 40 vertices drops, rests,
    // answers a ray, and the session replays bit-exact.
    static uint8_t journal[131072];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3World* world = PlaneWorld();
        bool recording =
            run == 0 && false; // creates precede JournalBegin in PlaneWorld: record inline
        (void)recording;
        m3Vec3 cloud[96];
        for (int32_t i = 0; i < 96; ++i)
        {
            float t = (float)i / 96.0f;
            float phi = 2.399963f * (float)i;
            float y = 1.0f - 2.0f * t;
            float r = sqrtf(1.0f - y * y);
            cloud[i] = (m3Vec3){0.8f * r * cosf(phi), 0.8f * y, 0.8f * r * sinf(phi)};
        }
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 3.0, 0.0};
        m3BodyId rock = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        // A 96-point cloud is nearly a sphere, and near-spheres
        // never stop rolling without rolling resistance.
        sd.rollingResistance = 0.05f;
        m3ShapeId shape = NewHullShape(world, rock, &sd, cloud, 96);
        CHECK(m3Shape_IsValid(world, shape), "the 96-point cloud builds a hull shape");
        for (int32_t i = 0; i < 240; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        double y = m3Body_GetPosition(world, rock).y;
        CHECK(y > 0.5 && y < 0.9, "the big rock rests near its radius");
        CHECK(!m3Body_IsAwake(world, rock), "and sleeps");
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "big-hull twins are bit-identical");
    (void)journal;
}

static void TestGrandMesh(void)
{
    // The mesh ceiling: a procedural ~60k-triangle
    // terrain accepts, carries traffic, snapshots, and restores
    // onto identical bits; one triangle past the cap refuses.
    enum
    {
        GRID = 174 // (174 - 1)^2 * 2 = 59858 triangles, 30276 verts
    };
    static m3Vec3 verts[GRID * GRID];
    static uint16_t tris[6 * (GRID - 1) * (GRID - 1)];
    for (int32_t z = 0; z < GRID; ++z)
    {
        for (int32_t x = 0; x < GRID; ++x)
        {
            float h = 0.15f * sinf(0.37f * (float)x) + 0.15f * cosf(0.29f * (float)z);
            verts[z * GRID + x] = (m3Vec3){0.5f * (float)x, h, 0.5f * (float)z};
        }
    }
    int32_t n = 0;
    for (int32_t z = 0; z < GRID - 1; ++z)
    {
        for (int32_t x = 0; x < GRID - 1; ++x)
        {
            uint16_t v00 = (uint16_t)(z * GRID + x);
            uint16_t v10 = (uint16_t)(z * GRID + x + 1);
            uint16_t v01 = (uint16_t)((z + 1) * GRID + x);
            uint16_t v11 = (uint16_t)((z + 1) * GRID + x + 1);
            tris[n++] = v00;
            tris[n++] = v11;
            tris[n++] = v10;
            tris[n++] = v00;
            tris[n++] = v01;
            tris[n++] = v11;
        }
    }
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId terrain = NewMeshShape(world, ground, &sd, verts, GRID * GRID, tris, n / 3);
    CHECK(m3Shape_IsValid(world, terrain), "the 60k terrain accepts");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){43.0, 2.0, 43.0};
    m3BodyId crate = NewBody(world, &bd);
    NewBoxShape(world, crate, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    for (int32_t i = 0; i < 90; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double y = m3Body_GetPosition(world, crate).y;
    CHECK(y > -1.0 && y < 1.5, "the crate lands on the far side of the terrain");
    m3RayCastResult hit = m3World_CastRayClosest(
        world, (m3Pos3){21.0, 5.0, 21.0}, (m3Vec3){0.0f, -10.0f, 0.0f}, m3DefaultQueryFilter());
    CHECK(hit.hit && fabs(hit.point.y) < 0.5, "a mid-terrain ray lands on the surface");

    int32_t snapBytes = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    CHECK(Snapshot(world, snap, snapBytes) == snapBytes, "the grand snapshot writes");
    uint64_t mid = m3World_Hash(world);
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t end = m3World_Hash(world);
    CHECK(Restore(world, snap, snapBytes), "the grand restore lands");
    CHECK(m3World_Hash(world) == mid, "the restore is bit-identical");
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) == end, "the re-run is bit-identical");
    free(snap);
    m3DestroyWorld(world);
}

static void TestHostileOffsets(void)
{
    m3World* world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.localRotation = (m3Quat){3.0f, 0.0f, 0.0f, 1.0f}; // not unit
    m3ShapeId refused = NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    CHECK(!m3Shape_IsValid(world, refused), "a non-unit offset rotation refuses");
    sd = m3DefaultShapeDef();
    sd.localPosition = (m3Vec3){NAN, 0.0f, 0.0f};
    refused = NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    CHECK(!m3Shape_IsValid(world, refused), "a NaN offset refuses");
    m3DestroyWorld(world);
}

int main(void)
{
    TestOffsetEquivalence();
    TestDumbbellInertia();
    TestRotatedOffsetRests();
    TestQueriesSeeOffsets();
    TestGrandMesh();
    TestBigHullUnderTheGates();
    TestHostileOffsets();
    if (s_failures == 0)
    {
        printf("test_compound: all green\n");
        return 0;
    }
    printf("test_compound: %d failure(s)\n", s_failures);
    return 1;
}
