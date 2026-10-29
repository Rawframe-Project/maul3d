// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The painted floor gate: a mesh triangle can wear its own
// surface material. The struck triangle's friction replaces the
// shape's in the contact mix, its surface velocity runs a conveyor,
// and hostile paint refuses loudly.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const m3Vec3 kQuadVerts[4] = {
    {-3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, 3.0f}, {-3.0f, 0.0f, 3.0f}};
static const uint16_t kQuadTris[6] = {0, 2, 1, 0, 3, 2};

static m3ShapeId Quad(m3World* world)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    return NewMeshShape(world, ground, &sd, kQuadVerts, 4, kQuadTris, 2);
}

static double SlideDistance(bool slippery)
{
    // One box, one shove, two floors: the painted low-friction
    // floor lets it slide far, the unpainted one grips.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3ShapeId floor = Quad(world);
    if (slippery)
    {
        m3MeshSurfaceMaterial mats[1];
        memset(mats, 0, sizeof(mats));
        mats[0].friction = 0.02f;
        uint8_t tri[2] = {0, 0};
        m3Shape_SetMeshMaterials(world, floor, mats, 1, tri);
    }
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-2.0, 0.3, 0.0};
    bd.linearVelocity = (m3Vec3){3.0f, 0.0f, 0.0f};
    m3BodyId box = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, box, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double travelled = m3Body_GetPosition(world, box).x + 2.0;
    m3DestroyWorld(world);
    return travelled;
}

static void TestPaintedFriction(void)
{
    double grip = SlideDistance(false);
    double slide = SlideDistance(true);
    CHECK(slide > grip + 0.5, "the painted slick floor lets the box slide farther");
}

static void TestConveyorTriangle(void)
{
    // A painted surface velocity turns the floor into a walkway: a
    // resting box gets carried; the same box on unpainted mesh
    // stays put.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        m3World* world = NewWorld(&wd);
        m3ShapeId floor = Quad(world);
        if (pass == 1)
        {
            m3MeshSurfaceMaterial mats[2];
            memset(mats, 0, sizeof(mats));
            mats[0].friction = 0.8f;
            mats[1].friction = 0.8f;
            mats[1].surfaceVelocity = (m3Vec3){1.5f, 0.0f, 0.0f};
            uint8_t tri[2] = {1, 1};
            m3Shape_SetMeshMaterials(world, floor, mats, 2, tri);
        }
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.3, 0.0};
        m3BodyId box = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        NewBoxShape(world, box, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
        for (int32_t i = 0; i < 150; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        double x = m3Body_GetPosition(world, box).x;
        if (pass == 0)
        {
            CHECK(fabs(x) < 0.05, "an unpainted mesh floor keeps the box in place");
        }
        else
        {
            CHECK(x > 0.5, "the painted conveyor triangle carries the box");
        }
        m3DestroyWorld(world);
    }
}

static void TestPaintWalls(void)
{
    // Hostile paint refuses loudly and moves no bits.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3ShapeId floor = Quad(world);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3ShapeId orb = NewSphereShape(world, ball, &sd, &s);
    uint64_t before = m3World_Hash(world);
    m3MeshSurfaceMaterial mats[2];
    memset(mats, 0, sizeof(mats));
    mats[0].friction = 0.5f;
    mats[1].friction = 0.5f;
    uint8_t tri[2] = {0, 1};
    m3Shape_SetMeshMaterials(world, orb, mats, 2, tri);   // not a mesh
    m3Shape_SetMeshMaterials(world, floor, mats, 0, tri); // zero groups
    m3Shape_SetMeshMaterials(world, floor, mats, 9, tri); // too many
    mats[1].friction = NAN;
    m3Shape_SetMeshMaterials(world, floor, mats, 2, tri); // hostile value
    mats[1].friction = 0.5f;
    uint8_t bad[2] = {0, 2};
    m3Shape_SetMeshMaterials(world, floor, mats, 2, bad); // group out of range
    CHECK(m3World_Hash(world) == before, "every refused paint moved no bits");
    m3DestroyWorld(world);
}

// A host allocator the test can turn off.
static void* ToggleAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
    return *(const bool*)context ? NULL : malloc(bytes);
}

static void ToggleFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    (void)context;
    free(memory);
}

// Whether a world's snapshot equals the given bytes. The hash leaves
// mesh materials out; the snapshot carries them.
static bool SnapshotIs(const m3World* world, const uint8_t* bytes, int32_t size)
{
    uint8_t* now = (uint8_t*)malloc((size_t)size);
    bool same =
        now != NULL && Snapshot(world, now, size) == size && memcmp(now, bytes, (size_t)size) == 0;
    free(now);
    return same;
}

static void TestPaintJournaledWithoutMemory(void)
{
    // Painting while journaling allocates nothing, so a host allocator
    // that refuses cannot drop the op from the tape after the paint is
    // applied: the tape replays to the painted world.
    bool refuse = false;
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.allocator = (m3Allocator){ToggleAlloc, ToggleFree, &refuse};
    m3World* world = NewWorld(&wd);
    m3ShapeId floor = Quad(world);
    int32_t size = m3World_GetSnapshotSize(world);
    uint8_t* unpainted = (uint8_t*)malloc((size_t)size);
    uint8_t* painted = (uint8_t*)malloc((size_t)size);
    CHECK(unpainted != NULL && painted != NULL && Snapshot(world, unpainted, size) == size,
          "the unpainted world is snapshotted");
    static uint8_t tape[4096];
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the journal starts");
    m3MeshSurfaceMaterial mats[1];
    memset(mats, 0, sizeof(mats));
    mats[0].friction = 0.02f;
    uint8_t tri[2] = {0, 0};
    refuse = true;
    CHECK_ACCEPTS(world, m3Shape_SetMeshMaterials(world, floor, mats, 1, tri),
                  "the paint applies with no memory to spare");
    refuse = false;
    CHECK(Snapshot(world, painted, size) == size && !SnapshotIs(world, unpainted, size),
          "the paint is state");
    int32_t tapeBytes = StopJournal(world);
    CHECK(tapeBytes > 0, "the paint is on the tape");
    CHECK(Restore(world, unpainted, size) && SnapshotIs(world, unpainted, size),
          "the world rolls back to unpainted");
    CHECK(ReplayJournal(world, tape, tapeBytes) && SnapshotIs(world, painted, size),
          "the tape replays to the painted world");
    free(painted);
    free(unpainted);
    m3DestroyWorld(world);
}

int main(void)
{
    TestPaintedFriction();
    TestConveyorTriangle();
    TestPaintWalls();
    TestPaintJournaledWithoutMemory();
    if (s_failures == 0)
    {
        printf("test_meshmaterial: all passed\n");
        return 0;
    }
    printf("test_meshmaterial: %d FAILURES\n", s_failures);
    return 1;
}
