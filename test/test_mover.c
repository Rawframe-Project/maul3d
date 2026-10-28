// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mover toolkit gate: pure capsule casts, contact plane
// collection, and the plane accumulator. Nothing here mutates a
// world: the hash fence proves it.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Motion locks from bits 0..5: linear x, y, z, then angular x, y, z.
static m3MotionLocks Locks(uint32_t bits)
{
    m3MotionLocks l = {(bits & 0x01u) != 0, (bits & 0x02u) != 0, (bits & 0x04u) != 0,
                       (bits & 0x08u) != 0, (bits & 0x10u) != 0, (bits & 0x20u) != 0};
    return l;
}

static void TestRigidWalkerRecipe(void)
{
    // The rigid character RECIPE on the public API alone: a
    // dynamic capsule with angular locks, steered by forces toward
    // a target speed, grounded by a ray. It crosses the yard
    // upright and stops when told; the guide documents exactly
    // this loop.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId hero = NewBody(world, &bd);
    m3ShapeDef hs = m3DefaultShapeDef();
    hs.density = 985.0f;
    // Frictionless BY RECIPE: an externally-forced capsule loses a
    // tug of war against static friction (0.42 mix here holds 2290
    // N against a 2000 N drive and the walker stands still); the
    // drive does the braking too, the classic controller contract.
    hs.friction = 0.0f;
    m3Capsule cap = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.35f};
    NewCapsuleShape(world, hero, &hs, &cap);
    m3Body_SetMotionLocks(world, hero, Locks(0x38)); // angular x, y, z: upright
    float mass = 556.0f;                             // the capsule at density 985

    for (int32_t i = 0; i < 300; ++i)
    {
        m3Vec3 v = m3Body_GetLinearVelocity(world, hero);
        float want = i < 180 ? 3.0f : 0.0f; // walk, then halt
        // Grounding via the mover toolkit itself: collect the
        // contact planes around the body's own capsule and look
        // for an upward-facing one that is NOT our own shape (a
        // ray would first find our own hull; the plane list makes
        // the self-filter one comparison).
        m3MoverPlane gp[8];
        int32_t gn =
            m3World_CollideMover(world, m3Body_GetPosition(world, hero), 0.5f, 0.36f, 0.05f, gp, 8);
        int onGround = 0;
        for (int32_t g = 0; g < gn; ++g)
        {
            if (gp[g].normal.y > 0.7f &&
                m3Shape_GetBody(world, gp[g].shapeId).index1 != hero.index1)
            {
                onGround = 1;
            }
        }
        if (onGround)
        {
            // The capsule weighs ~556 kg at this density: budget
            // the drive like a real actuator, not a wish.
            float fx = mass * (want - v.x) * 10.0f;
            fx = fx > 2000.0f ? 2000.0f : (fx < -2000.0f ? -2000.0f : fx);
            m3Body_ApplyForce(world, hero, (m3Vec3){fx, 0.0f, 0.0f});
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(world, hero);
    m3Vec3 v = m3Body_GetLinearVelocity(world, hero);
    CHECK(p.x > 4.0, "the walker crosses the yard");
    CHECK(fabsf(v.x) < 0.3f, "the walker halts when told");
    CHECK(p.y > 0.7 && p.y < 1.1, "the walker stays upright on its locks");
    m3Quat q = m3Body_GetRotation(world, hero);
    CHECK(fabsf(q.x) + fabsf(q.y) + fabsf(q.z) < 0.01f, "no tumble ever leaked in");
    m3DestroyWorld(world);
}

// A mover resting 1 cm above a static surface gets one plane from it,
// pointing up, whatever the surface is: a mesh, a height field grid, a
// voxel slab or a box. Each surface's body origin sits off the mover,
// where a point proxy of the surface once stood for all of it.
static void TestPlanesFromEverySurface(void)
{
    static const m3Vec3 quad[4] = {
        {0.0f, 0.0f, 0.0f}, {60.0f, 0.0f, 0.0f}, {60.0f, 0.0f, 60.0f}, {0.0f, 0.0f, 60.0f}};
    static const uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    static float flat[64 * 64];
    static uint8_t slab[16 * 16 * 16];
    memset(slab, 0, sizeof(slab));
    for (int32_t i = 0; i < 16 * 16; ++i)
    {
        slab[i % 16 + 16 * 16 * (i / 16)] = 1; // the bottom layer, y 0 to 4
    }
    const float tops[4] = {0.0f, 0.0f, 4.0f, 0.5f};
    for (int32_t kind = 0; kind < 4; ++kind)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        m3World* world = NewWorld(&wd);
        m3BodyDef gd = m3DefaultBodyDef();
        gd.position = (m3Pos3){-30.0, 0.0, -30.0};
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3ShapeId surface = m3_nullShapeId;
        if (kind == 0)
        {
            surface = NewMeshShape(world, ground, &sd, quad, 4, tris, 2);
        }
        else if (kind == 1)
        {
            surface = NewHeightFieldGridShape(world, ground, &sd, flat, 64, 64, 1.0f);
        }
        else if (kind == 2)
        {
            surface = NewVoxelChunkShape(world, ground, &sd, slab, NULL, 4.0f);
        }
        else
        {
            surface = NewBoxShape(world, ground, &sd, (m3Vec3){60.0f, 0.5f, 60.0f});
        }
        m3MoverPlane planes[4];
        int32_t count = m3World_CollideMover(world, (m3Pos3){0.0, (double)tops[kind] + 1.41, 0.0},
                                             1.0f, 0.4f, 0.05f, planes, 4);
        CHECK(count == 1 && planes[0].shapeId.index1 == surface.index1 &&
                  planes[0].normal.y > 0.99f && fabsf(planes[0].separation - 0.01f) < 1.0e-4f,
              "a mover above a static surface gets its plane");
        m3DestroyWorld(world);
    }
}

int main(void)
{
    TestPlanesFromEverySurface();
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    gd.position = (m3Pos3){2.0, 1.0, 0.0};
    m3BodyId wallBody = NewBody(world, &gd);
    NewBoxShape(world, wallBody, &sd, (m3Vec3){0.2f, 2.0f, 2.0f});
    gd.position = (m3Pos3){-2.0, 1.0, 0.0};
    m3BodyId sensorBody = NewBody(world, &gd);
    m3ShapeDef sens = m3DefaultShapeDef();
    sens.isSensor = true;
    NewBoxShape(world, sensorBody, &sens, (m3Vec3){0.5f, 2.0f, 2.0f});

    uint64_t before = m3World_Hash(world);

    // Cast: a mover dropped from above strikes the floor.
    m3RayCastResult hit =
        m3World_CastMover(world, (m3Pos3){0.0, 3.0, 0.0}, 0.5f, 0.4f, (m3Vec3){0.0f, -5.0f, 0.0f});
    CHECK(hit.hit, "the falling mover strikes the floor");
    CHECK(hit.normal.y > 0.9f, "the strike normal points up");

    // Collide: standing on the floor beside the wall yields both
    // planes; the sensor contributes nothing.
    m3MoverPlane planes[8];
    int32_t n = m3World_CollideMover(world, (m3Pos3){1.35, 0.9, 0.0}, 0.5f, 0.4f, 0.05f, planes, 8);
    CHECK(n == 2, "floor and wall each contribute one plane");
    int sawFloor = 0;
    int sawWall = 0;
    for (int32_t i = 0; i < n; ++i)
    {
        if (planes[i].normal.y > 0.9f)
        {
            sawFloor = 1;
        }
        if (planes[i].normal.x < -0.9f)
        {
            sawWall = 1;
        }
    }
    CHECK(sawFloor && sawWall, "the planes face out of their shapes");
    int32_t nSensor =
        m3World_CollideMover(world, (m3Pos3){-2.0, 0.92, 0.0}, 0.5f, 0.4f, 0.05f, planes, 8);
    CHECK(nSensor == 1 && planes[0].normal.y > 0.9f,
          "a sensor is invisible; only the floor speaks");

    // Solve: a downhill-and-forward wish against the floor plane
    // keeps its slide and loses its sink.
    m3MoverPlane fp;
    fp.normal = (m3Vec3){0.0f, 1.0f, 0.0f};
    fp.separation = 0.0f;
    fp.shapeId = m3_nullShapeId;
    m3MoverMove solved = m3SolveMover((m3Vec3){1.0f, -0.5f, 0.0f}, &fp, 1);
    CHECK(fabsf(solved.translation.x - 1.0f) < 1.0e-5f, "the slide survives");
    CHECK(solved.translation.y > -1.0e-5f, "the sink is absorbed");
    CHECK(solved.pressed == 1u, "the floor is pressed");

    // The corner: two planes leave only the free axis.
    m3MoverPlane corner[3];
    corner[0] = fp;
    corner[1].normal = (m3Vec3){-1.0f, 0.0f, 0.0f};
    corner[1].separation = 0.0f;
    corner[1].shapeId = m3_nullShapeId;
    solved = m3SolveMover((m3Vec3){1.0f, -0.5f, 0.7f}, corner, 2);
    CHECK(solved.translation.x < 1.0e-5f && solved.translation.y > -1.0e-5f &&
              fabsf(solved.translation.z - 0.7f) < 1.0e-5f,
          "the corner leaves only the free axis");

    // Three walls meeting at a corner of a box stop every component that
    // points into them; the velocity clip keeps what slides.
    corner[2].normal = (m3Vec3){0.0f, 0.0f, -1.0f};
    corner[2].separation = 0.0f;
    corner[2].shapeId = m3_nullShapeId;
    solved = m3SolveMover((m3Vec3){1.0f, -0.5f, 0.7f}, corner, 3);
    CHECK(fabsf(solved.translation.x) < 1.0e-5f && fabsf(solved.translation.y) < 1.0e-5f &&
              fabsf(solved.translation.z) < 1.0e-5f,
          "a box corner stops the move entirely");
    m3Vec3 slide = m3ClipMoverVelocity((m3Vec3){1.0f, -0.5f, -0.3f}, corner, 3, solved.pressed);
    CHECK(fabsf(slide.x) < 1.0e-5f && fabsf(slide.y) < 1.0e-5f && fabsf(slide.z + 0.3f) < 1.0e-5f,
          "the clip keeps the part that leaves the corner");

    CHECK(m3World_Hash(world) == before, "the toolkit moved no bits");
    m3DestroyWorld(world);
    TestRigidWalkerRecipe();
    if (s_failures == 0)
    {
        printf("test_mover: all passed\n");
        return 0;
    }
    printf("test_mover: %d FAILURES\n", s_failures);
    return 1;
}
