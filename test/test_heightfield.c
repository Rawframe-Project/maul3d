// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The native terrain gate: a grid heightfield lands as raw
// samples (the low-memory path beside meshes), refuses hostile grids
// in both doors, carries bodies on its surface, and answers rays and
// overlaps.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static void FillHills(float* h, int32_t nx, int32_t nz)
{
    for (int32_t z = 0; z < nz; ++z)
    {
        for (int32_t x = 0; x < nx; ++x)
        {
            h[z * nx + x] = 0.25f * (float)((x * 7 + z * 3) % 5);
        }
    }
}

static void TestCreateAndWalls(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float hills[16 * 16];
    FillHills(hills, 16, 16);
    m3ShapeId terrain = NewHeightFieldGridShape(world, ground, &sd, hills, 16, 16, 0.5f);
    CHECK(m3Shape_IsValid(world, terrain), "the native grid lands");

    m3BodyDef dd = m3DefaultBodyDef();
    dd.type = m3_dynamicBody;
    dd.position = (m3Pos3){0.0, 5.0, 0.0};
    m3BodyId mover = NewBody(world, &dd);
    uint64_t before = m3World_Hash(world);
    CHECK(!m3Shape_IsValid(world, NewHeightFieldGridShape(world, mover, &sd, hills, 16, 16, 0.5f)),
          "a dynamic body refuses terrain");
    CHECK(!m3Shape_IsValid(world, NewHeightFieldGridShape(world, ground, &sd, hills, 1, 16, 0.5f)),
          "a one-column grid refuses");
    CHECK(!m3Shape_IsValid(world, NewHeightFieldGridShape(world, ground, &sd, hills, 16, 16, 0.0f)),
          "a zero cell refuses");
    static float poisoned[16 * 16];
    FillHills(poisoned, 16, 16);
    poisoned[40] = NAN;
    CHECK(!m3Shape_IsValid(world,
                           NewHeightFieldGridShape(world, ground, &sd, poisoned, 16, 16, 0.5f)),
          "a NaN sample refuses");
    CHECK(m3World_Hash(world) == before, "refused terrain moved no bits");
    DestroyBody(world, mover);

    DestroyShape(world, terrain);
    CHECK(!m3Shape_IsValid(world, terrain), "destroyed terrain goes stale");
    m3DestroyWorld(world);
}

static void TestContacts(void)
{
    // Bodies land ON native terrain. A ball rests on a flat
    // grid, a box slides across it without ghost snags, and a ball
    // on a slope rolls downhill.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 16;
    wd.shapeCapacity = 16;
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float flat[33 * 33]; // zeros: a flat 16m field
    m3ShapeId terrain = NewHeightFieldGridShape(world, ground, &sd, flat, 33, 33, 0.5f);
    CHECK(m3Shape_IsValid(world, terrain), "the flat field lands");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
    NewSphereShape(world, ball, &sd, &s);

    bd.position = (m3Pos3){-4.0, 1.0, 3.0};
    bd.linearVelocity = (m3Vec3){4.0f, 0.0f, 0.0f};
    m3BodyId slider = NewBody(world, &bd);
    m3ShapeDef bs = m3DefaultShapeDef();
    bs.friction = 0.05f;
    NewBoxShape(world, slider, &bs, (m3Vec3){0.3f, 0.3f, 0.3f});

    double maxSliderY = 0.0;
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        if (i > 60)
        {
            double y = m3Body_GetPosition(world, slider).y;
            maxSliderY = y > maxSliderY ? y : maxSliderY;
        }
    }
    double ballY = m3Body_GetPosition(world, ball).y;
    CHECK(ballY > 0.3 && ballY < 0.5, "the ball rests on the field");
    CHECK(m3Body_GetPosition(world, slider).x > -1.0, "the slick box slides across cells");
    CHECK(maxSliderY < 0.45, "no ghost snag pops the slider");
    m3DestroyWorld(world);

    // The slope: heights rise along x; a ball rolls downhill (-x).
    m3WorldDef wd2 = m3DefaultWorldDef();
    wd2.bodyCapacity = 8;
    wd2.shapeCapacity = 8;
    m3World* hill = NewWorld(&wd2);
    m3BodyDef gd2 = m3DefaultBodyDef();
    gd2.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId ground2 = NewBody(hill, &gd2);
    static float ramp[33 * 33];
    for (int32_t z = 0; z < 33; ++z)
    {
        for (int32_t x = 0; x < 33; ++x)
        {
            ramp[z * 33 + x] = 0.3f * (float)x;
        }
    }
    CHECK(m3Shape_IsValid(hill, NewHeightFieldGridShape(hill, ground2, &sd, ramp, 33, 33, 0.5f)),
          "the ramp lands");
    m3BodyDef rb = m3DefaultBodyDef();
    rb.type = m3_dynamicBody;
    rb.position = (m3Pos3){0.0, 6.0, 0.0}; // the ramp is 4.8 high here
    m3BodyId roller = NewBody(hill, &rb);
    NewSphereShape(hill, roller, &sd, &s);
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(hill, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(hill, roller).x < -1.0, "the ball rolls downhill");
    m3DestroyWorld(hill);
}

static void TestRayAndOverlap(void)
{
    // Rays land on terrain triangles and the overlap family
    // sees them.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-4.0, 0.0, -4.0};
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float ramp[17 * 17];
    for (int32_t z = 0; z < 17; ++z)
    {
        for (int32_t x = 0; x < 17; ++x)
        {
            ramp[z * 17 + x] = 0.2f * (float)x;
        }
    }
    m3ShapeId terrain = NewHeightFieldGridShape(world, ground, &sd, ramp, 17, 17, 0.5f);
    CHECK(m3Shape_IsValid(world, terrain), "the ray ramp lands");

    // Straight down INSIDE a cell (a ray exactly on a shared corner
    // can lose the watertight edge test by one float ulp, the same
    // contract as the mesh path): local x = 4.1 on the y = 0.4 x
    // plane gives 1.64.
    m3CastHit hit = {0};
    bool landed =
        m3World_CastRayClosest(world, (m3Pos3){0.1, 5.0, 0.05}, (m3Vec3){0.0f, -10.0f, 0.0f},
                               m3DefaultQueryFilter(), &hit);
    CHECK(landed, "the ray finds the terrain");
    CHECK(fabs(hit.point.y - 1.64) < 0.02, "the ray lands on the sampled height");
    CHECK(hit.normal.y > 0.9f, "the slope normal points mostly up");

    // A miss beside the grid stays a miss.
    m3CastHit miss = {0};
    bool missHit =
        m3World_CastRayClosest(world, (m3Pos3){40.0, 5.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f},
                               m3DefaultQueryFilter(), &miss);
    CHECK(!missHit, "a ray beside the grid misses");

    m3ShapeId found[8];
    int32_t n = m3World_OverlapSphere(world, (m3Pos3){0.0, 1.6, 0.0}, 0.5f, m3DefaultQueryFilter(),
                                      found, 8);
    CHECK(n == 1 && found[0].index1 == terrain.index1, "the overlap family sees terrain");
    n = m3World_OverlapSphere(world, (m3Pos3){0.0, 6.0, 0.0}, 0.5f, m3DefaultQueryFilter(), found,
                              8);
    CHECK(n == 0, "a sphere above the ramp sees nothing");
    m3DestroyWorld(world);
}

// A flat grid of 1 m cells at height 2, its body's origin 2 m below
// the surface at the grid's corner. Casts, fast bodies and characters
// meet the surface everywhere; none meets the origin, which a point
// proxy of the grid once stood for.
static m3World* RaisedGrid(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-32.0, 0.0, -32.0};
    m3BodyId ground = NewBody(world, &gd);
    static float heights[64 * 64];
    for (int32_t i = 0; i < 64 * 64; ++i)
    {
        heights[i] = 2.0f;
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId terrain = NewHeightFieldGridShape(world, ground, &sd, heights, 64, 64, 1.0f);
    CHECK(m3Shape_IsValid(world, terrain), "the raised grid lands");
    return world;
}

static void TestCastsMeetTheGrid(void)
{
    m3World* world = RaisedGrid();
    m3QueryFilter all = m3DefaultQueryFilter();
    const m3Pos3 starts[2] = {{0.3, 6.0, 0.2}, {-31.7, 6.0, -31.8}};
    const m3Vec3 down = {0.0f, -10.0f, 0.0f};
    for (int32_t i = 0; i < 2; ++i)
    {
        m3CastHit sphere = {0};
        bool sphereHit = m3World_CastSphereClosest(world, starts[i], 0.5f, down, all, &sphere);
        CHECK(sphereHit && fabsf(sphere.fraction - 0.35f) < 1.0e-3f && sphere.normal.y > 0.99f,
              "a sphere cast meets the surface, in the middle and over the corner");
        m3CastHit capsule = {0};
        bool capsuleHit =
            m3World_CastCapsuleClosest(world, starts[i], (m3Vec3){0.0f, 0.25f, 0.0f},
                                       (m3Vec3){0.0f, -0.25f, 0.0f}, 0.5f, down, all, &capsule);
        CHECK(capsuleHit && fabsf(capsule.fraction - 0.325f) < 1.0e-3f,
              "a capsule cast meets the surface");
        m3CastHit box = {0};
        bool boxHit = m3World_CastBoxClosest(world, starts[i], (m3Vec3){0.5f, 0.5f, 0.5f},
                                             m3MakeIdentityQuat(), down, all, &box);
        CHECK(boxHit && fabsf(box.fraction - 0.35f) < 1.0e-3f, "a box cast meets the surface");
        m3CastHit mover = {0};
        bool moverHit = m3World_CastMover(world, starts[i], 0.5f, 0.25f, down, &mover);
        CHECK(moverHit && fabsf(mover.fraction - 0.325f) < 1.0e-3f,
              "a mover cast meets the surface");
    }
    m3DestroyWorld(world);
}

// A body crossing the grid in one step stops on it, a bullet or not, and
// a character walking down onto it stands on it.
static void TestFastBodiesAndCharactersStayOnTheGrid(void)
{
    m3World* world = RaisedGrid();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId fast[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.enableBullet = i == 1;
        bd.position = (m3Pos3){-10.0 + 20.0 * i, 6.0, 0.5};
        bd.linearVelocity = (m3Vec3){0.0f, -2000.0f, 0.0f};
        fast[i] = NewBody(world, &bd);
        m3Sphere pellet = {{0.0f, 0.0f, 0.0f}, 0.05f};
        NewSphereShape(world, fast[i], &sd, &pellet);
    }
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){5.0, 5.0, 5.0};
    m3CharacterId walker = NewCharacter(world, &cd);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(world, walker, (m3Vec3){0.0f, -0.2f, 0.0f});
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    for (int32_t i = 0; i < 2; ++i)
    {
        CHECK(m3Body_GetPosition(world, fast[i]).y > 1.9,
              "a body crossing the grid in a step stops on it");
    }
    CHECK(m3Character_GetPosition(world, walker).y > 2.0,
          "a character walking down onto the grid stands on it");
    m3DestroyWorld(world);
}

int main(void)
{
    TestCreateAndWalls();
    TestContacts();
    TestRayAndOverlap();
    TestCastsMeetTheGrid();
    TestFastBodiesAndCharactersStayOnTheGrid();
    if (s_failures == 0)
    {
        printf("test_heightfield: all passed\n");
        return 0;
    }
    printf("test_heightfield: %d FAILURES\n", s_failures);
    return 1;
}
