// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark scenes, shared by the benchmark, which times them, and by
// test/test_bench.c, which checks the world hashes they end on. A runner
// takes an optional clock: the test passes none, since a test never reads
// the wall clock.

#include "scenes.h"

#include "helpers.h"

#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <stdlib.h>
#include <string.h>

static double Now(BenchClock* clock)
{
    return clock != NULL ? clock() : 0.0;
}

static uint64_t SplitMix(uint64_t* state)
{
    *state += 0x9E3779B97F4A7C15ull;
    uint64_t z = *state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static double RandRange(uint64_t* state, double lo, double hi)
{
    return lo + (hi - lo) * ((double)(SplitMix(state) >> 11) / 9007199254740992.0);
}

// Draws x, then y, then z. Every draw is a statement of its own: C leaves
// the order of an initializer's expressions to the compiler, and GCC and
// Clang choose differently.
static m3Vec3 RandVec3(uint64_t* state, double lo, double hi)
{
    float x = (float)RandRange(state, lo, hi);
    float y = (float)RandRange(state, lo, hi);
    float z = (float)RandRange(state, lo, hi);
    return (m3Vec3){x, y, z};
}

// Draws x, then z, in [lo, hi] around the height y.
static m3Pos3 RandPosXZ(uint64_t* state, double lo, double hi, double y)
{
    double x = RandRange(state, lo, hi);
    double z = RandRange(state, lo, hi);
    return (m3Pos3){x, y, z};
}

// Draws x, then y, then z: x and z in [lo, hi], y in [yLo, yHi].
static m3Pos3 RandPos3(uint64_t* state, double lo, double hi, double yLo, double yHi)
{
    double x = RandRange(state, lo, hi);
    double y = RandRange(state, yLo, yHi);
    double z = RandRange(state, lo, hi);
    return (m3Pos3){x, y, z};
}

static void AddPlane(m3World* world)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
}

// Scene 1: the sphere pyramid, the golden scene's big brother.
static m3World* ScenePyramid(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    m3World* world = NewWorld(&def);
    AddPlane(world);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    for (int32_t layer = 0; layer < 6; ++layer)
    {
        int32_t n = 6 - layer;
        for (int32_t i = 0; i < n; ++i)
        {
            for (int32_t j = 0; j < n; ++j)
            {
                bd.position = (m3Pos3){(double)i - 0.5 * (double)n + 0.05 * (double)layer,
                                       0.5 + 1.02 * (double)layer, (double)j - 0.5 * (double)n};
                m3BodyId body = NewBody(world, &bd);
                m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
                NewSphereShape(world, body, &sd, &ball);
            }
        }
    }
    return world;
}

// Scene 2: the hull jam, SAT-heavy boxes and rocks in a pit.
static m3World* SceneHullJam(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    m3World* world = NewWorld(&def);
    AddPlane(world);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    uint64_t rng = 0xBEEFull;
    for (int32_t k = 0; k < 80; ++k)
    {
        bd.position = RandPosXZ(&rng, -2.0, 2.0, 0.6 + 0.5 * (double)k);
        m3BodyId body = NewBody(world, &bd);
        if (k % 3 == 0)
        {
            m3Vec3 cloud[12];
            for (int32_t p = 0; p < 12; ++p)
            {
                cloud[p] = RandVec3(&rng, -0.3, 0.3);
            }
            if (!m3Shape_IsValid(world, NewHullShape(world, body, &sd, cloud, 12)))
            {
                NewBoxShape(world, body, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
            }
        }
        else
        {
            NewBoxShape(world, body, &sd, (m3Vec3){0.3f, 0.25f, 0.35f});
        }
    }
    return world;
}

// Scene 3b: the grand mesh, a ~60k-triangle terrain under
// a crate rain: the mesh ceiling, timed and pinned.
static m3World* SceneMeshGrand(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-43.0, 0.0, -43.0};
    m3BodyId terrain = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    enum
    {
        GG = 174 // (174 - 1)^2 * 2 = 59858 triangles
    };
    static m3Vec3 verts[GG * GG];
    static uint16_t tris[6 * (GG - 1) * (GG - 1)];
    for (int32_t z = 0; z < GG; ++z)
    {
        for (int32_t x = 0; x < GG; ++x)
        {
            // The engine's own cosine and sine: the platform's sinf and
            // cosf differ in the last bits (Windows x64 against the rest),
            // which would move the terrain and so the pinned hash.
            float ripple = m3ComputeCosSin(0.31f * (float)x).s;
            float swell = m3ComputeCosSin(0.23f * (float)z).c;
            float h = 0.2f * ripple + 0.2f * swell;
            verts[z * GG + x] = (m3Vec3){0.5f * (float)x, h, 0.5f * (float)z};
        }
    }
    int32_t n = 0;
    for (int32_t z = 0; z < GG - 1; ++z)
    {
        for (int32_t x = 0; x < GG - 1; ++x)
        {
            uint16_t v00 = (uint16_t)(z * GG + x);
            uint16_t v10 = (uint16_t)(z * GG + x + 1);
            uint16_t v01 = (uint16_t)((z + 1) * GG + x);
            uint16_t v11 = (uint16_t)((z + 1) * GG + x + 1);
            tris[n++] = v00;
            tris[n++] = v11;
            tris[n++] = v10;
            tris[n++] = v00;
            tris[n++] = v01;
            tris[n++] = v11;
        }
    }
    NewMeshShape(world, terrain, &sd, verts, GG * GG, tris, n / 3);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    uint64_t rng = 0x6BA2Dull;
    for (int32_t k = 0; k < 80; ++k)
    {
        bd.position = RandPosXZ(&rng, -30.0, 30.0, 2.0 + 0.35 * (double)k);
        m3BodyId body = NewBody(world, &bd);
        NewBoxShape(world, body, &sd, (m3Vec3){0.35f, 0.35f, 0.35f});
    }
    return world;
}

// Scene 3: the mesh field, a midphase workout.
static m3World* SceneMeshField(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-16.0, 0.0, -16.0};
    m3BodyId terrain = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    float heights[32 * 32];
    uint64_t hseed = 0xFEEDull;
    for (int32_t i = 0; i < 32 * 32; ++i)
    {
        heights[i] = (float)RandRange(&hseed, 0.0, 0.8);
    }
    NewHeightFieldMeshShape(world, terrain, &sd, heights, 32, 32, 1.0f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    uint64_t rng = 0xFACEull;
    for (int32_t k = 0; k < 60; ++k)
    {
        bd.position = RandPosXZ(&rng, -10.0, 10.0, 2.0 + 0.4 * (double)k);
        m3BodyId body = NewBody(world, &bd);
        int32_t family = k % 3;
        if (family == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.35f};
            NewSphereShape(world, body, &sd, &ball);
        }
        else if (family == 1)
        {
            NewBoxShape(world, body, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
        }
        else
        {
            m3Capsule capsule = {{-0.3f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, 0.2f};
            NewCapsuleShape(world, body, &sd, &capsule);
        }
    }
    return world;
}

// Scene 4: voxfort, the destruction demo. A hollow voxel fort is
// shelled by
// deterministic charges; the HOST reads fragment events and turns
// recipes into dynamic bodies (small islands become QuickHull
// bodies from their voxel corner clouds, large ones become their
// bounds boxes with density matched to the event mass), then
// bullets rake the breach. Public API only, front to back.
static m3World* SceneVoxfort(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 512;
    def.shapeCapacity = 512;
    def.voxelCapacity = 2;
    m3World* world = NewWorld(&def);
    AddPlane(world);

    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    // A hollow keep: footprint 2..13, walls one voxel thick, eight
    // voxels tall, on a solid base layer (the anchor).
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y <= 8; ++y)
    {
        for (int32_t i = 2; i <= 13; ++i)
        {
            voxels[i + 16 * (y + 16 * 2)] = 1;
            voxels[i + 16 * (y + 16 * 13)] = 1;
            voxels[2 + 16 * (y + 16 * i)] = 1;
            voxels[13 + 16 * (y + 16 * i)] = 1;
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId keepBody = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    NewVoxelChunkShape(world, keepBody, &sd, voxels, NULL, 1.0f);
    return world;
}

// The host recipe: an island becomes a dynamic body. Small islands
// get the full QuickHull treatment from their voxel corner clouds;
// larger ones get their bounds box; density matches the event mass.
static void SpawnFragment(m3World* world, const m3FragmentEvent* ev, const uint16_t* recipe)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = ev->worldCenter;
    m3BodyId body = NewBody(world, &bd);
    m3Vec3 half = {0.5f * (float)(ev->upperBound[0] - ev->lowerBound[0] + 1),
                   0.5f * (float)(ev->upperBound[1] - ev->lowerBound[1] + 1),
                   0.5f * (float)(ev->upperBound[2] - ev->lowerBound[2] + 1)};
    m3ShapeDef fd = m3DefaultShapeDef();
    fd.friction = 0.6f;
    fd.density = ev->mass / (8.0f * half.x * half.y * half.z);
    if (ev->voxelCount <= 8 && ev->recipeStart >= 0)
    {
        m3Vec3 cloud[64];
        int32_t points = 0;
        for (int32_t k = 0; k < ev->recipeCount; ++k)
        {
            uint16_t v = recipe[ev->recipeStart + k];
            float x = (float)(v % 16);
            float y = (float)((v / 16) % 16);
            float z = (float)(v / 256);
            for (int32_t c = 0; c < 8; ++c)
            {
                cloud[points++] = (m3Vec3){x + ((c & 1) != 0 ? 1.0f : 0.0f) - ev->localCenter.x,
                                           y + ((c & 2) != 0 ? 1.0f : 0.0f) - ev->localCenter.y,
                                           z + ((c & 4) != 0 ? 1.0f : 0.0f) - ev->localCenter.z};
            }
        }
        if (m3Shape_IsValid(world, NewHullShape(world, body, &fd, cloud, points)))
        {
            return;
        }
        // A degenerate cloud (a lone flat island) falls back to the
        // bounds box below, deterministically.
    }
    NewBoxShape(world, body, &fd, half);
}

// Scene 5: the city block, the 5k-body scale proof. A grid
// of welded voxel tower stacks takes a five-thousand-body rain of
// mixed shapes; mid-run a fracture wave carves the towers and the
// host recipe turns the falling sections into dynamic rubble. The
// three phases (settle, storm, aftermath) time separately: settle
// is the landing pile-up, storm is destruction under full load,
// aftermath is the sleeping-heavy sandbox regime a real game
// lives in. No engine changes rode in with this scene: it is the
// measuring stick, not the surgery.
static m3World* SceneCityblock(m3ShapeId* outTowers, int32_t* outTowerCount)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8192;
    def.shapeCapacity = 8192;
    def.voxelCapacity = 20;
    m3World* world = NewWorld(&def);
    AddPlane(world);

    // Nine tower stacks on a 3x3 grid, two welded chunks tall.
    // Each chunk: a full base layer (the anchor) under a 6x6-cell
    // shaft. Half-meter cells: 8 meters of chunk, 16 of tower.
    static uint8_t tower[16 * 16 * 16];
    memset(tower, 0, sizeof(tower));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            tower[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y < 16; ++y)
    {
        for (int32_t z = 5; z <= 10; ++z)
        {
            for (int32_t x = 5; x <= 10; ++x)
            {
                tower[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    int32_t towers = 0;
    for (int32_t gz = 0; gz < 3; ++gz)
    {
        for (int32_t gx = 0; gx < 3; ++gx)
        {
            double bx = -28.0 + 20.0 * (double)gx;
            double bz = -28.0 + 20.0 * (double)gz;
            m3BodyDef gd = m3DefaultBodyDef();
            gd.position = (m3Pos3){bx, 0.0, bz};
            m3ShapeId lower =
                NewVoxelChunkShape(world, NewBody(world, &gd), &sd, tower, NULL, 0.5f);
            gd.position = (m3Pos3){bx, 8.0, bz}; // exactly one extent up: welded
            m3ShapeId upper =
                NewVoxelChunkShape(world, NewBody(world, &gd), &sd, tower, NULL, 0.5f);
            if (outTowers != NULL && towers + 2 <= 18)
            {
                outTowers[towers] = lower;
                outTowers[towers + 1] = upper;
            }
            towers += 2;
        }
    }
    if (outTowerCount != NULL)
    {
        *outTowerCount = towers;
    }

    // The rain: five thousand mixed bodies seeded once, spread over
    // the whole block, high enough to land through the settle phase.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    sd.rollingResistance = 0.05f; // soft ground: rain that lands
                                  // eventually STOPS ROLLING and
                                  // sleeps
    uint64_t rng = 0xC17B10C4ull;
    for (int32_t k = 0; k < 5000; ++k)
    {
        bd.position = RandPos3(&rng, -34.0, 34.0, 10.0, 42.0);
        m3BodyId body = NewBody(world, &bd);
        int32_t family = k % 4;
        if (family == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, (float)RandRange(&rng, 0.2, 0.45)};
            NewSphereShape(world, body, &sd, &ball);
        }
        else if (family == 1)
        {
            NewBoxShape(world, body, &sd, RandVec3(&rng, 0.15, 0.4));
        }
        else if (family == 2)
        {
            m3Capsule capsule = {
                {-0.25f, 0.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, (float)RandRange(&rng, 0.12, 0.25)};
            NewCapsuleShape(world, body, &sd, &capsule);
        }
        else
        {
            m3Vec3 cloud[8];
            for (int32_t c = 0; c < 8; ++c)
            {
                cloud[c] = RandVec3(&rng, -0.3, 0.3);
            }
            if (!m3Shape_IsValid(world, NewHullShape(world, body, &sd, cloud, 8)))
            {
                NewBoxShape(world, body, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
            }
        }
    }
    return world;
}

// Scene 5: the joint rig. Hinge chains hang from the ceiling, balls hang
// on sliders whose anchors sit 2 m off their centers, and box pairs welded
// at their far corners rest on the floor: every joint kind that solves a
// coupled block, on levers that couple its rows.
static m3World* SceneJointRig(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 1024;
    def.shapeCapacity = 1024;
    def.jointCapacity = 1024;
    m3World* world = NewWorld(&def);
    AddPlane(world);
    m3BodyDef sd0 = m3DefaultBodyDef();
    m3BodyId ceiling = NewBody(world, &sd0);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t c = 0; c < 32; ++c)
    {
        double x = -16.0 + (double)c;
        m3BodyId previous = ceiling;
        m3Vec3 previousAnchor = {(float)x, 20.0f, 0.0f};
        for (int32_t k = 0; k < 12; ++k)
        {
            bd.position = (m3Pos3){x + 0.5 * (double)k + 0.25, 20.0, 0.0};
            m3BodyId link = NewBody(world, &bd);
            m3Capsule pill = {{-0.2f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f}, 0.05f};
            NewCapsuleShape(world, link, &sd, &pill);
            m3JointDef jd = m3DefaultJointDef();
            jd.type = m3_revoluteJoint;
            jd.bodyIdA = previous;
            jd.bodyIdB = link;
            jd.localAnchorA = previousAnchor;
            jd.localAnchorB = (m3Vec3){-0.25f, 0.0f, 0.0f};
            jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
            jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
            NewJoint(world, &jd);
            previous = link;
            previousAnchor = (m3Vec3){0.25f, 0.0f, 0.0f};
        }
    }
    for (int32_t i = 0; i < 32; ++i)
    {
        double z = 6.0 + (double)i;
        bd.position = (m3Pos3){2.0, 10.0, z};
        m3BodyId ball = NewBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.25f};
        NewSphereShape(world, ball, &sd, &sphere);
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_prismaticJoint;
        jd.bodyIdA = ceiling;
        jd.bodyIdB = ball;
        jd.localAnchorA = (m3Vec3){0.0f, 10.0f, (float)z};
        jd.localAnchorB = (m3Vec3){-2.0f, 0.0f, 0.0f};
        jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
        jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
        jd.enableLimit = true;
        jd.lowerLimit = -1.0f;
        jd.upperLimit = 1.0f;
        NewJoint(world, &jd);
    }
    for (int32_t i = 0; i < 32; ++i)
    {
        double x = -16.0 + (double)i;
        bd.position = (m3Pos3){x, 0.5, -6.0};
        m3BodyId a = NewBody(world, &bd);
        NewBoxShape(world, a, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        bd.position = (m3Pos3){x, 1.3, -6.0};
        m3BodyId b = NewBody(world, &bd);
        NewBoxShape(world, b, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_weldJoint;
        jd.bodyIdA = a;
        jd.bodyIdB = b;
        jd.localAnchorA = (m3Vec3){0.4f, 0.4f, 0.4f};
        jd.localAnchorB = (m3Vec3){0.4f, -0.4f, 0.4f};
        NewJoint(world, &jd);
    }
    return world;
}

// Scene 6: the vehicle rig. Cars of a box chassis and four sphere wheels
// on wheel joints, each sprung, limited and driven, the front pair
// steering, drive across the floor in four lanes.
static m3World* SceneVehicleRig(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    def.jointCapacity = 256;
    m3World* world = NewWorld(&def);
    AddPlane(world);
    for (int32_t car = 0; car < 32; ++car)
    {
        m3Pos3 at = {-24.0 + 6.0 * (double)(car % 8), 0.7, -12.0 + 8.0 * (double)(car / 8)};
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = at;
        m3BodyId chassis = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        sd.density = 300.0f;
        NewBoxShape(world, chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
        for (int32_t w = 0; w < 4; ++w)
        {
            m3Vec3 local = {(w & 1) != 0 ? 0.8f : -0.8f, -0.35f, (w & 2) != 0 ? 0.45f : -0.45f};
            bd.position =
                (m3Pos3){at.x + (double)local.x, at.y + (double)local.y, at.z + (double)local.z};
            m3BodyId wheel = NewBody(world, &bd);
            m3ShapeDef ws = m3DefaultShapeDef();
            ws.density = 120.0f;
            ws.friction = 1.0f;
            m3Sphere tire = {{0.0f, 0.0f, 0.0f}, 0.3f};
            NewSphereShape(world, wheel, &ws, &tire);
            m3JointDef jd = m3DefaultJointDef();
            jd.type = m3_wheelJoint;
            jd.bodyIdA = chassis;
            jd.bodyIdB = wheel;
            jd.localAnchorA = local;
            jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
            jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
            jd.enableLimit = true;
            jd.lowerLimit = -0.1f;
            jd.upperLimit = 0.1f;
            jd.enableMotor = true;
            jd.motorSpeed = -10.0f;
            jd.maxMotorEffort = 200.0f;
            m3JointId joint = NewJoint(world, &jd);
            m3Joint_SetSpringHertz(world, joint, 4.0f);
            m3Joint_SetSpringDampingRatio(world, joint, 0.7f);
            m3Joint_EnableSpring(world, joint, true);
            m3Joint_SetTargetTranslation(world, joint, 0.0f);
            if ((w & 1) != 0)
            {
                m3WheelJoint_SetSteeringHertz(world, joint, 5.0f);
                m3WheelJoint_SetSteeringDampingRatio(world, joint, 0.7f);
                m3WheelJoint_SetMaxSteeringTorque(world, joint, 0.0f);
                m3WheelJoint_SetTargetSteeringAngle(world, joint, car % 2 != 0 ? 0.2f : -0.2f);
                m3WheelJoint_EnableSteering(world, joint, true);
            }
        }
    }
    return world;
}

// One ragdoll at its pelvis: a torso, a head on a neck socket, arms on
// shoulder sockets with hinged elbows, and legs on hip sockets with
// hinged knees, ten bodies on nine joints.
static void AddRagdoll(m3World* world, m3Pos3 at)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef sd = m3DefaultShapeDef();
    bd.position = (m3Pos3){at.x, at.y + 0.3, at.z};
    m3BodyId torso = NewBody(world, &bd);
    NewBoxShape(world, torso, &sd, (m3Vec3){0.2f, 0.3f, 0.12f});
    bd.position = (m3Pos3){at.x, at.y + 0.8, at.z};
    m3BodyId head = NewBody(world, &bd);
    m3Sphere skull = {{0.0f, 0.0f, 0.0f}, 0.12f};
    NewSphereShape(world, head, &sd, &skull);
    m3JointDef socket = m3DefaultJointDef();
    socket.type = m3_sphericalJoint;
    socket.bodyIdA = torso;
    socket.bodyIdB = head;
    socket.localAnchorA = (m3Vec3){0.0f, 0.35f, 0.0f};
    socket.localAnchorB = (m3Vec3){0.0f, -0.15f, 0.0f};
    socket.enableCone = true;
    socket.coneAngle = 0.5f;
    NewJoint(world, &socket);
    m3Capsule limb = {{0.0f, -0.15f, 0.0f}, {0.0f, 0.15f, 0.0f}, 0.05f};
    for (int32_t side = 0; side < 2; ++side)
    {
        float s = side == 0 ? -1.0f : 1.0f;
        for (int32_t leg = 0; leg < 2; ++leg)
        {
            float rootX = leg == 0 ? 0.28f * s : 0.1f * s;
            float rootY = leg == 0 ? 0.25f : -0.3f;
            bd.position = (m3Pos3){at.x + (double)rootX, at.y + 0.3 + (double)rootY - 0.2, at.z};
            m3BodyId upper = NewBody(world, &bd);
            NewCapsuleShape(world, upper, &sd, &limb);
            bd.position = (m3Pos3){at.x + (double)rootX, at.y + 0.3 + (double)rootY - 0.55, at.z};
            m3BodyId lower = NewBody(world, &bd);
            NewCapsuleShape(world, lower, &sd, &limb);
            socket.bodyIdA = torso;
            socket.bodyIdB = upper;
            socket.localAnchorA = (m3Vec3){rootX, rootY, 0.0f};
            socket.localAnchorB = (m3Vec3){0.0f, 0.2f, 0.0f};
            socket.coneAngle = 1.2f;
            NewJoint(world, &socket);
            m3JointDef hinge = m3DefaultJointDef();
            hinge.type = m3_revoluteJoint;
            hinge.bodyIdA = upper;
            hinge.bodyIdB = lower;
            hinge.localAnchorA = (m3Vec3){0.0f, -0.175f, 0.0f};
            hinge.localAnchorB = (m3Vec3){0.0f, 0.175f, 0.0f};
            hinge.localAxisA = (m3Vec3){1.0f, 0.0f, 0.0f};
            hinge.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f};
            hinge.enableLimit = true;
            hinge.lowerLimit = leg == 0 ? 0.0f : -2.0f;
            hinge.upperLimit = leg == 0 ? 2.0f : 0.0f;
            NewJoint(world, &hinge);
        }
    }
}

// Scene 7: the ragdolls. Forty-eight of them fall in a staggered heap.
static m3World* SceneRagdolls(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 1024;
    def.shapeCapacity = 1024;
    def.jointCapacity = 1024;
    m3World* world = NewWorld(&def);
    AddPlane(world);
    for (int32_t i = 0; i < 48; ++i)
    {
        AddRagdoll(world, (m3Pos3){-3.0 + 1.5 * (double)(i % 4), 2.0 + 1.2 * (double)(i / 4),
                                   -1.0 + 0.7 * (double)((i / 4) % 3) + 0.1 * (double)(i % 4)});
    }
    return world;
}

uint64_t BenchStepScene(BenchScene scene, int32_t steps, BenchClock* clock, double* ms)
{
    static m3World* (*const builders[BENCH_SCENE_COUNT])(void) = {
        ScenePyramid,  SceneHullJam,    SceneMeshField, SceneMeshGrand,
        SceneJointRig, SceneVehicleRig, SceneRagdolls};
    m3World* world = builders[scene]();
    double t0 = Now(clock);
    for (int32_t i = 0; i < steps; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    *ms = Now(clock) - t0;
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

uint64_t BenchVoxfort(int32_t steps, BenchClock* clock, double* ms)
{
    m3World* world = SceneVoxfort();
    // The chunk shape id is deterministic: the first shape after the
    // plane (index1 = 2 in this scene).
    m3ShapeId keep = {2, 1};
    double t0 = Now(clock);
    uint64_t rng = 0xF0F7ull;
    int32_t shells = 0;
    for (int32_t i = 0; i < steps; ++i)
    {
        if (i % 15 == 5)
        {
            // A shell: carve a deterministic 2x2x2 charge out of the
            // south wall, marching along it, then spawn what fell.
            int32_t sx = 2 + (shells * 3) % 11;
            int32_t sy = 1 + (shells * 2) % 6;
            int32_t lo[3] = {sx, sy, 2};
            int32_t hi[3] = {sx + 1 < 16 ? sx + 1 : 15, sy + 1, 2};
            int32_t cleared = 0;
            (void)m3VoxelChunkShape_ClearBox(world, keep, lo, hi, &cleared);
            shells += 1;
            m3FragmentEvents fragments = m3World_GetFragmentEvents(world);
            for (int32_t e = 0; e < fragments.fragmentCount && e < 16; ++e)
            {
                SpawnFragment(world, &fragments.fragmentEvents[e], fragments.recipe);
            }
        }
        if (i % 45 == 20)
        {
            // Bullets rake the wall: stopped by standing voxels,
            // through the breaches once the charges open them.
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.enableBullet = true;
            bd.position = (m3Pos3){RandRange(&rng, -4.0, 4.0), 2.5, -20.0};
            bd.linearVelocity = (m3Vec3){0.0f, 0.0f, 150.0f};
            m3BodyId bullet = NewBody(world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            m3Sphere slug = {{0.0f, 0.0f, 0.0f}, 0.15f};
            NewSphereShape(world, bullet, &sd, &slug);
        }
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    *ms = Now(clock) - t0;
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

uint64_t BenchCityblock(int32_t totalSteps, BenchClock* clock, double phaseMs[3])
{
    m3ShapeId towerIds[18];
    int32_t towerCount = 0;
    m3World* world = SceneCityblock(towerIds, &towerCount);
    int32_t phase = totalSteps / 3;
    int32_t carves = 0;
    double marks[4] = {0.0, 0.0, 0.0, 0.0};
    marks[0] = Now(clock);
    for (int32_t i = 0; i < totalSteps; ++i)
    {
        if (i >= phase && i < 2 * phase && (i - phase) % 10 == 0)
        {
            // The fracture wave: slice a shaft above its base, one
            // tower after another; what falls becomes host rubble.
            m3ShapeId victim = towerIds[carves % towerCount];
            int32_t cy = 2 + (carves / towerCount) % 3;
            int32_t lo[3] = {5, cy, 5};
            int32_t hi[3] = {10, cy, 10};
            int32_t cleared = 0;
            (void)m3VoxelChunkShape_ClearBox(world, victim, lo, hi, &cleared);
            carves += 1;
            m3FragmentEvents fragments = m3World_GetFragmentEvents(world);
            for (int32_t e = 0; e < fragments.fragmentCount && e < 24; ++e)
            {
                SpawnFragment(world, &fragments.fragmentEvents[e], fragments.recipe);
            }
        }
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        if (i + 1 == phase)
        {
            marks[1] = Now(clock);
        }
        if (i + 1 == 2 * phase)
        {
            marks[2] = Now(clock);
        }
    }
    marks[3] = Now(clock);
    phaseMs[0] = marks[1] - marks[0];
    phaseMs[1] = marks[2] - marks[1];
    phaseMs[2] = marks[3] - marks[2];
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

BenchRollback BenchRollback5k(int32_t settleSteps, BenchClock* clock)
{
    // The city block run to mid-storm, then the full snapshot, a restore
    // and a re-fought tail.
    BenchRollback result = {0};
    m3ShapeId towerIds[18];
    int32_t towerCount = 0;
    m3World* world = SceneCityblock(towerIds, &towerCount);
    for (int32_t i = 0; i < settleSteps; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int64_t snapBytes = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    double t0 = Now(clock);
    result.bytes = Snapshot(world, snap, snapBytes);
    double t1 = Now(clock);
    uint64_t before = m3World_Hash(world);
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t after = m3World_Hash(world);
    double t2 = Now(clock);
    Restore(world, snap, result.bytes);
    double t3 = Now(clock);
    result.restoreExact = m3World_Hash(world) == before;
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    result.refightExact = m3World_Hash(world) == after;
    result.snapshotMs = t1 - t0;
    result.restoreMs = t3 - t2;
    free(snap);
    m3DestroyWorld(world);
    return result;
}

uint64_t BenchSmoke10k(int32_t steps, BenchClock* clock, double* ms)
{
    // Ten thousand mixed bodies rain onto a plane.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16384;
    def.shapeCapacity = 16384;
    def.voxelCapacity = 20;
    m3World* sea = NewWorld(&def);
    AddPlane(sea);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    sd.rollingResistance = 0.05f;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    uint64_t rng = 0x10A11ull;
    for (int32_t k = 0; k < 10000; ++k)
    {
        bd.position = RandPos3(&rng, -46.0, 46.0, 8.0, 60.0);
        m3BodyId body = NewBody(sea, &bd);
        if (k % 3 == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, (float)RandRange(&rng, 0.2, 0.4)};
            NewSphereShape(sea, body, &sd, &ball);
        }
        else if (k % 3 == 1)
        {
            NewBoxShape(sea, body, &sd, RandVec3(&rng, 0.15, 0.35));
        }
        else
        {
            m3Capsule c = {
                {-0.2f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f}, (float)RandRange(&rng, 0.12, 0.2)};
            NewCapsuleShape(sea, body, &sd, &c);
        }
    }
    double t0 = Now(clock);
    for (int32_t i = 0; i < steps; ++i)
    {
        (void)m3World_Step(sea, 1.0f / 60.0f, 4);
    }
    *ms = Now(clock) - t0;
    uint64_t hash = m3World_Hash(sea);
    m3DestroyWorld(sea);
    return hash;
}

uint64_t BenchSoftfield(int32_t steps, BenchClock* clock, double* ms)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2048;
    def.shapeCapacity = 2048;
    def.softBodyCapacity = 8;
    m3World* field = NewWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef ground = m3DefaultBodyDef();
    NewBoxShape(field, NewBody(field, &ground), &sd, (m3Vec3){10.0f, 0.5f, 10.0f});
    // The far field: spheres, capsules and boxes in a ring 40 to 90 m out.
    uint64_t rng = 0x50F7F1E1Dull;
    for (int32_t k = 0; k < 2000; ++k)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        m3Pos3 p = RandPos3(&rng, -90.0, 90.0, 0.0, 20.0);
        if (p.x > -40.0 && p.x < 40.0 && p.z > -40.0 && p.z < 40.0)
        {
            p.x += p.x < 0.0 ? -50.0 : 50.0;
        }
        bd.position = p;
        m3BodyId body = NewBody(field, &bd);
        if (k % 3 == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, (float)RandRange(&rng, 0.3, 1.0)};
            NewSphereShape(field, body, &sd, &ball);
        }
        else if (k % 3 == 1)
        {
            m3Capsule c = {
                {-0.5f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, (float)RandRange(&rng, 0.2, 0.5)};
            NewCapsuleShape(field, body, &sd, &c);
        }
        else
        {
            NewBoxShape(field, body, &sd, RandVec3(&rng, 0.3, 1.0));
        }
    }
    for (int32_t i = 0; i < 4; ++i)
    {
        m3SoftBodyDef cloth = m3DefaultSoftBodyDef();
        cloth.position = (m3Pos3){-8.0 + 4.5 * (double)i, 3.0 + 0.5 * (double)i, -6.0};
        cloth.countX = 12;
        cloth.countY = 1;
        cloth.countZ = 12;
        cloth.spacing = 0.25f;
        cloth.radius = 0.06f;
        NewSoftBody(field, &cloth);
    }
    for (int32_t i = 0; i < 2; ++i)
    {
        m3SoftBodyDef jelly = m3DefaultSoftBodyDef();
        jelly.position = (m3Pos3){-4.0 + 6.0 * (double)i, 2.0, 3.0};
        jelly.countX = 5;
        jelly.countY = 5;
        jelly.countZ = 5;
        jelly.spacing = 0.3f;
        jelly.compliance = 1.0e-4f;
        jelly.radius = 0.1f;
        jelly.pressure = 1.0f;
        NewSoftBody(field, &jelly);
    }
    double t0 = Now(clock);
    for (int32_t i = 0; i < steps; ++i)
    {
        (void)m3World_Step(field, 1.0f / 60.0f, 4);
    }
    *ms = Now(clock) - t0;
    uint64_t hash = m3World_Hash(field);
    m3DestroyWorld(field);
    return hash;
}
