// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The C snippets of docs/user/, built and run as written. Each test sets
// up what its snippet assumes, runs the snippet unchanged, and checks the
// outcome the page describes. tools/check_guide.py fails when a snippet on
// a page no longer appears here.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/maul3d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// A world with a floor plane at y = -5 and a light crate dropped above
// it at the origin.
static m3World* FloorWorld(m3BodyId* crateOut)
{
    m3WorldDef worldDef = m3DefaultWorldDef();
    worldDef.bodyCapacity = 8;
    worldDef.shapeCapacity = 8;
    m3World* world = NewWorld(&worldDef);
    m3BodyDef groundDef = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &groundDef);
    m3ShapeDef shapeDef = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, -5.0f};
    NewPlaneShape(world, ground, &shapeDef, &floor);
    m3BodyDef crateDef = m3DefaultBodyDef();
    crateDef.type = m3_dynamicBody;
    crateDef.position = (m3Pos3){0.0, 2.0, 0.0};
    *crateOut = NewBody(world, &crateDef);
    m3ShapeDef crateShape = m3DefaultShapeDef();
    crateShape.density = 400.0f; // lighter than water
    NewBoxShape(world, *crateOut, &crateShape, (m3Vec3){0.3f, 0.3f, 0.3f});
    return world;
}

// water.md: a pool with a current. The crate falls in, bobs, floats at
// the surface and drifts along x; after five seconds it is still in the
// pool, whose far edge is at x = 10.
static void TestWaterPool(void)
{
    m3BodyId crate;
    m3World* world = FloorWorld(&crate);

    m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
    def.bounds.lowerBound = (m3Pos3){-10.0, -5.0, -10.0};
    def.bounds.upperBound = (m3Pos3){10.0, 0.0, 10.0}; // the surface is y = 0
    def.flow = (m3Vec3){1.5f, 0.0f, 0.0f};             // a current along x
    m3WaterVolumeId pool;
    m3Result result = m3CreateWaterVolume(world, &def, &pool);

    CHECK(result == m3_success && m3WaterVolume_IsValid(world, pool), "the pool fills");
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(world, crate);
    CHECK(p.y > -0.3 && p.y < 0.8, "the light crate floats at the surface");
    CHECK(p.x > 3.0 && p.x < 9.0, "the current carries it along x");
    m3DestroyWorld(world);
}

// A world with a floor plane at y = 0.
static m3World* FlatWorld(void)
{
    m3WorldDef worldDef = m3DefaultWorldDef();
    worldDef.bodyCapacity = 8;
    worldDef.shapeCapacity = 8;
    m3World* world = NewWorld(&worldDef);
    m3BodyDef groundDef = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &groundDef);
    m3ShapeDef shapeDef = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &shapeDef, &floor);
    return world;
}

// character.md: a walker. It lands on the floor and walks two seconds at
// 2 m/s along x, standing on the floor at the end.
static void TestCharacterWalk(void)
{
    m3World* world = FlatWorld();

    m3CharacterDef def = m3DefaultCharacterDef();
    def.position = (m3Pos3){0.0, 1.0, 0.0}; // the capsule's centre
    m3CharacterId hero;
    m3Result result = m3CreateCharacter(world, &def, &hero);

    float fallSpeed = 0.0f; // the host's own vertical speed, m/s
    for (int32_t tick = 0; tick < 120; ++tick)
    {
        float dt = 1.0f / 60.0f;
        fallSpeed -= 9.81f * dt; // gravity is the host's job
        m3Vec3 move = {2.0f * dt, fallSpeed * dt, 0.0f};
        m3Character_Move(world, hero, move);
        if (m3Character_IsGrounded(world, hero))
        {
            fallSpeed = 0.0f; // standing: no fall carries over
        }
        (void)m3World_Step(world, dt, 4);
    }

    CHECK(result == m3_success, "the character is created");
    m3Pos3 p = m3Character_GetPosition(world, hero);
    CHECK(p.x > 3.9 && p.x < 4.1, "it walks four meters");
    CHECK(p.y > 0.89 && p.y < 0.95, "its feet rest one skin above the floor");
    CHECK(m3Character_IsGrounded(world, hero), "it stands on the floor");
    m3DestroyWorld(world);
}

// soft-bodies.md: a rope hung by one end. Laid out along x, it swings
// down under its pin like a pendulum and keeps its length.
static void TestRope(void)
{
    m3World* world = FlatWorld();

    m3SoftBodyDef def = m3DefaultSoftBodyDef();
    def.position = (m3Pos3){0.0, 5.0, 0.0}; // the lattice's lowest corner
    def.countX = 10;                        // ten particles along x
    def.countY = 1;
    def.countZ = 1;
    m3SoftBodyId rope;
    m3Result result = m3CreateSoftBody(world, &def, &rope);
    if (result == m3_success)
    {
        result = m3SoftBody_PinParticle(world, rope, 0); // hang it by one end
    }

    CHECK(result == m3_success, "the rope is created and pinned");
    CHECK(m3SoftBody_GetParticleCount(world, rope) == 10, "it has ten particles");
    double lowest = 5.0;
    double longest = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 end = m3SoftBody_GetParticlePosition(world, rope, 9);
        double length = sqrt(end.x * end.x + (end.y - 5.0) * (end.y - 5.0));
        lowest = end.y < lowest ? end.y : lowest;
        longest = length > longest ? length : longest;
    }
    m3Pos3 pin = m3SoftBody_GetParticlePosition(world, rope, 0);
    CHECK(pin.x == 0.0 && pin.y == 5.0, "the pinned end holds still");
    CHECK(lowest < 3.0, "the free end swings down under the pin");
    CHECK(longest < 2.4, "the rope keeps its length of 2.25 m");
    m3DestroyWorld(world);
}

// vehicles.md: a four-wheeled car on full throttle. Its wheels find the
// floor and the rear pair drives it forward along +x.
static void TestCar(void)
{
    m3World* world = FlatWorld();
    m3BodyDef chassisDef = m3DefaultBodyDef();
    chassisDef.type = m3_dynamicBody;
    chassisDef.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId chassis = NewBody(world, &chassisDef);
    m3ShapeDef chassisShape = m3DefaultShapeDef();
    chassisShape.density = 300.0f; // about a tonne
    NewBoxShape(world, chassis, &chassisShape, (m3Vec3){1.5f, 0.3f, 0.9f});

    m3VehicleDef def = m3DefaultVehicleDef();
    def.chassisId = chassis; // a dynamic body, forward along its local +x
    def.wheelCount = 4;
    const float wheelX[4] = {1.2f, 1.2f, -1.2f, -1.2f};
    const float wheelZ[4] = {-0.8f, 0.8f, -0.8f, 0.8f};
    for (int32_t w = 0; w < 4; ++w)
    {
        def.wheels[w].anchor = (m3Vec3){wheelX[w], -0.2f, wheelZ[w]};
        def.wheels[w].steerable = w < 2; // the front pair steers
        def.wheels[w].driven = w >= 2;   // the rear pair drives
    }
    m3VehicleId car;
    m3Result result = m3CreateVehicle(world, &def, &car);
    m3Vehicle_SetCommands(world, car, 1.0f, 0.0f, 0.0f); // full throttle, straight

    CHECK(result == m3_success, "the car is created");
    for (int32_t i = 0; i < 180; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(world, chassis);
    CHECK(p.x > 2.0 && p.x < 10.0, "the rear wheels drive it forward");
    CHECK(p.z > -0.05 && p.z < 0.05, "it runs straight");
    for (int32_t w = 0; w < 4; ++w)
    {
        CHECK(m3Vehicle_IsWheelGrounded(world, car, w), "every wheel is on the floor");
    }
    m3DestroyWorld(world);
}

// destruction.md: a tower cut through. The 64 cells above the cut come
// loose as one fragment of 0.125 m3 cells, removed from the grid.
static void TestChunkCut(void)
{
    m3World* world = FlatWorld();
    m3BodyDef groundDef = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &groundDef);

    uint8_t voxels[4096] = {0}; // index x + 16 * (y + 16 * z)
    for (int32_t z = 0; z < 4; ++z)
    {
        for (int32_t y = 0; y < 8; ++y)
        {
            for (int32_t x = 0; x < 4; ++x)
            {
                voxels[x + 16 * (y + 16 * z)] = 1; // a 4 by 8 by 4 tower
            }
        }
    }
    m3ShapeDef chunkDef = m3DefaultShapeDef();
    m3ShapeId chunk;
    m3Result result = m3CreateVoxelChunkShape(world, ground, &chunkDef, voxels, NULL, 0.5f, &chunk);

    // Cut the tower through at y = 2: everything above it comes loose.
    const int32_t lo[3] = {0, 2, 0};
    const int32_t hi[3] = {3, 2, 3};
    int32_t cleared = 0;
    if (result == m3_success)
    {
        result = m3VoxelChunkShape_ClearBox(world, chunk, lo, hi, &cleared);
    }
    m3FragmentEvents fragments = m3World_GetFragmentEvents(world);

    CHECK(result == m3_success && cleared == 16, "the cut clears one layer of 16 cells");
    CHECK(fragments.fragmentCount == 1, "the top comes loose as one fragment");
    CHECK(fragments.fragmentCount == 1 && fragments.fragmentEvents[0].voxelCount == 80 &&
              fragments.fragmentEvents[0].recipeCount == 80,
          "it holds the 80 cells above the cut");
    CHECK(fragments.fragmentCount == 1 && fragments.fragmentEvents[0].mass == 10.0f,
          "its mass is 80 cells of 0.125 m3 at density 1");
    m3DestroyWorld(world);
}

// destruction.md: a blast beside a ball. The ball is pushed away from
// the blast.
static void TestExplosion(void)
{
    m3World* world = FlatWorld();
    m3BodyDef ballDef = m3DefaultBodyDef();
    ballDef.type = m3_dynamicBody;
    ballDef.position = (m3Pos3){3.0, 3.0, 1.0};
    m3BodyId ball = NewBody(world, &ballDef);
    m3ShapeDef ballShape = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, ball, &ballShape, &sphere);

    m3ExplosionDef blast = m3DefaultExplosionDef();
    blast.position = (m3Pos3){1.0, 3.0, 1.0};
    blast.radius = 2.0f;          // full strength out to 2 m
    blast.falloff = 3.0f;         // fading to nothing at 5 m
    blast.impulsePerArea = 50.0f; // N s for each square metre facing the blast
    blast.voxelCarve = 1.0f;      // bites a 1 m sphere out of chunks
    m3World_Explode(world, &blast);

    m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
    CHECK(v.x > 1.0f, "the ball flies away from the blast along +x");
    CHECK(v.y > -0.01f && v.y < 0.01f && v.z > -0.01f && v.z < 0.01f, "straight away from it");
    m3DestroyWorld(world);
}

// snapshots-and-rollback.md: a round trip. A ball falls for a second
// after the snapshot; restored and stepped the same second again, the
// world lands on the same hash.
static void TestSnapshotRoundTrip(void)
{
    m3World* world = FlatWorld();
    m3BodyDef ballDef = m3DefaultBodyDef();
    ballDef.type = m3_dynamicBody;
    ballDef.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId ball = NewBody(world, &ballDef);
    m3ShapeDef ballShape = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, ball, &ballShape, &sphere);

    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* buffer = malloc((size_t)size);
    int64_t written = 0;
    m3Result result = m3World_Snapshot(world, buffer, size, &written);

    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t first = m3World_Hash(world);
    double fallen = m3Body_GetPosition(world, ball).y;

    if (result == m3_success)
    {
        result = m3World_Restore(world, buffer, written);
    }
    free(buffer);

    CHECK(result == m3_success && written == size, "the snapshot is taken and restored");
    CHECK(m3Body_GetPosition(world, ball).y == 3.0, "the restore puts the ball back");
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(fallen < 3.0 && m3World_Hash(world) == first, "the second run lands on the same bits");
    m3DestroyWorld(world);
}

// A world with a floor and a ball dropped from 3 m, built the same way
// every time; ballOut, when not null, receives the ball.
static m3World* BallWorld(m3BodyId* ballOut)
{
    m3World* world = FlatWorld();
    m3BodyDef ballDef = m3DefaultBodyDef();
    ballDef.type = m3_dynamicBody;
    ballDef.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId ball = NewBody(world, &ballDef);
    m3ShapeDef ballShape = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, ball, &ballShape, &sphere);
    if (ballOut != NULL)
    {
        *ballOut = ball;
    }
    return world;
}

// journal-and-replays.md: a second recorded, then replayed into a twin
// built the same way. The twin ends on the recorder's hash.
static void TestJournalReplay(void)
{
    m3World* world = BallWorld(NULL);
    m3World* twin = BallWorld(NULL);

    uint8_t tape[4096];
    m3Result result = m3World_StartJournal(world, tape, sizeof tape);
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int64_t tapeBytes = 0;
    if (result == m3_success)
    {
        result = m3World_StopJournal(world, &tapeBytes);
    }

    if (result == m3_success)
    {
        result = m3World_ReplayJournal(twin, tape, tapeBytes);
    }

    CHECK(result == m3_success && tapeBytes > 16, "the session is recorded and replayed");
    CHECK(m3World_Hash(twin) == m3World_Hash(world), "the twin lands on the recorder's bits");
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

// getting-started.md: a first world. The snippet returns early when the
// world is refused, so the test reports how far it got: the ball's
// height once it rests.
static void FirstWorld(double* heightOut)
{
    m3WorldDef worldDef = m3DefaultWorldDef();
    m3World* world = NULL;
    m3Result result = m3CreateWorld(&worldDef, &world);
    if (result != m3_success)
    {
        return;
    }

    m3BodyDef groundDef = m3DefaultBodyDef(); // a static body
    m3BodyId ground;
    result = m3CreateBody(world, &groundDef, &ground);
    m3ShapeDef shapeDef = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f}; // solid below y = 0
    m3ShapeId floorShape;
    if (result == m3_success)
    {
        result = m3CreatePlaneShape(world, ground, &shapeDef, &floor, &floorShape);
    }

    m3BodyDef ballDef = m3DefaultBodyDef();
    ballDef.type = m3_dynamicBody;
    ballDef.position = (m3Pos3){0.0, 4.0, 0.0};
    m3BodyId ball;
    if (result == m3_success)
    {
        result = m3CreateBody(world, &ballDef, &ball);
    }
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId ballShape;
    if (result == m3_success)
    {
        result = m3CreateSphereShape(world, ball, &shapeDef, &sphere, &ballShape);
    }

    for (int32_t i = 0; i < 120 && result == m3_success; ++i)
    {
        result = m3World_Step(world, 1.0f / 60.0f, 4); // 60 Hz, four substeps
    }
    m3Pos3 p = m3Body_GetPosition(world, ball); // resting on the floor: y = 0.5
    m3DestroyWorld(world);

    *heightOut = p.y;
}

static void TestFirstWorld(void)
{
    double height = -1.0;
    FirstWorld(&height);
    CHECK(height > 0.49 && height < 0.51, "the ball rests on the floor");
}

// determinism.md: two worlds built and stepped the same way agree to
// the bit; a nudge to one shows up in the hash and in the compare.
static void TestCompareWorlds(void)
{
    m3World* worldA = BallWorld(NULL);
    m3BodyId ballB;
    m3World* worldB = BallWorld(&ballB);
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(worldA, 1.0f / 60.0f, 4);
        (void)m3World_Step(worldB, 1.0f / 60.0f, 4);
    }

    uint64_t hashA = m3World_Hash(worldA);
    uint64_t hashB = m3World_Hash(worldB);
    m3BodyDiff rows[8];
    int32_t differing = 0;
    m3Result result = m3World_Compare(worldA, worldB, rows, 8, &differing);

    CHECK(result == m3_success && hashA == hashB && differing == 0, "twins agree to the bit");
    m3Body_ApplyLinearImpulse(worldB, ballB, (m3Vec3){0.001f, 0.0f, 0.0f});
    (void)m3World_Step(worldA, 1.0f / 60.0f, 4);
    (void)m3World_Step(worldB, 1.0f / 60.0f, 4);
    result = m3World_Compare(worldA, worldB, rows, 8, &differing);
    CHECK(m3World_Hash(worldA) != m3World_Hash(worldB), "a nudge moves the hash");
    CHECK(result == m3_success && differing == 1 && rows[0].bodyId.index1 == ballB.index1,
          "and the compare names the nudged ball");
    m3DestroyWorld(worldB);
    m3DestroyWorld(worldA);
}

// bodies-and-shapes.md: a crate with two angular locks. It falls onto
// the floor and comes to rest on it.
static void TestCrate(void)
{
    m3World* world = FlatWorld();

    m3BodyDef def = m3DefaultBodyDef();
    def.type = m3_dynamicBody;
    def.position = (m3Pos3){0.0, 2.0, 0.0};
    def.motionLocks.angularX = true; // stays upright: no tipping about x or z
    def.motionLocks.angularZ = true;
    m3BodyId crate;
    m3Result result = m3CreateBody(world, &def, &crate);
    m3ShapeDef shapeDef = m3DefaultShapeDef();
    shapeDef.density = 500.0f; // kg/m3
    m3ShapeId box;
    if (result == m3_success)
    {
        result = m3CreateBoxShape(world, crate, &shapeDef, (m3Vec3){0.5f, 0.5f, 0.5f}, &box);
    }

    CHECK(result == m3_success, "the crate and its box are created");
    float mass = m3Body_GetMass(world, crate);
    CHECK(mass > 499.9f && mass < 500.1f, "a 1 m cube at 500 kg/m3 weighs 500 kg");
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double y = m3Body_GetPosition(world, crate).y;
    CHECK(y > 0.49 && y < 0.51, "it rests on the floor");
    m3DestroyWorld(world);
}

// queries-and-events.md: a ray cast down onto the floor plane meets it
// at the origin.
static void TestRayDown(void)
{
    m3World* world = FlatWorld();

    m3Pos3 from = {0.0, 10.0, 0.0};
    m3Vec3 down = {0.0f, -20.0f, 0.0f}; // the ray runs to y = -10
    m3CastHit hit;
    bool found = m3World_CastRayClosest(world, from, down, m3DefaultQueryFilter(), &hit);
    m3Pos3 ground = from;
    if (found)
    {
        ground = hit.point; // where the ray met the floor
    }

    CHECK(found && ground.y > -1e-6 && ground.y < 1e-6, "the ray meets the floor at y = 0");
    CHECK(found && hit.fraction == 0.5f, "half way along its 20 m");
    m3DestroyWorld(world);
}

// joints.md: a door on a motorized hinge. The motor swings it open
// until the limit stops it at 1.5 rad.
static void TestDoorHinge(void)
{
    m3World* world = FlatWorld();
    m3BodyDef frameDef = m3DefaultBodyDef();
    m3BodyId frame = NewBody(world, &frameDef);
    m3BodyDef doorDef = m3DefaultBodyDef();
    doorDef.type = m3_dynamicBody;
    doorDef.position = (m3Pos3){0.5, 1.1, 0.0};
    m3BodyId door = NewBody(world, &doorDef);
    m3ShapeDef doorShape = m3DefaultShapeDef();
    doorShape.density = 100.0f;
    NewBoxShape(world, door, &doorShape, (m3Vec3){0.5f, 1.0f, 0.05f});

    m3RevoluteJointDef def = m3DefaultRevoluteJointDef();
    // The door frame is a static body at the origin, the door a dynamic
    // body 0.5 m along x; the hinge is given in each body's frame.
    def.bodyIdA = frame;
    def.bodyIdB = door;
    def.localAnchorA = (m3Vec3){0.0f, 1.1f, 0.0f};
    def.localAnchorB = (m3Vec3){-0.5f, 0.0f, 0.0f};
    def.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f}; // a vertical hinge
    def.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    def.enableMotor = true;
    def.motorSpeed = 1.0f;       // rad/s
    def.maxMotorEffort = 500.0f; // N m
    def.enableLimit = true;
    def.lowerAngle = 0.0f;
    def.upperAngle = 1.5f; // it opens to about 86 degrees
    m3JointId hinge;
    m3Result result = m3CreateRevoluteJoint(world, &def, &hinge);

    CHECK(result == m3_success, "the hinge is created");
    for (int32_t i = 0; i < 180; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    float angle = m3Joint_GetAngle(world, hinge);
    CHECK(angle > 1.45f && angle < 1.55f, "the door stops at its 1.5 rad limit");
    m3DestroyWorld(world);
}

// limits.md: the pair check. A world with room for one pair, stepped
// with four balls on the floor, drops pairs and says so.
static void TestPairsDropped(void)
{
    m3WorldDef worldDef = m3DefaultWorldDef();
    worldDef.bodyCapacity = 8;
    worldDef.shapeCapacity = 8;
    worldDef.pairCapacity = 1;
    m3World* world = NewWorld(&worldDef);
    m3BodyDef groundDef = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &groundDef);
    m3ShapeDef shapeDef = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &shapeDef, &floor);
    for (int32_t i = 0; i < 4; ++i)
    {
        m3BodyDef ballDef = m3DefaultBodyDef();
        ballDef.type = m3_dynamicBody;
        ballDef.position = (m3Pos3){(double)i * 2.0, 0.5, 0.0};
        m3BodyId ball = NewBody(world, &ballDef);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
        NewSphereShape(world, ball, &shapeDef, &sphere);
    }
    float dt = 1.0f / 60.0f;
    bool tooFewPairs = false;

    uint64_t dropped = m3World_GetCounters(world).pairsDropped;
    (void)m3World_Step(world, dt, 4);
    if (m3World_GetCounters(world).pairsDropped != dropped)
    {
        tooFewPairs = true; // give the world a larger m3WorldDef.pairCapacity
    }

    CHECK(tooFewPairs, "four balls on the floor need more than one pair");
    m3DestroyWorld(world);
}

int main(void)
{
    TestFirstWorld();
    TestCrate();
    TestRayDown();
    TestDoorHinge();
    TestPairsDropped();
    TestWaterPool();
    TestCharacterWalk();
    TestRope();
    TestCar();
    TestChunkCut();
    TestExplosion();
    TestSnapshotRoundTrip();
    TestJournalReplay();
    TestCompareWorlds();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
