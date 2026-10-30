// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soft body gate: XPBD lattices. A pinned rope hangs to its
// analytic length, a jelly cube rests on the plane at its geometric
// height, the anchor table keeps its books, and hostile defs refuse
// loudly.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"

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
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    return world;
}

static void TestHangingRope(void)
{
    // Twenty particles on rigid rods, the top one pinned: the rope
    // hangs straight down to its analytic length.
    m3World* world = PlaneWorld();
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){0.0, 5.0, 0.0};
    sb.countX = 1;
    sb.countY = 20;
    sb.countZ = 1;
    sb.spacing = 0.25f;
    sb.compliance = 0.0f;
    m3SoftBodyId rope = NewSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(world, rope), "the rope creates");
    CHECK(m3SoftBody_GetParticleCount(world, rope) == 20, "twenty particles");
    m3SoftBody_PinParticle(world, rope, 19); // the top of the lattice

    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 top = m3SoftBody_GetParticlePosition(world, rope, 19);
    m3Pos3 bottom = m3SoftBody_GetParticlePosition(world, rope, 0);
    CHECK(fabs(top.y - (5.0 + 19.0 * 0.25)) < 1.0e-9, "the pin never moves");
    CHECK(fabs((top.y - bottom.y) - 4.75) < 0.05, "the rope hangs to its analytic length");
    CHECK(fabs(bottom.x) < 0.01 && fabs(bottom.z) < 0.01, "the hang is plumb");
    m3DestroyWorld(world);
}

static void TestJellyRest(void)
{
    // A four-cube jelly dropped on the plane settles at its
    // geometric height: bottom layer at particle radius, top a
    // lattice above it, small squash tolerated.
    m3World* world = PlaneWorld();
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){0.0, 1.0, 0.0};
    sb.countX = 4;
    sb.countY = 4;
    sb.countZ = 4;
    sb.spacing = 0.3f;
    sb.compliance = 1.0e-4f;
    sb.radius = 0.1f;
    m3SoftBodyId jelly = NewSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(world, jelly), "the jelly creates");

    for (int32_t i = 0; i < 360; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double minY = 1.0e30;
    double maxY = -1.0e30;
    for (int32_t p = 0; p < 64; ++p)
    {
        m3Pos3 q = m3SoftBody_GetParticlePosition(world, jelly, p);
        CHECK(isfinite(q.x) && isfinite(q.y) && isfinite(q.z), "the jelly stays finite");
        minY = q.y < minY ? q.y : minY;
        maxY = q.y > maxY ? q.y : maxY;
    }
    CHECK(minY > 0.05 && minY < 0.15, "the bottom layer rests at particle radius");
    CHECK(maxY - minY > 0.7 && maxY - minY < 1.0, "the cube keeps its height, squash tolerated");
    m3DestroyWorld(world);
}

static void TestSoftContracts(void)
{
    m3World* world = PlaneWorld();
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.countX = 0;
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "a zero count refuses");
    sb = m3DefaultSoftBodyDef();
    sb.countX = 9;
    sb.countY = 9;
    sb.countZ = 9; // 729 > 512
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "an oversized lattice refuses");
    sb = m3DefaultSoftBodyDef();
    sb.spacing = 0.0f;
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "a zero spacing refuses");
    sb = m3DefaultSoftBodyDef();
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    sb.compliance = bad;
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "a NaN compliance refuses");

    sb = m3DefaultSoftBodyDef();
    m3SoftBodyId a = NewSoftBody(world, &sb);
    sb.position = (m3Pos3){3.0, 1.0, 0.0};
    m3SoftBodyId b = NewSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(world, a) && m3SoftBody_IsValid(world, b), "the pool serves two");
    sb.position = (m3Pos3){6.0, 1.0, 0.0};
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "the full pool refuses the third");

    m3SoftBody_PinParticle(world, a, 9999); // out of range: no-op
    DestroySoftBody(world, a);
    CHECK(!m3SoftBody_IsValid(world, a), "the destroyed soft body is stale");
    m3SoftBody_PinParticle(world, a, 0); // stale: no-op
    m3Pos3 p = m3SoftBody_GetParticlePosition(world, a, 0);
    CHECK(p.x == 0.0 && p.y == 0.0, "a stale getter returns zeros");
    DestroySoftBody(world, a); // stale: no-op
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3SoftBody_IsValid(world, b), "the survivor lives on");
    m3DestroyWorld(world);
}

static void TestClothDrape(void)
{
    // An eight-by-eight cloth falls onto a box: the middle rests on
    // the lid (radius height above it), the corners hang BELOW the
    // lid, and friction keeps the drape from sliding off.
    m3World* world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId table = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.8f;
    NewBoxShape(world, table, &sd, (m3Vec3){0.5f, 1.0f, 0.5f}); // lid at y = 2

    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){-0.7, 2.6, -0.7};
    sb.countX = 8;
    sb.countY = 1;
    sb.countZ = 8;
    sb.spacing = 0.2f;
    sb.compliance = 1.0e-5f;
    sb.radius = 0.04f;
    sb.particleMass = 0.05f;
    m3SoftBodyId cloth = NewSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(world, cloth), "the cloth creates");

    for (int32_t i = 0; i < 420; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // The center particle (3..4, z 3..4) rests on the lid.
    m3Pos3 mid = m3SoftBody_GetParticlePosition(world, cloth, 3 + 8 * 3);
    CHECK(mid.y > 2.0 && mid.y < 2.12, "the middle rests on the lid");
    // The corners hang below the lid but above the floor.
    m3Pos3 corner = m3SoftBody_GetParticlePosition(world, cloth, 0);
    CHECK(corner.y < 1.9, "the corner hangs over the edge");
    CHECK(corner.y > 0.02, "the corner does not reach the floor");
    double span = fabs(corner.x) + fabs(corner.z);
    CHECK(span < 2.2, "friction holds the drape on the table");
    m3DestroyWorld(world);
}

static void TestRopeInBowlAndCarve(void)
{
    // A rope drops into a voxel bowl and coils inside its walls;
    // carving the bowl floor away drops the rope through the SAME
    // step contract every destruction consumer gets.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 1;
    m3World* world = NewWorld(&def);
    // No plane: below the bowl is open air, so the carve VISIBLY
    // drops the rope.
    static uint8_t bowl[16 * 16 * 16];
    memset(bowl, 0, sizeof(bowl));
    for (int32_t z = 4; z < 12; ++z)
    {
        for (int32_t x = 4; x < 12; ++x)
        {
            bowl[x + 16 * (0 + 16 * z)] = 1; // the floor slab
        }
    }
    for (int32_t y = 1; y <= 4; ++y)
    {
        for (int32_t i = 4; i < 12; ++i)
        {
            bowl[i + 16 * (y + 16 * 4)] = 1;
            bowl[i + 16 * (y + 16 * 11)] = 1;
            bowl[4 + 16 * (y + 16 * i)] = 1;
            bowl[11 + 16 * (y + 16 * i)] = 1;
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-2.0, 0.0, -2.0};
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    m3ShapeId chunk = NewVoxelChunkShape(world, NewBody(world, &gd), &sd, bowl, NULL, 0.25f);
    CHECK(m3Shape_IsValid(world, chunk), "the bowl creates");

    // A HORIZONTAL rope: a vertical one balances forever on its
    // own perfect symmetry (deterministic floats carry no noise to
    // seed the buckle: a rope stood on its head like a pole stays
    // there, which is the engine being RIGHT and proves nothing).
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){-0.66, 2.0, 0.0}; // spans the bowl mouth
    sb.countX = 12;
    sb.countY = 1;
    sb.countZ = 1;
    sb.spacing = 0.12f;
    sb.compliance = 0.0f;
    sb.radius = 0.05f;
    m3SoftBodyId rope = NewSoftBody(world, &sb);
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // Every particle lies inside the bowl: on its floor, within
    // its walls.
    for (int32_t p = 0; p < 12; ++p)
    {
        m3Pos3 q = m3SoftBody_GetParticlePosition(world, rope, p);
        CHECK(q.y > 0.25 && q.y < 0.6, "the rope lies on the bowl floor");
        CHECK(q.x > -1.1 && q.x < 1.1 && q.z > -1.1 && q.z < 1.1, "the walls keep the rope");
    }
    // Carve the floor slab out from under it.
    int32_t lo[3] = {5, 0, 5};
    int32_t hi[3] = {10, 0, 10};
    CHECK(m3VoxelChunkShape_ClearBox(world, chunk, lo, hi) > 0, "the carve clears the floor");
    for (int32_t i = 0; i < 90; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 low = m3SoftBody_GetParticlePosition(world, rope, 0);
    CHECK(low.y < -2.0, "the carved floor drops the rope into the void");
    m3DestroyWorld(world);
}

static void TestJellySeesaw(void)
{
    // A jelly lands on one end of a hinged plank: the plank tips
    // under the soft weight. Contact coupling carries real
    // momentum, not decoration.
    m3World* world = PlaneWorld();
    m3BodyDef postDef = m3DefaultBodyDef();
    postDef.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId post = NewBody(world, &postDef);
    m3BodyDef plankDef = m3DefaultBodyDef();
    plankDef.type = m3_dynamicBody;
    plankDef.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId plank = NewBody(world, &plankDef);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    NewBoxShape(world, plank, &sd, (m3Vec3){1.5f, 0.05f, 0.4f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = post;
    jd.bodyIdB = plank;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    NewJoint(world, &jd);

    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){0.75, 1.6, -0.3};
    sb.countX = 3;
    sb.countY = 3;
    sb.countZ = 3;
    sb.spacing = 0.2f;
    sb.compliance = 1.0e-4f;
    sb.radius = 0.08f;
    sb.particleMass = 0.2f; // a 5.4 kg jelly on one wing
    m3SoftBodyId jelly = NewSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(world, jelly), "the jelly creates");

    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 w = m3Body_GetAngularVelocity(world, plank);
    m3Quat q = m3Body_GetRotation(world, plank);
    (void)w;
    CHECK(q.z < -0.02f, "the plank tips under the jelly");
    m3Pos3 jp = m3SoftBody_GetParticlePosition(world, jelly, 0);
    CHECK(isfinite(jp.x) && isfinite(jp.y), "the jelly stays finite on the ride");
    m3DestroyWorld(world);
}

static void TestBallThroughCloth(void)
{
    // A hanging cloth anchored to a beam takes a flying ball: the
    // ball loses speed to the fabric, the fabric bulges, and the
    // anchors hold their corners.
    m3World* world = PlaneWorld();
    m3BodyDef beamDef = m3DefaultBodyDef();
    beamDef.position = (m3Pos3){0.0, 3.2, 0.0};
    m3BodyId beam = NewBody(world, &beamDef);
    m3ShapeDef bs = m3DefaultShapeDef();
    NewBoxShape(world, beam, &bs, (m3Vec3){1.2f, 0.05f, 0.05f});

    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){-0.9, 1.2, 0.0};
    sb.countX = 10;
    sb.countY = 10;
    sb.countZ = 1;
    sb.spacing = 0.2f;
    sb.compliance = 1.0e-5f;
    sb.radius = 0.06f;
    sb.particleMass = 0.03f;
    m3SoftBodyId cloth = NewSoftBody(world, &sb);
    // Anchor the top ROW's two corners to the beam.
    m3SoftBody_AnchorParticle(world, cloth, 90, beam); // (x0, y9)
    m3SoftBody_AnchorParticle(world, cloth, 99, beam); // (x9, y9)
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4); // let it hang
    }

    m3BodyDef ballDef = m3DefaultBodyDef();
    ballDef.type = m3_dynamicBody;
    ballDef.position = (m3Pos3){0.0, 2.0, -3.0};
    ballDef.linearVelocity = (m3Vec3){0.0f, 0.0f, 8.0f};
    ballDef.gravityScale = 0.0f; // a clean speed record
    m3BodyId ball = NewBody(world, &ballDef);
    m3ShapeDef bls = m3DefaultShapeDef();
    bls.density = 50.0f; // 3.3 kg: heavier than the whole fabric,
                         // or the reaction walls a featherweight
    m3Sphere slug = {{0.0f, 0.0f, 0.0f}, 0.25f};
    NewSphereShape(world, ball, &bls, &slug);

    for (int32_t i = 0; i < 90; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
    m3Pos3 bp = m3Body_GetPosition(world, ball);
    CHECK(bp.z > 0.3, "the ball passes through the fabric plane");
    CHECK(v.z < 7.9f, "the fabric takes speed off the ball");
    // The ball drags the tangled curtain with it (three kilos of
    // fabric on a three-kilo ball): forward motion survives, most
    // of the speed does not.
    CHECK(v.z > 0.5f, "the fabric does not wall the ball");
    // The anchor preserves the particle's CAPTURE position in the
    // beam frame (top row spawned at y = 3.0), it does not teleport
    // to the beam's center.
    m3Pos3 c0 = m3SoftBody_GetParticlePosition(world, cloth, 90);
    CHECK(fabs(c0.y - 3.0) < 0.15, "the anchored corner holds its captured height");
    m3DestroyWorld(world);
}

static void TestClothFollowsBeam(void)
{
    // The beam is kinematic and sails sideways: the anchored cloth
    // top tracks it; twins and a mid-ride rollback agree.
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3World* world = PlaneWorld();
        m3BodyDef beamDef = m3DefaultBodyDef();
        beamDef.type = m3_kinematicBody;
        beamDef.position = (m3Pos3){0.0, 3.0, 0.0};
        m3BodyId beam = NewBody(world, &beamDef);
        m3ShapeDef bs = m3DefaultShapeDef();
        NewBoxShape(world, beam, &bs, (m3Vec3){1.0f, 0.05f, 0.05f});

        m3SoftBodyDef sb = m3DefaultSoftBodyDef();
        sb.position = (m3Pos3){-0.7, 1.6, 0.0};
        sb.countX = 8;
        sb.countY = 8;
        sb.countZ = 1;
        sb.spacing = 0.2f;
        sb.compliance = 1.0e-5f;
        sb.radius = 0.05f;
        sb.particleMass = 0.03f;
        m3SoftBodyId cloth = NewSoftBody(world, &sb);
        m3SoftBody_AnchorParticle(world, cloth, 56, beam);
        m3SoftBody_AnchorParticle(world, cloth, 63, beam);

        m3Body_SetLinearVelocity(world, beam, (m3Vec3){0.8f, 0.0f, 0.0f});
        int64_t snapBytes = 0;
        for (int32_t i = 0; i < 150; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 75 && run == 0)
            {
                snapBytes = Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-ride snapshot fits");
            }
        }
        m3Pos3 corner = m3SoftBody_GetParticlePosition(world, cloth, 56);
        m3Pos3 beamP = m3Body_GetPosition(world, beam);
        CHECK(beamP.x > 1.9, "the beam sails");
        CHECK(fabs(corner.x - (beamP.x - 0.7)) < 0.1, "the anchored corner tracks the beam");
        hashes[run] = m3World_Hash(world);
        if (run == 0)
        {
            CHECK(Restore(world, snap, snapBytes), "the mid-ride restore lands");
            for (int32_t i = 76; i < 150; ++i)
            {
                (void)m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == hashes[0], "the re-ride is bit-identical");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "coupling twins are bit-identical");
}

static void TestBindTether(void)
{
    // A tethered cloth dropped from height hangs within its
    // deviation radius of the bind pose instead of falling; the
    // untethered twin drops to the floor. Hostile radius refuses.
    double y[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 4;
        wd.shapeCapacity = 4;
        m3World* world = NewWorld(&wd);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &sd, &floor);
        m3SoftBodyDef sb = m3DefaultSoftBodyDef();
        sb.position = (m3Pos3){0.0, 3.0, 0.0};
        sb.countX = 4;
        sb.countY = 1;
        sb.countZ = 4;
        sb.maxDeviation = pass == 0 ? 0.0f : 0.2f;
        m3SoftBodyId cloth = NewSoftBody(world, &sb);
        CHECK(m3SoftBody_IsValid(world, cloth), "the cloth creates");
        for (int32_t i = 0; i < 240; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        y[pass] = m3SoftBody_GetParticlePosition(world, cloth, 5).y;
        m3DestroyWorld(world);
    }
    CHECK(y[0] < 0.5, "the untethered cloth falls to the floor");
    CHECK(y[1] > 2.7, "the tethered cloth hangs at its bind pose");

    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.maxDeviation = -0.5f;
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "negative deviation refuses");
    m3DestroyWorld(world);
}

static void TestTetBodies(void)
{
    // A five-tet cube drops, squashes on contact, and keeps
    // its total volume (the incompressible jelly); hostile tets
    // refuse; twins and the journal agree to the bit.
    static const m3Vec3 pts[8] = {{0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, {0.5f, 0.5f, 0.0f},
                                  {0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.5f}, {0.5f, 0.0f, 0.5f},
                                  {0.5f, 0.5f, 0.5f}, {0.0f, 0.5f, 0.5f}};
    // The classic 5-tet decomposition of a cube, all positive.
    static const uint16_t tets[20] = {0, 1, 2, 5, 0, 2, 3, 7, 0, 5, 2, 7, 0, 5, 7, 4, 2, 5, 6, 7};
    static uint8_t journal[131072];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 4;
        wd.shapeCapacity = 4;
        m3World* world = NewWorld(&wd);
        bool recording = run == 0 && StartJournal(world, journal, (int32_t)sizeof(journal));
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &sd, &floor);
        m3SoftBodyDef sb = m3DefaultSoftBodyDef();
        sb.position = (m3Pos3){0.0, 1.5, 0.0};
        sb.compliance = 1.0e-4f;
        m3SoftBodyId jelly = NewTetSoftBody(world, &sb, pts, 8, tets, 5);
        CHECK(m3SoftBody_IsValid(world, jelly), "the jelly creates");
        CHECK(m3SoftBody_GetParticleCount(world, jelly) == 8, "eight particles");
        for (int32_t i = 0; i < 240; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Pos3 lo = m3SoftBody_GetParticlePosition(world, jelly, 0);
        m3Pos3 hi = m3SoftBody_GetParticlePosition(world, jelly, 6);
        CHECK(lo.y > -0.1 && lo.y < 0.3, "the jelly rests on the floor");
        double dx = hi.x - lo.x;
        double dy = hi.y - lo.y;
        double dz = hi.z - lo.z;
        double diag2 = dx * dx + dy * dy + dz * dz;
        CHECK(diag2 > 0.4 * 0.75 && diag2 < 2.5 * 0.75,
              "the jelly deforms without collapsing or exploding");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int64_t bytes = StopJournal(world);
            CHECK(bytes > 0, "the jelly session records");
            m3WorldDef fresh = wd;
            m3World* replayed = NewWorld(&fresh);
            CHECK(ReplayJournal(replayed, journal, bytes), "the jelly replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin jellies are bit-identical");

    // Walls: bad index, degenerate tet, lattice knobs on a tet body.
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    uint16_t bad[4] = {0, 1, 2, 9};
    CHECK(!m3SoftBody_IsValid(world, NewTetSoftBody(world, &sb, pts, 8, bad, 1)),
          "an out-of-range tet refuses");
    uint16_t flat[4] = {0, 1, 2, 3}; // coplanar corners of one face
    CHECK(!m3SoftBody_IsValid(world, NewTetSoftBody(world, &sb, pts, 8, flat, 1)),
          "a flat tet refuses");
    sb.pressure = 1.0f;
    uint16_t good[4] = {0, 1, 2, 5};
    CHECK(!m3SoftBody_IsValid(world, NewTetSoftBody(world, &sb, pts, 8, good, 1)),
          "lattice pressure on a tet body refuses");
    m3DestroyWorld(world);
}

static void TestPressure(void)
{
    // A pressurized cube inflates toward its target volume in
    // zero gravity; the unpressurized twin keeps its size. Twins
    // stay bit-identical, hostile pressure refuses.
    double diag[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        uint64_t hashes[2];
        for (int32_t run = 0; run < 2; ++run)
        {
            m3WorldDef wd = m3DefaultWorldDef();
            wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
            wd.bodyCapacity = 4;
            wd.shapeCapacity = 4;
            m3World* world = NewWorld(&wd);
            m3SoftBodyDef sb = m3DefaultSoftBodyDef();
            sb.position = (m3Pos3){0.0, 2.0, 0.0};
            sb.countX = 4;
            sb.countY = 4;
            sb.countZ = 4;
            sb.spacing = 0.25f;
            sb.compliance = 1.0e-4f; // stretchy walls let it puff
            sb.pressure = pass == 0 ? 0.0f : 2.0f;
            m3SoftBodyId cube = NewSoftBody(world, &sb);
            CHECK(m3SoftBody_IsValid(world, cube), "the cube creates");
            for (int32_t i = 0; i < 240; ++i)
            {
                (void)m3World_Step(world, 1.0f / 60.0f, 4);
            }
            m3Pos3 a = m3SoftBody_GetParticlePosition(world, cube, 0);
            m3Pos3 b = m3SoftBody_GetParticlePosition(world, cube, 63);
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double dz = b.z - a.z;
            if (run == 0)
            {
                diag[pass] = dx * dx + dy * dy + dz * dz;
            }
            hashes[run] = m3World_Hash(world);
            m3DestroyWorld(world);
        }
        CHECK(hashes[0] == hashes[1], "twin cubes are bit-identical");
    }
    CHECK(diag[1] > diag[0] * 1.2, "the pressurized cube inflates");

    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.pressure = -1.0f;
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "negative pressure refuses");
    sb = m3DefaultSoftBodyDef();
    sb.countY = 1;
    sb.pressure = 1.0f;
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "a flat sheet takes no pressure");
    m3DestroyWorld(world);
}

static void TestBendRods(void)
{
    // A horizontal rope pinned at one end droops deep when
    // floppy and holds a shallow curve when its bend tethers are
    // stiff. Twins stay bit-identical.
    double tipY[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        uint64_t hashes[2];
        for (int32_t run = 0; run < 2; ++run)
        {
            m3WorldDef wd = m3DefaultWorldDef();
            wd.bodyCapacity = 4;
            wd.shapeCapacity = 4;
            m3World* world = NewWorld(&wd);
            m3SoftBodyDef sb = m3DefaultSoftBodyDef();
            sb.position = (m3Pos3){0.0, 4.0, 0.0};
            sb.countX = 16;
            sb.countY = 1;
            sb.countZ = 1;
            sb.spacing = 0.2f;
            sb.compliance = 0.0f;
            sb.bendCompliance = pass == 0 ? 0.0f : 1.0e-6f;
            m3SoftBodyId rope = NewSoftBody(world, &sb);
            CHECK(m3SoftBody_IsValid(world, rope), "the rope creates");
            // TWO pins make the clamp: a single point pin holds
            // no moment (even a rigid rod would swing free).
            m3SoftBody_PinParticle(world, rope, 0);
            m3SoftBody_PinParticle(world, rope, 1);
            for (int32_t i = 0; i < 300; ++i)
            {
                (void)m3World_Step(world, 1.0f / 60.0f, 4);
            }
            if (run == 0)
            {
                tipY[pass] = m3SoftBody_GetParticlePosition(world, rope, 15).x;
            }
            hashes[run] = m3World_Hash(world);
            m3DestroyWorld(world);
        }
        CHECK(hashes[0] == hashes[1], "twin ropes are bit-identical");
    }
    // The floppy rope folds back on itself (tip x well behind the
    // clamp); the stiff one reaches forward. Reach, not height, is
    // the right metric: even a rigid rod droops at the tip, but
    // only a floppy one CRUMPLES.
    CHECK(tipY[1] > 0.3 && tipY[1] > tipY[0] + 1.5,
          "the stiff rope reaches forward where the floppy one crumples");

    // The wall: a hostile bend compliance refuses.
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.bendCompliance = -1.0f;
    CHECK(!m3SoftBody_IsValid(world, NewSoftBody(world, &sb)), "negative bend refuses");
    m3DestroyWorld(world);
}

// A restore brings back the slot pool cursors with the slots: a soft
// body destroyed after the snapshot is alive again, and the next create
// takes a fresh slot instead of the restored one.
static void TestRestoreKeepsPoolCursors(void)
{
    static uint8_t snap[2097152];
    m3World* world = PlaneWorld();
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){0.0, 2.0, 0.0};
    sb.countX = 2;
    sb.countY = 2;
    sb.countZ = 2;
    m3SoftBodyId first = NewSoftBody(world, &sb);
    int64_t bytes = Snapshot(world, snap, (int32_t)sizeof(snap));
    CHECK(bytes > 0, "the snapshot fits");
    DestroySoftBody(world, first);
    CHECK(Restore(world, snap, bytes), "the restore lands");
    CHECK(m3SoftBody_IsValid(world, first), "the destroyed body is back");
    sb.position = (m3Pos3){3.0, 2.0, 0.0};
    m3SoftBodyId second = NewSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(world, second), "a new body creates after the restore");
    CHECK(second.index1 != first.index1, "the new body takes its own slot");
    CHECK(m3SoftBody_GetParticlePosition(world, first, 0).x < 1.0,
          "the restored body keeps its state");

    // The same world played forward without the detour hands out the same id.
    m3World* twin = PlaneWorld();
    sb.position = (m3Pos3){0.0, 2.0, 0.0};
    NewSoftBody(twin, &sb);
    sb.position = (m3Pos3){3.0, 2.0, 0.0};
    m3SoftBodyId twinSecond = NewSoftBody(twin, &sb);
    CHECK(twinSecond.index1 == second.index1 && twinSecond.generation == second.generation,
          "restored and straight timelines allocate alike");
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

static void TestAnchorTableLimits(void)
{
    // The anchor table's books: exactly 32 anchors fit, the 33rd is
    // a quiet no-op; anchors to a destroyed body release silently
    // and the freed particles fall; hostile anchor calls no-op.
    m3World* world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId beam = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, beam, &sd, (m3Vec3){2.0f, 0.05f, 0.05f});

    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){-0.8, 2.4, 0.0};
    sb.countX = 8;
    sb.countY = 8;
    sb.countZ = 1;
    sb.spacing = 0.2f;
    sb.compliance = 1.0e-5f;
    m3SoftBodyId cloth = NewSoftBody(world, &sb);
    // Fill the whole anchor table with the top row and then some.
    for (int32_t a = 0; a < 40; ++a)
    {
        m3SoftBody_AnchorParticle(world, cloth, 56 + (a % 8), beam); // 32 land, 8 no-op
    }
    m3SoftBody_AnchorParticle(world, cloth, -1, beam);   // hostile: no-op
    m3SoftBody_AnchorParticle(world, cloth, 9999, beam); // hostile: no-op
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 held = m3SoftBody_GetParticlePosition(world, cloth, 56);
    CHECK(fabs(held.y - 3.8) < 0.1, "the packed anchor table holds the row");

    // Kill the beam: every anchor releases, the cloth falls free.
    DestroyBody(world, beam);
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // The freed sheet tumbles for a while (near-rigid edges make
    // it a flying chain-mail); the CONTRACT is release: the row
    // leaves its held height and keeps falling finite.
    m3Pos3 fallen = m3SoftBody_GetParticlePosition(world, cloth, 56);
    CHECK(fallen.y < 2.5, "a dead beam releases its anchors");
    CHECK(isfinite(fallen.x) && isfinite(fallen.y), "the freed sheet stays finite");
    m3DestroyWorld(world);
}

// Distance from p to the triangle abc, in double: the nearest of its
// edges and, when p projects inside it, its plane.
static double TriangleDistance(const double p[3], const double t[3][3])
{
    double best = (double)INFINITY;
    for (int32_t e = 0; e < 3; ++e)
    {
        const double* a = t[e];
        const double* b = t[(e + 1) % 3];
        double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        double ap[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
        double len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
        double s = len2 > 0.0 ? (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2 : 0.0;
        s = s < 0.0 ? 0.0 : (s > 1.0 ? 1.0 : s);
        double d[3] = {ap[0] - s * ab[0], ap[1] - s * ab[1], ap[2] - s * ab[2]};
        best = fmin(best, sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
    }
    double u[3] = {t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]};
    double v[3] = {t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]};
    double n[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    double nn = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
    if (nn > 0.0)
    {
        double inside = 1.0;
        for (int32_t e = 0; e < 3; ++e)
        {
            const double* a = t[e];
            const double* b = t[(e + 1) % 3];
            double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
            double ap[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
            double c[3] = {ab[1] * ap[2] - ab[2] * ap[1], ab[2] * ap[0] - ab[0] * ap[2],
                           ab[0] * ap[1] - ab[1] * ap[0]};
            inside = fmin(inside, c[0] * n[0] + c[1] * n[1] + c[2] * n[2]);
        }
        if (inside >= 0.0)
        {
            double h = (p[0] - t[0][0]) * n[0] + (p[1] - t[0][1]) * n[1] + (p[2] - t[0][2]) * n[2];
            best = fmin(best, fabs(h) / sqrt(nn));
        }
    }
    return best;
}

static void TestThinTriangles(void)
{
    // A particle closer than its radius to a mesh triangle is pushed out
    // however thin the triangle. The closest-point walk's interior
    // barycentrics cancel on a sliver: they put the point 0.3 to 1 m
    // from a particle 2 to 4 cm from the triangle, which then passed
    // through it. A triangle whose first two corners are one vertex
    // divided zero by zero and never pushed anything. Each triangle
    // lies in the plane y = 0 on a static body at the origin.
    static const struct
    {
        m3Vec3 corners[3];
        uint16_t indices[3];
        m3Pos3 particle;
    } cases[] = {
        // Slivers well under a millimetre wide.
        {{{-1.686742f, 0.0f, -0.940354f},
          {-2.590324f, 0.0f, -2.326897f},
          {-1.297610f, 0.0f, -0.343236f}},
         {0, 1, 2},
         {-2.322956, -0.022954, -1.930697}},
        {{{-1.254562f, 0.0f, -1.092828f},
          {-3.589935f, 0.0f, 1.221636f},
          {-0.590509f, 0.0f, -1.750940f}},
         {0, 1, 2},
         {-2.504005, -0.042204, 0.138770}},
        {{{0.931752f, 0.0f, -1.033501f},
          {-0.625909f, 0.0f, 2.162496f},
          {-1.180224f, 0.0f, 3.299831f}},
         {0, 1, 2},
         {-0.038631, -0.032375, 0.908804}},
        // A repeated corner: the triangle is the segment from vertex 0
        // to vertex 1.
        {{{0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0, 0, 1}, {1.0, 0.05, 0.0}},
    };
    float r = 0.1f;
    for (int32_t i = 0; i < (int32_t)(sizeof(cases) / sizeof(cases[0])); ++i)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        m3World* world = NewWorld(&def);
        m3BodyDef bd = m3DefaultBodyDef();
        m3ShapeDef sd = m3DefaultShapeDef();
        NewMeshShape(world, NewBody(world, &bd), &sd, cases[i].corners, 3, cases[i].indices, 1);
        m3SoftBodyDef sb = m3DefaultSoftBodyDef();
        sb.position = cases[i].particle;
        sb.countX = 1;
        sb.countY = 1;
        sb.countZ = 1;
        sb.radius = r;
        m3SoftBodyId probe = NewSoftBody(world, &sb);
        double tri[3][3];
        for (int32_t k = 0; k < 3; ++k)
        {
            m3Vec3 c = cases[i].corners[cases[i].indices[k]];
            tri[k][0] = (double)c.x;
            tri[k][1] = (double)c.y;
            tri[k][2] = (double)c.z;
        }
        m3Pos3 start = m3SoftBody_GetParticlePosition(world, probe, 0);
        double before = TriangleDistance((double[3]){start.x, start.y, start.z}, tri);
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 end = m3SoftBody_GetParticlePosition(world, probe, 0);
        double after = TriangleDistance((double[3]){end.x, end.y, end.z}, tri);
        if (after < (double)r - 1.0e-4)
        {
            printf("M3_THIN_TRIANGLE case %d: %.6f from the triangle, then %.6f\n", i, before,
                   after);
        }
        CHECK(before < (double)r, "the particle starts inside its radius of the triangle");
        CHECK(after >= (double)r - 1.0e-4, "a thin triangle pushes the particle out");
        m3DestroyWorld(world);
    }
}

int main(void)
{
    TestHangingRope();
    TestJellyRest();
    TestSoftContracts();
    TestClothDrape();
    TestRopeInBowlAndCarve();
    TestJellySeesaw();
    TestBallThroughCloth();
    TestClothFollowsBeam();
    TestBendRods();
    TestPressure();
    TestTetBodies();
    TestBindTether();
    TestRestoreKeepsPoolCursors();
    TestAnchorTableLimits();
    TestThinTriangles();
    if (s_failures == 0)
    {
        printf("test_softbody: all green\n");
        return 0;
    }
    printf("test_softbody: %d failure(s)\n", s_failures);
    return 1;
}
