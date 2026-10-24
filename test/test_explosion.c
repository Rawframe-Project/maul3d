// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The explosion gate: one journaled call pushes every dynamic
// convex shape in range. A symmetric ring flies outward on mirrored
// speeds, the falloff band grades the push, sleepers in range wake
// while sleepers beyond it nap on, a negative impulse implodes, the
// filter and static bodies stand aside, an off-center hit spins its
// target, the tree's history changes nothing, and the whole storm
// rides twins, replay, rollback, and a hostile tape without losing a
// bit.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static m3World* BlastWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    return NewWorld(&def);
}

static m3BodyId Crate(m3World* world, double x, double y, double z)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, y, z};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return body;
}

static float Speed(const m3World* world, m3BodyId body)
{
    m3Vec3 v = m3Body_GetLinearVelocity(world, body);
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static void TestRingAndFalloff(void)
{
    m3World* world = BlastWorld();
    // Four crates on the axes at 4 m, one in the falloff band at
    // 12 m, one beyond every reach at 30 m.
    m3BodyId east = Crate(world, 4.0, 0.0, 0.0);
    m3BodyId west = Crate(world, -4.0, 0.0, 0.0);
    m3BodyId north = Crate(world, 0.0, 0.0, 4.0);
    m3BodyId south = Crate(world, 0.0, 0.0, -4.0);
    m3BodyId graze = Crate(world, 12.0, 0.0, 0.0);
    m3BodyId far = Crate(world, 30.0, 0.0, 0.0);
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.radius = 10.0f;
    def.falloff = 5.0f;
    def.impulsePerArea = 4.0f;
    m3World_Explode(world, &def);
    CHECK(Speed(world, east) > 1.0f, "the blast moves the ring");
    CHECK(fabsf(Speed(world, east) - Speed(world, west)) < 1e-6f,
          "mirrored crates get mirrored speeds");
    CHECK(fabsf(Speed(world, north) - Speed(world, south)) < 1e-6f,
          "the mirror holds on both axes");
    CHECK(m3Body_GetLinearVelocity(world, east).x > 0.0f, "east flies east");
    CHECK(m3Body_GetLinearVelocity(world, west).x < 0.0f, "west flies west");
    CHECK(Speed(world, graze) > 0.0f, "the falloff band still pushes");
    CHECK(Speed(world, graze) < Speed(world, east), "the falloff band pushes less");
    CHECK(Speed(world, far) == 0.0f, "beyond radius plus falloff nothing moves");
    // An implosion pulls the same ring inward.
    def.impulsePerArea = -4.0f;
    m3World_Explode(world, &def);
    CHECK(fabsf(Speed(world, east)) < 1e-5f, "the implosion refunds the blast exactly");
    m3DestroyWorld(world);
}

static void TestWakeFilterAndStatics(void)
{
    m3World* world = BlastWorld();
    m3BodyId sleeper = Crate(world, 3.0, 0.0, 0.0);
    m3BodyId dreamer = Crate(world, 40.0, 0.0, 0.0);
    m3Body_SetAwake(world, sleeper, false);
    m3Body_SetAwake(world, dreamer, false);
    // A crate the filter excludes: category 2, blast masks it out.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-3.0, 0.0, 0.0};
    m3BodyId ghost = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.categoryBits = 2ull;
    NewBoxShape(world, ghost, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    // A static wall inside the blast.
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){0.0, -3.0, 0.0};
    m3BodyId wall = NewBody(world, &wd);
    m3ShapeDef wsd = m3DefaultShapeDef();
    NewBoxShape(world, wall, &wsd, (m3Vec3){2.0f, 0.5f, 2.0f});
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.radius = 8.0f;
    def.falloff = 2.0f;
    def.impulsePerArea = 3.0f;
    def.filter.maskBits = ~2ull;
    m3World_Explode(world, &def);
    CHECK(m3Body_IsAwake(world, sleeper), "a sleeper in range wakes");
    CHECK(Speed(world, sleeper) > 0.0f, "the woken sleeper moves");
    CHECK(!m3Body_IsAwake(world, dreamer), "a sleeper beyond range naps on");
    CHECK(Speed(world, ghost) == 0.0f, "the filtered crate never feels the blast");
    CHECK(Speed(world, wall) == 0.0f, "the static wall stands");
    m3Vec3 wv = m3Body_GetLinearVelocity(world, wall);
    CHECK(wv.x == 0.0f && wv.y == 0.0f && wv.z == 0.0f, "statics take no velocity at all");
    m3DestroyWorld(world);
}

static void TestOffCenterSpin(void)
{
    // The blast center level with the crate's lower edge: the
    // closest point is off the center of mass, the lever arm turns
    // the push into spin.
    m3World* world = BlastWorld();
    m3BodyId crate = Crate(world, 3.0, 0.0, 0.0);
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, -0.45, 0.0};
    def.radius = 6.0f;
    def.falloff = 2.0f;
    def.impulsePerArea = 3.0f;
    m3World_Explode(world, &def);
    m3Vec3 w = m3Body_GetAngularVelocity(world, crate);
    CHECK(Speed(world, crate) > 0.0f, "the off-center blast still pushes");
    CHECK(sqrtf(w.x * w.x + w.y * w.y + w.z * w.z) > 0.01f, "the lever arm spins the crate");
    m3DestroyWorld(world);
}

static void TestVoxelCarveCoupling(void)
{
    // A 16-tall single-voxel column: the blast bites out its waist
    // (y 7..9 at carve radius 1.2), the anchored base survives, the
    // freed top emits exactly one fragment island of six cells.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3World* world = BlastWorld();
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        static uint8_t voxels[16 * 16 * 16];
        memset(voxels, 0, sizeof(voxels));
        for (int32_t y = 0; y < 16; ++y)
        {
            voxels[8 + 16 * (y + 16 * 8)] = 1;
        }
        m3ShapeId column = NewVoxelChunkShape(world, ground, &sd, voxels, NULL, 1.0f);
        CHECK(m3Shape_IsValid(world, column), "the column creates");
        m3ExplosionDef def = m3DefaultExplosionDef();
        def.position = (m3Pos3){8.5, 8.5, 8.5};
        def.radius = 2.0f;
        def.falloff = 1.0f;
        def.impulsePerArea = 0.0f; // a pure carve: the bite is the test
        def.voxelCarve = 1.2f;
        m3World_Explode(world, &def);
        const m3FragmentEvent* events = m3World_GetFragmentEvents(world).fragmentEvents;
        int32_t eventCount = m3World_GetFragmentEvents(world).fragmentCount;
        CHECK(eventCount == 1, "the freed top is exactly one island");
        if (eventCount == 1)
        {
            CHECK(events[0].voxelCount == 6, "the island is the six cells above the bite");
            CHECK(events[0].boundsLo[1] == 10 && events[0].boundsHi[1] == 15,
                  "the island spans y 10 through 15");
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin carves are bit-identical");

    // The same column untouched hashes differently: the carve is
    // real state, not an event-only side effect.
    m3World* control = BlastWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(control, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t y = 0; y < 16; ++y)
    {
        voxels[8 + 16 * (y + 16 * 8)] = 1;
    }
    NewVoxelChunkShape(control, ground, &sd, voxels, NULL, 1.0f);
    CHECK(m3World_Hash(control) != hashes[0], "the carve changed the chunk's bits");
    m3DestroyWorld(control);
}

// A blast scene: four voxel columns around the center, a compound body
// of six spheres and four crates. The rebuilt twin holds
// the same state over a balanced tree instead of the insertion-order
// one, so the tree visits its leaves in another order.
static m3World* HistoryScene(bool rebuild, m3BodyId bodies[5])
{
    m3World* world = BlastWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t y = 0; y < 16; ++y)
    {
        voxels[8 + 16 * (y + 16 * 8)] = 1;
    }
    static const double kColumns[4][2] = {{-9.0, -8.0}, {-7.0, -8.0}, {-8.0, -9.0}, {-8.0, -7.0}};
    for (int32_t c = 0; c < 4; ++c)
    {
        m3BodyDef gd = m3DefaultBodyDef();
        gd.position = (m3Pos3){kColumns[c][0], 0.0, kColumns[c][1]};
        NewVoxelChunkShape(world, NewBody(world, &gd), &sd, voxels, NULL, 1.0f);
    }
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.3, 8.9, 2.1};
    bodies[0] = NewBody(world, &bd);
    for (int32_t k = 0; k < 6; ++k)
    {
        m3ShapeDef part = m3DefaultShapeDef();
        part.localPosition = (m3Vec3){0.37f * (float)(k % 3) - 0.4f, 0.29f * (float)(k / 3),
                                      0.11f * (float)k - 0.3f};
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.2f + 0.03f * (float)k};
        NewSphereShape(world, bodies[0], &part, &ball);
    }
    for (int32_t k = 0; k < 4; ++k)
    {
        bodies[1 + k] = Crate(world, -2.5 + 1.7 * k, 7.0 + 0.6 * k, -1.3 + 0.9 * k);
    }
    if (rebuild)
    {
        CHECK(m3World_RebuildBroadphase(world) == m3_success, "the twin rebuilds its tree");
    }
    return world;
}

static void TestTreeHistoryChangesNothing(void)
{
    // The blast visits the shapes in slot order, so impulses on one body
    // sum in the same order and the chunks emit fragments in the same
    // order whatever shape the tree has.
    m3BodyId bodies[2][5];
    m3World* plain = HistoryScene(false, bodies[0]);
    m3World* rebuilt = HistoryScene(true, bodies[1]);
    CHECK(m3World_Hash(plain) == m3World_Hash(rebuilt), "the twins hash equal before the blast");
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.5, 8.5, 0.5};
    def.radius = 6.0f;
    def.falloff = 2.0f;
    def.impulsePerArea = 3.0f;
    def.voxelCarve = 1.6f;
    m3World_Explode(plain, &def);
    m3World_Explode(rebuilt, &def);
    CHECK(m3World_Hash(plain) == m3World_Hash(rebuilt), "and after it");
    bool sameMotion = true;
    for (int32_t i = 0; i < 5; ++i)
    {
        m3Vec3 v[2];
        m3Vec3 w[2];
        for (int32_t t = 0; t < 2; ++t)
        {
            m3World* world = t == 0 ? plain : rebuilt;
            v[t] = m3Body_GetLinearVelocity(world, bodies[t][i]);
            w[t] = m3Body_GetAngularVelocity(world, bodies[t][i]);
        }
        sameMotion = sameMotion && memcmp(&v[0], &v[1], sizeof(m3Vec3)) == 0 &&
                     memcmp(&w[0], &w[1], sizeof(m3Vec3)) == 0;
    }
    CHECK(sameMotion, "every body leaves with bit-equal velocities");
    m3FragmentEvents a = m3World_GetFragmentEvents(plain);
    m3FragmentEvents b = m3World_GetFragmentEvents(rebuilt);
    CHECK(a.fragmentCount == 4, "every column loses its top");
    bool sameEvents = a.fragmentCount == b.fragmentCount;
    for (int32_t i = 0; sameEvents && i < a.fragmentCount; ++i)
    {
        const m3FragmentEvent* x = &a.fragmentEvents[i];
        const m3FragmentEvent* y = &b.fragmentEvents[i];
        sameEvents = x->chunkShapeId.index1 == y->chunkShapeId.index1 &&
                     x->voxelCount == y->voxelCount && x->recipeStart == y->recipeStart &&
                     memcmp(x->boundsLo, y->boundsLo, 3) == 0 &&
                     memcmp(x->boundsHi, y->boundsHi, 3) == 0;
    }
    CHECK(sameEvents, "the fragment events come in the same order");
    m3DestroyWorld(rebuilt);
    m3DestroyWorld(plain);
}

static void TestSoftPushAndKickWindow(void)
{
    // The blast pushes a lattice through a pending kick that the
    // next step integrates exactly once, and the kick window is
    // real rollback state: a snapshot taken between the blast and
    // the step must re-run onto identical bits.
    static uint8_t snap[2097152];
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    wd.bodyCapacity = 16;
    wd.shapeCapacity = 16;
    wd.softBodyCapacity = 4;
    m3World* world = NewWorld(&wd);
    m3SoftBodyDef sd = m3DefaultSoftBodyDef();
    sd.position = (m3Pos3){3.0, -0.5, -0.5};
    sd.countX = 2;
    sd.countY = 2;
    sd.countZ = 2;
    sd.spacing = 1.0f;
    m3SoftBodyId jelly = NewSoftBody(world, &sd);
    CHECK(m3SoftBody_IsValid(world, jelly), "the lattice creates");
    double beforeX = m3SoftBody_GetParticlePosition(world, jelly, 0).x;
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.radius = 8.0f;
    def.falloff = 2.0f;
    // Particle facing area is tiny (pi r squared), so the test
    // blast is strong to make the push visible over half a second.
    def.impulsePerArea = 50.0f;
    m3World_Explode(world, &def);
    int32_t snapBytes = Snapshot(world, snap, (int32_t)sizeof(snap));
    CHECK(snapBytes > 0, "the kick-window snapshot fits");
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t after = m3World_Hash(world);
    CHECK(m3SoftBody_GetParticlePosition(world, jelly, 0).x > beforeX + 0.1,
          "the pushed lattice flies away from the blast");
    CHECK(Restore(world, snap, snapBytes), "the kick-window restore lands");
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) == after, "the re-run kick lands on the same bits");
    m3DestroyWorld(world);

    // softPush zero stands the lattice down: nothing moves at all.
    m3World* still = NewWorld(&wd);
    m3SoftBodyId calm = NewSoftBody(still, &sd);
    def.softPush = 0.0f;
    m3World_Explode(still, &def);
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(still, 1.0f / 60.0f, 4);
    }
    double calmX = m3SoftBody_GetParticlePosition(still, calm, 0).x;
    CHECK(calmX > 2.9 && calmX < 3.1, "softPush zero leaves the lattice in place");
    m3DestroyWorld(still);
}

static void TestHostileDefWall(void)
{
    m3World* world = BlastWorld();
    m3BodyId crate = Crate(world, 3.0, 0.0, 0.0);
    uint64_t before = m3World_Hash(world);
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.impulsePerArea = 5.0f;
    def.radius = -1.0f;
    m3World_Explode(world, &def);
    def = m3DefaultExplosionDef();
    def.impulsePerArea = 5.0f;
    def.falloff = -0.5f;
    m3World_Explode(world, &def);
    def = m3DefaultExplosionDef();
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    def.impulsePerArea = bad;
    m3World_Explode(world, &def);
    def = m3DefaultExplosionDef();
    def.position.x = (double)bad;
    def.impulsePerArea = 5.0f;
    m3World_Explode(world, &def);
    def = m3DefaultExplosionDef();
    def.impulsePerArea = 5.0f;
    def.internalValue = 0; // a dead cookie bounces at the door
    m3World_Explode(world, &def);
    CHECK(m3World_Hash(world) == before, "five hostile blasts changed nothing");
    CHECK(Speed(world, crate) == 0.0f, "the crate never moved");
    m3DestroyWorld(world);
}

static void TestHostileTape(void)
{
    // One explode op on tape, then surgical damage: a NaN center and
    // a negative radius must fail the replay loudly.
    static uint8_t tape[256];
    m3World* world = BlastWorld();
    Crate(world, 3.0, 0.0, 0.0);
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the tape opens");
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.impulsePerArea = 2.0f;
    m3World_Explode(world, &def);
    int32_t bytes = StopJournal(world);
    CHECK(bytes == 8 + (int32_t)sizeof(m3ExplosionDef), "one blast op is header plus def");
    m3DestroyWorld(world);

    m3World* twin = BlastWorld();
    m3BodyId crate = Crate(twin, 3.0, 0.0, 0.0);
    uint8_t hostile[256];
    memcpy(hostile, tape, (size_t)bytes);
    double nanD;
    uint64_t nanBits = 0x7FF8000000000000ull;
    memcpy(&nanD, &nanBits, sizeof(nanD));
    memcpy(hostile + 8, &nanD, sizeof(nanD)); // position.x
    CHECK(!ReplayJournal(twin, hostile, bytes), "a NaN center bounces");
    memcpy(hostile, tape, (size_t)bytes);
    float negative = -2.0f;
    memcpy(hostile + 8 + 40, &negative, sizeof(negative)); // radius
    CHECK(!ReplayJournal(twin, hostile, bytes), "a negative radius bounces");
    CHECK(Speed(twin, crate) == 0.0f, "the bounced tapes moved nothing");
    CHECK(ReplayJournal(twin, tape, bytes), "the clean tape lands");
    CHECK(Speed(twin, crate) > 0.0f, "the replayed blast pushes the twin crate");
    m3DestroyWorld(twin);
}

int main(void)
{
    TestRingAndFalloff();
    TestWakeFilterAndStatics();
    TestOffCenterSpin();
    TestVoxelCarveCoupling();
    TestTreeHistoryChangesNothing();
    TestSoftPushAndKickWindow();
    TestHostileDefWall();
    TestHostileTape();
    if (s_failures == 0)
    {
        printf("test_explosion: all passed\n");
        return 0;
    }
    return 1;
}
