// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restore refuses a structure this engine could not have written. Each
// case below takes a snapshot of the fuzz scene, breaks one link or
// cross-reference that every block check passes (a cycle, a twice-listed
// or missing member, a box inside out, a count that disagrees), and
// requires the restore to refuse it as invalid input with the world left
// as it was. Every whole-structure check (src/snapshot_links.c and
// src/snapshot_values.c) has a case here, so removing one fails this
// suite, not only the fuzzer. The other half: a snapshot the engine
// writes restores whenever it is taken, also between a call and the step
// that follows it.

#include "fuzz_scene.h"
#include "state_prefix.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    // The snapshot header ahead of the fixed prefix.
    HEADER_BYTES = M3_SNAPSHOT_HEADER_BYTES
};

// Where element i of the field at offset sits in a snapshot of world.
static uint8_t* At(const m3World* world, uint8_t* snap, size_t field, size_t width, int32_t i)
{
    int64_t at = m3StateFieldAt(world, field);
    CHECK(at >= 0, "the field is snapshot state");
    return snap + HEADER_BYTES + (at < 0 ? 0 : at) + width * (size_t)i;
}

static int32_t GetI32(const uint8_t* at)
{
    int32_t value;
    memcpy(&value, at, sizeof(value));
    return value;
}

static void PutI32(uint8_t* at, int32_t value)
{
    memcpy(at, &value, sizeof(value));
}

static void PutF32(uint8_t* at, float value)
{
    memcpy(at, &value, sizeof(value));
}

#define FIELD(name)     offsetof(m3World, name)
#define I32_AT(name, i) At(world, snap, FIELD(name), sizeof(int32_t), (i))

typedef struct Scene
{
    m3World* world;
    int32_t ground; // the static body under the floor plane
    int32_t crate;  // a dynamic body with one box
    int32_t ball;   // a dynamic body with one sphere
} Scene;

typedef void (*Break)(const Scene* scene, uint8_t* snap);

// --- One break per check ---------------------------------------------------

static void FreeCountOff(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    uint8_t* at = I32_AT(bodies.bodyPool.freeCount, 0);
    PutI32(at, GetI32(at) + 1);
}

static void HullRefCountOff(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t shape = world->bodies.bodyShapeHead[scene->crate];
    uint8_t* at = I32_AT(hulls.hullRefCounts, world->shapes.shapeHullIndex[shape]);
    PutI32(at, GetI32(at) + 1);
}

static void BoxNamesMesh(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    PutI32(I32_AT(shapes.shapeMeshIndex, world->bodies.bodyShapeHead[scene->crate]), 0);
}

static void ShapeListCycle(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t shape = world->bodies.bodyShapeHead[scene->crate];
    PutI32(I32_AT(shapes.shapeNext, shape), shape);
}

static void JointListCycle(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t joint = world->joints.bodyJointHead[scene->ball];
    size_t next = world->joints.jointBodyA[joint] == scene->ball ? FIELD(joints.jointNextA)
                                                                 : FIELD(joints.jointNextB);
    PutI32(At(world, snap, next, sizeof(int32_t), joint), joint);
}

static void JointOnOneBody(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t joint = world->joints.bodyJointHead[scene->ball];
    PutI32(I32_AT(joints.jointBodyB, joint), world->joints.jointBodyA[joint]);
}

static void LeafInsideOut(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t leaf = world->broadphase.proxyIds[world->bodies.bodyShapeHead[scene->crate]];
    uint8_t* node = At(world, snap, FIELD(broadphase.tree.nodes), sizeof(m3TreeNode), leaf);
    double lo = world->broadphase.tree.nodes[leaf].hi[1] + 1.0;
    memcpy(node + offsetof(m3TreeNode, lo) + sizeof(double), &lo, sizeof(lo));
}

static void ProxyOnAnotherLeaf(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t box = world->bodies.bodyShapeHead[scene->crate];
    int32_t sphere = world->bodies.bodyShapeHead[scene->ball];
    PutI32(I32_AT(broadphase.proxyIds, box), world->broadphase.proxyIds[sphere]);
}

static void PairsOutOfOrder(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    uint64_t first = world->contacts.pairKeys[0];
    uint64_t second = world->contacts.pairKeys[1];
    memcpy(At(world, snap, FIELD(contacts.pairKeys), sizeof(uint64_t), 0), &second, 8);
    memcpy(At(world, snap, FIELD(contacts.pairKeys), sizeof(uint64_t), 1), &first, 8);
}

static void WorldWithoutStiffness(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    PutF32(At(world, snap, FIELD(contactHertz), sizeof(float), 0), 0.0f);
}

static void LockBitUnknown(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    *At(world, snap, FIELD(bodies.bodyLocks), 1, scene->crate) = 0x80;
}

// A density no setter accepts on a dynamic body: the shape weighs less
// than the mass floor. The box's mass comes from its hull in the tail.
static void BallBelowMassFloor(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    PutF32(At(world, snap, FIELD(shapes.shapeDensity), sizeof(float),
              world->bodies.bodyShapeHead[scene->ball]),
           1.0e-30f);
}

static void BoxBelowMassFloor(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    PutF32(At(world, snap, FIELD(shapes.shapeDensity), sizeof(float),
              world->bodies.bodyShapeHead[scene->crate]),
           1.0e-30f);
}

// A stored contact impulse the next step would carry into its warm start.
static void ContactImpulseNotFinite(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    uint8_t* manifold = At(world, snap, FIELD(contacts.manifolds), sizeof(m3Manifold), 0);
    float nan;
    uint32_t bits = 0x7FC00000u;
    memcpy(&nan, &bits, sizeof(nan));
    PutF32(manifold + offsetof(m3Manifold, points) + offsetof(m3ManifoldPoint, normalImpulse), nan);
}

static void SphereRadiusNegative(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    uint8_t* geom = At(world, snap, FIELD(shapes.shapeGeom), sizeof(m3ShapeGeom),
                       world->bodies.bodyShapeHead[scene->ball]);
    PutF32(geom + offsetof(m3ShapeGeom, s), -0.3f);
}

static void JointFrameLong(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    m3Quat twice = {0.0f, 0.0f, 0.0f, 2.0f};
    memcpy(At(world, snap, FIELD(joints.jointFrameQA), sizeof(m3Quat),
              world->joints.bodyJointHead[scene->ball]),
           &twice, sizeof(twice));
}

static int32_t LiveSlot(const uint8_t* alive, int32_t capacity)
{
    for (int32_t i = 0; i < capacity; ++i)
    {
        if (alive[i] != 0)
        {
            return i;
        }
    }
    CHECK(false, "the scene holds one of each kind");
    return 0;
}

static void CharacterWithoutBody(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->characters.charPool.alive, world->characters.charPool.capacity);
    PutI32(I32_AT(characters.charBody, slot), -1);
}

static void CharacterOnStaticBody(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->characters.charPool.alive, world->characters.charPool.capacity);
    PutI32(I32_AT(characters.charBody, slot), scene->ground);
}

static void GearWithoutDrivetrain(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->vehicles.vehPool.alive, world->vehicles.vehPool.capacity);
    *At(world, snap, FIELD(vehicles.vehDtGear), 1, slot) = 3;
}

static void VehicleWithoutChassis(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->vehicles.vehPool.alive, world->vehicles.vehPool.capacity);
    PutI32(I32_AT(vehicles.vehChassis, slot), -1);
}

static void VehicleOnDeadChassis(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->vehicles.vehPool.alive, world->vehicles.vehPool.capacity);
    int32_t dead = world->bodies.bodyPool.capacity - 1; // past every live body
    CHECK(world->bodies.bodyPool.alive[dead] == 0, "the last body slot is dead");
    PutI32(I32_AT(vehicles.vehChassis, slot), dead);
}

static void VehicleOnStaleChassis(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->vehicles.vehPool.alive, world->vehicles.vehPool.capacity);
    uint32_t stale = world->vehicles.vehChassisGen[slot] + 1u;
    memcpy(At(world, snap, FIELD(vehicles.vehChassisGen), sizeof(uint32_t), slot), &stale,
           sizeof(stale));
}

static void EdgePastParticles(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->softBodies.softPool.alive, world->softBodies.softPool.capacity);
    uint16_t past = (uint16_t)world->softBodies.softParticleCount[slot];
    memcpy(At(world, snap, FIELD(softBodies.softEdgeA), sizeof(uint16_t),
              slot * M3_SOFTBODY_MAX_EDGES),
           &past, sizeof(past));
}

static void LatticeOverflowing(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->softBodies.softPool.alive, world->softBodies.softPool.capacity);
    uint16_t most = 0xFFFF;
    memcpy(At(world, snap, FIELD(softBodies.softDimX), sizeof(uint16_t), slot), &most, 2);
    memcpy(At(world, snap, FIELD(softBodies.softDimY), sizeof(uint16_t), slot), &most, 2);
    memcpy(At(world, snap, FIELD(softBodies.softDimZ), sizeof(uint16_t), slot), &most, 2);
}

static void JointFlagUnknown(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    *At(world, snap, FIELD(joints.jointFlags), 1, world->joints.bodyJointHead[scene->ball]) = 0x80;
}

static void HingeWithGenericModes(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    uint16_t modes = 1;
    memcpy(At(world, snap, FIELD(joints.jointGenericModes), sizeof(uint16_t),
              world->joints.bodyJointHead[scene->ball]),
           &modes, sizeof(modes));
}

typedef struct Case
{
    const char* name;
    Break apply;
} Case;

static const Case s_cases[] = {
    {"a free count the free queue does not hold", FreeCountOff},
    {"a hull reference count one too high", HullRefCountOff},
    {"a box that names a mesh", BoxNamesMesh},
    {"a shape list that loops", ShapeListCycle},
    {"a joint list that loops", JointListCycle},
    {"a joint between a body and itself", JointOnOneBody},
    {"a tree leaf inside out", LeafInsideOut},
    {"a proxy on another shape's leaf", ProxyOnAnotherLeaf},
    {"contact pairs out of order", PairsOutOfOrder},
    {"a world with no contact stiffness", WorldWithoutStiffness},
    {"a lock bit no setter writes", LockBitUnknown},
    {"a ball below the mass floor", BallBelowMassFloor},
    {"a box below the mass floor", BoxBelowMassFloor},
    {"a contact impulse that is not a number", ContactImpulseNotFinite},
    {"a sphere with a negative radius", SphereRadiusNegative},
    {"a joint frame twice unit length", JointFrameLong},
    {"a live character with no body slot", CharacterWithoutBody},
    {"a character on a static body", CharacterOnStaticBody},
    {"a gear with no drivetrain", GearWithoutDrivetrain},
    {"a vehicle with no chassis", VehicleWithoutChassis},
    {"a vehicle on a dead chassis", VehicleOnDeadChassis},
    {"a vehicle on a chassis of another generation", VehicleOnStaleChassis},
    {"a soft edge past the particles", EdgePastParticles},
    {"a lattice whose dimensions overflow 32 bits", LatticeOverflowing},
    {"a joint flag no setter writes", JointFlagUnknown},
    {"a hinge with generic modes", HingeWithGenericModes},
};

static void TestRestoreRefusesBrokenStructures(void)
{
    m3WorldDef def = FuzzWorldDef();
    Scene scene = {NewWorld(&def), 0, 0, 0};
    FuzzScene built;
    CHECK(FuzzBuild(scene.world, &built), "the scene builds");
    FuzzStep(scene.world, 4);
    scene.ground = 0;
    scene.crate = built.crate.index1 - 1;
    scene.ball = built.ball.index1 - 1;
    CHECK(scene.world->bodies.types[scene.ground] == (uint8_t)m3_staticBody, "the ground");
    CHECK(scene.world->contacts.pairCount >= 2, "the scene has pairs to reorder");

    int32_t size = m3World_SnapshotSize(scene.world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    uint8_t* broken = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && broken != NULL, "snapshot buffers");
    if (clean == NULL || broken == NULL)
    {
        free(clean);
        free(broken);
        m3DestroyWorld(scene.world);
        return;
    }
    CHECK(Snapshot(scene.world, clean, size) == size, "snapshot taken");
    CHECK(m3World_Restore(scene.world, clean, size) == m3_success, "the clean snapshot restores");
    uint64_t hash = m3World_Hash(scene.world);

    for (size_t c = 0; c < sizeof(s_cases) / sizeof(s_cases[0]); ++c)
    {
        memcpy(broken, clean, (size_t)size);
        s_cases[c].apply(&scene, broken);
        CHECK(memcmp(broken, clean, (size_t)size) != 0, s_cases[c].name);
        m3Result result = m3World_Restore(scene.world, broken, size);
        if (result != m3_errorInvalid)
        {
            printf("  not refused: %s\n", s_cases[c].name);
        }
        CHECK(result == m3_errorInvalid, "a broken structure is refused as invalid input");
        CHECK(m3World_Hash(scene.world) == hash, "and the world is left as it was");
    }

    free(broken);
    free(clean);
    m3DestroyWorld(scene.world);
}

// --- What the engine writes between a call and the next step -------------

typedef void (*Call)(m3World* world, const FuzzScene* scene);

static void DestroyBall(m3World* world, const FuzzScene* scene)
{
    m3ShapeId shapes[1];
    CHECK(m3Body_GetShapes(world, scene->ball, shapes, 1) == 1, "the ball has its sphere");
    CHECK(m3DestroyShape(world, shapes[0]) == m3_success, "the sphere is destroyed");
}

static void DestroyBallAndCreate(m3World* world, const FuzzScene* scene)
{
    DestroyBall(world, scene);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere small = {{0.0f, 0.0f, 0.0f}, 0.1f};
    CHECK(m3Shape_IsValid(world, NewSphereShape(world, scene->crate, &sd, &small)),
          "a new shape takes the freed slot");
}

static void TeleportCrate(m3World* world, const FuzzScene* scene)
{
    m3Body_SetTransform(world, scene->crate, (m3Pos3){40.0, 5.0, 40.0}, m3MakeIdentityQuat());
}

typedef struct EngineCase
{
    const char* name;
    Call call;
} EngineCase;

static const EngineCase s_engineCases[] = {
    {"a shape destroyed, its pairs not yet rebuilt", DestroyBall},
    {"a destroyed shape's slot taken by a new one", DestroyBallAndCreate},
    {"a body teleported away from its pairs", TeleportCrate},
};

static void TestRestoreAcceptsWhatTheEngineWrites(void)
{
    for (size_t c = 0; c < sizeof(s_engineCases) / sizeof(s_engineCases[0]); ++c)
    {
        m3WorldDef def = FuzzWorldDef();
        m3World* world = NewWorld(&def);
        FuzzScene scene;
        CHECK(FuzzBuild(world, &scene), "the scene builds");
        FuzzStep(world, 4);
        CHECK(world->contacts.pairCount > 0, "the scene has pairs");
        s_engineCases[c].call(world, &scene);
        int32_t size = m3World_SnapshotSize(world);
        uint8_t* bytes = (uint8_t*)malloc((size_t)size);
        CHECK(bytes != NULL && Snapshot(world, bytes, size) == size, "snapshot taken");
        uint64_t hash = m3World_Hash(world);
        bool restored = bytes != NULL && Restore(world, bytes, size);
        if (!restored)
        {
            printf("  refused: %s\n", s_engineCases[c].name);
        }
        CHECK(restored, "a snapshot the engine wrote before its next step restores");
        CHECK(m3World_Hash(world) == hash, "and restoring it changes nothing");
        FuzzStep(world, 2);
        free(bytes);
        m3DestroyWorld(world);
    }
}

int main(void)
{
    TestRestoreRefusesBrokenStructures();
    TestRestoreAcceptsWhatTheEngineWrites();
    if (s_failures == 0)
    {
        printf("test_restore_links: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
