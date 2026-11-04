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

static void JointOnBodyPastPool(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    PutI32(I32_AT(joints.jointBodyB, world->joints.bodyJointHead[scene->ball]), INT32_MAX);
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

// The first pair made touching, with one contact point, and its two
// shapes made sensors: the broad phase never pairs two sensors, so no
// world holds such a pair.
static void TouchingPairOfSensors(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    uint64_t key = world->contacts.pairKeys[0];
    uint8_t* manifold = At(world, snap, FIELD(contacts.manifolds), sizeof(m3Manifold), 0);
    if (GetI32(manifold + offsetof(m3Manifold, pointCount)) == 0)
    {
        PutI32(manifold + offsetof(m3Manifold, pointCount), 1);
    }
    *At(world, snap, FIELD(shapes.shapeSensor), 1, (int32_t)(key >> 32)) = 1;
    *At(world, snap, FIELD(shapes.shapeSensor), 1, (int32_t)(key & 0xFFFFFFFFu)) = 1;
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

// A sleeping body that still moves or spins: the engine writes zero
// velocities with every freeze.
static void CrateSleepsMoving(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    *At(world, snap, FIELD(bodies.awake), 1, scene->crate) = 0;
    m3Vec3 rest = {0.0f, 0.0f, 0.0f};
    memcpy(At(world, snap, FIELD(bodies.angularVelocities), sizeof(m3Vec3), scene->crate), &rest,
           sizeof(rest));
    m3Vec3 moving = {1.0f, 0.0f, 0.0f};
    memcpy(At(world, snap, FIELD(bodies.linearVelocities), sizeof(m3Vec3), scene->crate), &moving,
           sizeof(moving));
}

static void CrateSleepsSpinning(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    *At(world, snap, FIELD(bodies.awake), 1, scene->crate) = 0;
    m3Vec3 rest = {0.0f, 0.0f, 0.0f};
    memcpy(At(world, snap, FIELD(bodies.linearVelocities), sizeof(m3Vec3), scene->crate), &rest,
           sizeof(rest));
    m3Vec3 spinning = {0.0f, -0.0f, 0.0f};
    memcpy(At(world, snap, FIELD(bodies.angularVelocities), sizeof(m3Vec3), scene->crate),
           &spinning, sizeof(spinning));
}

// Only a dynamic body sleeps: create and a type change wake every body,
// and nothing puts another type to sleep.
static void GroundSleeps(const Scene* scene, uint8_t* snap)
{
    *At(scene->world, snap, FIELD(bodies.awake), 1, scene->ground) = 0;
}

static void KinematicSleeps(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t kinematic = 0;
    while (kinematic < world->bodies.bodyPool.maxIndex &&
           world->bodies.types[kinematic] != (uint8_t)m3_kinematicBody)
    {
        kinematic += 1;
    }
    CHECK(kinematic < world->bodies.bodyPool.maxIndex, "the scene holds a kinematic body");
    *At(world, snap, FIELD(bodies.awake), 1, kinematic) = 0;
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
              slot * M3_MAX_SOFTBODY_EDGES),
           &past, sizeof(past));
}

// The slot of the scene's tet body, the soft body that has tets.
static int32_t TetSlot(const m3World* world)
{
    for (int32_t s = 0; s < world->softBodies.softPool.capacity; ++s)
    {
        if (world->softBodies.softPool.alive[s] != 0 && world->softBodies.softTetCount[s] > 0)
        {
            return s;
        }
    }
    CHECK(false, "the scene holds a tet body");
    return 0;
}

static void TetCornerPastParticles(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = TetSlot(world);
    uint16_t past = (uint16_t)world->softBodies.softParticleCount[slot];
    memcpy(
        At(world, snap, FIELD(softBodies.softTetD), sizeof(uint16_t), slot * M3_MAX_SOFTBODY_TETS),
        &past, sizeof(past));
}

static void AnchorPastParticles(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->softBodies.softPool.alive, world->softBodies.softPool.capacity);
    PutI32(I32_AT(softBodies.softAnchorCount, slot), 1);
    PutI32(I32_AT(softBodies.softAnchorParticle, slot * M3_MAX_SOFTBODY_ANCHORS),
           world->softBodies.softParticleCount[slot]);
}

static void SoftAnchorPastParticles(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->softBodies.softPool.alive, world->softBodies.softPool.capacity);
    int32_t other = TetSlot(world);
    CHECK(other != slot, "two soft bodies to anchor");
    int32_t k = slot * M3_MAX_SOFTBODY_ANCHORS;
    PutI32(I32_AT(softBodies.softSoftCount, slot), 1);
    PutI32(I32_AT(softBodies.softSoftParticleA, k), world->softBodies.softParticleCount[slot]);
    PutI32(I32_AT(softBodies.softSoftSlotB, k), other);
    PutI32(At(world, snap, FIELD(softBodies.softSoftGenB), sizeof(uint32_t), k),
           (int32_t)world->softBodies.softPool.generations[other]);
    PutI32(I32_AT(softBodies.softSoftParticleB, k), 0);
}

static void VoxelCountOff(const Scene* scene, uint8_t* snap)
{
    const m3World* world = scene->world;
    int32_t slot = LiveSlot(world->voxels.voxelPool.alive, world->voxels.voxelPool.capacity);
    uint8_t* count = At(world, snap, FIELD(voxels.voxelData), sizeof(m3VoxelChunkData), slot) +
                     offsetof(m3VoxelChunkData, filledCount);
    PutI32(count, GetI32(count) + 1);
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
    {"a joint on a body past the pool", JointOnBodyPastPool},
    {"a joint between a body and itself", JointOnOneBody},
    {"a tree leaf inside out", LeafInsideOut},
    {"a proxy on another shape's leaf", ProxyOnAnotherLeaf},
    {"contact pairs out of order", PairsOutOfOrder},
    {"a touching pair of two sensors", TouchingPairOfSensors},
    {"a world with no contact stiffness", WorldWithoutStiffness},
    {"a lock bit no setter writes", LockBitUnknown},
    {"a sleeping body that moves", CrateSleepsMoving},
    {"a sleeping body whose spin is a negative zero", CrateSleepsSpinning},
    {"a static body that sleeps", GroundSleeps},
    {"a kinematic body that sleeps", KinematicSleeps},
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
    {"a tet corner past the particles", TetCornerPastParticles},
    {"a body anchor past the particles", AnchorPastParticles},
    {"a soft anchor past the particles", SoftAnchorPastParticles},
    {"a voxel chunk whose filled count is one off", VoxelCountOff},
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

    int64_t size = m3World_GetSnapshotSize(scene.world);
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
        if (result != m3_errorFormat)
        {
            printf("  not refused: %s\n", s_cases[c].name);
        }
        CHECK(result == m3_errorFormat, "a broken structure is refused as malformed bytes");
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

static void PutCrateToSleep(m3World* world, const FuzzScene* scene)
{
    m3Body_SetAwake(world, scene->crate, false);
    CHECK(!m3Body_IsAwake(world, scene->crate), "the crate sleeps");
}

// A type change wakes the body, so a sleeping crate made kinematic is
// awake, and a target it then takes writes its velocities at the step.
static void MakeSleepingCrateKinematic(m3World* world, const FuzzScene* scene)
{
    PutCrateToSleep(world, scene);
    CHECK(m3Body_SetType(world, scene->crate, m3_kinematicBody) == m3_success,
          "the crate turns kinematic");
    CHECK(m3Body_IsAwake(world, scene->crate), "and wakes");
    m3Body_SetAwake(world, scene->crate, false);
    CHECK(m3Body_IsAwake(world, scene->crate), "a kinematic body refuses to sleep");
}

static void SteerKinematicCrate(m3World* world, const FuzzScene* scene)
{
    MakeSleepingCrateKinematic(world, scene);
    CHECK(m3Body_SetTargetTransform(world, scene->crate, (m3Pos3){0.0, 3.0, 0.0},
                                    m3MakeIdentityQuat()) == m3_success,
          "the crate takes a target");
    FuzzStep(world, 1);
    m3Vec3 v = m3Body_GetLinearVelocity(world, scene->crate);
    CHECK(m3Body_IsAwake(world, scene->crate) && v.y != 0.0f, "and moves to it awake");
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
    {"a body put to sleep", PutCrateToSleep},
    {"a sleeping body made kinematic", MakeSleepingCrateKinematic},
    {"a kinematic body moving to a target", SteerKinematicCrate},
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
        int64_t size = m3World_GetSnapshotSize(world);
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

// The other side of the first case: the destroyed sphere's pairs keep
// their keys until the next step, but none of them touches, since the
// destroy zeroed them. A touching one, which no world holds, would make
// the contact readback and the debug draw read the dead shape's body
// slot, -1, so restore refuses it.
static void TestRestoreRefusesTouchingPairOnDeadShape(void)
{
    m3WorldDef def = FuzzWorldDef();
    m3World* world = NewWorld(&def);
    FuzzScene scene;
    CHECK(FuzzBuild(world, &scene), "the scene builds");
    FuzzStep(world, 4);
    m3ShapeId shapes[1];
    CHECK(m3Body_GetShapes(world, scene.ball, shapes, 1) == 1, "the ball has its sphere");
    int32_t dead = (int32_t)shapes[0].index1 - 1;
    DestroyBall(world, &scene);
    int32_t pair = -1;
    for (int32_t i = 0; i < world->contacts.pairCount && pair < 0; ++i)
    {
        uint64_t key = world->contacts.pairKeys[i];
        pair = (int32_t)(key >> 32) == dead || (int32_t)(key & 0xFFFFFFFFu) == dead ? i : -1;
    }
    CHECK(pair >= 0 && world->contacts.manifolds[pair].pointCount == 0,
          "a pair still names the dead sphere, untouched");
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "snapshot taken");
    if (snap == NULL || pair < 0)
    {
        free(snap);
        m3DestroyWorld(world);
        return;
    }
    uint64_t hash = m3World_Hash(world);
    PutI32(At(world, snap, FIELD(contacts.manifolds), sizeof(m3Manifold), pair) +
               offsetof(m3Manifold, pointCount),
           1);
    CHECK(m3World_Restore(world, snap, size) == m3_errorFormat,
          "a touching pair on a dead shape is refused as malformed bytes");
    CHECK(m3World_Hash(world) == hash, "and the world is left as it was");
    free(snap);
    m3DestroyWorld(world);
}

int main(void)
{
    TestRestoreRefusesBrokenStructures();
    TestRestoreAcceptsWhatTheEngineWrites();
    TestRestoreRefusesTouchingPairOnDeadShape();
    if (s_failures == 0)
    {
        printf("test_restore_links: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
