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
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    // The snapshot header ahead of the fixed prefix (src/snapshot.c
    // asserts its size).
    HEADER_BYTES = 48
};

// Where element i of the field at offset sits in a snapshot of world.
static uint8_t* At(const m3World* world, uint8_t* snap, size_t field, size_t width, int32_t i)
{
    int32_t at = m3StateFieldAt(world, field);
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
    uint16_t stale = (uint16_t)(world->vehicles.vehChassisGen[slot] + 1u);
    memcpy(At(world, snap, FIELD(vehicles.vehChassisGen), sizeof(uint16_t), slot), &stale,
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

// --- Magnitudes at their edges -----------------------------------------------

// The live slots the edge cases write into.
typedef struct Edges
{
    const m3World* world;
    int32_t ball;    // a dynamic body with one sphere
    int32_t hinge;   // a revolute joint
    int32_t hero;    // a character, whose body is kinematic
    int32_t vehicle; // its first wheel
    int32_t soft;    // its first particle
    int32_t pool;    // a water volume
} Edges;

typedef void (*Put)(const Edges* e, uint8_t* snap, float value);

// Component c of element i of an m3Vec3 field.
static uint8_t* V3At(const Edges* e, uint8_t* snap, size_t field, int32_t i, int32_t c)
{
    return At(e->world, snap, field, sizeof(m3Vec3), i) + sizeof(float) * (size_t)c;
}

static void PutGravity(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(gravity), 0, 1), v);
}

static void PutContactHertz(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(contactHertz), sizeof(float), 0), v);
}

static void PutSpeedCap(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(maximumLinearSpeed), sizeof(float), 0), v);
}

static void PutGustScale(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(windGustScale), sizeof(float), 0), v);
}

static void PutBallVelocity(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(bodies.linearVelocities), e->ball, 0), v);
}

static void PutBallSpin(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(bodies.angularVelocities), e->ball, 2), v);
}

static void PutBallGravityScale(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(bodies.gravityScales), sizeof(float), e->ball), v);
}

static void PutBallDamping(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(bodies.linearDamping), sizeof(float), e->ball), v);
}

static void PutBallForce(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(bodies.bodyForce), e->ball, 1), v);
}

static int32_t HeroBody(const Edges* e)
{
    return e->world->characters.charBody[e->hero];
}

static void PutKinematicVelocity(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(bodies.linearVelocities), HeroBody(e), 0), v);
}

// A servo target v meters from the kinematic body along x.
static void PutTargetReach(const Edges* e, uint8_t* snap, float v)
{
    m3Transform target = e->world->bodies.transforms[HeroBody(e)];
    target.p.x += (double)v;
    *At(e->world, snap, FIELD(bodies.bodyHasTarget), 1, HeroBody(e)) = 1;
    memcpy(At(e->world, snap, FIELD(bodies.bodyTarget), sizeof(m3Transform), HeroBody(e)), &target,
           sizeof(target));
}

static int32_t BallShape(const Edges* e)
{
    return e->world->bodies.bodyShapeHead[e->ball];
}

static void PutDensity(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(shapes.shapeDensity), sizeof(float), BallShape(e)), v);
}

static void PutRestitution(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(shapes.shapeRestitution), sizeof(float), BallShape(e)), v);
}

static void PutSurfaceVelocity(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(shapes.shapeSurfaceVel), BallShape(e), 0), v);
}

static void PutMotorSpeed(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(joints.jointMotor), e->hinge, 0), v);
}

static void PutMotorEffort(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(joints.jointMotor), e->hinge, 1), v);
}

static void PutAnchor(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(joints.jointLocalA), e->hinge, 0), v);
}

static void PutSpringHertz(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(joints.jointSpring), e->hinge, 0), v);
}

static void PutHeroMass(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(characters.charMass), sizeof(float), e->hero), v);
}

static void PutWheelRadius(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(vehicles.vehWheelRadius), sizeof(float),
              e->vehicle * M3_VEHICLE_MAX_WHEELS),
           v);
}

static void PutDriveForce(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(vehicles.vehDriveForce), sizeof(float), e->vehicle), v);
}

static void PutWheelSpeed(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(vehicles.vehWheelLon), sizeof(float),
              e->vehicle * M3_VEHICLE_MAX_WHEELS),
           v);
}

static void PutParticleInvMass(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(softBodies.softInvMass), sizeof(float),
              e->soft * M3_SOFTBODY_MAX_PARTICLES),
           v);
}

static void PutParticleKick(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(softBodies.softKick), e->soft * M3_SOFTBODY_MAX_PARTICLES, 0), v);
}

static void PutWaterFlow(const Edges* e, uint8_t* snap, float v)
{
    PutF32(V3At(e, snap, FIELD(water.waterFlow), e->pool, 0), v);
}

static void PutWaterDensity(const Edges* e, uint8_t* snap, float v)
{
    PutF32(At(e->world, snap, FIELD(water.waterDensity), sizeof(float), e->pool), v);
}

typedef struct EdgeCase
{
    const char* name;
    Put put;
    float limit; // restores
    float past;  // the next float beyond it, refused
} EdgeCase;

static float Up(float x)
{
    return nextafterf(x, 2.0f * x + 1.0f);
}

static float Down(float x)
{
    return nextafterf(x, -1.0f);
}

static void TestRestoreBoundsAtTheirEdges(void)
{
    const EdgeCase cases[] = {
        {"gravity", PutGravity, -M3_ACCEL_LIMIT, -Up(M3_ACCEL_LIMIT)},
        {"contact hertz", PutContactHertz, M3_HERTZ_LIMIT, Up(M3_HERTZ_LIMIT)},
        {"speed cap", PutSpeedCap, M3_SPEED_LIMIT, Up(M3_SPEED_LIMIT)},
        {"gust scale", PutGustScale, M3_RATIO_LIMIT, Up(M3_RATIO_LIMIT)},
        {"dynamic velocity", PutBallVelocity, M3_SPEED_LIMIT, Up(M3_SPEED_LIMIT)},
        {"spin", PutBallSpin, -M3_SPEED_LIMIT, -Up(M3_SPEED_LIMIT)},
        {"gravity scale", PutBallGravityScale, M3_RATIO_LIMIT, Up(M3_RATIO_LIMIT)},
        {"damping", PutBallDamping, 0.0f, Down(0.0f)},
        {"held force", PutBallForce, M3_FORCE_LIMIT, Up(M3_FORCE_LIMIT)},
        {"kinematic velocity", PutKinematicVelocity, M3_KINEMATIC_SPEED_LIMIT,
         Up(M3_KINEMATIC_SPEED_LIMIT)},
        {"servo reach", PutTargetReach, (float)M3_SERVO_REACH, Up((float)M3_SERVO_REACH)},
        {"shape density", PutDensity, M3_DENSITY_LIMIT, Up(M3_DENSITY_LIMIT)},
        {"restitution", PutRestitution, M3_RATIO_LIMIT, Up(M3_RATIO_LIMIT)},
        {"surface velocity", PutSurfaceVelocity, M3_SPEED_LIMIT, Up(M3_SPEED_LIMIT)},
        {"motor speed", PutMotorSpeed, M3_SPEED_LIMIT, Up(M3_SPEED_LIMIT)},
        {"motor effort", PutMotorEffort, M3_FORCE_LIMIT, Up(M3_FORCE_LIMIT)},
        {"joint anchor", PutAnchor, M3_LOCAL_LIMIT, Up(M3_LOCAL_LIMIT)},
        {"spring hertz", PutSpringHertz, M3_HERTZ_LIMIT, Up(M3_HERTZ_LIMIT)},
        {"character mass", PutHeroMass, M3_MASS_LIMIT, Up(M3_MASS_LIMIT)},
        {"wheel radius", PutWheelRadius, M3_WHEEL_RADIUS_FLOOR, Down(M3_WHEEL_RADIUS_FLOOR)},
        {"drive force", PutDriveForce, M3_FORCE_LIMIT, Up(M3_FORCE_LIMIT)},
        {"wheel contact speed", PutWheelSpeed, 2.0f * M3_KINEMATIC_SPEED_LIMIT,
         Up(2.0f * M3_KINEMATIC_SPEED_LIMIT)},
        {"particle mass floor", PutParticleInvMass, 1.0f / M3_MASS_FLOOR, Up(1.0f / M3_MASS_FLOOR)},
        {"particle kick", PutParticleKick, M3_SPEED_LIMIT, Up(M3_SPEED_LIMIT)},
        {"water flow", PutWaterFlow, M3_SPEED_LIMIT, Up(M3_SPEED_LIMIT)},
        {"water density", PutWaterDensity, M3_DENSITY_LIMIT, Up(M3_DENSITY_LIMIT)},
    };
    m3WorldDef def = FuzzWorldDef();
    m3World* world = NewWorld(&def);
    FuzzScene built;
    CHECK(FuzzBuild(world, &built), "the scene builds");
    FuzzStep(world, 4);
    Edges e = {world,
               built.ball.index1 - 1,
               built.hinge.index1 - 1,
               built.hero.index1 - 1,
               LiveSlot(world->vehicles.vehPool.alive, world->vehicles.vehPool.capacity),
               LiveSlot(world->softBodies.softPool.alive, world->softBodies.softPool.capacity),
               built.pool.index1 - 1};
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    uint8_t* edited = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && edited != NULL && Snapshot(world, clean, size) == size,
          "snapshot taken");
    if (clean == NULL || edited == NULL)
    {
        free(clean);
        free(edited);
        m3DestroyWorld(world);
        return;
    }
    uint64_t hash = m3World_Hash(world);
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c)
    {
        memcpy(edited, clean, (size_t)size);
        cases[c].put(&e, edited, cases[c].limit);
        bool atLimit = m3World_Restore(world, edited, size) == m3_success;
        CHECK(Restore(world, clean, size) && m3World_Hash(world) == hash, "back to the scene");
        memcpy(edited, clean, (size_t)size);
        cases[c].put(&e, edited, cases[c].past);
        bool pastRefused = m3World_Restore(world, edited, size) == m3_errorInvalid;
        if (!atLimit || !pastRefused)
        {
            printf("  %s: at the limit %s, past it %s\n", cases[c].name,
                   atLimit ? "restores" : "refused", pastRefused ? "refused" : "restores");
        }
        CHECK(atLimit, "a value at its limit restores");
        CHECK(pastRefused, "one just past it is refused");
        CHECK(m3World_Hash(world) == hash, "and the world is left as it was");
    }
    free(edited);
    free(clean);
    m3DestroyWorld(world);
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
    TestRestoreBoundsAtTheirEdges();
    if (s_failures == 0)
    {
        printf("test_restore_links: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
