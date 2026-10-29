// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The journal's bytes hold no padding. A record copies each public def it
// embeds member by member into zeroed storage, so a tape is the same
// whatever the caller's def held in its padding, and every record is
// zeroed before it is filled, so its own padding never holds what the
// stack held. Both checks take the padding map from the compiler, GCC's
// __builtin_clear_padding, so a def member the copy forgets fails here
// whatever its size. test/CMakeLists.txt builds the suite only where the
// compiler has the builtin, which the GCC cells do. The layout itself is
// pinned at build time in src/format_layout.c.

#include "helpers.h"
#include "journal.h"
#include "test_harness.h"
#include "world_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A def's copy against the compiler's map: from a def whose every byte is
// 0x01 (true for a bool, a tiny float, a small integer), the copy keeps
// every member byte and zeroes every padding byte, as the builtin does.
#define CHECK_COPY(type, copy)                                                                     \
    do                                                                                             \
    {                                                                                              \
        type in;                                                                                   \
        type want;                                                                                 \
        type got;                                                                                  \
        memset(&in, 0x01, sizeof(in));                                                             \
        memcpy(&want, &in, sizeof(want));                                                          \
        __builtin_clear_padding(&want);                                                            \
        memset(&got, 0xAA, sizeof(got));                                                           \
        copy(&got, &in);                                                                           \
        CHECK(memcmp(&got, &want, sizeof(got)) == 0,                                               \
              #copy " copies every member of " #type " and zeroes its padding");                   \
    } while (0)

static void TestDefCopiesMatchThePaddingMap(void)
{
    CHECK_COPY(m3BodyDef, m3JournalBodyDef);
    CHECK_COPY(m3CharacterDef, m3JournalCharacterDef);
    CHECK_COPY(m3SoftBodyDef, m3JournalSoftBodyDef);
    CHECK_COPY(m3VehicleDef, m3JournalVehicleDef);
    CHECK_COPY(m3WaterVolumeDef, m3JournalWaterVolumeDef);
    CHECK_COPY(m3ShapeDef, m3JournalShapeDef);
    CHECK_COPY(m3JointDef, m3JournalJointDef);
    CHECK_COPY(m3DrivetrainDef, m3JournalDrivetrainDef);
    CHECK_COPY(m3ExplosionDef, m3JournalExplosionDef);
}

// The def a host hands over: the model's members, and fill in every
// padding byte.
#define PADDED(type, out, model, fill)                                                             \
    do                                                                                             \
    {                                                                                              \
        type map;                                                                                  \
        memset(&map, 0xFF, sizeof(map));                                                           \
        __builtin_clear_padding(&map);                                                             \
        const uint8_t* isMember = (const uint8_t*)&map;                                            \
        const uint8_t* from = (const uint8_t*)&(model);                                            \
        uint8_t* to = (uint8_t*)&(out);                                                            \
        for (size_t i = 0; i < sizeof(type); ++i)                                                  \
        {                                                                                          \
            to[i] = isMember[i] != 0 ? from[i] : (fill);                                           \
        }                                                                                          \
    } while (0)

// Paints the stack the next library call runs on with fill, so a record
// the library builds there and leaves partly unwritten carries fill onto
// the tape. Called through a volatile pointer, so it is never inlined, and
// with volatile stores, so none is dropped.
static void PaintStack(uint8_t fill)
{
    volatile uint8_t area[16384];
    for (size_t i = 0; i < sizeof(area); ++i)
    {
        area[i] = fill;
    }
}

static void (*volatile s_paint)(uint8_t) = PaintStack;

// One session through every recorder that copies a def: a body, each kind
// of shape create, a joint, a character, both soft body creates, a
// vehicle with its drivetrain, a water volume and an explosion. Every def
// is handed over with fill in its padding, and every recording call runs
// on a stack painted with fill. The records with padding of their own
// are all here: the hull, height field grid and voxel chunk creates, the
// filter and both pushes at a point. Returns the tape's bytes.
static int32_t RecordSession(uint8_t fill, uint8_t* tape, int32_t capacity)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    CHECK(StartJournal(world, tape, capacity), "the journal starts");

    m3BodyDef bodyModel = m3DefaultBodyDef();
    m3BodyDef bd;
    PADDED(m3BodyDef, bd, bodyModel, fill);
    s_paint(fill);
    m3BodyId ground = NewBody(world, &bd);
    bodyModel.type = m3_dynamicBody;
    bodyModel.position = (m3Pos3){0.0, 2.0, 0.0};
    PADDED(m3BodyDef, bd, bodyModel, fill);
    s_paint(fill);
    m3BodyId a = NewBody(world, &bd);
    bodyModel.position = (m3Pos3){2.0, 2.0, 0.0};
    PADDED(m3BodyDef, bd, bodyModel, fill);
    s_paint(fill);
    m3BodyId b = NewBody(world, &bd);

    m3ShapeDef shapeModel = m3DefaultShapeDef();
    m3ShapeDef sd;
    PADDED(m3ShapeDef, sd, shapeModel, fill);
    s_paint(fill);
    m3ShapeId box = NewBoxShape(world, a, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    static const m3Vec3 points[4] = {
        {0.0f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}, {0.0f, 0.6f, 0.0f}, {0.0f, 0.0f, 0.6f}};
    s_paint(fill);
    m3ShapeId hull = NewHullShape(world, b, &sd, points, 4);
    static const m3Vec3 floor[4] = {
        {-5.0f, 0.0f, -5.0f}, {5.0f, 0.0f, -5.0f}, {5.0f, 0.0f, 5.0f}, {-5.0f, 0.0f, 5.0f}};
    static const uint16_t floorTris[6] = {0, 2, 1, 0, 3, 2};
    s_paint(fill);
    m3ShapeId mesh = NewMeshShape(world, ground, &sd, floor, 4, floorTris, 2);
    static const float heights[9] = {0.0f, 0.1f, 0.0f, 0.1f, 0.2f, 0.1f, 0.0f, 0.1f, 0.0f};
    s_paint(fill);
    m3ShapeId terrain = NewHeightFieldGridShape(world, ground, &sd, heights, 3, 3, 1.0f);
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    voxels[0] = 1;
    s_paint(fill);
    m3ShapeId chunk = NewVoxelChunkShape(world, ground, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(world, box) && m3Shape_IsValid(world, hull) &&
              m3Shape_IsValid(world, mesh) && m3Shape_IsValid(world, terrain) &&
              m3Shape_IsValid(world, chunk),
          "every kind of shape creates");
    s_paint(fill);
    m3Shape_SetFilter(world, box, 0x2u, 0xFFu, -3);
    s_paint(fill);
    m3Body_ApplyForceAtPoint(world, a, (m3Vec3){1.0f, 0.0f, 0.0f}, (m3Pos3){0.0, 2.5, 0.0});
    s_paint(fill);
    m3Body_ApplyLinearImpulseAtPoint(world, b, (m3Vec3){0.0f, 0.5f, 0.0f}, (m3Pos3){2.0, 2.5, 0.0});

    m3JointDef jointModel = m3DefaultJointDef();
    jointModel.type = m3_distanceJoint;
    jointModel.bodyIdA = a;
    jointModel.bodyIdB = b;
    jointModel.minLength = 1.0f;
    jointModel.maxLength = 3.0f;
    m3JointDef jd;
    PADDED(m3JointDef, jd, jointModel, fill);
    s_paint(fill);
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the joint creates");

    m3CharacterDef characterModel = m3DefaultCharacterDef();
    characterModel.position = (m3Pos3){-4.0, 2.0, 0.0};
    m3CharacterDef cd;
    PADDED(m3CharacterDef, cd, characterModel, fill);
    s_paint(fill);
    CHECK(m3Character_IsValid(world, NewCharacter(world, &cd)), "the character creates");

    m3SoftBodyDef softModel = m3DefaultSoftBodyDef();
    softModel.position = (m3Pos3){8.0, 3.0, 0.0};
    m3SoftBodyDef soft;
    PADDED(m3SoftBodyDef, soft, softModel, fill);
    s_paint(fill);
    CHECK(m3SoftBody_IsValid(world, NewSoftBody(world, &soft)), "the lattice soft body creates");
    static const uint16_t tets[4] = {0, 1, 2, 3};
    softModel.position = (m3Pos3){10.0, 3.0, 0.0};
    PADDED(m3SoftBodyDef, soft, softModel, fill);
    s_paint(fill);
    CHECK(m3SoftBody_IsValid(world, NewSoftBodyTet(world, &soft, points, 4, tets, 1)),
          "the tet soft body creates");

    bodyModel.position = (m3Pos3){-8.0, 1.0, 0.0};
    PADDED(m3BodyDef, bd, bodyModel, fill);
    s_paint(fill);
    m3BodyId chassis = NewBody(world, &bd);
    s_paint(fill);
    NewBoxShape(world, chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    m3VehicleDef vehicleModel = m3DefaultVehicleDef();
    vehicleModel.chassisId = chassis;
    vehicleModel.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vehicleModel.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vehicleModel.wheels[w].driven = true;
    }
    m3VehicleDef vd;
    PADDED(m3VehicleDef, vd, vehicleModel, fill);
    s_paint(fill);
    m3VehicleId car = NewVehicle(world, &vd);
    CHECK(m3Vehicle_IsValid(world, car), "the vehicle creates");
    m3DrivetrainDef drivetrainModel = m3DefaultDrivetrainDef();
    m3DrivetrainDef dt;
    PADDED(m3DrivetrainDef, dt, drivetrainModel, fill);
    s_paint(fill);
    m3Vehicle_SetDrivetrain(world, car, &dt);

    m3WaterVolumeDef waterModel = m3DefaultWaterVolumeDef();
    waterModel.lo = (m3Pos3){20.0, 0.0, 0.0};
    waterModel.hi = (m3Pos3){24.0, 2.0, 4.0};
    m3WaterVolumeDef wv;
    PADDED(m3WaterVolumeDef, wv, waterModel, fill);
    s_paint(fill);
    CHECK(m3WaterVolume_IsValid(world, NewWaterVolume(world, &wv)), "the water volume creates");

    m3ExplosionDef blastModel = m3DefaultExplosionDef();
    blastModel.position = (m3Pos3){0.0, 1.0, 0.0};
    blastModel.impulsePerArea = 1.0f;
    m3ExplosionDef blast;
    PADDED(m3ExplosionDef, blast, blastModel, fill);
    s_paint(fill);
    m3World_Explode(world, &blast);

    s_paint(fill);
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_GetCounters(world).misuse == 0, "every call of the session lands");
    int32_t bytes = StopJournal(world);
    m3DestroyWorld(world);
    return bytes;
}

static void TestTapeIgnoresDefPadding(void)
{
    enum
    {
        TAPE_BYTES = 1 << 20
    };
    uint8_t* zeroed = (uint8_t*)malloc(TAPE_BYTES);
    uint8_t* filled = (uint8_t*)malloc(TAPE_BYTES);
    int32_t zeroedBytes = RecordSession(0x00, zeroed, TAPE_BYTES);
    int32_t filledBytes = RecordSession(0xFF, filled, TAPE_BYTES);
    CHECK(zeroedBytes > 0 && zeroedBytes == filledBytes, "both sessions record the same length");
    CHECK(zeroedBytes > 0 && zeroedBytes == filledBytes &&
              memcmp(zeroed, filled, (size_t)zeroedBytes) == 0,
          "a def's padding never reaches the tape");
    free(zeroed);
    free(filled);
}

int main(void)
{
    TestDefCopiesMatchThePaddingMap();
    TestTapeIgnoresDefPadding();
    if (s_failures == 0)
    {
        printf("test_format: all green\n");
        return 0;
    }
    printf("test_format: %d failure(s)\n", s_failures);
    return 1;
}
