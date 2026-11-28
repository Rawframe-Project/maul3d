// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A capsule whose two points meet is a sphere, and every call refuses one:
// m3CreateCapsuleShape and m3Shape_SetCapsule return m3_errorInvalid.
// Only a character's own capsule may have zero height, as its stance.
// Replay and restore take a capsule only in a form those calls make: a
// tape whose capsule create has its points moved together, and a snapshot
// of a crate's capsule with its points moved together, are refused, while
// a character's zero-height capsule still restores.

#include "helpers.h"
#include "journal.h"
#include "shape.h"
#include "state_prefix.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    TAPE_BYTES = 1 << 16
};

#define F(field) offsetof(m3World, field)

static m3World* DefaultWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    return NewWorld(&def);
}

// A crate with a capsule standing on its axis.
static m3ShapeId BuildCrate(m3World* world)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.25f};
    m3ShapeId shape = m3_nullShapeId;
    CHECK(m3CreateCapsuleShape(world, crate, &sd, &capsule, &shape) == m3_success,
          "the crate's capsule is built");
    return shape;
}

static void TestCallsRefuseAZeroLengthCapsule(void)
{
    m3World* world = DefaultWorld();
    m3ShapeId shape = BuildCrate(world);
    m3BodyId crate = m3Shape_GetBody(world, shape);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule point = {{0.0f, 0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.25f};
    m3ShapeId other = m3_nullShapeId;
    CHECK(m3CreateCapsuleShape(world, crate, &sd, &point, &other) == m3_errorInvalid,
          "create refuses a capsule whose points meet");
    CHECK(m3Shape_SetCapsule(world, shape, &point) == m3_errorInvalid, "and so does the setter");
    m3DestroyWorld(world);
}

static void TestReplayRefusesAZeroLengthCapsule(void)
{
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    m3World* world = DefaultWorld();
    CHECK(tape != NULL && StartJournal(world, tape, TAPE_BYTES), "the journal starts");
    if (tape == NULL)
    {
        m3DestroyWorld(world);
        return;
    }
    (void)BuildCrate(world);
    int64_t bytes = StopJournal(world);
    m3DestroyWorld(world);

    m3World* again = DefaultWorld();
    CHECK(ReplayJournal(again, tape, bytes), "the tape replays");
    m3DestroyWorld(again);

    int32_t create = -1;
    for (int32_t at = M3_JOURNAL_HEADER_BYTES; at + 8 <= bytes;)
    {
        int32_t op;
        int32_t size;
        memcpy(&op, tape + at, sizeof(op));
        memcpy(&size, tape + at + 4, sizeof(size));
        if (op == (int32_t)m3_opCreateShape)
        {
            create = at + 8;
        }
        at += 8 + size;
    }
    CHECK(create >= 0, "the capsule's create is on the tape");
    if (create >= 0)
    {
        uint8_t* geom = tape + create + offsetof(m3CreateShapeOp, geom);
        memcpy(geom + offsetof(m3ShapeGeom, v2), geom + offsetof(m3ShapeGeom, v), sizeof(m3Vec3));
        m3World* refused = DefaultWorld();
        CHECK(m3World_ReplayJournal(refused, tape, bytes) == m3_errorFormat,
              "a capsule create with its points together is refused");
        m3DestroyWorld(refused);
    }
    free(tape);
}

// Moves the capsule's second point onto its first in a snapshot of the
// world, and restores it into the world.
static m3Result RestoreCollapsed(m3World* world, m3ShapeId shape)
{
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "snapshot taken");
    if (snap == NULL)
    {
        return m3_errorFormat;
    }
    uint8_t* geom = snap + M3_SNAPSHOT_HEADER_BYTES + m3StateFieldAt(world, F(shapes.shapeGeom)) +
                    sizeof(m3ShapeGeom) * (size_t)(shape.index1 - 1);
    memcpy(geom + offsetof(m3ShapeGeom, v2), geom + offsetof(m3ShapeGeom, v), sizeof(m3Vec3));
    m3Result result = m3World_Restore(world, snap, size);
    free(snap);
    return result;
}

static void TestRestoreRefusesAZeroLengthCapsule(void)
{
    m3World* world = DefaultWorld();
    m3ShapeId shape = BuildCrate(world);
    uint64_t hash = m3World_Hash(world);
    CHECK(RestoreCollapsed(world, shape) == m3_errorFormat,
          "a crate's capsule with its points together is refused");
    CHECK(m3World_Hash(world) == hash, "and the world is left as it was");
    m3DestroyWorld(world);
}

// A character created with no height stands on a capsule whose points
// meet, and its world restores.
static void TestCharacterKeepsZeroHeight(void)
{
    m3World* world = DefaultWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.halfHeight = 0.0f;
    (void)NewCharacter(world, &cd);
    int32_t capsule = -1;
    for (int32_t s = 0; s < world->shapes.shapePool.capacity; ++s)
    {
        if (world->shapes.shapePool.alive[s] != 0 &&
            world->shapes.shapeType[s] == (uint8_t)m3_capsuleShape)
        {
            capsule = s;
        }
    }
    CHECK(capsule >= 0 && !m3CapsuleHasLength(&world->shapes.shapeGeom[capsule]),
          "the character's capsule has zero height");
    uint64_t hash = m3World_Hash(world);
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "snapshot taken");
    if (snap != NULL)
    {
        CHECK(Restore(world, snap, size) && m3World_Hash(world) == hash,
              "and its world restores to the same state");
        free(snap);
    }
    m3DestroyWorld(world);
}

int main(void)
{
    TestCallsRefuseAZeroLengthCapsule();
    TestReplayRefusesAZeroLengthCapsule();
    TestRestoreRefusesAZeroLengthCapsule();
    TestCharacterKeepsZeroHeight();
    if (s_failures == 0)
    {
        printf("test_shape_defs: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
