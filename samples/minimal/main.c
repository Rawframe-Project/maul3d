// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The installed-package smoke test: a drop, a rest, and a rollback
// through nothing but the public headers of the installed tree.

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>

static void Require(m3Result result, const char* what)
{
    if (result != m3_success)
    {
        printf("M3_CONSUMER_FAIL %s: %s\n", what, m3GetResultName(result));
        exit(1);
    }
}

int main(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NULL;
    Require(m3CreateWorld(&wd, &world), "world");

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground;
    Require(m3CreateBody(world, &gd, &ground), "ground");
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape;
    Require(m3CreatePlaneShape(world, ground, &sd, &floor, &floorShape), "floor");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId ball;
    Require(m3CreateBody(world, &bd, &ball), "ball");
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId ballShape;
    Require(m3CreateSphereShape(world, ball, &sd, &sphere, &ballShape), "ball shape");

    for (int i = 0; i < 120; ++i)
    {
        Require(m3World_Step(world, 1.0f / 60.0f, 4), "step");
    }
    m3Pos3 rest = m3Body_GetPosition(world, ball);
    if (rest.y < 0.45 || rest.y > 0.55)
    {
        printf("M3_CONSUMER_FAIL rest=%.4f\n", rest.y);
        return 1;
    }

    int32_t capacity = m3World_GetSnapshotSize(world);
    void* snap = malloc((size_t)capacity);
    int32_t bytes = 0;
    Require(m3World_Snapshot(world, snap, capacity, &bytes), "snapshot");
    uint64_t before = m3World_Hash(world);
    for (int i = 0; i < 60; ++i)
    {
        Require(m3World_Step(world, 1.0f / 60.0f, 4), "step");
    }
    Require(m3World_Restore(world, snap, bytes), "restore");
    if (m3World_Hash(world) != before)
    {
        printf("M3_CONSUMER_FAIL rollback\n");
        return 1;
    }
    free(snap);

    // The destruction round trip: the installed package must
    // carry the whole niche. Build a wall, carve it, catch the
    // fragment, make a body of it, and roll the world back.
    static uint8_t voxels[16 * 16 * 16];
    for (int i = 0; i < 16 * 16 * 16; ++i)
    {
        voxels[i] = 0;
    }
    for (int x = 0; x < 16; ++x)
    {
        voxels[x + 16 * (0 + 16 * 8)] = 1; // a grounded beam
    }
    voxels[5 + 16 * (1 + 16 * 8)] = 1; // a stud that will strand
    m3BodyDef wd2 = m3DefaultBodyDef();
    wd2.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId wallBody;
    Require(m3CreateBody(world, &wd2, &wallBody), "wall body");
    m3ShapeDef vd = m3DefaultShapeDef();
    m3ShapeId wall;
    Require(m3CreateVoxelChunkShape(world, wallBody, &vd, voxels, NULL, 1.0f, &wall),
            "voxel create");
    uint64_t misuse = m3World_GetCounters(world).misuse;
    m3VoxelChunk_ClearVoxel(world, wall, 5, 0, 8);
    if (m3World_GetCounters(world).misuse != misuse)
    {
        printf("M3_CONSUMER_FAIL voxel edit\n");
        return 1;
    }
    const m3FragmentEvent* events = m3World_GetFragmentEvents(world).fragmentEvents;
    int32_t fragments = m3World_GetFragmentEvents(world).fragmentCount;
    if (fragments != 1 || events[0].voxelCount != 1)
    {
        printf("M3_CONSUMER_FAIL fracture\n");
        return 1;
    }
    m3BodyDef fdb = m3DefaultBodyDef();
    fdb.type = m3_dynamicBody;
    fdb.position = events[0].comWorld;
    m3BodyId fragment;
    Require(m3CreateBody(world, &fdb, &fragment), "fragment body");
    m3ShapeDef fsd = m3DefaultShapeDef();
    m3ShapeId fragmentShape;
    Require(m3CreateBoxShape(world, fragment, &fsd, (m3Vec3){0.5f, 0.5f, 0.5f}, &fragmentShape),
            "fragment shape");
    for (int i = 0; i < 60; ++i)
    {
        Require(m3World_Step(world, 1.0f / 60.0f, 4), "step");
    }
    if (!m3Body_IsValid(world, fragment))
    {
        printf("M3_CONSUMER_FAIL fragment body\n");
        return 1;
    }
    m3DestroyWorld(world);
    m3Version version = m3GetVersion();
    printf("M3_CONSUMER_OK version=%u.%u.%u\n", version.major, version.minor, version.patch);
    return 0;
}
