// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The rollback loop: snapshot, simulate a mispredicted future, restore,
// and simulate the corrected one. The restored world is bit-identical
// to the snapshot, so both timelines start from the same state.

#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#include <stdio.h>
#include <stdlib.h>

// Stops the sample on a refused call, naming the call and the reason.
static void Require(m3Result result, const char* what)
{
    if (result != m3_success)
    {
        fprintf(stderr, "%s: %s\n", what, m3GetResultName(result));
        exit(1);
    }
}

int main(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NULL;
    Require(m3CreateWorld(&wd, &world), "m3CreateWorld");
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground;
    Require(m3CreateBody(world, &gd, &ground), "m3CreateBody");
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId shape;
    Require(m3CreatePlaneShape(world, ground, &sd, &floor, &shape), "m3CreatePlaneShape");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId ball;
    Require(m3CreateBody(world, &bd, &ball), "m3CreateBody");
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    Require(m3CreateSphereShape(world, ball, &sd, &sphere, &shape), "m3CreateSphereShape");

    int64_t capacity = m3World_GetSnapshotSize(world);
    void* snap = malloc((size_t)capacity);
    if (snap == NULL)
    {
        return 1;
    }
    int64_t size = 0;
    Require(m3World_Snapshot(world, snap, capacity, &size), "m3World_Snapshot");
    uint64_t then = m3World_Hash(world);

    // The mispredicted future: the ball is kicked sideways.
    m3Body_SetLinearVelocity(world, ball, (m3Vec3){5.0f, 0.0f, 0.0f});
    for (int i = 0; i < 30; ++i)
    {
        Require(m3World_Step(world, 1.0f / 60.0f, 4), "step");
    }

    // The corrected input arrives: rewind and simulate again.
    Require(m3World_Restore(world, snap, size), "m3World_Restore");
    int same = m3World_Hash(world) == then;
    for (int i = 0; i < 30; ++i)
    {
        Require(m3World_Step(world, 1.0f / 60.0f, 4), "step");
    }
    printf("restored bit-exact: %s, world hash %016llx\n", same ? "yes" : "no",
           (unsigned long long)m3World_Hash(world));
    free(snap);
    m3DestroyWorld(world);
    return same ? 0 : 1;
}
