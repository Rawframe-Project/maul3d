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

int main(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(m3CreateBody(world, &gd), &sd, &floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3CreateSphereShape(ball, &sd, &sphere);

    int32_t size = m3World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    if (snap == NULL || m3World_Snapshot(world, snap, size) != size)
    {
        return 1;
    }
    uint64_t then = m3World_Hash(world);

    // The mispredicted future: the ball is kicked sideways.
    m3Body_SetLinearVelocity(ball, (m3Vec3){5.0f, 0.0f, 0.0f});
    for (int i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }

    // The corrected input arrives: rewind and simulate again.
    m3World_Restore(world, snap, size);
    int same = m3World_Hash(world) == then;
    for (int i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    printf("restored bit-exact: %s, world hash %016llx\n", same ? "yes" : "no",
           (unsigned long long)m3World_Hash(world));
    free(snap);
    m3DestroyWorld(world);
    return same ? 0 : 1;
}
