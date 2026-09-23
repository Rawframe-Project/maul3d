// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ten boxes fall onto a plane and settle. The printed world hash is
// the same on every machine and architecture.

#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#include <stdio.h>

int main(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3WorldId world = m3CreateWorld(&wd);

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    for (int i = 0; i < 10; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 1.0 + 1.05 * i, 0.0};
        m3BodyId box = m3CreateBody(world, &bd);
        m3CreateBoxShape(box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    }

    for (int i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    printf("world hash %016llx\n", (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
    return 0;
}
