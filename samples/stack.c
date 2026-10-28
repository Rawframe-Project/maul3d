// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ten boxes fall onto a plane and settle. The printed world hash is
// the same on every machine and architecture.

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
        fprintf(stderr, "%s: %s\n", what, m3ResultName(result));
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

    for (int i = 0; i < 10; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 1.0 + 1.05 * i, 0.0};
        m3BodyId box;
        Require(m3CreateBody(world, &bd, &box), "m3CreateBody");
        Require(m3CreateBoxShape(world, box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f}, &shape),
                "m3CreateBoxShape");
    }

    for (int i = 0; i < 300; ++i)
    {
        Require(m3World_Step(world, 1.0f / 60.0f, 4), "step");
    }
    printf("world hash %016llx\n", (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
    return 0;
}
