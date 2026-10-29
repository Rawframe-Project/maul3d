// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A character walks into a flight of stairs and climbs it. Gravity is
// the host's job: each tick moves the capsule forward and down, and the
// controller collides, slides and steps up the risers.

#include "maul3d/character.h"

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

    // Five 0.2 m risers, 0.5 m deep, starting at x = 2.
    for (int s = 0; s < 5; ++s)
    {
        m3BodyDef stepDef = m3DefaultBodyDef();
        stepDef.position = (m3Pos3){2.25 + 0.5 * s + 1.5, 0.1 + 0.1 * s, 0.0};
        m3BodyId step;
        Require(m3CreateBody(world, &stepDef, &step), "m3CreateBody");
        Require(m3CreateBoxShape(world, step, &sd,
                                 (m3Vec3){0.25f + 1.5f, 0.1f + 0.1f * (float)s, 1.0f}, &shape),
                "m3CreateBoxShape");
    }

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3CharacterId hero;
    Require(m3CreateCharacter(world, &cd, &hero), "m3CreateCharacter");

    for (int i = 0; i < 240; ++i)
    {
        m3Character_Move(world, hero, (m3Vec3){0.04f, -0.04f, 0.0f});
        Require(m3World_Step(world, 1.0f / 60.0f, 4), "step");
    }
    m3Pos3 p = m3Character_GetPosition(world, hero);
    printf("character at x = %.3f, y = %.3f, grounded: %s\n", p.x, p.y,
           m3Character_IsGrounded(world, hero) ? "yes" : "no");
    printf("world hash %016llx\n", (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
    return 0;
}
