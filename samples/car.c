// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A raycast car drives straight for two seconds, then the same chassis
// runs as a tank and pivots in place on opposite track commands.
// Commands are journaled state, so a recorded drive replays bit-exact.

#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/vehicle.h"
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

static void Drive(m3World* world, int steps)
{
    for (int i = 0; i < steps; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
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
    bd.position = (m3Pos3){0.0, 0.65, 0.0};
    m3BodyId chassis;
    Require(m3CreateBody(world, &bd, &chassis), "m3CreateBody");
    m3ShapeDef cs = m3DefaultShapeDef();
    cs.density = 300.0f;
    Require(m3CreateBoxShape(world, chassis, &cs, (m3Vec3){1.0f, 0.25f, 0.5f}, &shape),
            "m3CreateBoxShape");

    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 4;
    for (int w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor = (m3Vec3){(w & 1) ? 0.8f : -0.8f, -0.25f, (w & 2) ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
        vd.wheels[w].steerable = (w & 1) != 0;
    }
    m3VehicleId car;
    Require(m3CreateVehicle(world, &vd, &car), "m3CreateVehicle");

    m3Vehicle_SetCommands(world, car, 1.0f, 0.0f, 0.0f); // throttle, steer, brake
    Drive(world, 120);
    m3Pos3 p = m3Body_GetPosition(world, chassis);
    printf("car after 2 s: x = %.3f m\n", p.x);

    m3Vehicle_SetCommands(world, car, 0.0f, 0.0f, 1.0f); // stop before pivoting
    Drive(world, 120);
    m3Vehicle_SetTankCommands(world, car, 1.0f, -1.0f, 0.0f); // left, right, brake
    Drive(world, 60);
    printf("world hash %016llx\n", (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
    return 0;
}
