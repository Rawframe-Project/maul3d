// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soft-vs-soft gate: two lattices refuse to share
// space, and a rope drapes over a cloth instead of falling through.

#include "test_harness.h"

#include "maul3d/shape.h"
#include "maul3d/softbody.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3WorldId PlaneWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.softBodyCapacity = 4;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);
    return world;
}

static double MinPairDistance(m3SoftBodyId a, m3SoftBodyId b)
{
    int32_t na = m3SoftBody_GetParticleCount(a);
    int32_t nb = m3SoftBody_GetParticleCount(b);
    double best = 1.0e30;
    for (int32_t i = 0; i < na; ++i)
    {
        m3Pos3 pa = m3SoftBody_GetParticlePosition(a, i);
        for (int32_t j = 0; j < nb; ++j)
        {
            m3Pos3 pb = m3SoftBody_GetParticlePosition(b, j);
            double dx = pa.x - pb.x;
            double dy = pa.y - pb.y;
            double dz = pa.z - pb.z;
            double d = sqrt(dx * dx + dy * dy + dz * dz);
            if (d < best)
            {
                best = d;
            }
        }
    }
    return best;
}

static void TestJelliesRefuseToMerge(void)
{
    // One jelly dropped onto another: they stack, and no particle
    // pair ends closer than the contact target minus half the
    // smaller radius (the overlap law from the plan).
    m3WorldId world = PlaneWorld();
    m3SoftBodyDef sd = m3DefaultSoftBodyDef();
    sd.countX = 4;
    sd.countY = 4;
    sd.countZ = 4;
    sd.spacing = 0.3f;
    sd.radius = 0.12f;
    sd.position = (m3Pos3){0.0, 0.15, 0.0};
    m3SoftBodyId lower = m3CreateSoftBody(world, &sd);
    // Dead-center drop from a modest height: the first draft's
    // offset landing SLID OFF sideways (correct physics, wrong
    // premise for a stacking proof).
    sd.position = (m3Pos3){0.0, 1.4, 0.0};
    m3SoftBodyId upper = m3CreateSoftBody(world, &sd);
    CHECK(m3SoftBody_IsValid(lower) && m3SoftBody_IsValid(upper), "both jellies create");
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double gap = MinPairDistance(lower, upper);
    CHECK(gap > 0.24 - 0.06, "no particle pair interpenetrates past the law");
    // And the upper jelly actually RESTS on the lower one, well
    // above where it would sit had it fallen through to the plane.
    double lowY = 1.0e30;
    for (int32_t i = 0; i < m3SoftBody_GetParticleCount(upper); ++i)
    {
        double y = m3SoftBody_GetParticlePosition(upper, i).y;
        if (y < lowY)
        {
            lowY = y;
        }
    }
    // Resting ON the lower jelly means the upper one's floor sits
    // far above the ground plane's particle-radius shelf.
    CHECK(lowY > 0.6, "the upper jelly stacks instead of merging or sliding off");
    m3DestroyWorld(world);
}

static void TestRopeDrapesOverCloth(void)
{
    // A cloth pinned at its four corners, a rope dropped across:
    // the rope must NOT fall through the sheet.
    m3WorldId world = PlaneWorld();
    m3SoftBodyDef cd = m3DefaultSoftBodyDef();
    cd.countX = 8;
    cd.countY = 1;
    cd.countZ = 8;
    cd.spacing = 0.25f;
    cd.radius = 0.1f;
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3SoftBodyId cloth = m3CreateSoftBody(world, &cd);
    m3SoftBody_PinParticle(cloth, 0);
    m3SoftBody_PinParticle(cloth, 7);
    m3SoftBody_PinParticle(cloth, 56);
    m3SoftBody_PinParticle(cloth, 63);

    m3SoftBodyDef rd = m3DefaultSoftBodyDef();
    rd.countX = 10;
    rd.countY = 1;
    rd.countZ = 1;
    rd.spacing = 0.22f;
    rd.radius = 0.09f;
    rd.position = (m3Pos3){-0.2, 3.0, 0.875};
    m3SoftBodyId rope = m3CreateSoftBody(world, &rd);

    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double ropeLow = 1.0e30;
    for (int32_t i = 0; i < m3SoftBody_GetParticleCount(rope); ++i)
    {
        double y = m3SoftBody_GetParticlePosition(rope, i).y;
        if (y < ropeLow)
        {
            ropeLow = y;
        }
    }
    CHECK(ropeLow > 1.2, "the rope drapes over the cloth instead of falling through");
    m3DestroyWorld(world);
}

static void TestRopeBridge(void)
{
    // The deep dive's showcase: a rope anchored between two pinned
    // cloths carries a dropped rigid ball. Soft-vs-soft contact,
    // soft-to-soft anchors, and rigid coupling in one scene.
    m3WorldId world = PlaneWorld();
    m3SoftBodyDef cd = m3DefaultSoftBodyDef();
    cd.countX = 5;
    cd.countY = 1;
    cd.countZ = 5;
    cd.spacing = 0.25f;
    cd.radius = 0.1f;
    cd.position = (m3Pos3){-2.2, 2.0, -0.5};
    m3SoftBodyId west = m3CreateSoftBody(world, &cd);
    cd.position = (m3Pos3){1.2, 2.0, -0.5};
    m3SoftBodyId east = m3CreateSoftBody(world, &cd);
    for (int32_t c = 0; c < 25; c += 4)
    {
        m3SoftBody_PinParticle(west, c);
        m3SoftBody_PinParticle(east, c);
    }

    m3SoftBodyDef rd = m3DefaultSoftBodyDef();
    rd.countX = 9;
    rd.countY = 1;
    rd.countZ = 1;
    rd.spacing = 0.25f;
    rd.radius = 0.09f;
    rd.particleMass = 0.3f;
    rd.position = (m3Pos3){-1.1, 2.05, 0.0};
    m3SoftBodyId rope = m3CreateSoftBody(world, &rd);
    // Tie the rope's ends to the inner edges of the two cloths.
    m3SoftBody_AnchorToSoft(rope, 0, west, 12);
    m3SoftBody_AnchorToSoft(rope, 8, east, 10);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.2, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 0.25f; // light enough for a soft bridge
    m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.18f};
    m3CreateSphereShape(ball, &sd, &s);

    for (int32_t i = 0; i < 360; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double bally = m3Body_GetPosition(ball).y;
    CHECK(bally > 0.6, "the bridge carries the ball above the floor");
    m3Pos3 r0 = m3SoftBody_GetParticlePosition(rope, 0);
    m3Pos3 r8 = m3SoftBody_GetParticlePosition(rope, 8);
    m3Pos3 w12 = m3SoftBody_GetParticlePosition(west, 12);
    m3Pos3 e10 = m3SoftBody_GetParticlePosition(east, 10);
    double dw = sqrt((r0.x - w12.x) * (r0.x - w12.x) + (r0.y - w12.y) * (r0.y - w12.y) +
                     (r0.z - w12.z) * (r0.z - w12.z));
    double de = sqrt((r8.x - e10.x) * (r8.x - e10.x) + (r8.y - e10.y) * (r8.y - e10.y) +
                     (r8.z - e10.z) * (r8.z - e10.z));
    CHECK(dw < 0.05 && de < 0.05, "the anchors hold the rope to both cloths");
    m3DestroyWorld(world);
}

int main(void)
{
    TestJelliesRefuseToMerge();
    TestRopeDrapesOverCloth();
    TestRopeBridge();
    if (s_failures == 0)
    {
        printf("test_softsoft: all green\n");
        return 0;
    }
    printf("test_softsoft: %d failure(s)\n", s_failures);
    return 1;
}
