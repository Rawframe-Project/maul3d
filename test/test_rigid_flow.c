// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The free rotation of a rigid body: the principal frame a mass update
// computes, and the splitting the position step turns a free body with,
// measured against Euler's equations integrated in double.

#include "helpers.h"
#include "rigid_flow.h"
#include "test_harness.h"
#include "world_internal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// The tensor R diag(moments) R^T of a frame and its moments, in double.
static void Rebuild(m3Vec3 moments, m3Quat frame, double out[3][3])
{
    m3Vec3 axes[3] = {m3RotateVec3(frame, (m3Vec3){1.0f, 0.0f, 0.0f}),
                      m3RotateVec3(frame, (m3Vec3){0.0f, 1.0f, 0.0f}),
                      m3RotateVec3(frame, (m3Vec3){0.0f, 0.0f, 1.0f})};
    const double m[3] = {(double)moments.x, (double)moments.y, (double)moments.z};
    for (int32_t i = 0; i < 3; ++i)
    {
        for (int32_t j = 0; j < 3; ++j)
        {
            double sum = 0.0;
            for (int32_t k = 0; k < 3; ++k)
            {
                const float* e = &axes[k].x;
                sum += (double)e[i] * m[k] * (double)e[j];
            }
            out[i][j] = sum;
        }
    }
}

// The largest difference between a tensor and the one its frame rebuilds,
// relative to the largest moment.
static double RebuildError(const m3Mat3* inertia, m3Vec3 moments, m3Quat frame)
{
    double rebuilt[3][3];
    Rebuild(moments, frame, rebuilt);
    const double in[3][3] = {
        {(double)inertia->cx.x, (double)inertia->cy.x, (double)inertia->cz.x},
        {(double)inertia->cx.y, (double)inertia->cy.y, (double)inertia->cz.y},
        {(double)inertia->cx.z, (double)inertia->cy.z, (double)inertia->cz.z},
    };
    double worst = 0.0;
    for (int32_t i = 0; i < 3; ++i)
    {
        for (int32_t j = 0; j < 3; ++j)
        {
            worst = fmax(worst, fabs(rebuilt[i][j] - in[i][j]));
        }
    }
    return worst / (double)moments.x;
}

// Each axis of the frame has its largest component positive, the first
// on a tie, except where the last axis was negated to make a rotation.
static bool AxesFollowSignRule(m3Quat frame)
{
    for (int32_t j = 0; j < 2; ++j)
    {
        m3Vec3 e = m3RotateVec3(frame, (m3Vec3){j == 0 ? 1.0f : 0.0f, j == 1 ? 1.0f : 0.0f, 0.0f});
        const float* c = &e.x;
        int32_t big = 0;
        for (int32_t k = 1; k < 3; ++k)
        {
            if (fabsf(c[k]) > fabsf(c[big]))
            {
                big = k;
            }
        }
        if (c[big] < 0.0f)
        {
            return false;
        }
    }
    return true;
}

// A diagonal tensor keeps its moments, sorted largest first, with the
// frame that permutes its axes; a sorted one keeps the identity frame.
static void TestPrincipalAxesOfDiagonal(void)
{
    m3Mat3 sorted = {{0.8f, 0.0f, 0.0f}, {0.0f, 0.6f, 0.0f}, {0.0f, 0.0f, 0.3f}};
    m3Vec3 moments;
    m3Quat frame;
    m3PrincipalAxes(&sorted, &moments, &frame);
    CHECK(moments.x == 0.8f && moments.y == 0.6f && moments.z == 0.3f,
          "a sorted diagonal tensor keeps its moments");
    CHECK(frame.x == 0.0f && frame.y == 0.0f && frame.z == 0.0f && frame.w == 1.0f,
          "a sorted diagonal tensor keeps the identity frame");

    m3Mat3 unsorted = {{0.3f, 0.0f, 0.0f}, {0.0f, 0.8f, 0.0f}, {0.0f, 0.0f, 0.6f}};
    m3PrincipalAxes(&unsorted, &moments, &frame);
    CHECK(moments.x == 0.8f && moments.y == 0.6f && moments.z == 0.3f,
          "an unsorted diagonal tensor's moments come out largest first");
    CHECK(RebuildError(&unsorted, moments, frame) <= 1.0e-6,
          "the permuting frame rebuilds the unsorted tensor");
    CHECK(frame.w >= 0.0f && AxesFollowSignRule(frame),
          "the permuting frame follows the sign rule");

    m3Mat3 zero = m3MakeZeroMat3();
    m3PrincipalAxes(&zero, &moments, &frame);
    CHECK(moments.x == 0.0f && moments.y == 0.0f && moments.z == 0.0f && frame.w == 1.0f,
          "a zero tensor has zero moments and the identity frame");
    CHECK(!m3FreeRotationApplies(moments), "a zero tensor takes no free rotation");
}

// A tensor turned off its axes: the moments come back, the frame rebuilds
// the tensor, and the frame is a rotation with its axes signed by the rule.
static void TestPrincipalAxesOfTurnedTensor(void)
{
    const m3Quat turns[3] = {
        {0.0f, 0.0f, 0.38268343f, 0.92387953f},
        {0.27059805f, 0.27059805f, 0.65328148f, 0.65328148f},
        {-0.5f, 0.5f, -0.5f, 0.5f},
    };
    const m3Vec3 moments = {3.0f, 2.0f, 0.5f};
    for (int32_t c = 0; c < 3; ++c)
    {
        double built[3][3];
        Rebuild(moments, turns[c], built);
        m3Mat3 inertia = {{(float)built[0][0], (float)built[1][0], (float)built[2][0]},
                          {(float)built[0][1], (float)built[1][1], (float)built[2][1]},
                          {(float)built[0][2], (float)built[1][2], (float)built[2][2]}};
        m3Vec3 found;
        m3Quat frame;
        m3PrincipalAxes(&inertia, &found, &frame);
        CHECK(fabsf(found.x - 3.0f) <= 1.0e-5f && fabsf(found.y - 2.0f) <= 1.0e-5f &&
                  fabsf(found.z - 0.5f) <= 1.0e-5f,
              "the moments of a turned tensor come back");
        CHECK(RebuildError(&inertia, found, frame) <= 1.0e-6, "the frame rebuilds a turned tensor");
        double norm = sqrt((double)frame.x * (double)frame.x + (double)frame.y * (double)frame.y +
                           (double)frame.z * (double)frame.z + (double)frame.w * (double)frame.w);
        CHECK(fabs(norm - 1.0) <= 1.0e-6 && frame.w >= 0.0f, "the frame is a unit quaternion");
        CHECK(AxesFollowSignRule(frame), "the frame's axes follow the sign rule");
    }
}

// A seeded run of random tensors, some with two nearly equal moments,
// which take Jacobi several sweeps: each rebuilds to float rounding, its
// moments come out sorted, and its frame is a rotation whose axes follow
// the sign rule.
static void TestPrincipalAxesOfRandomTensors(void)
{
    uint64_t state = 0x9E3779B97F4A7C15ull;
    double worstRebuild = 0.0;
    bool sorted = true;
    bool signs = true;
    bool rotations = true;
    for (int32_t c = 0; c < 200; ++c)
    {
        float u[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            u[k] = (float)((double)(state >> 40) / 16777216.0) * 2.0f - 1.0f;
        }
        m3Quat turn = m3NormalizeQuat((m3Quat){u[0], u[1], u[2], u[3] + 0.1f});
        float spread = c % 4 == 0 ? 1.0e-3f : 1.0f;
        m3Vec3 moments = {2.0f + spread, 2.0f, 0.5f + 0.4f * fabsf(u[0])};
        double built[3][3];
        Rebuild(moments, turn, built);
        m3Mat3 inertia = {{(float)built[0][0], (float)built[1][0], (float)built[2][0]},
                          {(float)built[0][1], (float)built[1][1], (float)built[2][1]},
                          {(float)built[0][2], (float)built[1][2], (float)built[2][2]}};
        m3Vec3 found;
        m3Quat frame;
        m3PrincipalAxes(&inertia, &found, &frame);
        worstRebuild = fmax(worstRebuild, RebuildError(&inertia, found, frame));
        sorted = sorted && found.x >= found.y && found.y >= found.z;
        signs = signs && AxesFollowSignRule(frame);
        m3Vec3 x = m3RotateVec3(frame, (m3Vec3){1.0f, 0.0f, 0.0f});
        m3Vec3 y = m3RotateVec3(frame, (m3Vec3){0.0f, 1.0f, 0.0f});
        m3Vec3 z = m3RotateVec3(frame, (m3Vec3){0.0f, 0.0f, 1.0f});
        rotations = rotations && m3Dot3(m3Cross3(x, y), z) > 0.99f;
    }
    printf("random tensors: worst rebuild %.3g\n", worstRebuild);
    CHECK(worstRebuild <= 5.0e-7, "every random tensor rebuilds from its frame");
    CHECK(sorted, "every random tensor's moments come out largest first");
    CHECK(signs, "every random frame follows the sign rule");
    CHECK(rotations, "every random frame is a rotation");
}

// Euler's equations for a free body in its principal frame, dL/dt =
// L x (L / I), integrated with RK4 at a step far below the world's.
static void EulerRk4(const double moments[3], double L[3], double h)
{
    double k[4][3];
    double at[3];
    for (int32_t stage = 0; stage < 4; ++stage)
    {
        double scale = stage == 0 ? 0.0 : (stage == 3 ? h : 0.5 * h);
        for (int32_t i = 0; i < 3; ++i)
        {
            at[i] = L[i] + (stage == 0 ? 0.0 : scale * k[stage - 1][i]);
        }
        double w[3] = {at[0] / moments[0], at[1] / moments[1], at[2] / moments[2]};
        k[stage][0] = at[1] * w[2] - at[2] * w[1];
        k[stage][1] = at[2] * w[0] - at[0] * w[2];
        k[stage][2] = at[0] * w[1] - at[1] * w[0];
    }
    for (int32_t i = 0; i < 3; ++i)
    {
        L[i] += h / 6.0 * (k[0][i] + 2.0 * k[1][i] + 2.0 * k[2][i] + k[3][i]);
    }
}

// A box of half extents 0.25, 0.5 and 0.75 m, whose long axis has the
// smallest moment, tumbles at 20 rad/s for 10 s at four substeps, spun
// near each principal axis, the unstable middle one included. Its world
// angular momentum stays, its energy holds, and the long axis's share of
// its momentum stays on the orbit Euler's equations give.
static void TestTriaxialBoxFollowsEuler(void)
{
    const double half[3] = {0.25, 0.5, 0.75};
    const double dirs[3][3] = {{1.0, 0.3, 0.2}, {0.2, 1.0, 0.3}, {0.3, 0.2, 1.0}};
    for (int32_t d = 0; d < 3; ++d)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        m3World* world = NewWorld(&def);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.angularDamping = 0.0f;
        bd.enableSleep = false;
        m3BodyId body = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        NewBoxShape(world, body, &sd, (m3Vec3){(float)half[0], (float)half[1], (float)half[2]});
        double mass = 8.0 * half[0] * half[1] * half[2] * (double)sd.density;
        const double moments[3] = {mass * (half[1] * half[1] + half[2] * half[2]) / 3.0,
                                   mass * (half[0] * half[0] + half[2] * half[2]) / 3.0,
                                   mass * (half[0] * half[0] + half[1] * half[1]) / 3.0};
        double n =
            sqrt(dirs[d][0] * dirs[d][0] + dirs[d][1] * dirs[d][1] + dirs[d][2] * dirs[d][2]);
        m3Vec3 w = {(float)(20.0 * dirs[d][0] / n), (float)(20.0 * dirs[d][1] / n),
                    (float)(20.0 * dirs[d][2] / n)};
        m3Body_SetAngularVelocity(world, body, w);
        double reference[3] = {moments[0] * (double)w.x, moments[1] * (double)w.y,
                               moments[2] * (double)w.z};
        const double start[3] = {reference[0], reference[1], reference[2]};
        double startSize = sqrt(start[0] * start[0] + start[1] * start[1] + start[2] * start[2]);
        double startEnergy =
            (double)w.x * reference[0] + (double)w.y * reference[1] + (double)w.z * reference[2];
        double worstMomentum = 0.0;
        double worstEnergy = 0.0;
        for (int32_t i = 0; i < 600; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            for (int32_t r = 0; r < 256; ++r)
            {
                EulerRk4(moments, reference, 1.0 / 60.0 / 256.0);
            }
            m3Quat q = m3Body_GetRotation(world, body);
            m3Vec3 b = m3InvRotateVec3(q, m3Body_GetAngularVelocity(world, body));
            m3Vec3 lb = {(float)(moments[0] * (double)b.x), (float)(moments[1] * (double)b.y),
                         (float)(moments[2] * (double)b.z)};
            m3Vec3 lw = m3RotateVec3(q, lb);
            double dx = (double)lw.x - start[0];
            double dy = (double)lw.y - start[1];
            double dz = (double)lw.z - start[2];
            worstMomentum = fmax(worstMomentum, sqrt(dx * dx + dy * dy + dz * dz) / startSize);
            double energy = (double)b.x * (double)lb.x + (double)b.y * (double)lb.y +
                            (double)b.z * (double)lb.z;
            worstEnergy = fmax(worstEnergy, fabs(energy / startEnergy - 1.0));
            if (i == 599)
            {
                double share = fabs((double)lb.z) / startSize;
                double exact = fabs(reference[2]) / startSize;
                printf("triaxial %d: momentum %.3g energy %.3g long-axis share %.4f exact %.4f\n",
                       d, worstMomentum, worstEnergy, share, exact);
                CHECK(fabs(share - exact) <= 0.005, "the long axis's share stays on Euler's orbit");
            }
        }
        CHECK(worstMomentum <= 5.0e-4, "a tumbling box keeps its world angular momentum");
        CHECK(worstEnergy <= 1.0e-4, "a tumbling box keeps its energy");
        m3DestroyWorld(world);
    }
}

// A stick laid along a diagonal of two body axes has equal moments about
// them and a product of inertia, so only its principal frame shows it is
// a symmetric top. Spun 45 degrees off its axis it keeps its energy as a
// stick along a body axis does.
static void TestTiltedStickIsSymmetricTop(void)
{
    for (int32_t pair = 0; pair < 3; ++pair)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        m3World* world = NewWorld(&def);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.angularDamping = 0.0f;
        bd.enableSleep = false;
        m3BodyId body = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        float end[3] = {0.0f, 0.0f, 0.0f};
        end[pair] = 0.70710678f;
        end[(pair + 1) % 3] = 0.70710678f;
        m3Capsule capsule = {{-end[0], -end[1], -end[2]}, {end[0], end[1], end[2]}, 0.05f};
        NewCapsuleShape(world, body, &sd, &capsule);
        int32_t slot = (int32_t)body.index1 - 1;
        const m3Mat3* inertia = &world->bodies.inertiaLocal[slot];
        const float products[3] = {inertia->cy.x, inertia->cz.y, inertia->cz.x};
        CHECK(products[pair] != 0.0f, "the tilted stick has a product of inertia");
        // The stick's axis, and a direction across it.
        m3Vec3 axis = {end[0] * 1.41421356f, end[1] * 1.41421356f, end[2] * 1.41421356f};
        float across[3] = {0.0f, 0.0f, 0.0f};
        across[(pair + 2) % 3] = 1.0f;
        m3Vec3 w = m3Add3(m3MulSV3(7.0710678f, axis),
                          m3MulSV3(7.0710678f, (m3Vec3){across[0], across[1], across[2]}));
        m3Body_SetAngularVelocity(world, body, w);
        m3Mat3 tensor = *inertia;
        double startEnergy = (double)m3Dot3(w, m3MulMV3(tensor, w));
        double worst = 0.0;
        for (int32_t i = 0; i < 600; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            m3Vec3 b = m3InvRotateVec3(m3Body_GetRotation(world, body),
                                       m3Body_GetAngularVelocity(world, body));
            double energy = (double)m3Dot3(b, m3MulMV3(tensor, b));
            worst = fmax(worst, fabs(energy / startEnergy - 1.0));
        }
        printf("tilted stick %d: energy %.3g\n", pair, worst);
        CHECK(worst <= 1.0e-4, "a tilted stick keeps its energy");
        m3DestroyWorld(world);
    }
}

// A thin rod's momentum about its own axis is its spin times its smallest
// moment, far below the rounding of its spin across the axis at the large
// ones. A symmetric top keeps its spin about its axis, so the rod does at
// every moment ratio from 1e-1 down to 1e-12, spun along its axis or off
// it, over 10 s at four substeps of 60 Hz.
static void TestThinRodKeepsAxialSpin(void)
{
    const float tilts[2] = {0.0f, 0.5f};
    for (int32_t t = 0; t < 2; ++t)
    {
        double worst = 0.0;
        double worstRatio = 0.0;
        for (int32_t e = 0; e <= 220; ++e)
        {
            double ratio = pow(10.0, -1.0 - 0.05 * (double)e);
            const m3Vec3 moments = {1.0f, 1.0f, (float)ratio};
            // The principal frame lays the rod's axis, the smallest moment's,
            // along world x.
            const m3Quat frame = {0.0f, 0.70710678f, 0.0f, 0.70710678f};
            m3Quat q = m3MakeIdentityQuat();
            m3Vec3 w = {3.0f, tilts[t], 0.0f};
            for (int32_t i = 0; i < 2400; ++i)
            {
                m3FreeRotation(moments, frame, q, w, 1.0f / 240.0f, &q, &w);
            }
            m3Vec3 axis = m3RotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f});
            double axial = (double)w.x * (double)axis.x + (double)w.y * (double)axis.y +
                           (double)w.z * (double)axis.z;
            double error = fabs(axial / 3.0 - 1.0);
            if (!(error <= worst))
            {
                worst = error;
                worstRatio = ratio;
            }
        }
        printf("thin rod, tilt %g: axial spin %.3g at ratio %.3g\n", (double)tilts[t], worst,
               worstRatio);
        CHECK(worst <= 1.0e-4, "a thin rod keeps its spin about its axis");
    }
    // A 2 m rod of radius 1.2 cm, moments about 1.5e-4 apart, in a world.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.angularDamping = 0.0f;
    bd.enableSleep = false;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule rod = {{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 0.012f};
    NewCapsuleShape(world, body, &sd, &rod);
    m3Body_SetAngularVelocity(world, body, (m3Vec3){3.0f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 600; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 axis = m3RotateVec3(m3Body_GetRotation(world, body), (m3Vec3){1.0f, 0.0f, 0.0f});
    double axial = (double)m3Dot3(m3Body_GetAngularVelocity(world, body), axis);
    printf("thin rod in a world: axial spin %.6f\n", axial);
    CHECK(fabs(axial / 3.0 - 1.0) <= 1.0e-4, "a thin rod in a world keeps its spin about its axis");
    m3DestroyWorld(world);
}

// A body whose rotation is locked about two axes turns at its spin about
// the third, as a planar body does, rather than precessing as a free one
// would: a box tilted off its principal axes, locked to turn about world
// z only, keeps its spin bit for bit.
static void TestLockedBodyTurnsAtItsSpin(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.angularDamping = 0.0f;
    bd.enableSleep = false;
    bd.rotation = m3NormalizeQuat((m3Quat){0.25881905f, 0.0f, 0.0f, 0.96592583f});
    bd.motionLocks.angularX = true;
    bd.motionLocks.angularY = true;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, body, &sd, (m3Vec3){0.25f, 0.5f, 0.75f});
    const m3Vec3 w = {0.0f, 0.0f, 9.0f};
    m3Body_SetAngularVelocity(world, body, w);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 after = m3Body_GetAngularVelocity(world, body);
    CHECK(memcmp(&after, &w, sizeof(w)) == 0, "a locked body keeps its spin bit for bit");
    m3DestroyWorld(world);
}

// A sphere's spin is constant, and the step leaves it bit for bit while
// the sphere turns about it.
static void TestSphereKeepsSpin(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.angularDamping = 0.0f;
    bd.enableSleep = false;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, body, &sd, &ball);
    const m3Vec3 w = {3.0f, -11.0f, 7.0f};
    m3Body_SetAngularVelocity(world, body, w);
    m3Quat start = m3Body_GetRotation(world, body);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 after = m3Body_GetAngularVelocity(world, body);
    CHECK(memcmp(&after, &w, sizeof(w)) == 0, "a sphere keeps its spin bit for bit");
    m3Quat q = m3Body_GetRotation(world, body);
    CHECK(fabsf(q.w - start.w) > 0.1f, "the sphere turns");
    m3DestroyWorld(world);
}

int main(void)
{
    TestPrincipalAxesOfDiagonal();
    TestPrincipalAxesOfTurnedTensor();
    TestPrincipalAxesOfRandomTensors();
    TestTriaxialBoxFollowsEuler();
    TestTiltedStickIsSymmetricTop();
    TestThinRodKeepsAxialSpin();
    TestLockedBodyTurnsAtItsSpin();
    TestSphereKeepsSpin();
    if (s_failures == 0)
    {
        printf("test_rigid_flow: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
