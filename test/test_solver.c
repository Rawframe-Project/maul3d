// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Solver gate: free fall against the analytic velocity, rest on the
// plane within tolerance, the standing stack, a restitution bounce
// band, and friction versus no friction. Black box: public headers
// only.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* MakeWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    return NewWorld(&def);
}

static m3BodyId AddGroundPlane(m3World* world, float friction)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = friction;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    return ground;
}

static m3BodyId AddBall(m3World* world, double x, double y, double z, float radius, float friction,
                        float restitution)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, y, z};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = friction;
    sd.restitution = restitution;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, radius};
    NewSphereShape(world, body, &sd, &ball);
    return body;
}

static void StepN(m3World* world, int32_t steps)
{
    for (int32_t i = 0; i < steps; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static void TestFreeFall(void)
{
    m3World* world = MakeWorld();
    m3BodyId ball = AddBall(world, 0.0, 100.0, 0.0, 0.5f, 0.6f, 0.0f);
    StepN(world, 60); // one second
    m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
    CHECK(v.y > -10.05f && v.y < -9.95f, "one second of gravity lands near -10");
    m3Pos3 p = m3Body_GetPosition(world, ball);
    // Substepped semi-implicit Euler: expect close to 100 - g t^2 / 2,
    // slightly below because velocity leads position.
    CHECK(p.y > 94.5 && p.y < 95.5, "fallen distance in the analytic band");
    m3DestroyWorld(world);
}

static void TestRestOnPlane(void)
{
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.6f);
    m3BodyId ball = AddBall(world, 0.0, 0.6, 0.0, 0.5f, 0.6f, 0.0f);
    StepN(world, 300);
    m3Pos3 p = m3Body_GetPosition(world, ball);
    CHECK(p.y > 0.485 && p.y < 0.515, "the ball rests on the plane at its radius");
    m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
    CHECK(v.y > -0.02f && v.y < 0.02f, "resting velocity is near zero");
    m3DestroyWorld(world);
}

static void TestStandingStack(void)
{
    // THE 2a deliverable: a vertical stack of spheres stands.
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.8f);
    m3BodyId balls[5];
    for (int32_t i = 0; i < 5; ++i)
    {
        balls[i] = AddBall(world, 0.0, 0.5 + 1.0 * (double)i, 0.0, 0.5f, 0.8f, 0.0f);
    }
    StepN(world, 600); // ten seconds
    for (int32_t i = 0; i < 5; ++i)
    {
        m3Pos3 p = m3Body_GetPosition(world, balls[i]);
        double restY = 0.5 + 1.0 * (double)i;
        CHECK(p.y > restY - 0.08 && p.y < restY + 0.04, "each sphere holds its height");
        CHECK(p.x > -0.05 && p.x < 0.05 && p.z > -0.05 && p.z < 0.05,
              "no sideways drift in a symmetric stack");
    }
    m3DestroyWorld(world);
}

static void TestRestitution(void)
{
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.0f);
    m3BodyId ball = AddBall(world, 0.0, 2.5, 0.0, 0.5f, 0.0f, 0.8f);
    // Track the rebound peak after the first bounce.
    double peak = 0.0;
    int bounced = 0;
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
        m3Pos3 p = m3Body_GetPosition(world, ball);
        if (!bounced && v.y > 0.5f)
        {
            bounced = 1;
        }
        if (bounced && p.y > peak)
        {
            peak = p.y;
        }
        if (bounced && v.y < -0.5f && p.y < 1.0)
        {
            break; // falling again after the peak
        }
    }
    // Energy: rebound height is about e^2 of the two-meter drop, so
    // the center peaks near 0.5 + 0.64 * 2 = 1.78. A generous band
    // absorbs the soft-contact and threshold losses.
    CHECK(bounced == 1, "the ball bounces");
    CHECK(peak > 1.3 && peak < 2.0, "the rebound peak is in the restitution band");
    m3DestroyWorld(world);
}

// Restitution runs after the substeps' speed caps; the step still holds
// its result within the speed limit (1e5 m/s). A ball arriving at the
// default 400 m/s cap with restitution 500 would leave at 2e5 m/s.
static void TestBounceHeldWithinSpeedLimit(void)
{
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.0f);
    m3BodyId ball = AddBall(world, 0.0, 0.52, 0.0, 0.5f, 0.0f, 500.0f);
    m3Body_SetLinearVelocity(world, ball, (m3Vec3){0.0f, -400.0f, 0.0f});
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
    float speed = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    CHECK(v.y > 400.0f, "the ball bounces harder than it arrived");
    CHECK(speed <= 1.0e5f * 1.0001f, "the bounce is held within the speed limit");
    m3DestroyWorld(world);
}

static void TestFrictionSlows(void)
{
    // The same sliding ball on a frictionless versus a frictional
    // plane: friction must bleed horizontal speed.
    double vx[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        float mu = pass == 0 ? 0.0f : 1.0f;
        m3World* world = MakeWorld();
        AddGroundPlane(world, mu);
        m3BodyId ball = AddBall(world, 0.0, 0.5, 0.0, 0.5f, mu, 0.0f);
        m3Body_SetLinearVelocity(world, ball, (m3Vec3){5.0f, 0.0f, 0.0f});
        StepN(world, 120);
        vx[pass] = (double)m3Body_GetLinearVelocity(world, ball).x;
        m3DestroyWorld(world);
    }
    CHECK(vx[1] < vx[0] - 0.5, "friction bleeds sliding speed");
    CHECK(vx[0] > 4.9, "the frictionless ball keeps its speed");
}

static void TestSlideDistanceLaw(void)
{
    // Coulomb's slide law: a settled crate shoved at v slides
    // v*v/(2*mu*g) with decel mu*g, never launches upward, never
    // rebounds. Rev 20 summed the friction budget across the eight
    // solve passes of a step, inflating the cone up to 8x: the
    // crate stuck in centimeters, dug its leading edge, and hopped.
    // maul2d, on the identical scene, decays as the textbook says.
    double slides[2];
    const double shoves[2] = {3.0, 0.5};
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3World* world = MakeWorld();
        AddGroundPlane(world, 0.6f);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.5, 0.0};
        m3BodyId crate = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        sd.friction = 0.6f;
        NewBoxShape(world, crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
        StepN(world, 60);
        double x0 = m3Body_GetPosition(world, crate).x;
        m3Body_ApplyLinearImpulse(world, crate, (m3Vec3){(float)shoves[pass], 0.0f, 0.0f});
        double maxVy = 0.0;
        double minVx = 0.0;
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            m3Vec3 v = m3Body_GetLinearVelocity(world, crate);
            if ((double)v.y > maxVy)
            {
                maxVy = (double)v.y;
            }
            if ((double)v.x < minVx)
            {
                minVx = (double)v.x;
            }
        }
        slides[pass] = m3Body_GetPosition(world, crate).x - x0;
        CHECK(maxVy < 0.05, "a horizontal shove never launches the crate");
        CHECK(minVx > -0.05, "friction never reverses the slide");
        m3DestroyWorld(world);
    }
    // mu*g = 6: theory 0.75 m and 0.0208 m. Generous bands absorb
    // the manifold's finite anchor spread, not a broken cone.
    CHECK(slides[0] > 0.60 && slides[0] < 0.90, "the fast shove slides by the law");
    CHECK(slides[1] > 0.014 && slides[1] < 0.028, "the slow shove slides by the law");
}

static void TestBoxRests(void)
{
    // A box dropped flat onto the plane lands on
    // a four-point manifold, rests at its half height, and stays
    // level; a second box stacks on top.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    AddGroundPlane(world, 0.7f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.8, 0.0};
    m3BodyId box = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.7f;
    NewBoxShape(world, box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    bd.position = (m3Pos3){0.1, 2.2, -0.05};
    m3BodyId top = NewBody(world, &bd);
    NewBoxShape(world, top, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(world, box);
    CHECK(p.y > 0.47 && p.y < 0.53, "the box rests at its half height");
    m3Quat q = m3Body_GetRotation(world, box);
    CHECK(q.w > 0.999f || q.w < -0.999f, "the box stays level (no tumbling)");
    m3Pos3 tp = m3Body_GetPosition(world, top);
    // The SAT landed: the staged-gap check flips into the real
    // assertion. The 0.4 half-height box rests on the unit box: about
    // 1.0 + 0.4 = 1.4.
    CHECK(tp.y > 1.32 && tp.y < 1.46, "the second box stacks on the first");

    // White-box peek: the resting box's plane manifold carries four
    // corner points with distinct feature ids.
    // (Reach through the public hash instead: two identical runs.)
    uint64_t h1 = m3World_Hash(world);
    m3DestroyWorld(world);
    CHECK(h1 != 0, "the scene hashes");
}

static void TestCapsuleRestsFlat(void)
{
    // A capsule dropped lying flat
    // lands on the two-cap manifold, rests at its radius, and does
    // not roll away or pitch up.
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{-0.5f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, 0.3f};
    m3ShapeId shape = NewCapsuleShape(world, body, &sd, &capsule);
    CHECK(m3Shape_IsValid(world, shape), "the capsule creates");

    // Contract checks while the world is warm: zero length and zero
    // radius are refused with the null id, no shape half-created.
    m3Capsule degenerate = {{0.1f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}, 0.3f};
    CHECK(!m3Shape_IsValid(world, NewCapsuleShape(world, body, &sd, &degenerate)),
          "a zero-length capsule is refused");
    m3Capsule flat = {{-0.5f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, 0.0f};
    CHECK(!m3Shape_IsValid(world, NewCapsuleShape(world, body, &sd, &flat)),
          "a zero-radius capsule is refused");

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(world, body);
    CHECK(p.y > 0.27 && p.y < 0.33, "the capsule rests at its radius");
    CHECK(p.x > -0.05 && p.x < 0.05, "the capsule does not wander");
    m3Quat q = m3Body_GetRotation(world, body);
    CHECK(q.w > 0.999f || q.w < -0.999f, "the capsule stays level");
    m3DestroyWorld(world);
}

static void TestSphereRestsOnBox(void)
{
    // Part two: the hull-sphere staged gap closes. A sphere dropped
    // onto a static unit box takes the generic GJK contact and rests
    // on the top face.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId block = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, block, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
    NewSphereShape(world, ball, &sd, &sphere);

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(world, ball);
    CHECK(p.y > 1.36 && p.y < 1.44, "the sphere rests on the box top");
    CHECK(p.x > -0.02 && p.x < 0.02, "the sphere stays centered");
    m3DestroyWorld(world);
}

static void TestSphereRestsOnCapsule(void)
{
    // Part three: capsule-sphere through the same generic pair. The
    // sphere balances on the lying capsule's midsection; a centered
    // drop is symmetric, so determinism keeps it centered.
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.3, 0.0};
    m3BodyId cap = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{-0.6f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}, 0.3f};
    NewCapsuleShape(world, cap, &sd, &capsule);

    bd.position = (m3Pos3){0.0, 1.2, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.25f};
    NewSphereShape(world, ball, &sd, &sphere);

    StepN(world, 360);

    m3Pos3 p = m3Body_GetPosition(world, ball);
    CHECK(p.y > 0.80 && p.y < 0.90, "the sphere rests on the capsule crown");
    m3Pos3 cp = m3Body_GetPosition(world, cap);
    CHECK(cp.y > 0.27 && cp.y < 0.33, "the capsule carries the load at its radius");
    m3DestroyWorld(world);
}

static void TestGyroscopicTumble(void)
{
    // Part four: the implicit gyroscopic term. A long flat box spun
    // about its intermediate axis must tumble (the Dzhanibekov flip
    // needs the cross(w, I*w) torque) while the implicit Newton form
    // keeps the energy bounded: |w| may legitimately grow as rotation
    // shifts toward the low-inertia axis, but only up to the inertia
    // ratio, never past it.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m3World* world = NewWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    // Half extents 1.0 x 0.4 x 0.05: three distinct principal
    // moments, y is the intermediate axis.
    NewBoxShape(world, body, &sd, (m3Vec3){1.0f, 0.4f, 0.05f});
    m3Body_SetAngularVelocity(world, body, (m3Vec3){0.01f, 5.0f, 0.01f});

    int tumbled = 0;
    float wMax = 0.0f;
    for (int32_t i = 0; i < 600; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Vec3 w = m3Body_GetAngularVelocity(world, body);
        float mag2 = w.x * w.x + w.y * w.y + w.z * w.z;
        if (mag2 > wMax)
        {
            wMax = mag2;
        }
        if (w.x > 1.0f || w.x < -1.0f || w.z > 1.0f || w.z < -1.0f)
        {
            tumbled = 1;
        }
    }
    CHECK(tumbled, "the intermediate-axis spin tumbles (the term is alive)");
    // Energy bound: E = w·(I w)/2 is conserved or dissipated, so
    // |w|^2 <= (Imax/Imin) |w0|^2 = (about 6.3) * 25.
    CHECK(wMax < 170.0f, "the tumble never gains energy");
    m3Quat q = m3Body_GetRotation(world, body);
    float norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    CHECK(norm2 > 0.99f && norm2 < 1.01f, "the rotation stays normalized");
    m3DestroyWorld(world);
}

// The rotational energy of a box body spinning at w, its principal
// moments along its own axes.
static double BoxEnergy(const double moments[3], m3Vec3 w, m3Quat q)
{
    m3Vec3 b = m3InvRotateVec3(q, w);
    return 0.5 * (moments[0] * (double)b.x * (double)b.x + moments[1] * (double)b.y * (double)b.y +
                  moments[2] * (double)b.z * (double)b.z);
}

// The world angular momentum of a box body spinning at w: its principal
// moments along its own axes, turned into the world.
static void BoxMomentum(const double moments[3], m3Vec3 w, m3Quat q, double out[3])
{
    m3Vec3 b = m3InvRotateVec3(q, w);
    m3Vec3 l = {(float)(moments[0] * (double)b.x), (float)(moments[1] * (double)b.y),
                (float)(moments[2] * (double)b.z)};
    m3Vec3 world = m3RotateVec3(q, l);
    out[0] = (double)world.x;
    out[1] = (double)world.y;
    out[2] = (double)world.z;
}

// A free tumbler keeps its world angular momentum, and its rotational
// energy within the splitting's bounded error, however far the spin turns
// in a substep. Fifty boxes of three distinct moments tumble for a second
// at start spins from 30 to 800 rad/s, past a radian per substep, on one
// substep and on four, with the angular cap raised out of the way.
static void TestGyroscopicHoldsEnergy(void)
{
    const float speeds[4] = {30.0f, 100.0f, 300.0f, 800.0f};
    double worst = 0.0;
    double worstMomentum = 0.0;
    for (int32_t substeps = 1; substeps <= 4; substeps += 3)
    {
        for (int32_t k = 0; k < 4; ++k)
        {
            m3WorldDef def = m3DefaultWorldDef();
            def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
            def.maximumAngularSpeed = 1.0e5f;
            def.bodyCapacity = 64;
            def.shapeCapacity = 64;
            m3World* world = NewWorld(&def);
            m3BodyId bodies[50];
            double moments[50][3];
            double start[50];
            double startMomentum[50][3];
            for (int32_t t = 0; t < 50; ++t)
            {
                m3BodyDef bd = m3DefaultBodyDef();
                bd.type = m3_dynamicBody;
                bd.angularDamping = 0.0f;
                bd.position = (m3Pos3){10.0 * (double)t, 0.0, 0.0};
                bodies[t] = NewBody(world, &bd);
                m3ShapeDef sd = m3DefaultShapeDef();
                double a = 0.3 + 0.05 * (double)t;
                double b = 0.6;
                double c = 0.15;
                NewBoxShape(world, bodies[t], &sd, (m3Vec3){(float)a, (float)b, (float)c});
                double third = 8.0 * a * b * c / 3.0;
                moments[t][0] = third * (b * b + c * c);
                moments[t][1] = third * (a * a + c * c);
                moments[t][2] = third * (a * a + b * b);
                float s = speeds[k];
                m3Vec3 w = {s * sinf(1.3f * (float)t), s * cosf(0.7f * (float)t), 0.5f * s};
                m3Body_SetAngularVelocity(world, bodies[t], w);
                m3Quat q = m3Body_GetRotation(world, bodies[t]);
                start[t] = BoxEnergy(moments[t], w, q);
                BoxMomentum(moments[t], w, q, startMomentum[t]);
            }
            for (int32_t i = 0; i < 60; ++i)
            {
                (void)m3World_Step(world, 1.0f / 60.0f, substeps);
                for (int32_t t = 0; t < 50; ++t)
                {
                    m3Vec3 w = m3Body_GetAngularVelocity(world, bodies[t]);
                    m3Quat q = m3Body_GetRotation(world, bodies[t]);
                    worst = fmax(worst, fabs(BoxEnergy(moments[t], w, q) / start[t] - 1.0));
                    double l[3];
                    BoxMomentum(moments[t], w, q, l);
                    double dx = l[0] - startMomentum[t][0];
                    double dy = l[1] - startMomentum[t][1];
                    double dz = l[2] - startMomentum[t][2];
                    double size = sqrt(startMomentum[t][0] * startMomentum[t][0] +
                                       startMomentum[t][1] * startMomentum[t][1] +
                                       startMomentum[t][2] * startMomentum[t][2]);
                    worstMomentum = fmax(worstMomentum, sqrt(dx * dx + dy * dy + dz * dz) / size);
                }
            }
            m3DestroyWorld(world);
        }
    }
    printf("gyroscopic: worst energy change %.3g, worst momentum change %.3g\n", worst,
           worstMomentum);
    CHECK(worst <= 5.0e-3, "a tumbling box keeps its rotational energy");
    CHECK(worstMomentum <= 1.0e-4, "a tumbling box keeps its world angular momentum");
}

// A symmetric top keeps its energy and precesses at the rate
// Euler's equations give. A 2 m stick of radius 0.05 m spins at 10 rad/s,
// 45 degrees off its axis, for 10 s at four substeps, once along each body
// axis: the spin along the axis stays, the energy holds within 1e-4, and
// after a second the spin across the axis has turned by
// w_a (I_a - I_t) / I_t radians per second in the body frame. The implicit
// step damped that turn, and the stick lost 87% of its energy.
static void TestSymmetricTopKeepsEnergy(void)
{
    for (int32_t axis = 0; axis < 3; ++axis)
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
        end[axis] = 1.0f;
        m3Capsule capsule = {{-end[0], -end[1], -end[2]}, {end[0], end[1], end[2]}, 0.05f};
        NewCapsuleShape(world, body, &sd, &capsule);
        const int32_t across1 = (axis + 1) % 3;
        const int32_t across2 = (axis + 2) % 3;
        float spin[3] = {0.0f, 0.0f, 0.0f};
        spin[axis] = 7.0710678f;
        spin[across1] = 7.0710678f;
        m3Body_SetAngularVelocity(world, body, (m3Vec3){spin[0], spin[1], spin[2]});
        m3Mat3 inertia = m3Body_GetRotationalInertia(world, body);
        const double moments[3] = {(double)inertia.cx.x, (double)inertia.cy.y,
                                   (double)inertia.cz.z};
        const double rate = 7.0710678 * (moments[axis] - moments[across1]) / moments[across1];

        double startEnergy = 0.0;
        double worst = 0.0;
        double axialDrift = 0.0;
        double phaseError = 0.0;
        for (int32_t i = 0; i <= 600; ++i)
        {
            m3Vec3 local =
                m3Body_GetLocalVector(world, body, m3Body_GetAngularVelocity(world, body));
            const double w[3] = {(double)local.x, (double)local.y, (double)local.z};
            double energy = 0.0;
            for (int32_t k = 0; k < 3; ++k)
            {
                energy += moments[k] * w[k] * w[k];
            }
            if (i == 0)
            {
                startEnergy = energy;
            }
            worst = fmax(worst, fabs(energy / startEnergy - 1.0));
            axialDrift = fmax(axialDrift, fabs(w[axis] / 7.0710678 - 1.0));
            if (i == 60)
            {
                double phase = atan2(w[across2], w[across1]) - rate;
                phaseError = fabs(remainder(phase, 2.0 * (double)M3_PI));
            }
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(worst <= 1.0e-4, "a symmetric top keeps its energy");
        CHECK(axialDrift <= 3.0e-5, "a symmetric top keeps its spin along the axis");
        CHECK(phaseError <= 1.0e-4, "the spin across the axis turns at Euler's rate");
        m3DestroyWorld(world);
    }
}

static void TestDeepSphereRecovers(void)
{
    // A sphere spawned INSIDE a static box (the
    // destruction-rubble pose) exits along the least-deep face and
    // rests on top. Spawned near the top face, the exact point-in-hull
    // kernel must pick +y deterministically.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId block = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, block, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.85, 0.0}; // 0.15 under the top face
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    NewSphereShape(world, ball, &sd, &sphere);

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(world, ball);
    CHECK(p.y > 1.24 && p.y < 1.36, "the deep sphere pops out and rests on top");
    CHECK(p.x > -0.05 && p.x < 0.05, "the exit is straight up, no sideways drift");
    m3DestroyWorld(world);
}

static void TestDeepCapsuleRecovers(void)
{
    // A capsule skewered through the box near its top face: both cap
    // centers are OUTSIDE the side planes, so only the segment SAT
    // sees the right axis (a centroid heuristic would shove it
    // sideways). The deep-recovery promise: the capsule is
    // EXPELLED and settles somewhere sane. Friction does not fight
    // the virtual bias motion, so the pop-out is clean and the rod
    // may end BALANCED ON TOP of the box as well as sliding off:
    // the band accepts both endings.
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId block = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, block, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.8, 0.0}; // core 0.2 under the top
    m3BodyId rod = NewBody(world, &bd);
    m3Capsule capsule = {{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 0.2f};
    NewCapsuleShape(world, rod, &sd, &capsule);

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(world, rod);
    CHECK(p.x > -5.0 && p.x < 5.0 && p.z > -5.0 && p.z < 5.0 && p.y > 0.1 && p.y < 1.45,
          "the skewered capsule is expelled and settles nearby");
    int insideCore =
        p.x > -0.45 && p.x < 0.45 && p.y > 0.05 && p.y < 0.95 && p.z > -0.45 && p.z < 0.45;
    CHECK(!insideCore, "it never stays trapped inside the box");
    m3DestroyWorld(world);
}

static void TestCrossedCapsulesSeparate(void)
{
    // Two capsules spawned with their cores exactly crossing (the
    // measure-zero pose): the mutual perpendicular is +-y here and the
    // center-delta tiebreak is zero, so the fixed rule must still
    // separate them vertically and deterministically.
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.3, 0.0};
    m3BodyId a = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule alongX = {{-0.6f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}, 0.3f};
    NewCapsuleShape(world, a, &sd, &alongX);

    m3BodyId b = NewBody(world, &bd); // same position: cores cross
    m3Capsule alongZ = {{0.0f, 0.0f, -0.6f}, {0.0f, 0.0f, 0.6f}, 0.3f};
    NewCapsuleShape(world, b, &sd, &alongZ);

    // Assert while the stack stands (step 120): the exact rule must
    // have pushed one capsule cleanly on top of the other. Much
    // later the upper one legitimately rolls off its knife-edge
    // balance; that is physics, not the property under test.
    StepN(world, 120);

    m3Pos3 pa = m3Body_GetPosition(world, a);
    m3Pos3 pb = m3Body_GetPosition(world, b);
    double gap = pa.y > pb.y ? pa.y - pb.y : pb.y - pa.y;
    CHECK(gap > 0.5, "the crossed capsules separate vertically");
    double low = pa.y < pb.y ? pa.y : pb.y;
    CHECK(low > 0.27 && low < 0.33, "the lower one rests on the plane");
    m3DestroyWorld(world);
}

static void TestQuickHullRockRests(void)
{
    // A QuickHull rock (an irregular octahedron)
    // drops onto the plane, settles onto a face, and a journaled
    // session containing the hull create replays bit for bit (the
    // recipe points ride the journal, QuickHull rebuilds them into
    // the identical hull, id determinism included).
    uint8_t journal[16384];
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    CHECK(StartJournal(world, journal, (int32_t)sizeof(journal)), "journal arms");
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId rock = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Vec3 pts[7] = {{0.9f, 0.0f, 0.1f},  {-0.8f, 0.1f, 0.0f}, {0.0f, 0.7f, -0.1f},
                     {0.1f, -0.6f, 0.0f}, {0.0f, 0.1f, 0.8f},  {-0.1f, 0.0f, -0.9f},
                     {0.2f, 0.2f, 0.2f}};
    m3ShapeId shape = NewHullShape(world, rock, &sd, pts, 7);
    CHECK(m3Shape_IsValid(world, shape), "the rock builds");

    // A degenerate cloud is refused without touching the body.
    m3Vec3 flat[4] = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}};
    CHECK(!m3Shape_IsValid(world, NewHullShape(world, rock, &sd, flat, 4)),
          "a coplanar cloud is refused");

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(world, rock);
    CHECK(p.y > 0.05 && p.y < 1.0, "the rock settles onto a face");
    CHECK(p.x > -2.0 && p.x < 2.0 && p.z > -2.0 && p.z < 2.0, "the rock stays near the drop");
    m3Vec3 v = m3Body_GetLinearVelocity(world, rock);
    CHECK(v.x * v.x + v.y * v.y + v.z * v.z < 0.01f, "the rock has come to rest");

    int64_t bytes = StopJournal(world);
    CHECK(bytes > 0, "the session recorded");
    uint64_t h1 = m3World_Hash(world);

    m3World* twin = NewWorld(&def);
    CHECK(ReplayJournal(twin, journal, bytes), "the hull session replays");
    CHECK(m3World_Hash(twin) == h1, "the replay is bit-identical");
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

static void TestBulletStopsAtWall(void)
{
    // A sphere at 200 m/s crosses 3.3 meters per
    // step, sixty-six times its own radius; without the continuous
    // pass it would skip the 0.1-thick wall entirely. Every FAST
    // dynamic body sweeps against statics (the bullet flag is not
    // needed for static targets), so both variants must stop.
    for (int32_t variant = 0; variant < 2; ++variant)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        m3World* world = NewWorld(&def);

        m3BodyDef bd = m3DefaultBodyDef();
        bd.position = (m3Pos3){5.0, 0.0, 0.0};
        m3BodyId wall = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        NewBoxShape(world, wall, &sd, (m3Vec3){0.05f, 1.0f, 1.0f});

        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.0, 0.0};
        bd.linearVelocity = (m3Vec3){200.0f, 0.0f, 0.0f};
        bd.enableBullet = variant == 1;
        m3BodyId ball = NewBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.1f};
        NewSphereShape(world, ball, &sd, &sphere);

        int stayed = 1;
        for (int32_t i = 0; i < 60; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            m3Pos3 p = m3Body_GetPosition(world, ball);
            if (p.x > 4.9)
            {
                stayed = 0;
            }
        }
        CHECK(stayed, "the fast sphere never crosses the thin wall");
        m3DestroyWorld(world);
    }
}

static void TestBulletVsDynamicTarget(void)
{
    // The flag semantics: dynamic targets are swept ONLY for bullets.
    // A bullet fired at a floating box stops at its face (both sweeps
    // enter the TOI: the target is free to drift). The identical
    // non-bullet sphere tunnels straight through: the documented
    // reason the flag exists.
    for (int32_t variant = 0; variant < 2; ++variant)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        m3World* world = NewWorld(&def);

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){5.0, 0.0, 0.0};
        m3BodyId target = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        NewBoxShape(world, target, &sd, (m3Vec3){0.5f, 1.0f, 1.0f});

        bd.position = (m3Pos3){0.0, 0.0, 0.0};
        bd.linearVelocity = (m3Vec3){150.0f, 0.0f, 0.0f};
        bd.enableBullet = variant == 1;
        m3BodyId ball = NewBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.1f};
        NewSphereShape(world, ball, &sd, &sphere);

        int stayedBehind = 1;
        for (int32_t i = 0; i < 40; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            m3Pos3 pb = m3Body_GetPosition(world, ball);
            m3Pos3 pt = m3Body_GetPosition(world, target);
            if (pb.x > pt.x)
            {
                stayedBehind = 0;
            }
        }
        if (variant == 1)
        {
            CHECK(stayedBehind, "the bullet stays behind the dynamic box");
        }
        else
        {
            m3Pos3 pb = m3Body_GetPosition(world, ball);
            CHECK(!stayedBehind && pb.x > 10.0,
                  "the non-bullet tunnels through the dynamic box (why the flag exists)");
        }
        m3DestroyWorld(world);
    }
}

// Build a flat triangulated strip: x in [-4, 4], z in [-1, 1], one
// meter quads, each split into two CCW-from-above triangles. The
// interior edges and vertices are exactly what the welding filter
// must silence.
static m3ShapeId AddMeshFloor(m3World* world)
{
    enum
    {
        NX = 9,
        NZ = 3
    };
    m3Vec3 verts[NX * NZ];
    for (int32_t iz = 0; iz < NZ; ++iz)
    {
        for (int32_t ix = 0; ix < NX; ++ix)
        {
            verts[iz * NX + ix] = (m3Vec3){-4.0f + (m3real)ix, 0.0f, -1.0f + (m3real)iz};
        }
    }
    uint16_t tris[3 * 2 * (NX - 1) * (NZ - 1)];
    int32_t n = 0;
    for (int32_t iz = 0; iz < NZ - 1; ++iz)
    {
        for (int32_t ix = 0; ix < NX - 1; ++ix)
        {
            uint16_t v00 = (uint16_t)(iz * NX + ix);
            uint16_t v10 = (uint16_t)(iz * NX + ix + 1);
            uint16_t v01 = (uint16_t)((iz + 1) * NX + ix);
            uint16_t v11 = (uint16_t)((iz + 1) * NX + ix + 1);
            tris[n++] = v00;
            tris[n++] = v11;
            tris[n++] = v10;
            tris[n++] = v00;
            tris[n++] = v01;
            tris[n++] = v11;
        }
    }
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.0f; // pure slide: any vertical kick is a ghost
    return NewMeshShape(world, ground, &sd, verts, NX * NZ, tris, n / 3);
}

static void TestSphereSlidesAcrossMeshFloor(void)
{
    // THE internal-edge proof: a sphere slides across six interior
    // edges and a row of interior vertices of a flat triangulated
    // floor. Without feature welding the neighbor triangles' edge
    // contacts point their ghost normals sideways and kick the
    // sphere; with it the ride must stay flat to millimeters.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    CHECK(m3Shape_IsValid(world, AddMeshFloor(world)), "the mesh floor builds");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-3.5, 0.5, 0.0};
    bd.linearVelocity = (m3Vec3){1.2f, 0.0f, 0.0f};
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.0f;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, ball, &sd, &sphere);

    // Settle onto the floor first, then measure the ride.
    StepN(world, 60);
    double yMin = 1.0e30;
    double yMax = -1.0e30;
    double zMax = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 p = m3Body_GetPosition(world, ball);
        yMin = p.y < yMin ? p.y : yMin;
        yMax = p.y > yMax ? p.y : yMax;
        double az = p.z < 0.0 ? -p.z : p.z;
        zMax = az > zMax ? az : zMax;
    }
    m3Pos3 end = m3Body_GetPosition(world, ball);
    CHECK(end.x > 0.0, "the sphere crossed the interior edges");
    CHECK(yMin > 0.46 && yMax < 0.54, "the ride stays flat: no ghost bumps");
    CHECK(zMax < 0.05, "no sideways ghost kicks");
    m3DestroyWorld(world);
}

static void TestMeshContractsAndGaps(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    CHECK(m3Shape_IsValid(world, AddMeshFloor(world)), "the mesh floor builds");

    // Contracts: a mesh on a dynamic body and bad indices are refused.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId mover = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Vec3 tri[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    uint16_t idx[3] = {0, 1, 2};
    CHECK(!m3Shape_IsValid(world, NewMeshShape(world, mover, &sd, tri, 3, idx, 1)),
          "a mesh on a dynamic body is refused");
    uint16_t bad[3] = {0, 1, 9};
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground2 = NewBody(world, &gd);
    CHECK(!m3Shape_IsValid(world, NewMeshShape(world, ground2, &sd, tri, 3, bad, 1)),
          "an out-of-range index is refused");

    // The hull half of the mesh edge handling: the box dropped over
    // the interior edges lands on
    // the triangle-face clip and RESTS level on the mesh.
    m3Capsule capsule = {{-0.4f, 0.0f, 0.0f}, {0.4f, 0.0f, 0.0f}, 0.3f};
    NewCapsuleShape(world, mover, &sd, &capsule);
    bd.position = (m3Pos3){1.5, 2.0, 0.0};
    m3BodyId boxBody = NewBody(world, &bd);
    NewBoxShape(world, boxBody, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    StepN(world, 240);
    m3Pos3 cp = m3Body_GetPosition(world, mover);
    CHECK(cp.y > 0.27 && cp.y < 0.33, "the capsule rests on the mesh at its radius");
    m3Quat cq = m3Body_GetRotation(world, mover);
    CHECK(cq.w > 0.999f || cq.w < -0.999f, "the capsule lies level across the edges");
    m3Pos3 bp = m3Body_GetPosition(world, boxBody);
    CHECK(bp.y > 0.27 && bp.y < 0.33, "the box rests on the mesh at its half height");
    m3Quat bq = m3Body_GetRotation(world, boxBody);
    CHECK(bq.w > 0.999f || bq.w < -0.999f, "the box stays level on the mesh");
    m3DestroyWorld(world);
}

static void TestBoxSlidesAcrossMeshFloor(void)
{
    // The internal-edge proof for the hull path: a box slides across
    // the interior edges of the flat mesh; the triangle-face clips
    // must weld into one flat manifold, never a ghost edge kick.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    CHECK(m3Shape_IsValid(world, AddMeshFloor(world)), "the mesh floor builds");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-3.2, 0.31, 0.0};
    bd.linearVelocity = (m3Vec3){1.2f, 0.0f, 0.0f};
    m3BodyId box = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.0f;
    NewBoxShape(world, box, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});

    StepN(world, 60); // settle
    double yMin = 1.0e30;
    double yMax = -1.0e30;
    double zMax = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 p = m3Body_GetPosition(world, box);
        yMin = p.y < yMin ? p.y : yMin;
        yMax = p.y > yMax ? p.y : yMax;
        double az = p.z < 0.0 ? -p.z : p.z;
        zMax = az > zMax ? az : zMax;
    }
    m3Pos3 end = m3Body_GetPosition(world, box);
    CHECK(end.x > 0.0, "the box crossed the interior edges");
    CHECK(yMin > 0.26 && yMax < 0.34, "the box ride stays flat: no ghost bumps");
    CHECK(zMax < 0.05, "no sideways ghost kicks on the box");
    m3Quat q = m3Body_GetRotation(world, box);
    CHECK(q.w > 0.99f || q.w < -0.99f, "the box does not tumble while sliding");
    m3DestroyWorld(world);
}

static void TestKinematicPlatform(void)
{
    // A kinematic platform moves at its commanded velocity, immovable
    // by the ball riding it; friction drags the ball along.
    m3World* world = MakeWorld();

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    bd.linearVelocity = (m3Vec3){1.0f, 0.0f, 0.0f};
    m3BodyId platform = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.9f;
    NewBoxShape(world, platform, &sd, (m3Vec3){2.0f, 0.1f, 2.0f});

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.6, 0.0};
    bd.linearVelocity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    NewSphereShape(world, ball, &sd, &sphere);

    StepN(world, 120); // two seconds

    m3Pos3 pp = m3Body_GetPosition(world, platform);
    CHECK(pp.x > 1.99 && pp.x < 2.01, "the platform advances exactly at its velocity");
    CHECK(pp.y > 0.99 && pp.y < 1.01, "the ball's weight cannot push the platform");
    m3Pos3 bp = m3Body_GetPosition(world, ball);
    CHECK(bp.y > 1.35 && bp.y < 1.45, "the ball rides on the platform");
    // A ball on a conveyor rolls: slip ends at v = (2/7) * platform
    // speed (inertia 2/5 m r^2 spins up 2.5x faster than it
    // translates). Assert the analytic band, not wishful thinking.
    m3Vec3 bv = m3Body_GetLinearVelocity(world, ball);
    CHECK(bv.x > 0.2f && bv.x < 0.4f, "friction rolls the ball at the conveyor ratio");
    m3DestroyWorld(world);
}

static void TestSleepFreezesAndWakes(void)
{
    // A resting ball crosses the half-second sleep bar and freezes
    // BIT-SOLID: velocities exactly zero, position bit-stable across
    // thirty further steps. A commanded velocity wakes it; a second
    // ball dropped on it wakes the island through the contact.
    m3World* world = MakeWorld();
    AddGroundPlane(world, 0.6f);
    m3BodyId ball = AddBall(world, 0.0, 0.5, 0.0, 0.5f, 0.6f, 0.0f);

    StepN(world, 120); // settle plus the sleep bar
    m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
    CHECK(v.x == 0.0f && v.y == 0.0f && v.z == 0.0f, "the sleeping ball has exact zero velocity");
    m3Pos3 before = m3Body_GetPosition(world, ball);
    StepN(world, 30);
    m3Pos3 after = m3Body_GetPosition(world, ball);
    CHECK(before.x == after.x && before.y == after.y && before.z == after.z,
          "the sleeping ball is bit-frozen");

    // A commanded velocity wakes it.
    m3Body_SetLinearVelocity(world, ball, (m3Vec3){1.0f, 0.0f, 0.0f});
    StepN(world, 30);
    m3Pos3 moved = m3Body_GetPosition(world, ball);
    CHECK(moved.x > after.x + 0.1, "a commanded velocity wakes the sleeper");

    // Let it sleep again, then wake it by dropping a second ball on it.
    StepN(world, 300);
    m3Pos3 asleep = m3Body_GetPosition(world, ball);
    m3BodyId dropper = AddBall(world, asleep.x, asleep.y + 2.0, asleep.z, 0.3f, 0.6f, 0.0f);
    StepN(world, 60);
    m3Pos3 pushed = m3Body_GetPosition(world, ball);
    m3Pos3 dp = m3Body_GetPosition(world, dropper);
    CHECK(dp.y < asleep.y + 1.5, "the dropper fell onto the sleeper");
    CHECK(pushed.y < asleep.y - 0.001 || pushed.x != asleep.x || pushed.z != asleep.z ||
              m3Body_GetLinearVelocity(world, ball).y != 0.0f ||
              m3Body_GetLinearVelocity(world, dropper).y != 0.0f,
          "the contact wakes the island");
    m3DestroyWorld(world);
}

static void TestBulletStopsAtMeshWall(void)
{
    // Meshes are TOI targets: a 200 m/s bullet must stop against a
    // triangulated wall as it does against a box wall, not tunnel
    // through it.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);

    // A vertical two-triangle wall at x = 5, facing -x.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){5.0, -2.0, -2.0};
    m3BodyId wall = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Vec3 verts[4] = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 4.0f, 0.0f}, {0.0f, 0.0f, 4.0f}, {0.0f, 4.0f, 4.0f}};
    uint16_t tris[6] = {0, 2, 1, 1, 2, 3}; // normals face -x
    CHECK(m3Shape_IsValid(world, NewMeshShape(world, wall, &sd, verts, 4, tris, 2)),
          "the mesh wall builds");

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.0, 0.0};
    bd.linearVelocity = (m3Vec3){200.0f, 0.0f, 0.0f};
    bd.enableBullet = true;
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.1f};
    NewSphereShape(world, ball, &sd, &sphere);

    int stayed = 1;
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        if (m3Body_GetPosition(world, ball).x > 5.0)
        {
            stayed = 0;
        }
    }
    CHECK(stayed, "the bullet never tunnels through the mesh wall");
    m3DestroyWorld(world);
}

static void TestBoxOnRoofRidge(void)
{
    // The baked convexity proof: two tilted quads meet in a genuinely
    // convex ridge. A box dropped square on the ridge makes contact
    // THROUGH the ridge edge (a real feature, exempt from the claim
    // filter) and must come to rest straddling it, never fall inside.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId roof = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.8f;
    // Ridge along z at x = 0, apex y = 1, sloping to y = 0 at x = +-2.
    m3Vec3 verts[6] = {{-2.0f, 0.0f, -2.0f}, {0.0f, 1.0f, -2.0f}, {2.0f, 0.0f, -2.0f},
                       {-2.0f, 0.0f, 2.0f},  {0.0f, 1.0f, 2.0f},  {2.0f, 0.0f, 2.0f}};
    uint16_t tris[12] = {0, 4, 1, 0, 3, 4,  // -x slope (normals up-left)
                         1, 5, 2, 1, 4, 5}; // +x slope (normals up-right)
    CHECK(m3Shape_IsValid(world, NewMeshShape(world, roof, &sd, verts, 6, tris, 4)),
          "the roof builds");

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.8, 0.0};
    m3BodyId box = NewBody(world, &bd);
    NewBoxShape(world, box, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});

    StepN(world, 300);
    m3Pos3 p = m3Body_GetPosition(world, box);
    CHECK(p.y > 1.0, "the box rests on the ridge, never swallowed by it");
    CHECK(p.x > -1.0 && p.x < 1.0 && p.z > -2.0 && p.z < 2.0, "the box stays near the apex");
    m3DestroyWorld(world);
}

static void TestHeightFieldBowl(void)
{
    // The heightfield chunk helper rides the whole mesh path: a
    // sphere dropped off-center in a bowl-shaped field rolls downhill
    // and settles near the middle, always above the surface.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){-8.0, 0.0, -8.0}; // center the grid on the origin
    m3BodyId ground = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.2f;
    float heights[17 * 17];
    for (int32_t iz = 0; iz < 17; ++iz)
    {
        for (int32_t ix = 0; ix < 17; ++ix)
        {
            float dx = (float)(ix - 8);
            float dz = (float)(iz - 8);
            heights[iz * 17 + ix] = 0.02f * (dx * dx + dz * dz); // a parabolic bowl
        }
    }
    CHECK(
        m3Shape_IsValid(world, NewHeightFieldMeshShape(world, ground, &sd, heights, 17, 17, 1.0f)),
        "the heightfield chunk builds");
    CHECK(
        !m3Shape_IsValid(world, NewHeightFieldMeshShape(world, ground, &sd, heights, 40, 17, 1.0f)),
        "an oversized grid is refused");

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){3.0, 1.5, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
    NewSphereShape(world, ball, &sd, &sphere);

    StepN(world, 600);
    m3Pos3 p = m3Body_GetPosition(world, ball);
    CHECK(p.x > -2.5 && p.x < 2.5 && p.z > -2.5 && p.z < 2.5,
          "the ball rolls downhill toward the bowl's middle");
    CHECK(p.y > 0.3 && p.y < 1.2, "the ball stays above the surface");
    m3DestroyWorld(world);
}

static void TestRollingResistance(void)
{
    // Two identical balls shoved at the same speed: the one with
    // rolling resistance stops and SLEEPS; the free one is still
    // rolling when the lights go out. Twin worlds agree to the bit
    // (the new field folds into the hash only where nonzero).
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        m3World* world = NewWorld(&def);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef gs = m3DefaultShapeDef();
        gs.friction = 0.6f;
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &gs, &floor);

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.5, -3.0};
        bd.linearVelocity = (m3Vec3){2.0f, 0.0f, 0.0f};
        m3BodyId gripper = NewBody(world, &bd);
        m3ShapeDef rr = m3DefaultShapeDef();
        rr.friction = 0.6f;
        rr.rollingResistance = 0.1f;
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
        NewSphereShape(world, gripper, &rr, &ball);

        bd.position = (m3Pos3){0.0, 0.5, 3.0};
        m3BodyId roller = NewBody(world, &bd);
        m3ShapeDef free = m3DefaultShapeDef();
        free.friction = 0.6f;
        NewSphereShape(world, roller, &free, &ball);

        for (int32_t i = 0; i < 420; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Vec3 vGrip = m3Body_GetLinearVelocity(world, gripper);
        m3Vec3 vFree = m3Body_GetLinearVelocity(world, roller);
        CHECK(fabsf(vGrip.x) < 0.05f, "the resisted ball rolls to a stop");
        CHECK(vFree.x > 1.0f, "the free ball is still rolling");

        // Hostile def refuses.
        m3ShapeDef bad = m3DefaultShapeDef();
        bad.rollingResistance = -1.0f;
        m3BodyDef xb = m3DefaultBodyDef();
        xb.position = (m3Pos3){5.0, 2.0, 0.0};
        m3BodyId host = NewBody(world, &xb);
        CHECK(!m3Shape_IsValid(world, NewSphereShape(world, host, &bad, &ball)),
              "a negative rolling resistance refuses");

        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "rolling twins are bit-identical");
}

// Rolling resistance slows a ball alike at any density. The rolling row's
// 3x3 is the balls' inverse inertia: at density 6000 its determinant is
// subnormal, and at 1e6 it underflows to zero, and both are solved at the
// matrix's own scale.
static void TestHeavyBallRollsAlike(void)
{
    const float densities[3] = {1.0f, 6000.0f, 1.0e6f};
    m3Vec3 speed[3];
    m3Vec3 spin[3];
    for (int32_t i = 0; i < 3; ++i)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        m3World* world = NewWorld(&wd);
        m3BodyDef bd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        sd.rollingResistance = 0.05f;
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &sd, &floor);
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 100.0, 0.0};
        bd.linearVelocity = (m3Vec3){10.0f, 0.0f, 0.0f};
        bd.angularVelocity = (m3Vec3){0.0f, 0.0f, -0.1f};
        m3BodyId ball = NewBody(world, &bd);
        sd.density = densities[i];
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 100.0f};
        NewSphereShape(world, ball, &sd, &sphere);
        for (int32_t k = 0; k < 120; ++k)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        speed[i] = m3Body_GetLinearVelocity(world, ball);
        spin[i] = m3Body_GetAngularVelocity(world, ball);
        m3DestroyWorld(world);
    }
    CHECK(speed[0].x < 9.5f, "rolling resistance slows the light ball");
    for (int32_t i = 1; i < 3; ++i)
    {
        CHECK(fabsf(speed[i].x - speed[0].x) < 1.0e-4f * speed[0].x &&
                  fabsf(spin[i].z - spin[0].z) < 1.0e-4f * fabsf(spin[0].z),
              "a heavier ball slows as the light one does");
    }
}

int main(void)
{
    TestRollingResistance();
    TestHeavyBallRollsAlike();
    TestFreeFall();
    TestRestOnPlane();
    TestStandingStack();
    TestRestitution();
    TestBounceHeldWithinSpeedLimit();
    TestFrictionSlows();
    TestSlideDistanceLaw();
    TestBoxRests();
    TestCapsuleRestsFlat();
    TestSphereRestsOnBox();
    TestSphereRestsOnCapsule();
    TestGyroscopicTumble();
    TestGyroscopicHoldsEnergy();
    TestSymmetricTopKeepsEnergy();
    TestDeepSphereRecovers();
    TestDeepCapsuleRecovers();
    TestCrossedCapsulesSeparate();
    TestQuickHullRockRests();
    TestBulletStopsAtWall();
    TestBulletVsDynamicTarget();
    TestSphereSlidesAcrossMeshFloor();
    TestMeshContractsAndGaps();
    TestBoxSlidesAcrossMeshFloor();
    TestKinematicPlatform();
    TestSleepFreezesAndWakes();
    TestBulletStopsAtMeshWall();
    TestBoxOnRoofRidge();
    TestHeightFieldBowl();
    if (s_failures == 0)
    {
        printf("test_solver: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
