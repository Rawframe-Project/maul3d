// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint gate: every joint kind against analytic physics (the pendulum,
// a hanging chain, doors, sliders, cones, welds, ropes and springs),
// the row Jacobians against finite differences, and the def contracts.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static double RandD(uint64_t* state, double lo, double hi)
{
    *state += 0x9E3779B97F4A7C15ull;
    uint64_t z = *state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z = z ^ (z >> 31);
    return lo + (hi - lo) * ((double)(z >> 11) / 9007199254740992.0);
}

static m3Quat RandQuat(uint64_t* state)
{
    m3Quat q = {(float)RandD(state, -1.0, 1.0), (float)RandD(state, -1.0, 1.0),
                (float)RandD(state, -1.0, 1.0), (float)RandD(state, -1.0, 1.0)};
    return m3NormalizeQuat(q);
}

static void StepN(m3World* world, int32_t steps)
{
    for (int32_t i = 0; i < steps; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static void TestPendulum(void)
{
    // A point pendulum of length one released horizontal: the bob
    // must pass the bottom near sqrt(2 g L) and the rod length must
    // hold within the soft-constraint band the whole swing.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);

    m3BodyDef ad = m3DefaultBodyDef();
    m3BodyId anchor = NewBody(world, &ad); // shapeless static pivot

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.0, 0.0, 0.0};
    m3BodyId bob = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.1f};
    NewSphereShape(world, bob, &sd, &ball);

    m3JointDef jd = m3DefaultJointDef();
    jd.bodyIdA = anchor;
    jd.bodyIdB = bob;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-1.0f, 0.0f, 0.0f}; // the rod end on the bob
    m3JointId joint = NewJoint(world, &jd);
    CHECK(m3Joint_IsValid(world, joint), "the pendulum joint creates");

    double vMax = 0.0;
    double maxStretch = 0.0;
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 p = m3Body_GetPosition(world, bob);
        m3Vec3 v = m3Body_GetLinearVelocity(world, bob);
        double speed = (double)m3Dot3(v, v);
        vMax = speed > vMax ? speed : vMax;
        m3Quat q = m3Body_GetRotation(world, bob);
        m3Vec3 arm = m3RotateVec3(q, (m3Vec3){-1.0f, 0.0f, 0.0f});
        double px = p.x + (double)arm.x;
        double py = p.y + (double)arm.y;
        double pz = p.z + (double)arm.z;
        double drift = px * px + py * py + pz * pz; // pinned point vs origin
        maxStretch = drift > maxStretch ? drift : maxStretch;
    }
    // sqrt(2 * 10 * 1) = 4.47; the soft constraint bleeds a little.
    CHECK(vMax > 15.0 && vMax < 21.0, "the bob passes the bottom in the energy band");
    CHECK(maxStretch < 0.005, "the pinned point holds inside the soft band");
    m3DestroyWorld(world);
}

// A joint error pushes its bodies back no faster than the speed limit
// (1e5 m/s): a ball socket pulled 1e4 m out of place closes by at most
// 1e5 m/s times the step, and closes completely within a few steps.
static void TestFarErrorClosesAtSpeedLimit(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.jointCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId anchor = NewBody(world, &bd);
    bd.type = m3_dynamicBody;
    m3BodyId bob = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, bob, &sd, &ball);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = bob;
    NewJoint(world, &jd);
    m3Body_SetTransform(world, bob, (m3Pos3){1.0e4, 0.0, 0.0}, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
    (void)m3World_Step(world, 1.0f / 60.0f, 1);
    double x = m3Body_GetPosition(world, bob).x;
    CHECK(x < 1.0e4 && x >= 1.0e4 - 1.0e5 / 60.0 * 1.0001,
          "the error closes no faster than the speed limit");
    StepN(world, 20);
    CHECK(fabs(m3Body_GetPosition(world, bob).x) < 1.0, "the error closes");
    m3DestroyWorld(world);
}

static void TestChainSettlesAndSleeps(void)
{
    // Four links hang from a static anchor, settle vertically, and
    // the whole articulated island crosses into sleep TOGETHER (the
    // joint-union proof); the sleeping chain is bit-frozen.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3World* world = NewWorld(&def);

    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){0.0, 4.0, 0.0};
    m3BodyId anchor = NewBody(world, &ad);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    // A frictionless multi-pendulum is chaotic and conserves energy
    // forever; damping is how a rope stops being a perpetual motion
    // machine. Start near vertical, bleed the sway out.
    bd.linearDamping = 1.5f;
    bd.angularDamping = 1.5f;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId previous = anchor;
    m3BodyId links[4];
    for (int32_t k = 0; k < 4; ++k)
    {
        bd.position = (m3Pos3){0.05 * (double)(k + 1), 4.0 - 0.5 * (double)(k + 1), 0.0};
        links[k] = NewBody(world, &bd);
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.12f};
        NewSphereShape(world, links[k], &sd, &ball);

        m3JointDef jd = m3DefaultJointDef();
        jd.bodyIdA = previous;
        jd.bodyIdB = links[k];
        jd.localAnchorA = (m3Vec3){0.0f, k == 0 ? 0.0f : -0.25f, 0.0f};
        jd.localAnchorB = (m3Vec3){0.0f, 0.25f, 0.0f};
        CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "a chain link joins");
        previous = links[k];
    }

    StepN(world, 900); // settle plus the sleep bar
    for (int32_t k = 0; k < 4; ++k)
    {
        m3Pos3 p = m3Body_GetPosition(world, links[k]);
        CHECK(p.x > -0.05 && p.x < 0.05, "the chain hangs vertical");
        m3Vec3 v = m3Body_GetLinearVelocity(world, links[k]);
        CHECK(v.x == 0.0f && v.y == 0.0f && v.z == 0.0f, "the settled link sleeps");
    }
    m3Pos3 before = m3Body_GetPosition(world, links[3]);
    StepN(world, 30);
    m3Pos3 after = m3Body_GetPosition(world, links[3]);
    CHECK(before.x == after.x && before.y == after.y && before.z == after.z,
          "the sleeping chain is bit-frozen");

    // Waking the TOP link must wake the whole articulated island.
    m3Body_SetLinearVelocity(world, links[0], (m3Vec3){2.0f, 0.0f, 0.0f});
    StepN(world, 30);
    m3Pos3 tail = m3Body_GetPosition(world, links[3]);
    CHECK(tail.x != after.x || tail.y != after.y, "the joint union wakes the whole chain");
    m3DestroyWorld(world);
}

static void TestJointContracts(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 1;
    m3World* world = NewWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    m3BodyId a = NewBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.0, 0.0, 0.0};
    m3BodyId b = NewBody(world, &bd);
    m3BodyId c = NewBody(world, &bd);

    m3JointDef jd = m3DefaultJointDef();
    jd.bodyIdA = a;
    jd.bodyIdB = a;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a self-joint is refused");
    jd.bodyIdB = NewBody(world, &ad); // static-static
    jd.bodyIdA = a;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "an immovable pair is refused");
    jd.bodyIdB = b;
    m3JointId first = NewJoint(world, &jd);
    CHECK(m3Joint_IsValid(world, first), "the single slot fills");
    jd.bodyIdB = c;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "the joint pool refuses past capacity");

    // Destroying a body cascades its joints.
    DestroyBody(world, b);
    CHECK(!m3Joint_IsValid(world, first), "destroying a body destroys its joints");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    m3DestroyWorld(world);
}

static m3BodyId MakeDoor(m3World* world, m3JointDef* jdOut)
{
    // A door panel hinged to a static post on the y axis: the hinge
    // line passes through x = 0, the panel extends to +x.
    m3BodyDef pd = m3DefaultBodyDef();
    m3BodyId post = NewBody(world, &pd);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.5, 0.0, 0.0};
    m3BodyId door = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, door, &sd, (m3Vec3){0.5f, 1.0f, 0.05f});

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = post;
    jd.bodyIdB = door;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.5f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    *jdOut = jd;
    return door;
}

static double DoorAngle(const m3World* world, m3BodyId door)
{
    // The hinge axis is world y and the door starts along +x, so the
    // yaw of the body quaternion IS the door angle (collinearity
    // keeps the other two rotations locked).
    m3Quat q = m3Body_GetRotation(world, door);
    return 2.0 * (double)m3Atan2(q.y, q.w);
}

static void TestDoorSwings(void)
{
    // Gravity off: a pushed door spins about the hinge axis only,
    // the hinge line never drifts, and no off-axis rotation leaks.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3JointDef jd;
    m3BodyId door = MakeDoor(world, &jd);
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the hinge creates");
    m3Body_SetAngularVelocity(world, door, (m3Vec3){0.0f, 2.0f, 0.0f});

    double maxDrift = 0.0;
    double maxOffAxis = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 p = m3Body_GetPosition(world, door);
        m3Quat q = m3Body_GetRotation(world, door);
        m3Vec3 hinge = m3RotateVec3(q, (m3Vec3){-0.5f, 0.0f, 0.0f});
        double hx = p.x + (double)hinge.x;
        double hy = p.y + (double)hinge.y;
        double hz = p.z + (double)hinge.z;
        double drift = hx * hx + hy * hy + hz * hz;
        maxDrift = drift > maxDrift ? drift : maxDrift;
        double off = (double)(q.x * q.x + q.z * q.z); // yaw-only means x, z stay zero
        maxOffAxis = off > maxOffAxis ? off : maxOffAxis;
    }
    CHECK(maxDrift < 0.0005, "the hinge line never drifts");
    CHECK(maxOffAxis < 0.001, "no off-axis rotation leaks");
    m3DestroyWorld(world);
}

static void TestDoorLimits(void)
{
    // Limits at plus and minus a quarter turn: a hard shove never
    // carries the door past the stops (plus the speculative band).
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3JointDef jd;
    m3BodyId door = MakeDoor(world, &jd);
    jd.enableLimit = true;
    jd.lowerLimit = -0.7854f;
    jd.upperLimit = 0.7854f;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the limited hinge creates");
    m3Body_SetAngularVelocity(world, door, (m3Vec3){0.0f, 12.0f, 0.0f});

    double maxAngle = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        if (i == 120)
        {
            m3Body_SetAngularVelocity(world, door, (m3Vec3){0.0f, -12.0f, 0.0f});
        }
        double a = DoorAngle(world, door);
        double mag = a < 0.0 ? -a : a;
        maxAngle = mag > maxAngle ? mag : maxAngle;
    }
    CHECK(maxAngle < 0.83, "the stops hold against a hard shove both ways");
    CHECK(maxAngle > 0.70, "the door actually reached the stops");
    m3DestroyWorld(world);
}

static void TestDoorMotor(void)
{
    // A strong motor reaches its commanded speed; a feeble motor
    // against the same door proves the torque cap (it cannot).
    for (int32_t variant = 0; variant < 2; ++variant)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        m3World* world = NewWorld(&def);
        m3JointDef jd;
        m3BodyId door = MakeDoor(world, &jd);
        jd.enableMotor = true;
        jd.motorSpeed = 2.0f;
        jd.maxMotorEffort = variant == 0 ? 50.0f : 0.02f;
        CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the motored hinge creates");

        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Vec3 w = m3Body_GetAngularVelocity(world, door);
        if (variant == 0)
        {
            CHECK(w.y > 1.9f && w.y < 2.1f, "the strong motor reaches its speed");
        }
        else
        {
            CHECK(w.y < 1.0f, "the feeble motor is honestly torque-capped");
        }
        m3DestroyWorld(world);
    }
}

static void TestPrismaticJacobianFiniteDifference(void)
{
    // The slide rows reach from A's center to B's anchor: their angular
    // part on A is cross(rA + d, axis), not cross(rA, axis). Held to the
    // numerics here: for random poses and velocities the analytic Cdot
    // rows (the axis and both perpendiculars) must match central
    // differences of C.
    uint64_t state = 0xA5A5A5A5ull;
    int32_t checked = 0;
    for (int32_t round = 0; round < 20; ++round)
    {
        // Random poses near the origin, random anchors, random axis.
        m3Vec3 pA = {(float)RandD(&state, -1.0, 1.0), (float)RandD(&state, -1.0, 1.0),
                     (float)RandD(&state, -1.0, 1.0)};
        m3Vec3 pB = {(float)RandD(&state, -1.0, 1.0), (float)RandD(&state, -1.0, 1.0),
                     (float)RandD(&state, -1.0, 1.0)};
        m3Quat qA = RandQuat(&state);
        m3Quat qB = RandQuat(&state);
        m3Vec3 laA = {(float)RandD(&state, -0.5, 0.5), (float)RandD(&state, -0.5, 0.5),
                      (float)RandD(&state, -0.5, 0.5)};
        m3Vec3 laB = {(float)RandD(&state, -0.5, 0.5), (float)RandD(&state, -0.5, 0.5),
                      (float)RandD(&state, -0.5, 0.5)};
        m3Vec3 axisLocal =
            m3Normalize3((m3Vec3){(float)RandD(&state, -1.0, 1.0), (float)RandD(&state, -1.0, 1.0),
                                  (float)RandD(&state, -1.0, 1.0)});
        m3Vec3 vA = {(float)RandD(&state, -2.0, 2.0), (float)RandD(&state, -2.0, 2.0),
                     (float)RandD(&state, -2.0, 2.0)};
        m3Vec3 vB = {(float)RandD(&state, -2.0, 2.0), (float)RandD(&state, -2.0, 2.0),
                     (float)RandD(&state, -2.0, 2.0)};
        m3Vec3 wA = {(float)RandD(&state, -2.0, 2.0), (float)RandD(&state, -2.0, 2.0),
                     (float)RandD(&state, -2.0, 2.0)};
        m3Vec3 wB = {(float)RandD(&state, -2.0, 2.0), (float)RandD(&state, -2.0, 2.0),
                     (float)RandD(&state, -2.0, 2.0)};

        // C rows as a function of the pose.
        // rA = rot(qA, laA), rB = rot(qB, laB),
        // d = pB + rB - pA - rA, axis/perps fixed in A.
        // Any perpendicular pair does for the numerics; the test is
        // self-contained math and needs no engine internals.
        m3Vec3 up = axisLocal.x * axisLocal.x < 0.9f ? (m3Vec3){1.0f, 0.0f, 0.0f}
                                                     : (m3Vec3){0.0f, 1.0f, 0.0f};
        m3Vec3 t1 = m3Normalize3(m3Cross3(axisLocal, up));
        m3Vec3 t2 = m3Cross3(axisLocal, t1);

        // Analytic Cdot with the FULL arms.
        m3Vec3 rA = m3RotateVec3(qA, laA);
        m3Vec3 rB = m3RotateVec3(qB, laB);
        m3Vec3 d = m3Sub3(m3Add3(pB, rB), m3Add3(pA, rA));
        m3Vec3 axis = m3RotateVec3(qA, axisLocal);
        m3Vec3 pY = m3RotateVec3(qA, t1);
        m3Vec3 pZ = m3RotateVec3(qA, t2);
        m3Vec3 vRel = m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
        double cdot[3] = {(double)m3Dot3(vRel, axis), (double)m3Dot3(vRel, pY),
                          (double)m3Dot3(vRel, pZ)};

        // Numeric Cdot by CENTRAL difference: the one-sided form
        // drowns in float cancellation; central kills the O(h) term
        // and h = 1e-3 keeps the round-off noise two decades down.
        const double h = 1.0e-3;
        m3Vec3 pAp = m3Add3(pA, m3MulSV3((float)h, vA));
        m3Vec3 pBp = m3Add3(pB, m3MulSV3((float)h, vB));
        m3Quat qAp = m3IntegrateRotation(qA, m3MulSV3((float)h, wA));
        m3Quat qBp = m3IntegrateRotation(qB, m3MulSV3((float)h, wB));
        m3Vec3 pAm = m3Sub3(pA, m3MulSV3((float)h, vA));
        m3Vec3 pBm = m3Sub3(pB, m3MulSV3((float)h, vB));
        m3Quat qAm = m3IntegrateRotation(qA, m3MulSV3(-(float)h, wA));
        m3Quat qBm = m3IntegrateRotation(qB, m3MulSV3(-(float)h, wB));

        m3Vec3 rAp = m3RotateVec3(qAp, laA);
        m3Vec3 rBp = m3RotateVec3(qBp, laB);
        m3Vec3 dp2 = m3Sub3(m3Add3(pBp, rBp), m3Add3(pAp, rAp));
        m3Vec3 axP = m3RotateVec3(qAp, axisLocal);
        m3Vec3 pYp = m3RotateVec3(qAp, t1);
        m3Vec3 pZp = m3RotateVec3(qAp, t2);
        m3Vec3 rAm = m3RotateVec3(qAm, laA);
        m3Vec3 rBm = m3RotateVec3(qBm, laB);
        m3Vec3 dm2 = m3Sub3(m3Add3(pBm, rBm), m3Add3(pAm, rAm));
        m3Vec3 axM = m3RotateVec3(qAm, axisLocal);
        m3Vec3 pYm = m3RotateVec3(qAm, t1);
        m3Vec3 pZm = m3RotateVec3(qAm, t2);
        double cp[3] = {(double)m3Dot3(dp2, axP), (double)m3Dot3(dp2, pYp),
                        (double)m3Dot3(dp2, pZp)};
        double cm[3] = {(double)m3Dot3(dm2, axM), (double)m3Dot3(dm2, pYm),
                        (double)m3Dot3(dm2, pZm)};
        for (int32_t k = 0; k < 3; ++k)
        {
            double fd = (cp[k] - cm[k]) / (2.0 * h);
            double err = fd - cdot[k];
            err = err < 0.0 ? -err : err;
            double scale = 1.0;
            double mag = cdot[k] < 0.0 ? -cdot[k] : cdot[k];
            scale = mag > 1.0 ? mag : 1.0;
            CHECK(err / scale < 5.0e-3, "the slide rows match the numerics");
            checked += 1;
        }
    }
    CHECK(checked == 60, "all sixty rows were checked");
}

static void TestElevator(void)
{
    // A motored vertical slider lifts a heavy platform to its upper
    // stop and holds it there against gravity: the elevator.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId shaft = NewBody(world, &gd);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId car = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, car, &sd, (m3Vec3){0.5f, 0.1f, 0.5f});

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_prismaticJoint;
    jd.bodyIdA = shaft;
    jd.bodyIdB = car;
    jd.localAnchorA = (m3Vec3){0.0f, 0.5f, 0.0f}; // the rail zero
    jd.localAnchorB = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.enableLimit = true;
    jd.lowerLimit = 0.0f;
    jd.upperLimit = 2.0f;
    jd.enableMotor = true;
    jd.motorSpeed = 1.0f;
    jd.maxMotorEffort = 200.0f; // plenty against gravity
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the elevator joint creates");

    StepN(world, 300); // five seconds: 2 m of travel plus the hold
    m3Pos3 p = m3Body_GetPosition(world, car);
    CHECK(p.y > 2.40 && p.y < 2.60, "the car rides to the upper stop and holds");
    CHECK(p.x > -0.01 && p.x < 0.01 && p.z > -0.01 && p.z < 0.01, "the rail holds sideways");
    m3Quat q = m3Body_GetRotation(world, car);
    CHECK(q.w > 0.999f, "the rotation lock holds the car level");

    // Cut the motor by destroying and recreating without it: simpler,
    // command the motor downward with a feeble force: gravity wins.
    m3DestroyWorld(world);
}

static void TestSliderGravityAlongRail(void)
{
    // A frictionless slider tilted 30 degrees: the car accelerates
    // along the rail at g sin(theta), the classic inclined plane
    // WITHOUT contact (the joint IS the plane). Analytic check.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId rail = NewBody(world, &gd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId car = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.2f};
    NewSphereShape(world, car, &sd, &ball);

    // Axis 30 degrees off horizontal in the xz-free xy plane.
    m3Vec3 axis = {0.8660254f, -0.5f, 0.0f};
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_prismaticJoint;
    jd.bodyIdA = rail;
    jd.bodyIdB = car;
    jd.localAxisA = axis;
    jd.localAxisB = axis;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the tilted slider creates");

    StepN(world, 60); // one second
    m3Vec3 v = m3Body_GetLinearVelocity(world, car);
    // v = g sin(30) t along the axis = 5 m/s; components (4.33, -2.5).
    float speed = sqrtf(m3Dot3(v, v));
    CHECK(speed > 4.7f && speed < 5.3f, "the car obeys g sin(theta) on the rail");
    CHECK(v.x > 4.0f && v.y < -2.2f, "the velocity points down the rail");
    m3DestroyWorld(world);
}

static double TwistOf(m3Quat q)
{
    double tw =
        (double)q.w < 0.0 ? atan2(-(double)q.z, -(double)q.w) : atan2((double)q.z, (double)q.w);
    return 2.0 * tw;
}

static double SwingOf(m3Quat q)
{
    double x = sqrt((double)q.z * (double)q.z + (double)q.w * (double)q.w);
    double y = sqrt((double)q.x * (double)q.x + (double)q.y * (double)q.y);
    return 2.0 * atan2(y, x);
}

static void TestTwistJacobianFiniteDifference(void)
{
    // The spherical joint's angle rows. With q = conj(qA) qB = (x, y, z,
    // w), the twist 2 atan2(z, w) has the gradient, in A's frame,
    // ((w y + z x), (z y - w x), (w^2 + z^2)) / (w^2 + z^2), and the
    // swing, the angle between the frame z axes, grows along their
    // normalized cross product. Claim: d(twist)/dt = dot(wB - wA,
    // J_twist) and d(swing)/dt = dot(wB - wA, swingAxis), both held to
    // central differences over random moderate poses (swing kept under
    // two radians, away from the antipodal singularity).
    uint64_t state = 0x5EED5EEDull;
    int32_t checked = 0;
    for (int32_t round = 0; round < 20; ++round)
    {
        m3Quat qA = RandQuat(&state);
        // qB = qA composed with a bounded random rotation.
        m3Vec3 rot = {(float)RandD(&state, -0.8, 0.8), (float)RandD(&state, -0.8, 0.8),
                      (float)RandD(&state, -0.8, 0.8)};
        m3Quat qB = m3IntegrateRotation(qA, rot);
        m3Vec3 wA = {(float)RandD(&state, -2.0, 2.0), (float)RandD(&state, -2.0, 2.0),
                     (float)RandD(&state, -2.0, 2.0)};
        m3Vec3 wB = {(float)RandD(&state, -2.0, 2.0), (float)RandD(&state, -2.0, 2.0),
                     (float)RandD(&state, -2.0, 2.0)};

        m3Vec3 coneAxis = m3RotateVec3(qA, (m3Vec3){0.0f, 0.0f, 1.0f});
        m3Vec3 twistAxis = m3RotateVec3(qB, (m3Vec3){0.0f, 0.0f, 1.0f});
        m3Vec3 swing = m3Cross3(coneAxis, twistAxis);
        float len2 = m3Dot3(swing, swing);
        if (len2 < 1.0e-6f)
        {
            continue; // aligned pose: the axis is arbitrary, skip
        }
        swing = m3MulSV3(1.0f / sqrtf(len2), swing);
        m3Quat conjA = {-qA.x, -qA.y, -qA.z, qA.w};
        m3Quat relQ = m3NormalizeQuat(m3MulQuat(conjA, qB));
        float denom = relQ.z * relQ.z + relQ.w * relQ.w;
        if (denom < 1.0e-6f)
        {
            continue; // near the swing singularity
        }
        m3Quat q = relQ.w < 0.0f ? (m3Quat){-relQ.x, -relQ.y, -relQ.z, -relQ.w} : relQ;
        m3Vec3 local = {(q.w * q.y + q.z * q.x) / denom, (q.z * q.y - q.w * q.x) / denom, 1.0f};
        m3Vec3 twistJac = m3RotateVec3(qA, local);

        double analyticTwist = (double)m3Dot3(m3Sub3(wB, wA), twistJac);
        double analyticSwing = (double)m3Dot3(m3Sub3(wB, wA), swing);

        const double h = 1.0e-3;
        m3Quat qAp = m3IntegrateRotation(qA, m3MulSV3((float)h, wA));
        m3Quat qBp = m3IntegrateRotation(qB, m3MulSV3((float)h, wB));
        m3Quat qAm = m3IntegrateRotation(qA, m3MulSV3(-(float)h, wA));
        m3Quat qBm = m3IntegrateRotation(qB, m3MulSV3(-(float)h, wB));
        m3Quat cAp = {-qAp.x, -qAp.y, -qAp.z, qAp.w};
        m3Quat cAm = {-qAm.x, -qAm.y, -qAm.z, qAm.w};
        m3Quat relP = m3NormalizeQuat(m3MulQuat(cAp, qBp));
        m3Quat relM = m3NormalizeQuat(m3MulQuat(cAm, qBm));

        double fdTwist = (TwistOf(relP) - TwistOf(relM)) / (2.0 * h);
        double fdSwing = (SwingOf(relP) - SwingOf(relM)) / (2.0 * h);

        double scaleT = analyticTwist < 0.0 ? -analyticTwist : analyticTwist;
        scaleT = scaleT > 1.0 ? scaleT : 1.0;
        double errT = fdTwist - analyticTwist;
        errT = errT < 0.0 ? -errT : errT;
        CHECK(errT / scaleT < 1.0e-2, "the twist gradient matches the numerics");

        double scaleS = analyticSwing < 0.0 ? -analyticSwing : analyticSwing;
        scaleS = scaleS > 1.0 ? scaleS : 1.0;
        double errS = fdSwing - analyticSwing;
        errS = errS < 0.0 ? -errS : errS;
        CHECK(errS / scaleS < 1.0e-2, "the swing axis row matches the numerics");
        checked += 1;
    }
    CHECK(checked >= 15, "enough non-degenerate poses were checked");
}

// An arm on a spherical joint, sticking out horizontal from its socket, with
// its 30-degree cone on or off. Dropped from horizontal (swing 90 degrees),
// it must be caught by the cone and settle hanging inside it, never outside
// the band.
static m3World* ShoulderWorld(bool cone, m3BodyId* armOut, m3JointId* shoulderOut)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    // The socket frame points its z DOWN so the cone opens downward.
    m3BodyId socket = NewBody(world, &ad);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.6, 0.0, 0.0}; // arm sticks out horizontal
    bd.linearDamping = 1.0f;
    bd.angularDamping = 1.0f;
    *armOut = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{-0.5f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, 0.1f};
    NewCapsuleShape(world, *armOut, &sd, &capsule);

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = socket;
    jd.bodyIdB = *armOut;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.6f, 0.0f, 0.0f};
    // The cone axis points along +x, HORIZONTAL: gravity wants the
    // arm hanging straight down (swing 90 degrees), the cone stops
    // the droop at 30 degrees below the horizon. Pointing the cone
    // downward would make hanging the cone CENTER and catch nothing
    // (the first version of this test learned that the hard way).
    jd.localAxisA = (m3Vec3){1.0f, 0.0f, 0.0f};
    jd.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f}; // the arm's long axis
    jd.enableCone = cone;
    jd.coneAngle = 0.5236f; // 30 degrees
    *shoulderOut = NewJoint(world, &jd);
    CHECK(m3Joint_IsValid(world, *shoulderOut), "the shoulder creates");
    return world;
}

// The sine of the arm's angle below the horizon.
static double ArmDroop(const m3World* world, m3BodyId arm, double* lengthOut)
{
    m3Pos3 p = m3Body_GetPosition(world, arm);
    double len = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    *lengthOut = len;
    return -p.y / len;
}

static void TestShoulderCone(void)
{
    m3BodyId arm;
    m3JointId shoulder;
    m3World* world = ShoulderWorld(true, &arm, &shoulder);
    StepN(world, 480);
    double len;
    double droop = ArmDroop(world, arm, &len);
    CHECK(len > 0.55 && len < 0.65, "the arm still hangs from the socket");
    CHECK(droop > 0.44 && droop < 0.56, "the cone catches the droop at thirty degrees");
    CHECK(m3Body_GetPosition(world, arm).x > 0.4, "the arm stays out along the cone axis");
    m3DestroyWorld(world);
}

// The same shoulder built with its cone off hangs straight down; the cone
// switched on at run time lifts the arm back into it, and switched off
// again lets it fall.
static void TestShoulderConeAtRunTime(void)
{
    m3BodyId arm;
    m3JointId shoulder;
    m3World* world = ShoulderWorld(false, &arm, &shoulder);
    StepN(world, 480);
    double len;
    CHECK(ArmDroop(world, arm, &len) > 0.95, "without its cone the arm hangs straight down");
    m3SphericalJoint_SetConeAngle(world, shoulder, 0.5236f);
    m3SphericalJoint_EnableCone(world, shoulder, true);
    StepN(world, 480);
    double droop = ArmDroop(world, arm, &len);
    CHECK(len > 0.55 && len < 0.65, "the arm still hangs from the socket");
    CHECK(droop > 0.44 && droop < 0.56, "the cone switched on lifts it to thirty degrees");
    m3SphericalJoint_EnableCone(world, shoulder, false);
    StepN(world, 480);
    CHECK(ArmDroop(world, arm, &len) > 0.95, "switched off, it lets the arm fall again");
    CHECK(m3World_GetCounters(world).misuseCount == 0, "no call was refused");
    m3DestroyWorld(world);
}

// The cone calls' rules: the angle is finite and never set negative, the
// switch refuses a negative angle a def left, and every other joint type
// refuses the calls and counts a misuse each.
static void TestConeCalls(void)
{
    m3BodyId arm;
    m3JointId shoulder;
    m3World* world = ShoulderWorld(false, &arm, &shoulder);
    CHECK(!m3SphericalJoint_IsConeEnabled(world, shoulder) &&
              m3SphericalJoint_GetConeAngle(world, shoulder) == 0.5236f,
          "the def's cone reads back, switched off");
    m3SphericalJoint_SetConeAngle(world, shoulder, 0.4f);
    CHECK(m3SphericalJoint_GetConeAngle(world, shoulder) == 0.4f &&
              !m3SphericalJoint_IsConeEnabled(world, shoulder),
          "an angle is taken with the cone off");
    m3SphericalJoint_EnableCone(world, shoulder, true);
    CHECK(m3SphericalJoint_IsConeEnabled(world, shoulder), "the cone switches on");
    CHECK(m3World_GetCounters(world).misuseCount == 0, "and nothing was refused");

    static const float bad[] = {-0.1f, NAN, INFINITY, -INFINITY};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
    {
        uint64_t misuse = m3World_GetCounters(world).misuseCount;
        m3SphericalJoint_SetConeAngle(world, shoulder, bad[i]);
        CHECK(m3World_GetCounters(world).misuseCount == misuse + 1u &&
                  m3SphericalJoint_GetConeAngle(world, shoulder) == 0.4f,
              "a negative or non-finite angle is refused and changes nothing");
    }
    m3SphericalJoint_EnableCone(world, shoulder, false);
    uint64_t misuse = m3World_GetCounters(world).misuseCount;
    m3SphericalJoint_SetConeAngle(world, shoulder, -0.1f);
    CHECK(m3World_GetCounters(world).misuseCount == misuse + 1u, "with the cone off too");

    // The body sleeps; a cone call wakes it.
    m3Body_SetAwake(world, arm, false);
    m3SphericalJoint_SetConeAngle(world, shoulder, 0.3f);
    CHECK(m3Body_IsAwake(world, arm), "a cone call wakes the joint's bodies");
    m3DestroyWorld(world);

    // A def may leave a negative angle while its cone is off; the switch
    // refuses to turn the cone on over it, and takes it once moved.
    m3WorldDef wd = m3DefaultWorldDef();
    world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId socket = NewBody(world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, -1.0, 0.0};
    m3BodyId bob = NewBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.bodyIdA = socket;
    jd.bodyIdB = bob;
    jd.coneAngle = -0.5f;
    m3JointId ball = NewJoint(world, &jd);
    CHECK(m3Joint_IsValid(world, ball) && m3SphericalJoint_GetConeAngle(world, ball) == -0.5f,
          "a def leaves a negative angle with its cone off");
    m3SphericalJoint_EnableCone(world, ball, true);
    CHECK(!m3SphericalJoint_IsConeEnabled(world, ball) &&
              m3World_GetCounters(world).misuseCount == 1u,
          "the switch refuses to turn the cone on over it");
    m3SphericalJoint_EnableCone(world, ball, false);
    CHECK(m3World_GetCounters(world).misuseCount == 1u, "switching it off again is taken");
    m3SphericalJoint_SetConeAngle(world, ball, 0.2f);
    m3SphericalJoint_EnableCone(world, ball, true);
    CHECK(m3SphericalJoint_IsConeEnabled(world, ball) &&
              m3World_GetCounters(world).misuseCount == 1u,
          "once moved, the cone switches on");

    // Every other type refuses the four calls, and a stale id too.
    jd.type = m3_revoluteJoint;
    jd.coneAngle = 0.0f;
    m3JointId hinge = NewJoint(world, &jd);
    m3JointId gone = NewJoint(world, &jd);
    DestroyJoint(world, gone);
    m3JointId others[2] = {hinge, gone};
    for (int32_t i = 0; i < 2; ++i)
    {
        misuse = m3World_GetCounters(world).misuseCount;
        m3SphericalJoint_EnableCone(world, others[i], true);
        m3SphericalJoint_SetConeAngle(world, others[i], 0.3f);
        bool on = m3SphericalJoint_IsConeEnabled(world, others[i]);
        float angle = m3SphericalJoint_GetConeAngle(world, others[i]);
        CHECK(!on && angle == 0.0f && m3World_GetCounters(world).misuseCount == misuse + 4u,
              "a hinge or a stale id refuses each cone call, and the getters read false and zero");
    }
    CHECK(!m3Joint_IsLimitEnabled(world, hinge) && !m3Joint_IsMotorEnabled(world, hinge),
          "and the hinge keeps its own switches");
    m3DestroyWorld(world);
}

// Records the shoulder's cone moved to 0.6 and switched on, with every 0.6
// in the tape's records set to angle, and tells whether a fresh shoulder
// replays it.
static bool ReplaysCone(float angle)
{
    static uint8_t tape[16384];
    m3BodyId arm;
    m3JointId shoulder;
    m3World* world = ShoulderWorld(false, &arm, &shoulder);
    CHECK(StartJournal(world, tape, (int32_t)sizeof(tape)), "the shoulder tape records");
    m3SphericalJoint_SetConeAngle(world, shoulder, 0.6f);
    m3SphericalJoint_EnableCone(world, shoulder, true);
    StepN(world, 30);
    int64_t bytes = StopJournal(world);
    const float recorded = 0.6f;
    int32_t patched = 0;
    for (int32_t i = 16; i + 4 <= bytes; ++i) // past the tape's 16-byte header
    {
        if (memcmp(tape + i, &recorded, 4) == 0)
        {
            memcpy(tape + i, &angle, 4);
            ++patched;
        }
    }
    CHECK(patched == 2, "the angle sits in the two cone records");
    m3World* replayed = ShoulderWorld(false, &arm, &shoulder);
    bool ok = ReplayJournal(replayed, tape, bytes);
    if (ok && angle == recorded)
    {
        CHECK(m3World_Hash(replayed) == m3World_Hash(world), "the cone replays bit-identical");
    }
    m3DestroyWorld(replayed);
    m3DestroyWorld(world);
    return ok;
}

static void TestConeReplay(void)
{
    CHECK(ReplaysCone(0.6f), "a recorded cone replays");
    CHECK(!ReplaysCone(-1.0f), "a recorded cone switched on over a negative angle is refused");
    CHECK(!ReplaysCone(NAN), "and so is a NaN angle");
}

static void TestTwistClamp(void)
{
    // A spinning body on a twist-limited spherical joint: the twist
    // angle must stay inside the band while the spin bleeds into the
    // stops (gravity off, aligned frames, pure twist geometry).
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    m3BodyId anchor = NewBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId spinner = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, spinner, &sd, (m3Vec3){0.4f, 0.1f, 0.4f});

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = spinner;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.enableLimit = true;
    jd.lowerLimit = -0.5f;
    jd.upperLimit = 0.5f;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the twist-limited joint creates");
    m3Body_SetAngularVelocity(world, spinner, (m3Vec3){0.0f, 0.0f, 6.0f});

    double maxTwist = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        if (i == 120)
        {
            m3Body_SetAngularVelocity(world, spinner, (m3Vec3){0.0f, 0.0f, -6.0f});
        }
        m3Quat q = m3Body_GetRotation(world, spinner);
        double tw = TwistOf(q);
        double mag = tw < 0.0 ? -tw : tw;
        maxTwist = mag > maxTwist ? mag : maxTwist;
    }
    CHECK(maxTwist < 0.55, "the twist stops hold from both directions");
    CHECK(maxTwist > 0.42, "the stops were actually reached");
    m3DestroyWorld(world);
}

// Coverage: the joint frame builder (quat whose z-axis is
// the hinge axis) branches on the largest matrix diagonal; axes
// pointing down each principal direction walk all four branches.
static void TestFrameBuilderBranches(void)
{
    m3Vec3 axes[6] = {{1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                      {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f}};
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        def.jointCapacity = 8;
        m3World* world = NewWorld(&def);
        m3BodyDef ad = m3DefaultBodyDef();
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        for (int32_t i = 0; i < 6; ++i)
        {
            ad.position = (m3Pos3){2.0 * (double)i, 2.0, 0.0};
            m3BodyId anchor = NewBody(world, &ad);
            bd.position = (m3Pos3){2.0 * (double)i, 1.0, 0.0};
            m3BodyId swinger = NewBody(world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.2f};
            NewSphereShape(world, swinger, &sd, &ball);
            m3JointDef jd = m3DefaultJointDef();
            jd.type = m3_revoluteJoint;
            jd.bodyIdA = anchor;
            jd.bodyIdB = swinger;
            jd.localAnchorB = (m3Vec3){0.0f, 1.0f, 0.0f};
            jd.localAxisA = axes[i];
            jd.localAxisB = axes[i];
            CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)),
                  "every principal axis builds a frame");
        }
        for (int32_t i = 0; i < 60; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "all frame branches are bit-deterministic");
}

// The weld and the rope.
static void TestWeldActsAsOneBody(void)
{
    // Two crates welded at an offset must move as one rigid body:
    // after a tumble and a kick, the relative pose drift stays in
    // the solver's soft band. Twins for determinism.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        def.jointCapacity = 4;
        m3World* world = NewWorld(&def);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        NewPlaneShape(world, ground, &sd, &floor);

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 3.0, 0.0};
        m3BodyId a = NewBody(world, &bd);
        NewBoxShape(world, a, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        bd.position = (m3Pos3){0.9, 3.0, 0.0};
        m3BodyId b = NewBody(world, &bd);
        NewBoxShape(world, b, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});

        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_weldJoint;
        jd.bodyIdA = a;
        jd.bodyIdB = b;
        jd.localAnchorA = (m3Vec3){0.45f, 0.0f, 0.0f};
        jd.localAnchorB = (m3Vec3){-0.45f, 0.0f, 0.0f};
        CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the weld creates (axes ignored)");

        m3Body_SetAngularVelocity(world, a, (m3Vec3){3.0f, 5.0f, 2.0f});
        m3Body_SetLinearVelocity(world, a, (m3Vec3){2.0f, 3.0f, 1.0f});
        for (int32_t i = 0; i < 300; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Pos3 pa = m3Body_GetPosition(world, a);
        m3Pos3 pb = m3Body_GetPosition(world, b);
        double dx = pb.x - pa.x;
        double dy = pb.y - pa.y;
        double dz = pb.z - pa.z;
        double gap = sqrt(dx * dx + dy * dy + dz * dz);
        CHECK(gap > 0.85 && gap < 0.95, "the welded gap holds through the tumble");
        m3Quat qa = m3Body_GetRotation(world, a);
        m3Quat qb = m3Body_GetRotation(world, b);
        float dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
        float align = dot < 0.0f ? -dot : dot;
        CHECK(align > 0.999f, "the welded rotations stay aligned");
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the weld is bit-deterministic");
}

static void TestRopeAndRod(void)
{
    // A rope (range 0..2) lets the ball free-fall until taut, then
    // holds; a rod (2..2) holds both ways. Both lengths analytic.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){0.0, 10.0, 0.0};
    m3BodyId anchor = NewBody(world, &ad);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 9.5, 0.0}; // slack: 0.5 below, rope is 2
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.2f};
    NewSphereShape(world, ball, &sd, &sphere);

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = ball;
    jd.minLength = 0.0f;
    jd.maxLength = 2.0f;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the rope creates");

    // Ten steps of slack: free fall matches gravity analytically
    // (the rope must not touch a slack body).
    for (int32_t i = 0; i < 10; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(world, ball);
    CHECK(v.y < -1.55f && v.y > -1.72f, "a slack rope leaves free fall alone");
    for (int32_t i = 0; i < 290; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(world, ball);
    double hang = 10.0 - p.y;
    CHECK(hang > 1.95 && hang < 2.05, "the taut rope hangs at its length");

    // The rod: constrained BOTH ways; launch the ball upward and the
    // rod reels it back: distance never leaves the band for long,
    // and at rest it is exact.
    m3BodyDef bd2 = m3DefaultBodyDef();
    bd2.type = m3_dynamicBody;
    bd2.position = (m3Pos3){4.0, 8.0, 0.0};
    m3BodyId bob = NewBody(world, &bd2);
    NewSphereShape(world, bob, &sd, &sphere);
    m3BodyDef ad2 = m3DefaultBodyDef();
    ad2.position = (m3Pos3){4.0, 10.0, 0.0};
    m3BodyId anchor2 = NewBody(world, &ad2);
    m3JointDef rod = m3DefaultJointDef();
    rod.type = m3_distanceJoint;
    rod.bodyIdA = anchor2;
    rod.bodyIdB = bob;
    rod.minLength = 2.0f;
    rod.maxLength = 2.0f;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &rod)), "the rod creates");
    m3Body_SetLinearVelocity(world, bob, (m3Vec3){0.0f, 6.0f, 0.0f}); // push UP
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    p = m3Body_GetPosition(world, bob);
    double dx = p.x - 4.0;
    double dy = p.y - 10.0;
    double rodLen = sqrt(dx * dx + dy * dy + p.z * p.z);
    CHECK(rodLen > 1.95 && rodLen < 2.05, "the rod holds its length both ways");
    m3DestroyWorld(world);
}

static void TestDistanceSpring(void)
{
    // The soft spring (motor-field reuse): no gravity, a ball
    // displaced off the rest length oscillates through it and the
    // damping settles it back to rest.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){0.0, 5.0, 0.0};
    m3BodyId anchor = NewBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){3.5, 5.0, 0.0}; // rest is 2: stretched 1.5
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.2f};
    NewSphereShape(world, ball, &sd, &sphere);

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = ball;
    jd.minLength = 0.0f;
    jd.maxLength = 5.0f; // wide bounds: the spring owns the middle
    jd.enableSpring = true;
    jd.springHertz = 1.5f;
    jd.springDampingRatio = 0.2f;
    jd.length = 2.0f;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the spring creates");

    int32_t crossings = 0;
    double previous = 3.5 - 2.0;
    for (int32_t i = 0; i < 600; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 p = m3Body_GetPosition(world, ball);
        double excess = (p.x - 0.0) - 2.0; // distance minus rest
        if ((excess > 0.0) != (previous > 0.0))
        {
            crossings += 1;
        }
        previous = excess;
    }
    CHECK(crossings >= 2, "the spring oscillates through its rest length");
    m3Pos3 p = m3Body_GetPosition(world, ball);
    CHECK(p.x > 1.7 && p.x < 2.3, "the damped spring settles near rest");
    m3DestroyWorld(world);
}

static void TestNewJointContracts(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId a = NewBody(world, &bd);
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId b = NewBody(world, &bd);

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.enableLimit = true; // the range is the lengths, not a limit
    jd.lowerLimit = 0.5f;
    jd.upperLimit = 2.0f;
    jd.maxLength = 2.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a distance joint refuses a limit");
    jd.enableLimit = false;
    jd.minLength = 2.5f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "an unordered length range refuses");
    jd.minLength = -1.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a negative minimum length refuses");
    jd.minLength = 0.5f;
    jd.enableMotor = true;
    jd.motorSpeed = 1.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a distance joint refuses a motor");
    jd.enableMotor = false;
    jd.enableSpring = true;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a spring with zero hertz refuses");
    jd.springHertz = 2.0f;
    jd.length = -1.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a negative rest length refuses");
    jd.length = 3.0f; // past the range: the range holds what the spring pulls toward
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the honest rope creates");
    m3DestroyWorld(world);
}

// The generic 6-DOF reproduces its ancestors in behavioral
// bands (bitwise agreement is neither expected nor sought: the row
// structures differ by design).
static m3JointDef GenericBase(m3BodyId a, m3BodyId b)
{
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_genericJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    for (int32_t k = 0; k < 3; ++k)
    {
        jd.genericLinear[k] = (uint8_t)m3_axisLocked;
        jd.genericAngular[k] = (uint8_t)m3_axisFree;
    }
    return jd;
}

static void TestGenericAsSpherical(void)
{
    // All linear locked, all angular free = a ball joint: the
    // pendulum swings inside the spherical energy band and the pin
    // holds.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){0.0, 6.0, 0.0};
    m3BodyId anchor = NewBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.0, 6.0, 0.0};
    m3BodyId bob = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.2f};
    NewSphereShape(world, bob, &sd, &ball);
    m3JointDef jd = GenericBase(anchor, bob);
    jd.localAnchorB = (m3Vec3){-1.0f, 0.0f, 0.0f};
    uint64_t misuse = m3World_GetCounters(world).misuseCount;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the generic ball creates");
    CHECK(m3World_GetCounters(world).misuseCount == misuse,
          "and a valid def is never counted as misuse");
    double maxSpeed2 = 0.0;
    double maxPinDrift2 = 0.0;
    for (int32_t i = 0; i < 480; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Vec3 v = m3Body_GetLinearVelocity(world, bob);
        double s2 = (double)(v.x * v.x + v.y * v.y + v.z * v.z);
        if (s2 > maxSpeed2)
        {
            maxSpeed2 = s2;
        }
        m3Pos3 p = m3Body_GetPosition(world, bob);
        m3Quat q = m3Body_GetRotation(world, bob);
        // The pinned point: bob position plus rotated (-1, 0, 0)
        // must stay at the anchor.
        m3Vec3 arm = m3RotateVec3(q, (m3Vec3){-1.0f, 0.0f, 0.0f});
        double dx = p.x + (double)arm.x - 0.0;
        double dy = p.y + (double)arm.y - 6.0;
        double dz = p.z + (double)arm.z - 0.0;
        double d2 = dx * dx + dy * dy + dz * dz;
        if (d2 > maxPinDrift2)
        {
            maxPinDrift2 = d2;
        }
    }
    CHECK(maxSpeed2 > 15.0 && maxSpeed2 < 21.0, "the generic pendulum stays in the energy band");
    CHECK(maxPinDrift2 < 0.005, "the generic pin holds like the spherical's");
    m3DestroyWorld(world);
}

static void TestGenericAsHingeWithMotorAndLimits(void)
{
    // Angular z limited with x and y locked = a hinge with range;
    // the motor drives it into the upper stop and holds.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef fd = m3DefaultBodyDef();
    fd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId frame = NewBody(world, &fd);
    m3BodyDef dd = m3DefaultBodyDef();
    dd.type = m3_dynamicBody;
    dd.position = (m3Pos3){1.0, 2.0, 0.0};
    m3BodyId door = NewBody(world, &dd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, door, &sd, (m3Vec3){0.9f, 0.5f, 0.05f});

    m3JointDef jd = GenericBase(frame, door);
    jd.localAnchorB = (m3Vec3){-1.0f, 0.0f, 0.0f};
    jd.genericAngular[0] = (uint8_t)m3_axisLocked;
    jd.genericAngular[1] = (uint8_t)m3_axisLocked;
    jd.genericAngular[2] = (uint8_t)m3_axisLimited;
    jd.genericAngularLower[2] = -0.5f;
    jd.genericAngularUpper[2] = 1.0f;
    jd.genericMotorAxis = 5; // angular z
    jd.motorSpeed = 2.0f;
    jd.maxMotorEffort = 50.0f;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the generic hinge creates");
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Quat q = m3Body_GetRotation(world, door);
    float angle = 2.0f * atan2f(q.z, q.w);
    CHECK(angle > 0.93f && angle < 1.07f, "the motor parks the door at the upper stop");
    CHECK(q.x * q.x + q.y * q.y < 0.001f, "the locked axes admit no wobble");
    m3DestroyWorld(world);
}

static void TestGenericAsSlider(void)
{
    // Linear z limited with the rest locked, all angular locked =
    // a slider on the frame's z axis; gravity along the axis rests
    // the cart on the lower stop.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){0.0, 5.0, 0.0};
    m3BodyId rail = NewBody(world, &rd);
    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){0.0, 5.0, 0.0};
    m3BodyId cart = NewBody(world, &cd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, cart, &sd, (m3Vec3){0.2f, 0.2f, 0.2f});

    m3JointDef jd = GenericBase(rail, cart);
    // The rail's z axis points down the world y: axis (0, 1, 0).
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.genericLinear[2] = (uint8_t)m3_axisLimited; // along the rail
    jd.genericLinearLower[2] = -2.0f;
    jd.genericLinearUpper[2] = 2.0f;
    for (int32_t k = 0; k < 3; ++k)
    {
        jd.genericAngular[k] = (uint8_t)m3_axisLocked;
    }
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the generic slider creates");
    for (int32_t i = 0; i < 300; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(world, cart);
    CHECK(p.y > 2.9 && p.y < 3.1, "gravity rests the cart on the lower stop");
    CHECK(p.x * p.x + p.z * p.z < 0.001, "the perpendicular locks hold the rail line");

    // The new generic state rides the rollback like everything else.
    int64_t bytes = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)bytes);
    CHECK(Snapshot(world, snap, bytes) == bytes, "the generic snapshot writes");
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t ahead = m3World_Hash(world);
    CHECK(Restore(world, snap, bytes), "the generic restore lands");
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) == ahead, "the generic joint rolls back bit-exact");
    free(snap);
    m3DestroyWorld(world);
}

static void TestGenericContracts(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId a = NewBody(world, &bd);
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId b = NewBody(world, &bd);

    m3JointDef jd = GenericBase(a, b);
    jd.genericAngular[0] = (uint8_t)m3_axisLimited;
    jd.genericAngular[1] = (uint8_t)m3_axisLimited;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)),
          "two limited angular axes refuse (the v1 rule)");
    jd = GenericBase(a, b);
    jd.genericAngular[0] = (uint8_t)m3_axisLimited;
    jd.genericAngular[1] = (uint8_t)m3_axisLocked;
    jd.genericAngular[2] = (uint8_t)m3_axisFree;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)),
          "a mixed locked-free pair beside a limited axis refuses");
    jd = GenericBase(a, b);
    jd.genericMotorAxis = 1; // linear y, which is locked
    jd.motorSpeed = 1.0f;
    jd.maxMotorEffort = 10.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a motor on a locked axis refuses");
    jd = GenericBase(a, b);
    jd.genericLinear[0] = 7;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "an unknown mode refuses");
    jd = GenericBase(a, b);
    jd.genericLinear[2] = (uint8_t)m3_axisLimited;
    jd.genericLinearLower[2] = 1.0f;
    jd.genericLinearUpper[2] = -1.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "inverted generic limits refuse");
    jd = GenericBase(a, b);
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the plain generic ball creates");
    m3DestroyWorld(world);
}

// A ball-and-socket joint holds a hung cube at every mass the API takes.
// Past about 1e15 kg the point block's determinant, a product of three
// inverse masses, underflows to zero though the block is well
// conditioned; it is solved in double, as a subnormal one is.
static void TestHeavyBodyHangs(void)
{
    const float densities[4] = {1.0f, 1.0e5f, 1.0e6f, 1.0e6f};
    const float halves[4] = {1000.0f, 1000.0f, 2154.0f, 1.0e4f}; // 8e9 to 8e18 kg
    for (int32_t i = 0; i < 4; ++i)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        m3World* world = NewWorld(&wd);
        m3BodyDef bd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &bd);
        bd.type = m3_dynamicBody;
        m3BodyId cube = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        sd.density = densities[i];
        NewBoxShape(world, cube, &sd, (m3Vec3){halves[i], halves[i], halves[i]});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_sphericalJoint;
        jd.bodyIdA = ground;
        jd.bodyIdB = cube;
        CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the socket builds");
        for (int32_t k = 0; k < 60; ++k)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        char what[96];
        snprintf(what, sizeof(what), "a %.0e kg cube stays on its socket",
                 (double)m3Body_GetMass(world, cube));
        CHECK(fabs(m3Body_GetPosition(world, cube).y) < 1.0e-3, what);
        m3DestroyWorld(world);
    }
}

// A weld holds a cube by the midpoint of a top edge, so gravity turns it
// about the weld unless the turn block holds. That block's determinant is
// a product of three inverse inertias: subnormal for a 100 m cube of
// density 100 (8e8 kg), and zero from a 1 km cube of density 1 (8e9 kg)
// on, which let the cube swing. Both are solved at the block's scale.
static void TestHeavyBodyWelds(void)
{
    const float densities[3] = {100.0f, 1.0f, 1.0e6f};
    const float halves[3] = {100.0f, 1000.0f, 1.0e4f}; // 8e8, 8e9 and 8e18 kg
    for (int32_t i = 0; i < 3; ++i)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        m3World* world = NewWorld(&wd);
        m3BodyDef bd = m3DefaultBodyDef();
        m3BodyId ground = NewBody(world, &bd);
        bd.type = m3_dynamicBody;
        m3BodyId cube = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        sd.density = densities[i];
        NewBoxShape(world, cube, &sd, (m3Vec3){halves[i], halves[i], halves[i]});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_weldJoint;
        jd.bodyIdA = ground;
        jd.bodyIdB = cube;
        jd.localAnchorA = (m3Vec3){-halves[i], halves[i], 0.0f};
        jd.localAnchorB = jd.localAnchorA;
        CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the weld builds");
        for (int32_t k = 0; k < 60; ++k)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        char what[96];
        snprintf(what, sizeof(what), "a %.0e kg cube stays on its weld",
                 (double)m3Body_GetMass(world, cube));
        CHECK(fabs(m3Body_GetPosition(world, cube).y) < 1.0e-3, what);
        m3DestroyWorld(world);
    }
}

// The worst distance, over 1 s, between the ground's pivot and the anchor
// of a 65 kg ball of radius 0.25 on an arm of length arm, the angle
// degrees below the x axis in the xy plane: across the slider's axis, z,
// for the slider, and in full for the others.
// freeTurn frees that angular axis of a generic joint, -1 none.
static double PivotDriftFree(int32_t type, float degrees, float arm, int32_t freeTurn)
{
    float angle = degrees * M3_PI / 180.0f;
    m3Vec3 offset = {arm * cosf(angle), -arm * sinf(angle), 0.0f};
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){(double)offset.x, (double)offset.y, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 1000.0f;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.25f};
    NewSphereShape(world, ball, &sd, &sphere);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = type;
    jd.bodyIdA = ground;
    jd.bodyIdB = ball;
    jd.localAnchorB = m3MulSV3(-1.0f, offset);
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    if (freeTurn >= 0)
    {
        jd.genericAngular[freeTurn] = m3_axisFree;
    }
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the long arm joint builds");
    double worst = 0.0;
    for (int32_t k = 0; k < 60; ++k)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 at = m3Body_GetWorldPoint(world, ball, jd.localAnchorB);
        double z = type == m3_prismaticJoint ? 0.0 : (double)at.z;
        worst = fmax(worst, sqrt((double)at.x * at.x + (double)at.y * at.y + z * z));
    }
    m3DestroyWorld(world);
    return worst;
}

static double PivotDrift(int32_t type, float degrees, float arm)
{
    return PivotDriftFree(type, degrees, arm, -1);
}

// On a 1000 m arm the ball's mass * arm^2 / inertia is 4e7: each float
// entry of the point block rounds the mass away, and off the axes its
// determinant cancels to zero, which let the ball fall freely, or to
// noise, which flung it some 2 km. The block is solved in double then,
// and the socket and the hinge hold their pivot at every angle.
static void TestLongArmHolds(void)
{
    const int32_t types[2] = {m3_sphericalJoint, m3_revoluteJoint};
    const float degrees[3] = {30.0f, 45.0f, 60.0f};
    for (int32_t t = 0; t < 2; ++t)
    {
        for (int32_t d = 0; d < 3; ++d)
        {
            char what[96];
            snprintf(what, sizeof(what), "a %s on a 1000 m arm at %.0f degrees holds its pivot",
                     t == 0 ? "socket" : "hinge", (double)degrees[d]);
            CHECK(PivotDrift(types[t], degrees[d], 1000.0f) < 0.01, what);
        }
    }
}

// A joint's coupled rows on a body far from its center of mass once ran
// one block after another, and each undid most of the other's work: a
// slider on a 10 m arm sagged 0.54 m off its axis in a second, on a
// 100 m arm 2.4 m. Solved as one block, an offset anchor holds like one
// at the center.
static void TestOffsetAnchorsHold(void)
{
    // The generic joint locks any subset of its six axes: all six make it
    // a weld, all but the turn about z a hinge, and it holds like them.
    const int32_t types[5] = {m3_prismaticJoint, m3_revoluteJoint, m3_weldJoint, m3_genericJoint,
                              m3_genericJoint};
    const int32_t freeTurns[5] = {-1, -1, -1, -1, 2};
    const char* names[5] = {"slider", "hinge", "weld", "generic weld", "generic hinge"};
    const float arms[3] = {1.0f, 10.0f, 100.0f};
    for (int32_t t = 0; t < 5; ++t)
    {
        for (int32_t a = 0; a < 3; ++a)
        {
            char what[96];
            snprintf(what, sizeof(what), "a %s on a %.0f m arm holds its anchor", names[t],
                     (double)arms[a]);
            double drift = PivotDriftFree(types[t], 0.0f, arms[a], freeTurns[t]);
            if (!(drift < 1.0e-3))
            {
                printf("  %s: %.3g m\n", what, drift);
            }
            CHECK(drift < 1.0e-3, what);
        }
    }
}

int main(void)
{
    TestGenericAsSpherical();
    TestGenericAsHingeWithMotorAndLimits();
    TestGenericAsSlider();
    TestGenericContracts();
    TestWeldActsAsOneBody();
    TestRopeAndRod();
    TestDistanceSpring();
    TestNewJointContracts();
    TestFrameBuilderBranches();
    TestPendulum();
    TestFarErrorClosesAtSpeedLimit();
    TestChainSettlesAndSleeps();
    TestJointContracts();
    TestDoorSwings();
    TestDoorLimits();
    TestDoorMotor();
    TestPrismaticJacobianFiniteDifference();
    TestElevator();
    TestSliderGravityAlongRail();
    TestTwistJacobianFiniteDifference();
    TestShoulderCone();
    TestShoulderConeAtRunTime();
    TestConeCalls();
    TestConeReplay();
    TestTwistClamp();
    TestHeavyBodyHangs();
    TestHeavyBodyWelds();
    TestLongArmHolds();
    TestOffsetAnchorsHold();
    if (s_failures == 0)
    {
        printf("test_joints: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
