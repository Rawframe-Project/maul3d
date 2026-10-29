// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint runtime gate: limits and motors bind mid-run,
// the collide toggle separates an overlapped pair, readback lands
// in the m*g band on a hanging load, breakage fires exactly once
// at the analytic threshold, and hostile runtime values refuse.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m3World* SmallWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    return NewWorld(&def);
}

static m3BodyId Box(m3World* world, m3Pos3 at, float half, int dynamic)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = dynamic ? m3_dynamicBody : m3_staticBody;
    bd.position = at;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, body, &sd, (m3Vec3){half, half, half});
    return body;
}

static void StepN(m3World* world, int32_t n)
{
    for (int32_t i = 0; i < n; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static void TestRuntimeMotorAndLimits(void)
{
    // A hinge door: the runtime motor swings it, the runtime limit
    // stops the swing at the analytic angle.
    m3World* world = SmallWorld();
    m3BodyId post = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId door = Box(world, (m3Pos3){0.8, 2.0, 0.0}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = post;
    jd.bodyIdB = door;
    jd.localAnchorA = (m3Vec3){0.4f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.4f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    m3JointId hinge = NewJoint(world, &jd);
    StepN(world, 30);
    CHECK(fabsf(m3Joint_GetAngle(world, hinge)) < 0.05f, "the unmotored door hangs still");

    m3Joint_SetMotorSpeed(world, hinge, 2.0f);
    m3Joint_SetMaxMotorEffort(world, hinge, 100.0f);
    m3Joint_EnableMotor(world, hinge, true);
    StepN(world, 60);
    float swung = m3Joint_GetAngle(world, hinge);
    CHECK(fabsf(swung) > 0.8f, "the runtime motor swings the door");

    m3Joint_SetMotorSpeed(world, hinge, -2.0f);
    m3Joint_SetMaxMotorEffort(world, hinge, 100.0f);
    m3Joint_EnableMotor(world, hinge, true);
    m3Joint_SetLimits(world, hinge, -0.5f, 0.5f);
    m3Joint_EnableLimit(world, hinge, true);
    StepN(world, 120);
    float held = m3Joint_GetAngle(world, hinge);
    CHECK(held > -0.65f && held < -0.35f, "the runtime limit holds the swing at the stop");
    m3DestroyWorld(world);
}

static double CenterDistance(const m3World* world, m3BodyId a, m3BodyId b)
{
    m3Pos3 pa = m3Body_GetPosition(world, a);
    m3Pos3 pb = m3Body_GetPosition(world, b);
    double dx = pb.x - pa.x;
    double dy = pb.y - pa.y;
    double dz = pb.z - pa.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static void TestCollideToggle(void)
{
    // Two overlapped jointed boxes: connected pairs skip contacts
    // by default, so they interpenetrate in peace; the runtime
    // toggle turns contacts on and the pair pushes apart.
    m3World* world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sg = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sg, &fl);
    m3BodyId a = Box(world, (m3Pos3){0.0, 0.5, 0.0}, 0.5f, 1);
    m3BodyId b = Box(world, (m3Pos3){0.6, 0.5, 0.0}, 0.5f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.localAnchorA = (m3Vec3){0.3f, 0.6f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.3f, 0.6f, 0.0f};
    m3JointId link = NewJoint(world, &jd);
    StepN(world, 60);
    double gap0 = CenterDistance(world, a, b);
    CHECK(gap0 < 0.95, "connected boxes overlap in peace");
    CHECK(!m3Joint_IsConnectedCollisionEnabled(world, link), "the flag reads back off");
    m3Joint_EnableConnectedCollision(world, link, true);
    CHECK(m3Joint_IsConnectedCollisionEnabled(world, link), "the flag reads back on");
    StepN(world, 120);
    // The pair hinges about the joint as it separates, sideways or one
    // over the other; either way the unit boxes end at least a box apart.
    double gap1 = CenterDistance(world, a, b);
    CHECK(gap1 > gap0 + 0.2 && gap1 > 1.0, "contacts on: the pair pushes apart");
    m3DestroyWorld(world);
}

static void TestReadbackBand(void)
{
    // A 1 kg crate hanging from a static anchor: the joint carries
    // m*g = 10 N once settled. Torque stays near zero.
    m3World* world = SmallWorld();
    m3BodyId anchor = Box(world, (m3Pos3){0.0, 5.0, 0.0}, 0.2f, 0);
    m3BodyId crate = Box(world, (m3Pos3){0.0, 3.8, 0.0}, 0.5f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = crate;
    jd.localAnchorA = (m3Vec3){0.0f, -0.2f, 0.0f};
    jd.localAnchorB = (m3Vec3){0.0f, 0.5f, 0.0f};
    m3JointId rope = NewJoint(world, &jd);
    StepN(world, 180);
    m3real force = m3Joint_GetConstraintForce(world, rope);
    m3real torque = m3Joint_GetConstraintTorque(world, rope);
    CHECK(force > 8.0f && force < 12.0f, "the rope carries the m*g band");
    CHECK(torque < 2.0f, "a ball joint carries no torque worth naming");
    m3DestroyWorld(world);
}

static void TestBreakage(void)
{
    // The same hanging crate, heavier: a cap ABOVE the load holds
    // forever; a cap UNDER the load breaks exactly once, the event
    // carries the id, and the crate falls free.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3World* world = SmallWorld();
        m3BodyId anchor = Box(world, (m3Pos3){0.0, 6.0, 0.0}, 0.2f, 0);
        m3BodyId crate = Box(world, (m3Pos3){0.0, 4.3, 0.0}, 1.0f, 1); // mass 8, weight 80
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_sphericalJoint;
        jd.bodyIdA = anchor;
        jd.bodyIdB = crate;
        jd.localAnchorA = (m3Vec3){0.0f, -0.2f, 0.0f};
        jd.localAnchorB = (m3Vec3){0.0f, 1.0f, 0.0f};
        m3JointId rope = NewJoint(world, &jd);
        StepN(world, 60); // settle first, break laws read clean loads
        m3Joint_SetBreakThresholds(world, rope, pass == 0 ? 200.0f : 40.0f, 0.0f);
        int32_t breaks = 0;
        bool idMatches = true;
        for (int32_t i = 0; i < 120; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            const m3JointBreakEvent* ev = m3World_GetJointEvents(world).breakEvents;
            int32_t n = m3World_GetJointEvents(world).breakCount;
            for (int32_t k = 0; k < n; ++k)
            {
                if (ev[k].jointId.index1 != rope.index1)
                {
                    idMatches = false;
                }
            }
            breaks += n;
        }
        if (pass == 0)
        {
            CHECK(breaks == 0, "a cap above the load holds");
            CHECK(m3Joint_IsValid(world, rope), "the held joint stays valid");
        }
        else
        {
            CHECK(breaks == 1, "a cap under the load breaks exactly once");
            CHECK(idMatches, "the break event carries the joint id");
            CHECK(!m3Joint_IsValid(world, rope), "the broken joint destroyed itself");
            CHECK(m3Body_GetPosition(world, crate).y < 2.0, "the freed crate falls");
        }
        m3DestroyWorld(world);
    }
}

static void TestSpringServo(void)
{
    // The hinge door driven by the runtime angle spring: with the
    // hinge axis vertical, gravity has no say, and the door parks
    // in the target band. 8-6b.
    m3World* world = SmallWorld();
    m3BodyId post = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId door = Box(world, (m3Pos3){0.8, 2.0, 0.0}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = post;
    jd.bodyIdB = door;
    jd.localAnchorA = (m3Vec3){0.4f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.4f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    m3JointId hinge = NewJoint(world, &jd);
    m3Joint_SetSpringHertz(world, hinge, 8.0f);
    m3Joint_SetSpringDampingRatio(world, hinge, 1.0f);
    m3Joint_EnableSpring(world, hinge, true);
    m3Joint_SetTargetAngle(world, hinge, 0.6f);
    StepN(world, 240);
    float angle = m3Joint_GetAngle(world, hinge);
    CHECK(fabsf(fabsf(angle) - 0.6f) < 0.1f, "the angle servo parks in the target band");
    m3DestroyWorld(world);
}

static void TestSpringDampingContrast(void)
{
    // The same slider, two damping ratios: the underdamped run
    // overshoots its target, the critically damped run does not.
    // Gravity-free so the axis owns the whole story.
    double overshoot[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        def.jointCapacity = 4;
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        m3World* world = NewWorld(&def);
        m3BodyId rail = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
        m3BodyId slider = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.3f, 1);
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_prismaticJoint;
        jd.bodyIdA = rail;
        jd.bodyIdB = slider;
        jd.localAxisA = (m3Vec3){1.0f, 0.0f, 0.0f};
        jd.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f};
        m3JointId slide = NewJoint(world, &jd);
        m3Joint_SetSpringHertz(world, slide, 2.0f);
        m3Joint_SetSpringDampingRatio(world, slide, pass == 0 ? 0.05f : 1.5f);
        m3Joint_EnableSpring(world, slide, true);
        m3Joint_SetTargetTranslation(world, slide, 1.0f);
        double peak = 0.0;
        for (int32_t i = 0; i < 300; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
            double t = (double)m3Joint_GetTranslation(world, slide);
            if (t > peak)
            {
                peak = t;
            }
        }
        overshoot[pass] = peak - 1.0;
        double rest = (double)m3Joint_GetTranslation(world, slide);
        CHECK(fabs(rest - 1.0) < 0.08, "the translation servo settles at the target");
        m3DestroyWorld(world);
    }
    CHECK(overshoot[0] > 0.1, "the underdamped spring overshoots");
    CHECK(overshoot[1] < 0.05, "the damped spring does not");
}

static void TestSphericalRotationDrive(void)
{
    // A floating crate driven to a quarter turn about y by the
    // rotation spring. Gravity-free; the drive owns the pose.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3World* world = NewWorld(&def);
    m3BodyId anchor = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId crate = Box(world, (m3Pos3){0.0, 2.0, 1.2}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = crate;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.6f};
    jd.localAnchorB = (m3Vec3){0.0f, 0.0f, -0.6f};
    // The target lives in the joint frames (frame z = the local
    // axis, as documented). Put the frame's z
    // on world y, so a frame-z target turns the crate about y.
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    m3JointId ball = NewJoint(world, &jd);
    float s = sinf(0.25f * M3_PI * 0.5f);
    float c = cosf(0.25f * M3_PI * 0.5f);
    m3Quat target = {0.0f, 0.0f, s, c}; // 45 degrees about frame z
    m3Joint_SetSpringHertz(world, ball, 6.0f);
    m3Joint_SetSpringDampingRatio(world, ball, 1.0f);
    m3Joint_EnableSpring(world, ball, true);
    m3Joint_SetTargetRotation(world, ball, target);
    StepN(world, 300);
    m3Quat q = m3Body_GetRotation(world, crate);
    m3Quat want = {0.0f, s, 0.0f, c}; // 45 degrees about world y
    float dot = fabsf(q.x * want.x + q.y * want.y + q.z * want.z + q.w * want.w);
    CHECK(dot > 0.98f, "the rotation drive lands the crate near the target pose");
    m3DestroyWorld(world);
}

static void TestHostileRuntime(void)
{
    m3World* world = SmallWorld();
    m3BodyId a = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId b = Box(world, (m3Pos3){1.0, 2.0, 0.0}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    m3JointId link = NewJoint(world, &jd);
    CHECK_REFUSES(world, m3Joint_SetLimits(world, link, 1.0f, -1.0f), "lower > upper refuses");
    CHECK_REFUSES(world, m3Joint_SetMotorSpeed(world, link, 1.0f), "a ball joint takes no motor");
    CHECK_REFUSES(world, m3Joint_EnableSpring(world, link, true), "a zero-hertz spring refuses");
    CHECK_REFUSES(world, m3Joint_SetSpringHertz(world, link, NAN), "a NaN spring hertz refuses");
    m3Joint_SetBreakThresholds(world, link, -1.0f, 0.0f);
    m3Joint_SetTargetAngle(world, link, 0.5f);       // ball joint: wrong type, no-op
    m3Joint_SetTargetTranslation(world, link, 0.5f); // wrong type, no-op
    m3Joint_SetTargetRotation(world, link, (m3Quat){3.0f, 0.0f, 0.0f, 1.0f}); // non-unit refused
    m3JointId stale = link;
    stale.index1 += 99;
    CHECK_REFUSES(world, m3Joint_EnableMotor(world, stale, true), "a stale joint refuses");
    CHECK(m3Joint_GetConstraintForce(world, stale) == 0.0f, "stale id reads zero");
    CHECK(m3Joint_GetAngle(world, link) == 0.0f, "GetAngle on a ball joint reads zero");
    CHECK(m3Joint_GetTranslation(world, link) == 0.0f, "GetTranslation on a ball joint reads zero");
    StepN(world, 10);
    CHECK(m3Joint_IsValid(world, link), "hostile args left the joint whole");
    m3DestroyWorld(world);
}

static void TestSwitchesKeepValues(void)
{
    // Each switch and each value has its own call: values set while a
    // part is off wait for it, switching off keeps them, and the getters
    // read back what the setters stored.
    m3World* world = SmallWorld();
    m3BodyId a = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId b = Box(world, (m3Pos3){1.0, 2.0, 0.0}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.localAnchorA = (m3Vec3){0.5f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.5f, 0.0f, 0.0f};
    jd.lowerLimit = 0.5f; // a disabled def range may be inverted
    jd.upperLimit = -0.5f;
    m3JointId hinge = NewJoint(world, &jd);
    CHECK_REFUSES(world, m3Joint_EnableLimit(world, hinge, true),
                  "an inverted range cannot be switched on");
    CHECK_ACCEPTS(world, m3Joint_EnableLimit(world, hinge, false), "but can be switched off");
    CHECK_ACCEPTS(world, m3Joint_SetLimits(world, hinge, -0.25f, 0.75f), "an ordered range sets");
    CHECK(!m3Joint_IsLimitEnabled(world, hinge), "setting the range leaves the limit off");
    CHECK_ACCEPTS(world, m3Joint_EnableLimit(world, hinge, true), "and the limit switches on");
    float lower = 0.0f;
    float upper = 0.0f;
    m3Joint_GetLimits(world, hinge, &lower, &upper);
    CHECK(m3Joint_IsLimitEnabled(world, hinge) && lower == -0.25f && upper == 0.75f,
          "the limit reads back on with its range");
    m3Joint_GetLimits(world, hinge, NULL, NULL);

    CHECK_ACCEPTS(world, m3Joint_SetSpringHertz(world, hinge, 0.0f), "an off spring takes 0 Hz");
    CHECK_REFUSES(world, m3Joint_EnableSpring(world, hinge, true), "but cannot switch on at 0 Hz");
    m3Joint_SetSpringHertz(world, hinge, 4.0f);
    m3Joint_SetSpringDampingRatio(world, hinge, 0.6f);
    CHECK_ACCEPTS(world, m3Joint_EnableSpring(world, hinge, true), "a 4 Hz spring switches on");
    CHECK_REFUSES(world, m3Joint_SetSpringHertz(world, hinge, 0.0f), "an on spring keeps > 0 Hz");
    m3Joint_EnableSpring(world, hinge, false);
    CHECK(!m3Joint_IsSpringEnabled(world, hinge) && m3Joint_GetSpringHertz(world, hinge) == 4.0f &&
              m3Joint_GetSpringDampingRatio(world, hinge) == 0.6f,
          "switching the spring off keeps its values");

    m3Joint_SetMotorSpeed(world, hinge, -2.0f);
    m3Joint_SetMaxMotorEffort(world, hinge, 30.0f);
    CHECK(!m3Joint_IsMotorEnabled(world, hinge), "motor values leave the motor off");
    m3Joint_EnableMotor(world, hinge, true);
    CHECK(m3Joint_IsMotorEnabled(world, hinge) && m3Joint_GetMotorSpeed(world, hinge) == -2.0f &&
              m3Joint_GetMaxMotorEffort(world, hinge) == 30.0f,
          "the motor reads back on with its values");

    // A weld takes none of the three; its getters read zeros.
    jd = m3DefaultJointDef();
    jd.type = m3_fixedJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    m3JointId weld = NewJoint(world, &jd);
    CHECK_REFUSES(world, m3Joint_EnableLimit(world, weld, true), "a weld takes no limit");
    CHECK_REFUSES(world, m3Joint_SetLimits(world, weld, 0.0f, 1.0f), "nor a limit range");
    CHECK_REFUSES(world, m3Joint_SetSpringHertz(world, weld, 1.0f), "nor a spring");
    CHECK_REFUSES(world, m3Joint_SetMaxMotorEffort(world, weld, 1.0f), "nor a motor");
    m3Joint_GetLimits(world, weld, &lower, &upper);
    CHECK(lower == 0.0f && upper == 0.0f && !m3Joint_IsLimitEnabled(world, weld) &&
              m3Joint_GetSpringHertz(world, weld) == 0.0f &&
              m3Joint_GetMotorSpeed(world, weld) == 0.0f,
          "a weld's getters read zeros");
    m3JointId stale = weld;
    stale.index1 += 99;
    CHECK_REFUSES(world, (void)m3Joint_IsSpringEnabled(world, stale), "a stale getter counts");

    StepN(world, 30);
    CHECK(m3Joint_IsValid(world, hinge), "the hinge runs on its parts");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRuntimeMotorAndLimits();
    TestCollideToggle();
    TestReadbackBand();
    TestBreakage();
    TestSpringServo();
    TestSpringDampingContrast();
    TestSphericalRotationDrive();
    TestHostileRuntime();
    TestSwitchesKeepValues();
    if (s_failures == 0)
    {
        printf("test_jointdrive: all green\n");
        return 0;
    }
    printf("test_jointdrive: %d failure(s)\n", s_failures);
    return 1;
}
