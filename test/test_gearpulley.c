// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The transmission gate: a GEAR holds spinA + ratio * spinB
// at its create value across two bodies hinged on a common frame
// (counter-rotation under positive ratio, the external mesh), and a
// PULLEY holds length1 + ratio * length2 across two world anchors
// (rigid both ways). Both refuse hostile defs.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Mesh
{
    m3World* world;
    m3BodyId gearA;
    m3BodyId gearB;
    m3JointId gear;
} Mesh;

// Two disks hinged about z on a static frame, meshed 2:1 in zero
// gravity, at rest.
static Mesh NewMesh(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 8;
    m3World* world = NewWorld(&wd);
    m3BodyDef fd = m3DefaultBodyDef();
    m3BodyId frame = NewBody(world, &fd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-0.5, 2.0, 0.0};
    m3BodyId gearA = NewBody(world, &bd);
    bd.position = (m3Pos3){0.5, 2.0, 0.0};
    m3BodyId gearB = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, gearA, &sd, (m3Vec3){0.3f, 0.3f, 0.1f});
    NewBoxShape(world, gearB, &sd, (m3Vec3){0.3f, 0.3f, 0.1f});
    m3JointDef hd = m3DefaultJointDef();
    hd.type = m3_revoluteJoint;
    hd.bodyIdA = frame;
    hd.bodyIdB = gearA;
    hd.localAnchorA = (m3Vec3){-0.5f, 2.0f, 0.0f};
    hd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    hd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    CHECK(m3Joint_IsValid(world, NewJoint(world, &hd)), "hinge A creates");
    hd.bodyIdB = gearB;
    hd.localAnchorA = (m3Vec3){0.5f, 2.0f, 0.0f};
    CHECK(m3Joint_IsValid(world, NewJoint(world, &hd)), "hinge B creates");
    m3JointDef gd = m3DefaultJointDef();
    gd.type = m3_gearJoint;
    gd.bodyIdA = gearA;
    gd.bodyIdB = gearB;
    gd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    gd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    gd.ratio = 2.0f;
    m3JointId gear = NewJoint(world, &gd);
    CHECK(m3Joint_IsValid(world, gear), "the gear creates");
    return (Mesh){world, gearA, gearB, gear};
}

static void TestGearCouplesSpin(void)
{
    // Kick one disk and the mesh splits the spin with the opposite
    // sign, and keeps the split.
    Mesh mesh = NewMesh();
    m3World* world = mesh.world;
    m3BodyId gearA = mesh.gearA;
    m3BodyId gearB = mesh.gearB;
    m3Body_ApplyAngularImpulse(world, gearA, (m3Vec3){0.0f, 0.0f, 0.05f});
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 wA = m3Body_GetAngularVelocity(world, gearA);
    m3Vec3 wB = m3Body_GetAngularVelocity(world, gearB);
    CHECK(fabsf(wA.z) > 0.05f, "the kicked gear genuinely spins");
    CHECK(wA.z * wB.z < 0.0f, "positive ratio counter-rotates (the external mesh)");
    CHECK(fabsf(wA.z + 2.0f * wB.z) < 0.02f, "the mesh holds spinA + 2 spinB");
    m3DestroyWorld(world);
}

static void TestGearTakesNoMotor(void)
{
    // The gear keeps each disk's spin at create in the motor slot: a
    // motor call that reached it turned the disks at rest to a new
    // mesh, and a speed past a half turn left a snapshot restore
    // refuses.
    Mesh mesh = NewMesh();
    CHECK_REFUSES(mesh.world, m3Joint_SetMotorSpeed(mesh.world, mesh.gear, 1000.0f),
                  "a gear takes no motor");
    for (int32_t i = 0; i < 60; ++i)
    {
        (void)m3World_Step(mesh.world, 1.0f / 60.0f, 4);
    }
    m3Quat qA = m3Body_GetRotation(mesh.world, mesh.gearA);
    m3Quat qB = m3Body_GetRotation(mesh.world, mesh.gearB);
    CHECK(fabsf(qA.z) < 1e-6f && fabsf(qB.z) < 1e-6f, "the meshed disks stay at rest");
    int32_t size = m3World_SnapshotSize(mesh.world);
    void* snapshot = malloc((size_t)size);
    int32_t bytes = snapshot != NULL ? Snapshot(mesh.world, snapshot, size) : -1;
    CHECK(bytes > 0, "the mesh snapshots");
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 8;
    m3World* restored = NewWorld(&wd);
    CHECK(bytes > 0 && Restore(restored, snapshot, bytes), "and the snapshot restores");
    m3DestroyWorld(restored);
    free(snapshot);
    m3DestroyWorld(mesh.world);
}

static void PulleyLengths(const m3World* world, m3BodyId a, m3BodyId b, double* len1, double* len2)
{
    m3Pos3 pa = m3Body_GetPosition(world, a);
    m3Pos3 pb = m3Body_GetPosition(world, b);
    double dax = pa.x + 1.0;
    double day = pa.y - 4.0;
    double dbx = pb.x - 1.0;
    double dby = pb.y - 4.0;
    *len1 = sqrt(dax * dax + day * day + pa.z * pa.z);
    *len2 = sqrt(dbx * dbx + dby * dby + pb.z * pb.z);
}

static void TestPulleyTradesRope(void)
{
    // A heavy crate and a light one hang from ropes over two world
    // anchors: the heavy side sinks, the light side rises, and
    // length1 + ratio * length2 keeps its create value throughout.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        float ratio = pass == 0 ? 1.0f : 2.0f;
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        wd.jointCapacity = 4;
        m3World* world = NewWorld(&wd);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-1.0, 2.0, 0.0};
        m3BodyId heavy = NewBody(world, &bd);
        bd.position = (m3Pos3){1.0, 2.0, 0.0};
        m3BodyId light = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        NewBoxShape(world, heavy, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
        NewBoxShape(world, light, &sd, (m3Vec3){0.2f, 0.2f, 0.2f});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_pulleyJoint;
        jd.bodyIdA = heavy;
        jd.bodyIdB = light;
        jd.groundAnchorA = (m3Pos3){-1.0, 4.0, 0.0};
        jd.groundAnchorB = (m3Pos3){1.0, 4.0, 0.0};
        jd.ratio = ratio;
        CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the pulley creates");
        double constant = 2.0 + (double)ratio * 2.0;
        for (int32_t i = 0; i < 150; ++i)
        {
            (void)m3World_Step(world, 1.0f / 60.0f, 4);
        }
        double len1;
        double len2;
        PulleyLengths(world, heavy, light, &len1, &len2);
        CHECK(m3Body_GetPosition(world, heavy).y < 1.9, "the heavy side sinks");
        CHECK(m3Body_GetPosition(world, light).y > 2.1, "the light side rises");
        CHECK(fabs(len1 + (double)ratio * len2 - constant) < 0.03,
              "the rope law holds through the trade");
        m3DestroyWorld(world);
    }
}

static void TestTransmissionWalls(void)
{
    // Hostile defs refuse loudly and leak nothing.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = NewBody(world, &bd);
    bd.position = (m3Pos3){1.0, 2.0, 0.0};
    m3BodyId b = NewBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_gearJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.ratio = 0.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a zero-ratio gear refuses");
    jd.ratio = 1.0f;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 0.0f};
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a zero-axis gear refuses");
    jd = m3DefaultJointDef();
    jd.type = m3_pulleyJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.groundAnchorA = (m3Pos3){0.0, 4.0, 0.0};
    jd.groundAnchorB = (m3Pos3){1.0, 4.0, 0.0};
    jd.ratio = -1.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a negative-ratio pulley refuses");
    jd.ratio = 0.0f;
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a zero-ratio pulley refuses");
    jd.ratio = 1.0f;
    jd.groundAnchorA = (m3Pos3){(double)NAN, 4.0, 0.0};
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a NaN world anchor refuses");
    jd.groundAnchorA = (m3Pos3){0.0, 2.0, 0.0}; // ON the body anchor
    CHECK(!m3Joint_IsValid(world, NewJoint(world, &jd)), "a rope end on its pulley refuses");
    m3DestroyWorld(world);
}

int main(void)
{
    TestGearCouplesSpin();
    TestGearTakesNoMotor();
    TestPulleyTradesRope();
    TestTransmissionWalls();
    if (s_failures == 0)
    {
        printf("test_gearpulley: all passed\n");
        return 0;
    }
    printf("test_gearpulley: %d FAILURES\n", s_failures);
    return 1;
}
