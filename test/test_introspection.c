// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The introspection gate: counters report the live world
// exactly, and above all READING IS PURE: a twin that polls counters
// and profile every tick lands on the same bits as a twin that never
// looks.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/body.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"

#include <stdio.h>
#include <string.h>

static m3World* BuildYard(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    def.jointCapacity = 4;
    def.softBodyCapacity = 2;
    def.voxelCapacity = 2;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    for (int32_t i = 0; i < 3; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.55 + 1.1 * (double)i, 0.0};
        NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    }
    return world;
}

static void TestCountsMatchTheScene(void)
{
    m3World* world = BuildYard();
    m3Counters c = m3World_GetCounters(world);
    CHECK(c.bodyCount == 4, "one ground and three crates");
    CHECK(c.shapeCount == 4, "one plane and three boxes");
    CHECK(c.jointCount == 0, "no joints yet");
    CHECK(c.hullCount >= 1, "boxes intern hull slabs");
    CHECK(c.islandCount == 0 && c.colorCount == 0, "step statistics are zero before a step");
    CHECK(c.snapshotBytes > 0, "the snapshot size reports");
    CHECK(c.snapshotBytes == m3World_SnapshotSize(world), "the two size paths agree");

    // A joint arrives, then a lattice: the counters follow.
    m3BodyDef ad = m3DefaultBodyDef();
    ad.type = m3_dynamicBody;
    ad.position = (m3Pos3){3.0, 2.0, 0.0};
    m3BodyId a = NewBody(world, &ad);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, a, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    ad.position = (m3Pos3){3.0, 1.0, 0.0};
    m3BodyId b = NewBody(world, &ad);
    NewBoxShape(world, b, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    m3JointId link = NewJoint(world, &jd);
    m3SoftBodyDef sbd = m3DefaultSoftBodyDef();
    sbd.position = (m3Pos3){-4.0, 3.0, 0.0};
    sbd.countX = 2;
    sbd.countY = 2;
    sbd.countZ = 2;
    sbd.spacing = 0.5f;
    m3SoftBodyId jelly = NewSoftBody(world, &sbd);
    CHECK(m3SoftBody_IsValid(world, jelly), "the lattice creates");
    c = m3World_GetCounters(world);
    CHECK(c.bodyCount == 6, "six bodies now");
    CHECK(c.jointCount == 1, "the joint counts");
    CHECK(c.softBodyCount == 1, "the lattice counts");

    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    c = m3World_GetCounters(world);
    CHECK(c.contactCount > 0, "the settled stack has contacts");
    CHECK(c.awakeCount >= 1, "something is awake");
    CHECK(c.islandCount >= 1, "the step counted its islands");
    CHECK(c.colorCount >= 1, "the step counted its colors");
    CHECK(c.treeHeight >= 1, "the tree has height");
    CHECK(c.scratchPeak > 0, "the step used scratch");
    CHECK(c.scratchPeak <= c.scratchCapacity, "the peak fits the reserve");

    // Destruction shrinks the census.
    DestroyJoint(world, link);
    DestroyBody(world, b);
    c = m3World_GetCounters(world);
    CHECK(c.bodyCount == 5, "the destroyed body left the count");
    CHECK(c.jointCount == 0, "the destroyed joint left the count");
    m3DestroyWorld(world);
}

static void TestReadingIsPure(void)
{
    // The determinism guard: one twin polls the observers every
    // tick, the other never looks, and the bits must agree.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3World* world = BuildYard();
        for (int32_t i = 0; i < 120; ++i)
        {
            if (run == 0)
            {
                m3Counters c = m3World_GetCounters(world);
                m3Profile p = m3World_GetProfile(world);
                (void)c;
                (void)p;
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "watching the world never changes it");
}

static void TestNamesAndHooks(void)
{
    static uint8_t journal[65536];
    static uint8_t snap[2097152];
    m3World* world = BuildYard();
    CHECK(StartJournal(world, journal, (int32_t)sizeof(journal)), "the tape opens");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){5.0, 1.0, 0.0};
    m3BodyId hero = NewBody(world, &bd);
    CHECK(m3Body_GetName(world, hero)[0] == 0, "a new body is unnamed");
    m3Body_SetName(world, hero, "the crate of destiny");
    CHECK(strcmp(m3Body_GetName(world, hero), "the crate of destiny") == 0, "the name reads back");
    m3Body_SetName(world, hero, "a name far too long to fit inside thirty one bytes of storage");
    CHECK(strlen(m3Body_GetName(world, hero)) == 31, "long names truncate to the cap");
    m3Body_SetName(world, hero, "hero");
    uint64_t before = m3World_Hash(world);
    m3Body_SetName(world, hero, "renamed");
    CHECK(m3World_Hash(world) == before, "a name is never a hash input");
    int32_t bytes = StopJournal(world);
    CHECK(bytes > 0, "the naming session records");
    m3World* twin = BuildYard();
    CHECK(ReplayJournal(twin, journal, bytes), "the naming session replays");
    CHECK(strcmp(m3Body_GetName(twin, hero), "renamed") == 0, "the replayed name lands");
    m3DestroyWorld(twin);
    int32_t snapBytes = Snapshot(world, snap, (int32_t)sizeof(snap));
    CHECK(snapBytes > 0, "the named snapshot fits");
    m3Body_SetName(world, hero, "clobbered");
    CHECK(Restore(world, snap, snapBytes), "the restore lands");
    CHECK(strcmp(m3Body_GetName(world, hero), "renamed") == 0, "the snapshot carries the name");
    m3BodyId stale = {99, 7};
    m3Body_SetName(world, stale, "ghost");
    CHECK(m3Body_GetName(world, stale)[0] == 0, "a stale id stays nameless");
    m3DestroyWorld(world);
}

static void TestContactReadback(void)
{
    m3World* world = BuildYard();
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // The stack settled: the middle crate touches above and below.
    m3Counters c = m3World_GetCounters(world);
    CHECK(c.contactCount > 0, "the yard has contacts");
    m3ContactData data[8];
    int32_t total = 0;
    int32_t maxPer = 0;
    // The readback needs real ids: build a fresh two-crate stack.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){10.0, 0.55, 0.0};
    m3BodyId a = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId shapeA = NewBoxShape(world, a, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    bd.position = (m3Pos3){10.0, 1.6, 0.0};
    m3BodyId b = NewBody(world, &bd);
    NewBoxShape(world, b, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    total = m3Body_GetContactData(world, a, data, 8);
    CHECK(total >= 1, "the crate reports its touches");
    maxPer = data[0].pointCount;
    CHECK(maxPer >= 1 && maxPer <= 4, "the manifold point count is sane");
    CHECK(data[0].normalImpulses[0] >= 0.0f, "normal impulses are never negative");
    double y = data[0].points[0].y;
    CHECK(y > -0.2 && y < 1.7, "the contact points sit near the stack");
    int32_t viaShape = m3Shape_GetContactData(world, shapeA, data, 8);
    CHECK(viaShape == total, "the shape view agrees with the body view");
    m3ContactData none[2];
    m3BodyId stale = {99, 7};
    CHECK(m3Body_GetContactData(world, stale, none, 2) == 0, "a stale id reads zero contacts");
    m3DestroyWorld(world);
}

static bool Near(double a, double b, double tolerance)
{
    return a - b <= tolerance && b - a <= tolerance;
}

static bool NearF(float a, float b, float tolerance)
{
    return a - b <= tolerance && b - a <= tolerance;
}

static void TestReadback(void)
{
    // The readers Maul2D has, on a two-shape body with a joint.
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.0, 2.0, 3.0};
    bd.rotation = (m3Quat){0.0f, 0.70710678f, 0.0f, 0.70710678f}; // 90 degrees about y
    bd.linearVelocity = (m3Vec3){1.0f, 0.0f, 0.0f};
    bd.angularVelocity = (m3Vec3){0.0f, 2.0f, 0.0f};
    bd.gravityScale = 0.5f;
    bd.linearDamping = 0.25f;
    bd.angularDamping = 0.75f;
    bd.isBullet = true;
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 2.0f;
    sd.userData = 42u;
    m3ShapeId box = NewBoxShape(world, body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    CHECK(NearF(m3Body_GetMass(world, body), 2.0f, 1e-5f),
          "the unit box at density two weighs two");
    sd.isSensor = true;
    m3Sphere probe = {{0.0f, 0.0f, 0.0f}, 0.25f};
    m3ShapeId sensor = NewSphereShape(world, body, &sd, &probe);
    m3BodyDef ad = m3DefaultBodyDef();
    m3BodyId anchor = NewBody(world, &ad);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = anchor;
    jd.bodyIdB = body;
    m3JointId joint = NewJoint(world, &jd);

    m3Transform xf = m3Body_GetTransform(world, body);
    CHECK(xf.p.x == 1.0 && xf.p.y == 2.0 && xf.p.z == 3.0, "body transform");
    m3Mat3 inertia = m3Body_GetRotationalInertia(world, body);
    CHECK(inertia.cx.x > 0.0f && inertia.cy.y > 0.0f && inertia.cz.z > 0.0f, "inertia tensor");
    m3Pos3 p = m3Body_GetWorldPoint(world, body, (m3Vec3){1.0f, 0.0f, 0.0f});
    CHECK(Near(p.x, 1.0, 1e-5) && Near(p.z, 2.0, 1e-5), "local +x maps to world -z");
    m3Vec3 back = m3Body_GetLocalPoint(world, body, p);
    CHECK(NearF(back.x, 1.0f, 1e-5f) && NearF(back.z, 0.0f, 1e-5f), "and back");
    m3Vec3 v = m3Body_GetWorldVector(world, body, (m3Vec3){0.0f, 0.0f, 1.0f});
    CHECK(NearF(v.x, 1.0f, 1e-5f), "local +z points along world +x");
    CHECK(NearF(m3Body_GetLocalVector(world, body, v).z, 1.0f, 1e-5f), "and back again");
    m3Vec3 pv = m3Body_GetWorldPointVelocity(world, body, (m3Pos3){1.0, 2.0, 2.0});
    CHECK(NearF(pv.x, -1.0f, 1e-5f), "v + w x r at a world point");
    m3Vec3 lv = m3Body_GetLocalPointVelocity(world, body, (m3Vec3){1.0f, 0.0f, 0.0f});
    CHECK(NearF(lv.x, pv.x, 1e-5f) && NearF(lv.z, pv.z, 1e-5f), "the same point, given locally");
    CHECK(m3Body_GetGravityScale(world, body) == 0.5f &&
              m3Body_GetLinearDamping(world, body) == 0.25f &&
              m3Body_GetAngularDamping(world, body) == 0.75f && m3Body_IsBullet(world, body),
          "def settings read back");
    m3AabbResult bounds = m3Body_ComputeAabb(world, body);
    CHECK(bounds.lowerBound.x <= 0.5 && bounds.upperBound.x >= 1.5 && bounds.lowerBound.y <= 1.5,
          "the body bounds hold its box");
    m3ShapeId shapes[4];
    CHECK(m3Body_GetShapes(world, body, shapes, 4) == 2 && shapes[0].index1 == box.index1 &&
              shapes[1].index1 == sensor.index1,
          "both shapes, in slot order");
    CHECK(m3Body_GetShapes(world, body, shapes, 1) == 2, "the total survives a short buffer");
    m3JointId joints[2];
    CHECK(m3Body_GetJoints(world, body, joints, 2) == 1 && joints[0].index1 == joint.index1,
          "the joint on the body");

    CHECK(m3Shape_GetType(world, box) == m3_hullShape &&
              m3Shape_GetType(world, sensor) == m3_sphereShape,
          "shape types");
    CHECK(m3Shape_GetUserData(world, box) == 42u && m3Shape_IsSensor(world, sensor) &&
              !m3Shape_IsSensor(world, box),
          "shape settings read back");
    m3AabbResult sb = m3Shape_GetAabb(world, sensor);
    CHECK(sb.lowerBound.y <= 1.75 && sb.upperBound.y >= 2.25, "the sensor bounds hold it");

    CHECK(m3Joint_GetType(world, joint) == m3_sphericalJoint, "joint type");
    CHECK(m3Joint_GetBodyA(world, joint).index1 == anchor.index1 &&
              m3Joint_GetBodyB(world, joint).index1 == body.index1,
          "joint bodies");

    m3BodyId all[4];
    CHECK(m3World_GetBodies(world, all, 4) == 2, "the world's bodies");
    CHECK(m3World_GetJoints(world, NULL, 0) == 1, "the world's joints, counted");
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_GetStepCount(world) == 2u, "two steps taken");
    m3DestroyWorld(world);
}

int main(void)
{
    TestCountsMatchTheScene();
    TestReadingIsPure();
    TestNamesAndHooks();
    TestContactReadback();
    TestReadback();
    if (s_failures == 0)
    {
        printf("test_introspection: all passed\n");
        return 0;
    }
    return 1;
}
