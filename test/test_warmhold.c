// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The warm start hold: prepare scales each loaded contact and joint
// impulse, direction kept, so its warm start stays within M3_WARM_LIMIT
// at that step's own masses and levers (src/core.h). The factor at its
// edges, the hold acting on impulses a body's mass change left behind,
// and a world at the heaviest mass against a kinematic body that
// snapshots, restores and steps on alike.

#include "contact_solver.h"
#include "helpers.h"
#include "joint_solver.h"
#include "solver_math.h"
#include "test_harness.h"
#include "world_internal.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The held scenes' hashes, from test/hashes.txt: every CI cell, each
// SIMD backend and the scalar path, steps them to the same bits.
#ifndef M3_GOLDEN_WARM_CONTACTS
#define M3_GOLDEN_WARM_CONTACTS 0
#endif
#ifndef M3_GOLDEN_WARM_JOINTS
#define M3_GOLDEN_WARM_JOINTS 0
#endif

static const float s_dt = 1.0f / 60.0f;

static m3Mat3 Identity(void)
{
    m3Mat3 m = m3MakeZeroMat3();
    m.cx.x = 1.0f;
    m.cy.y = 1.0f;
    m.cz.z = 1.0f;
    return m;
}

// The factor is exactly 1 up to each limit and holds the result within it
// just past.
static void TestHoldAtItsEdges(void)
{
    m3Mat3 unit = Identity();
    m3Mat3 none = m3MakeZeroMat3();
    // Inverse mass 1, inverse inertia 1, no lever: the reach is
    // (1 + 3 * 1 * (0 + 1)) times the impulses' sum.
    double edge = (double)M3_WARM_LIMIT / 4.0;
    CHECK(m3WarmHold(1.0f, &unit, 0.0f, &none, 0.0, edge) == 1.0f,
          "an impulse that reaches the limit exactly is not held");
    double past = nextafter(edge, 2.0 * edge);
    m3real scale = m3WarmHold(1.0f, &unit, 0.0f, &none, 0.0, past);
    CHECK(scale < 1.0f, "one just past it is held");
    CHECK((double)scale * 4.0 * past <= (double)M3_WARM_LIMIT * (1.0 + 1.0e-6),
          "to within the limit");
    CHECK(m3WarmHold(0.0f, &none, 1.0f, &unit, 0.0, past) == scale,
          "the same on B's side as on A's");
    CHECK(m3WarmHold(1.0f, &unit, 2.0f, &unit, 0.0, edge) < 1.0f, "the lighter side decides");

    // Two immovable sides reach nothing; the moment, impulse times lever
    // plus one, still binds.
    double moment = (double)M3_WARM_MOMENT_LIMIT / 2.0;
    CHECK(m3WarmHold(0.0f, &none, 0.0f, &none, 1.0, moment) == 1.0f,
          "a moment at its limit is not held");
    scale = m3WarmHold(0.0f, &none, 0.0f, &none, 1.0, nextafter(moment, 2.0 * moment));
    CHECK(scale < 1.0f, "one just past it is held");
    CHECK((double)scale * 2.0 * moment <= (double)M3_WARM_MOMENT_LIMIT * (1.0 + 1.0e-6),
          "to within the limit");
}

typedef struct Heavy
{
    m3World* world;
    m3BodyId body;
    m3ShapeId shape;
} Heavy;

// A 200 m box at the density limit, 8e12 kg, under the gravity limit at a
// thousandfold gravity scale, held by the ground or by a joint above it.
static Heavy HeavyScene(bool hung)
{
    m3WorldDef def = m3DefaultWorldDef();
    Heavy scene = {NewWorld(&def), m3_nullBodyId, m3_nullShapeId};
    m3World_SetGravity(scene.world, (m3Vec3){0.0f, -M3_ACCEL_LIMIT, 0.0f});
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(scene.world, &bd);
    (void)NewBoxShape(scene.world, ground, &sd, (m3Vec3){1000.0f, 1.0f, 1000.0f});
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 101.0, 0.0};
    bd.gravityScale = 1000.0f;
    scene.body = NewBody(scene.world, &bd);
    sd.density = M3_DENSITY_LIMIT;
    scene.shape = NewBoxShape(scene.world, scene.body, &sd, (m3Vec3){100.0f, 100.0f, 100.0f});
    if (hung)
    {
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_sphericalJoint;
        jd.bodyIdA = ground;
        jd.bodyIdB = scene.body;
        jd.localAnchorA = (m3Vec3){0.0f, 201.0f, 0.0f};
        jd.localAnchorB = (m3Vec3){0.0f, 100.0f, 0.0f};
        (void)NewJoint(scene.world, &jd);
    }
    for (int32_t i = 0; i < 10; ++i)
    {
        m3World_Step(scene.world, s_dt, 4);
    }
    return scene;
}

static bool Finite3(m3Vec3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool Within(m3Vec3 v, float limit)
{
    return fabsf(v.x) <= limit && fabsf(v.y) <= limit && fabsf(v.z) <= limit;
}

// The body keeps its shape and becomes 1.6e-6 kg, just over the mass
// floor, while its contacts and joints still hold what it carried at
// 8e12 kg.
static void Lighten(Heavy* scene)
{
    m3Shape_SetDensity(scene->world, scene->shape, 2.0e-13f);
    float invMass = scene->world->bodies.invMass[scene->body.index1 - 1];
    CHECK(invMass > 1.0e5f, "the body is light now");
}

// The steps after stay finite and within the speed limit, to the
// committed hash.
static void StepsOnFinite(Heavy* scene, const char* name, uint64_t committed)
{
    for (int32_t i = 0; i < 4; ++i)
    {
        m3World_Step(scene->world, s_dt, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(scene->world, scene->body);
    m3Vec3 w = m3Body_GetAngularVelocity(scene->world, scene->body);
    m3Pos3 p = m3Body_GetPosition(scene->world, scene->body);
    CHECK(Finite3(v) && Finite3(w) && isfinite(p.x) && isfinite(p.y) && isfinite(p.z),
          "the step after stays finite");
    CHECK(Within(v, M3_SPEED_LIMIT) && Within(w, M3_SPEED_LIMIT), "and within the speed limit");
    uint64_t hash = m3World_Hash(scene->world);
    printf("M3_%s_HASH=%016llx\n", name, (unsigned long long)hash);
    CHECK(hash == committed, "the held scene steps to its committed hash");
}

// A contact's stored impulses meet a body made light since: prepare holds
// them all by one factor, so their ratios hold, and the step stays finite.
static void TestHoldOnContacts(void)
{
    Heavy scene = HeavyScene(false);
    m3World* world = scene.world;
    CHECK(world->contacts.pairCount == 1, "the box rests on the ground");
    const m3Manifold* manifold = &world->contacts.manifolds[0];
    CHECK(manifold->pointCount >= 2, "on several points");
    Lighten(&scene);

    m3ContactConstraint* prepared =
        (m3ContactConstraint*)calloc((size_t)world->contacts.pairCount, sizeof(*prepared));
    CHECK(prepared != NULL, "constraint storage");
    if (prepared == NULL)
    {
        m3DestroyWorld(world);
        return;
    }
    CHECK(m3PrepareContacts(world, prepared, s_dt / 4.0f) == 1, "the contact prepares");
    const m3ContactConstraint* c = &prepared[0];
    double scale = (double)c->points[0].normalImpulse / (double)manifold->points[0].normalImpulse;
    CHECK(scale > 0.0 && scale < 1.0e-6, "the hold scales the loaded impulses down");
    double reach = 0.0;
    bool kept = true;
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        double ratio =
            (double)c->points[k].normalImpulse / (double)manifold->points[k].normalImpulse;
        kept = kept && fabs(ratio / scale - 1.0) < 1.0e-6;
        reach += (double)c->invMassB * fabs((double)c->points[k].normalImpulse);
    }
    CHECK(kept, "every point by the same factor, so the direction is kept");
    CHECK(reach <= (double)M3_WARM_LIMIT, "and the warm start's linear reach is within the limit");
    free(prepared);

    StepsOnFinite(&scene, "WARM_CONTACTS", M3_GOLDEN_WARM_CONTACTS);
    m3DestroyWorld(world);
}

// The same for a joint's five impulse slots.
static void TestHoldOnJoints(void)
{
    Heavy scene = HeavyScene(true);
    m3World* world = scene.world;
    int32_t joint = world->joints.bodyJointHead[scene.body.index1 - 1];
    CHECK(joint >= 0, "the box hangs from its joint");
    m3Vec3 stored = world->joints.jointImpulse[joint];
    CHECK(fabsf(stored.y) > 1.0e15f, "which carries its weight");
    Lighten(&scene);

    m3JointConstraint* prepared =
        (m3JointConstraint*)calloc((size_t)world->joints.jointPool.maxIndex, sizeof(*prepared));
    CHECK(prepared != NULL, "constraint storage");
    if (prepared == NULL)
    {
        m3DestroyWorld(world);
        return;
    }
    CHECK(m3PrepareJoints(world, prepared, s_dt / 4.0f) == 1, "the joint prepares");
    m3Vec3 loaded = prepared[0].impulse;
    double scale = (double)loaded.y / (double)stored.y;
    CHECK(scale > 0.0 && scale < 1.0e-6, "the hold scales the loaded impulse down");
    bool kept =
        (stored.x == 0.0f ? loaded.x == 0.0f
                          : fabs((double)loaded.x / (double)stored.x / scale - 1.0) < 1.0e-6) &&
        (stored.z == 0.0f ? loaded.z == 0.0f
                          : fabs((double)loaded.z / (double)stored.z / scale - 1.0) < 1.0e-6);
    CHECK(kept, "every component by the same factor");
    double reach = (double)prepared[0].invMassB *
                   ((double)fabsf(loaded.x) + (double)fabsf(loaded.y) + (double)fabsf(loaded.z));
    CHECK(reach <= (double)M3_WARM_LIMIT, "and the warm start's linear reach is within the limit");
    free(prepared);

    StepsOnFinite(&scene, "WARM_JOINTS", M3_GOLDEN_WARM_JOINTS);
    m3DestroyWorld(world);
}

// The heaviest body the limits allow, twelve boxes at the density and
// size limits (9.6e19 kg), resting on a kinematic box spinning at the
// speed limit: its contact points move at about 1e9 m/s. After some
// steps the world snapshots, a second world restores it, and both step
// on to the same hash, finite throughout; the restored world's own
// snapshot restores too.
static void TestHeaviestAgainstKinematicRoundTrip(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3World* copy = NewWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.angularVelocity = (m3Vec3){0.0f, M3_SPEED_LIMIT, 0.0f};
    m3BodyId spinner = NewBody(world, &bd);
    (void)NewBoxShape(world, spinner, &sd, (m3Vec3){M3_LOCAL_LIMIT, 1.0f, M3_LOCAL_LIMIT});
    bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, (double)M3_LOCAL_LIMIT + 1.0, 0.0};
    m3BodyId heavy = NewBody(world, &bd);
    sd.density = M3_DENSITY_LIMIT;
    for (int32_t i = 0; i < 12; ++i)
    {
        (void)NewBoxShape(world, heavy, &sd,
                          (m3Vec3){M3_LOCAL_LIMIT, M3_LOCAL_LIMIT, M3_LOCAL_LIMIT});
    }
    float mass = 1.0f / world->bodies.invMass[heavy.index1 - 1];
    CHECK(mass > 9.0e19f && mass <= M3_MASS_LIMIT, "the body is near the heaviest allowed");
    for (int32_t i = 0; i < 20; ++i)
    {
        m3World_Step(world, s_dt, 4);
    }
    CHECK(world->contacts.pairCount > 0, "the bodies touch");

    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL, "snapshot storage");
    if (snap == NULL)
    {
        m3DestroyWorld(world);
        m3DestroyWorld(copy);
        return;
    }
    CHECK(Snapshot(world, snap, size) == size, "the world snapshots");
    CHECK(Restore(copy, snap, size), "and a second world restores it");
    CHECK(m3World_Hash(copy) == m3World_Hash(world), "to the same state");
    for (int32_t i = 0; i < 20; ++i)
    {
        m3World_Step(world, s_dt, 4);
        m3World_Step(copy, s_dt, 4);
    }
    CHECK(m3World_Hash(copy) == m3World_Hash(world), "both step on alike");
    m3Vec3 v = m3Body_GetLinearVelocity(copy, heavy);
    m3Vec3 w = m3Body_GetAngularVelocity(copy, heavy);
    CHECK(Finite3(v) && Finite3(w), "finite");
    CHECK(Snapshot(copy, snap, size) == size && Restore(copy, snap, size),
          "and the restored world's own snapshot restores");
    free(snap);
    m3DestroyWorld(world);
    m3DestroyWorld(copy);
}

int main(void)
{
    TestHoldAtItsEdges();
    TestHoldOnContacts();
    TestHoldOnJoints();
    TestHeaviestAgainstKinematicRoundTrip();
    if (s_failures == 0)
    {
        printf("test_warmhold: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
