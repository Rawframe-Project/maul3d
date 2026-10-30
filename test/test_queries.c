// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Events and queries gate: contact begin/end streams against
// hand-counted expectations, analytic ray hits on every shape family,
// closest-of-many ordering, casts and overlaps.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/shape.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void TestContactEvents(void)
{
    // A ball drops onto the plane: exactly one begin event over the
    // whole approach and rest. A commanded launch upward separates
    // them: exactly one end event.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = NewPlaneShape(world, ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId ballShape = NewSphereShape(world, ball, &sd, &sphere);

    int32_t begins = 0;
    int32_t ends = 0;
    m3ContactBeginEvent firstBegin;
    memset(&firstBegin, 0, sizeof(firstBegin));
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        const m3ContactBeginEvent* ev = m3World_GetContactEvents(world).beginEvents;
        int32_t n = m3World_GetContactEvents(world).beginCount;
        if (n > 0 && begins == 0)
        {
            firstBegin = ev[0];
        }
        begins += n;
        n = m3World_GetContactEvents(world).endCount;
        ends += n;
    }
    CHECK(begins == 1, "one begin event for the landing");
    CHECK(ends == 0, "no end event while resting");
    CHECK(firstBegin.shapeIdA.index1 == floorShape.index1 &&
              firstBegin.shapeIdB.index1 == ballShape.index1,
          "the begin event names the pair in canonical order");
    CHECK(m3Shape_IsValid(world, firstBegin.shapeIdA) &&
              m3Shape_IsValid(world, firstBegin.shapeIdB),
          "event ids are live handles");

    m3Body_SetLinearVelocity(world, ball, (m3Vec3){0.0f, 8.0f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t n = m3World_GetContactEvents(world).endCount;
        ends += n;
    }
    CHECK(ends == 1, "one end event for the launch");

    // Events are transient: a snapshot restore clears them.
    int32_t snapBytes = m3World_GetSnapshotSize(world);
    void* snap = malloc((size_t)snapBytes);
    CHECK(Snapshot(world, snap, snapBytes) == snapBytes, "snapshot writes");
    CHECK(Restore(world, snap, snapBytes), "snapshot restores");
    int32_t n = m3World_GetContactEvents(world).beginCount;
    CHECK(n == 0, "restore clears begin events");
    n = m3World_GetContactEvents(world).endCount;
    CHECK(n == 0, "restore clears end events");
    free(snap);
    m3DestroyWorld(world);
}

static void TestRayHitsEveryFamily(void)
{
    // Analytic hits: one shape of every family at a known place, one
    // ray each, fraction and normal checked by hand.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();

    bd.position = (m3Pos3){10.0, 0.0, 0.0};
    m3BodyId sphereBody = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(world, sphereBody, &sd, &sphere);

    bd.position = (m3Pos3){10.0, 10.0, 0.0};
    m3BodyId boxBody = NewBody(world, &bd);
    NewBoxShape(world, boxBody, &sd, (m3Vec3){1.0f, 1.0f, 1.0f});

    bd.position = (m3Pos3){10.0, 20.0, 0.0};
    m3BodyId capBody = NewBody(world, &bd);
    m3Capsule capsule = {{0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.5f};
    NewCapsuleShape(world, capBody, &sd, &capsule);

    bd.position = (m3Pos3){8.0, 30.0, -2.0};
    m3BodyId meshBody = NewBody(world, &bd);
    m3Vec3 verts[4] = {
        {0.0f, 0.0f, 0.0f}, {4.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 4.0f}, {4.0f, 0.0f, 4.0f}};
    uint16_t tris[6] = {0, 2, 1, 1, 2, 3};
    CHECK(m3Shape_IsValid(world, NewMeshShape(world, meshBody, &sd, verts, 4, tris, 2)),
          "the query mesh builds");

    // Sphere: from the origin along +x, entry at x = 9, fraction 0.45.
    m3CastHit hit = {0};
    bool found = m3World_CastRayClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f},
                                        m3DefaultQueryFilter(), &hit);
    CHECK(found, "the sphere ray hits");
    CHECK(hit.fraction > 0.449f && hit.fraction < 0.451f, "sphere entry fraction");
    CHECK(hit.normal.x < -0.99f, "sphere entry normal faces the ray");
    CHECK(hit.point.x > 8.99 && hit.point.x < 9.01, "sphere entry point");

    // Box: entry at x = 9, fraction (9 - 0) / 20.
    found = m3World_CastRayClosest(world, (m3Pos3){0.0, 10.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.fraction > 0.449f && hit.fraction < 0.451f, "box entry fraction");
    CHECK(hit.normal.x < -0.99f, "box entry normal");

    // Capsule side: entry at x = 9.5.
    found = m3World_CastRayClosest(world, (m3Pos3){0.0, 20.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.fraction > 0.474f && hit.fraction < 0.476f, "capsule side fraction");

    // Mesh: straight down onto the quad at (10, 30, 0).
    found = m3World_CastRayClosest(world, (m3Pos3){10.0, 35.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.fraction > 0.499f && hit.fraction < 0.501f, "mesh hit fraction");
    CHECK(hit.normal.y > 0.99f, "mesh hit normal is up");

    // A miss stays a miss.
    found = m3World_CastRayClosest(world, (m3Pos3){0.0, 50.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(!found, "empty space misses");
    found = m3World_CastRayClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){0.0f, 0.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a zero ray misses by contract");
    m3DestroyWorld(world);
}

// The closest casts' contract: each clears hitOut first. A hit fills it and
// returns true; a miss returns false and leaves zeros; a NaN translation
// does too and counts one misuse; a null hitOut misses and counts one.
// Each cast aims at a box below it, away from it, with no hitOut, and
// along a NaN.
static void TestCastHitContract(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId box = NewBoxShape(world, NewBody(world, &bd), &sd, (m3Vec3){2.0f, 0.5f, 2.0f});
    m3Pos3 above = {0.0, 3.0, 0.0};
    m3Vec3 down = {0.0f, -5.0f, 0.0f};
    m3Vec3 up = {0.0f, 5.0f, 0.0f};
    m3Vec3 cube = {0.2f, 0.2f, 0.2f};
    m3Quat still = m3MakeIdentityQuat();
    static const m3Vec3 cloud[2] = {{-0.2f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f}};
    m3QueryFilter all = m3DefaultQueryFilter();
    for (int32_t cast = 0; cast < 6; ++cast)
    {
        for (int32_t aim = 0; aim < 4; ++aim)
        {
            m3Vec3 t = aim == 1 ? up : (aim == 3 ? (m3Vec3){0.0f, NAN, 0.0f} : down);
            // A stale hit the cast must not leave behind.
            m3CastHit hit = {{77u, 5u}, {1.0, 2.0, 3.0}, {4.0f, 5.0f, 6.0f}, 0.25f};
            const m3CastHit zero = {0};
            m3CastHit* out = aim == 2 ? NULL : &hit;
            uint64_t misuse = m3World_GetCounters(world).misuse;
            bool found = false;
            switch (cast)
            {
            case 0:
                found = m3World_CastRayClosest(world, above, t, all, out);
                break;
            case 1:
                found = m3World_CastSphereClosest(world, above, 0.2f, t, all, out);
                break;
            case 2:
                found =
                    m3World_CastCapsuleClosest(world, above, cloud[0], cloud[1], 0.1f, t, all, out);
                break;
            case 3:
                found = m3World_CastBoxClosest(world, above, cube, still, t, all, out);
                break;
            case 4:
                found = m3World_CastHullClosest(world, above, cloud, 2, t, all, out);
                break;
            default:
                found = m3World_CastMover(world, above, 0.3f, 0.2f, t, out);
                break;
            }
            uint64_t counted = m3World_GetCounters(world).misuse - misuse;
            bool ok = false;
            if (aim == 0)
            {
                ok = found && hit.shapeId.index1 == box.index1 && hit.fraction > 0.0f &&
                     hit.fraction < 1.0f && counted == 0u;
            }
            else if (aim == 1 || aim == 3)
            {
                ok = !found && memcmp(&hit, &zero, sizeof(hit)) == 0 &&
                     counted == (aim == 3 ? 1u : 0u);
            }
            else
            {
                ok = !found && counted == 1u;
            }
            if (!ok)
            {
                printf("  cast %d, aim %d: found %d, misuse %d\n", (int)cast, (int)aim, found,
                       (int)counted);
            }
            CHECK(ok, "a hit fills hitOut, a miss or a refusal zeroes it, a null one counts");
        }
    }
    m3DestroyWorld(world);
}

static void TestRayClosestOfMany(void)
{
    // Two spheres on the ray: the nearer one wins; from the other
    // side, the other one wins. Plus the plane pass.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();

    bd.position = (m3Pos3){5.0, 0.0, 0.0};
    m3BodyId nearBody = NewBody(world, &bd);
    m3Sphere s1 = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId nearShape = NewSphereShape(world, nearBody, &sd, &s1);
    bd.position = (m3Pos3){15.0, 0.0, 0.0};
    m3BodyId farBody = NewBody(world, &bd);
    m3ShapeId farShape = NewSphereShape(world, farBody, &sd, &s1);

    m3CastHit hit = {0};
    bool found = m3World_CastRayClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f},
                                        m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.shapeId.index1 == nearShape.index1, "the nearer sphere wins");
    found = m3World_CastRayClosest(world, (m3Pos3){20.0, 0.0, 0.0}, (m3Vec3){-20.0f, 0.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.shapeId.index1 == farShape.index1,
          "from the other side the other sphere wins");

    // The infinite plane through the dedicated pass.
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, -2.0f};
    NewPlaneShape(world, ground, &sd, &floor);
    found = m3World_CastRayClosest(world, (m3Pos3){0.0, 3.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.fraction > 0.499f && hit.fraction < 0.501f, "the plane pass hits");
    m3DestroyWorld(world);
}

static void TestShapeCasts(void)
{
    // A sphere cast toward a wall touches when the SURFACES meet:
    // the fraction accounts for both radii, checked by hand.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){10.0, 0.0, 0.0};
    m3BodyId wall = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, wall, &sd, (m3Vec3){0.5f, 2.0f, 2.0f});

    // Cast a 0.5-radius sphere from the origin: the box face sits at
    // x = 9.5, the sphere surface leads by 0.5, so centers touch at
    // x = 9.0: fraction 9/20 = 0.45 (minus the slop skin).
    m3CastHit hit = {0};
    bool found =
        m3World_CastSphereClosest(world, (m3Pos3){0.0, 0.0, 0.0}, 0.5f, (m3Vec3){20.0f, 0.0f, 0.0f},
                                  m3DefaultQueryFilter(), &hit);
    CHECK(found, "the sphere cast hits");
    CHECK(hit.fraction > 0.44f && hit.fraction < 0.4505f, "the cast fraction counts both skins");
    CHECK(hit.normal.x < -0.99f, "the cast normal faces the caster");

    // A capsule cast: the lower cap leads; the analytic touch uses
    // the cap's forward surface exactly like the sphere.
    found = m3World_CastCapsuleClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){0.0f, -0.5f, 0.0f},
                                       (m3Vec3){0.0f, 0.5f, 0.0f}, 0.3f,
                                       (m3Vec3){20.0f, 0.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found, "the capsule cast hits");
    CHECK(hit.fraction > 0.45f && hit.fraction < 0.4605f, "the capsule fraction is analytic");

    // Start overlapped: the documented contract is a hit at zero.
    found = m3World_CastSphereClosest(world, (m3Pos3){9.6, 0.0, 0.0}, 0.5f,
                                      (m3Vec3){5.0f, 0.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.fraction == 0.0f, "a start-overlapped cast hits at zero");

    // A cast that misses stays a miss.
    found = m3World_CastSphereClosest(world, (m3Pos3){0.0, 10.0, 0.0}, 0.5f,
                                      (m3Vec3){20.0f, 0.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a clear cast misses");
    m3DestroyWorld(world);
}

static void TestRayStartInsideContract(void)
{
    // The documented family-by-family contract: rays MISS shapes
    // they start inside (front faces only, everywhere).
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();

    bd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId a = NewBody(world, &bd);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(world, a, &sd, &ball);
    m3CastHit hit = {0};
    bool found = m3World_CastRayClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){5.0f, 0.0f, 0.0f},
                                        m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a ray starting inside a sphere misses it");

    bd.position = (m3Pos3){10.0, 0.0, 0.0};
    m3BodyId b = NewBody(world, &bd);
    NewBoxShape(world, b, &sd, (m3Vec3){1.0f, 1.0f, 1.0f});
    found = m3World_CastRayClosest(world, (m3Pos3){10.0, 0.0, 0.0}, (m3Vec3){5.0f, 0.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a ray starting inside a hull misses it");

    bd.position = (m3Pos3){20.0, 0.0, 0.0};
    m3BodyId c = NewBody(world, &bd);
    m3Capsule capsule = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.5f};
    NewCapsuleShape(world, c, &sd, &capsule);
    found = m3World_CastRayClosest(world, (m3Pos3){20.0, 0.0, 0.0}, (m3Vec3){5.0f, 0.0f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a ray starting inside a capsule misses it");
    m3DestroyWorld(world);
}

static void TestMultiHitSorted(void)
{
    // Two spheres and the floor plane along one ray: three hits,
    // sorted by fraction, and the capacity drop takes the far end.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){4.0, 4.0, 0.0};
    m3BodyId nearBody = NewBody(world, &bd);
    m3Sphere s1 = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, nearBody, &sd, &s1);
    bd.position = (m3Pos3){8.0, 2.0, 0.0};
    m3BodyId midBody = NewBody(world, &bd);
    NewSphereShape(world, midBody, &sd, &s1);

    // A ray sloping down through both spheres into the floor.
    m3CastHit hits[8];
    int32_t n = m3World_CastRayAll(world, (m3Pos3){0.0, 6.0, 0.0}, (m3Vec3){16.0f, -8.0f, 0.0f},
                                   m3DefaultQueryFilter(), hits, 8);
    CHECK(n == 3, "three hits along the ray");
    CHECK(hits[0].fraction < hits[1].fraction && hits[1].fraction < hits[2].fraction,
          "the hits are sorted by fraction");
    CHECK(hits[2].normal.y > 0.99f, "the last hit is the floor");

    // Capacity two: the FAR hit drops.
    m3CastHit two[2];
    int32_t n2 = m3World_CastRayAll(world, (m3Pos3){0.0, 6.0, 0.0}, (m3Vec3){16.0f, -8.0f, 0.0f},
                                    m3DefaultQueryFilter(), two, 2);
    CHECK(n2 == 3, "a short array still counts every hit");
    CHECK(two[0].fraction == hits[0].fraction && two[1].fraction == hits[1].fraction,
          "the near hits survive the cap");
    m3DestroyWorld(world);
}

static void TestPointAndOverlaps(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = NewPlaneShape(world, ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){5.0, 2.0, 0.0};
    m3BodyId boxBody = NewBody(world, &bd);
    m3ShapeId boxShape = NewBoxShape(world, boxBody, &sd, (m3Vec3){1.0f, 1.0f, 1.0f});

    // Point containment per family.
    m3ShapeId inBox = m3World_TestPoint(world, (m3Pos3){5.2, 2.3, 0.4});
    CHECK(inBox.index1 == boxShape.index1, "the point inside the box finds the box");
    m3ShapeId below = m3World_TestPoint(world, (m3Pos3){100.0, -0.5, 0.0});
    CHECK(below.index1 == floorShape.index1, "a point below the floor is inside the half space");
    m3ShapeId nowhere = m3World_TestPoint(world, (m3Pos3){5.0, 8.0, 0.0});
    CHECK(nowhere.index1 == 0, "open air contains nothing");

    // Overlaps: the AABB box catches the box, the sphere probes reach.
    m3ShapeId found[8];
    int32_t n = m3World_OverlapAabb(world, (m3WorldAabb){{4.0, 1.0, -1.0}, {6.0, 3.0, 1.0}},
                                    m3DefaultQueryFilter(), found, 8);
    CHECK(n == 1 && found[0].index1 == boxShape.index1, "the AABB overlap finds the box");
    n = m3World_OverlapSphere(world, (m3Pos3){5.0, 4.5, 0.0}, 1.6f, m3DefaultQueryFilter(), found,
                              8);
    CHECK(n == 1 && found[0].index1 == boxShape.index1, "the sphere overlap reaches the box");
    n = m3World_OverlapSphere(world, (m3Pos3){5.0, 4.5, 0.0}, 1.4f, m3DefaultQueryFilter(), found,
                              8);
    CHECK(n == 0, "just out of reach finds nothing");
    n = m3World_OverlapSphere(world, (m3Pos3){0.0, 0.5, 0.0}, 0.6f, m3DefaultQueryFilter(), found,
                              8);
    CHECK(n == 1 && found[0].index1 == floorShape.index1, "the sphere overlap reaches the floor");
    m3DestroyWorld(world);
}

static void TestSensorPassThrough(void)
{
    // A ball falls THROUGH a static sensor box: begin fires on
    // entry, end on exit, and the fall never slows (the free-fall
    // analytic is the no-response proof). The contact streams stay
    // silent for the sensor pair.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId zone = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.isSensor = true;
    CHECK(m3Shape_IsValid(world, NewBoxShape(world, zone, &sd, (m3Vec3){1.0f, 0.5f, 1.0f})),
          "the sensor zone builds");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 6.0, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3ShapeDef bsd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    NewSphereShape(world, ball, &bsd, &sphere);

    int32_t sensorBegins = 0;
    int32_t sensorEnds = 0;
    int32_t contactBegins = 0;
    for (int32_t i = 0; i < 90; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t n = m3World_GetSensorEvents(world).beginCount;
        sensorBegins += n;
        n = m3World_GetSensorEvents(world).endCount;
        sensorEnds += n;
        n = m3World_GetContactEvents(world).beginCount;
        contactBegins += n;
    }
    CHECK(sensorBegins == 1, "one sensor begin on entry");
    CHECK(sensorEnds == 1, "one sensor end on exit");
    CHECK(contactBegins == 0, "the contact stream stays silent for sensors");
    // Free fall for 1.5 s from rest: y = 6 - 0.5 * 10 * 1.5^2 = -5.25
    // (substep integration lands within a band).
    m3Pos3 p = m3Body_GetPosition(world, ball);
    CHECK(p.y < -5.0 && p.y > -5.6, "the fall never slowed: sensors do not respond");
    m3DestroyWorld(world);
}

static void TestSensorSleepAndBullets(void)
{
    // A sensor neither wakes a sleeper nor stops a bullet, and a
    // body asleep INSIDE a sensor holds its begin without an end.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.4, 0.0};
    m3BodyId sleeper = NewBody(world, &bd);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
    NewSphereShape(world, sleeper, &sd, &ball);

    // A sensor zone AROUND the resting spot, created before sleep.
    m3BodyDef zd = m3DefaultBodyDef();
    zd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId zone = NewBody(world, &zd);
    m3ShapeDef zsd = m3DefaultShapeDef();
    zsd.isSensor = true;
    NewBoxShape(world, zone, &zsd, (m3Vec3){2.0f, 1.0f, 2.0f});

    int32_t begins = 0;
    int32_t ends = 0;
    for (int32_t i = 0; i < 120; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t n = m3World_GetSensorEvents(world).beginCount;
        begins += n;
        n = m3World_GetSensorEvents(world).endCount;
        ends += n;
    }
    m3Vec3 v = m3Body_GetLinearVelocity(world, sleeper);
    CHECK(v.x == 0.0f && v.y == 0.0f && v.z == 0.0f,
          "the body sleeps INSIDE the sensor (overlap wakes no one)");
    CHECK(begins == 1 && ends == 0, "the begin holds through sleep with no end");

    // A bullet crosses a sensor wall without slowing.
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){10.0, 5.0, 0.0};
    m3BodyId wall = NewBody(world, &wd);
    m3ShapeDef wsd = m3DefaultShapeDef();
    wsd.isSensor = true;
    NewBoxShape(world, wall, &wsd, (m3Vec3){0.05f, 2.0f, 2.0f});

    bd.position = (m3Pos3){4.0, 5.0, 0.0};
    bd.linearVelocity = (m3Vec3){200.0f, 0.0f, 0.0f};
    bd.gravityScale = 0.0f;
    bd.enableBullet = true;
    m3BodyId bullet = NewBody(world, &bd);
    m3Sphere small = {{0.0f, 0.0f, 0.0f}, 0.1f};
    NewSphereShape(world, bullet, &sd, &small);

    for (int32_t i = 0; i < 10; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 bp = m3Body_GetPosition(world, bullet);
    CHECK(bp.x > 20.0, "the bullet crosses the sensor wall unimpeded");
    m3DestroyWorld(world);
}

// Coverage: the mesh and plane branches of every cast and
// overlap entry were unexercised (the coverage run named them), so
// this scene points casts at a mesh ramp, a plane, a capsule, and a
// hull, and probes every point-inside family.
static void TestCastsAgainstMeshAndPlane(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.meshCapacity = 1;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = NewPlaneShape(world, ground, &sd, &floor);

    m3BodyDef md = m3DefaultBodyDef();
    md.position = (m3Pos3){4.0, 1.0, 0.0};
    m3BodyId meshBody = NewBody(world, &md);
    m3Vec3 quad[4] = {
        {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, 1.0f}, {-1.0f, 0.0f, 1.0f}};
    uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    m3ShapeId meshShape = NewMeshShape(world, meshBody, &sd, quad, 4, tris, 2);

    // A sphere cast straight down onto the mesh quad at y = 1:
    // center stops at 1.3, so fraction = (3 - 1.3) / 3.
    m3CastHit hit = {0};
    bool found =
        m3World_CastSphereClosest(world, (m3Pos3){4.0, 4.0, 0.0}, 0.3f, (m3Vec3){0.0f, -3.0f, 0.0f},
                                  m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.shapeId.index1 == meshShape.index1, "the sphere cast hits the mesh");
    CHECK(hit.fraction > 0.895f && hit.fraction < 0.905f, "the mesh cast fraction is analytic");

    // The same cast far from the quad falls through to the plane at
    // y = 0: fraction = (4 - 0.3) / 8.
    found = m3World_CastSphereClosest(world, (m3Pos3){-6.0, 4.0, 0.0}, 0.3f,
                                      (m3Vec3){0.0f, -8.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.shapeId.index1 == floorShape.index1, "the sphere cast hits the plane");
    CHECK(hit.fraction > 0.455f && hit.fraction < 0.4635f, "the plane cast fraction is analytic");

    // A capsule cast onto the plane, and one starting overlapped.
    found = m3World_CastCapsuleClosest(world, (m3Pos3){-6.0, 2.0, 4.0}, (m3Vec3){0.0f, -0.4f, 0.0f},
                                       (m3Vec3){0.0f, 0.4f, 0.0f}, 0.2f,
                                       (m3Vec3){0.0f, -4.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.shapeId.index1 == floorShape.index1, "the capsule cast hits the plane");
    CHECK(hit.fraction > 0.34f && hit.fraction < 0.36f, "the capsule fraction is analytic");
    found = m3World_CastSphereClosest(world, (m3Pos3){-6.0, 0.1, 0.0}, 0.3f,
                                      (m3Vec3){0.0f, -1.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.fraction == 0.0f, "a cast born inside the plane reports zero");

    // A cast that misses everything reports the miss.
    found = m3World_CastSphereClosest(world, (m3Pos3){0.0, 50.0, 0.0}, 0.3f,
                                      (m3Vec3){1.0f, 0.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a cast into open air misses");

    m3DestroyWorld(world);
}

static void TestOverlapAndInsideFamilies(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.meshCapacity = 1;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = NewPlaneShape(world, ground, &sd, &floor);
    m3Vec3 quad[4] = {
        {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, 1.0f}, {-1.0f, 0.0f, 1.0f}};
    uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    m3BodyDef md = m3DefaultBodyDef();
    md.position = (m3Pos3){6.0, 2.0, 0.0};
    m3BodyId meshBody = NewBody(world, &md);
    m3ShapeId meshShape = NewMeshShape(world, meshBody, &sd, quad, 4, tris, 2);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){-4.0, 1.0, 0.0};
    m3BodyId pillBody = NewBody(world, &bd);
    m3Capsule pill = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.4f};
    m3ShapeId pillShape = NewCapsuleShape(world, pillBody, &sd, &pill);
    bd.position = (m3Pos3){-8.0, 1.0, 0.0};
    m3BodyId rockBody = NewBody(world, &bd);
    m3Vec3 cloud[6] = {{0.5f, 0.0f, 0.0f},  {-0.5f, 0.0f, 0.0f}, {0.0f, 0.5f, 0.0f},
                       {0.0f, -0.5f, 0.0f}, {0.0f, 0.0f, 0.5f},  {0.0f, 0.0f, -0.5f}};
    m3ShapeId rockShape = NewHullShape(world, rockBody, &sd, cloud, 6);

    // OverlapSphere against the mesh: within a radius of a vertex
    // hits, just beyond misses (the vertex-reach bound).
    m3ShapeId found[8];
    int32_t n = m3World_OverlapSphere(world, (m3Pos3){7.2, 2.0, 1.2}, 0.4f, m3DefaultQueryFilter(),
                                      found, 8);
    CHECK(n == 1 && found[0].index1 == meshShape.index1, "the sphere reach finds the mesh");
    n = m3World_OverlapSphere(world, (m3Pos3){7.2, 2.0, 1.2}, 0.1f, m3DefaultQueryFilter(), found,
                              8);
    CHECK(n == 0, "just out of reach finds nothing");

    // OverlapSphere and OverlapAabb against the plane.
    n = m3World_OverlapSphere(world, (m3Pos3){0.0, 0.2, 0.0}, 0.4f, m3DefaultQueryFilter(), found,
                              8);
    CHECK(n == 1 && found[0].index1 == floorShape.index1, "the sphere reaches the plane");
    n = m3World_OverlapAabb(world, (m3WorldAabb){{-1.0, -0.5, -1.0}, {1.0, 0.5, 1.0}},
                            m3DefaultQueryFilter(), found, 8);
    CHECK(n == 1 && found[0].index1 == floorShape.index1, "the AABB straddles the plane");
    n = m3World_OverlapAabb(world, (m3WorldAabb){{-1.0, 2.0, -1.0}, {1.0, 3.0, 1.0}},
                            m3DefaultQueryFilter(), found, 8);
    CHECK(n == 0, "an AABB above the plane is clear");

    // Point-inside for the remaining families: capsule (cylinder
    // and cap regions), hull, and the mesh open-surface rule.
    m3ShapeId inside = m3World_TestPoint(world, (m3Pos3){-4.0, 1.2, 0.1});
    CHECK(inside.index1 == pillShape.index1, "a point in the capsule cylinder is inside");
    inside = m3World_TestPoint(world, (m3Pos3){-4.0, 1.8, 0.0});
    CHECK(inside.index1 == pillShape.index1, "a point in the capsule cap is inside");
    inside = m3World_TestPoint(world, (m3Pos3){-4.0, 2.5, 0.0});
    CHECK(inside.index1 == 0, "above the capsule is outside");
    inside = m3World_TestPoint(world, (m3Pos3){-8.0, 1.1, 0.1});
    CHECK(inside.index1 == rockShape.index1, "a point in the hull is inside");
    inside = m3World_TestPoint(world, (m3Pos3){6.0, 2.0, 0.0});
    CHECK(inside.index1 == 0, "a point ON a mesh is outside: meshes are open surfaces");

    m3DestroyWorld(world);
}

// A point far from a voxel chunk lies outside it, on every axis and both
// sides, however far. A point in a filled cell is still inside.
static void TestFarPointsMissVoxels(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    static uint8_t cells[16 * 16 * 16];
    memset(cells, 1, sizeof(cells));
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId chunk = NewVoxelChunkShape(world, ground, &sd, cells, NULL, 0.25f);
    CHECK(m3World_TestPoint(world, (m3Pos3){0.5, 0.5, 0.5}).index1 == chunk.index1,
          "a point in a filled cell is inside the chunk");
    static const double far[] = {1.0e10, 3.0e9, -3.0e9, -1.0e10, 1.0e30};
    int32_t outside = 0;
    for (size_t i = 0; i < sizeof(far) / sizeof(far[0]); ++i)
    {
        outside += m3World_TestPoint(world, (m3Pos3){far[i], 0.5, 0.5}).index1 == 0 ? 1 : 0;
        outside += m3World_TestPoint(world, (m3Pos3){0.5, far[i], 0.5}).index1 == 0 ? 1 : 0;
        outside += m3World_TestPoint(world, (m3Pos3){0.5, 0.5, far[i]}).index1 == 0 ? 1 : 0;
    }
    CHECK(outside == 3 * (int32_t)(sizeof(far) / sizeof(far[0])),
          "a far point on any axis is outside the chunk");
    m3DestroyWorld(world);
}

// The generic casts. Boxes carry orientation, hulls carry
// their cloud, and every cast keeps the family contracts.
static void TestGenericCasts(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3World* world = NewWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = NewPlaneShape(world, ground, &sd, &floor);

    // An axis-aligned unit box cast onto the plane: the lower face
    // leads, contact when center reaches 0.5: fraction = 3.5 / 8
    // within the conservative-advance slop band.
    m3Quat identity = {0.0f, 0.0f, 0.0f, 1.0f};
    m3CastHit hit = {0};
    bool found =
        m3World_CastBoxClosest(world, (m3Pos3){0.0, 4.0, 0.0}, (m3Vec3){0.5f, 0.5f, 0.5f}, identity,
                               (m3Vec3){0.0f, -8.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.shapeId.index1 == floorShape.index1, "the box cast hits the plane");
    CHECK(hit.fraction > 0.430f && hit.fraction < 0.440f, "the aligned fraction is analytic");

    // The same box yawed 45 degrees about z: a corner leads, the
    // vertical reach grows to sqrt(2)/2, so contact comes earlier:
    // fraction = (4 - 0.7071) / 8 minus the slop band.
    float s45 = 0.3826834f; // sin(pi/8)
    float c45 = 0.9238795f; // cos(pi/8)
    m3Quat yaw45 = {0.0f, 0.0f, s45, c45};
    found =
        m3World_CastBoxClosest(world, (m3Pos3){0.0, 4.0, 0.0}, (m3Vec3){0.5f, 0.5f, 0.5f}, yaw45,
                               (m3Vec3){0.0f, -8.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found, "the rotated box cast hits");
    CHECK(hit.fraction > 0.404f && hit.fraction < 0.412f,
          "the rotated fraction shows the leading corner");

    // A tetrahedron cloud cast onto a box target: the cloud's
    // lowest vertex leads.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){5.0, 1.0, 0.0};
    m3BodyId crateBody = NewBody(world, &bd);
    m3ShapeId crate = NewBoxShape(world, crateBody, &sd, (m3Vec3){1.0f, 1.0f, 1.0f});
    m3Vec3 tetra[4] = {
        {0.0f, -0.4f, 0.0f}, {0.3f, 0.4f, 0.0f}, {-0.3f, 0.4f, 0.3f}, {-0.3f, 0.4f, -0.3f}};
    found = m3World_CastHullClosest(world, (m3Pos3){5.0, 6.0, 0.0}, tetra, 4,
                                    (m3Vec3){0.0f, -8.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.shapeId.index1 == crate.index1, "the hull cast hits the crate");
    // Crate top at y = 2, lowest cloud point 0.4 below base:
    // fraction = (6 - 0.4 - 2) / 8 within the slop band.
    CHECK(hit.fraction > 0.443f && hit.fraction < 0.452f, "the hull fraction is analytic");

    // Start-overlapped and hostile contracts.
    found =
        m3World_CastBoxClosest(world, (m3Pos3){0.0, 0.2, 0.0}, (m3Vec3){0.5f, 0.5f, 0.5f}, identity,
                               (m3Vec3){0.0f, -1.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(found && hit.fraction == 0.0f, "a box born inside the plane reports zero");
    found =
        m3World_CastBoxClosest(world, (m3Pos3){0.0, 4.0, 0.0}, (m3Vec3){0.5f, 0.0f, 0.5f}, identity,
                               (m3Vec3){0.0f, -8.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a flat box misses by contract");
    found = m3World_CastHullClosest(world, (m3Pos3){0.0, 4.0, 0.0}, tetra, 1,
                                    (m3Vec3){0.0f, -8.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a one-point cloud misses by contract (that is a ray)");
    m3Vec3 poisoned[4] = {tetra[0], tetra[1], {NAN, 0.0f, 0.0f}, tetra[3]};
    uint64_t misuse = m3World_GetCounters(world).misuse;
    found = m3World_CastHullClosest(world, (m3Pos3){0.0, 4.0, 0.0}, poisoned, 4,
                                    (m3Vec3){0.0f, -8.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(!found && m3World_GetCounters(world).misuse == misuse + 1,
          "a cloud with a non-finite point misses and counts one misuse");
    found =
        m3World_CastBoxClosest(world, (m3Pos3){40.0, 4.0, 40.0}, (m3Vec3){0.5f, 0.5f, 0.5f},
                               identity, (m3Vec3){0.0f, -2.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a cast into open air misses");
    // The float budget under stress: a finite translation whose
    // square overflows float would mint NaN inside the kernels, so
    // every cast path refuses it with a clean miss.
    found = m3World_CastSphereClosest(world, (m3Pos3){0.0, 4.0, 0.0}, 0.5f,
                                      (m3Vec3){1.0e30f, 0.0f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a planetary sphere cast misses by contract");
    found =
        m3World_CastBoxClosest(world, (m3Pos3){0.0, 4.0, 0.0}, (m3Vec3){0.5f, 0.5f, 0.5f}, identity,
                               (m3Vec3){0.0f, -1.0e30f, 0.0f}, m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a planetary box cast misses by contract");
    found = m3World_CastRayClosest(world, (m3Pos3){0.0, 4.0, 0.0}, (m3Vec3){0.0f, -1.0e30f, 0.0f},
                                   m3DefaultQueryFilter(), &hit);
    CHECK(!found, "a planetary ray misses by contract");
    m3CastHit all[4];
    CHECK(m3World_CastRayAll(world, (m3Pos3){0.0, 4.0, 0.0}, (m3Vec3){0.0f, -1.0e30f, 0.0f},
                             m3DefaultQueryFilter(), all, 4) == 0,
          "a planetary all-hits ray reports nothing");
    m3DestroyWorld(world);
}

static void TestOverlapBeyondTwoHundredFiftySix(void)
{
    // 300 static spheres in one box: an overlap query with room for
    // all of them returns all 300 in ascending order, and a smaller
    // array keeps exactly the lowest slots, whatever the tree order.
    enum
    {
        BALLS = 300
    };
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = BALLS + 4;
    def.shapeCapacity = BALLS + 4;
    m3World* world = NewWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef gd = m3DefaultBodyDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = NewPlaneShape(world, NewBody(world, &gd), &sd, &floor);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.2f};
    for (int32_t i = 0; i < BALLS; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.position = (m3Pos3){(double)(i % 20), 1.0, (double)(i / 20)};
        NewSphereShape(world, NewBody(world, &bd), &sd, &sphere);
    }
    static m3ShapeId all[BALLS + 8];
    int32_t n = m3World_OverlapAabb(world, (m3WorldAabb){{-1.0, 0.5, -1.0}, {21.0, 1.5, 16.0}},
                                    m3DefaultQueryFilter(), all, BALLS + 8);
    bool ascending = true;
    for (int32_t i = 1; i < n; ++i)
    {
        ascending = ascending && all[i - 1].index1 < all[i].index1;
    }
    CHECK(n == BALLS && ascending, "all 300 overlaps come back in ascending order");

    m3ShapeId low[10];
    int32_t k = m3World_OverlapAabb(world, (m3WorldAabb){{-1.0, 0.5, -1.0}, {21.0, 1.5, 16.0}},
                                    m3DefaultQueryFilter(), low, 10);
    CHECK(k == BALLS && low[0].index1 == all[0].index1 && low[9].index1 == all[9].index1,
          "a short array keeps the lowest slots and counts them all");

    // The mover sees the floor plane (the lowest slot, outside the
    // tree) and the ball above it, merged into one ascending list.
    m3MoverPlane planes[8];
    int32_t p = m3World_CollideMover(world, (m3Pos3){0.0, 0.45, 0.0}, 0.2f, 0.3f, 0.05f, planes, 8);
    bool planesAscending = true;
    for (int32_t i = 1; i < p; ++i)
    {
        planesAscending =
            planesAscending && planes[i - 1].shapeId.index1 < planes[i].shapeId.index1;
    }
    CHECK(p >= 2 && planesAscending, "mover planes arrive in ascending shape order");
    CHECK(planes[0].shapeId.index1 == floorShape.index1, "the floor plane comes first");
    m3DestroyWorld(world);
}

// A long, shallow cast or a fast body over a flat grid, mesh or voxel
// floor meets it where its path does, past more pieces than the 64 the
// cast and the sweep once tried in storage order. Each floor is 128 m
// along x; a 0.5 m sphere's path drops 2 m on its way across.
static m3World* LongFloor(int32_t kind, m3real* topOut)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.maximumLinearSpeed = 1.0e4f;
    m3World* world = NewWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float flat[129 * 5];
    static m3Vec3 vertices[129 * 5];
    static uint16_t indices[6 * 128 * 4];
    static uint8_t cells[16 * 16 * 16];
    *topOut = 0.0f;
    if (kind == 0)
    {
        NewHeightFieldGridShape(world, ground, &sd, flat, 129, 5, 1.0f);
    }
    else if (kind == 1)
    {
        int32_t n = 0;
        for (int32_t z = 0; z < 5; ++z)
        {
            for (int32_t x = 0; x < 129; ++x)
            {
                vertices[z * 129 + x] = (m3Vec3){(m3real)x, 0.0f, (m3real)z};
            }
        }
        for (int32_t z = 0; z < 4; ++z)
        {
            for (int32_t x = 0; x < 128; ++x)
            {
                uint16_t v00 = (uint16_t)(z * 129 + x);
                uint16_t v10 = (uint16_t)(v00 + 1);
                uint16_t v01 = (uint16_t)(v00 + 129);
                uint16_t v11 = (uint16_t)(v01 + 1);
                const uint16_t quad[6] = {v00, v11, v10, v00, v01, v11};
                memcpy(&indices[n], quad, sizeof(quad));
                n += 6;
            }
        }
        NewMeshShape(world, ground, &sd, vertices, 129 * 5, indices, n / 3);
    }
    else
    {
        // A checkerboard of 8 m cubes, so its surface holds 128 boxes.
        memset(cells, 0, sizeof(cells));
        for (int32_t z = 0; z < 16; ++z)
        {
            for (int32_t x = 0; x < 16; ++x)
            {
                cells[x + 16 * 16 * z] = (uint8_t)((x + z) % 2 == 0 ? 1 : 0);
            }
        }
        NewVoxelChunkShape(world, ground, &sd, cells, NULL, 8.0f);
        *topOut = 8.0f;
    }
    return world;
}

static void TestLongPathsMeetFarPieces(void)
{
    const char* names[3] = {"grid", "mesh", "voxel"};
    for (int32_t kind = 0; kind < 3; ++kind)
    {
        m3real top;
        m3World* world = LongFloor(kind, &top);
        // The path drops 2 m. Over the voxel chunk it runs along the
        // diagonal, so its box covers all 128 cubes, and meets a far cube
        // at fraction 0.85.
        m3real across = kind == 2 ? 120.0f : 0.0f;
        m3real meet = kind == 2 ? 0.85f : 0.5f;
        m3Pos3 start = {0.5, (double)(top + 0.5f + 2.0f * meet), kind == 2 ? 0.5 : 2.5};
        m3Vec3 path = {kind == 2 ? 120.0f : 100.0f, -2.0f, across};
        m3CastHit cast = {0};
        bool castHit =
            m3World_CastSphereClosest(world, start, 0.5f, path, m3DefaultQueryFilter(), &cast);
        printf("long cast over the %s: hit %d fraction %g\n", names[kind], (int)castHit,
               (double)cast.fraction);
        // The time of impact stops within its 5 mm tolerance of the floor.
        CHECK(castHit && fabsf(cast.fraction - meet) < 5.0e-3f && cast.normal.y > 0.99f,
              "a long shallow cast meets the floor where its path does");

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = start;
        bd.linearVelocity = m3MulSV3(60.0f, path);
        m3BodyId pellet = NewBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
        NewSphereShape(world, pellet, &sd, &ball);
        (void)m3World_Step(world, 1.0f / 60.0f, 1);
        m3Pos3 at = m3Body_GetPosition(world, pellet);
        printf("fast body over the %s: x %g y %g\n", names[kind], at.x, at.y);
        CHECK(at.y > (double)top + 0.4, "a fast body at a shallow angle stops on the floor");
        m3DestroyWorld(world);
    }
}

// A ray 0.3 m off a unit sphere's center line, or a unit capsule's
// side, meets it with the right normal from any distance a cast takes.
// The quadratic b^2 - a c once lost the normal from 10 km and gave a
// NaN hit from 1e10 m. The hit point moves only by the float
// fraction's resolution, as it does for a box.
static void TestFarRaysKeepTheirNormal(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(world, ball, &sd, &sphere);
    bd.position = (m3Pos3){0.0, 0.0, 10.0};
    m3BodyId rod = NewBody(world, &bd);
    m3Capsule capsule = {{0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}, 1.0f};
    NewCapsuleShape(world, rod, &sd, &capsule);
    const double across = sqrt(1.0 - 0.09);
    for (int32_t shape = 0; shape < 2; ++shape)
    {
        double worstNormal = 0.0;
        bool hits = true;
        bool points = true;
        for (double d = 10.0; d <= 1.0e17; d *= 10.0)
        {
            m3Pos3 origin = {d, 0.3, shape == 0 ? 0.0 : 10.0};
            m3CastHit hit = {0};
            bool found =
                m3World_CastRayClosest(world, origin, (m3Vec3){(float)(-2.0 * d), 0.0f, 0.0f},
                                       m3DefaultQueryFilter(), &hit);
            hits = hits && found && isfinite(hit.point.x) && isfinite(hit.fraction);
            worstNormal = fmax(worstNormal, fabs((double)hit.normal.x - across) +
                                                fabs((double)hit.normal.y - 0.3) +
                                                fabs((double)hit.normal.z));
            points = points && fabs(hit.point.x - across) <= 1.2e-7 * d + 1.0e-3;
        }
        printf("far rays, %s: worst normal error %.3g\n", shape == 0 ? "sphere" : "capsule",
               worstNormal);
        CHECK(hits, "a far ray hits with a finite point and fraction");
        CHECK(worstNormal <= 1.0e-5, "a far ray keeps its normal");
        CHECK(points, "a far ray's point moves only by the fraction's resolution");
    }
    m3DestroyWorld(world);
}

// A diagonal ray 0.3 m off a unit sphere's center, or a unit capsule's
// axis, keeps its normal within 2 |m| FLT_EPSILON, two float ulps of the
// origin's distance |m| at most, per meter of radius. The origin's offset
// from the shape is a float, so its components are already rounded that
// far, and the closest approach q takes that error. An axis ray rounds q
// only at q's own size, as above; a diagonal one is off by about 1e-3
// at 1e4 m.
static void TestDiagonalFarRaysKeepTheirNormal(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3World* world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(world, ball, &sd, &sphere);
    bd.position = (m3Pos3){0.0, 0.0, 10.0};
    m3BodyId rod = NewBody(world, &bd);
    m3Capsule capsule = {{0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}, 1.0f};
    NewCapsuleShape(world, rod, &sd, &capsule);
    const double across = sqrt(1.0 - 0.09);
    for (int32_t shape = 0; shape < 2; ++shape)
    {
        // The sphere's ray runs along (1, 2, 2) / 3; the capsule's across
        // its axis along (3, 4, 0) / 5. Each passes 0.3 m off the center
        // along a unit vector w across the ray.
        const double u[3] = {shape == 0 ? 1.0 / 3.0 : 0.6, shape == 0 ? 2.0 / 3.0 : 0.8,
                             shape == 0 ? 2.0 / 3.0 : 0.0};
        const double w[3] = {shape == 0 ? 2.0 / 3.0 : 0.8, shape == 0 ? -2.0 / 3.0 : -0.6,
                             shape == 0 ? 1.0 / 3.0 : 0.0};
        const double z = shape == 0 ? 0.0 : 10.0;
        double worstRatio = 0.0;
        bool hits = true;
        for (double d = 10.0; d <= 1.0e6; d *= 10.0)
        {
            m3Pos3 origin = {-d * u[0] + 0.3 * w[0], -d * u[1] + 0.3 * w[1],
                             z - d * u[2] + 0.3 * w[2]};
            m3Vec3 path = {(float)(2.0 * d * u[0]), (float)(2.0 * d * u[1]),
                           (float)(2.0 * d * u[2])};
            m3CastHit hit = {0};
            bool found = m3World_CastRayClosest(world, origin, path, m3DefaultQueryFilter(), &hit);
            hits = hits && found && isfinite(hit.fraction);
            const float n[3] = {hit.normal.x, hit.normal.y, hit.normal.z};
            double error = 0.0;
            for (int32_t k = 0; k < 3; ++k)
            {
                error = fmax(error, fabs((double)n[k] - (0.3 * w[k] - across * u[k])));
            }
            worstRatio = fmax(worstRatio, error / (d * (double)FLT_EPSILON));
        }
        printf("diagonal far rays, %s: worst normal error %.3g |m| FLT_EPSILON\n",
               shape == 0 ? "sphere" : "capsule", worstRatio);
        CHECK(hits, "a diagonal far ray hits with a finite fraction");
        CHECK(worstRatio <= 2.0, "a diagonal far ray keeps its normal within 2 |m| FLT_EPSILON");
    }
    m3DestroyWorld(world);
}

// The list calls' outputs: a NULL array with capacity 0 measures the
// total, an array shorter than the total still returns it, and a NULL array
// with a positive capacity or a negative capacity refuses with one misuse.
// One scene for all ten: three balls resting on a floor plane.
typedef struct ListScene
{
    m3World* world;
    m3BodyId floorBody;
    m3ShapeId floor;
    m3BodyId hub; // two shapes and two joints, away from the balls
} ListScene;

typedef int32_t ListCall(const ListScene* scene, void* out, int32_t capacity);

static int32_t ListAabb(const ListScene* sc, void* out, int32_t capacity)
{
    return m3World_OverlapAabb(sc->world, (m3WorldAabb){{-3.0, -0.5, -1.0}, {3.0, 1.5, 1.0}},
                               m3DefaultQueryFilter(), (m3ShapeId*)out, capacity);
}

static int32_t ListSphere(const ListScene* sc, void* out, int32_t capacity)
{
    return m3World_OverlapSphere(sc->world, (m3Pos3){0.0, 0.5, 0.0}, 2.0f, m3DefaultQueryFilter(),
                                 (m3ShapeId*)out, capacity);
}

static int32_t ListCapsule(const ListScene* sc, void* out, int32_t capacity)
{
    return m3World_OverlapCapsule(sc->world, (m3Pos3){-1.5, 0.5, 0.0}, (m3Pos3){1.5, 0.5, 0.0},
                                  0.6f, m3DefaultQueryFilter(), (m3ShapeId*)out, capacity);
}

static int32_t ListBox(const ListScene* sc, void* out, int32_t capacity)
{
    return m3World_OverlapBox(sc->world, (m3Pos3){0.0, 0.5, 0.0}, (m3Vec3){2.0f, 0.6f, 0.6f},
                              m3MakeIdentityQuat(), m3DefaultQueryFilter(), (m3ShapeId*)out,
                              capacity);
}

static int32_t ListHull(const ListScene* sc, void* out, int32_t capacity)
{
    static const m3Vec3 ends[2] = {{-1.5f, 0.0f, 0.0f}, {1.5f, 0.0f, 0.0f}};
    return m3World_OverlapHullPoints(sc->world, (m3Pos3){0.0, 0.5, 0.0}, ends, 2, 0.6f,
                                     m3DefaultQueryFilter(), (m3ShapeId*)out, capacity);
}

static int32_t ListRay(const ListScene* sc, void* out, int32_t capacity)
{
    return m3World_CastRayAll(sc->world, (m3Pos3){-3.0, 0.5, 0.0}, (m3Vec3){6.0f, 0.0f, 0.0f},
                              m3DefaultQueryFilter(), (m3CastHit*)out, capacity);
}

static int32_t ListMover(const ListScene* sc, void* out, int32_t capacity)
{
    return m3World_CollideMover(sc->world, (m3Pos3){0.0, 0.5, 1.0}, 0.2f, 0.3f, 1.0f,
                                (m3MoverPlane*)out, capacity);
}

static int32_t ListBodyContacts(const ListScene* sc, void* out, int32_t capacity)
{
    return m3Body_GetContactData(sc->world, sc->floorBody, (m3ContactData*)out, capacity);
}

static int32_t ListShapeContacts(const ListScene* sc, void* out, int32_t capacity)
{
    return m3Shape_GetContactData(sc->world, sc->floor, (m3ContactData*)out, capacity);
}

static int32_t ListBodies(const ListScene* sc, void* out, int32_t capacity)
{
    return m3World_GetBodies(sc->world, (m3BodyId*)out, capacity);
}

static int32_t ListJoints(const ListScene* sc, void* out, int32_t capacity)
{
    return m3World_GetJoints(sc->world, (m3JointId*)out, capacity);
}

static int32_t ListBodyShapes(const ListScene* sc, void* out, int32_t capacity)
{
    return m3Body_GetShapes(sc->world, sc->hub, (m3ShapeId*)out, capacity);
}

static int32_t ListBodyJoints(const ListScene* sc, void* out, int32_t capacity)
{
    return m3Body_GetJoints(sc->world, sc->hub, (m3JointId*)out, capacity);
}

static void TestListOutputs(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    ListScene sc;
    sc.world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    sc.floorBody = NewBody(sc.world, &bd);
    m3Plane plane = {{0.0f, 1.0f, 0.0f}, 0.0f};
    sc.floor = NewPlaneShape(sc.world, sc.floorBody, &sd, &plane);
    bd.type = m3_dynamicBody;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    for (int32_t i = 0; i < 3; ++i)
    {
        bd.position = (m3Pos3){(double)(i - 1), 0.5, 0.0};
        NewSphereShape(sc.world, NewBody(sc.world, &bd), &sd, &ball);
    }
    for (int32_t i = 0; i < 3; ++i)
    {
        (void)m3World_Step(sc.world, 1.0f / 60.0f, 4);
    }
    // Built after the steps, far from every query above.
    bd.type = m3_staticBody;
    bd.position = (m3Pos3){20.0, 20.0, 0.0};
    sc.hub = NewBody(sc.world, &bd);
    NewSphereShape(sc.world, sc.hub, &sd, &ball);
    NewSphereShape(sc.world, sc.hub, &sd, &ball);
    bd.type = m3_dynamicBody;
    for (int32_t i = 0; i < 2; ++i)
    {
        bd.position = (m3Pos3){22.0 + 2.0 * i, 20.0, 0.0};
        m3JointDef jd = m3DefaultJointDef();
        jd.bodyIdA = sc.hub;
        jd.bodyIdB = NewBody(sc.world, &bd);
        NewJoint(sc.world, &jd);
    }
    static const struct
    {
        ListCall* call;
        const char* name;
    } calls[] = {
        {ListAabb, "OverlapAabb"},
        {ListSphere, "OverlapSphere"},
        {ListCapsule, "OverlapCapsule"},
        {ListBox, "OverlapBox"},
        {ListHull, "OverlapHullPoints"},
        {ListRay, "CastRayAll"},
        {ListMover, "CollideMover"},
        {ListBodyContacts, "Body_GetContactData"},
        {ListShapeContacts, "Shape_GetContactData"},
        {ListBodies, "World_GetBodies"},
        {ListJoints, "World_GetJoints"},
        {ListBodyShapes, "Body_GetShapes"},
        {ListBodyJoints, "Body_GetJoints"},
    };
    // One entry of any call's type: each call stores into the member of
    // its own type, and the union's size and alignment cover them all.
    static union
    {
        m3ContactData contact;
        m3CastHit hit;
        m3MoverPlane plane;
        m3ShapeId shape;
        m3BodyId body;
        m3JointId joint;
    } one[1];
    for (size_t c = 0; c < sizeof(calls) / sizeof(calls[0]); ++c)
    {
        uint64_t misuse = m3World_GetCounters(sc.world).misuse;
        int32_t total = calls[c].call(&sc, NULL, 0);
        bool measured = total >= 2 && m3World_GetCounters(sc.world).misuse == misuse;
        int32_t shortTotal = calls[c].call(&sc, one, 1);
        bool counted = shortTotal == total && m3World_GetCounters(sc.world).misuse == misuse;
        int32_t refusedNull = calls[c].call(&sc, NULL, 3);
        bool nullRefused = refusedNull == 0 && m3World_GetCounters(sc.world).misuse == misuse + 1u;
        int32_t refusedNegative = calls[c].call(&sc, one, -1);
        bool negativeRefused =
            refusedNegative == 0 && m3World_GetCounters(sc.world).misuse == misuse + 2u;
        if (!measured || !counted || !nullRefused || !negativeRefused)
        {
            printf("  %s: total %d, short %d, NULL %d, negative %d\n", calls[c].name, (int)total,
                   (int)shortTotal, (int)refusedNull, (int)refusedNegative);
        }
        CHECK(measured, "a NULL array with capacity 0 measures the total, two or more here");
        CHECK(counted, "a one-entry array returns the same total");
        CHECK(nullRefused && negativeRefused,
              "a NULL array with a positive capacity, or a negative capacity, refuses");
    }
    m3DestroyWorld(sc.world);
}

int main(void)
{
    TestOverlapBeyondTwoHundredFiftySix();
    TestLongPathsMeetFarPieces();
    TestContactEvents();
    TestRayHitsEveryFamily();
    TestRayClosestOfMany();
    TestListOutputs();
    TestCastHitContract();
    TestShapeCasts();
    TestRayStartInsideContract();
    TestMultiHitSorted();
    TestPointAndOverlaps();
    TestCastsAgainstMeshAndPlane();
    TestOverlapAndInsideFamilies();
    TestFarPointsMissVoxels();
    TestGenericCasts();
    TestSensorPassThrough();
    TestSensorSleepAndBullets();
    TestFarRaysKeepTheirNormal();
    TestDiagonalFarRaysKeepTheirNormal();
    if (s_failures == 0)
    {
        printf("test_queries: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
