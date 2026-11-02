// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The contact lever ratio. A body held by a joint and pressed onto the
// ground by a contact far from its center of mass is held by two
// constraints that turn it against each other: each impulse the contact
// gives, times its arm, the joint must undo. The contact rows are solved
// in float lanes, one constraint after another, and past a ratio of mass
// times squared arm over moment near 1e7 they no longer settle, so the
// contact lets the body sink through its ground. Every call that can carry
// a dynamic body past M3_CONTACT_LEVER_LIMIT refuses with m3_errorLimit,
// counts no misuse and changes nothing; restore and replay refuse such
// bytes.

#include "helpers.h"
#include "journal.h"
#include "state_prefix.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A core of radius 1 and density 1e6 has 0.4 times its mass in moment, so a
// colliding shell of radius 1 whose center lies d from it has a ratio of
// about (d + 1)^2 / 0.4: 631 m stays just inside 1e6, and 9000 m lies far
// past it.
static const float s_near = 630.0f;
static const float s_far = 9000.0f;

static uint64_t Misuse(const m3World* world)
{
    return m3World_GetCounters(world).misuseCount;
}

static m3World* Yard(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    return NewWorld(&wd);
}

// A body at (0, 1, 0) of this type carrying the dense core, which collides
// with nothing.
static m3BodyId CoreBody(m3World* world, m3BodyType type)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = type;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId body = NewBody(world, &bd);
    m3ShapeDef core = m3DefaultShapeDef();
    core.density = 1.0e6f;
    core.categoryBits = 0;
    core.maskBits = 0;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 1.0f};
    CHECK(m3Shape_IsValid(world, NewSphereShape(world, body, &core, &ball)), "the core creates");
    return body;
}

// The 1 microgram shell of radius 1, its center at distance ahead and one
// meter up in the body's frame, colliding as mask says.
static m3Result AddShell(m3World* world, m3BodyId body, float distance, uint64_t mask,
                         m3ShapeId* shellOut)
{
    m3ShapeDef shell = m3DefaultShapeDef();
    shell.density = 1.01e-6f / 4.18879f;
    shell.maskBits = mask;
    m3Sphere ball = {{distance, 1.0f, 0.0f}, 1.0f};
    return m3CreateSphereShape(world, body, &shell, &ball, shellOut);
}

// The issue's scene: the shell's create is refused, and the world is as it
// was.
static void TestFarShellRefused(void)
{
    m3World* world = Yard();
    m3BodyId body = CoreBody(world, m3_dynamicBody);
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = Misuse(world);
    m3ShapeId shell = {7u, 7u};
    CHECK(AddShell(world, body, s_far, ~0ull, &shell) == m3_errorLimit && shell.index1 == 0,
          "a colliding shell 9000 m out is refused as limit");
    CHECK(m3World_Hash(world) == hash && Misuse(world) == misuse,
          "with the world left as it was and no misuse counted");
    CHECK(AddShell(world, body, s_near, ~0ull, &shell) == m3_success,
          "the same shell 630 m out creates");
    m3DestroyWorld(world);
}

// The far-contact scene inside the bound: the core hangs from a socket 1 m
// behind it and the shell rests on a small box. The socket's rest sag,
// under a millimeter, times the shell's lever lets the shell settle about
// 0.1 m into the box within two seconds; from there it holds still for
// eight more, where a shell past the bound sinks 0.4 m every step and falls
// through the box.
static void TestNearShellHolds(void)
{
    m3World* world = Yard();
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &bd);
    bd.position = (m3Pos3){(double)s_near, 0.0, 0.0};
    m3BodyId slab = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(world, slab, &sd, (m3Vec3){2.0f, 1.0f, 2.0f});
    m3BodyId body = CoreBody(world, m3_dynamicBody);
    m3ShapeId shell;
    CHECK(AddShell(world, body, s_near, ~0ull, &shell) == m3_success, "the shell creates");
    m3JointDef jd = m3DefaultJointDef();
    jd.bodyIdA = ground;
    jd.bodyIdB = body;
    jd.localAnchorA = (m3Vec3){-1.0f, 1.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-1.0f, 0.0f, 0.0f};
    jd.enableConnectedCollision = true;
    CHECK(m3Joint_IsValid(world, NewJoint(world, &jd)), "the socket holds the core");
    m3Vec3 bottom = {s_near, 0.0f, 0.0f};
    double settled = 0.0;
    double lowestCore = 1.0;
    double drift = 0.0;
    for (int32_t i = 0; i < 600; ++i)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        double shellY = m3Body_GetWorldPoint(world, body, bottom).y;
        lowestCore = fmin(lowestCore, m3Body_GetPosition(world, body).y);
        if (i == 119)
        {
            settled = shellY;
        }
        if (i >= 120)
        {
            drift = fmax(drift, fabs(shellY - settled));
        }
    }
    CHECK(lowestCore > 0.999, "the core does not sink");
    CHECK(settled > 0.8, "the shell settles on the box, well above its middle");
    CHECK(drift < 1.0e-3, "and then holds still");
    m3DestroyWorld(world);
}

// A body whose light ballast, a ball of radius 100 that collides with
// nothing, gives the moment that keeps a far colliding shell inside the
// bound.
static m3BodyId BallastBody(m3World* world, m3ShapeId* ballastOut, m3ShapeId* shellOut)
{
    m3BodyId body = CoreBody(world, m3_dynamicBody);
    m3ShapeDef ballast = m3DefaultShapeDef();
    ballast.density = 10.0f;
    ballast.categoryBits = 0;
    ballast.maskBits = 0;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 100.0f};
    *ballastOut = NewSphereShape(world, body, &ballast, &ball);
    CHECK(m3Shape_IsValid(world, *ballastOut), "the ballast creates");
    CHECK(AddShell(world, body, 2000.0f, ~0ull, shellOut) == m3_success,
          "with the ballast, a shell 2000 m out creates");
    return body;
}

// Each change that would carry the body past the bound is refused as limit,
// counts no misuse and changes nothing: destroying or lightening the
// ballast, moving the shell farther, and letting a far shell collide.
// Without the ballast the ratio is near 1e7; with a ballast of density
// 1e-4, near 5e6.
static void TestChangesKeepTheRatio(void)
{
    m3World* world = Yard();
    m3ShapeId ballast;
    m3ShapeId shell;
    (void)BallastBody(world, &ballast, &shell);
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = Misuse(world);
    CHECK(m3DestroyShape(world, ballast) == m3_errorLimit, "destroying the ballast is refused");
    CHECK(m3Shape_SetDensity(world, ballast, 1.0e-4f) == m3_errorLimit,
          "lightening the ballast is refused");
    CHECK(m3World_Hash(world) == hash && Misuse(world) == misuse,
          "each leaves the world as it was and counts no misuse");
    CHECK(m3Shape_SetDensity(world, ballast, 20.0f) == m3_success, "a heavier ballast is taken");
    m3DestroyWorld(world);

    world = Yard();
    m3BodyId body = CoreBody(world, m3_dynamicBody);
    CHECK(AddShell(world, body, s_near, ~0ull, &shell) == m3_success, "the near shell creates");
    hash = m3World_Hash(world);
    misuse = Misuse(world);
    m3Sphere farther = {{s_far, 1.0f, 0.0f}, 1.0f};
    CHECK(m3Shape_SetSphere(world, shell, &farther) == m3_errorLimit,
          "moving the shell 9000 m out is refused");
    CHECK(m3World_Hash(world) == hash && Misuse(world) == misuse,
          "the world is as it was and no misuse counted");
    m3DestroyWorld(world);

    world = Yard();
    body = CoreBody(world, m3_dynamicBody);
    CHECK(AddShell(world, body, s_far, 0, &shell) == m3_success,
          "a far shell that collides with nothing creates");
    hash = m3World_Hash(world);
    misuse = Misuse(world);
    CHECK(m3Shape_SetFilter(world, shell, 1u, ~0ull, 0) == m3_errorLimit,
          "letting it collide is refused");
    CHECK(m3Shape_SetFilter(world, shell, 0, 0, 1) == m3_errorLimit, "and so is a positive group");
    uint64_t category = 7u;
    uint64_t mask = 7u;
    int32_t group = 7;
    m3Shape_GetFilter(world, shell, &category, &mask, &group);
    CHECK(category == 1u && mask == 0 && group == 0, "the shell keeps its filter");
    CHECK(m3World_Hash(world) == hash && Misuse(world) == misuse,
          "the world is as it was and no misuse counted");
    CHECK(m3Shape_SetFilter(world, shell, 2u, 0, -1) == m3_success,
          "a filter under which it still collides with nothing is taken");
    m3DestroyWorld(world);
}

// A static body may carry the far shell; it turns dynamic only within the
// bound.
static void TestTypeChangeKeepsTheRatio(void)
{
    m3World* world = Yard();
    m3BodyId body = CoreBody(world, m3_staticBody);
    m3ShapeId shell;
    CHECK(AddShell(world, body, s_far, ~0ull, &shell) == m3_success,
          "a static body takes the far shell");
    uint64_t hash = m3World_Hash(world);
    uint64_t misuse = Misuse(world);
    CHECK(m3Body_SetType(world, body, m3_dynamicBody) == m3_errorLimit,
          "turning it dynamic is refused");
    CHECK(m3Body_GetType(world, body) == m3_staticBody && m3World_Hash(world) == hash &&
              Misuse(world) == misuse,
          "the body stays static and no misuse is counted");
    CHECK(m3Body_SetType(world, body, m3_kinematicBody) == m3_success,
          "a kinematic body, which no contact turns, is taken");
    m3DestroyWorld(world);
}

// A snapshot of the far shell that collides with nothing restores; the
// same bytes with its mask set are refused.
static void TestRestoreKeepsTheRatio(void)
{
    m3World* world = Yard();
    m3BodyId body = CoreBody(world, m3_dynamicBody);
    m3ShapeId shell;
    CHECK(AddShell(world, body, s_far, 0, &shell) == m3_success, "the silent far shell creates");
    int64_t size = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "the world snapshots");
    if (snap != NULL)
    {
        uint64_t hash = m3World_Hash(world);
        uint64_t misuse = Misuse(world);
        CHECK(Restore(world, snap, size) && m3World_Hash(world) == hash, "and restores");
        int64_t at = m3StateFieldAt(world, offsetof(m3World, shapes.shapeMask));
        CHECK(at >= 0, "the mask is snapshot state");
        const uint64_t all = ~0ull;
        memcpy(snap + M3_SNAPSHOT_HEADER_BYTES + (at < 0 ? 0 : at) +
                   sizeof(uint64_t) * (size_t)(shell.index1 - 1),
               &all, sizeof(all));
        CHECK(m3World_Restore(world, snap, size) == m3_errorFormat,
              "a colliding far shell in the bytes is refused");
        CHECK(m3World_Hash(world) == hash && Misuse(world) == misuse,
              "with the world left as it was and no misuse counted");
    }
    free(snap);
    m3DestroyWorld(world);
}

// The tape that builds the silent far shell replays; the same tape with
// the shell's mask set in its create is refused.
static void TestReplayKeepsTheRatio(void)
{
    enum
    {
        TAPE_BYTES = 1 << 16
    };
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    m3World* world = Yard();
    CHECK(tape != NULL && StartJournal(world, tape, TAPE_BYTES), "the journal starts");
    if (tape == NULL)
    {
        m3DestroyWorld(world);
        return;
    }
    m3BodyId body = CoreBody(world, m3_dynamicBody);
    m3ShapeId shell;
    CHECK(AddShell(world, body, s_far, 0, &shell) == m3_success, "the silent far shell creates");
    int64_t bytes = StopJournal(world);
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);

    m3World* again = Yard();
    CHECK(ReplayJournal(again, tape, bytes) && m3World_Hash(again) == hash, "the tape replays");
    m3DestroyWorld(again);

    // The last shape create on the tape is the shell's.
    int32_t create = -1;
    for (int32_t at = M3_JOURNAL_HEADER_BYTES; at + 8 <= bytes;)
    {
        int32_t op;
        int32_t size;
        memcpy(&op, tape + at, sizeof(op));
        memcpy(&size, tape + at + 4, sizeof(size));
        create = op == (int32_t)m3_opCreateShape ? at + 8 : create;
        at += 8 + size;
    }
    CHECK(create >= 0, "the shell's create is on the tape");
    if (create >= 0)
    {
        const uint64_t all = ~0ull;
        memcpy(tape + create + offsetof(m3CreateShapeOp, def) + offsetof(m3ShapeDef, maskBits),
               &all, sizeof(all));
        m3World* edited = Yard();
        CHECK(m3World_ReplayJournal(edited, tape, bytes) == m3_errorFormat,
              "the tape with a colliding far shell is refused");
        m3DestroyWorld(edited);
    }
    free(tape);
}

int main(void)
{
    TestFarShellRefused();
    TestNearShellHolds();
    TestChangesKeepTheRatio();
    TestTypeChangeKeepsTheRatio();
    TestRestoreKeepsTheRatio();
    TestReplayKeepsTheRatio();
    if (s_failures == 0)
    {
        printf("test_contact_lever: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
