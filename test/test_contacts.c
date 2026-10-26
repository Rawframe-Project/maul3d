// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Narrowphase gate: the pure collide kernels against analytic cases,
// the speculative band, the fixed tangent basis rule, the warm-start
// carry through a full stash-scan-rebuild cycle, and the contact
// reduction's picks. White box.

#include "broad_phase.h"
#include "contact_kernel.h"
#include "helpers.h"
#include "manifold.h"
#include "narrowphase.h"
#include "solver_math.h"
#include "test_harness.h"
#include "world.h"
#include "world_internal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int NearF(m3real a, m3real b, m3real tol)
{
    m3real d = a - b;
    return (d < 0.0f ? -d : d) <= tol;
}

static void TestSphereKernels(void)
{
    // Two r=0.5 spheres, centers 0.9 apart along x: penetration 0.1.
    m3Manifold m = m3CollideSpheres((m3Vec3){0.9f, 0.0f, 0.0f}, 0.5f, 0.5f);
    CHECK(m.pointCount == 1, "overlapping spheres make one point");
    CHECK(NearF(m.normal.x, 1.0f, 1.0e-6f), "normal points from A to B");
    CHECK(NearF(m.points[0].separation, -0.1f, 1.0e-6f), "penetration is analytic");
    CHECK(NearF(m.points[0].anchorA.x, 0.5f, 1.0e-6f), "anchor A on A's surface");
    CHECK(NearF(m.points[0].anchorB.x, -0.5f, 1.0e-6f), "anchor B on B's surface");

    // The speculative band: a gap inside the margin still contacts.
    m = m3CollideSpheres((m3Vec3){1.01f, 0.0f, 0.0f}, 0.5f, 0.5f);
    CHECK(m.pointCount == 1 && m.points[0].separation > 0.0f, "speculative contact in the band");
    m = m3CollideSpheres((m3Vec3){1.10f, 0.0f, 0.0f}, 0.5f, 0.5f);
    CHECK(m.pointCount == 0, "beyond the margin there is no contact");

    // Concentric centers: the fixed +y fallback, never NaN.
    m = m3CollideSpheres((m3Vec3){0.0f, 0.0f, 0.0f}, 0.5f, 0.5f);
    CHECK(m.pointCount == 1 && m.normal.y == 1.0f, "concentric fallback is the fixed rule");

    // Plane versus sphere: center 0.45 above, r 0.5: penetration 0.05.
    m = m3CollidePlaneSphere((m3Vec3){0.0f, 1.0f, 0.0f}, 0.45f, 0.5f);
    CHECK(m.pointCount == 1, "plane contact");
    CHECK(NearF(m.points[0].separation, -0.05f, 1.0e-6f), "plane penetration is analytic");
    CHECK(m.normal.y == 1.0f, "plane normal is A to B");
}

static void TestTangentBasis(void)
{
    // Orthonormal for axis and skew normals, and bit-identical on
    // repeated calls (one rule, no state).
    m3Vec3 normals[5] = {{0.0f, 1.0f, 0.0f},
                         {1.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f},
                         {0.577350f, 0.577350f, 0.577350f},
                         {-0.267261f, 0.534522f, 0.801784f}};
    for (int32_t i = 0; i < 5; ++i)
    {
        m3Vec3 t1a;
        m3Vec3 t2a;
        m3Vec3 t1b;
        m3Vec3 t2b;
        m3MakeTangentBasis(normals[i], &t1a, &t2a);
        m3MakeTangentBasis(normals[i], &t1b, &t2b);
        CHECK(memcmp(&t1a, &t1b, sizeof(t1a)) == 0 && memcmp(&t2a, &t2b, sizeof(t2a)) == 0,
              "the basis is bit-stable");
        CHECK(NearF(m3Dot3(t1a, normals[i]), 0.0f, 1.0e-5f), "t1 orthogonal to n");
        CHECK(NearF(m3Dot3(t2a, normals[i]), 0.0f, 1.0e-5f), "t2 orthogonal to n");
        CHECK(NearF(m3Dot3(t1a, t2a), 0.0f, 1.0e-5f), "t1 orthogonal to t2");
        CHECK(NearF(m3Length3(t1a), 1.0f, 1.0e-5f), "t1 unit");
        CHECK(NearF(m3Length3(t2a), 1.0f, 1.0e-5f), "t2 unit");
    }
}

static void TestWarmStartCarry(void)
{
    // A sphere resting near a plane: build contacts, inject an
    // impulse, rebuild, and the impulse must persist by feature id;
    // separate the pair and rebuild, and it must vanish.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m3World* world = NewWorld(&def);
    m3World* w = world;

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.45, 0.0};
    m3BodyId ball = NewBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(world, ball, &sd, &sphere);

    CHECK(m3UpdatePairs(w) == m3_success, "pairs");
    CHECK(m3UpdateContacts(w, NULL, NULL, 0) == m3_success, "first contact build");
    CHECK(w->contacts.pairCount == 1 && w->contacts.manifolds[0].pointCount == 1,
          "one plane contact");
    CHECK((w->contacts.manifolds[0].points[0].flags & 1) == 0, "a fresh contact is not persisted");

    // Inject a solved impulse, then run the full stash-scan-rebuild
    // cycle the step will use (stash BEFORE the pair scan overwrites).
    w->contacts.manifolds[0].points[0].normalImpulse = 3.5f;
    uint64_t stashKeys[4];
    m3Manifold stashManifolds[4];
    int32_t stashCount = w->contacts.pairCount;
    memcpy(stashKeys, w->contacts.pairKeys, (size_t)stashCount * sizeof(uint64_t));
    memcpy(stashManifolds, w->contacts.manifolds, (size_t)stashCount * sizeof(m3Manifold));
    CHECK(m3UpdatePairs(w) == m3_success, "rescan");
    CHECK(m3UpdateContacts(w, stashKeys, stashManifolds, stashCount) == m3_success, "rebuild");
    CHECK(NearF(w->contacts.manifolds[0].points[0].normalImpulse, 3.5f, 1.0e-6f),
          "the warm-start impulse carries by feature id");
    CHECK((w->contacts.manifolds[0].points[0].flags & 1) == 1,
          "the carried point is marked persisted");

    // Separate the pair: the contact and its impulse vanish.
    m3Body_SetLinearVelocity(world, ball, (m3Vec3){0.0f, 0.0f, 0.0f});
    w->bodies.transforms[ball.index1 - 1].p.y = 10.0;
    stashCount = w->contacts.pairCount;
    memcpy(stashKeys, w->contacts.pairKeys, (size_t)stashCount * sizeof(uint64_t));
    memcpy(stashManifolds, w->contacts.manifolds, (size_t)stashCount * sizeof(m3Manifold));
    CHECK(m3UpdatePairs(w) == m3_success, "rescan after separation");
    CHECK(m3UpdateContacts(w, stashKeys, stashManifolds, stashCount) == m3_success,
          "rebuild after separation");
    int32_t contacts = 0;
    for (int32_t i = 0; i < w->contacts.pairCount; ++i)
    {
        contacts += w->contacts.manifolds[i].pointCount;
    }
    CHECK(contacts == 0, "a separated pair carries nothing");

    m3DestroyWorld(world);
}

// The solver's small solves fall back when a determinant leaves the
// normal floats or cancels: to another scale or to double. Every system here is ordinary: its
// direct solve is exact to rounding, each fallback must give the same solution, and the lane solve
// must give the scalar one bit for bit.
static uint32_t s_seed = 0x5CA1Eu;

static float Uniform(float lo, float hi)
{
    s_seed = s_seed * 1664525u + 1013904223u;
    return lo + (hi - lo) * (float)(s_seed >> 8) / 16777216.0f;
}

// A scale between 2^-20 and 2^21, not a power of two.
static float AnyScale(void)
{
    float scale = Uniform(1.0f, 2.0f);
    int32_t k = (int32_t)Uniform(-20.0f, 20.0f);
    for (; k > 0; --k)
    {
        scale *= 2.0f;
    }
    for (; k < 0; ++k)
    {
        scale *= 0.5f;
    }
    return scale;
}

static float Gap(m3Vec3 a, m3Vec3 b)
{
    m3Vec3 d = m3Sub3(a, b);
    return sqrtf(m3Dot3(d, d) / m3Dot3(a, a));
}

static void TestFallbackSolves(void)
{
    float worstSolve = 0.0f;
    float worstWide = 0.0f;
    float worstPair = 0.0f;
    float worstPairWide = 0.0f;
    int32_t lanesDiffer = 0;
    enum
    {
        CASES = 4000
    };
    float k[9][M3_LANES];
    float b[3][M3_LANES];
    float scale[M3_LANES];
    for (int32_t i = 0; i < CASES; ++i)
    {
        // A general, diagonally dominant 3x3 and a right-hand side.
        m3Mat3 J;
        float* entries[9] = {&J.cx.x, &J.cx.y, &J.cx.z, &J.cy.x, &J.cy.y,
                             &J.cy.z, &J.cz.x, &J.cz.y, &J.cz.z};
        for (int32_t e = 0; e < 9; ++e)
        {
            *entries[e] = Uniform(-1.0f, 1.0f) + (e % 4 == 0 ? 3.0f : 0.0f);
        }
        m3Vec3 rhs = {Uniform(-1.0f, 1.0f), Uniform(-1.0f, 1.0f), Uniform(-1.0f, 1.0f)};
        float s = AnyScale();
        m3Vec3 direct = m3Solve3(&J, rhs);
        m3Vec3 scaled = m3Solve3AtScale(&J, rhs, s);
        worstSolve = fmaxf(worstSolve, Gap(direct, scaled));
        double wide[9];
        for (int32_t e = 0; e < 9; ++e)
        {
            wide[e] = (double)*entries[e];
        }
        worstWide = fmaxf(worstWide, Gap(direct, m3Solve3Wide(wide, rhs)));

        int32_t lane = i % M3_LANES;
        for (int32_t e = 0; e < 9; ++e)
        {
            k[e][lane] = *entries[e];
        }
        b[0][lane] = rhs.x;
        b[1][lane] = rhs.y;
        b[2][lane] = rhs.z;
        scale[lane] = s;
        if (lane == M3_LANES - 1)
        {
            float out[3][M3_LANES];
            float outScaled[3][M3_LANES];
            m3SolveLanes3(k, b, out);
            m3SolveLanes3AtScale(k, b, scale, outScaled);
            for (int32_t l = 0; l < M3_LANES; ++l)
            {
                m3Mat3 Jl = {{k[0][l], k[1][l], k[2][l]},
                             {k[3][l], k[4][l], k[5][l]},
                             {k[6][l], k[7][l], k[8][l]}};
                m3Vec3 bl = {b[0][l], b[1][l], b[2][l]};
                m3Vec3 x = m3Solve3(&Jl, bl);
                m3Vec3 xs = m3Solve3AtScale(&Jl, bl, scale[l]);
                float lanes[6] = {out[0][l],       out[1][l],       out[2][l],
                                  outScaled[0][l], outScaled[1][l], outScaled[2][l]};
                float scalar[6] = {x.x, x.y, x.z, xs.x, xs.y, xs.z};
                lanesDiffer += memcmp(lanes, scalar, sizeof(lanes)) != 0;
            }
        }

        // A symmetric positive definite pair and its direct inverse.
        float k11 = Uniform(0.5f, 4.0f);
        float k22 = Uniform(0.5f, 4.0f);
        float k12 = Uniform(-0.9f, 0.9f) * sqrtf(k11 * k22);
        float det = k11 * k22 - k12 * k12;
        m3Vec3 inverse = {k22 / det, k11 / det, -k12 / det};
        float r[3] = {0.0f, 0.0f, 0.0f};
        CHECK(m3InvertPairAtScale(k11, k22, k12, s, r), "an ordinary pair inverts at any scale");
        worstPair = fmaxf(worstPair, Gap(inverse, (m3Vec3){r[0], r[1], r[2]}));
        float px = Uniform(-1.0f, 1.0f);
        float py = Uniform(-1.0f, 1.0f);
        m3Vec3 pair = {(k22 * px - k12 * py) / det, (k11 * py - k12 * px) / det, 0.0f};
        float sol[2] = {0.0f, 0.0f};
        CHECK(m3SolvePairWide((double)k11, (double)k22, (double)k12, px, py, sol),
              "an ordinary pair solves in double");
        worstPairWide = fmaxf(worstPairWide, Gap(pair, (m3Vec3){sol[0], sol[1], 0.0f}));
    }
    CHECK(worstSolve < 1.0e-5f, "a 3x3 solve at another scale matches the direct one");
    CHECK(worstWide < 1.0e-5f, "a 3x3 solve in double matches the direct one");
    CHECK(worstPair < 1.0e-5f, "a pair inverse at another scale matches the direct one");
    CHECK(worstPairWide < 1.0e-5f, "a pair solve in double matches the direct one");
    CHECK(lanesDiffer == 0, "the lane solves match the scalar ones bit for bit");
    printf("fallback solves: worst 3x3 at scale %.3g, in double %.3g; worst pair at scale %.3g, "
           "in double %.3g; over %d cases\n",
           (double)worstSolve, (double)worstWide, (double)worstPair, (double)worstPairWide, CASES);
}

// Checks one reduction of count points: between one and four picks,
// ascending, each in [0, count), the deepest among them. Returns how many.
static int32_t CheckPicks(const m3Vec3* points, const m3real* separations, int32_t count,
                          m3Vec3 normal, int32_t out[M3_MANIFOLD_MAX_POINTS])
{
    int32_t n = m3ReduceContactPoints(points, separations, count, normal, out);
    int32_t deepest = 0;
    for (int32_t i = 1; i < count; ++i)
    {
        deepest = separations[i] < separations[deepest] ? i : deepest;
    }
    int ok = n >= 1 && n <= M3_MANIFOLD_MAX_POINTS;
    int hasDeepest = 0;
    for (int32_t k = 0; ok && k < n; ++k)
    {
        ok = out[k] >= 0 && out[k] < count && (k == 0 || out[k] > out[k - 1]);
        hasDeepest |= out[k] == deepest;
    }
    CHECK(ok && hasDeepest,
          "the reduction picks ascending points of the patch, the deepest among them");
    return n;
}

static void TestReducePicks(void)
{
    enum
    {
        MAX_COUNT = 32,
        CASES = 2000
    };
    m3Vec3 points[MAX_COUNT];
    m3real separations[MAX_COUNT];
    int32_t out[M3_MANIFOLD_MAX_POINTS];

    // A 2 x 2 square's corners and edge midpoints, deepest at a corner:
    // the four corners span it.
    const m3Vec3 ring[8] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f},
                            {2.0f, 1.0f, 0.0f}, {2.0f, 2.0f, 0.0f}, {1.0f, 2.0f, 0.0f},
                            {0.0f, 2.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
    for (int32_t i = 0; i < 8; ++i)
    {
        points[i] = ring[i];
        separations[i] = i == 0 ? -0.1f : 0.0f;
    }
    m3Vec3 up = {0.0f, 0.0f, 1.0f};
    int32_t n = CheckPicks(points, separations, 8, up, out);
    CHECK(n == 4 && out[0] == 0 && out[1] == 2 && out[2] == 4 && out[3] == 6,
          "a square's ring reduces to its corners");

    // Patches that span less than a quad: points on one spot, and points
    // on one line.
    for (int32_t i = 0; i < 8; ++i)
    {
        points[i] = (m3Vec3){1.0f, -2.0f, 0.5f};
        separations[i] = -0.01f * (m3real)(i % 3);
    }
    CheckPicks(points, separations, 8, up, out);
    for (int32_t i = 0; i < 8; ++i)
    {
        points[i] = (m3Vec3){0.25f * (m3real)i, 0.0f, 0.0f};
        separations[i] = -0.01f * (m3real)i;
    }
    n = CheckPicks(points, separations, 8, up, out);
    CHECK(n <= 3, "points on one line reduce to at most three");

    // Random patches of five to 32 points around random normals, with
    // repeated points and repeated depths mixed in.
    for (int32_t c = 0; c < CASES; ++c)
    {
        int32_t count = 5 + (int32_t)Uniform(0.0f, (float)(MAX_COUNT - 4));
        count = count > MAX_COUNT ? MAX_COUNT : count;
        m3Vec3 normal = {Uniform(-1.0f, 1.0f), Uniform(-1.0f, 1.0f), Uniform(0.1f, 1.0f)};
        normal = m3Normalize3(normal);
        float size = AnyScale();
        for (int32_t i = 0; i < count; ++i)
        {
            if (i > 0 && Uniform(0.0f, 1.0f) < 0.2f)
            {
                int32_t from = (int32_t)Uniform(0.0f, (float)i);
                points[i] = points[from < i ? from : i - 1];
                separations[i] = separations[from < i ? from : i - 1];
                continue;
            }
            points[i] = (m3Vec3){size * Uniform(-1.0f, 1.0f), size * Uniform(-1.0f, 1.0f),
                                 size * Uniform(-1.0f, 1.0f)};
            separations[i] = Uniform(-0.1f, 0.0f);
        }
        CheckPicks(points, separations, count, normal, out);
    }
}

// Points that are not finite fail every comparison the reduction makes;
// each pick must still index the patch.
static void TestReducePicksNotFinite(void)
{
    m3Vec3 points[8];
    m3real separations[8];
    int32_t out[M3_MANIFOLD_MAX_POINTS];
    m3Vec3 up = {0.0f, 0.0f, 1.0f};
    for (int32_t i = 0; i < 8; ++i)
    {
        points[i] = (m3Vec3){NAN, NAN, NAN};
        separations[i] = -0.01f * (m3real)i;
    }
    CheckPicks(points, separations, 8, up, out);
    // Only the deepest point is not finite.
    for (int32_t i = 0; i < 8; ++i)
    {
        points[i] = (m3Vec3){(m3real)i, (m3real)(i % 3), 0.0f};
    }
    points[7] = (m3Vec3){NAN, 0.0f, 0.0f};
    CheckPicks(points, separations, 8, up, out);
}

int main(void)
{
    TestSphereKernels();
    TestTangentBasis();
    TestWarmStartCarry();
    TestFallbackSolves();
    TestReducePicks();
    TestReducePicksNotFinite();
    if (s_failures == 0)
    {
        printf("test_contacts: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
