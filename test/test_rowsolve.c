// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint row solves at the edges of float: one pass of the point block,
// of a coupled row pair and of a row block takes the impulse the exact
// system asks for, for a body whose mass rounds away in the float entries
// (65 kg with a moment of 1.6 kg m^2 on a 1000 m arm off the axes, mass *
// arm^2 / inertia 4e7) and for one so heavy that the float determinant
// underflows. The exact impulse comes from the systems' closed forms in
// double, where nothing cancels. A row block with a dependent row falls
// back to its rows one at a time. White box.

#include "joint_row.h"
#include "test_harness.h"

#include <math.h>
#include <stdio.h>

// A dynamic body B on a static A: inverse mass invMass, an isotropic
// inverse inertia invInertia, and a moving start.
static m3JointBodies Bodies(m3real invMass, m3real invInertia)
{
    m3JointBodies b = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {1.0f, -2.0f, 3.0f},
        {1.0e-3f, -2.0e-3f, 3.0e-3f},
        0.0f,
        invMass,
        m3MakeZeroMat3(),
        m3MakeZeroMat3(),
    };
    b.iB.cx.x = invInertia;
    b.iB.cy.y = invInertia;
    b.iB.cz.z = invInertia;
    return b;
}

typedef struct Case
{
    const char* name;
    m3real invMass;
    m3real invInertia;
    m3real arm;
    m3real degrees;
} Case;

// How far a float impulse lies from the exact one, relative to it.
static double Miss(const double* got, const double* exact, int32_t n)
{
    double gap = 0.0;
    double size = 0.0;
    for (int32_t k = 0; k < n; ++k)
    {
        gap += (got[k] - exact[k]) * (got[k] - exact[k]);
        size += exact[k] * exact[k];
    }
    return sqrt(gap / size);
}

static void CheckCase(const Case* c)
{
    char what[128];
    m3real angle = c->degrees * M3_PI / 180.0f;
    m3Vec3 armB = {c->arm * cosf(angle), -c->arm * sinf(angle), 0.0f};
    m3Vec3 none = {0.0f, 0.0f, 0.0f};
    double m = (double)c->invMass;
    double i = (double)c->invInertia;
    double a[3] = {(double)armB.x, (double)armB.y, (double)armB.z};
    double a2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];

    // The point block, m I + i (|a|^2 I - a a^T), has the inverse
    // a a^T / (m |a|^2) + (I - a a^T / |a|^2) / (m + i |a|^2).
    m3JointBodies b = Bodies(c->invMass, c->invInertia);
    m3Vec3 v = m3Add3(b.vB, m3Cross3(b.wB, armB));
    double r[3] = {(double)v.x, (double)v.y, (double)v.z};
    double along = (a[0] * r[0] + a[1] * r[1] + a[2] * r[2]) / a2;
    double point[3];
    for (int32_t k = 0; k < 3; ++k)
    {
        point[k] = -(along * a[k] / m + (r[k] - along * a[k]) / (m + i * a2));
    }
    m3Vec3 accumulated = none;
    m3SolvePointBlock(none, armB, &b, none, m3RigidDrive(0.0f), &accumulated, M3_ROW_FREE);
    double got[3] = {(double)accumulated.x, (double)accumulated.y, (double)accumulated.z};
    snprintf(what, sizeof(what), "%s: the point block takes the exact impulse", c->name);
    CHECK(Miss(got, point, 3) < 1.0e-4, what);

    // A slider's two held rows across its axis, z. Their turn parts are
    // a x x = -a_y z and a x y = a_x z, so the pair is m I + i q q^T with
    // q = (-a_y, a_x), whose inverse is (I - q q^T / |q|^2) / m +
    // q q^T / (|q|^2 (m + i |q|^2)).
    b = Bodies(c->invMass, c->invInertia);
    m3JointRow rows[2] = {m3LineRow(none, armB, (m3Vec3){1.0f, 0.0f, 0.0f}),
                          m3LineRow(none, armB, (m3Vec3){0.0f, 1.0f, 0.0f})};
    double speed[2] = {(double)m3RowSpeed(&rows[0], &b), (double)m3RowSpeed(&rows[1], &b)};
    double q[2] = {-a[1], a[0]};
    double q2 = q[0] * q[0] + q[1] * q[1];
    double across = (q[0] * speed[0] + q[1] * speed[1]) / q2;
    double pair[2];
    for (int32_t k = 0; k < 2; ++k)
    {
        pair[k] = -((speed[k] - across * q[k]) / m + across * q[k] / (m + i * q2));
    }
    const m3real bias[2] = {0.0f, 0.0f};
    m3real first = 0.0f;
    m3real second = 0.0f;
    m3SolveRowPair(rows, &b, bias, m3RigidDrive(0.0f), &first, &second);
    double gotPair[2] = {(double)first, (double)second};
    snprintf(what, sizeof(what), "%s: the row pair takes the exact impulse", c->name);
    CHECK(Miss(gotPair, pair, 2) < 1.0e-4, what);

    // The same point pin as three rows of one block.
    b = Bodies(c->invMass, c->invInertia);
    const m3JointRow pin[3] = {m3LineRow(none, armB, (m3Vec3){1.0f, 0.0f, 0.0f}),
                               m3LineRow(none, armB, (m3Vec3){0.0f, 1.0f, 0.0f}),
                               m3LineRow(none, armB, (m3Vec3){0.0f, 0.0f, 1.0f})};
    const m3real still[3] = {0.0f, 0.0f, 0.0f};
    m3real slots[3] = {0.0f, 0.0f, 0.0f};
    m3real* const into[3] = {&slots[0], &slots[1], &slots[2]};
    m3SolveRowBlock(pin, 3, &b, still, m3RigidDrive(0.0f), into);
    double gotBlock[3] = {(double)slots[0], (double)slots[1], (double)slots[2]};
    snprintf(what, sizeof(what), "%s: the block takes the exact impulse", c->name);
    CHECK(Miss(gotBlock, point, 3) < 1.0e-4, what);
}

static void TestRowSolvesExact(void)
{
    // 65.45 kg and 1.636 kg m^2: a ball of radius 0.25 and density 1000;
    // on a 1 m arm nothing cancels and the float solve stands. 8e18 kg
    // and 5.3e26 kg m^2: a cube of half extent 1e4 and density 1e6, whose
    // float determinants underflow.
    const Case cases[] = {
        {"light, 1000 m at 30 degrees", 1.0f / 65.45f, 1.0f / 1.636f, 1000.0f, 30.0f},
        {"light, 1000 m at 45 degrees", 1.0f / 65.45f, 1.0f / 1.636f, 1000.0f, 45.0f},
        {"light, 1000 m at 60 degrees", 1.0f / 65.45f, 1.0f / 1.636f, 1000.0f, 60.0f},
        {"light, 1 m at 30 degrees", 1.0f / 65.45f, 1.0f / 1.636f, 1.0f, 30.0f},
        {"heavy, 1e4 m at 30 degrees", 1.25e-19f, 1.875e-27f, 1.0e4f, 30.0f},
    };
    for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); ++k)
    {
        CheckCase(&cases[k]);
    }
}

// A block with a dependent row has no unique impulse. It takes the rows
// one at a time, as the solver did before blocks, and lands where they
// do.
static void TestSingularBlockFallsBack(void)
{
    m3Vec3 none = {0.0f, 0.0f, 0.0f};
    m3Vec3 armB = {0.5f, -0.25f, 0.0f};
    const m3JointRow rows[3] = {m3LineRow(none, armB, (m3Vec3){1.0f, 0.0f, 0.0f}),
                                m3LineRow(none, armB, (m3Vec3){0.0f, 1.0f, 0.0f}),
                                m3LineRow(none, armB, (m3Vec3){1.0f, 0.0f, 0.0f})};
    const m3real bias[3] = {0.1f, -0.2f, 0.3f};
    m3RowDrive drive = {0.0f, 1.0f, 0.0f};
    m3JointBodies block = Bodies(1.0f / 65.45f, 1.0f / 1.636f);
    m3real slots[3] = {0.0f, 0.0f, 0.0f};
    m3real* const into[3] = {&slots[0], &slots[1], &slots[2]};
    m3SolveRowBlock(rows, 3, &block, bias, drive, into);
    m3JointBodies one = Bodies(1.0f / 65.45f, 1.0f / 1.636f);
    m3real each[3] = {0.0f, 0.0f, 0.0f};
    for (int32_t k = 0; k < 3; ++k)
    {
        m3RowDrive own = {bias[k], 1.0f, 0.0f};
        (void)m3SolveRow(&rows[k], &one, own, &each[k], -M3_ROW_FREE, M3_ROW_FREE);
    }
    CHECK(slots[0] == each[0] && slots[1] == each[1] && slots[2] == each[2] &&
              block.vB.x == one.vB.x && block.vB.y == one.vB.y && block.vB.z == one.vB.z &&
              block.wB.z == one.wB.z,
          "a block with a dependent row takes its rows one at a time");
}

int main(void)
{
    TestRowSolvesExact();
    TestSingularBlockFallsBack();
    if (s_failures == 0)
    {
        printf("test_rowsolve: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
