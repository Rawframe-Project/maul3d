// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint lever ratio. A joint's point block is K = (wA + wB) E +
// [rA]^T IA [rA] + [rB]^T IB [rB], with w a body's inverse mass, I its
// inverse inertia and r the arm from its center to the anchor. Along each
// arm only the inverse masses remain, so the block loses its bodies' mass
// once the turn terms dwarf them. The ratio bounds that: each body's
// largest turn, summed, over the summed inverse masses. Both are taken in
// each body's own frame, so the bound holds at every relative orientation
// and is at most twice the block's own. Create, every change of a jointed
// body's mass or type, restore and replay keep every joint within it.

#include "joint_lever.h"

#include <math.h>

bool m3JointHasBlocks(uint8_t type)
{
    return type == (uint8_t)m3_sphericalJoint || type == (uint8_t)m3_revoluteJoint ||
           type == (uint8_t)m3_prismaticJoint || type == (uint8_t)m3_fixedJoint ||
           type == (uint8_t)m3_wheelJoint || type == (uint8_t)m3_motorJoint;
}

// M = [r]^T J [r] has rank 2 at most, r in its null space, so its
// eigenvalues are 0 and the roots of l^2 - trace l + minors, where minors
// sums its principal 2x2 minors. In double; sqrt rounds alike on every
// platform.
double m3LeverTurn(const m3Mat3* invInertia, m3Vec3 arm)
{
    const double j[3][3] = {
        {(double)invInertia->cx.x, (double)invInertia->cy.x, (double)invInertia->cz.x},
        {(double)invInertia->cx.y, (double)invInertia->cy.y, (double)invInertia->cz.y},
        {(double)invInertia->cx.z, (double)invInertia->cy.z, (double)invInertia->cz.z}};
    const double r[3][3] = {{0.0, -(double)arm.z, (double)arm.y},
                            {(double)arm.z, 0.0, -(double)arm.x},
                            {-(double)arm.y, (double)arm.x, 0.0}};
    double jr[3][3];
    double m[3][3];
    for (int32_t a = 0; a < 3; ++a)
    {
        for (int32_t c = 0; c < 3; ++c)
        {
            jr[a][c] = j[a][0] * r[0][c] + j[a][1] * r[1][c] + j[a][2] * r[2][c];
        }
    }
    for (int32_t a = 0; a < 3; ++a)
    {
        for (int32_t c = 0; c < 3; ++c)
        {
            m[a][c] = r[0][a] * jr[0][c] + r[1][a] * jr[1][c] + r[2][a] * jr[2][c];
        }
    }
    double trace = m[0][0] + m[1][1] + m[2][2];
    double minors = m[0][0] * m[1][1] - m[0][1] * m[1][0] + m[0][0] * m[2][2] - m[0][2] * m[2][0] +
                    m[1][1] * m[2][2] - m[1][2] * m[2][1];
    double disc = trace * trace - 4.0 * minors;
    return 0.5 * (trace + sqrt(disc > 0.0 ? disc : 0.0));
}

m3LeverBody m3LeverBodyOf(const m3World* world, int32_t b)
{
    m3LeverBody body = {0.0f, m3MakeZeroMat3(), {0.0f, 0.0f, 0.0f}};
    if (b >= 0)
    {
        body.invMass = world->bodies.invMass[b];
        body.invInertia = world->bodies.invInertiaLocal[b];
        body.center = world->bodies.localCenters[b];
    }
    return body;
}

bool m3LeverFits(uint8_t type, const m3LeverBody* a, m3Vec3 localA, const m3LeverBody* b,
                 m3Vec3 localB)
{
    if (!m3JointHasBlocks(type))
    {
        return true;
    }
    double w = (double)a->invMass + (double)b->invMass;
    double turn = m3LeverTurn(&a->invInertia, m3Sub3(localA, a->center)) +
                  m3LeverTurn(&b->invInertia, m3Sub3(localB, b->center));
    return turn <= M3_JOINT_LEVER_LIMIT * w; // false for NaN
}

// Joint j's fit with body taking next.
static bool JointFits(const m3World* world, int32_t j, int32_t body, const m3LeverBody* next)
{
    int32_t bodyA = world->joints.jointBodyA[j];
    int32_t bodyB = world->joints.jointBodyB[j];
    m3LeverBody a = bodyA == body ? *next : m3LeverBodyOf(world, bodyA);
    m3LeverBody b = bodyB == body ? *next : m3LeverBodyOf(world, bodyB);
    return m3LeverFits(world->joints.jointType[j], &a, world->joints.jointLocalA[j], &b,
                       world->joints.jointLocalB[j]);
}

bool m3BodyJointsFit(const m3World* world, int32_t body, const m3LeverBody* next)
{
    for (int32_t j = world->joints.bodyJointHead[body]; j != -1;
         j = world->joints.jointBodyA[j] == body ? world->joints.jointNextA[j]
                                                 : world->joints.jointNextB[j])
    {
        if (!JointFits(world, j, body, next))
        {
            return false;
        }
    }
    return true;
}
