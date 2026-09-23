// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The parallel joint: keeps the two frame z axes parallel, nothing else.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void SolveParallel(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 wA = s->wA;
    m3Vec3 wB = s->wB;
    const m3Quat* deltaRot = s->deltaRot;
    int useBias = s->useBias;
    // PARALLEL: the revolute's collinearity 2x2 and
    // nothing else. The twist about the shared axis and
    // every translation stay free, so the branch continues
    // PAST the shared point weld below.
    m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
    m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
    if (quatA.x * quatB.x + quatA.y * quatB.y + quatA.z * quatB.z + quatA.w * quatB.w < 0.0f)
    {
        quatB = (m3Quat){-quatB.x, -quatB.y, -quatB.z, -quatB.w};
    }
    m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
    m3Quat relQ = m3MulQuat(conjA, quatB);
    m3Vec3 perpX = m3JointPerpColumn(quatA, relQ, (m3Vec3){1.0f, 0.0f, 0.0f});
    m3Vec3 perpY = m3JointPerpColumn(quatA, relQ, (m3Vec3){0.0f, 1.0f, 0.0f});
    c->perpAxisX = perpX;
    c->perpAxisY = perpY;
    m3real biasX = 0.0f;
    m3real biasY = 0.0f;
    m3real massScale = 1.0f;
    m3real impulseScale = 0.0f;
    if (useBias)
    {
        biasX = c->softness.biasRate * relQ.x;
        biasY = c->softness.biasRate * relQ.y;
        massScale = c->softness.massScale;
        impulseScale = c->softness.impulseScale;
    }
    m3Vec3 sumX = m3Add3(m3MulMV3(c->invIA, perpX), m3MulMV3(c->invIB, perpX));
    m3Vec3 sumY = m3Add3(m3MulMV3(c->invIA, perpY), m3MulMV3(c->invIB, perpY));
    m3real kxx = m3Dot3(perpX, sumX);
    m3real kyy = m3Dot3(perpY, sumY);
    m3real kxy = m3Dot3(perpX, sumY);
    m3Vec3 wRel = m3Sub3(wB, wA);
    m3real cdotX = m3Dot3(wRel, perpX) + biasX;
    m3real cdotY = m3Dot3(wRel, perpY) + biasY;
    m3real det = kxx * kyy - kxy * kxy;
    m3real solX = 0.0f;
    m3real solY = 0.0f;
    if (det != 0.0f)
    {
        m3real inv = 1.0f / det;
        solX = inv * (kyy * cdotX - kxy * cdotY);
        solY = inv * (kxx * cdotY - kxy * cdotX);
    }
    m3real deltaX = -massScale * solX - impulseScale * c->perpImpulseX;
    m3real deltaY = -massScale * solY - impulseScale * c->perpImpulseY;
    c->perpImpulseX += deltaX;
    c->perpImpulseY += deltaY;
    m3Vec3 angular = m3Add3(m3MulSV3(deltaX, perpX), m3MulSV3(deltaY, perpY));
    wA = m3Sub3(wA, m3MulMV3(c->invIA, angular));
    wB = m3Add3(wB, m3MulMV3(c->invIB, angular));
    world->bodies.angularVelocities[c->bodyA] = wA;
    world->bodies.angularVelocities[c->bodyB] = wB;
    return;
}

const m3JointKind m3_parallelJointKind = {m3PrepareHingeFrame, m3WarmStartHinge, SolveParallel};
