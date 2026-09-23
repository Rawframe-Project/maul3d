// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The fixed joint: a shared point and a locked relative rotation.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void PrepareFixed(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    const m3Transform* xfA = f->xfA;
    const m3Transform* xfB = f->xfB;
    // The weld: the create-time relative pose lives in
    // the stored frames; the rotation lock drives the live
    // relative rotation back to identity between them, and
    // the shared point block below handles translation.
    c->frameQA = m3MulQuat(xfA->q, world->joints.jointFrameQA[j]);
    c->frameQB = m3MulQuat(xfB->q, world->joints.jointFrameQB[j]);
    c->angularImpulse = world->joints.jointAngularImpulse[j];
}

static void SolveFixed(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 vA = s->vA;
    m3Vec3 wA = s->wA;
    m3Vec3 vB = s->vB;
    m3Vec3 wB = s->wB;
    const m3Quat* deltaRot = s->deltaRot;
    int useBias = s->useBias;
    // The weld: the same rotation lock as the prismatic,
    // driving the live relative rotation to the create-time
    // pose; the shared point block below welds translation.
    m3Vec3 bias = {0.0f, 0.0f, 0.0f};
    m3real massScale = 1.0f;
    m3real impulseScale = 0.0f;
    if (useBias)
    {
        m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
        m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
        m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
        m3Quat relQ = m3MulQuat(conjA, quatB);
        m3Vec3 rotVec = m3JointQuatToRotationVec(relQ);
        m3Vec3 cErr = m3RotateVec3(quatA, rotVec); // the prismatic lock's
                                                   // sign convention
        bias = m3MulSV3(c->softness.biasRate, cErr);
        massScale = c->softness.massScale;
        impulseScale = c->softness.impulseScale;
    }
    m3Mat3 k;
    m3Vec3 basis[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    m3Vec3* cols[3] = {&k.cx, &k.cy, &k.cz};
    for (int32_t a = 0; a < 3; ++a)
    {
        *cols[a] = m3Add3(m3MulMV3(c->invIA, basis[a]), m3MulMV3(c->invIB, basis[a]));
    }
    m3Vec3 cdotRot = m3Sub3(wB, wA);
    m3Vec3 sol = m3Solve3(&k, m3Add3(cdotRot, bias));
    m3Vec3 delta = m3Sub3(m3MulSV3(-massScale, sol), m3MulSV3(impulseScale, c->angularImpulse));
    c->angularImpulse = m3Add3(c->angularImpulse, delta);
    wA = m3Sub3(wA, m3MulMV3(c->invIA, delta));
    wB = m3Add3(wB, m3MulMV3(c->invIB, delta));
    m3SolveJointPoint(world, c, s, vA, wA, vB, wB);
}

const m3JointKind m3_fixedJointKind = {PrepareFixed, m3WarmStartAngularLock, SolveFixed};
