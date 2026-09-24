// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Integration: the mover list, velocities and poses.

#include "integrate.h"

#include "body.h"
#include "solver.h"
#include "world_internal.h"

#include <math.h>

// The gyroscopic term of Euler's equations, integrated implicitly. In
// body coordinates the step solves I (w2 - w1) + h w2 x (I w2) = 0 for
// w2; one Newton iteration from w2 = w1 is enough at substep sizes, and
// the implicit form cannot pump energy into a tumbling body, whatever
// its shape. An isotropic tensor skips the step: w x (c w) vanishes in
// real arithmetic but not bit for bit in float, and spheres keep their
// exact trajectories. The gate compares are exact, so the branch is
// deterministic.
static m3Vec3 Gyroscopic(const m3World* world, int32_t body, m3Vec3 w, m3real h)
{
    const m3Mat3* inertia = &world->bodies.inertiaLocal[body];
    const m3real i00 = inertia->cx.x;
    const m3real i01 = inertia->cy.x;
    const m3real i02 = inertia->cz.x;
    const m3real i11 = inertia->cy.y;
    const m3real i12 = inertia->cz.y;
    const m3real i22 = inertia->cz.z;
    if (i01 == 0.0f && i02 == 0.0f && i12 == 0.0f && i00 == i11 && i11 == i22)
    {
        return w; // isotropic (or massless): the term vanishes
    }

    m3Quat q = world->bodies.transforms[body].q;
    m3Vec3 omega1 = m3InvRotateVec3(q, w);
    m3Vec3 omega2 = omega1;

    // One Newton iteration: residual b = I (w2 - w1) + h (w2 x I w2),
    // Jacobian J = I + h (skew(w2) I - skew(I w2)).
    const m3real w1 = omega2.x;
    const m3real w2 = omega2.y;
    const m3real w3 = omega2.z;
    const m3real Iw1 = i00 * w1 + i01 * w2 + i02 * w3;
    const m3real Iw2 = i01 * w1 + i11 * w2 + i12 * w3;
    const m3real Iw3 = i02 * w1 + i12 * w2 + i22 * w3;
    // omega2 - omega1 is zero on the first (only) iteration, so the
    // residual is just the gyroscopic term.
    m3Vec3 b = {
        h * (w2 * Iw3 - w3 * Iw2),
        h * (w3 * Iw1 - w1 * Iw3),
        h * (w1 * Iw2 - w2 * Iw1),
    };
    m3Mat3 J;
    J.cx = (m3Vec3){i00 + h * (w2 * i02 - w3 * i01), i01 + h * (w3 * i00 - w1 * i02 - Iw3),
                    i02 + h * (w1 * i01 - w2 * i00 + Iw2)};
    J.cy = (m3Vec3){i01 + h * (w2 * i12 - w3 * i11 + Iw3), i11 + h * (w3 * i01 - w1 * i12),
                    i12 + h * (w1 * i11 - w2 * i01 - Iw1)};
    J.cz = (m3Vec3){i02 + h * (w2 * i22 - w3 * i12 - Iw2), i12 + h * (w3 * i02 - w1 * i22 + Iw1),
                    i22 + h * (w1 * i12 - w2 * i02)};
    omega2 = m3Sub3(omega2, m3Solve3(&J, b));

    return m3RotateVec3(q, omega2);
}

// The mover list: awake dynamics and kinematics in ascending slot order,
// the bodies the substep loops touch. Kinematic targets turn into the
// velocities that land them this step. Returns the mover count.
int32_t m3BuildMovers(m3World* world, int32_t* movers, float dt)
{
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    int32_t moverCount = 0;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodies.bodyPool.alive[i] == 0 || world->bodies.bodyEnabled[i] == 0)
        {
            continue; // disabled bodies vanish from the step
        }
        uint8_t type = world->bodies.types[i];
        if (type == (uint8_t)m3_kinematicBody ||
            (type == (uint8_t)m3_dynamicBody && world->bodies.awake[i] != 0))
        {
            movers[moverCount] = i;
            moverCount += 1;
        }
        // A kinematic body with a target gets the velocities that land
        // it there this step, and the target is spent.
        if (type == (uint8_t)m3_kinematicBody && world->bodies.bodyHasTarget[i] != 0)
        {
            m3real servoInvDt = 1.0f / dt;
            const m3Transform* now = &world->bodies.transforms[i];
            const m3Transform* want = &world->bodies.bodyTarget[i];
            world->bodies.linearVelocities[i] =
                (m3Vec3){(m3real)(want->p.x - now->p.x) * servoInvDt,
                         (m3real)(want->p.y - now->p.y) * servoInvDt,
                         (m3real)(want->p.z - now->p.z) * servoInvDt};
            m3Quat dq = m3MulQuat(want->q, (m3Quat){-now->q.x, -now->q.y, -now->q.z, now->q.w});
            if (dq.w < 0.0f)
            {
                dq = (m3Quat){-dq.x, -dq.y, -dq.z, -dq.w};
            }
            world->bodies.angularVelocities[i] =
                m3MulSV3(2.0f * servoInvDt, (m3Vec3){dq.x, dq.y, dq.z});
            world->bodies.bodyHasTarget[i] = 0;
        }
    }

    return moverCount;
}

void m3IntegrateVelocities(m3World* world, const int32_t* movers, int32_t moverCount,
                           const m3Buoyancy* buoy, m3real h)
{
    // Integrate velocities (fixed body order): gravity, damping.
    for (int32_t m = 0; m < moverCount; ++m)
    {
        int32_t i = movers[m];
        if (world->bodies.types[i] != (uint8_t)m3_dynamicBody)
        {
            continue; // kinematics ride the list for positions only
        }
        m3Vec3 v = world->bodies.linearVelocities[i];
        m3Vec3 w = world->bodies.angularVelocities[i];
        v = m3Add3(v, m3MulSV3(h * world->bodies.gravityScales[i], world->gravity));
        // Host forces and torques integrate beside
        // gravity, every substep, so a force held for one step
        // delivers exactly force times dt.
        v = m3Add3(v, m3MulSV3(h * world->bodies.invMass[i], world->bodies.bodyForce[i]));
        if (world->bodies.bodyTorque[i].x != 0.0f || world->bodies.bodyTorque[i].y != 0.0f ||
            world->bodies.bodyTorque[i].z != 0.0f)
        {
            w = m3Add3(
                w, m3MulSV3(h, m3MulMV3(m3WorldInvInertia(world, i), world->bodies.bodyTorque[i])));
        }
        if (buoy->active > 0 && (buoy->lin[m] > 0.0f || buoy->force[m].y != 0.0f ||
                                 buoy->force[m].x != 0.0f || buoy->force[m].z != 0.0f))
        {
            // The water field: buoyant impulse, torque
            // about the submerged centroid, then drag pulls the
            // RELATIVE velocity toward the flow (the damping
            // recipe, recentered on the current).
            v = m3Add3(v, m3MulSV3(h * world->bodies.invMass[i], buoy->force[m]));
            w = m3Add3(w, m3MulSV3(h, m3MulMV3(m3WorldInvInertia(world, i), buoy->torque[m])));
            m3Vec3 rel = m3Sub3(v, buoy->flow[m]);
            v = m3Add3(buoy->flow[m], m3MulSV3(1.0f / (1.0f + h * buoy->lin[m]), rel));
            w = m3MulSV3(1.0f / (1.0f + h * buoy->ang[m]), w);
        }
        v = m3MulSV3(1.0f / (1.0f + h * world->bodies.linearDamping[i]), v);
        w = m3MulSV3(1.0f / (1.0f + h * world->bodies.angularDamping[i]), w);
        w = Gyroscopic(world, i, w, h);
        // Safety caps: a degenerate setup is held here instead of
        // pumping speed into infinity.
        m3real v2 = m3Dot3(v, v);
        m3real cap = world->maximumLinearSpeed;
        if (v2 > cap * cap)
        {
            v = m3MulSV3(cap / sqrtf(v2), v);
        }
        // The angular cap is waived for bodies that allow fast
        // rotation (bodyLocks bit 6). Its default sits far above any
        // real tumbling.
        if ((world->bodies.bodyLocks[i] & M3_LOCKS_ALLOW_FAST_ROTATION) == 0)
        {
            m3real w2 = m3Dot3(w, w);
            m3real wcap = world->maximumAngularSpeed;
            if (w2 > wcap * wcap)
            {
                w = m3MulSV3(wcap / sqrtf(w2), w);
            }
        }
        world->bodies.linearVelocities[i] = v;
        world->bodies.angularVelocities[i] = w;
    }
}

void m3IntegratePositions(m3World* world, const int32_t* movers, int32_t moverCount,
                          m3Vec3* deltaPos, m3Quat* deltaRot, m3real h)
{
    // Integrate positions and accumulate the substep deltas the
    // separation tracking reads.
    for (int32_t m = 0; m < moverCount; ++m)
    {
        int32_t i = movers[m];
        m3Vec3 v = world->bodies.linearVelocities[i];
        m3Vec3 w = world->bodies.angularVelocities[i];
        uint8_t locks = world->bodies.bodyLocks[i];
        if (locks != 0)
        {
            // Motion locks: locked components re-zero
            // every substep, in the STORED velocity too, so
            // contacts cannot bank motion on a frozen axis.
            if (locks & 1u)
                v.x = 0.0f;
            if (locks & 2u)
                v.y = 0.0f;
            if (locks & 4u)
                v.z = 0.0f;
            if (locks & 8u)
                w.x = 0.0f;
            if (locks & 16u)
                w.y = 0.0f;
            if (locks & 32u)
                w.z = 0.0f;
            world->bodies.linearVelocities[i] = v;
            world->bodies.angularVelocities[i] = w;
        }
        // Rigid bodies rotate about the center of mass: advance
        // the COM, spin, then place the origin back. A centered
        // body (lc zero) reduces to the plain origin update.
        m3Vec3 lc = world->bodies.localCenters[i];
        m3Vec3 rlcOld = m3RotateVec3(world->bodies.transforms[i].q, lc);
        double cx = world->bodies.transforms[i].p.x + (double)rlcOld.x + (double)(h * v.x);
        double cy = world->bodies.transforms[i].p.y + (double)rlcOld.y + (double)(h * v.y);
        double cz = world->bodies.transforms[i].p.z + (double)rlcOld.z + (double)(h * v.z);
        m3Vec3 dw = m3MulSV3(h, w);
        world->bodies.transforms[i].q = m3IntegrateRotation(world->bodies.transforms[i].q, dw);
        m3Vec3 rlcNew = m3RotateVec3(world->bodies.transforms[i].q, lc);
        world->bodies.transforms[i].p.x = cx - (double)rlcNew.x;
        world->bodies.transforms[i].p.y = cy - (double)rlcNew.y;
        world->bodies.transforms[i].p.z = cz - (double)rlcNew.z;
        deltaPos[i] = m3Add3(deltaPos[i], m3MulSV3(h, v));
        deltaRot[i] = m3IntegrateRotation(deltaRot[i], dw);
    }
}
