// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Integration: the mover list, velocities and poses.

#include "integrate.h"

#include "body_state.h"
#include "rigid_flow.h"
#include "solver_math.h"
#include "world_internal.h"

#include <math.h>

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
            float servoInvDt = 1.0f / dt;
            const m3Transform* now = &world->bodies.transforms[i];
            const m3Transform* want = &world->bodies.bodyTarget[i];
            world->bodies.linearVelocities[i] =
                (m3Vec3){(float)(want->p.x - now->p.x) * servoInvDt,
                         (float)(want->p.y - now->p.y) * servoInvDt,
                         (float)(want->p.z - now->p.z) * servoInvDt};
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

// The longest force or torque the setters let a body hold: M3_FORCE_LIMIT
// on every axis.
#define M3_HELD_FORCE_LENGTH (1.7320508f * M3_FORCE_LIMIT)

// The factor, at most 1, that brings the length of a + b within
// M3_HELD_FORCE_LENGTH. Exactly 1 when the sum is within it, so a step
// whose forces stay there computes the same bits as without the hold.
// Both are finite, so the sum's length in double is too.
static float HeldShare(m3Vec3 a, m3Vec3 b)
{
    double x = (double)a.x + (double)b.x;
    double y = (double)a.y + (double)b.y;
    double z = (double)a.z + (double)b.z;
    double length = sqrt(x * x + y * y + z * z);
    return length > (double)M3_HELD_FORCE_LENGTH ? (float)((double)M3_HELD_FORCE_LENGTH / length)
                                                 : 1.0f;
}

void m3IntegrateVelocities(m3World* world, const int32_t* movers, int32_t moverCount,
                           const m3Buoyancy* buoy, float h)
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
        // The host's force and torque and the water's are the only
        // ones the body carries in, and their totals are held to the
        // length a host force alone may reach, which the setters and
        // restore keep. The water's sums over every shape and volume,
        // and on a thin body its loose box could otherwise spin it past
        // float range, where a drag or damping factor of zero turns the
        // infinity into a NaN.
        bool wet = buoy->active > 0 && (buoy->lin[m] > 0.0f || buoy->force[m].y != 0.0f ||
                                        buoy->force[m].x != 0.0f || buoy->force[m].z != 0.0f);
        m3Vec3 force = world->bodies.bodyForce[i];
        m3Vec3 torque = world->bodies.bodyTorque[i];
        m3Vec3 wetForce = {0.0f, 0.0f, 0.0f};
        m3Vec3 wetTorque = {0.0f, 0.0f, 0.0f};
        if (wet)
        {
            wetForce = buoy->force[m];
            wetTorque = buoy->torque[m];
            float share = HeldShare(force, wetForce);
            if (share < 1.0f)
            {
                force = m3MulSV3(share, force);
                wetForce = m3MulSV3(share, wetForce);
            }
            share = HeldShare(torque, wetTorque);
            if (share < 1.0f)
            {
                torque = m3MulSV3(share, torque);
                wetTorque = m3MulSV3(share, wetTorque);
            }
        }
        v = m3Add3(v, m3MulSV3(h * world->bodies.gravityScales[i], world->gravity));
        // Host forces and torques integrate beside
        // gravity, every substep, so a force held for one step
        // delivers exactly force times dt.
        v = m3Add3(v, m3MulSV3(h * world->bodies.invMass[i], force));
        if (torque.x != 0.0f || torque.y != 0.0f || torque.z != 0.0f)
        {
            w = m3Add3(w, m3MulSV3(h, m3MulMV3(m3WorldInvInertia(world, i), torque)));
        }
        if (wet)
        {
            // The water field: buoyant impulse, torque
            // about the submerged centroid, then drag pulls the
            // RELATIVE velocity toward the flow (the damping
            // recipe, recentered on the current).
            v = m3Add3(v, m3MulSV3(h * world->bodies.invMass[i], wetForce));
            w = m3Add3(w, m3MulSV3(h, m3MulMV3(m3WorldInvInertia(world, i), wetTorque)));
            m3Vec3 rel = m3Sub3(v, buoy->flow[m]);
            v = m3Add3(buoy->flow[m], m3MulSV3(1.0f / (1.0f + h * buoy->lin[m]), rel));
            w = m3MulSV3(1.0f / (1.0f + h * buoy->ang[m]), w);
        }
        v = m3MulSV3(1.0f / (1.0f + h * world->bodies.linearDamping[i]), v);
        w = m3MulSV3(1.0f / (1.0f + h * world->bodies.angularDamping[i]), w);
        // Safety caps: a degenerate setup is held here instead of
        // pumping speed into infinity.
        v = m3CapSpeed(v, world->maximumLinearSpeed);
        // The angular cap is waived for bodies that allow fast
        // rotation (bodyLocks bit 6); m3CapMoverSpeeds still holds them
        // within the speed limit. Its default sits far above any real
        // tumbling.
        if ((world->bodies.bodyLocks[i] & M3_LOCKS_ALLOW_FAST_ROTATION) == 0)
        {
            w = m3CapSpeed(w, world->maximumAngularSpeed);
        }
        world->bodies.linearVelocities[i] = v;
        world->bodies.angularVelocities[i] = w;
    }
}

void m3CapMoverSpeeds(m3World* world, const int32_t* movers, int32_t moverCount)
{
    // Warm starting and restitution add stored impulses to velocities
    // after the caps above, which a world may set high and a flagged body
    // skips; this holds every dynamic mover within the speed limit before
    // positions or the next step read it. Kinematic movers take no
    // impulses, and their servo velocity must land its target.
    for (int32_t m = 0; m < moverCount; ++m)
    {
        int32_t i = movers[m];
        if (world->bodies.types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        world->bodies.linearVelocities[i] =
            m3CapSpeed(world->bodies.linearVelocities[i], M3_SPEED_LIMIT);
        world->bodies.angularVelocities[i] =
            m3CapSpeed(world->bodies.angularVelocities[i], M3_SPEED_LIMIT);
    }
}

// A dynamic body free to turn about every axis rotates as a free rigid
// body through the substep, which turns its spin with its pose and keeps
// its world angular momentum; the solve has already applied every
// impulse. A kinematic body, a body with a locked rotation axis or one
// whose moments leave float range turns at its spin. Returns the new
// rotation, and stores the new spin.
static m3Quat TurnPose(m3World* world, int32_t i, uint8_t locks, m3Quat q, float h)
{
    m3Vec3 w = world->bodies.angularVelocities[i];
    m3Vec3 moments = world->bodies.principalMoments[i];
    if (world->bodies.types[i] == (uint8_t)m3_dynamicBody && (locks & 56u) == 0 &&
        m3FreeRotationApplies(moments))
    {
        m3Quat turned;
        m3FreeRotation(moments, world->bodies.principalFrames[i], q, w, h, &turned,
                       &world->bodies.angularVelocities[i]);
        return turned;
    }
    return m3IntegrateRotation(q, m3MulSV3(h, w));
}

void m3IntegratePositions(m3World* world, const int32_t* movers, int32_t moverCount,
                          m3Vec3* deltaPos, m3Quat* deltaRot, float h)
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
            {
                v.x = 0.0f;
            }
            if (locks & 2u)
            {
                v.y = 0.0f;
            }
            if (locks & 4u)
            {
                v.z = 0.0f;
            }
            if (locks & 8u)
            {
                w.x = 0.0f;
            }
            if (locks & 16u)
            {
                w.y = 0.0f;
            }
            if (locks & 32u)
            {
                w.z = 0.0f;
            }
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
        m3Quat q = world->bodies.transforms[i].q;
        world->bodies.transforms[i].q = TurnPose(world, i, locks, q, h);
        m3Vec3 rlcNew = m3RotateVec3(world->bodies.transforms[i].q, lc);
        world->bodies.transforms[i].p.x = cx - (double)rlcNew.x;
        world->bodies.transforms[i].p.y = cy - (double)rlcNew.y;
        world->bodies.transforms[i].p.z = cz - (double)rlcNew.z;
        world->bodies.transforms[i].p = m3HoldPosition(world->bodies.transforms[i].p);
        deltaPos[i] = m3Add3(deltaPos[i], m3MulSV3(h, v));
        m3Quat turn = m3MulQuat(world->bodies.transforms[i].q, (m3Quat){-q.x, -q.y, -q.z, q.w});
        deltaRot[i] = m3NormalizeQuat(m3MulQuat(turn, deltaRot[i]));
    }
}
