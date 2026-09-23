// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Soft Step solver in 3D, ported from Maul2D's proven solver.c
// (itself adapted from Box2D v3, MIT, Erin Catto): prepare once, then
// per substep [integrate velocities, warm start, solve with bias,
// integrate positions, relax without bias], then a restitution pass
// and the impulse store. Separations are re-evaluated inside substeps
// from accumulated float deltas (never fresh world-space math), the
// Jacobian uses FIXED prepare-time anchors (reference discipline), and
// friction clamps to the Coulomb disc by scaling, never a per-axis
// box. 2a bodies are spheres: inertia is a scalar and gyroscopic
// torque is exactly zero, so the implicit gyroscopic solve arrives
// with hulls in 2b.

#include "solver.h"
#include "body.h"
#include "broad_phase.h"
#include "character.h"
#include "contact_solver.h"
#include "continuous.h"
#include "island.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "narrowphase.h"
#include "softbody.h"
#include "vehicle.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

// Reference formula (b2MakeSoft, copied verbatim from Maul2D):
// bias = w/(2z+hw), massScale = hw(2z+hw)/(1+hw(2z+hw)),
// impulseScale = 1/(1+hw(2z+hw)).
m3Softness m3MakeSoft(m3real hertz, m3real zeta, m3real h)
{
    if (hertz == 0.0f)
    {
        return (m3Softness){0.0f, 0.0f, 0.0f};
    }
    m3real omega = 2.0f * M3_PI * hertz;
    m3real a1 = 2.0f * zeta + h * omega;
    m3real a2 = h * omega * a1;
    m3real a3 = 1.0f / (1.0f + a2);
    return (m3Softness){omega / a1, a2 * a3, a3};
}

// I_w^-1 = R I_l^-1 R^T, built by applying the operator to the world
// basis vectors. Frozen at prepare like the anchors (reference
// discipline); the per-substep refresh arrives with the gyroscopic
// slice.
m3Mat3 m3WorldInvInertia(const m3World* world, int32_t body)
{
    if (world->bodies.types[body] != (uint8_t)m3_dynamicBody)
    {
        return m3MakeZeroMat3();
    }
    m3Quat q = world->bodies.transforms[body].q;
    m3Mat3 il = world->bodies.invInertiaLocal[body];
    m3Mat3 r;
    r.cx = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f})));
    r.cy = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 1.0f, 0.0f})));
    r.cz = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 0.0f, 1.0f})));
    return r;
}

// Solve J * x = b for a general 3x3 via Cramer's rule. A singular
// Jacobian returns zero, which leaves omega unchanged (the safe step).
m3Vec3 m3Solve3(const m3Mat3* J, m3Vec3 b)
{
    m3Vec3 cxy = m3Cross3(J->cy, J->cz);
    m3real det = m3Dot3(J->cx, cxy);
    if (det == 0.0f)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    m3real inv = 1.0f / det;
    m3Vec3 x;
    x.x = inv * m3Dot3(b, cxy);
    x.y = inv * m3Dot3(J->cx, m3Cross3(b, J->cz));
    x.z = inv * m3Dot3(J->cx, m3Cross3(J->cy, b));
    return x;
}

// Implicit gyroscopic torque (the reference's Newton-Raphson step on
// I*(w2 - w1) + h * cross(w2, I*w2) = 0, solved in body coordinates
// where the Jacobian is cheap). Long skinny bodies tumble correctly
// and never gain energy; the implicit form is unconditionally stable.
// Exactly isotropic tensors are gated out: cross(w, c*w) is zero in
// real arithmetic but not bit-zero in float, and spheres must keep
// their bit-identical trajectories. The gate compares are exact, so
// the branch itself is deterministic.
static m3Vec3 GyroscopicOmega(const m3World* world, int32_t body, m3Vec3 w, m3real h)
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

    // One Newton iteration (the reference count): residual
    // b = I*(w2 - w1) + h * (w2 x I*w2), Jacobian
    // J = I + h * (skew(w2) * I - skew(I*w2)).
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

// The per-mover water field of one step; active is zero when no volume
// is alive (or the scratch stalled), and then the arrays are unused.
typedef struct m3Buoyancy
{
    m3Vec3* force;
    m3Vec3* torque;
    m3Vec3* flow;
    float* lin;
    float* ang;
    int32_t active;
} m3Buoyancy;

// Resets the event streams, then walks the old and new pair lists (both
// sorted) and emits begin and end events in pair order.
static void EmitContactEvents(m3World* world, const uint64_t* oldKeys,
                              const m3Manifold* oldManifolds, int32_t oldCount)
{
    // Contact events: a canonical merge walk of the old and new pair
    // lists (both sorted). Serial and after the parallel narrowphase
    // on purpose: appends must happen in pair order, bit-stably.
    world->events.beginEventCount = 0;
    world->events.endEventCount = 0;
    world->events.sensorBeginEventCount = 0;
    world->events.sensorEndEventCount = 0;
    world->events.fragmentEventCount = 0;
    world->events.fragmentRecipeCount = 0;
    world->events.fragmentDropped = 0;
    world->events.hitEventCount = 0;
    world->events.hitEventsDropped = 0;
    world->events.moveEventCount = 0;
    world->joints.jointBreakEventCount = 0;
    {
        int32_t iNew = 0;
        int32_t iOld = 0;
        while (iNew < world->contacts.pairCount || iOld < oldCount)
        {
            uint64_t keyNew =
                iNew < world->contacts.pairCount ? world->contacts.pairKeys[iNew] : UINT64_MAX;
            uint64_t keyOld = iOld < oldCount ? oldKeys[iOld] : UINT64_MAX;
            int touchNew = 0;
            int touchOld = 0;
            uint64_t key;
            if (keyNew < keyOld)
            {
                key = keyNew;
                touchNew = world->contacts.manifolds[iNew].pointCount > 0;
                iNew += 1;
            }
            else if (keyOld < keyNew)
            {
                key = keyOld;
                touchOld = oldManifolds[iOld].pointCount > 0;
                iOld += 1;
            }
            else
            {
                key = keyNew;
                touchNew = world->contacts.manifolds[iNew].pointCount > 0;
                touchOld = oldManifolds[iOld].pointCount > 0;
                iNew += 1;
                iOld += 1;
            }
            if (touchNew == touchOld)
            {
                continue;
            }
            int32_t sA = (int32_t)(key >> 32);
            int32_t sB = (int32_t)(key & 0xFFFFFFFFu);
            // A vanished pair whose shape died emits nothing: the id
            // would be stale (documented on the API).
            if (world->shapes.shapePool.alive[sA] == 0 || world->shapes.shapePool.alive[sB] == 0)
            {
                continue;
            }
            m3ContactEvent event;
            event.shapeA =
                (m3ShapeId){sA + 1, world->worldIndex0, world->shapes.shapePool.generations[sA]};
            event.shapeB =
                (m3ShapeId){sB + 1, world->worldIndex0, world->shapes.shapePool.generations[sB]};
            int sensorPair =
                world->shapes.shapeSensor[sA] != 0 || world->shapes.shapeSensor[sB] != 0;
            if (sensorPair)
            {
                if (touchNew && world->events.sensorBeginEventCount < world->contacts.pairCapacity)
                {
                    world->events.sensorBeginEvents[world->events.sensorBeginEventCount++] = event;
                }
                else if (touchOld &&
                         world->events.sensorEndEventCount < world->contacts.pairCapacity)
                {
                    world->events.sensorEndEvents[world->events.sensorEndEventCount++] = event;
                }
            }
            else if (touchNew && world->events.beginEventCount < world->contacts.pairCapacity)
            {
                world->events.beginEvents[world->events.beginEventCount++] = event;
            }
            else if (touchOld && world->events.endEventCount < world->contacts.pairCapacity)
            {
                world->events.endEvents[world->events.endEventCount++] = event;
            }
        }
    }
}

static void SizeScratchForStep(m3World* world)
{
    // Pre-flight sizing (V-STALL; repositioned by V-LAYOUT): a
    // starved step is deterministic SIZE-DRIVEN state evolution,
    // and struct sizes are not part of the cross-platform
    // contract. The estimate uses PINNED per-item byte budgets
    // chosen to dominate every platform's real sizes, and it runs
    // AFTER the pair scan so the pair count is THIS step's count:
    // the one-step lag was V-LAYOUT's whole crime (a stale budget
    // let real consumption race capacity on pileup spikes, and the
    // winner depended on sizeof). Counts are pure state, so every
    // cell grows on the same tick; the reactive NULL returns below
    // are loud backstops an honest run can no longer reach.
    {
        int64_t need = 64 * 1024 + 128 * (int64_t)world->bodies.bodyPool.maxIndex +
                       64 * (int64_t)world->shapes.shapePool.maxIndex +
                       1024 * (int64_t)world->contacts.pairCount +
                       1024 * (int64_t)world->joints.jointPool.maxIndex;
        if (need > (int64_t)world->scratch.capacity && world->scratch.capacity < (1 << 28))
        {
            int32_t grown = world->scratch.capacity;
            while ((int64_t)grown < need && grown < (1 << 28))
            {
                grown *= 2;
            }
            m3StackDestroy(&world->scratch);
            world->scratch = m3StackCreate(grown);
        }
    }
}

// Begin-of-step centers of mass and rotations: the sweeps the continuous
// pass needs.
static void CaptureSweepStarts(const m3World* world, m3Pos3* com0, m3Quat* rot0)
{
    int32_t sweepMax = world->bodies.bodyPool.maxIndex;
    for (int32_t i = 0; i < sweepMax; ++i)
    {
        if (world->bodies.bodyPool.alive[i] == 0)
        {
            continue;
        }
        m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[i].q, world->bodies.localCenters[i]);
        com0[i].x = world->bodies.transforms[i].p.x + (double)rlc.x;
        com0[i].y = world->bodies.transforms[i].p.y + (double)rlc.y;
        com0[i].z = world->bodies.transforms[i].p.z + (double)rlc.z;
        rot0[i] = world->bodies.transforms[i].q;
    }
}

// The mover list: awake dynamics and kinematics in ascending slot order,
// the bodies the substep loops touch. Kinematic targets turn into the
// velocities that land them this step. Returns the mover count.
static int32_t BuildMovers(m3World* world, int32_t* movers, float dt)
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
        // The kinematic servo: choose velocities so this
        // step lands the body ON its target, then clear the order.
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

// Water volumes: per-mover buoyancy force, torque, drag rates and the
// blended flow, computed once per step from the step-start pose.
static void PrepareBuoyancy(m3World* world, const int32_t* movers, int32_t moverCount,
                            m3Buoyancy* b)
{
    memset(b, 0, sizeof(*b));
    for (int32_t k = 0; k < world->water.waterPool.maxIndex; ++k)
    {
        b->active += world->water.waterPool.alive[k];
    }
    if (b->active > 0 && moverCount > 0)
    {
        b->force = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
        b->torque = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
        b->flow = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
        b->lin = (float*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(float));
        b->ang = (float*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(float));
        if (b->force == NULL || b->torque == NULL || b->flow == NULL || b->lin == NULL ||
            b->ang == NULL)
        {
            b->active = 0; // transient scratch stall: dry step
        }
    }
    if (b->active > 0 && moverCount > 0)
    {
        for (int32_t m = 0; m < moverCount; ++m)
        {
            int32_t i = movers[m];
            b->force[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
            b->torque[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
            b->flow[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
            b->lin[m] = 0.0f;
            b->ang[m] = 0.0f;
            if (world->bodies.types[i] != (uint8_t)m3_dynamicBody)
            {
                continue;
            }
            m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[i].q, world->bodies.localCenters[i]);
            double comX = world->bodies.transforms[i].p.x + (double)rlc.x;
            double comY = world->bodies.transforms[i].p.y + (double)rlc.y;
            double comZ = world->bodies.transforms[i].p.z + (double)rlc.z;
            float fracSum = 0.0f;
            for (int32_t shape = world->bodies.bodyShapeHead[i]; shape >= 0;
                 shape = world->shapes.shapeNext[shape])
            {
                double slo[3];
                double shi[3];
                m3ShapeFatAabb(world, shape, slo, shi);
                double shapeVol = (shi[0] - slo[0]) * (shi[1] - slo[1]) * (shi[2] - slo[2]);
                if (!(shapeVol > 0.0))
                {
                    continue; // a plane's infinite box never swims
                }
                for (int32_t k = 0; k < world->water.waterPool.maxIndex; ++k)
                {
                    if (world->water.waterPool.alive[k] == 0)
                    {
                        continue;
                    }
                    double clo[3];
                    double chi[3];
                    clo[0] =
                        slo[0] > world->water.waterLo[k].x ? slo[0] : world->water.waterLo[k].x;
                    clo[1] =
                        slo[1] > world->water.waterLo[k].y ? slo[1] : world->water.waterLo[k].y;
                    clo[2] =
                        slo[2] > world->water.waterLo[k].z ? slo[2] : world->water.waterLo[k].z;
                    chi[0] =
                        shi[0] < world->water.waterHi[k].x ? shi[0] : world->water.waterHi[k].x;
                    chi[1] =
                        shi[1] < world->water.waterHi[k].y ? shi[1] : world->water.waterHi[k].y;
                    chi[2] =
                        shi[2] < world->water.waterHi[k].z ? shi[2] : world->water.waterHi[k].z;
                    if (chi[0] <= clo[0] || chi[1] <= clo[1] || chi[2] <= clo[2])
                    {
                        continue;
                    }
                    double subVol = (chi[0] - clo[0]) * (chi[1] - clo[1]) * (chi[2] - clo[2]);
                    float frac = (float)(subVol / shapeVol);
                    frac = frac > 1.0f ? 1.0f : frac;
                    // Buoyant force opposes gravity, applied at the
                    // clipped box centroid: a half-submerged crate
                    // rights itself, an off-center bite spins it.
                    m3Vec3 f =
                        m3MulSV3(-(float)subVol * world->water.waterDensity[k], world->gravity);
                    m3Vec3 r = {(float)(0.5 * (clo[0] + chi[0]) - comX),
                                (float)(0.5 * (clo[1] + chi[1]) - comY),
                                (float)(0.5 * (clo[2] + chi[2]) - comZ)};
                    b->force[m] = m3Add3(b->force[m], f);
                    b->torque[m] = m3Add3(b->torque[m], m3Cross3(r, f));
                    b->flow[m] = m3Add3(b->flow[m], m3MulSV3(frac, world->water.waterFlow[k]));
                    b->lin[m] += world->water.waterLinDrag[k] * frac;
                    b->ang[m] += world->water.waterAngDrag[k] * frac;
                    fracSum += frac;
                }
            }
            if (fracSum > 0.0f)
            {
                b->flow[m] = m3MulSV3(1.0f / fracSum, b->flow[m]);
            }
        }
    }
}

static void IntegrateVelocities(m3World* world, const int32_t* movers, int32_t moverCount,
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
        w = GyroscopicOmega(world, i, w, h);
        // Hard linear speed cap, the reference clamp.
        m3real v2 = m3Dot3(v, v);
        m3real cap = world->maximumLinearSpeed;
        if (v2 > cap * cap)
        {
            v = m3MulSV3(cap / sqrtf(v2), v);
        }
        // Hard angular speed cap with the reference's
        // allowFastRotation escape hatch on bodyLocks bit 6. The
        // default cap is a catastrophe guard like the 400 m/s
        // linear one, far above legal tumbling, so scenes that
        // never touch the knob keep their bits.
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

static void IntegratePositions(m3World* world, const int32_t* movers, int32_t moverCount,
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

static void AdvanceWind(m3World* world, float dt)
{
    // Wind phase: accumulated STATE, so a rollback resumes
    // the exact same gust wave. Wrapped to keep the float honest.
    if (world->windGustHertz > 0.0f)
    {
        world->windPhase += 2.0f * M3_PI * world->windGustHertz * dt;
        if (world->windPhase > 2.0f * M3_PI)
        {
            world->windPhase -= 2.0f * M3_PI * (m3real)(int32_t)(world->windPhase / (2.0f * M3_PI));
        }
    }
}

static void BreakJoints(m3World* world, m3real invH)
{
    // Joint breakage: reactions over threshold destroy the
    // joint and emit the break event, serially in ascending joint
    // order, a pure function of state (like fragmentation: derived
    // transitions never need their own journal op).
    {
        int32_t maxJoint = world->joints.jointPool.maxIndex;
        for (int32_t j = 0; j < maxJoint; ++j)
        {
            if (world->joints.jointPool.alive[j] == 0)
            {
                continue;
            }
            m3real maxForce = world->joints.jointBreak[j].x;
            m3real maxTorque = world->joints.jointBreak[j].y;
            if (maxForce == 0.0f && maxTorque == 0.0f)
            {
                continue;
            }
            m3real force;
            m3real torque;
            m3JointReactionMagnitudes(world, j, invH, &force, &torque);
            if ((maxForce > 0.0f && force > maxForce) || (maxTorque > 0.0f && torque > maxTorque))
            {
                m3JointId id = {j + 1, world->worldIndex0, world->joints.jointPool.generations[j]};
                m3AppendJointBreakEvent(world, id);
                m3DestroyJointInternal(world, j);
            }
        }
    }
}

// Labels every awake dynamic body with its island root and returns the
// island count (observer data).
static int32_t LabelIslands(m3World* world, const int32_t* islandParent)
{
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    // Island census before the sleep pass retires anyone: awake
    // dynamic union-find roots, an observer count, and the
    // per-body island label the extras draw tints by.
    // Sleeping bodies keep the label of the island they slept in.
    int32_t islands = 0;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodies.bodyPool.alive[i] != 0 &&
            world->bodies.types[i] == (uint8_t)m3_dynamicBody && world->bodies.awake[i] != 0)
        {
            int32_t root = i;
            while (islandParent[root] != root)
            {
                root = islandParent[root];
            }
            world->bodies.bodyIsland[i] = root;
            if (root == i)
            {
                islands += 1;
            }
        }
    }
    return islands;
}

static void EmitMoveEvents(m3World* world, const int32_t* movers, int32_t moverCount)
{
    // Body move events: one per mover, ascending body order
    // (the mover list is built that way), post-step transform, and
    // fellAsleep on the step the island dropped off. Capacity is
    // bodyCapacity: movers cannot overflow it.
    for (int32_t m = 0; m < moverCount; ++m)
    {
        int32_t i = movers[m];
        m3BodyMoveEvent* e = &world->events.moveEvents[world->events.moveEventCount++];
        e->body = (m3BodyId){i + 1, world->worldIndex0, world->bodies.bodyPool.generations[i]};
        e->transform = world->bodies.transforms[i];
        e->fellAsleep =
            world->bodies.types[i] == (uint8_t)m3_dynamicBody && world->bodies.awake[i] == 0;
    }
}

// Everything one step takes from the scratch stack, in the order it takes
// it. The order is part of the stall behavior: a starved step fails at
// the same allocation on every platform.
typedef struct m3StepScratch
{
    m3Pos3* com0; // begin-of-step centers of mass (continuous, sleep, riders)
    m3Quat* rot0;
    int32_t* islandParent;
    m3ContactConstraint* constraints;
    int32_t constraintCount;
    m3Vec3* deltaPos; // per-body position and rotation drift within the step
    m3Quat* deltaRot;
    m3SolverColoring coloring;
    int32_t usedColors;
    m3JointConstraint* joints;
    int32_t jointCount;
    int32_t* movers;
    int32_t moverCount;
    m3Buoyancy buoy;
    m3real h;
    m3real invH;
} m3StepScratch;

// A step that starved the scratch stalled harmlessly; the next one
// arrives with double the room. Growth is driven by sizes alone, so twins
// and replays stall and grow on the same ticks.
static void GrowScratchAfterStall(m3World* world)
{
    if (world->scratch.overflow != 0 && world->scratch.capacity < (1 << 28))
    {
        int32_t bigger = world->scratch.capacity * 2;
        m3StackDestroy(&world->scratch);
        world->scratch = m3StackCreate(bigger);
    }
}

// Copies the previous pairs and manifolds aside before the pair scan
// overwrites them; the warm-start carry and the event walk read the copy.
// The buffers belong to the world, so this never allocates.
static int32_t StashPairs(m3World* world)
{
    int32_t oldCount = world->contacts.pairCount;
    if (oldCount > 0)
    {
        memcpy(world->contacts.stashPairKeys, world->contacts.pairKeys,
               (size_t)oldCount * sizeof(uint64_t));
        memcpy(world->contacts.stashManifolds, world->contacts.manifolds,
               (size_t)oldCount * sizeof(m3Manifold));
    }
    return oldCount;
}

static void* ScratchArray(m3World* world, int32_t count, int32_t elementSize)
{
    return m3StackAlloc(&world->scratch, count > 0 ? count * elementSize : elementSize);
}

// The sweep starts, the island wake pass (a sleeping body touched by an
// awake one joins this step) and the per-body drift accumulators. Returns
// false on a scratch stall.
static bool BeginStep(m3World* world, m3StepScratch* s)
{
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    s->com0 = (m3Pos3*)ScratchArray(world, maxBody, (int32_t)sizeof(m3Pos3));
    s->rot0 = (m3Quat*)ScratchArray(world, maxBody, (int32_t)sizeof(m3Quat));
    if (s->com0 == NULL || s->rot0 == NULL)
    {
        return false; // unreachable after SizeScratchForStep; a backstop
    }
    CaptureSweepStarts(world, s->com0, s->rot0);
    s->islandParent = m3IslandWakePass(world);
    if (s->islandParent == NULL)
    {
        return false;
    }
    s->constraints = (m3ContactConstraint*)ScratchArray(world, world->contacts.pairCount,
                                                        (int32_t)sizeof(m3ContactConstraint));
    s->deltaPos = (m3Vec3*)ScratchArray(world, maxBody, (int32_t)sizeof(m3Vec3));
    s->deltaRot = (m3Quat*)ScratchArray(world, maxBody, (int32_t)sizeof(m3Quat));
    if (s->constraints == NULL || s->deltaPos == NULL || s->deltaRot == NULL)
    {
        return false;
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        s->deltaPos[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
        s->deltaRot[i] = m3MakeIdentityQuat();
    }
    return true;
}

// Contact and joint constraints, the graph coloring, the mover list and
// the water field for the substep loop. Returns false on a scratch stall.
static bool PrepareSolve(m3World* world, m3StepScratch* s, float dt, int32_t substeps)
{
    s->h = dt / (m3real)substeps;
    s->invH = s->h > 0.0f ? 1.0f / s->h : 0.0f;
    s->constraintCount = m3PrepareContacts(world, s->constraints, s->h);
    if (!m3BuildColoring(world, s->constraints, s->constraintCount, &s->coloring))
    {
        return false;
    }
    s->usedColors = 0;
    for (int32_t c = 0; c < M3_GRAPH_COLORS + 1; ++c)
    {
        if (s->coloring.starts[c + 1] > s->coloring.starts[c])
        {
            s->usedColors += 1;
        }
    }
    s->joints = (m3JointConstraint*)ScratchArray(world, world->joints.jointPool.maxIndex,
                                                 (int32_t)sizeof(m3JointConstraint));
    if (s->joints == NULL)
    {
        return false;
    }
    s->jointCount = m3PrepareJoints(world, s->joints, s->h);
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    s->movers = (int32_t*)m3StackAlloc(&world->scratch,
                                       maxBody > 0 ? maxBody * (int32_t)sizeof(int32_t) : 4);
    if (s->movers == NULL)
    {
        return false;
    }
    s->moverCount = BuildMovers(world, s->movers, dt);
    PrepareBuoyancy(world, s->movers, s->moverCount, &s->buoy);
    return true;
}

// The substep loop, the reference schedule: integrate velocities, warm
// start, solve with bias, integrate positions, relax without bias.
static void SolveSubsteps(m3World* world, m3StepScratch* s, int32_t substeps)
{
    for (int32_t sub = 0; sub < substeps; ++sub)
    {
        IntegrateVelocities(world, s->movers, s->moverCount, &s->buoy, s->h);
        m3WarmStartJoints(world, s->joints, s->jointCount, s->deltaRot);
        m3RunColored(world, &s->coloring, s->constraints, s->deltaPos, s->deltaRot, s->invH, 0, 1);
        m3SolveJoints(world, s->joints, s->jointCount, s->deltaPos, s->deltaRot, s->h, s->invH, 1);
        m3RunColored(world, &s->coloring, s->constraints, s->deltaPos, s->deltaRot, s->invH, 1, 0);
        IntegratePositions(world, s->movers, s->moverCount, s->deltaPos, s->deltaRot, s->h);
        m3SolveJoints(world, s->joints, s->jointCount, s->deltaPos, s->deltaRot, s->h, s->invH, 0);
        m3RunColored(world, &s->coloring, s->constraints, s->deltaPos, s->deltaRot, s->invH, 0, 0);
    }
}

// After the substeps: host forces are consumed (a force lives for one
// step, and only movers can carry one), restitution runs, impulses are
// stored for the next warm start, the wind advances and overloaded
// joints break.
static void FinishSolve(m3World* world, const m3StepScratch* s, float dt)
{
    for (int32_t m = 0; m < s->moverCount; ++m)
    {
        world->bodies.bodyForce[s->movers[m]] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->bodies.bodyTorque[s->movers[m]] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    m3Restitution(world, s->constraints, s->constraintCount);
    m3StoreContactImpulses(world, s->constraints, s->constraintCount);
    m3StoreJointImpulses(world, s->joints, s->jointCount);
    world->lastInvH = s->invH;
    AdvanceWind(world, dt);
    BreakJoints(world, s->invH);
}

void m3StepInternal(m3World* world, float dt, int32_t substeps)
{
    // The profile is written to the world only when the step completes,
    // so a stalled step keeps the previous one. Observer data only.
    m3Profile prof;
    memset(&prof, 0, sizeof(prof));
    double tStep = m3NowMs();

    GrowScratchAfterStall(world);
    int32_t oldCount = StashPairs(world);
    const uint64_t* oldKeys = world->contacts.stashPairKeys;
    const m3Manifold* oldManifolds = world->contacts.stashManifolds;

    // Vehicle suspension impulses land first, so the narrow phase and the
    // solver see the sprung chassis the way they see gravity.
    double t0 = m3NowMs();
    m3VehicleApplySuspension(world, dt);
    prof.vehicles = (float)(m3NowMs() - t0);

    t0 = m3NowMs();
    m3Result pairsResult = m3UpdatePairs(world);
    prof.broadphase = (float)(m3NowMs() - t0);
    t0 = m3NowMs();
    m3Result contactsResult = pairsResult == m3_success
                                  ? m3UpdateContacts(world, oldKeys, oldManifolds, oldCount)
                                  : pairsResult;
    prof.narrowphase = (float)(m3NowMs() - t0);
    if (pairsResult != m3_success || contactsResult != m3_success)
    {
        return; // a scratch stall (grown next step) or a full pair table:
                // the world stalls, never corrupts
    }

    t0 = m3NowMs();
    EmitContactEvents(world, oldKeys, oldManifolds, oldCount);
    prof.events = (float)(m3NowMs() - t0);

    t0 = m3NowMs();
    SizeScratchForStep(world);
    m3StackReset(&world->scratch);
    m3StepScratch s;
    memset(&s, 0, sizeof(s));
    if (!BeginStep(world, &s))
    {
        return; // scratch stall, grown next step
    }
    prof.prepare = (float)(m3NowMs() - t0);

    t0 = m3NowMs();
    if (!PrepareSolve(world, &s, dt, substeps))
    {
        return; // scratch stall, grown next step
    }
    SolveSubsteps(world, &s, substeps);
    FinishSolve(world, &s, dt);
    prof.solve = (float)(m3NowMs() - t0);

    t0 = m3NowMs();
    if (world->continuousEnabled != 0)
    {
        m3SolveContinuousPhase(world, s.com0, s.rot0);
    }
    prof.continuous = (float)(m3NowMs() - t0);
    int32_t islands = LabelIslands(world, s.islandParent);
    t0 = m3NowMs();
    if (world->sleepEnabled != 0)
    {
        m3IslandSleepPass(world, s.islandParent, s.com0, s.rot0, dt);
    }
    prof.sleep = (float)(m3NowMs() - t0);

    EmitMoveEvents(world, s.movers, s.moverCount);
    t0 = m3NowMs();
    m3CharacterCarryRiders(world, s.com0, s.rot0);
    prof.characters = (float)(m3NowMs() - t0);
    t0 = m3NowMs();
    m3SoftBodyPass(world, dt, substeps);
    prof.softBodies = (float)(m3NowMs() - t0);

    world->stepCount += 1;
    prof.step = (float)(m3NowMs() - tStep);
    world->profile = prof;
    world->lastIslandCount = islands;
    world->lastColorCount = s.usedColors;
    world->lastScratchPeak = world->scratch.top;
}

void m3World_Step(m3WorldId worldId, float dt, int32_t substeps)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(dt) || !(dt > 0.0f) || substeps < 1 ||
        substeps > M3_MAX_SUBSTEPS)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    world->contacts.stepVetoCount = 0;
    m3StepInternal(world, dt, substeps);
    if (world->recorder.journalActive != 0)
    {
        // Recording moved BEHIND the execution: nothing can
        // journal during a step, so callback-less streams are
        // byte-identical to the old order, and a step that vetoed
        // contacts writes those keys first. A bare replay (no
        // callback installed) then applies the recorded vetoes and
        // lands on the recorded bits: the tape is self-sufficient.
        if (world->contacts.stepVetoCount > 0)
        {
            m3JournalRecord(world, m3_opStepVetoes, world->contacts.stepVetoKeys,
                            world->contacts.stepVetoCount * (int32_t)sizeof(uint64_t));
        }
        m3OpStep record;
        memset(&record, 0, sizeof(record));
        record.dt = dt;
        record.substeps = substeps;
        m3JournalRecord(world, m3_opStep, &record, (int32_t)sizeof(record));
    }
}
