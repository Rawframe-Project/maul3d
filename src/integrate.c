// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Integration: the mover list, velocities and poses.

#include "integrate.h"

#include "body_state.h"
#include "solver_math.h"
#include "world_internal.h"

#include <math.h>

// Twice the rotational energy of omega under the inertia i, given by its
// six distinct entries: omega . (I omega).
static m3real TwiceEnergy(const m3real i[6], m3Vec3 omega)
{
    m3Vec3 io = {i[0] * omega.x + i[1] * omega.y + i[2] * omega.z,
                 i[1] * omega.x + i[3] * omega.y + i[4] * omega.z,
                 i[2] * omega.x + i[4] * omega.y + i[5] * omega.z};
    return m3Dot3(omega, io);
}

// One Newton iteration of I (w2 - w1) + h w2 x (I w2) = 0 from w2 = w1,
// in body coordinates, for the inertia's six distinct entries
// i00 i01 i02 i11 i12 i22, writing twice the start's energy,
// w1 . I w1, to start. Returns false when the residual leaves float
// range, for which the solve would take a zero step that reads as no
// change; its sum is checked, and one that overflows only sends the step
// to the next scale. The Jacobian's entries, the inertia plus h times the
// inertia times one spin component, stay in range for a body within the
// speed limit.
static bool NewtonPiece(const m3real i[6], m3Vec3* omegaInOut, m3real h, m3real* start)
{
    const m3Vec3 omega = *omegaInOut;
    const m3real i00 = i[0];
    const m3real i01 = i[1];
    const m3real i02 = i[2];
    const m3real i11 = i[3];
    const m3real i12 = i[4];
    const m3real i22 = i[5];
    // Residual b = I (w2 - w1) + h (w2 x I w2), Jacobian
    // J = I + h (skew(w2) I - skew(I w2)).
    const m3real w1 = omega.x;
    const m3real w2 = omega.y;
    const m3real w3 = omega.z;
    const m3real Iw1 = i00 * w1 + i01 * w2 + i02 * w3;
    const m3real Iw2 = i01 * w1 + i11 * w2 + i12 * w3;
    const m3real Iw3 = i02 * w1 + i12 * w2 + i22 * w3;
    // w2 - w1 is zero where the iteration starts, so the residual is just
    // the gyroscopic term.
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
    *start = w1 * Iw1 + w2 * Iw2 + w3 * Iw3;
    if (!m3FiniteF(b.x + b.y + b.z))
    {
        return false;
    }
    *omegaInOut = m3Sub3(omega, m3Solve3(&J, b));
    return true;
}

// A single Newton iteration that changes the spin by more than this
// fraction of it is redone in at least two pieces, up to
// M3_GYRO_MAX_PIECES, that each turn the spin at most M3_GYRO_PIECE_TURN
// radians.
#define M3_GYRO_REDO_CHANGE 0.5f
#define M3_GYRO_PIECE_TURN  0.5f
enum
{
    M3_GYRO_MAX_PIECES = 32
};
static_assert(M3_GYRO_MAX_PIECES >= 2, "a redone step takes at least two pieces");
// The energy the step may gain before it is scaled back: the two energies
// are computed in float, and below this they differ by their own
// rounding.
#define M3_GYRO_ENERGY_SLACK 0x1p-20f

// The gyroscopic term of Euler's equations, integrated implicitly. In
// body coordinates the step solves I (w2 - w1) + h w2 x (I w2) = 0 for
// w2. The exact solution never gains energy: w2 . I (w2 - w1) is zero, so
// w2 . I w2 is at most w1 . I w1. One Newton iteration from w1 is close
// to it while it changes the spin little, but it can gain energy, and
// near a singular Jacobian it overshoots by orders of magnitude: past
// about a radian of turn in the step, and for a rod or a plate, whose
// spin precesses many times faster than it turns, at far less. So an
// iteration that changes the spin by more than half of it is redone in
// pieces, and any energy the step still gains is scaled away, keeping the
// spin's direction.
bool m3GyroscopicStep(const m3Mat3* inertia, m3real scale, m3Quat q, m3Vec3 w, m3real h,
                      m3Vec3* out)
{
    const m3real i[6] = {inertia->cx.x / scale, inertia->cy.x / scale, inertia->cz.x / scale,
                         inertia->cy.y / scale, inertia->cz.y / scale, inertia->cz.z / scale};
    const m3Vec3 omega1 = m3InvRotateVec3(q, w);
    const m3real redo = M3_GYRO_REDO_CHANGE;
    m3Vec3 omega2 = omega1;
    m3real before = 0.0f;
    // One pass of one piece, and a second of at least two pieces when the
    // first changes the spin too much; the second always ends the loop.
    int32_t pieces = 1;
    for (;;)
    {
        const m3real piece = h / (m3real)pieces;
        omega2 = omega1;
        for (int32_t k = 0; k < pieces; ++k)
        {
            m3real start;
            if (!NewtonPiece(i, &omega2, piece, &start))
            {
                return false;
            }
            before = k == 0 ? start : before;
        }
        // Compared squared first, so an ordinary step takes no square
        // root.
        const m3Vec3 change = m3Sub3(omega2, omega1);
        const m3real spin2 = m3Dot3(omega1, omega1);
        if (pieces > 1 || !(m3Dot3(change, change) > redo * redo * spin2))
        {
            break;
        }
        // As many pieces as keep each one's turn within M3_GYRO_PIECE_TURN,
        // at least two, compared as a float first so that a count past the
        // int32 range takes the most.
        const m3real wanted = m3MaxF(2.0f, ceilf(h * sqrtf(spin2) / M3_GYRO_PIECE_TURN));
        pieces = wanted < (m3real)M3_GYRO_MAX_PIECES ? (int32_t)wanted : M3_GYRO_MAX_PIECES;
    }
    const m3real after = TwiceEnergy(i, omega2);
    // A spin or an energy past float range leaves the step to the next
    // scale.
    if (!m3FiniteF(before) || !m3FiniteF(after))
    {
        return false;
    }
    if (after > before * (1.0f + M3_GYRO_ENERGY_SLACK))
    {
        omega2 = m3MulSV3(sqrtf(before / after), omega2);
    }
    *out = m3RotateVec3(q, omega2);
    return true;
}

// A diagonal tensor with two equal moments is a symmetric top, which
// Euler's equations turn exactly: the spin along the third axis stays,
// and the spin across it turns about that axis at
// Omega = w_a (I_a - I_t) / I_t. The implicit step damps that turn, the
// precession of a tumbling rod, plate or square-section box, and with it
// most of the body's energy. The gate compares are exact, and the cosine
// and sine deterministic, so the branch is too. Returns false for any
// other tensor, which takes the implicit step.
static bool SymmetricTopTurn(const m3Mat3* inertia, m3Quat q, m3Vec3 w, m3real h, m3Vec3* out)
{
    if (inertia->cy.x != 0.0f || inertia->cz.x != 0.0f || inertia->cz.y != 0.0f)
    {
        return false;
    }
    m3Vec3 axis;
    m3real axial;
    m3real transverse;
    if (inertia->cy.y == inertia->cz.z)
    {
        axis = (m3Vec3){1.0f, 0.0f, 0.0f};
        axial = inertia->cx.x;
        transverse = inertia->cy.y;
    }
    else if (inertia->cx.x == inertia->cz.z)
    {
        axis = (m3Vec3){0.0f, 1.0f, 0.0f};
        axial = inertia->cy.y;
        transverse = inertia->cx.x;
    }
    else if (inertia->cx.x == inertia->cy.y)
    {
        axis = (m3Vec3){0.0f, 0.0f, 1.0f};
        axial = inertia->cz.z;
        transverse = inertia->cx.x;
    }
    else
    {
        return false;
    }
    // The turn is a rotation of w about the world axis e (Rodrigues), so
    // a zero turn leaves w bit for bit and no round trip through the body
    // frame rescales the spin each substep.
    const m3Vec3 e = m3RotateVec3(q, axis);
    const m3real turn = h * m3Dot3(e, w) * ((axial - transverse) / transverse);
    if (!m3FiniteF(turn))
    {
        return false;
    }
    // The half angle gives 1 - cos(turn) = 2 sin^2(turn / 2) to full
    // relative precision; 1 - cos rounds away most of its bits for a small
    // turn and grows the spin by a fraction of an ulp each substep.
    const m3CosSin half = m3ComputeCosSin(0.5f * turn);
    const m3real sine = 2.0f * half.s * half.c;
    const m3real versine = 2.0f * half.s * half.s;
    const m3Vec3 ew = m3Cross3(e, w);
    const m3Vec3 eew = m3Cross3(e, ew);
    m3Vec3 turned = m3Add3(w, m3Add3(m3MulSV3(sine, ew), m3MulSV3(versine, eew)));
    // A body's turn is the same every substep, and so is the rounding of
    // its sine and versine: left alone, it scales the spin the same way
    // each time. A first-order correction back to the incoming length
    // leaves only rounding that varies with the spin. It is applied as a
    // small delta, since a factor this close to 1 rounds finer below 1
    // than above and would scale the spin down each time instead.
    const m3real before = m3Dot3(w, w);
    const m3real after = m3Dot3(turned, turned);
    const m3real delta = 0.5f * (before - after) / after;
    if (m3FiniteF(delta))
    {
        turned = m3Add3(turned, m3MulSV3(delta, turned));
    }
    *out = turned;
    return true;
}

// An isotropic tensor skips the step: w x (c w) vanishes in real
// arithmetic but not bit for bit in float, and spheres keep their exact
// trajectories. A symmetric top turns exactly. The gate compares are
// exact, so the branches are deterministic.
static m3Vec3 Gyroscopic(const m3World* world, int32_t body, m3Vec3 w, m3real h)
{
    const m3Mat3* inertia = &world->bodies.inertiaLocal[body];
    const m3real i00 = inertia->cx.x;
    const m3real i11 = inertia->cy.y;
    const m3real i22 = inertia->cz.z;
    if (inertia->cy.x == 0.0f && inertia->cz.x == 0.0f && inertia->cz.y == 0.0f && i00 == i11 &&
        i11 == i22)
    {
        return w; // isotropic (or massless): the term vanishes
    }
    m3Quat q = world->bodies.transforms[body].q;
    m3Vec3 turned;
    if (SymmetricTopTurn(inertia, q, w, h, &turned))
    {
        return turned;
    }
    if (m3GyroscopicStep(inertia, 1.0f, q, w, h, &turned))
    {
        return turned;
    }
    // The Newton step works in w^2 times the inertia and its solve in the
    // cube of that, which a large body takes past float range. Every term
    // is linear in the inertia, so the step at unit inertia scale gives
    // the same spin. Should even that fail, the spin goes on unchanged, as
    // it does for a singular Jacobian.
    m3real scale = m3MaxF(i00, m3MaxF(i11, i22));
    if (scale > 0.0f && m3GyroscopicStep(inertia, scale, q, w, h, &turned))
    {
        return turned;
    }
    return w;
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
        m3Vec3 dw = m3MulSV3(h, w);
        world->bodies.transforms[i].q = m3IntegrateRotation(world->bodies.transforms[i].q, dw);
        m3Vec3 rlcNew = m3RotateVec3(world->bodies.transforms[i].q, lc);
        world->bodies.transforms[i].p.x = cx - (double)rlcNew.x;
        world->bodies.transforms[i].p.y = cy - (double)rlcNew.y;
        world->bodies.transforms[i].p.z = cz - (double)rlcNew.z;
        world->bodies.transforms[i].p = m3HoldPosition(world->bodies.transforms[i].p);
        deltaPos[i] = m3Add3(deltaPos[i], m3MulSV3(h, v));
        deltaRot[i] = m3IntegrateRotation(deltaRot[i], dw);
    }
}
