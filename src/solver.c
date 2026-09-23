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
#include "joint_solver.h"
#include "journal.h"
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

typedef struct m3ConstraintPoint
{
    m3Vec3 rA; // prepare-time anchors from each body's COM
    m3Vec3 rB;
    m3real baseSeparation; // separation minus dot(rB - rA, n) at prepare
    m3real normalMass;
    m3real leverArm;         // |rA - originA|: the twist budget arm
    m3real relativeVelocity; // vn at prepare, for the restitution pass
    m3real normalImpulse;
    m3real totalNormalImpulse; // summed across every solve pass of
                               // the step; the restitution gate
                               // reads it (a zero means the point
                               // never fired). The friction caps do
                               // NOT read it: they budget
                               // from the pass-local sum of live
                               // accumulators, the reference rule.
                               // The cross-pass sum inflated the
                               // cone up to 8x and a shoved crate
                               // stuck, dug its edge, and hopped.
} m3ConstraintPoint;

typedef struct m3ContactConstraint
{
    int32_t bodyA;
    int32_t bodyB;
    int32_t manifoldIndex;
    int32_t pointCount;
    m3Vec3 normal;
    m3Vec3 t1;
    m3Vec3 t2;
    // Central friction (rev 21, the reference layout): one 2x2 row
    // at the mean anchors plus a twist row about the normal, instead
    // of per-point tangent rows. Per-corner friction on a box gave
    // gravity a fake pitch lever (the vehicle arc's hub lesson, at
    // the contact level).
    m3Vec3 originA;
    m3Vec3 originB;
    m3real frictionK11; // inverted 2x2 tangent mass, symmetric
    m3real frictionK12;
    m3real frictionK22;
    m3real frictionImpulse1;
    m3real frictionImpulse2;
    m3real twistMass;
    m3real twistImpulse;
    m3real tangentVelocity1; // conveyor target, reference field
    m3real tangentVelocity2;
    m3real friction;
    m3real restitution;
    m3real invMassA;
    m3real invMassB;
    m3Mat3 invIA; // world-space inverse inertia, frozen at prepare
    m3Mat3 invIB;
    m3Softness softness;
    m3real rollingResistance; // max-mixed, extent-scaled
    m3Vec3 rollingImpulse;    // warm across steps via the manifold
    m3Mat3 rollingK;          // iA + iB, solved per relax iteration
    m3ConstraintPoint points[M3_MANIFOLD_MAX_POINTS];
} m3ContactConstraint;

static m3Vec3 VelocityAt(const m3World* world, int32_t body, m3Vec3 arm)
{
    m3Vec3 v = world->linearVelocities[body];
    m3Vec3 w = world->angularVelocities[body];
    return m3Add3(v, m3Cross3(w, arm));
}

static void ApplyImpulse(m3World* world, const m3ContactConstraint* c, m3Vec3 impulse, m3Vec3 rsA,
                         m3Vec3 rsB)
{
    if (world->types[c->bodyA] == (uint8_t)m3_dynamicBody)
    {
        world->linearVelocities[c->bodyA] =
            m3Sub3(world->linearVelocities[c->bodyA], m3MulSV3(c->invMassA, impulse));
        world->angularVelocities[c->bodyA] =
            m3Sub3(world->angularVelocities[c->bodyA], m3MulMV3(c->invIA, m3Cross3(rsA, impulse)));
    }
    if (world->types[c->bodyB] == (uint8_t)m3_dynamicBody)
    {
        world->linearVelocities[c->bodyB] =
            m3Add3(world->linearVelocities[c->bodyB], m3MulSV3(c->invMassB, impulse));
        world->angularVelocities[c->bodyB] =
            m3Add3(world->angularVelocities[c->bodyB], m3MulMV3(c->invIB, m3Cross3(rsB, impulse)));
    }
}

// Effective mass along one direction for one contact point's arms.
static m3real EffectiveMass(const m3ContactConstraint* c, m3Vec3 rA, m3Vec3 rB, m3Vec3 dir)
{
    m3Vec3 arm1 = m3Cross3(rA, dir);
    m3Vec3 arm2 = m3Cross3(rB, dir);
    m3real k = c->invMassA + c->invMassB + m3Dot3(arm1, m3MulMV3(c->invIA, arm1)) +
               m3Dot3(arm2, m3MulMV3(c->invIB, arm2));
    return k > 0.0f ? 1.0f / k : 0.0f;
}
// I_w^-1 = R I_l^-1 R^T, built by applying the operator to the world
// basis vectors. Frozen at prepare like the anchors (reference
// discipline); the per-substep refresh arrives with the gyroscopic
// slice.
m3Mat3 m3WorldInvInertia(const m3World* world, int32_t body)
{
    if (world->types[body] != (uint8_t)m3_dynamicBody)
    {
        return m3MakeZeroMat3();
    }
    m3Quat q = world->transforms[body].q;
    m3Mat3 il = world->invInertiaLocal[body];
    m3Mat3 r;
    r.cx = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f})));
    r.cy = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 1.0f, 0.0f})));
    r.cz = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 0.0f, 1.0f})));
    return r;
}

static int32_t PrepareContacts(m3World* world, m3ContactConstraint* constraints, m3real h)
{
    m3Softness soft = m3MakeSoft(world->contactHertz, world->contactDampingRatio, h);
    m3Softness staticSoft = m3MakeSoft(2.0f * world->contactHertz, world->contactDampingRatio, h);

    int32_t count = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        const m3Manifold* manifold = &world->manifolds[i];
        if (manifold->pointCount == 0)
        {
            continue;
        }
        uint64_t key = world->pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        int32_t bodyA = world->shapeBody[shapeA];
        int32_t bodyB = world->shapeBody[shapeB];
        int awakeDynA = world->types[bodyA] == (uint8_t)m3_dynamicBody && world->awake[bodyA] != 0;
        int awakeDynB = world->types[bodyB] == (uint8_t)m3_dynamicBody && world->awake[bodyB] != 0;
        if (!awakeDynA && !awakeDynB)
        {
            continue; // both sides frozen or immovable: impulses stay put
        }
        if (world->shapeSensor[shapeA] != 0 || world->shapeSensor[shapeB] != 0)
        {
            continue; // sensors detect, they never respond
        }
        if (world->replayVetoCount > 0)
        {
            // A recorded veto set owns this step: the keys are
            // canonical ascending, the pairs arrive in the same
            // order, and the callback (if any) stays silent so the
            // tape's truth cannot be second-guessed.
            int32_t lo = 0;
            int32_t hi = world->replayVetoCount - 1;
            int vetoed = 0;
            while (lo <= hi)
            {
                int32_t mid = (lo + hi) / 2;
                if (world->replayVetoKeys[mid] == key)
                {
                    vetoed = 1;
                    break;
                }
                if (world->replayVetoKeys[mid] < key)
                {
                    lo = mid + 1;
                }
                else
                {
                    hi = mid - 1;
                }
            }
            if (vetoed)
            {
                continue;
            }
        }
        else if (world->preSolveFn != NULL &&
                 (world->shapePreSolve[shapeA] != 0 || world->shapePreSolve[shapeB] != 0))
        {
            // The pre-solve veto: serial, canonical pair order.
            // The LOUD contract lives on the API: the callback must
            // be pure, or replay divergence is the host's own.
            int32_t deep = 0;
            for (int32_t k = 1; k < manifold->pointCount; ++k)
            {
                if (manifold->points[k].separation < manifold->points[deep].separation)
                {
                    deep = k;
                }
            }
            m3Vec3 lcB = m3RotateVec3(world->transforms[bodyB].q, world->localCenters[bodyB]);
            m3Pos3 point;
            point.x = world->transforms[bodyB].p.x + (double)lcB.x +
                      (double)manifold->points[deep].anchorB.x;
            point.y = world->transforms[bodyB].p.y + (double)lcB.y +
                      (double)manifold->points[deep].anchorB.y;
            point.z = world->transforms[bodyB].p.z + (double)lcB.z +
                      (double)manifold->points[deep].anchorB.z;
            m3ShapeId idA = {shapeA + 1, world->worldIndex0, world->shapePool.generations[shapeA]};
            m3ShapeId idB = {shapeB + 1, world->worldIndex0, world->shapePool.generations[shapeB]};
            if (!world->preSolveFn(idA, idB, point, manifold->normal, world->preSolveContext))
            {
                // Vetoed: no constraint this step. The key joins the
                // journal annex so a bare replay repeats the
                // decision; collection order = pair order = sorted.
                if (world->journalActive != 0 && world->stepVetoCount < world->pairCapacity)
                {
                    world->stepVetoKeys[world->stepVetoCount++] = key;
                }
                continue;
            }
        }

        m3ContactConstraint* c = constraints + count;
        count += 1;
        memset(c, 0, sizeof(*c));
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        c->manifoldIndex = i;
        c->pointCount = manifold->pointCount;
        c->normal = manifold->normal;
        m3MakeTangentBasis(c->normal, &c->t1, &c->t2);
        c->invMassA = world->types[bodyA] == (uint8_t)m3_dynamicBody ? world->invMass[bodyA] : 0.0f;
        c->invMassB = world->types[bodyB] == (uint8_t)m3_dynamicBody ? world->invMass[bodyB] : 0.0f;
        c->invIA = m3WorldInvInertia(world, bodyA);
        c->invIB = m3WorldInvInertia(world, bodyB);
        // Reference mixing: friction geometric, restitution maximum,
        // rolling resistance maximum scaled by the pair's extent
        // (the lever that turns the dimensionless knob into torque).
        // A painted mesh swaps ITS side of the mix for the
        // struck triangle's entry; the group index rides the first
        // point's flags (points are id-canonical, so the pick is
        // deterministic), and the whole manifold wears one material
        // (the welded points share a face by construction).
        float fricA = world->shapeFriction[shapeA];
        float fricB = world->shapeFriction[shapeB];
        float restA = world->shapeRestitution[shapeA];
        float restB = world->shapeRestitution[shapeB];
        float rollA = world->shapeRollingResistance[shapeA];
        float rollB = world->shapeRollingResistance[shapeB];
        m3Vec3 surfA = world->shapeSurfaceVel[shapeA];
        m3Vec3 surfB = world->shapeSurfaceVel[shapeB];
        if (world->shapeType[shapeA] == (uint8_t)m3_meshShape)
        {
            const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shapeA]];
            if (mesh->materialCount > 0)
            {
                // Group 0 catches out-of-range bits from a hostile
                // restored manifold: clamped, deterministic, in
                // bounds.
                int32_t mi = manifold->points[0].flags >> 12;
                const m3MeshSurfaceMaterial* m =
                    &mesh->materials[mi < mesh->materialCount ? mi : 0];
                fricA = m->friction;
                restA = m->restitution;
                rollA = m->rollingResistance;
                surfA = m->surfaceVelocity;
            }
        }
        if (world->shapeType[shapeB] == (uint8_t)m3_meshShape)
        {
            const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shapeB]];
            if (mesh->materialCount > 0)
            {
                int32_t mi = manifold->points[0].flags >> 12;
                const m3MeshSurfaceMaterial* m =
                    &mesh->materials[mi < mesh->materialCount ? mi : 0];
                fricB = m->friction;
                restB = m->restitution;
                rollB = m->rollingResistance;
                surfB = m->surfaceVelocity;
            }
        }
        c->friction = sqrtf(fricA * fricB);
        c->restitution = m3MaxF(restA, restB);
        c->rollingResistance =
            m3MaxF(rollA, rollB) * m3MaxF(world->maxExtents[bodyA], world->maxExtents[bodyB]);
        c->rollingImpulse = manifold->rollingImpulse; // reference warm start
        if (c->rollingResistance > 0.0f)
        {
            m3Mat3 sum = c->invIA;
            sum.cx = m3Add3(sum.cx, c->invIB.cx);
            sum.cy = m3Add3(sum.cy, c->invIB.cy);
            sum.cz = m3Add3(sum.cz, c->invIB.cz);
            c->rollingK = sum;
        }
        c->softness = (c->invMassA == 0.0f || c->invMassB == 0.0f) ? staticSoft : soft;

        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m3ConstraintPoint* cp = &c->points[k];
            cp->rA = manifold->points[k].anchorA;
            cp->rB = manifold->points[k].anchorB;
            cp->baseSeparation = manifold->points[k].separation -
                                 (m3Dot3(cp->rB, c->normal) - m3Dot3(cp->rA, c->normal));
            cp->normalMass = EffectiveMass(c, cp->rA, cp->rB, c->normal);
            cp->relativeVelocity =
                m3Dot3(m3Sub3(VelocityAt(world, bodyB, cp->rB), VelocityAt(world, bodyA, cp->rA)),
                       c->normal);
            cp->normalImpulse = manifold->points[k].normalImpulse;
            cp->totalNormalImpulse = 0.0f;
        }

        // Central friction preparation: mean anchors, per-point
        // lever arms for the twist budget, the coupled 2x2 tangent
        // mass inverted once, and warm impulses read from the
        // manifold's WORLD-frame storage so a rotated tangent basis
        // cannot corrupt the carry.
        m3Vec3 originA = {0.0f, 0.0f, 0.0f};
        m3Vec3 originB = {0.0f, 0.0f, 0.0f};
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            originA = m3Add3(originA, c->points[k].rA);
            originB = m3Add3(originB, c->points[k].rB);
        }
        m3real invCount = 1.0f / (m3real)c->pointCount;
        c->originA = m3MulSV3(invCount, originA);
        c->originB = m3MulSV3(invCount, originB);
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            c->points[k].leverArm = m3Length3(m3Sub3(c->points[k].rA, c->originA));
        }

        m3Vec3 rtA1 = m3Cross3(c->originA, c->t1);
        m3Vec3 rtA2 = m3Cross3(c->originA, c->t2);
        m3Vec3 rtB1 = m3Cross3(c->originB, c->t1);
        m3Vec3 rtB2 = m3Cross3(c->originB, c->t2);
        m3real k11 = c->invMassA + c->invMassB + m3Dot3(rtA1, m3MulMV3(c->invIA, rtA1)) +
                     m3Dot3(rtB1, m3MulMV3(c->invIB, rtB1));
        m3real k22 = c->invMassA + c->invMassB + m3Dot3(rtA2, m3MulMV3(c->invIA, rtA2)) +
                     m3Dot3(rtB2, m3MulMV3(c->invIB, rtB2));
        m3real k12 =
            m3Dot3(rtA1, m3MulMV3(c->invIA, rtA2)) + m3Dot3(rtB1, m3MulMV3(c->invIB, rtB2));
        m3real det = k11 * k22 - k12 * k12;
        if (det > 0.0f)
        {
            m3real invDet = 1.0f / det;
            c->frictionK11 = k22 * invDet;
            c->frictionK22 = k11 * invDet;
            c->frictionK12 = -k12 * invDet;
        }
        else
        {
            c->frictionK11 = 0.0f;
            c->frictionK22 = 0.0f;
            c->frictionK12 = 0.0f;
        }
        c->frictionImpulse1 = m3Dot3(manifold->frictionImpulse, c->t1);
        c->frictionImpulse2 = m3Dot3(manifold->frictionImpulse, c->t2);
        // Conveyor targets: the reference tangentVelocity,
        // finally fed. Sign law: friction drives the pair's B-minus-A
        // tangential speed TOWARD this target, so a belt at shape A
        // carries the other body along its surface velocity.
        m3Vec3 surf = m3Sub3(surfA, surfB);
        c->tangentVelocity1 = m3Dot3(surf, c->t1);
        c->tangentVelocity2 = m3Dot3(surf, c->t2);

        m3Vec3 iSumN = m3Add3(m3MulMV3(c->invIA, c->normal), m3MulMV3(c->invIB, c->normal));
        m3real twistK = m3Dot3(c->normal, iSumN);
        c->twistMass = twistK > 0.0f ? 1.0f / twistK : 0.0f;
        c->twistImpulse = manifold->twistImpulse;
    }
    // The pending recorded vetoes applied to exactly this prepare;
    // consume them so the next step decides for itself.
    world->replayVetoCount = 0;
    return count;
}

static void WarmStartOne(m3World* world, m3ContactConstraint* c)
{
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        m3ConstraintPoint* cp = &c->points[k];
        ApplyImpulse(world, c, m3MulSV3(cp->normalImpulse, c->normal), cp->rA, cp->rB);
    }
    m3Vec3 f = m3Add3(m3MulSV3(c->frictionImpulse1, c->t1), m3MulSV3(c->frictionImpulse2, c->t2));
    ApplyImpulse(world, c, f, c->originA, c->originB);
    // Twist and rolling are pure angular rows.
    m3Vec3 tw = m3Add3(m3MulSV3(c->twistImpulse, c->normal), c->rollingImpulse);
    if (world->types[c->bodyA] == (uint8_t)m3_dynamicBody)
    {
        world->angularVelocities[c->bodyA] =
            m3Sub3(world->angularVelocities[c->bodyA], m3MulMV3(c->invIA, tw));
    }
    if (world->types[c->bodyB] == (uint8_t)m3_dynamicBody)
    {
        world->angularVelocities[c->bodyB] =
            m3Add3(world->angularVelocities[c->bodyB], m3MulMV3(c->invIB, tw));
    }
}
static void SolveOneContact(m3World* world, m3ContactConstraint* c, const m3Vec3* deltaPos,
                            const m3Quat* deltaRot, m3real invH, bool useBias)
{
    {

        // Normal rows first, then friction rows, per the reference
        // schedule. The Jacobian keeps the fixed prepare-time anchors;
        // rotated anchors only measure the separation drift. The
        // friction budgets are PASS-LOCAL: the sum of live
        // normal accumulators in this pass, not a cross-pass total.
        m3real passNormal = 0.0f;
        m3real twistLimit = 0.0f;
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m3ConstraintPoint* cp = &c->points[k];
            m3Vec3 rsA = m3RotateVec3(deltaRot[c->bodyA], cp->rA);
            m3Vec3 rsB = m3RotateVec3(deltaRot[c->bodyB], cp->rB);
            m3Vec3 ds = m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), m3Sub3(rsB, rsA));
            m3real s = cp->baseSeparation + m3Dot3(ds, c->normal);

            m3real bias = 0.0f;
            m3real massScale = 1.0f;
            m3real impulseScale = 0.0f;
            if (s > 0.0f)
            {
                bias = s * invH; // speculative: prevent crossing
            }
            else if (useBias)
            {
                bias = m3MaxF(c->softness.biasRate * s, -world->contactPushMaxSpeed);
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }

            m3real vn = m3Dot3(
                m3Sub3(VelocityAt(world, c->bodyB, cp->rB), VelocityAt(world, c->bodyA, cp->rA)),
                c->normal);
            m3real impulse =
                -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
            m3real newImpulse = m3MaxF(cp->normalImpulse + impulse, 0.0f);
            impulse = newImpulse - cp->normalImpulse;
            cp->normalImpulse = newImpulse;
            cp->totalNormalImpulse += newImpulse; // the restitution gate
            passNormal += newImpulse;
            twistLimit += cp->leverArm * newImpulse;
            ApplyImpulse(world, c, m3MulSV3(impulse, c->normal), cp->rA, cp->rB);
        }

        // No friction while applying bias: the reference schedule.
        // Bias motion is virtual (the relax pass exists to remove
        // it); friction solved against it reads push-out as real
        // sliding.
        if (useBias)
        {
            return;
        }

        // Central twist friction: brakes spin about the contact
        // normal, budgeted by the lever-arm-weighted normal sum
        // (a single-point manifold has no twist authority).
        {
            m3Vec3 wRel =
                m3Sub3(world->angularVelocities[c->bodyB], world->angularVelocities[c->bodyA]);
            m3real twistSpeed = m3Dot3(c->normal, wRel);
            m3real maxTwist = c->friction * twistLimit;
            m3real delta = -c->twistMass * twistSpeed;
            m3real oldTwist = c->twistImpulse;
            m3real newTwist = oldTwist + delta;
            if (newTwist < -maxTwist)
            {
                newTwist = -maxTwist;
            }
            else if (newTwist > maxTwist)
            {
                newTwist = maxTwist;
            }
            delta = newTwist - oldTwist;
            c->twistImpulse = newTwist;
            m3Vec3 tw = m3MulSV3(delta, c->normal);
            if (world->types[c->bodyA] == (uint8_t)m3_dynamicBody)
            {
                world->angularVelocities[c->bodyA] =
                    m3Sub3(world->angularVelocities[c->bodyA], m3MulMV3(c->invIA, tw));
            }
            if (world->types[c->bodyB] == (uint8_t)m3_dynamicBody)
            {
                world->angularVelocities[c->bodyB] =
                    m3Add3(world->angularVelocities[c->bodyB], m3MulMV3(c->invIB, tw));
            }
        }

        // Rolling resistance (6-3, aligned fully in rev 21): a pure
        // angular row braking relative rotation, capped by the
        // pass-local normal budget, warm across steps through the
        // manifold like the reference. The 6-3 cold start only
        // held up because the inflated cross-pass budget hid it.
        // Without this row a sphere pile never stops rolling and
        // never sleeps.
        if (c->rollingResistance > 0.0f)
        {
            m3Vec3 wRel =
                m3Sub3(world->angularVelocities[c->bodyB], world->angularVelocities[c->bodyA]);
            m3Vec3 delta = m3MulSV3(-1.0f, m3Solve3(&c->rollingK, wRel));
            m3Vec3 accum = m3Add3(c->rollingImpulse, delta);
            m3real maxRoll = c->rollingResistance * passNormal; // pass-local
            m3real mag2 = m3Dot3(accum, accum);
            if (mag2 > maxRoll * maxRoll && mag2 > 0.0f)
            {
                accum = m3MulSV3(maxRoll / sqrtf(mag2), accum);
            }
            delta = m3Sub3(accum, c->rollingImpulse);
            c->rollingImpulse = accum;
            // Guarded like every contact write-back: a static body shared
            // across graph colors must never be written, even with an
            // unchanged value (concurrent identical writes still race).
            if (world->types[c->bodyA] == (uint8_t)m3_dynamicBody)
            {
                world->angularVelocities[c->bodyA] =
                    m3Sub3(world->angularVelocities[c->bodyA], m3MulMV3(c->invIA, delta));
            }
            if (world->types[c->bodyB] == (uint8_t)m3_dynamicBody)
            {
                world->angularVelocities[c->bodyB] =
                    m3Add3(world->angularVelocities[c->bodyB], m3MulMV3(c->invIB, delta));
            }
        }

        // Central friction: one coupled 2x2 row at the mean anchors,
        // clamped to the pass-local Coulomb circle.
        {
            m3Vec3 vrel = m3Sub3(VelocityAt(world, c->bodyB, c->originB),
                                 VelocityAt(world, c->bodyA, c->originA));
            m3real vt1 = m3Dot3(vrel, c->t1) - c->tangentVelocity1;
            m3real vt2 = m3Dot3(vrel, c->t2) - c->tangentVelocity2;
            m3real f1 = c->frictionImpulse1 - (c->frictionK11 * vt1 + c->frictionK12 * vt2);
            m3real f2 = c->frictionImpulse2 - (c->frictionK12 * vt1 + c->frictionK22 * vt2);
            m3real maxFriction = c->friction * passNormal;
            m3real mag2 = f1 * f1 + f2 * f2;
            if (mag2 > maxFriction * maxFriction)
            {
                m3real mag = sqrtf(mag2);
                m3real scale = mag > 0.0f ? maxFriction / mag : 0.0f;
                f1 *= scale;
                f2 *= scale;
            }
            m3real d1 = f1 - c->frictionImpulse1;
            m3real d2 = f2 - c->frictionImpulse2;
            c->frictionImpulse1 = f1;
            c->frictionImpulse2 = f2;
            ApplyImpulse(world, c, m3Add3(m3MulSV3(d1, c->t1), m3MulSV3(d2, c->t2)), c->originA,
                         c->originB);
        }
    }
}

// ---------------------------------------------------------------
// Joints: the spherical point constraint in soft-step form,
// the reference schedule (joints warm start and solve BEFORE the
// contacts inside every substep pass). Few joints, serial in
// canonical index order; they join the color palette when counts
// ever justify it (noted, not needed for correctness).
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// Graph coloring: constraints in one color share no awake
// dynamic body, so any schedule inside a color writes disjoint
// velocities and the bits cannot move. The greedy walk runs in
// canonical constraint order with first-free-bit colors; whatever
// cannot color inside the palette lands in the overflow bucket and
// solves serially. Wide 4-lane batching joins the profiling era
// (recorded in the plan); the parallel structure lands here.
// ---------------------------------------------------------------
#define M3_GRAPH_COLORS 16

typedef struct m3SolverColoring
{
    uint8_t* colors; // per constraint
    int32_t* lists;  // constraint indices grouped by color
    int32_t starts[M3_GRAPH_COLORS + 2];
} m3SolverColoring;

static int BuildColoring(m3World* world, m3ContactConstraint* constraints, int32_t count,
                         m3SolverColoring* out)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    out->colors = (uint8_t*)m3StackAlloc(&world->scratch, count > 0 ? count : 1);
    uint32_t* bodyMasks = (uint32_t*)m3StackAlloc(
        &world->scratch, maxBody > 0 ? maxBody * (int32_t)sizeof(uint32_t) : 4);
    out->lists =
        (int32_t*)m3StackAlloc(&world->scratch, count > 0 ? count * (int32_t)sizeof(int32_t) : 4);
    if (out->colors == NULL || bodyMasks == NULL || out->lists == NULL)
    {
        return 0;
    }
    memset(bodyMasks, 0, (size_t)(maxBody > 0 ? maxBody : 1) * sizeof(uint32_t));

    int32_t counts[M3_GRAPH_COLORS + 1];
    memset(counts, 0, sizeof(counts));
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = &constraints[i];
        int dynA = world->types[c->bodyA] == (uint8_t)m3_dynamicBody && world->awake[c->bodyA];
        int dynB = world->types[c->bodyB] == (uint8_t)m3_dynamicBody && world->awake[c->bodyB];
        uint32_t mask = (dynA ? bodyMasks[c->bodyA] : 0u) | (dynB ? bodyMasks[c->bodyB] : 0u);
        int32_t color = M3_GRAPH_COLORS; // overflow unless a bit frees up
        for (int32_t bit = 0; bit < M3_GRAPH_COLORS; ++bit)
        {
            if ((mask & (1u << bit)) == 0u)
            {
                color = bit;
                break;
            }
        }
        out->colors[i] = (uint8_t)color;
        if (color < M3_GRAPH_COLORS)
        {
            if (dynA)
            {
                bodyMasks[c->bodyA] |= 1u << color;
            }
            if (dynB)
            {
                bodyMasks[c->bodyB] |= 1u << color;
            }
        }
        counts[color] += 1;
    }
    int32_t cursor = 0;
    for (int32_t c = 0; c <= M3_GRAPH_COLORS; ++c)
    {
        out->starts[c] = cursor;
        cursor += counts[c];
    }
    out->starts[M3_GRAPH_COLORS + 1] = cursor;
    int32_t fill[M3_GRAPH_COLORS + 1];
    memcpy(fill, out->starts, sizeof(fill));
    for (int32_t i = 0; i < count; ++i)
    {
        out->lists[fill[out->colors[i]]++] = i; // ascending inside a color
    }
    return 1;
}

typedef struct m3SolveTaskContext
{
    m3World* world;
    m3ContactConstraint* constraints;
    const int32_t* list;
    const m3Vec3* deltaPos;
    const m3Quat* deltaRot;
    m3real invH;
    int useBias;
    int warmStartOnly;
} m3SolveTaskContext;

static void SolveRangeTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
    m3SolveTaskContext* ctx = (m3SolveTaskContext*)taskContext;
    for (int32_t k = startIndex; k < endIndex; ++k)
    {
        m3ContactConstraint* c = &ctx->constraints[ctx->list[k]];
        if (ctx->warmStartOnly)
        {
            WarmStartOne(ctx->world, c);
        }
        else
        {
            SolveOneContact(ctx->world, c, ctx->deltaPos, ctx->deltaRot, ctx->invH,
                            ctx->useBias != 0);
        }
    }
}

// Colors run in order with a barrier between them; inside a color
// the host may split any way it likes (disjoint bodies), and the
// overflow bucket always runs serial (its members may conflict).
static void RunColored(m3World* world, const m3SolverColoring* coloring,
                       m3ContactConstraint* constraints, const m3Vec3* deltaPos,
                       const m3Quat* deltaRot, m3real invH, int useBias, int warmStartOnly)
{
    for (int32_t color = 0; color <= M3_GRAPH_COLORS; ++color)
    {
        int32_t start = coloring->starts[color];
        int32_t end = coloring->starts[color + 1];
        int32_t size = end - start;
        if (size == 0)
        {
            continue;
        }
        m3SolveTaskContext ctx;
        ctx.world = world;
        ctx.constraints = constraints;
        ctx.list = coloring->lists + start;
        ctx.deltaPos = deltaPos;
        ctx.deltaRot = deltaRot;
        ctx.invH = invH;
        ctx.useBias = useBias;
        ctx.warmStartOnly = warmStartOnly;
        if (world->enqueueTask != NULL && size >= 16 && color < M3_GRAPH_COLORS)
        {
            void* task = world->enqueueTask(SolveRangeTask, size, 8, &ctx, world->userTaskContext);
            world->finishTask(task, world->userTaskContext);
        }
        else
        {
            SolveRangeTask(0, size, &ctx);
        }
    }
}

static void Restitution(m3World* world, m3ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = constraints + i;
        if (c->restitution == 0.0f)
        {
            continue;
        }
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m3ConstraintPoint* cp = &c->points[k];
            if (cp->relativeVelocity > -world->restitutionThreshold ||
                cp->totalNormalImpulse == 0.0f)
            {
                continue;
            }
            m3real vn = m3Dot3(
                m3Sub3(VelocityAt(world, c->bodyB, cp->rB), VelocityAt(world, c->bodyA, cp->rA)),
                c->normal);
            m3real impulse = -cp->normalMass * (vn + c->restitution * cp->relativeVelocity);
            m3real newImpulse = m3MaxF(cp->normalImpulse + impulse, 0.0f);
            impulse = newImpulse - cp->normalImpulse;
            cp->normalImpulse = newImpulse;
            ApplyImpulse(world, c, m3MulSV3(impulse, c->normal), cp->rA, cp->rB);
        }
    }
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
    const m3Mat3* inertia = &world->inertiaLocal[body];
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

    m3Quat q = world->transforms[body].q;
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

static void StoreImpulses(m3World* world, m3ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = constraints + i;
        m3Manifold* manifold = &world->manifolds[c->manifoldIndex];
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            manifold->points[k].normalImpulse = c->points[k].normalImpulse;
        }

        // Hit events (8-5, the reference recipe): at most one per
        // contact, for the fastest-approaching point, only when a
        // side opted in and the impact actually fired.
        uint64_t key = world->pairKeys[c->manifoldIndex];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if (world->shapeHitEvents[shapeA] != 0 || world->shapeHitEvents[shapeB] != 0)
        {
            int32_t best = -1;
            for (int32_t k = 0; k < c->pointCount; ++k)
            {
                if (c->points[k].relativeVelocity < -world->hitEventThreshold &&
                    c->points[k].totalNormalImpulse > 0.0f &&
                    (best < 0 || c->points[k].relativeVelocity < c->points[best].relativeVelocity))
                {
                    best = k;
                }
            }
            if (best >= 0)
            {
                if (world->hitEventCount < world->pairCapacity)
                {
                    m3HitEvent* e = &world->hitEvents[world->hitEventCount++];
                    e->shapeA = (m3ShapeId){shapeA + 1, world->worldIndex0,
                                            world->shapePool.generations[shapeA]};
                    e->shapeB = (m3ShapeId){shapeB + 1, world->worldIndex0,
                                            world->shapePool.generations[shapeB]};
                    m3Vec3 lcA =
                        m3RotateVec3(world->transforms[c->bodyA].q, world->localCenters[c->bodyA]);
                    e->point.x = world->transforms[c->bodyA].p.x + (double)lcA.x +
                                 (double)c->points[best].rA.x;
                    e->point.y = world->transforms[c->bodyA].p.y + (double)lcA.y +
                                 (double)c->points[best].rA.y;
                    e->point.z = world->transforms[c->bodyA].p.z + (double)lcA.z +
                                 (double)c->points[best].rA.z;
                    e->normal = c->normal;
                    e->approachSpeed = -c->points[best].relativeVelocity;
                }
                else
                {
                    world->hitEventsDropped += 1;
                }
            }
        }
        manifold->frictionImpulse =
            m3Add3(m3MulSV3(c->frictionImpulse1, c->t1), m3MulSV3(c->frictionImpulse2, c->t2));
        manifold->twistImpulse = c->twistImpulse;
        manifold->rollingImpulse = c->rollingImpulse;
    }
}
// ---------------------------------------------------------------
// Continuous collision, modeled on the reference
// b3SolveContinuous: any fast dynamic body sweeps against statics;
// a bullet additionally sweeps against non-bullet dynamics with the
// TARGET'S true sweep in the TOI (dynamic versus dynamic moves both
// bodies). A hit pulls the fast body back to the impact pose;
// velocity stays, and next step's speculative contact resolves the
// impact. Bullet versus bullet is not resolved (the reference
// limitation, kept and documented). Bodies are processed in
// ascending index order; later bodies see earlier pull-backs, all
// deterministic.
// ---------------------------------------------------------------

typedef struct m3ContinuousContext
{
    m3World* world;
    const m3Pos3* com0;
    const m3Quat* rot0;
    int32_t fastBody;
    int32_t fastShape;
    m3Pos3 base; // re-center: TOI floats stay small
    m3Sweep fastSweep;
    m3real fraction;
} m3ContinuousContext;

static m3Sweep MakeRelativeSweep(const m3World* world, int32_t body, const m3Pos3* com0,
                                 const m3Quat* rot0, m3Pos3 base)
{
    m3Sweep sweep;
    sweep.localCenter = world->localCenters[body];
    sweep.c1 = (m3Vec3){(m3real)(com0[body].x - base.x), (m3real)(com0[body].y - base.y),
                        (m3real)(com0[body].z - base.z)};
    m3Vec3 rlc = m3RotateVec3(world->transforms[body].q, world->localCenters[body]);
    sweep.c2 = (m3Vec3){(m3real)(world->transforms[body].p.x + (double)rlc.x - base.x),
                        (m3real)(world->transforms[body].p.y + (double)rlc.y - base.y),
                        (m3real)(world->transforms[body].p.z + (double)rlc.z - base.z)};
    sweep.q1 = rot0[body];
    sweep.q2 = world->transforms[body].q;
    return sweep;
}

static bool ContinuousQueryCallback(int32_t shape, void* userContext)
{
    m3ContinuousContext* ctx = (m3ContinuousContext*)userContext;
    m3World* world = ctx->world;
    if (shape == ctx->fastShape)
    {
        return true;
    }
    int32_t body = world->shapeBody[shape];
    if (body == ctx->fastBody)
    {
        return true;
    }
    if (world->bodyEnabled[body] == 0)
    {
        return true; // disabled bodies never block the fast mover
    }
    if (world->bulletFlags[body] != 0)
    {
        return true; // bullet versus bullet: skip (documented)
    }
    if (world->shapeSensor[shape] != 0)
    {
        return true; // sensors never stop anything
    }
    if (world->shapeType[shape] == (uint8_t)m3_voxelShape)
    {
        // Voxel TOI: sweep the fast shape against candidate
        // merged boxes in the CHUNK frame (the correct-frame
        // witnesses the architecture demands). Boxes are UNEXTENDED
        // here on purpose: the seam extension is a contact-only
        // device (coverage guarantees exactly one flush layer, and
        // a bullet must not stop against phantom solid a chunk
        // length behind a thin welded wall). A seam-grazing sweep
        // at worst stops a hair early and hands the rest of the
        // step to the welded contact solver.
        int32_t slot = world->shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxelSurface[slot];
        m3real cell = world->voxelData[slot].cellSize;
        m3Sweep chunkSweep = MakeRelativeSweep(world, body, ctx->com0, ctx->rot0, ctx->base);
        m3Vec3 scratchFast[2];
        m3DistanceProxy fastProxy = m3MakeShapeProxy(world, ctx->fastShape, scratchFast);

        const m3Transform* xfV = &world->transforms[body];
        m3Vec3 c1 =
            m3InvRotateVec3(xfV->q, (m3Vec3){(m3real)(ctx->com0[ctx->fastBody].x - xfV->p.x),
                                             (m3real)(ctx->com0[ctx->fastBody].y - xfV->p.y),
                                             (m3real)(ctx->com0[ctx->fastBody].z - xfV->p.z)});
        m3Vec3 rlc =
            m3RotateVec3(world->transforms[ctx->fastBody].q, world->localCenters[ctx->fastBody]);
        m3Vec3 c2 = m3InvRotateVec3(
            xfV->q,
            (m3Vec3){(m3real)(world->transforms[ctx->fastBody].p.x + (double)rlc.x - xfV->p.x),
                     (m3real)(world->transforms[ctx->fastBody].p.y + (double)rlc.y - xfV->p.y),
                     (m3real)(world->transforms[ctx->fastBody].p.z + (double)rlc.z - xfV->p.z)});
        m3real pad = world->maxExtents[ctx->fastBody] + M3_AABB_MARGIN;
        m3Vec3 lo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 hi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(&surface->bvh, lo, hi, gather);
        int32_t budget = 64;
        for (int32_t g = 0; g < gatherCount && budget > 0; ++g)
        {
            budget -= 1;
            m3Vec3 blo;
            m3Vec3 bhi;
            m3VoxelBoxBounds(surface, cell, gather[g], &blo, &bhi);
            m3Vec3 corners[8];
            for (int32_t k = 0; k < 8; ++k)
            {
                corners[k] = (m3Vec3){(k & 1) != 0 ? bhi.x : blo.x, (k & 2) != 0 ? bhi.y : blo.y,
                                      (k & 4) != 0 ? bhi.z : blo.z};
            }
            m3TOIInput input;
            input.proxyA.points = corners;
            input.proxyA.count = 8;
            input.proxyA.radius = 0.0f;
            input.proxyB = fastProxy;
            input.sweepA = chunkSweep;
            input.sweepB = ctx->fastSweep;
            input.maxFraction = ctx->fraction;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < ctx->fraction)
            {
                ctx->fraction = out.fraction;
            }
        }
        return true;
    }
    if (world->shapeType[shape] == (uint8_t)m3_meshShape)
    {
        // Mesh TOI: sweep the fast shape against every
        // candidate triangle. Each triangle is a three-point static
        // proxy in the mesh body's frame; the shared kernel does the
        // rest. Ascending triangle order, bounded candidates.
        const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shape]];
        m3Sweep meshSweep = MakeRelativeSweep(world, body, ctx->com0, ctx->rot0, ctx->base);
        m3Vec3 scratchFast[2];
        m3DistanceProxy fastProxy = m3MakeShapeProxy(world, ctx->fastShape, scratchFast);

        // The swept bounds of the fast body in mesh-local space, a
        // conservative box from the relative sweep endpoints.
        const m3Transform* xfM = &world->transforms[body];
        m3Vec3 c1 =
            m3InvRotateVec3(xfM->q, (m3Vec3){(m3real)(ctx->com0[ctx->fastBody].x - xfM->p.x),
                                             (m3real)(ctx->com0[ctx->fastBody].y - xfM->p.y),
                                             (m3real)(ctx->com0[ctx->fastBody].z - xfM->p.z)});
        m3Vec3 rlc =
            m3RotateVec3(world->transforms[ctx->fastBody].q, world->localCenters[ctx->fastBody]);
        m3Vec3 c2 = m3InvRotateVec3(
            xfM->q,
            (m3Vec3){(m3real)(world->transforms[ctx->fastBody].p.x + (double)rlc.x - xfM->p.x),
                     (m3real)(world->transforms[ctx->fastBody].p.y + (double)rlc.y - xfM->p.y),
                     (m3real)(world->transforms[ctx->fastBody].p.z + (double)rlc.z - xfM->p.z)});
        m3real pad = world->maxExtents[ctx->fastBody] + M3_AABB_MARGIN;
        m3Vec3 lo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 hi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount =
            m3MeshBvhGather(&world->meshBvh[world->shapeMeshIndex[shape]], lo, hi, gather);
        int32_t budget = 64;
        for (int32_t g = 0; g < gatherCount && budget > 0; ++g)
        {
            int32_t t = gather[g];
            m3Vec3 tv[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                            mesh->vertices[mesh->indices[3 * t + 1]],
                            mesh->vertices[mesh->indices[3 * t + 2]]};
            m3real tlx = m3MinF(tv[0].x, m3MinF(tv[1].x, tv[2].x));
            m3real thx = m3MaxF(tv[0].x, m3MaxF(tv[1].x, tv[2].x));
            m3real tly = m3MinF(tv[0].y, m3MinF(tv[1].y, tv[2].y));
            m3real thy = m3MaxF(tv[0].y, m3MaxF(tv[1].y, tv[2].y));
            m3real tlz = m3MinF(tv[0].z, m3MinF(tv[1].z, tv[2].z));
            m3real thz = m3MaxF(tv[0].z, m3MaxF(tv[1].z, tv[2].z));
            if (thx < lo.x || tlx > hi.x || thy < lo.y || tly > hi.y || thz < lo.z || tlz > hi.z)
            {
                continue;
            }
            budget -= 1;
            m3TOIInput input;
            input.proxyA.points = tv;
            input.proxyA.count = 3;
            input.proxyA.radius = 0.0f;
            input.proxyB = fastProxy;
            input.sweepA = meshSweep;
            input.sweepB = ctx->fastSweep;
            input.maxFraction = ctx->fraction;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < ctx->fraction)
            {
                ctx->fraction = out.fraction;
            }
        }
        return true;
    }
    if (world->types[body] != (uint8_t)m3_staticBody && world->bulletFlags[ctx->fastBody] == 0)
    {
        return true; // only bullets sweep against dynamics and kinematics
    }
    {
        // Filters: the continuous phase obeys the same rule
        // as the discrete pair scan.
        int32_t gi = world->shapeGroup[shape];
        int32_t gj = world->shapeGroup[ctx->fastShape];
        if (gi != 0 && gi == gj)
        {
            if (gi < 0)
            {
                return true;
            }
        }
        else if (!m3FilterPass(world->shapeCategory[shape], world->shapeMask[shape],
                               world->shapeCategory[ctx->fastShape],
                               world->shapeMask[ctx->fastShape]))
        {
            return true;
        }
    }

    m3TOIInput input;
    m3Vec3 scratchA[2];
    m3Vec3 scratchB[2];
    input.proxyA = m3MakeShapeProxy(world, shape, scratchA);
    input.proxyB = m3MakeShapeProxy(world, ctx->fastShape, scratchB);
    input.sweepA = MakeRelativeSweep(world, body, ctx->com0, ctx->rot0, ctx->base);
    input.sweepB = ctx->fastSweep;
    input.maxFraction = ctx->fraction;
    m3TOIOutput out = m3TimeOfImpact(&input);
    if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < ctx->fraction)
    {
        ctx->fraction = out.fraction;
    }
    return true;
}

// Plane targets are not in the tree: a bounded conservative advance
// against the analytic plane distance (support of the fast proxy at
// the swept pose, minus its radius).
static void ContinuousVersusPlane(const m3World* world, m3ContinuousContext* ctx,
                                  int32_t planeShape)
{
    m3Vec3 n = world->shapeGeom[planeShape].v;
    m3real offset =
        world->shapeGeom[planeShape].s -
        (m3real)((double)n.x * ctx->base.x + (double)n.y * ctx->base.y + (double)n.z * ctx->base.z);
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, ctx->fastShape, scratch);

    const m3real linearSlop = 0.005f;
    m3real target = m3MaxF(linearSlop, proxy.radius - linearSlop);
    m3real tolerance = 0.25f * linearSlop;

    // Conservative rate: linear travel plus the rotation arc.
    m3Vec3 travel = m3Sub3(ctx->fastSweep.c2, ctx->fastSweep.c1);
    m3Quat dq = m3MulQuat(ctx->fastSweep.q2, (m3Quat){-ctx->fastSweep.q1.x, -ctx->fastSweep.q1.y,
                                                      -ctx->fastSweep.q1.z, ctx->fastSweep.q1.w});
    m3real arc =
        2.0f * sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) * world->maxExtents[ctx->fastBody];
    m3real rate = sqrtf(m3Dot3(travel, travel)) + arc;
    if (!(rate > 0.0f))
    {
        return;
    }

    m3real t = 0.0f;
    for (int32_t iter = 0; iter < 25; ++iter)
    {
        m3Transform xf = m3GetSweepTransform(&ctx->fastSweep, t);
        m3real minD = 3.4e38f;
        for (int32_t k = 0; k < proxy.count; ++k)
        {
            m3Vec3 r = m3RotateVec3(xf.q, proxy.points[k]);
            m3real d = n.x * ((m3real)xf.p.x + r.x) + n.y * ((m3real)xf.p.y + r.y) +
                       n.z * ((m3real)xf.p.z + r.z);
            minD = m3MinF(minD, d);
        }
        m3real sep = minD - offset;
        if (sep <= 0.0f)
        {
            return; // started behind or overlapped: discrete owns it
        }
        if (sep <= target + tolerance)
        {
            // The reference rule the other three arms already obey:
            // a body ALREADY within the target distance at t = 0
            // belongs to the discrete speculative contact, not to
            // the continuous pull-back. This arm's missing guard
            // was the 6-3 lock: the pull-back at fraction zero
            // erased each step's integration while velocity stayed,
            // freezing bodies at the slop gap where the refunded
            // accumulator kept their friction at zero: eternal
            // frictionless skaters under every pile, sleep never
            // engaging, the aftermath phase priced like the storm.
            if (t > 0.0f && t < ctx->fraction)
            {
                ctx->fraction = t;
            }
            return;
        }
        t += (sep - target) / rate;
        if (t >= ctx->fraction)
        {
            return; // no earlier hit than the current best
        }
    }
}

static void SolveContinuousPhase(m3World* world, const m3Pos3* com0, const m3Quat* rot0)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        // Fast test: displacement plus rotation arc versus the
        // thinnest extent (the reference safety factor of one half).
        m3Vec3 rlc = m3RotateVec3(world->transforms[i].q, world->localCenters[i]);
        double cx = world->transforms[i].p.x + (double)rlc.x;
        double cy = world->transforms[i].p.y + (double)rlc.y;
        double cz = world->transforms[i].p.z + (double)rlc.z;
        m3Vec3 dc = {(m3real)(cx - com0[i].x), (m3real)(cy - com0[i].y), (m3real)(cz - com0[i].z)};
        m3Quat q0 = rot0[i];
        m3Quat dq = m3MulQuat(world->transforms[i].q, (m3Quat){-q0.x, -q0.y, -q0.z, q0.w});
        m3real arc = 2.0f * sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) * world->maxExtents[i];
        m3real maxMotion = sqrtf(m3Dot3(dc, dc)) + arc;
        if (!(maxMotion > 0.5f * world->minExtents[i]))
        {
            continue;
        }

        m3ContinuousContext ctx;
        ctx.world = world;
        ctx.com0 = com0;
        ctx.rot0 = rot0;
        ctx.fastBody = i;
        ctx.base = com0[i];
        ctx.fraction = 1.0f;
        ctx.fastSweep = MakeRelativeSweep(world, i, com0, rot0, ctx.base);

        for (int32_t s = world->bodyShapeHead[i]; s != -1; s = world->shapeNext[s])
        {
            if (world->shapeSensor[s] != 0)
            {
                continue; // a sensor on a fast body blocks nothing
            }
            ctx.fastShape = s;
            // Swept candidate box: both COM endpoints padded by the
            // body's max extent (a coarse superset; the TOI filters).
            double pad = (double)(world->maxExtents[i] + M3_AABB_MARGIN);
            double lo[3];
            double hi[3];
            lo[0] = (com0[i].x < cx ? com0[i].x : cx) - pad;
            lo[1] = (com0[i].y < cy ? com0[i].y : cy) - pad;
            lo[2] = (com0[i].z < cz ? com0[i].z : cz) - pad;
            hi[0] = (com0[i].x > cx ? com0[i].x : cx) + pad;
            hi[1] = (com0[i].y > cy ? com0[i].y : cy) + pad;
            hi[2] = (com0[i].z > cz ? com0[i].z : cz) + pad;
            m3TreeQuery(&world->tree, lo, hi, ContinuousQueryCallback, &ctx);

            // Planes take the dedicated pass (never in the tree).
            for (int32_t p = 0; p < maxShape; ++p)
            {
                if (world->shapePool.alive[p] != 0 && world->shapeType[p] == (uint8_t)m3_planeShape)
                {
                    ContinuousVersusPlane(world, &ctx, p);
                }
            }
        }

        if (ctx.fraction < 1.0f)
        {
            // Pull the body back to the impact pose. Velocity stays:
            // next step's speculative contact turns it into impulse.
            m3Transform xf = m3GetSweepTransform(&ctx.fastSweep, ctx.fraction);
            world->transforms[i].q = xf.q;
            world->transforms[i].p.x = ctx.base.x + xf.p.x;
            world->transforms[i].p.y = ctx.base.y + xf.p.y;
            world->transforms[i].p.z = ctx.base.z + xf.p.z;
        }
    }
}

// ---------------------------------------------------------------
// Islands and sleep (2b-10, the Maul2D recipe): union-find over the
// touching dynamic pairs in canonical order. The wake pass runs
// right after contacts are built (a sleeping body touched by an
// awake one must join THIS step's solve); the sleep decision runs at
// the end of the step with the same island labels. Sleeping bodies
// are bit-frozen: velocities zeroed once, integration and solving
// skip them, and the hash stands still.
// ---------------------------------------------------------------

static int32_t IslandFind(int32_t* parent, int32_t i)
{
    while (parent[i] != i)
    {
        parent[i] = parent[parent[i]]; // halving, deterministic
        i = parent[i];
    }
    return i;
}

static void IslandUnion(int32_t* parent, int32_t a, int32_t b)
{
    int32_t ra = IslandFind(parent, a);
    int32_t rb = IslandFind(parent, b);
    if (ra != rb)
    {
        // Lower root wins: canonical labels independent of order.
        if (ra < rb)
        {
            parent[rb] = ra;
        }
        else
        {
            parent[ra] = rb;
        }
    }
}

// Build islands from the current touching pairs and wake every
// island that contains an awake member or a moving kinematic
// neighbor. Returns the parent array (scratch-allocated).
static int32_t* IslandWakePass(m3World* world)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    int32_t* parent = (int32_t*)m3StackAlloc(&world->scratch,
                                             maxBody > 0 ? maxBody * (int32_t)sizeof(int32_t) : 4);
    uint8_t* forced = (uint8_t*)m3StackAlloc(&world->scratch, maxBody > 0 ? maxBody : 1);
    if (parent == NULL || forced == NULL)
    {
        return NULL;
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        parent[i] = i;
        forced[i] = 0;
    }
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->manifolds[i].pointCount == 0)
        {
            continue;
        }
        uint64_t key = world->pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if (world->shapeSensor[shapeA] != 0 || world->shapeSensor[shapeB] != 0)
        {
            continue; // sensor overlap couples nothing and wakes no one
        }
        int32_t bodyA = world->shapeBody[shapeA];
        int32_t bodyB = world->shapeBody[shapeB];
        int dynA = world->types[bodyA] == (uint8_t)m3_dynamicBody;
        int dynB = world->types[bodyB] == (uint8_t)m3_dynamicBody;
        if (dynA && dynB)
        {
            IslandUnion(parent, bodyA, bodyB);
        }
        else if (dynA || dynB)
        {
            // A moving kinematic neighbor forces its contact awake.
            int32_t kin = dynA ? bodyB : bodyA;
            int32_t dyn = dynA ? bodyA : bodyB;
            if (world->types[kin] == (uint8_t)m3_kinematicBody)
            {
                m3Vec3 v = world->linearVelocities[kin];
                m3Vec3 w = world->angularVelocities[kin];
                if (m3Dot3(v, v) > 0.0f || m3Dot3(w, w) > 0.0f)
                {
                    forced[dyn] = 1;
                }
            }
        }
    }
    // Joints couple islands exactly like touching contacts, and a
    // moving kinematic partner is a wake source through a joint too.
    int32_t maxJoint = world->jointPool.maxIndex;
    for (int32_t j = 0; j < maxJoint; ++j)
    {
        if (world->jointPool.alive[j] == 0)
        {
            continue;
        }
        int32_t bodyA = world->jointBodyA[j];
        int32_t bodyB = world->jointBodyB[j];
        int dynA = world->types[bodyA] == (uint8_t)m3_dynamicBody;
        int dynB = world->types[bodyB] == (uint8_t)m3_dynamicBody;
        if (dynA && dynB)
        {
            IslandUnion(parent, bodyA, bodyB);
        }
        else if (dynA || dynB)
        {
            int32_t kin = dynA ? bodyB : bodyA;
            int32_t dyn = dynA ? bodyA : bodyB;
            if (world->types[kin] == (uint8_t)m3_kinematicBody)
            {
                m3Vec3 v = world->linearVelocities[kin];
                m3Vec3 w = world->angularVelocities[kin];
                if (m3Dot3(v, v) > 0.0f || m3Dot3(w, w) > 0.0f)
                {
                    forced[dyn] = 1;
                }
            }
        }
    }

    // Aggregate: does any island member demand wakefulness?
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        if (world->awake[i] != 0 || forced[i] != 0)
        {
            forced[IslandFind(parent, i)] = 1;
        }
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        if (world->awake[i] == 0 && forced[IslandFind(parent, i)] != 0)
        {
            world->awake[i] = 1; // woken by the island: timers restart
            world->sleepTimes[i] = 0.0f;
        }
    }
    return parent;
}

// End-of-step sleep decision: timers advance for slow awake bodies,
// and an island sleeps only when EVERY member is ready.
static void IslandSleepPass(m3World* world, int32_t* parent, const m3Pos3* com0, const m3Quat* rot0,
                            float dt)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    m3real invDt = dt > 0.0f ? 1.0f / dt : 0.0f;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        m3Vec3 v = world->linearVelocities[i];
        m3Vec3 w = world->angularVelocities[i];
        m3real velocity = sqrtf(m3Dot3(v, v)) + sqrtf(m3Dot3(w, w)) * world->maxExtents[i];
        // Position correction counts too (the reference lesson: bias
        // pushes move bodies that report zero velocity).
        m3Vec3 rlc = m3RotateVec3(world->transforms[i].q, world->localCenters[i]);
        m3Vec3 dc = {(m3real)(world->transforms[i].p.x + (double)rlc.x - com0[i].x),
                     (m3real)(world->transforms[i].p.y + (double)rlc.y - com0[i].y),
                     (m3real)(world->transforms[i].p.z + (double)rlc.z - com0[i].z)};
        m3Quat q0 = rot0[i];
        m3Quat dq = m3MulQuat(world->transforms[i].q, (m3Quat){-q0.x, -q0.y, -q0.z, q0.w});
        m3real motion = sqrtf(m3Dot3(dc, dc)) + 2.0f *
                                                    sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) *
                                                    world->maxExtents[i];
        m3real sleepVelocity = m3MaxF(velocity, 0.5f * invDt * motion);
        if (world->bodyCanSleep[i] == 0)
        {
            sleepVelocity = 3.4e38f; // never below any threshold
        }
        if (sleepVelocity < world->bodySleepThreshold[i])
        {
            world->sleepTimes[i] += dt;
        }
        else
        {
            world->sleepTimes[i] = 0.0f;
        }
    }
    // Island readiness: every member past the time-to-sleep bar.
    uint8_t* ready = (uint8_t*)m3StackAlloc(&world->scratch, maxBody > 0 ? maxBody : 1);
    if (ready == NULL)
    {
        return;
    }
    memset(ready, 1, (size_t)(maxBody > 0 ? maxBody : 1));
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        if (world->sleepTimes[i] < 0.5f)
        {
            ready[IslandFind(parent, i)] = 0;
        }
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        if (ready[IslandFind(parent, i)] != 0)
        {
            // The whole island crosses together: freeze bit-solid.
            world->awake[i] = 0;
            world->linearVelocities[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
            world->angularVelocities[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
            // S-3b: the freeze step may have pushed this body into
            // overlaps no pair list ever saw; discover them now so
            // the frozen buffer equals what a full query would find.
            m3FreezeDiscoverPairs(world, i);
        }
    }
}

void m3StepInternal(m3World* world, float dt, int32_t substeps)
{
    // The step profile: coarse wall-clock brackets, written
    // to the world only at the single complete-step exit, so a
    // stalled step keeps the previous profile. Observer data only.
    m3Profile prof;
    memset(&prof, 0, sizeof(prof));
    double tStep = m3NowMs();

    // The documented growth (allocator.h, finally exercised by the
    // 6-1 city block): a step that starves the scratch stalls
    // loudly and harmlessly; the NEXT step arrives with double the
    // room. Size-driven, so twins and replays stall and grow on
    // exactly the same ticks: the stall is deterministic state
    // evolution, not noise. No scene under 5000 bodies had ever
    // overflowed 256 KB, which is how the promise stayed unpaid
    // for four phases.
    if (world->scratch.overflow != 0 && world->scratch.capacity < (1 << 28))
    {
        int32_t bigger = world->scratch.capacity * 2;
        m3StackDestroy(&world->scratch);
        world->scratch = m3StackCreate(bigger);
    }

    // Stash the previous pairs and manifolds before the scan
    // overwrites them; the warm-start carry and the event walk read
    // the stash. The buffers belong to the world, so this never
    // allocates and an early return cannot leak.
    int32_t oldCount = world->pairCount;
    const uint64_t* oldKeys = world->stashPairKeys;
    const m3Manifold* oldManifolds = world->stashManifolds;
    if (oldCount > 0)
    {
        memcpy(world->stashPairKeys, world->pairKeys, (size_t)oldCount * sizeof(uint64_t));
        memcpy(world->stashManifolds, world->manifolds, (size_t)oldCount * sizeof(m3Manifold));
    }

    // The suspension pass: vehicle impulses land here so the
    // narrowphase and solver see the sprung chassis the same way
    // they see gravity. Serial, slot order, canonical.
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
        return; // scratch starve (transient, grown next step) or a
                // full pair table: the world stalls, never corrupts
    }

    // Contact events: a canonical merge walk of the old and new pair
    // lists (both sorted). Serial and after the parallel narrowphase
    // on purpose: appends must happen in pair order, bit-stably.
    t0 = m3NowMs();
    world->beginEventCount = 0;
    world->endEventCount = 0;
    world->sensorBeginEventCount = 0;
    world->sensorEndEventCount = 0;
    world->fragmentEventCount = 0;
    world->fragmentRecipeCount = 0;
    world->fragmentDropped = 0;
    world->hitEventCount = 0;
    world->hitEventsDropped = 0;
    world->moveEventCount = 0;
    world->jointBreakEventCount = 0;
    {
        int32_t iNew = 0;
        int32_t iOld = 0;
        while (iNew < world->pairCount || iOld < oldCount)
        {
            uint64_t keyNew = iNew < world->pairCount ? world->pairKeys[iNew] : UINT64_MAX;
            uint64_t keyOld = iOld < oldCount ? oldKeys[iOld] : UINT64_MAX;
            int touchNew = 0;
            int touchOld = 0;
            uint64_t key;
            if (keyNew < keyOld)
            {
                key = keyNew;
                touchNew = world->manifolds[iNew].pointCount > 0;
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
                touchNew = world->manifolds[iNew].pointCount > 0;
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
            if (world->shapePool.alive[sA] == 0 || world->shapePool.alive[sB] == 0)
            {
                continue;
            }
            m3ContactEvent event;
            event.shapeA =
                (m3ShapeId){sA + 1, world->worldIndex0, world->shapePool.generations[sA]};
            event.shapeB =
                (m3ShapeId){sB + 1, world->worldIndex0, world->shapePool.generations[sB]};
            int sensorPair = world->shapeSensor[sA] != 0 || world->shapeSensor[sB] != 0;
            if (sensorPair)
            {
                if (touchNew && world->sensorBeginEventCount < world->pairCapacity)
                {
                    world->sensorBeginEvents[world->sensorBeginEventCount++] = event;
                }
                else if (touchOld && world->sensorEndEventCount < world->pairCapacity)
                {
                    world->sensorEndEvents[world->sensorEndEventCount++] = event;
                }
            }
            else if (touchNew && world->beginEventCount < world->pairCapacity)
            {
                world->beginEvents[world->beginEventCount++] = event;
            }
            else if (touchOld && world->endEventCount < world->pairCapacity)
            {
                world->endEvents[world->endEventCount++] = event;
            }
        }
    }

    prof.events = (float)(m3NowMs() - t0);
    t0 = m3NowMs();

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
        int64_t need = 64 * 1024 + 128 * (int64_t)world->bodyPool.maxIndex +
                       64 * (int64_t)world->shapePool.maxIndex + 1024 * (int64_t)world->pairCount +
                       1024 * (int64_t)world->jointPool.maxIndex;
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
    m3StackReset(&world->scratch);

    // Begin-of-step COM and rotation for every body: the sweeps the
    // continuous pass needs. The scan reads transforms but
    // never moves them, so capturing here equals capturing before
    // it, and now the capture lives under the fresh-count budget.
    int32_t sweepMax = world->bodyPool.maxIndex;
    m3Pos3* com0 =
        (m3Pos3*)m3StackAlloc(&world->scratch, sweepMax > 0 ? sweepMax * (int32_t)sizeof(m3Pos3)
                                                            : (int32_t)sizeof(m3Pos3));
    m3Quat* rot0 =
        (m3Quat*)m3StackAlloc(&world->scratch, sweepMax > 0 ? sweepMax * (int32_t)sizeof(m3Quat)
                                                            : (int32_t)sizeof(m3Quat));
    if (com0 == NULL || rot0 == NULL)
    {
        return; // unreachable backstop (see the pre-flight above)
    }
    for (int32_t i = 0; i < sweepMax; ++i)
    {
        if (world->bodyPool.alive[i] == 0)
        {
            continue;
        }
        m3Vec3 rlc = m3RotateVec3(world->transforms[i].q, world->localCenters[i]);
        com0[i].x = world->transforms[i].p.x + (double)rlc.x;
        com0[i].y = world->transforms[i].p.y + (double)rlc.y;
        com0[i].z = world->transforms[i].p.z + (double)rlc.z;
        rot0[i] = world->transforms[i].q;
    }

    // Islands and wake propagation BEFORE the solve: a sleeping body
    // touched by an awake one participates in this very step.
    int32_t* islandParent = IslandWakePass(world);
    if (islandParent == NULL)
    {
        return; // transient scratch stall, grown next step
    }

    // Solver scratch: constraints plus per-body delta accumulators.
    int32_t maxBody = world->bodyPool.maxIndex;
    m3ContactConstraint* constraints = (m3ContactConstraint*)m3StackAlloc(
        &world->scratch, world->pairCount > 0
                             ? world->pairCount * (int32_t)sizeof(m3ContactConstraint)
                             : (int32_t)sizeof(m3ContactConstraint));
    m3Vec3* deltaPos = (m3Vec3*)m3StackAlloc(
        &world->scratch, maxBody > 0 ? maxBody * (int32_t)sizeof(m3Vec3) : (int32_t)sizeof(m3Vec3));
    m3Quat* deltaRot = (m3Quat*)m3StackAlloc(
        &world->scratch, maxBody > 0 ? maxBody * (int32_t)sizeof(m3Quat) : (int32_t)sizeof(m3Quat));
    if (constraints == NULL || deltaPos == NULL || deltaRot == NULL)
    {
        return; // transient scratch stall, grown next step
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        deltaPos[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
        deltaRot[i] = m3MakeIdentityQuat();
    }

    prof.prepare = (float)(m3NowMs() - t0);
    t0 = m3NowMs();

    m3real h = dt / (m3real)substeps;
    m3real invH = h > 0.0f ? 1.0f / h : 0.0f;
    int32_t constraintCount = PrepareContacts(world, constraints, h);

    m3SolverColoring coloring;
    if (!BuildColoring(world, constraints, constraintCount, &coloring))
    {
        return; // transient scratch stall, grown next step
    }
    int32_t usedColors = 0;
    for (int32_t c = 0; c < M3_GRAPH_COLORS + 1; ++c)
    {
        if (coloring.starts[c + 1] > coloring.starts[c])
        {
            usedColors += 1;
        }
    }

    int32_t maxJointSlots = world->jointPool.maxIndex;
    m3JointConstraint* jointConstraints = (m3JointConstraint*)m3StackAlloc(
        &world->scratch, maxJointSlots > 0 ? maxJointSlots * (int32_t)sizeof(m3JointConstraint)
                                           : (int32_t)sizeof(m3JointConstraint));
    if (jointConstraints == NULL)
    {
        return; // transient scratch stall, grown next step
    }
    int32_t jointCount = m3PrepareJoints(world, jointConstraints, h);

    // The mover list: one scan builds the compact
    // list of bodies the substep loops touch (awake dynamics and
    // kinematics), in ascending slot order so iteration stays
    // canonical. Eight-plus full-array walks per step become one:
    // at five thousand bodies with most of the world asleep, the
    // integrate loops stop paying for the sleepers. Same math,
    // same order, not one bit of simulation moves.
    int32_t* movers = (int32_t*)m3StackAlloc(&world->scratch,
                                             maxBody > 0 ? maxBody * (int32_t)sizeof(int32_t) : 4);
    int32_t moverCount = 0;
    if (movers == NULL)
    {
        return; // transient scratch stall, grown next step
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->bodyEnabled[i] == 0)
        {
            continue; // disabled bodies vanish from the step
        }
        uint8_t type = world->types[i];
        if (type == (uint8_t)m3_kinematicBody ||
            (type == (uint8_t)m3_dynamicBody && world->awake[i] != 0))
        {
            movers[moverCount] = i;
            moverCount += 1;
        }
        // The kinematic servo: choose velocities so this
        // step lands the body ON its target, then clear the order.
        if (type == (uint8_t)m3_kinematicBody && world->bodyHasTarget[i] != 0)
        {
            m3real servoInvDt = 1.0f / dt;
            const m3Transform* now = &world->transforms[i];
            const m3Transform* want = &world->bodyTarget[i];
            world->linearVelocities[i] = (m3Vec3){(m3real)(want->p.x - now->p.x) * servoInvDt,
                                                  (m3real)(want->p.y - now->p.y) * servoInvDt,
                                                  (m3real)(want->p.z - now->p.z) * servoInvDt};
            m3Quat dq = m3MulQuat(want->q, (m3Quat){-now->q.x, -now->q.y, -now->q.z, now->q.w});
            if (dq.w < 0.0f)
            {
                dq = (m3Quat){-dq.x, -dq.y, -dq.z, -dq.w};
            }
            world->angularVelocities[i] = m3MulSV3(2.0f * servoInvDt, (m3Vec3){dq.x, dq.y, dq.z});
            world->bodyHasTarget[i] = 0;
        }
    }

    // Water volumes: per-mover buoyancy force, torque, drag
    // rates, and the blended flow, computed ONCE per step from the
    // step-start pose (the classic field-force approximation). No
    // volumes = no allocation, no arithmetic, the pre-18 bits.
    int32_t waterAlive = 0;
    for (int32_t k = 0; k < world->waterPool.maxIndex; ++k)
    {
        waterAlive += world->waterPool.alive[k];
    }
    m3Vec3* buoyForce = NULL;
    m3Vec3* buoyTorque = NULL;
    m3Vec3* buoyFlow = NULL;
    float* buoyLin = NULL;
    float* buoyAng = NULL;
    if (waterAlive > 0 && moverCount > 0)
    {
        buoyForce = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
        buoyTorque = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
        buoyFlow = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
        buoyLin = (float*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(float));
        buoyAng = (float*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(float));
        if (buoyForce == NULL || buoyTorque == NULL || buoyFlow == NULL || buoyLin == NULL ||
            buoyAng == NULL)
        {
            waterAlive = 0; // transient scratch stall: dry step
        }
    }
    if (waterAlive > 0 && moverCount > 0)
    {
        for (int32_t m = 0; m < moverCount; ++m)
        {
            int32_t i = movers[m];
            buoyForce[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
            buoyTorque[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
            buoyFlow[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
            buoyLin[m] = 0.0f;
            buoyAng[m] = 0.0f;
            if (world->types[i] != (uint8_t)m3_dynamicBody)
            {
                continue;
            }
            m3Vec3 rlc = m3RotateVec3(world->transforms[i].q, world->localCenters[i]);
            double comX = world->transforms[i].p.x + (double)rlc.x;
            double comY = world->transforms[i].p.y + (double)rlc.y;
            double comZ = world->transforms[i].p.z + (double)rlc.z;
            float fracSum = 0.0f;
            for (int32_t shape = world->bodyShapeHead[i]; shape >= 0;
                 shape = world->shapeNext[shape])
            {
                double slo[3];
                double shi[3];
                m3ShapeFatAabb(world, shape, slo, shi);
                double shapeVol = (shi[0] - slo[0]) * (shi[1] - slo[1]) * (shi[2] - slo[2]);
                if (!(shapeVol > 0.0))
                {
                    continue; // a plane's infinite box never swims
                }
                for (int32_t k = 0; k < world->waterPool.maxIndex; ++k)
                {
                    if (world->waterPool.alive[k] == 0)
                    {
                        continue;
                    }
                    double clo[3];
                    double chi[3];
                    clo[0] = slo[0] > world->waterLo[k].x ? slo[0] : world->waterLo[k].x;
                    clo[1] = slo[1] > world->waterLo[k].y ? slo[1] : world->waterLo[k].y;
                    clo[2] = slo[2] > world->waterLo[k].z ? slo[2] : world->waterLo[k].z;
                    chi[0] = shi[0] < world->waterHi[k].x ? shi[0] : world->waterHi[k].x;
                    chi[1] = shi[1] < world->waterHi[k].y ? shi[1] : world->waterHi[k].y;
                    chi[2] = shi[2] < world->waterHi[k].z ? shi[2] : world->waterHi[k].z;
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
                    m3Vec3 f = m3MulSV3(-(float)subVol * world->waterDensity[k], world->gravity);
                    m3Vec3 r = {(float)(0.5 * (clo[0] + chi[0]) - comX),
                                (float)(0.5 * (clo[1] + chi[1]) - comY),
                                (float)(0.5 * (clo[2] + chi[2]) - comZ)};
                    buoyForce[m] = m3Add3(buoyForce[m], f);
                    buoyTorque[m] = m3Add3(buoyTorque[m], m3Cross3(r, f));
                    buoyFlow[m] = m3Add3(buoyFlow[m], m3MulSV3(frac, world->waterFlow[k]));
                    buoyLin[m] += world->waterLinDrag[k] * frac;
                    buoyAng[m] += world->waterAngDrag[k] * frac;
                    fracSum += frac;
                }
            }
            if (fracSum > 0.0f)
            {
                buoyFlow[m] = m3MulSV3(1.0f / fracSum, buoyFlow[m]);
            }
        }
    }

    for (int32_t sub = 0; sub < substeps; ++sub)
    {
        // Integrate velocities (fixed body order): gravity, damping.
        for (int32_t m = 0; m < moverCount; ++m)
        {
            int32_t i = movers[m];
            if (world->types[i] != (uint8_t)m3_dynamicBody)
            {
                continue; // kinematics ride the list for positions only
            }
            m3Vec3 v = world->linearVelocities[i];
            m3Vec3 w = world->angularVelocities[i];
            v = m3Add3(v, m3MulSV3(h * world->gravityScales[i], world->gravity));
            // Host forces and torques integrate beside
            // gravity, every substep, so a force held for one step
            // delivers exactly force times dt.
            v = m3Add3(v, m3MulSV3(h * world->invMass[i], world->bodyForce[i]));
            if (world->bodyTorque[i].x != 0.0f || world->bodyTorque[i].y != 0.0f ||
                world->bodyTorque[i].z != 0.0f)
            {
                w = m3Add3(
                    w, m3MulSV3(h, m3MulMV3(m3WorldInvInertia(world, i), world->bodyTorque[i])));
            }
            if (waterAlive > 0 && (buoyLin[m] > 0.0f || buoyForce[m].y != 0.0f ||
                                   buoyForce[m].x != 0.0f || buoyForce[m].z != 0.0f))
            {
                // The water field: buoyant impulse, torque
                // about the submerged centroid, then drag pulls the
                // RELATIVE velocity toward the flow (the damping
                // recipe, recentered on the current).
                v = m3Add3(v, m3MulSV3(h * world->invMass[i], buoyForce[m]));
                w = m3Add3(w, m3MulSV3(h, m3MulMV3(m3WorldInvInertia(world, i), buoyTorque[m])));
                m3Vec3 rel = m3Sub3(v, buoyFlow[m]);
                v = m3Add3(buoyFlow[m], m3MulSV3(1.0f / (1.0f + h * buoyLin[m]), rel));
                w = m3MulSV3(1.0f / (1.0f + h * buoyAng[m]), w);
            }
            v = m3MulSV3(1.0f / (1.0f + h * world->linearDamping[i]), v);
            w = m3MulSV3(1.0f / (1.0f + h * world->angularDamping[i]), w);
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
            if ((world->bodyLocks[i] & M3_LOCKS_ALLOW_FAST_ROTATION) == 0)
            {
                m3real w2 = m3Dot3(w, w);
                m3real wcap = world->maximumAngularSpeed;
                if (w2 > wcap * wcap)
                {
                    w = m3MulSV3(wcap / sqrtf(w2), w);
                }
            }
            world->linearVelocities[i] = v;
            world->angularVelocities[i] = w;
        }

        m3WarmStartJoints(world, jointConstraints, jointCount, deltaRot);
        RunColored(world, &coloring, constraints, deltaPos, deltaRot, invH, 0, 1);
        m3SolveJoints(world, jointConstraints, jointCount, deltaPos, deltaRot, h, invH, 1);
        RunColored(world, &coloring, constraints, deltaPos, deltaRot, invH, 1, 0);

        // Integrate positions and accumulate the substep deltas the
        // separation tracking reads.
        for (int32_t m = 0; m < moverCount; ++m)
        {
            int32_t i = movers[m];
            m3Vec3 v = world->linearVelocities[i];
            m3Vec3 w = world->angularVelocities[i];
            uint8_t locks = world->bodyLocks[i];
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
                world->linearVelocities[i] = v;
                world->angularVelocities[i] = w;
            }
            // Rigid bodies rotate about the center of mass: advance
            // the COM, spin, then place the origin back. A centered
            // body (lc zero) reduces to the plain origin update.
            m3Vec3 lc = world->localCenters[i];
            m3Vec3 rlcOld = m3RotateVec3(world->transforms[i].q, lc);
            double cx = world->transforms[i].p.x + (double)rlcOld.x + (double)(h * v.x);
            double cy = world->transforms[i].p.y + (double)rlcOld.y + (double)(h * v.y);
            double cz = world->transforms[i].p.z + (double)rlcOld.z + (double)(h * v.z);
            m3Vec3 dw = m3MulSV3(h, w);
            world->transforms[i].q = m3IntegrateRotation(world->transforms[i].q, dw);
            m3Vec3 rlcNew = m3RotateVec3(world->transforms[i].q, lc);
            world->transforms[i].p.x = cx - (double)rlcNew.x;
            world->transforms[i].p.y = cy - (double)rlcNew.y;
            world->transforms[i].p.z = cz - (double)rlcNew.z;
            deltaPos[i] = m3Add3(deltaPos[i], m3MulSV3(h, v));
            deltaRot[i] = m3IntegrateRotation(deltaRot[i], dw);
        }

        // Relax: remove the bias energy (reference schedule).
        m3SolveJoints(world, jointConstraints, jointCount, deltaPos, deltaRot, h, invH, 0);
        RunColored(world, &coloring, constraints, deltaPos, deltaRot, invH, 0, 0);
    }

    // Host force accumulators are consumed: a force lives for one
    // step. Only movers could carry one (application wakes).
    for (int32_t m = 0; m < moverCount; ++m)
    {
        world->bodyForce[movers[m]] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->bodyTorque[movers[m]] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }

    Restitution(world, constraints, constraintCount);
    StoreImpulses(world, constraints, constraintCount);
    m3StoreJointImpulses(world, jointConstraints, jointCount);
    world->lastInvH = invH;

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

    // Joint breakage: reactions over threshold destroy the
    // joint and emit the break event, serially in ascending joint
    // order, a pure function of state (like fragmentation: derived
    // transitions never need their own journal op).
    {
        int32_t maxJoint = world->jointPool.maxIndex;
        for (int32_t j = 0; j < maxJoint; ++j)
        {
            if (world->jointPool.alive[j] == 0)
            {
                continue;
            }
            m3real maxForce = world->jointBreak[j].x;
            m3real maxTorque = world->jointBreak[j].y;
            if (maxForce == 0.0f && maxTorque == 0.0f)
            {
                continue;
            }
            m3real force;
            m3real torque;
            m3JointReactionMagnitudes(world, j, invH, &force, &torque);
            if ((maxForce > 0.0f && force > maxForce) || (maxTorque > 0.0f && torque > maxTorque))
            {
                m3JointId id = {j + 1, world->worldIndex0, world->jointPool.generations[j]};
                m3AppendJointBreakEvent(world, id);
                m3DestroyJointInternal(world, j);
            }
        }
    }

    prof.solve = (float)(m3NowMs() - t0);
    t0 = m3NowMs();
    if (world->continuousEnabled != 0)
    {
        SolveContinuousPhase(world, com0, rot0);
    }
    prof.continuous = (float)(m3NowMs() - t0);
    // Island census before the sleep pass retires anyone: awake
    // dynamic union-find roots, an observer count, and the
    // per-body island label the extras draw tints by.
    // Sleeping bodies keep the label of the island they slept in.
    int32_t islands = 0;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] != 0 && world->types[i] == (uint8_t)m3_dynamicBody &&
            world->awake[i] != 0)
        {
            int32_t root = i;
            while (islandParent[root] != root)
            {
                root = islandParent[root];
            }
            world->bodyIsland[i] = root;
            if (root == i)
            {
                islands += 1;
            }
        }
    }
    t0 = m3NowMs();
    if (world->sleepEnabled != 0)
    {
        IslandSleepPass(world, islandParent, com0, rot0, dt);
    }
    prof.sleep = (float)(m3NowMs() - t0);

    // Body move events: one per mover, ascending body order
    // (the mover list is built that way), post-step transform, and
    // fellAsleep on the step the island dropped off. Capacity is
    // bodyCapacity: movers cannot overflow it.
    for (int32_t m = 0; m < moverCount; ++m)
    {
        int32_t i = movers[m];
        m3BodyMoveEvent* e = &world->moveEvents[world->moveEventCount++];
        e->body = (m3BodyId){i + 1, world->worldIndex0, world->bodyPool.generations[i]};
        e->transform = world->transforms[i];
        e->fellAsleep = world->types[i] == (uint8_t)m3_dynamicBody && world->awake[i] == 0;
    }
    t0 = m3NowMs();
    m3CharacterCarryRiders(world, com0, rot0);
    prof.characters = (float)(m3NowMs() - t0);
    t0 = m3NowMs();
    m3SoftBodyPass(world, dt, substeps);
    prof.softBodies = (float)(m3NowMs() - t0);

    world->stepCount += 1;
    prof.step = (float)(m3NowMs() - tStep);
    world->profile = prof;
    world->lastIslandCount = islands;
    world->lastColorCount = usedColors;
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
    world->stepVetoCount = 0;
    m3StepInternal(world, dt, substeps);
    if (world->journalActive != 0)
    {
        // Recording moved BEHIND the execution: nothing can
        // journal during a step, so callback-less streams are
        // byte-identical to the old order, and a step that vetoed
        // contacts writes those keys first. A bare replay (no
        // callback installed) then applies the recorded vetoes and
        // lands on the recorded bits: the tape is self-sufficient.
        if (world->stepVetoCount > 0)
        {
            m3JournalRecord(world, m3_opStepVetoes, world->stepVetoKeys,
                            world->stepVetoCount * (int32_t)sizeof(uint64_t));
        }
        struct
        {
            float dt;
            int32_t substeps;
        } record;
        memset(&record, 0, sizeof(record));
        record.dt = dt;
        record.substeps = substeps;
        m3JournalRecord(world, m3_opStep, &record, (int32_t)sizeof(record));
    }
}
