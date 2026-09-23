// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The contact solver: prepare, warm start and solve by graph color, then
// restitution and the impulse store.

#include "contact_solver.h"

#include "manifold.h"
#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static m3Vec3 VelocityAt(const m3World* world, int32_t body, m3Vec3 arm)
{
    m3Vec3 v = world->bodies.linearVelocities[body];
    m3Vec3 w = world->bodies.angularVelocities[body];
    return m3Add3(v, m3Cross3(w, arm));
}

static void ApplyImpulse(m3World* world, const m3ContactConstraint* c, m3Vec3 impulse, m3Vec3 rsA,
                         m3Vec3 rsB)
{
    if (world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody)
    {
        world->bodies.linearVelocities[c->bodyA] =
            m3Sub3(world->bodies.linearVelocities[c->bodyA], m3MulSV3(c->invMassA, impulse));
        world->bodies.angularVelocities[c->bodyA] = m3Sub3(
            world->bodies.angularVelocities[c->bodyA], m3MulMV3(c->invIA, m3Cross3(rsA, impulse)));
    }
    if (world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody)
    {
        world->bodies.linearVelocities[c->bodyB] =
            m3Add3(world->bodies.linearVelocities[c->bodyB], m3MulSV3(c->invMassB, impulse));
        world->bodies.angularVelocities[c->bodyB] = m3Add3(
            world->bodies.angularVelocities[c->bodyB], m3MulMV3(c->invIB, m3Cross3(rsB, impulse)));
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
int32_t m3PrepareContacts(m3World* world, m3ContactConstraint* constraints, m3real h)
{
    m3Softness soft = m3MakeSoft(world->contactHertz, world->contactDampingRatio, h);
    m3Softness staticSoft = m3MakeSoft(2.0f * world->contactHertz, world->contactDampingRatio, h);

    int32_t count = 0;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        const m3Manifold* manifold = &world->contacts.manifolds[i];
        if (manifold->pointCount == 0)
        {
            continue;
        }
        uint64_t key = world->contacts.pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        int32_t bodyA = world->shapes.shapeBody[shapeA];
        int32_t bodyB = world->shapes.shapeBody[shapeB];
        int awakeDynA = world->bodies.types[bodyA] == (uint8_t)m3_dynamicBody &&
                        world->bodies.awake[bodyA] != 0;
        int awakeDynB = world->bodies.types[bodyB] == (uint8_t)m3_dynamicBody &&
                        world->bodies.awake[bodyB] != 0;
        if (!awakeDynA && !awakeDynB)
        {
            continue; // both sides frozen or immovable: impulses stay put
        }
        if (world->shapes.shapeSensor[shapeA] != 0 || world->shapes.shapeSensor[shapeB] != 0)
        {
            continue; // sensors detect, they never respond
        }
        if (world->contacts.replayVetoCount > 0)
        {
            // A recorded veto set owns this step: the keys are
            // canonical ascending, the pairs arrive in the same
            // order, and the callback (if any) stays silent so the
            // tape's truth cannot be second-guessed.
            int32_t lo = 0;
            int32_t hi = world->contacts.replayVetoCount - 1;
            int vetoed = 0;
            while (lo <= hi)
            {
                int32_t mid = (lo + hi) / 2;
                if (world->contacts.replayVetoKeys[mid] == key)
                {
                    vetoed = 1;
                    break;
                }
                if (world->contacts.replayVetoKeys[mid] < key)
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
        else if (world->preSolveFn != NULL && (world->shapes.shapePreSolve[shapeA] != 0 ||
                                               world->shapes.shapePreSolve[shapeB] != 0))
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
            m3Vec3 lcB =
                m3RotateVec3(world->bodies.transforms[bodyB].q, world->bodies.localCenters[bodyB]);
            m3Pos3 point;
            point.x = world->bodies.transforms[bodyB].p.x + (double)lcB.x +
                      (double)manifold->points[deep].anchorB.x;
            point.y = world->bodies.transforms[bodyB].p.y + (double)lcB.y +
                      (double)manifold->points[deep].anchorB.y;
            point.z = world->bodies.transforms[bodyB].p.z + (double)lcB.z +
                      (double)manifold->points[deep].anchorB.z;
            m3ShapeId idA = {shapeA + 1, world->worldIndex0,
                             world->shapes.shapePool.generations[shapeA]};
            m3ShapeId idB = {shapeB + 1, world->worldIndex0,
                             world->shapes.shapePool.generations[shapeB]};
            if (!world->preSolveFn(idA, idB, point, manifold->normal, world->preSolveContext))
            {
                // Vetoed: no constraint this step. The key joins the
                // journal annex so a bare replay repeats the
                // decision; collection order = pair order = sorted.
                if (world->recorder.journalActive != 0 &&
                    world->contacts.stepVetoCount < world->contacts.pairCapacity)
                {
                    world->contacts.stepVetoKeys[world->contacts.stepVetoCount++] = key;
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
        c->invMassA = world->bodies.types[bodyA] == (uint8_t)m3_dynamicBody
                          ? world->bodies.invMass[bodyA]
                          : 0.0f;
        c->invMassB = world->bodies.types[bodyB] == (uint8_t)m3_dynamicBody
                          ? world->bodies.invMass[bodyB]
                          : 0.0f;
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
        float fricA = world->shapes.shapeFriction[shapeA];
        float fricB = world->shapes.shapeFriction[shapeB];
        float restA = world->shapes.shapeRestitution[shapeA];
        float restB = world->shapes.shapeRestitution[shapeB];
        float rollA = world->shapes.shapeRollingResistance[shapeA];
        float rollB = world->shapes.shapeRollingResistance[shapeB];
        m3Vec3 surfA = world->shapes.shapeSurfaceVel[shapeA];
        m3Vec3 surfB = world->shapes.shapeSurfaceVel[shapeB];
        if (world->shapes.shapeType[shapeA] == (uint8_t)m3_meshShape)
        {
            const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shapeA]];
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
        if (world->shapes.shapeType[shapeB] == (uint8_t)m3_meshShape)
        {
            const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shapeB]];
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
        c->rollingResistance = m3MaxF(rollA, rollB) * m3MaxF(world->bodies.maxExtents[bodyA],
                                                             world->bodies.maxExtents[bodyB]);
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
    world->contacts.replayVetoCount = 0;
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
    if (world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody)
    {
        world->bodies.angularVelocities[c->bodyA] =
            m3Sub3(world->bodies.angularVelocities[c->bodyA], m3MulMV3(c->invIA, tw));
    }
    if (world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody)
    {
        world->bodies.angularVelocities[c->bodyB] =
            m3Add3(world->bodies.angularVelocities[c->bodyB], m3MulMV3(c->invIB, tw));
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
            m3Vec3 wRel = m3Sub3(world->bodies.angularVelocities[c->bodyB],
                                 world->bodies.angularVelocities[c->bodyA]);
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
            if (world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody)
            {
                world->bodies.angularVelocities[c->bodyA] =
                    m3Sub3(world->bodies.angularVelocities[c->bodyA], m3MulMV3(c->invIA, tw));
            }
            if (world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody)
            {
                world->bodies.angularVelocities[c->bodyB] =
                    m3Add3(world->bodies.angularVelocities[c->bodyB], m3MulMV3(c->invIB, tw));
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
            m3Vec3 wRel = m3Sub3(world->bodies.angularVelocities[c->bodyB],
                                 world->bodies.angularVelocities[c->bodyA]);
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
            if (world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody)
            {
                world->bodies.angularVelocities[c->bodyA] =
                    m3Sub3(world->bodies.angularVelocities[c->bodyA], m3MulMV3(c->invIA, delta));
            }
            if (world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody)
            {
                world->bodies.angularVelocities[c->bodyB] =
                    m3Add3(world->bodies.angularVelocities[c->bodyB], m3MulMV3(c->invIB, delta));
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

int m3BuildColoring(m3World* world, m3ContactConstraint* constraints, int32_t count,
                    m3SolverColoring* out)
{
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
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
        int dynA = world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody &&
                   world->bodies.awake[c->bodyA];
        int dynB = world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody &&
                   world->bodies.awake[c->bodyB];
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
void m3RunColored(m3World* world, const m3SolverColoring* coloring,
                  m3ContactConstraint* constraints, const m3Vec3* deltaPos, const m3Quat* deltaRot,
                  m3real invH, int useBias, int warmStartOnly)
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

void m3Restitution(m3World* world, m3ContactConstraint* constraints, int32_t count)
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
void m3StoreContactImpulses(m3World* world, m3ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = constraints + i;
        m3Manifold* manifold = &world->contacts.manifolds[c->manifoldIndex];
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            manifold->points[k].normalImpulse = c->points[k].normalImpulse;
        }

        // Hit events (8-5, the reference recipe): at most one per
        // contact, for the fastest-approaching point, only when a
        // side opted in and the impact actually fired.
        uint64_t key = world->contacts.pairKeys[c->manifoldIndex];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if (world->shapes.shapeHitEvents[shapeA] != 0 || world->shapes.shapeHitEvents[shapeB] != 0)
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
                if (world->events.hitEventCount < world->contacts.pairCapacity)
                {
                    m3HitEvent* e = &world->events.hitEvents[world->events.hitEventCount++];
                    e->shapeA = (m3ShapeId){shapeA + 1, world->worldIndex0,
                                            world->shapes.shapePool.generations[shapeA]};
                    e->shapeB = (m3ShapeId){shapeB + 1, world->worldIndex0,
                                            world->shapes.shapePool.generations[shapeB]};
                    m3Vec3 lcA = m3RotateVec3(world->bodies.transforms[c->bodyA].q,
                                              world->bodies.localCenters[c->bodyA]);
                    e->point.x = world->bodies.transforms[c->bodyA].p.x + (double)lcA.x +
                                 (double)c->points[best].rA.x;
                    e->point.y = world->bodies.transforms[c->bodyA].p.y + (double)lcA.y +
                                 (double)c->points[best].rA.y;
                    e->point.z = world->bodies.transforms[c->bodyA].p.z + (double)lcA.z +
                                 (double)c->points[best].rA.z;
                    e->normal = c->normal;
                    e->approachSpeed = -c->points[best].relativeVelocity;
                }
                else
                {
                    world->events.hitEventsDropped += 1;
                }
            }
        }
        manifold->frictionImpulse =
            m3Add3(m3MulSV3(c->frictionImpulse1, c->t1), m3MulSV3(c->frictionImpulse2, c->t2));
        manifold->twistImpulse = c->twistImpulse;
        manifold->rollingImpulse = c->rollingImpulse;
    }
}
