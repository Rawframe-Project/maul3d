// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact preparation: which touching pairs get a constraint this step,
// the surface material the pair wears, and each constraint's rows at
// the step's starting pose.

#include "contact_solver.h"

#include "manifold.h"
#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static bool AwakeDynamic(const m3World* world, int32_t body)
{
    return world->bodies.types[body] == (uint8_t)m3_dynamicBody && world->bodies.awake[body] != 0;
}

static bool SortedContains(const uint64_t* keys, int32_t count, uint64_t key)
{
    int32_t lo = 0;
    int32_t hi = count - 1;
    while (lo <= hi)
    {
        int32_t mid = (lo + hi) / 2;
        if (keys[mid] == key)
        {
            return true;
        }
        if (keys[mid] < key)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid - 1;
        }
    }
    return false;
}

// The world position of the manifold's deepest point, on B's side.
static m3Pos3 DeepestPoint(const m3World* world, const m3Manifold* manifold, int32_t bodyB)
{
    int32_t deep = 0;
    for (int32_t k = 1; k < manifold->pointCount; ++k)
    {
        if (manifold->points[k].separation < manifold->points[deep].separation)
        {
            deep = k;
        }
    }
    m3Transform t = world->bodies.transforms[bodyB];
    m3Vec3 center = m3RotateVec3(t.q, world->bodies.localCenters[bodyB]);
    m3Vec3 anchor = manifold->points[deep].anchorB;
    m3Pos3 point = {t.p.x + (double)center.x + (double)anchor.x,
                    t.p.y + (double)center.y + (double)anchor.y,
                    t.p.z + (double)center.z + (double)anchor.z};
    return point;
}

// True when the host's pre-solve callback turns the pair down. A replay
// carries the recorded decisions instead and never calls the host, so a
// tape's outcome cannot be second-guessed; a live veto joins the
// journal so a replay repeats it. Keys arrive in canonical ascending
// order in both cases. The callback must be pure: replay divergence
// from an impure one is the host's own.
static bool Vetoed(m3World* world, const m3Manifold* manifold, uint64_t key, int32_t shapeA,
                   int32_t shapeB)
{
    if (world->contacts.replayVetoCount > 0)
    {
        return SortedContains(world->contacts.replayVetoKeys, world->contacts.replayVetoCount, key);
    }
    if (world->preSolveFn == NULL ||
        (world->shapes.shapePreSolve[shapeA] == 0 && world->shapes.shapePreSolve[shapeB] == 0))
    {
        return false;
    }
    m3Pos3 point = DeepestPoint(world, manifold, world->shapes.shapeBody[shapeB]);
    m3ShapeId idA = {shapeA + 1, world->idWorld, world->shapes.shapePool.generations[shapeA]};
    m3ShapeId idB = {shapeB + 1, world->idWorld, world->shapes.shapePool.generations[shapeB]};
    if (world->preSolveFn(idA, idB, point, manifold->normal, world->preSolveContext))
    {
        return false;
    }
    if (world->recorder.journalActive != 0 &&
        world->contacts.stepVetoCount < world->contacts.pairCapacity)
    {
        world->contacts.stepVetoKeys[world->contacts.stepVetoCount++] = key;
    }
    return true;
}

typedef struct Surface
{
    float friction;
    float restitution;
    float rolling;
    m3Vec3 velocity;
} Surface;

// A shape's surface. A mesh with materials wears the entry of the
// struck triangle's group, carried in the first point's flags (points
// are in canonical order, and welded points share a face); a group out
// of range, as a hostile restored manifold could hold, falls to group 0.
static Surface ShapeSurface(const m3World* world, int32_t shape, const m3Manifold* manifold)
{
    Surface s = {world->shapes.shapeFriction[shape], world->shapes.shapeRestitution[shape],
                 world->shapes.shapeRollingResistance[shape], world->shapes.shapeSurfaceVel[shape]};
    if (world->shapes.shapeType[shape] == (uint8_t)m3_meshShape)
    {
        const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
        if (mesh->materialCount > 0)
        {
            int32_t group = manifold->points[0].flags >> 12;
            const m3MeshSurfaceMaterial* m =
                &mesh->materials[group < mesh->materialCount ? group : 0];
            s = (Surface){m->friction, m->restitution, m->rollingResistance, m->surfaceVelocity};
        }
    }
    return s;
}

// Friction mixes by geometric mean, restitution by maximum, rolling
// resistance by maximum times the larger body extent (the lever that
// turns the dimensionless knob into a torque). Friction drives B's
// tangential speed relative to A toward the surfaces' difference, so a
// belt on A carries B along.
static void PrepareMaterial(const m3World* world, m3ContactConstraint* c,
                            const m3Manifold* manifold, int32_t shapeA, int32_t shapeB)
{
    Surface a = ShapeSurface(world, shapeA, manifold);
    Surface b = ShapeSurface(world, shapeB, manifold);
    c->friction = sqrtf(a.friction * b.friction);
    c->restitution = m3MaxF(a.restitution, b.restitution);
    c->rollingResistance =
        m3MaxF(a.rolling, b.rolling) *
        m3MaxF(world->bodies.maxExtents[c->bodyA], world->bodies.maxExtents[c->bodyB]);
    m3Vec3 belt = m3Sub3(a.velocity, b.velocity);
    c->tangentVelocity1 = m3Dot3(belt, c->t1);
    c->tangentVelocity2 = m3Dot3(belt, c->t2);
}

static m3Vec3 PointVelocity(const m3World* world, int32_t body, m3Vec3 arm)
{
    m3Vec3 v = world->bodies.linearVelocities[body];
    m3Vec3 w = world->bodies.angularVelocities[body];
    return m3Add3(v, m3Cross3(w, arm));
}

// The inverse mass a row along dir through the arms meets; zero for a
// row that cannot move.
static m3real RowMass(const m3ContactConstraint* c, m3Vec3 rA, m3Vec3 rB, m3Vec3 dir)
{
    m3Vec3 turnA = m3Cross3(rA, dir);
    m3Vec3 turnB = m3Cross3(rB, dir);
    m3real k = c->invMassA + c->invMassB + m3Dot3(turnA, m3MulMV3(c->invIA, turnA)) +
               m3Dot3(turnB, m3MulMV3(c->invIB, turnB));
    return k > 0.0f ? 1.0f / k : 0.0f;
}

// The normal rows. The solver measures separation as the base plus the
// normal part of the anchors' drift, so the anchors' normal gap at
// prepare is taken out of the base here once.
static void PreparePoints(const m3World* world, m3ContactConstraint* c, const m3Manifold* manifold)
{
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        m3ConstraintPoint* cp = &c->points[k];
        const m3ManifoldPoint* mp = &manifold->points[k];
        cp->rA = mp->anchorA;
        cp->rB = mp->anchorB;
        cp->baseSeparation =
            mp->separation - (m3Dot3(cp->rB, c->normal) - m3Dot3(cp->rA, c->normal));
        cp->normalMass = RowMass(c, cp->rA, cp->rB, c->normal);
        m3Vec3 vA = PointVelocity(world, c->bodyA, cp->rA);
        m3Vec3 vB = PointVelocity(world, c->bodyB, cp->rB);
        cp->relativeVelocity = m3Dot3(m3Sub3(vB, vA), c->normal);
        cp->normalImpulse = mp->normalImpulse;
        cp->totalNormalImpulse = 0.0f;
    }
}

// The rows at the manifold's center: the tangent pair, inverted once,
// whose warm impulse comes from the manifold's world-frame store so a
// new tangent basis cannot scramble it; the twist row about the normal
// with each point's lever arm for its budget; and the rolling row.
static void PrepareCenterRows(m3ContactConstraint* c, const m3Manifold* manifold)
{
    m3Vec3 sumA = {0.0f, 0.0f, 0.0f};
    m3Vec3 sumB = {0.0f, 0.0f, 0.0f};
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        sumA = m3Add3(sumA, c->points[k].rA);
        sumB = m3Add3(sumB, c->points[k].rB);
    }
    m3real share = 1.0f / (m3real)c->pointCount;
    c->originA = m3MulSV3(share, sumA);
    c->originB = m3MulSV3(share, sumB);
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        c->points[k].leverArm = m3Length3(m3Sub3(c->points[k].rA, c->originA));
    }

    m3Vec3 a1 = m3Cross3(c->originA, c->t1);
    m3Vec3 a2 = m3Cross3(c->originA, c->t2);
    m3Vec3 b1 = m3Cross3(c->originB, c->t1);
    m3Vec3 b2 = m3Cross3(c->originB, c->t2);
    m3real k11 = c->invMassA + c->invMassB + m3Dot3(a1, m3MulMV3(c->invIA, a1)) +
                 m3Dot3(b1, m3MulMV3(c->invIB, b1));
    m3real k22 = c->invMassA + c->invMassB + m3Dot3(a2, m3MulMV3(c->invIA, a2)) +
                 m3Dot3(b2, m3MulMV3(c->invIB, b2));
    m3real k12 = m3Dot3(a1, m3MulMV3(c->invIA, a2)) + m3Dot3(b1, m3MulMV3(c->invIB, b2));
    m3real det = k11 * k22 - k12 * k12;
    m3real inv = det > 0.0f ? 1.0f / det : 0.0f;
    c->frictionK11 = det > 0.0f ? k22 * inv : 0.0f;
    c->frictionK22 = det > 0.0f ? k11 * inv : 0.0f;
    c->frictionK12 = det > 0.0f ? -k12 * inv : 0.0f;
    c->frictionImpulse1 = m3Dot3(manifold->frictionImpulse, c->t1);
    c->frictionImpulse2 = m3Dot3(manifold->frictionImpulse, c->t2);

    m3Vec3 spin = m3Add3(m3MulMV3(c->invIA, c->normal), m3MulMV3(c->invIB, c->normal));
    m3real twistK = m3Dot3(c->normal, spin);
    c->twistMass = twistK > 0.0f ? 1.0f / twistK : 0.0f;
    c->twistImpulse = manifold->twistImpulse;

    c->rollingImpulse = manifold->rollingImpulse;
    if (c->rollingResistance > 0.0f)
    {
        c->rollingK.cx = m3Add3(c->invIA.cx, c->invIB.cx);
        c->rollingK.cy = m3Add3(c->invIA.cy, c->invIB.cy);
        c->rollingK.cz = m3Add3(c->invIA.cz, c->invIB.cz);
    }
}

static void PrepareConstraint(const m3World* world, m3ContactConstraint* c, int32_t pair,
                              const m3Softness soft[2])
{
    const m3Manifold* manifold = &world->contacts.manifolds[pair];
    uint64_t key = world->contacts.pairKeys[pair];
    int32_t shapeA = (int32_t)(key >> 32);
    int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
    memset(c, 0, sizeof(*c));
    c->bodyA = world->shapes.shapeBody[shapeA];
    c->bodyB = world->shapes.shapeBody[shapeB];
    c->manifoldIndex = pair;
    c->pointCount = manifold->pointCount;
    c->normal = manifold->normal;
    m3MakeTangentBasis(c->normal, &c->t1, &c->t2);
    bool dynamicA = world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody;
    bool dynamicB = world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody;
    c->invMassA = dynamicA ? world->bodies.invMass[c->bodyA] : 0.0f;
    c->invMassB = dynamicB ? world->bodies.invMass[c->bodyB] : 0.0f;
    c->invIA = m3WorldInvInertia(world, c->bodyA);
    c->invIB = m3WorldInvInertia(world, c->bodyB);
    // A contact against something immovable is stiffer: a soft ground row
    // stores energy under a tall stack.
    c->softness = soft[c->invMassA == 0.0f || c->invMassB == 0.0f ? 1 : 0];
    PrepareMaterial(world, c, manifold, shapeA, shapeB);
    PreparePoints(world, c, manifold);
    PrepareCenterRows(c, manifold);
}

int32_t m3PrepareContacts(m3World* world, m3ContactConstraint* constraints, m3real h)
{
    m3Softness soft[2] = {
        m3MakeSoft(world->contactHertz, world->contactDampingRatio, h),
        m3MakeSoft(2.0f * world->contactHertz, world->contactDampingRatio, h),
    };
    int32_t count = 0;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        const m3Manifold* manifold = &world->contacts.manifolds[i];
        uint64_t key = world->contacts.pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if (manifold->pointCount == 0 || (!AwakeDynamic(world, world->shapes.shapeBody[shapeA]) &&
                                          !AwakeDynamic(world, world->shapes.shapeBody[shapeB])))
        {
            continue; // nothing touching, or nothing awake to move
        }
        if (world->shapes.shapeSensor[shapeA] != 0 || world->shapes.shapeSensor[shapeB] != 0)
        {
            continue; // sensors observe, they never push
        }
        if (Vetoed(world, manifold, key, shapeA, shapeB))
        {
            continue;
        }
        PrepareConstraint(world, constraints + count, i, soft);
        count += 1;
    }
    // Recorded vetoes cover exactly this step.
    world->contacts.replayVetoCount = 0;
    return count;
}
