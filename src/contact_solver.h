// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact constraints: one per touching manifold that can move. Each has
// a normal row per point, and at the manifold's center one friction row
// pair, a twist row about the normal and a rolling row. Built by
// contact_prepare.c, solved by graph color in contact_solver.c.

#ifndef MAUL3D_SRC_CONTACT_SOLVER_H
#define MAUL3D_SRC_CONTACT_SOLVER_H

#include "solver.h"
#include "world_internal.h"

// Constraints in one graph color share no awake dynamic body, so a color
// may solve in any split without moving a bit. Colors are assigned
// greedily in canonical constraint order; what does not fit the palette
// lands in the overflow, solved serially.
#define M3_GRAPH_COLORS 16

typedef struct m3ConstraintPoint
{
    m3Vec3 rA; // anchors from each center of mass, fixed at prepare
    m3Vec3 rB;
    m3real baseSeparation; // separation minus the anchors' normal gap at prepare
    m3real normalMass;
    m3real leverArm;         // distance from the manifold center: the twist budget arm
    m3real relativeVelocity; // normal speed at prepare, negative when closing
    m3real normalImpulse;
    // Summed over every pass of the step: a zero means the point never
    // pushed, and restitution skips it. The friction budgets use the
    // current pass's impulses instead; a sum across passes would inflate
    // the cone several times over.
    m3real totalNormalImpulse;
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
    // Friction acts at the mean of the anchors, not per point: friction
    // at each corner of a box gives gravity a false pitching lever.
    m3Vec3 originA;
    m3Vec3 originB;
    m3real frictionK11; // inverse of the 2x2 tangent mass, symmetric
    m3real frictionK12;
    m3real frictionK22;
    m3real frictionImpulse1;
    m3real frictionImpulse2;
    m3real twistMass;
    m3real twistImpulse;
    m3real tangentVelocity1; // conveyor target along t1 and t2
    m3real tangentVelocity2;
    m3real friction;
    m3real restitution;
    m3real invMassA;
    m3real invMassB;
    m3Mat3 invIA; // world inverse inertia, fixed at prepare
    m3Mat3 invIB;
    m3Softness softness;
    m3real rollingResistance; // mixed by maximum, scaled by the pair's extent
    m3Vec3 rollingImpulse;
    m3Mat3 rollingK; // iA + iB
    m3ConstraintPoint points[M3_MANIFOLD_MAX_POINTS];
} m3ContactConstraint;

typedef struct m3SolverColoring
{
    uint8_t* colors; // per constraint
    int32_t* lists;  // constraint indices grouped by color
    int32_t starts[M3_GRAPH_COLORS + 2];
} m3SolverColoring;

struct m3ContactBlock;

// A step's contact work. The colored constraints also sit in lane
// blocks (contact_kernel.h), which carry their impulses through the
// substeps; without scratch for them every row runs scalar, with the
// same result.
typedef struct m3ContactPlan
{
    m3ContactConstraint* constraints;
    int32_t count;
    m3SolverColoring coloring;
    struct m3ContactBlock* blocks;            // NULL: scalar rows only
    int32_t blockStarts[M3_GRAPH_COLORS + 1]; // per color, then the end
    const m3Vec3* deltaPos; // how far each body moved and turned since the step began
    const m3Quat* deltaRot;
    m3real invH;
} m3ContactPlan;

typedef enum m3ContactStage
{
    m3_contactWarmStart,
    m3_contactSolve,       // normal rows push overlap out, nothing else
    m3_contactRelax,       // rigid normal rows, then twist, rolling and friction
    m3_contactRestitution, // bounce, once per step, canonical order
    m3_contactStore,       // impulses back to the manifolds, hit events
} m3ContactStage;

int32_t m3PrepareContacts(m3World* world, m3ContactConstraint* constraints, m3real h);

// Colors the prepared constraints. Returns false on a scratch stall.
bool m3ColorContacts(m3World* world, m3ContactPlan* plan);

void m3RunContactStage(m3World* world, const m3ContactPlan* plan, m3ContactStage stage);

#endif // MAUL3D_SRC_CONTACT_SOLVER_H
