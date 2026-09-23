// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The contact solver: soft contact constraints with friction, twist
// and rolling rows, solved in graph colors, then restitution and the
// impulse store.

#ifndef MAUL3D_SRC_CONTACT_SOLVER_H
#define MAUL3D_SRC_CONTACT_SOLVER_H

#include "solver.h"
#include "world_internal.h"

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
    // Central friction: one 2x2 row
    // at the mean anchors plus a twist row about the normal, instead
    // of per-point tangent rows. Per-corner friction on a box gave
    // gravity a fake pitch lever.
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

typedef struct m3SolverColoring
{
    uint8_t* colors; // per constraint
    int32_t* lists;  // constraint indices grouped by color
    int32_t starts[M3_GRAPH_COLORS + 2];
} m3SolverColoring;

int32_t m3PrepareContacts(m3World* world, m3ContactConstraint* constraints, m3real h);
int m3BuildColoring(m3World* world, m3ContactConstraint* constraints, int32_t count,
                    m3SolverColoring* out);
void m3RunColored(m3World* world, const m3SolverColoring* coloring,
                  m3ContactConstraint* constraints, const m3Vec3* deltaPos, const m3Quat* deltaRot,
                  m3real invH, int useBias, int warmStartOnly);
void m3Restitution(m3World* world, m3ContactConstraint* constraints, int32_t count);
void m3StoreContactImpulses(m3World* world, m3ContactConstraint* constraints, int32_t count);

#endif // MAUL3D_SRC_CONTACT_SOLVER_H
