// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The contact kernel: lane blocks of contact constraints and the one
// kernel that runs the warm start, solve and relax stages on them.

#ifndef MAUL3D_SRC_CONTACT_KERNEL_H
#define MAUL3D_SRC_CONTACT_KERNEL_H

#include "contact_solver.h"

// Lane width is a layout constant, never a semantics knob: every lane
// runs the scalar rows' exact IEEE sequence, so how constraints are
// packed into lanes cannot move a bit.
#define M3_LANES 8

// One contact point of every lane. A lane whose constraint has fewer
// points carries zeros here and an inactive mask.
typedef struct m3LanePoint
{
    float rA[3][M3_LANES];
    float rB[3][M3_LANES];
    float baseSeparation[M3_LANES];
    float normalMass[M3_LANES];
    float leverArm[M3_LANES];
    float normalImpulse[M3_LANES];
    float totalNormalImpulse[M3_LANES];
    float active[M3_LANES]; // all bits set where the lane has this point
} m3LanePoint;

// Up to M3_LANES constraints of one color: they share no dynamic body.
// Unused lanes repeat lane 0's bodies with both sides inert, so they
// read valid velocities and write none. Matrices are stored by column.
typedef struct m3ContactBlock
{
    int32_t lanes;
    int32_t pointCount; // the most points of any lane
    int32_t bodyA[M3_LANES];
    int32_t bodyB[M3_LANES];
    int32_t constraint[M3_LANES];
    float movesA[M3_LANES]; // all bits set where the side is dynamic
    float movesB[M3_LANES];
    float normal[3][M3_LANES];
    float t1[3][M3_LANES];
    float t2[3][M3_LANES];
    float originA[3][M3_LANES];
    float originB[3][M3_LANES];
    float frictionK11[M3_LANES];
    float frictionK12[M3_LANES];
    float frictionK22[M3_LANES];
    float frictionImpulse1[M3_LANES];
    float frictionImpulse2[M3_LANES];
    float twistMass[M3_LANES];
    float twistImpulse[M3_LANES];
    float tangentVelocity1[M3_LANES];
    float tangentVelocity2[M3_LANES];
    float friction[M3_LANES];
    float invMassA[M3_LANES];
    float invMassB[M3_LANES];
    float invIA[9][M3_LANES];
    float invIB[9][M3_LANES];
    float biasRate[M3_LANES];
    float massScale[M3_LANES];
    float impulseScale[M3_LANES];
    float rollingResistance[M3_LANES];
    float rollingImpulse[3][M3_LANES];
    float rollingK[9][M3_LANES];
    m3LanePoint points[M3_MANIFOLD_MAX_POINTS];
} m3ContactBlock;

// Lanes are packed into a zeroed block: lane 0 first, then the rest,
// padding after the last constraint.
void m3PackContactLane(const m3World* world, m3ContactBlock* block, int32_t lane,
                       const m3ContactConstraint* c, int32_t index);
void m3PadContactLane(m3ContactBlock* block, int32_t lane);

// The impulses of every lane back into their constraints.
void m3UnpackContactBlock(const m3ContactBlock* block, m3ContactConstraint* constraints);

// Runs the warm start, solve or relax stage on one block. The block's
// constraints share no dynamic body, so blocks of one color may run at
// once.
void m3RunContactBlock(m3World* world, const m3ContactPlan* plan, m3ContactBlock* block,
                       m3ContactStage stage);

#endif // MAUL3D_SRC_CONTACT_KERNEL_H
