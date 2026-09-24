// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint rows. Every joint is a list of scalar rows, each a linear
// function J of the two bodies' velocities that the solver drives
// toward a target:
//   J v = linA . vA + angA . wA + linB . vB + angB . wB.
// An impulse lambda on a row changes the velocities by M^-1 J^T lambda.
// The kinds build their rows from the current pose every pass; this
// module solves them alone, as coupled pairs, and as the two 3x3 blocks
// that pin a point and lock a rotation.

#ifndef MAUL3D_SRC_JOINT_ROW_H
#define MAUL3D_SRC_JOINT_ROW_H

#include "solver.h"

// The two bodies of a joint as one pass sees them.
typedef struct m3JointBodies
{
    m3Vec3 vA;
    m3Vec3 wA;
    m3Vec3 vB;
    m3Vec3 wB;
    m3real mA;
    m3real mB;
    m3Mat3 iA; // world inverse inertia
    m3Mat3 iB;
} m3JointBodies;

typedef struct m3JointRow
{
    m3Vec3 linA;
    m3Vec3 angA;
    m3Vec3 linB;
    m3Vec3 angB;
} m3JointRow;

// How a row is driven in one pass. The impulse added is
//   -mass (scale J v + bias) - leak accumulated.
typedef struct m3RowDrive
{
    m3real bias;
    m3real scale;
    m3real leak;
} m3RowDrive;

// No bound on an accumulated impulse.
#define M3_ROW_FREE 3.0e38f

// B's point minus A's point moving along u, where u is fixed in the world
// (leverA = A's arm) or turns with A (leverA = A's arm plus the gap).
m3JointRow m3LineRow(m3Vec3 leverA, m3Vec3 armB, m3Vec3 u);

// B's spin minus A's about axis.
m3JointRow m3TurnRow(m3Vec3 axis);

m3JointRow m3ScaleRow(m3JointRow row, m3real s);

m3real m3RowSpeed(const m3JointRow* row, const m3JointBodies* b);

// The row's effective mass, zero when the row cannot move.
m3real m3RowMass(const m3JointRow* row, const m3JointBodies* b);

void m3PushRow(const m3JointRow* row, m3JointBodies* b, m3real impulse);

// Solves one row with its accumulated impulse held in [lo, hi]. Returns
// the impulse applied.
m3real m3SolveRow(const m3JointRow* row, m3JointBodies* b, m3RowDrive drive, m3real* accumulated,
                  m3real lo, m3real hi);

// Solves two coupled rows, both driven with the same scale and leak and
// each with its own bias. A singular pair falls back to the rows one at
// a time.
void m3SolveRowPair(const m3JointRow rows[2], m3JointBodies* b, const m3real bias[2],
                    m3RowDrive drive, m3real* first, m3real* second);

// The three rows that pin B's anchor to A's, solved as one block, with
// the accumulated impulse held within a ball of radius budget.
void m3SolvePointBlock(m3Vec3 armA, m3Vec3 armB, m3JointBodies* b, m3Vec3 bias, m3RowDrive drive,
                       m3Vec3* accumulated, m3real budget);
void m3PushPoint(m3Vec3 armA, m3Vec3 armB, m3JointBodies* b, m3Vec3 impulse);

// The three rows that lock B's rotation to A's, solved as one block,
// with the accumulated impulse held within a ball of radius budget.
void m3SolveTurnBlock(m3JointBodies* b, m3Vec3 bias, m3RowDrive drive, m3Vec3* accumulated,
                      m3real budget);
void m3PushTurn(m3JointBodies* b, m3Vec3 impulse);

// Drives. A rigid row meets bias exactly; a held row pulls its error C
// out through the softness while biased and is rigid while relaxing; a
// spring pulls in every pass; a limit is speculative while open
// (C > 0), held while violated.
m3RowDrive m3RigidDrive(m3real bias);
m3RowDrive m3SpringDrive(m3Softness soft, m3real C);
m3RowDrive m3HeldDrive(m3Softness soft, m3real C, bool biased);
m3RowDrive m3LimitDrive(m3Softness soft, m3real C, m3real invH, bool biased);

#endif // MAUL3D_SRC_JOINT_ROW_H
