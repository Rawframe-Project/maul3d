// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint rows: building, measuring, applying and solving them.

#include "joint_row.h"

#include <math.h>

m3JointRow m3LineRow(m3Vec3 leverA, m3Vec3 armB, m3Vec3 u)
{
    m3JointRow row = {m3MulSV3(-1.0f, u), m3MulSV3(-1.0f, m3Cross3(leverA, u)), u,
                      m3Cross3(armB, u)};
    return row;
}

m3JointRow m3TurnRow(m3Vec3 axis)
{
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    m3JointRow row = {zero, m3MulSV3(-1.0f, axis), zero, axis};
    return row;
}

m3JointRow m3ScaleRow(m3JointRow row, m3real s)
{
    m3JointRow scaled = {m3MulSV3(s, row.linA), m3MulSV3(s, row.angA), m3MulSV3(s, row.linB),
                         m3MulSV3(s, row.angB)};
    return scaled;
}

m3real m3RowSpeed(const m3JointRow* row, const m3JointBodies* b)
{
    return m3Dot3(row->linA, b->vA) + m3Dot3(row->angA, b->wA) + m3Dot3(row->linB, b->vB) +
           m3Dot3(row->angB, b->wB);
}

// J M^-1 J'^T for two rows.
static m3real Coupling(const m3JointRow* r, const m3JointRow* s, const m3JointBodies* b)
{
    return b->mA * m3Dot3(r->linA, s->linA) + m3Dot3(r->angA, m3MulMV3(b->iA, s->angA)) +
           b->mB * m3Dot3(r->linB, s->linB) + m3Dot3(r->angB, m3MulMV3(b->iB, s->angB));
}

m3real m3RowMass(const m3JointRow* row, const m3JointBodies* b)
{
    m3real k = Coupling(row, row, b);
    return k > 0.0f ? 1.0f / k : 0.0f;
}

void m3PushRow(const m3JointRow* row, m3JointBodies* b, m3real impulse)
{
    b->vA = m3Add3(b->vA, m3MulSV3(b->mA * impulse, row->linA));
    b->wA = m3Add3(b->wA, m3MulMV3(b->iA, m3MulSV3(impulse, row->angA)));
    b->vB = m3Add3(b->vB, m3MulSV3(b->mB * impulse, row->linB));
    b->wB = m3Add3(b->wB, m3MulMV3(b->iB, m3MulSV3(impulse, row->angB)));
}

m3real m3SolveRow(const m3JointRow* row, m3JointBodies* b, m3RowDrive drive, m3real* accumulated,
                  m3real lo, m3real hi)
{
    m3real mass = m3RowMass(row, b);
    if (mass == 0.0f)
    {
        return 0.0f;
    }
    m3real old = *accumulated;
    m3real speed = m3RowSpeed(row, b);
    m3real next = old - mass * (drive.scale * speed + drive.bias) - drive.leak * old;
    next = next < lo ? lo : (next > hi ? hi : next);
    *accumulated = next;
    m3PushRow(row, b, next - old);
    return next - old;
}

void m3SolveRowPair(const m3JointRow rows[2], m3JointBodies* b, const m3real bias[2],
                    m3RowDrive drive, m3real* first, m3real* second)
{
    m3real k11 = Coupling(&rows[0], &rows[0], b);
    m3real k12 = Coupling(&rows[0], &rows[1], b);
    m3real k22 = Coupling(&rows[1], &rows[1], b);
    m3real det = k11 * k22 - k12 * k12;
    if (!(det > 0.0f))
    {
        m3RowDrive one = {bias[0], drive.scale, drive.leak};
        m3RowDrive two = {bias[1], drive.scale, drive.leak};
        m3SolveRow(&rows[0], b, one, first, -M3_ROW_FREE, M3_ROW_FREE);
        m3SolveRow(&rows[1], b, two, second, -M3_ROW_FREE, M3_ROW_FREE);
        return;
    }
    m3real x = drive.scale * m3RowSpeed(&rows[0], b) + bias[0];
    m3real y = drive.scale * m3RowSpeed(&rows[1], b) + bias[1];
    m3real inv = 1.0f / det;
    m3real dx = -inv * (k22 * x - k12 * y) - drive.leak * *first;
    m3real dy = -inv * (k11 * y - k12 * x) - drive.leak * *second;
    *first += dx;
    *second += dy;
    m3PushRow(&rows[0], b, dx);
    m3PushRow(&rows[1], b, dy);
}

// Keeps an accumulated impulse within a ball; returns the applied part.
static m3Vec3 WithinBall(m3Vec3* accumulated, m3Vec3 impulse, m3real budget)
{
    m3Vec3 old = *accumulated;
    m3Vec3 next = m3Add3(old, impulse);
    if (budget < M3_ROW_FREE)
    {
        m3real length2 = m3Dot3(next, next);
        if (length2 > budget * budget)
        {
            next = m3MulSV3(budget / sqrtf(length2), next);
        }
    }
    *accumulated = next;
    return m3Sub3(next, old);
}

void m3PushPoint(m3Vec3 armA, m3Vec3 armB, m3JointBodies* b, m3Vec3 impulse)
{
    b->vA = m3Sub3(b->vA, m3MulSV3(b->mA, impulse));
    b->wA = m3Sub3(b->wA, m3MulMV3(b->iA, m3Cross3(armA, impulse)));
    b->vB = m3Add3(b->vB, m3MulSV3(b->mB, impulse));
    b->wB = m3Add3(b->wB, m3MulMV3(b->iB, m3Cross3(armB, impulse)));
}

// K = (mA + mB) I - [rA] IA [rA] - [rB] IB [rB], column by column.
void m3SolvePointBlock(m3Vec3 armA, m3Vec3 armB, m3JointBodies* b, m3Vec3 bias, m3RowDrive drive,
                       m3Vec3* accumulated, m3real budget)
{
    m3Mat3 k;
    const m3Vec3 basis[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    m3Vec3* columns[3] = {&k.cx, &k.cy, &k.cz};
    for (int32_t a = 0; a < 3; ++a)
    {
        m3Vec3 e = basis[a];
        m3Vec3 turnA = m3Cross3(m3MulMV3(b->iA, m3Cross3(armA, e)), armA);
        m3Vec3 turnB = m3Cross3(m3MulMV3(b->iB, m3Cross3(armB, e)), armB);
        *columns[a] = m3Add3(m3MulSV3(b->mA + b->mB, e), m3Add3(turnA, turnB));
    }
    m3Vec3 speed =
        m3Sub3(m3Add3(b->vB, m3Cross3(b->wB, armB)), m3Add3(b->vA, m3Cross3(b->wA, armA)));
    m3Vec3 rhs = m3Add3(m3MulSV3(drive.scale, speed), bias);
    m3Vec3 impulse = m3Sub3(m3MulSV3(-1.0f, m3Solve3(&k, rhs)), m3MulSV3(drive.leak, *accumulated));
    m3PushPoint(armA, armB, b, WithinBall(accumulated, impulse, budget));
}

void m3PushTurn(m3JointBodies* b, m3Vec3 impulse)
{
    b->wA = m3Sub3(b->wA, m3MulMV3(b->iA, impulse));
    b->wB = m3Add3(b->wB, m3MulMV3(b->iB, impulse));
}

void m3SolveTurnBlock(m3JointBodies* b, m3Vec3 bias, m3RowDrive drive, m3Vec3* accumulated,
                      m3real budget)
{
    m3Mat3 k = {m3Add3(b->iA.cx, b->iB.cx), m3Add3(b->iA.cy, b->iB.cy), m3Add3(b->iA.cz, b->iB.cz)};
    m3Vec3 rhs = m3Add3(m3MulSV3(drive.scale, m3Sub3(b->wB, b->wA)), bias);
    m3Vec3 impulse = m3Sub3(m3MulSV3(-1.0f, m3Solve3(&k, rhs)), m3MulSV3(drive.leak, *accumulated));
    m3PushTurn(b, WithinBall(accumulated, impulse, budget));
}

m3RowDrive m3RigidDrive(m3real bias)
{
    m3RowDrive drive = {bias, 1.0f, 0.0f};
    return drive;
}

m3RowDrive m3SpringDrive(m3Softness soft, m3real C)
{
    m3RowDrive drive = {soft.massScale * soft.biasRate * C, soft.massScale, soft.impulseScale};
    return drive;
}

m3RowDrive m3HeldDrive(m3Softness soft, m3real C, bool biased)
{
    return biased ? m3SpringDrive(soft, C) : m3RigidDrive(0.0f);
}

m3RowDrive m3LimitDrive(m3Softness soft, m3real C, m3real invH, bool biased)
{
    return C > 0.0f ? m3RigidDrive(C * invH) : m3HeldDrive(soft, C, biased);
}
