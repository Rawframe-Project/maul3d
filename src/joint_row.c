// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint rows: building, measuring, applying and solving them.

#include "joint_row.h"

#include "solver_math.h"

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

// The same coupling in double.
static double WideCoupling(const m3JointRow* r, const m3JointRow* s, const m3JointBodies* b)
{
    return (double)b->mA * m3WideDot(r->linA, s->linA) + m3WideForm(&b->iA, r->angA, s->angA) +
           (double)b->mB * m3WideDot(r->linB, s->linB) + m3WideForm(&b->iB, r->angB, s->angB);
}

m3real m3RowMass(const m3JointRow* row, const m3JointBodies* b)
{
    m3real k = Coupling(row, row, b);
    return k > 0.0f ? 1.0f / k : 0.0f;
}

// A row's correction speed, held within the speed limit: an error of any
// size is pushed out no faster than a body may move, which keeps every
// solve below finite. A bias within the limit is returned unchanged.
static m3real CapBias(m3real bias)
{
    return m3ClampF(bias, -M3_SPEED_LIMIT, M3_SPEED_LIMIT);
}

static m3Vec3 CapBias3(m3Vec3 bias)
{
    m3Vec3 capped = {CapBias(bias.x), CapBias(bias.y), CapBias(bias.z)};
    return capped;
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
    m3real next = old - mass * (drive.scale * speed + CapBias(drive.bias)) - drive.leak * old;
    m3real bound = mass * M3_ROW_SPEED_BOUND;
    lo = m3MaxF(lo, -bound);
    hi = m3MinF(hi, bound);
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
    m3real x = drive.scale * m3RowSpeed(&rows[0], b) + CapBias(bias[0]);
    m3real y = drive.scale * m3RowSpeed(&rows[1], b) + CapBias(bias[1]);
    // The direct result stands for every ordinary pair, so it keeps its
    // bits; a pair whose float determinant underflows or cancels is solved
    // with its couplings taken in double (m3DirectHolds).
    m3real solved[2];
    if (m3DirectHolds(det, k11 * k22))
    {
        m3real inv = 1.0f / det;
        solved[0] = inv * (k22 * x - k12 * y);
        solved[1] = inv * (k11 * y - k12 * x);
    }
    else if (!m3SolvePairWide(WideCoupling(&rows[0], &rows[0], b),
                              WideCoupling(&rows[1], &rows[1], b),
                              WideCoupling(&rows[0], &rows[1], b), x, y, solved))
    {
        m3RowDrive one = {bias[0], drive.scale, drive.leak};
        m3RowDrive two = {bias[1], drive.scale, drive.leak};
        m3SolveRow(&rows[0], b, one, first, -M3_ROW_FREE, M3_ROW_FREE);
        m3SolveRow(&rows[1], b, two, second, -M3_ROW_FREE, M3_ROW_FREE);
        return;
    }
    m3real dx = -solved[0] - drive.leak * *first;
    m3real dy = -solved[1] - drive.leak * *second;
    // Each row's share held within the speed bound at the row's own mass.
    m3real boundX = M3_ROW_SPEED_BOUND / k11;
    m3real boundY = M3_ROW_SPEED_BOUND / k22;
    m3real nextX = *first + dx;
    m3real nextY = *second + dy;
    if (m3AbsF(nextX) > boundX)
    {
        dx = m3ClampF(nextX, -boundX, boundX) - *first;
    }
    if (m3AbsF(nextY) > boundY)
    {
        dy = m3ClampF(nextY, -boundY, boundY) - *second;
    }
    *first += dx;
    *second += dy;
    m3PushRow(&rows[0], b, dx);
    m3PushRow(&rows[1], b, dy);
}

// Keeps an accumulated impulse within a ball: the budget, and the speed
// bound at the largest mass the block shows along a single axis (k is
// the block's inverse mass). Returns the applied part.
static m3Vec3 WithinBall(m3Vec3* accumulated, m3Vec3 impulse, m3real budget, const m3Mat3* k)
{
    m3real minInverse = m3MinF(k->cx.x, m3MinF(k->cy.y, k->cz.z));
    m3real bound = minInverse > 0.0f ? M3_ROW_SPEED_BOUND / minInverse : M3_ROW_FREE;
    m3real limit = m3MinF(budget, bound);
    m3Vec3 old = *accumulated;
    m3Vec3 next = m3Add3(old, impulse);
    m3real length2 = m3Dot3(next, next);
    if (length2 > limit * limit)
    {
        next = m3MulSV3(limit / sqrtf(length2), next);
    }
    *accumulated = next;
    return m3Sub3(next, old);
}

// Factors K = L D L^T in place, in double and without square roots, then
// solves for x by two substitutions. False, with x untouched, when a
// pivot falls below floor times its diagonal: the block has a dependent
// row, or one too few digits tell apart.
static bool SolveBlockWide(double k[M3_BLOCK_ROWS][M3_BLOCK_ROWS], int32_t count, double* x,
                           double floor)
{
    if (count < 1 || count > M3_BLOCK_ROWS)
    {
        return false;
    }
    double d[M3_BLOCK_ROWS];
    for (int32_t j = 0; j < count; ++j)
    {
        double pivot = k[j][j];
        for (int32_t m = 0; m < j; ++m)
        {
            pivot -= k[j][m] * k[j][m] * d[m];
        }
        if (!(pivot > floor * k[j][j]))
        {
            return false;
        }
        d[j] = pivot;
        for (int32_t i = j + 1; i < count; ++i)
        {
            double sum = k[i][j];
            for (int32_t m = 0; m < j; ++m)
            {
                sum -= k[i][m] * k[j][m] * d[m];
            }
            k[i][j] = sum / pivot;
        }
    }
    for (int32_t i = 0; i < count; ++i)
    {
        for (int32_t m = 0; m < i; ++m)
        {
            x[i] -= k[i][m] * x[m];
        }
    }
    for (int32_t i = count - 1; i >= 0; --i)
    {
        x[i] /= d[i];
        for (int32_t m = i + 1; m < count; ++m)
        {
            x[i] -= k[m][i] * x[m];
        }
    }
    return true;
}

// M^-1 J^T for one row in double: what an impulse on it does to each
// body's velocity and spin, the linear and angular parts of A, then B's.
static void WideResponse(const m3JointRow* r, const m3JointBodies* b, double out[12])
{
    const m3Vec3 parts[4] = {r->linA, r->angA, r->linB, r->angB};
    const m3Mat3* inertia[2] = {&b->iA, &b->iB};
    const m3real masses[2] = {b->mA, b->mB};
    for (int32_t body = 0; body < 2; ++body)
    {
        m3Vec3 lin = parts[2 * body];
        m3Vec3 ang = parts[2 * body + 1];
        const m3Mat3* w = inertia[body];
        double* o = out + 6 * body;
        o[0] = (double)masses[body] * (double)lin.x;
        o[1] = (double)masses[body] * (double)lin.y;
        o[2] = (double)masses[body] * (double)lin.z;
        o[3] = (double)w->cx.x * (double)ang.x + (double)w->cy.x * (double)ang.y +
               (double)w->cz.x * (double)ang.z;
        o[4] = (double)w->cx.y * (double)ang.x + (double)w->cy.y * (double)ang.y +
               (double)w->cz.y * (double)ang.z;
        o[5] = (double)w->cx.z * (double)ang.x + (double)w->cy.z * (double)ang.y +
               (double)w->cz.z * (double)ang.z;
    }
}

// J times a response in double: one coupling of the block.
static double WideApply(const m3JointRow* r, const double w[12])
{
    const m3Vec3 parts[4] = {r->linA, r->angA, r->linB, r->angB};
    double sum = 0.0;
    for (int32_t p = 0; p < 4; ++p)
    {
        sum += (double)parts[p].x * w[3 * p] + (double)parts[p].y * w[3 * p + 1] +
               (double)parts[p].z * w[3 * p + 2];
    }
    return sum;
}

// K's lower half built again from the rows in double, where a float K
// cancelled too far.
static void BuildBlockWide(const m3JointRow* rows, int32_t count, const m3JointBodies* b,
                           double k[M3_BLOCK_ROWS][M3_BLOCK_ROWS])
{
    double wide[M3_BLOCK_ROWS][12];
    for (int32_t i = 0; i < count; ++i)
    {
        WideResponse(&rows[i], b, wide[i]);
        for (int32_t j = 0; j <= i; ++j)
        {
            k[i][j] = WideApply(&rows[i], wide[j]);
        }
    }
}

// Clamps each row's accumulated impulse by the solved step x and pushes
// what changed as one velocity change, from the responses M^-1 J^T
// already built for K.
static void PushBlock(const m3JointRow* response, const bool* linear, const m3real* diagonal,
                      const double* x, int32_t count, m3JointBodies* b, m3RowDrive drive,
                      m3real* const* accumulated)
{
    m3JointRow change = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    for (int32_t i = 0; i < count; ++i)
    {
        m3real old = *accumulated[i];
        m3real bound = M3_ROW_SPEED_BOUND / diagonal[i];
        m3real next = m3ClampF(old - (m3real)x[i] - drive.leak * old, -bound, bound);
        *accumulated[i] = next;
        m3real applied = next - old;
        const m3JointRow* w = &response[i];
        if (linear[i])
        {
            change.linA = m3Add3(change.linA, m3MulSV3(applied, w->linA));
            change.linB = m3Add3(change.linB, m3MulSV3(applied, w->linB));
        }
        change.angA = m3Add3(change.angA, m3MulSV3(applied, w->angA));
        change.angB = m3Add3(change.angB, m3MulSV3(applied, w->angB));
    }
    b->vA = m3Add3(b->vA, change.linA);
    b->wA = m3Add3(b->wA, change.angA);
    b->vB = m3Add3(b->vB, change.linB);
    b->wB = m3Add3(b->wB, change.angB);
}

static bool IsZero3(m3Vec3 v)
{
    return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f;
}

void m3SolveRowBlock(const m3JointRow* rows, int32_t count, m3JointBodies* b, const m3real* bias,
                     m3RowDrive drive, m3real* const* accumulated)
{
    // K and the responses M^-1 J^T are built in float, like every other
    // row's mass, and factored in double. A turn row has no linear part,
    // so its terms with one are left out: they add exact zeros.
    double k[M3_BLOCK_ROWS][M3_BLOCK_ROWS];
    double x[M3_BLOCK_ROWS];
    m3JointRow response[M3_BLOCK_ROWS];
    bool linear[M3_BLOCK_ROWS];
    const m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    for (int32_t i = 0; i < count; ++i)
    {
        const m3JointRow* r = &rows[i];
        linear[i] = !(IsZero3(r->linA) && IsZero3(r->linB));
        m3Vec3 angA = m3MulMV3(b->iA, r->angA);
        m3Vec3 angB = m3MulMV3(b->iB, r->angB);
        response[i] =
            linear[i] ? (m3JointRow){m3MulSV3(b->mA, r->linA), angA, m3MulSV3(b->mB, r->linB), angB}
                      : (m3JointRow){zero, angA, zero, angB};
        for (int32_t j = 0; j <= i; ++j)
        {
            const m3JointRow* w = &response[j];
            m3real turn = m3Dot3(r->angA, w->angA) + m3Dot3(r->angB, w->angB);
            if (linear[i] && linear[j])
            {
                turn = m3Dot3(r->linA, w->linA) + m3Dot3(r->angA, w->angA) +
                       m3Dot3(r->linB, w->linB) + m3Dot3(r->angB, w->angB);
            }
            k[i][j] = (double)turn;
        }
        m3real speed = m3Dot3(r->angA, b->wA) + m3Dot3(r->angB, b->wB);
        if (linear[i])
        {
            speed = m3RowSpeed(r, b);
        }
        x[i] = (double)(drive.scale * speed + CapBias(bias[i]));
    }
    m3real diagonal[M3_BLOCK_ROWS];
    for (int32_t i = 0; i < count; ++i)
    {
        diagonal[i] = (m3real)k[i][i];
    }
    // A float K holds about seven digits: a pivot that cancels below 1e-4
    // of its diagonal has too few left, so K is built again in double,
    // and only a block singular even there falls back to the rows.
    bool solved = SolveBlockWide(k, count, x, 1.0e-4);
    if (!solved)
    {
        BuildBlockWide(rows, count, b, k);
        solved = SolveBlockWide(k, count, x, 1.0e-12);
    }
    if (!solved)
    {
        for (int32_t i = 0; i < count; ++i)
        {
            m3RowDrive one = {bias[i], drive.scale, drive.leak};
            m3SolveRow(&rows[i], b, one, accumulated[i], -M3_ROW_FREE, M3_ROW_FREE);
        }
        return;
    }
    PushBlock(response, linear, diagonal, x, count, b, drive, accumulated);
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
    m3Vec3 rhs = m3Add3(m3MulSV3(drive.scale, speed), CapBias3(bias));
    // The direct result stands for every ordinary block; one whose float
    // determinant underflows or cancels is built and solved in double.
    m3Vec3 solved;
    if (!m3Solve3Direct(&k, rhs, &solved))
    {
        double wide[9];
        m3WidePointBlock((double)b->mA + (double)b->mB, &b->iA, armA, &b->iB, armB, wide);
        solved = m3Solve3Wide(wide, rhs);
    }
    m3Vec3 impulse = m3Sub3(m3MulSV3(-1.0f, solved), m3MulSV3(drive.leak, *accumulated));
    m3PushPoint(armA, armB, b, WithinBall(accumulated, impulse, budget, &k));
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
    m3Vec3 rhs = m3Add3(m3MulSV3(drive.scale, m3Sub3(b->wB, b->wA)), CapBias3(bias));
    m3Vec3 impulse = m3Sub3(m3MulSV3(-1.0f, m3Solve3(&k, rhs)), m3MulSV3(drive.leak, *accumulated));
    m3PushTurn(b, WithinBall(accumulated, impulse, budget, &k));
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
