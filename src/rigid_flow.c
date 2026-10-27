// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The free rotation of a rigid body. docs/internals/rotation.md is the
// specification; each step below follows it in its order, so another
// library that follows it lands the same pose bit for bit where it runs
// the same float operations.

#include "rigid_flow.h"

#include "core.h"

#include <float.h>
#include <math.h>

enum
{
    M3_JACOBI_MAX_SWEEPS = 16,
    M3_FLOW_MAX_PIECES = 64
};

// The largest residual turn one piece of a substep takes, in radians.
#define M3_FLOW_PIECE_TURN 0.25f

// One Jacobi rotation zeroing a[p][q] of the symmetric a, accumulated into
// the columns of v.
static void JacobiRotate(double a[3][3], double v[3][3], int32_t p, int32_t q)
{
    double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
    double t = 1.0 / (fabs(theta) + sqrt(theta * theta + 1.0));
    if (theta < 0.0)
    {
        t = -t;
    }
    double c = 1.0 / sqrt(t * t + 1.0);
    double s = t * c;
    int32_t r = 3 - p - q;
    double apr = a[p][r];
    double aqr = a[q][r];
    a[p][p] -= t * a[p][q];
    a[q][q] += t * a[p][q];
    a[p][q] = 0.0;
    a[q][p] = 0.0;
    a[p][r] = c * apr - s * aqr;
    a[r][p] = a[p][r];
    a[q][r] = s * apr + c * aqr;
    a[r][q] = a[q][r];
    for (int32_t k = 0; k < 3; ++k)
    {
        double vkp = v[k][p];
        double vkq = v[k][q];
        v[k][p] = c * vkp - s * vkq;
        v[k][q] = s * vkp + c * vkq;
    }
}

// Sorts the eigenpairs by moment, largest first; a tie keeps the lower
// column first.
static void SortAxes(double d[3], double v[3][3])
{
    for (int32_t i = 1; i < 3; ++i)
    {
        for (int32_t j = i; j > 0 && d[j] > d[j - 1]; --j)
        {
            double dt = d[j];
            d[j] = d[j - 1];
            d[j - 1] = dt;
            for (int32_t k = 0; k < 3; ++k)
            {
                double vt = v[k][j];
                v[k][j] = v[k][j - 1];
                v[k][j - 1] = vt;
            }
        }
    }
}

// Each column's largest component positive, the first on a tie; then a
// determinant of -1 negates the third column.
static void FixSigns(double v[3][3])
{
    for (int32_t j = 0; j < 3; ++j)
    {
        int32_t big = 0;
        for (int32_t k = 1; k < 3; ++k)
        {
            if (fabs(v[k][j]) > fabs(v[big][j]))
            {
                big = k;
            }
        }
        if (v[big][j] < 0.0)
        {
            for (int32_t k = 0; k < 3; ++k)
            {
                v[k][j] = -v[k][j];
            }
        }
    }
    double det = v[0][0] * (v[1][1] * v[2][2] - v[2][1] * v[1][2]) -
                 v[0][1] * (v[1][0] * v[2][2] - v[2][0] * v[1][2]) +
                 v[0][2] * (v[1][0] * v[2][1] - v[2][0] * v[1][1]);
    if (det < 0.0)
    {
        for (int32_t k = 0; k < 3; ++k)
        {
            v[k][2] = -v[k][2];
        }
    }
}

// The rotation matrix v as a unit quaternion with w >= 0, by the largest
// of its four squared components.
static m3Quat MatrixToQuat(const double v[3][3])
{
    double tr = v[0][0] + v[1][1] + v[2][2];
    double q[4];
    if (tr >= v[0][0] && tr >= v[1][1] && tr >= v[2][2])
    {
        double s = 2.0 * sqrt(1.0 + tr);
        q[3] = 0.25 * s;
        q[0] = (v[2][1] - v[1][2]) / s;
        q[1] = (v[0][2] - v[2][0]) / s;
        q[2] = (v[1][0] - v[0][1]) / s;
    }
    else if (v[0][0] >= v[1][1] && v[0][0] >= v[2][2])
    {
        double s = 2.0 * sqrt(1.0 + v[0][0] - v[1][1] - v[2][2]);
        q[3] = (v[2][1] - v[1][2]) / s;
        q[0] = 0.25 * s;
        q[1] = (v[0][1] + v[1][0]) / s;
        q[2] = (v[0][2] + v[2][0]) / s;
    }
    else if (v[1][1] >= v[2][2])
    {
        double s = 2.0 * sqrt(1.0 + v[1][1] - v[0][0] - v[2][2]);
        q[3] = (v[0][2] - v[2][0]) / s;
        q[0] = (v[0][1] + v[1][0]) / s;
        q[1] = 0.25 * s;
        q[2] = (v[1][2] + v[2][1]) / s;
    }
    else
    {
        double s = 2.0 * sqrt(1.0 + v[2][2] - v[0][0] - v[1][1]);
        q[3] = (v[1][0] - v[0][1]) / s;
        q[0] = (v[0][2] + v[2][0]) / s;
        q[1] = (v[1][2] + v[2][1]) / s;
        q[2] = 0.25 * s;
    }
    double sign = q[3] < 0.0 ? -1.0 : 1.0;
    double inv = sign / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    m3Quat out = {(m3real)(q[0] * inv), (m3real)(q[1] * inv), (m3real)(q[2] * inv),
                  (m3real)(q[3] * inv)};
    return out;
}

void m3PrincipalAxes(const m3Mat3* inertia, m3Vec3* momentsOut, m3Quat* frameOut)
{
    double a[3][3] = {
        {(double)inertia->cx.x, (double)inertia->cy.x, (double)inertia->cz.x},
        {(double)inertia->cy.x, (double)inertia->cy.y, (double)inertia->cz.y},
        {(double)inertia->cz.x, (double)inertia->cz.y, (double)inertia->cz.z},
    };
    double v[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    static const int32_t pairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
    for (int32_t sweep = 0; sweep < M3_JACOBI_MAX_SWEEPS; ++sweep)
    {
        if (a[0][1] == 0.0 && a[0][2] == 0.0 && a[1][2] == 0.0)
        {
            break;
        }
        for (int32_t k = 0; k < 3; ++k)
        {
            if (a[pairs[k][0]][pairs[k][1]] != 0.0)
            {
                JacobiRotate(a, v, pairs[k][0], pairs[k][1]);
            }
        }
    }
    double d[3] = {a[0][0], a[1][1], a[2][2]};
    SortAxes(d, v);
    FixSigns(v);
    *momentsOut = (m3Vec3){(m3real)d[0], (m3real)d[1], (m3real)d[2]};
    *frameOut = MatrixToQuat(v);
}

bool m3FreeRotationApplies(m3Vec3 moments)
{
    return m3FiniteF(moments.x) && moments.z >= FLT_MIN && moments.z / moments.x >= FLT_MIN;
}

// Adds a turn by angle about one coordinate axis of the principal frame
// to turn, and turns the momentum's two components across it the other
// way, so
// the world momentum stays and the component along the axis is untouched.
// The half angle gives the quaternion, and the double-angle forms the
// cosine and sine for the momentum.
static void Residual(m3Quat* turn, m3Vec3* L, int32_t axis, m3real inverse, m3real inverseMid,
                     m3real t)
{
    m3real* l = &L->x;
    m3real angle = t * l[axis] * (inverse - inverseMid);
    m3CosSin half = m3ComputeCosSin(0.5f * angle);
    m3Quat r = {axis == 0 ? half.s : 0.0f, 0.0f, axis == 2 ? half.s : 0.0f, half.c};
    *turn = m3MulQuat(*turn, r);
    m3real c = half.c * half.c - half.s * half.s;
    m3real s = 2.0f * half.s * half.c;
    // R(-angle) about e_axis: for the cyclic pair (j, k) after the axis,
    // l_j' = c l_j + s l_k and l_k' = c l_k - s l_j.
    int32_t j = (axis + 1) % 3;
    int32_t k = (axis + 2) % 3;
    m3real lj = l[j];
    m3real lk = l[k];
    m3real nj = c * lj + s * lk;
    m3real nk = c * lk - s * lj;
    // A body's turn repeats every substep, and so would the rounding of
    // its cosine and sine, scaling the pair the same way each time. A
    // first-order correction back to the pair's length, applied as a small
    // delta, leaves only rounding that varies with the momentum.
    m3real before = lj * lj + lk * lk;
    m3real after = nj * nj + nk * nk;
    m3real delta = 0.5f * (before - after) / after;
    if (m3FiniteF(delta))
    {
        nj += delta * nj;
        nk += delta * nk;
    }
    l[j] = nj;
    l[k] = nk;
}

// The quaternion of a turn by angle about a unit axis.
static m3Quat AxisTurn(m3Vec3 axis, m3real angle)
{
    m3CosSin half = m3ComputeCosSin(0.5f * angle);
    m3Quat r = {half.s * axis.x, half.s * axis.y, half.s * axis.z, half.c};
    return r;
}

// The splitting's error comes from the two residual turns not commuting:
// per substep it grows as a^2 b and a b^2 for the residual angles a and b,
// bounded here by |L|, and vanishes when either is zero, as for a
// symmetric top. A substep whose error measure, the cube root of the
// larger product, passes M3_FLOW_PIECE_TURN takes the residual turns in
// equal pieces, each piece cutting the error by the square of their count.
static int32_t PieceCount(m3Vec3 L, m3real invMax, m3real invMid, m3real invMin, m3real h)
{
    m3real length = m3Length3(L);
    m3real a = h * length * (invMin - invMid);
    m3real b = h * length * (invMid - invMax);
    m3real pieces = ceilf(cbrtf(m3MaxF(a * a * b, a * b * b)) * (1.0f / M3_FLOW_PIECE_TURN));
    if (!(pieces > 1.0f))
    {
        return 1;
    }
    return pieces < (m3real)M3_FLOW_MAX_PIECES ? (int32_t)pieces : M3_FLOW_MAX_PIECES;
}

// The world spin of momentum L at the principal pose qp. The spin is
// divided, not multiplied by the inverses: I times the rounded 1 / I is the
// same factor for a body every substep, and would scale its spin by it
// each time. The next substep reads its momentum back from this spin and
// pose; their roundings change slowly with a slowly turning body, so they
// add up like a bias over many substeps, and a first-order correction
// makes the momentum read back as long as L.
static m3Vec3 SpinOf(m3Vec3 L, m3Vec3 scaled, m3Quat qp)
{
    m3Vec3 spin = {L.x, L.y / scaled.y, L.z / scaled.z};
    m3Vec3 out = m3RotateVec3(qp, spin);
    m3Vec3 back = m3InvRotateVec3(qp, out);
    m3Vec3 readBack = {back.x, scaled.y * back.y, scaled.z * back.z};
    m3real read = m3Dot3(readBack, readBack);
    m3real delta = 0.5f * (m3Dot3(L, L) - read) / read;
    if (m3FiniteF(delta))
    {
        out = m3Add3(out, m3MulSV3(delta, out));
    }
    return out;
}

void m3FreeRotation(m3Vec3 moments, m3Quat frame, m3Quat q, m3Vec3 w, m3real h, m3Quat* qOut,
                    m3Vec3* wOut)
{
    // Only the moments' ratios shape the flow, so it runs on moments
    // divided by the largest: the momentum then stays near the spin's size
    // for a body of any mass, where its square would leave float range.
    const m3Vec3 scaled = {1.0f, moments.y / moments.x, moments.z / moments.x};
    if (scaled.y == 1.0f && scaled.z == 1.0f)
    {
        // Equal moments: the spin is constant, and the body turns about it
        // at its rate. The spin is left bit for bit.
        m3real rate = m3Length3(w);
        if (rate > 0.0f)
        {
            q = m3NormalizeQuat(m3MulQuat(AxisTurn(m3MulSV3(1.0f / rate, w), h * rate), q));
        }
        *qOut = q;
        *wOut = w;
        return;
    }
    const m3real invMax = 1.0f;
    const m3real invMid = 1.0f / scaled.y;
    const m3real invMin = 1.0f / scaled.z;
    m3Vec3 wp = m3InvRotateVec3(m3NormalizeQuat(m3MulQuat(q, frame)), w);
    m3Vec3 L = {wp.x, scaled.y * wp.y, scaled.z * wp.z};
    int32_t count = PieceCount(L, invMax, invMid, invMin, h);
    m3real t = h / (m3real)count;
    // The turns compose into one rotation in the principal frame, which
    // the pose takes once, through the frame: re-basing the pose itself
    // into the principal frame and back would round it every substep, and
    // turn a body whose spin does not.
    m3Quat turn = m3MakeIdentityQuat();
    for (int32_t k = 0; k < count; ++k)
    {
        Residual(&turn, &L, 2, invMin, invMid, 0.5f * t);
        Residual(&turn, &L, 0, invMax, invMid, t);
        Residual(&turn, &L, 2, invMin, invMid, 0.5f * t);
    }
    // The Casimir turn: the body turns about its momentum at |L| / I_mid,
    // which leaves the momentum's components where they are. The turn is
    // exact at any angle.
    m3real length = m3Length3(L);
    if (length > 0.0f)
    {
        turn = m3MulQuat(turn, AxisTurn(m3MulSV3(1.0f / length, L), h * length * invMid));
    }
    m3Quat unframe = {-frame.x, -frame.y, -frame.z, frame.w};
    m3Quat next = m3NormalizeQuat(m3MulQuat(q, m3MulQuat(m3MulQuat(frame, turn), unframe)));
    *wOut = SpinOf(L, scaled, m3NormalizeQuat(m3MulQuat(next, frame)));
    *qOut = next;
}
