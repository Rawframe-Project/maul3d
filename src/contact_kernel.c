// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The contact kernel. Each stage gathers the velocities of a block's
// bodies into lane vectors, works through the rows of every lane at
// once, and scatters the velocities back.
//
// The rows are the scalar rows of contact_solver.c, operation for
// operation: the same products and sums in the same order, no fused
// multiply-add, and each branch of the scalar code taken as a select
// between both outcomes. A lane therefore produces the bits the scalar
// row produces for its constraint, and a test holds the two paths to
// that. What the scalar code skips (a point the constraint does not
// have, a side that does not move, rolling with no resistance) the
// kernel computes and then discards.

#include "contact_kernel.h"

#include "simd.h"
#include "world_internal.h"

#include <string.h>

typedef struct V3
{
    m3f8 x;
    m3f8 y;
    m3f8 z;
} V3;

// Columns, like m3Mat3.
typedef struct Mat
{
    V3 cx;
    V3 cy;
    V3 cz;
} Mat;

static V3 LoadV3(const float src[3][M3_LANES])
{
    V3 r = {m3F8Load(src[0]), m3F8Load(src[1]), m3F8Load(src[2])};
    return r;
}

static void StoreV3(float dst[3][M3_LANES], V3 v)
{
    m3F8Store(dst[0], v.x);
    m3F8Store(dst[1], v.y);
    m3F8Store(dst[2], v.z);
}

static Mat LoadMat(const float src[9][M3_LANES])
{
    Mat m = {LoadV3((const float (*)[M3_LANES])src[0]), LoadV3((const float (*)[M3_LANES])src[3]),
             LoadV3((const float (*)[M3_LANES])src[6])};
    return m;
}

static V3 Add3(V3 a, V3 b)
{
    V3 r = {m3F8Add(a.x, b.x), m3F8Add(a.y, b.y), m3F8Add(a.z, b.z)};
    return r;
}

static V3 Sub3(V3 a, V3 b)
{
    V3 r = {m3F8Sub(a.x, b.x), m3F8Sub(a.y, b.y), m3F8Sub(a.z, b.z)};
    return r;
}

static V3 Scale3(m3f8 s, V3 v)
{
    V3 r = {m3F8Mul(s, v.x), m3F8Mul(s, v.y), m3F8Mul(s, v.z)};
    return r;
}

static m3f8 Dot3(V3 a, V3 b)
{
    return m3F8Add(m3F8Add(m3F8Mul(a.x, b.x), m3F8Mul(a.y, b.y)), m3F8Mul(a.z, b.z));
}

static V3 Cross3(V3 a, V3 b)
{
    V3 r = {m3F8Sub(m3F8Mul(a.y, b.z), m3F8Mul(a.z, b.y)),
            m3F8Sub(m3F8Mul(a.z, b.x), m3F8Mul(a.x, b.z)),
            m3F8Sub(m3F8Mul(a.x, b.y), m3F8Mul(a.y, b.x))};
    return r;
}

static V3 MulMV3(const Mat* m, V3 v)
{
    V3 r = {m3F8Add(m3F8Add(m3F8Mul(m->cx.x, v.x), m3F8Mul(m->cy.x, v.y)), m3F8Mul(m->cz.x, v.z)),
            m3F8Add(m3F8Add(m3F8Mul(m->cx.y, v.x), m3F8Mul(m->cy.y, v.y)), m3F8Mul(m->cz.y, v.z)),
            m3F8Add(m3F8Add(m3F8Mul(m->cx.z, v.x), m3F8Mul(m->cy.z, v.y)), m3F8Mul(m->cz.z, v.z))};
    return r;
}

static V3 Select3(m3f8 mask, V3 a, V3 b)
{
    V3 r = {m3F8Select(mask, a.x, b.x), m3F8Select(mask, a.y, b.y), m3F8Select(mask, a.z, b.z)};
    return r;
}

// m3RotateVec3: t = 2 (u x v), v + w t + u x t.
static V3 Rotate(V3 u, m3f8 w, V3 v)
{
    V3 t = Scale3(m3F8Set1(2.0f), Cross3(u, v));
    return Add3(Add3(v, Scale3(w, t)), Cross3(u, t));
}

// m3Solve3 by Cramer's rule; a singular matrix gives zero.
static V3 Solve3(const Mat* k, V3 b)
{
    V3 cxy = Cross3(k->cy, k->cz);
    m3f8 det = Dot3(k->cx, cxy);
    m3f8 inv = m3F8Div(m3F8Set1(1.0f), det);
    V3 x = {m3F8Mul(inv, Dot3(b, cxy)), m3F8Mul(inv, Dot3(k->cx, Cross3(b, k->cz))),
            m3F8Mul(inv, Dot3(k->cx, Cross3(k->cy, b)))};
    V3 zero = {m3F8Zero(), m3F8Zero(), m3F8Zero()};
    return Select3(m3F8EQ(det, m3F8Zero()), zero, x);
}

// Both bodies of every lane during one stage.
typedef struct Pair
{
    V3 vA;
    V3 wA;
    V3 vB;
    V3 wB;
    m3f8 movesA;
    m3f8 movesB;
    m3f8 mA;
    m3f8 mB;
    Mat iA;
    Mat iB;
} Pair;

static void GatherVec3(const m3Vec3* values, const int32_t* bodies, float lanes[3][M3_LANES])
{
    for (int32_t lane = 0; lane < M3_LANES; ++lane)
    {
        m3Vec3 v = values[bodies[lane]];
        lanes[0][lane] = v.x;
        lanes[1][lane] = v.y;
        lanes[2][lane] = v.z;
    }
}

static Pair GatherPair(const m3World* world, const m3ContactBlock* b)
{
    float lanes[4][3][M3_LANES];
    GatherVec3(world->bodies.linearVelocities, b->bodyA, lanes[0]);
    GatherVec3(world->bodies.angularVelocities, b->bodyA, lanes[1]);
    GatherVec3(world->bodies.linearVelocities, b->bodyB, lanes[2]);
    GatherVec3(world->bodies.angularVelocities, b->bodyB, lanes[3]);
    Pair p;
    p.vA = LoadV3((const float (*)[M3_LANES])lanes[0]);
    p.wA = LoadV3((const float (*)[M3_LANES])lanes[1]);
    p.vB = LoadV3((const float (*)[M3_LANES])lanes[2]);
    p.wB = LoadV3((const float (*)[M3_LANES])lanes[3]);
    p.movesA = m3F8Load(b->movesA);
    p.movesB = m3F8Load(b->movesB);
    p.mA = m3F8Load(b->invMassA);
    p.mB = m3F8Load(b->invMassB);
    p.iA = LoadMat((const float (*)[M3_LANES])b->invIA);
    p.iB = LoadMat((const float (*)[M3_LANES])b->invIB);
    return p;
}

// Only dynamic sides are written: a static or kinematic body appears in
// many lanes of one color, and even an unchanged write would race.
static void ScatterPair(m3World* world, const m3ContactBlock* b, const Pair* p)
{
    float lanes[4][3][M3_LANES];
    StoreV3(lanes[0], p->vA);
    StoreV3(lanes[1], p->wA);
    StoreV3(lanes[2], p->vB);
    StoreV3(lanes[3], p->wB);
    for (int32_t lane = 0; lane < b->lanes; ++lane)
    {
        int32_t body = b->bodyA[lane];
        if (world->bodies.types[body] == (uint8_t)m3_dynamicBody)
        {
            world->bodies.linearVelocities[body] =
                (m3Vec3){lanes[0][0][lane], lanes[0][1][lane], lanes[0][2][lane]};
            world->bodies.angularVelocities[body] =
                (m3Vec3){lanes[1][0][lane], lanes[1][1][lane], lanes[1][2][lane]};
        }
        body = b->bodyB[lane];
        if (world->bodies.types[body] == (uint8_t)m3_dynamicBody)
        {
            world->bodies.linearVelocities[body] =
                (m3Vec3){lanes[2][0][lane], lanes[2][1][lane], lanes[2][2][lane]};
            world->bodies.angularVelocities[body] =
                (m3Vec3){lanes[3][0][lane], lanes[3][1][lane], lanes[3][2][lane]};
        }
    }
}

// B's point velocity minus A's.
static V3 RelativeVelocity(const Pair* p, V3 rA, V3 rB)
{
    return Sub3(Add3(p->vB, Cross3(p->wB, rB)), Add3(p->vA, Cross3(p->wA, rA)));
}

// The impulse on B at rB and its opposite on A at rA, in the lanes of
// mask.
static void Push(Pair* p, V3 impulse, V3 rA, V3 rB, m3f8 mask)
{
    m3f8 onA = m3F8And(p->movesA, mask);
    m3f8 onB = m3F8And(p->movesB, mask);
    p->vA = Select3(onA, Sub3(p->vA, Scale3(p->mA, impulse)), p->vA);
    p->wA = Select3(onA, Sub3(p->wA, MulMV3(&p->iA, Cross3(rA, impulse))), p->wA);
    p->vB = Select3(onB, Add3(p->vB, Scale3(p->mB, impulse)), p->vB);
    p->wB = Select3(onB, Add3(p->wB, MulMV3(&p->iB, Cross3(rB, impulse))), p->wB);
}

// An angular impulse on B and its opposite on A, in the lanes of mask.
static void Turn(Pair* p, V3 impulse, m3f8 mask)
{
    m3f8 onA = m3F8And(p->movesA, mask);
    m3f8 onB = m3F8And(p->movesB, mask);
    p->wA = Select3(onA, Sub3(p->wA, MulMV3(&p->iA, impulse)), p->wA);
    p->wB = Select3(onB, Add3(p->wB, MulMV3(&p->iB, impulse)), p->wB);
}

static m3f8 AllLanes(void)
{
    return m3F8EQ(m3F8Zero(), m3F8Zero());
}

static void WarmStart(Pair* p, const m3ContactBlock* b)
{
    V3 n = LoadV3(b->normal);
    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        const m3LanePoint* pt = &b->points[k];
        Push(p, Scale3(m3F8Load(pt->normalImpulse), n), LoadV3(pt->rA), LoadV3(pt->rB),
             m3F8Load(pt->active));
    }
    V3 f = Add3(Scale3(m3F8Load(b->frictionImpulse1), LoadV3(b->t1)),
                Scale3(m3F8Load(b->frictionImpulse2), LoadV3(b->t2)));
    Push(p, f, LoadV3(b->originA), LoadV3(b->originB), AllLanes());
    Turn(p, Add3(Scale3(m3F8Load(b->twistImpulse), n), LoadV3(b->rollingImpulse)), AllLanes());
}

// The pass's normal impulses: their sum budgets friction and rolling,
// their lever-weighted sum the twist.
typedef struct Budget
{
    m3f8 normal;
    m3f8 twist;
} Budget;

// How far each lane's bodies moved and turned since the step began.
typedef struct Drift
{
    V3 drift; // B's translation minus A's
    V3 uA;    // the rotations' vector and scalar parts
    m3f8 wA;
    V3 uB;
    m3f8 wB;
} Drift;

static Drift GatherDrift(const m3ContactPlan* plan, const m3ContactBlock* b)
{
    float pos[2][3][M3_LANES];
    float rot[2][4][M3_LANES];
    GatherVec3(plan->deltaPos, b->bodyA, pos[0]);
    GatherVec3(plan->deltaPos, b->bodyB, pos[1]);
    for (int32_t lane = 0; lane < M3_LANES; ++lane)
    {
        m3Quat qA = plan->deltaRot[b->bodyA[lane]];
        m3Quat qB = plan->deltaRot[b->bodyB[lane]];
        rot[0][0][lane] = qA.x;
        rot[0][1][lane] = qA.y;
        rot[0][2][lane] = qA.z;
        rot[0][3][lane] = qA.w;
        rot[1][0][lane] = qB.x;
        rot[1][1][lane] = qB.y;
        rot[1][2][lane] = qB.z;
        rot[1][3][lane] = qB.w;
    }
    Drift d;
    d.drift =
        Sub3(LoadV3((const float (*)[M3_LANES])pos[1]), LoadV3((const float (*)[M3_LANES])pos[0]));
    d.uA = LoadV3((const float (*)[M3_LANES])rot[0]);
    d.wA = m3F8Load(rot[0][3]);
    d.uB = LoadV3((const float (*)[M3_LANES])rot[1]);
    d.wB = m3F8Load(rot[1][3]);
    return d;
}

// The normal rows of contact_solver.c. The scalar row picks the target,
// mass scale and leak by branch; here all are computed and selected.
static Budget NormalRows(const m3World* world, const m3ContactPlan* plan, Pair* p,
                         m3ContactBlock* b, bool soft)
{
    Budget budget = {m3F8Zero(), m3F8Zero()};
    Drift d = GatherDrift(plan, b);
    V3 n = LoadV3(b->normal);
    m3f8 zero = m3F8Zero();
    m3f8 one = m3F8Set1(1.0f);
    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        m3LanePoint* pt = &b->points[k];
        m3f8 active = m3F8Load(pt->active);
        V3 rA = LoadV3(pt->rA);
        V3 rB = LoadV3(pt->rB);
        V3 turnedA = Rotate(d.uA, d.wA, rA);
        V3 turnedB = Rotate(d.uB, d.wB, rB);
        m3f8 s =
            m3F8Add(m3F8Load(pt->baseSeparation), Dot3(Add3(d.drift, Sub3(turnedB, turnedA)), n));
        m3f8 apart = m3F8GT(s, zero);
        m3f8 target = m3F8Mul(s, m3F8Set1(plan->invH));
        m3f8 scale = one;
        m3f8 leak = zero;
        if (soft)
        {
            m3f8 push =
                m3F8Max(m3F8Mul(m3F8Load(b->biasRate), s), m3F8Set1(-world->contactPushMaxSpeed));
            target = m3F8Select(apart, target, push);
            scale = m3F8Select(apart, one, m3F8Load(b->massScale));
            leak = m3F8Select(apart, zero, m3F8Load(b->impulseScale));
        }
        else
        {
            target = m3F8Select(apart, target, zero);
        }
        m3f8 vn = Dot3(RelativeVelocity(p, rA, rB), n);
        m3f8 old = m3F8Load(pt->normalImpulse);
        m3f8 change =
            m3F8Sub(m3F8Mul(m3F8Mul(m3F8Neg(m3F8Load(pt->normalMass)), scale), m3F8Add(vn, target)),
                    m3F8Mul(leak, old));
        m3f8 next = m3F8Select(active, m3F8Max(m3F8Add(old, change), zero), old);
        m3F8Store(pt->normalImpulse, next);
        m3f8 total = m3F8Load(pt->totalNormalImpulse);
        m3F8Store(pt->totalNormalImpulse, m3F8Select(active, m3F8Add(total, next), total));
        budget.normal = m3F8Select(active, m3F8Add(budget.normal, next), budget.normal);
        budget.twist = m3F8Select(
            active, m3F8Add(budget.twist, m3F8Mul(m3F8Load(pt->leverArm), next)), budget.twist);
        Push(p, Scale3(m3F8Sub(next, old), n), rA, rB, active);
    }
    return budget;
}

// Brakes spin about the normal within the twist budget.
static void TwistRow(Pair* p, m3ContactBlock* b, Budget budget)
{
    V3 n = LoadV3(b->normal);
    m3f8 spin = Dot3(n, Sub3(p->wB, p->wA));
    m3f8 bound = m3F8Mul(m3F8Load(b->friction), budget.twist);
    m3f8 old = m3F8Load(b->twistImpulse);
    m3f8 next = m3F8Sub(old, m3F8Mul(m3F8Load(b->twistMass), spin));
    m3f8 low = m3F8Neg(bound);
    next = m3F8Select(m3F8LT(next, low), low, m3F8Select(m3F8GT(next, bound), bound, next));
    m3F8Store(b->twistImpulse, next);
    Turn(p, Scale3(m3F8Sub(next, old), n), AllLanes());
}

// Brakes relative rotation within the rolling ball, in the lanes that
// have rolling resistance.
static void RollingRow(Pair* p, m3ContactBlock* b, Budget budget)
{
    m3f8 resistance = m3F8Load(b->rollingResistance);
    m3f8 rolls = m3F8GT(resistance, m3F8Zero());
    Mat k = LoadMat((const float (*)[M3_LANES])b->rollingK);
    V3 old = LoadV3(b->rollingImpulse);
    V3 delta = Scale3(m3F8Set1(-1.0f), Solve3(&k, Sub3(p->wB, p->wA)));
    V3 next = Add3(old, delta);
    m3f8 bound = m3F8Mul(resistance, budget.normal);
    m3f8 length2 = Dot3(next, next);
    m3f8 over = m3F8And(m3F8GT(length2, m3F8Mul(bound, bound)), m3F8GT(length2, m3F8Zero()));
    next = Select3(over, Scale3(m3F8Div(bound, m3F8Sqrt(length2)), next), next);
    delta = Sub3(next, old);
    StoreV3(b->rollingImpulse, Select3(rolls, next, old));
    Turn(p, delta, rolls);
}

// The tangent pair at the manifold center within the Coulomb circle.
static void FrictionRows(Pair* p, m3ContactBlock* b, Budget budget)
{
    V3 t1 = LoadV3(b->t1);
    V3 t2 = LoadV3(b->t2);
    m3f8 k11 = m3F8Load(b->frictionK11);
    m3f8 k12 = m3F8Load(b->frictionK12);
    m3f8 k22 = m3F8Load(b->frictionK22);
    m3f8 old1 = m3F8Load(b->frictionImpulse1);
    m3f8 old2 = m3F8Load(b->frictionImpulse2);
    V3 originA = LoadV3(b->originA);
    V3 originB = LoadV3(b->originB);
    V3 v = RelativeVelocity(p, originA, originB);
    m3f8 vt1 = m3F8Sub(Dot3(v, t1), m3F8Load(b->tangentVelocity1));
    m3f8 vt2 = m3F8Sub(Dot3(v, t2), m3F8Load(b->tangentVelocity2));
    m3f8 f1 = m3F8Sub(old1, m3F8Add(m3F8Mul(k11, vt1), m3F8Mul(k12, vt2)));
    m3f8 f2 = m3F8Sub(old2, m3F8Add(m3F8Mul(k12, vt1), m3F8Mul(k22, vt2)));
    m3f8 bound = m3F8Mul(m3F8Load(b->friction), budget.normal);
    m3f8 length2 = m3F8Add(m3F8Mul(f1, f1), m3F8Mul(f2, f2));
    m3f8 over = m3F8GT(length2, m3F8Mul(bound, bound));
    m3f8 length = m3F8Sqrt(length2);
    m3f8 scale = m3F8Select(m3F8GT(length, m3F8Zero()), m3F8Div(bound, length), m3F8Zero());
    f1 = m3F8Select(over, m3F8Mul(f1, scale), f1);
    f2 = m3F8Select(over, m3F8Mul(f2, scale), f2);
    V3 delta = Add3(Scale3(m3F8Sub(f1, old1), t1), Scale3(m3F8Sub(f2, old2), t2));
    m3F8Store(b->frictionImpulse1, f1);
    m3F8Store(b->frictionImpulse2, f2);
    Push(p, delta, originA, originB, AllLanes());
}

void m3RunContactBlock(m3World* world, const m3ContactPlan* plan, m3ContactBlock* block,
                       m3ContactStage stage)
{
    Pair p = GatherPair(world, block);
    if (stage == m3_contactWarmStart)
    {
        WarmStart(&p, block);
    }
    else
    {
        Budget budget = NormalRows(world, plan, &p, block, stage == m3_contactSolve);
        if (stage == m3_contactRelax)
        {
            TwistRow(&p, block, budget);
            RollingRow(&p, block, budget);
            FrictionRows(&p, block, budget);
        }
    }
    ScatterPair(world, block, &p);
}

static float Mask(bool on)
{
    uint32_t bits = on ? 0xFFFFFFFFu : 0u;
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static void PutV3(float dst[3][M3_LANES], int32_t lane, m3Vec3 v)
{
    dst[0][lane] = v.x;
    dst[1][lane] = v.y;
    dst[2][lane] = v.z;
}

static void PutMat(float dst[9][M3_LANES], int32_t lane, const m3Mat3* m)
{
    PutV3((float (*)[M3_LANES])dst[0], lane, m->cx);
    PutV3((float (*)[M3_LANES])dst[3], lane, m->cy);
    PutV3((float (*)[M3_LANES])dst[6], lane, m->cz);
}

void m3PackContactLane(const m3World* world, m3ContactBlock* block, int32_t lane,
                       const m3ContactConstraint* c, int32_t index)
{
    block->bodyA[lane] = c->bodyA;
    block->bodyB[lane] = c->bodyB;
    block->constraint[lane] = index;
    block->movesA[lane] = Mask(world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody);
    block->movesB[lane] = Mask(world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody);
    PutV3(block->normal, lane, c->normal);
    PutV3(block->t1, lane, c->t1);
    PutV3(block->t2, lane, c->t2);
    PutV3(block->originA, lane, c->originA);
    PutV3(block->originB, lane, c->originB);
    block->frictionK11[lane] = c->frictionK11;
    block->frictionK12[lane] = c->frictionK12;
    block->frictionK22[lane] = c->frictionK22;
    block->frictionImpulse1[lane] = c->frictionImpulse1;
    block->frictionImpulse2[lane] = c->frictionImpulse2;
    block->twistMass[lane] = c->twistMass;
    block->twistImpulse[lane] = c->twistImpulse;
    block->tangentVelocity1[lane] = c->tangentVelocity1;
    block->tangentVelocity2[lane] = c->tangentVelocity2;
    block->friction[lane] = c->friction;
    block->invMassA[lane] = c->invMassA;
    block->invMassB[lane] = c->invMassB;
    PutMat(block->invIA, lane, &c->invIA);
    PutMat(block->invIB, lane, &c->invIB);
    block->biasRate[lane] = c->softness.biasRate;
    block->massScale[lane] = c->softness.massScale;
    block->impulseScale[lane] = c->softness.impulseScale;
    block->rollingResistance[lane] = c->rollingResistance;
    PutV3(block->rollingImpulse, lane, c->rollingImpulse);
    PutMat(block->rollingK, lane, &c->rollingK);
    for (int32_t k = 0; k < M3_MANIFOLD_MAX_POINTS; ++k)
    {
        m3LanePoint* pt = &block->points[k];
        bool has = k < c->pointCount;
        const m3ConstraintPoint* cp = &c->points[has ? k : 0];
        m3Vec3 zero = {0.0f, 0.0f, 0.0f};
        PutV3(pt->rA, lane, has ? cp->rA : zero);
        PutV3(pt->rB, lane, has ? cp->rB : zero);
        pt->baseSeparation[lane] = has ? cp->baseSeparation : 0.0f;
        pt->normalMass[lane] = has ? cp->normalMass : 0.0f;
        pt->leverArm[lane] = has ? cp->leverArm : 0.0f;
        pt->normalImpulse[lane] = has ? cp->normalImpulse : 0.0f;
        pt->totalNormalImpulse[lane] = has ? cp->totalNormalImpulse : 0.0f;
        pt->active[lane] = Mask(has);
    }
    if (c->pointCount > block->pointCount)
    {
        block->pointCount = c->pointCount;
    }
}

void m3PadContactLane(m3ContactBlock* block, int32_t lane)
{
    // The block starts zeroed, so the lane's masks are already off; it
    // only needs valid bodies to read.
    block->bodyA[lane] = block->bodyA[0];
    block->bodyB[lane] = block->bodyB[0];
    block->constraint[lane] = -1;
}

void m3UnpackContactBlock(const m3ContactBlock* block, m3ContactConstraint* constraints)
{
    for (int32_t lane = 0; lane < block->lanes; ++lane)
    {
        m3ContactConstraint* c = &constraints[block->constraint[lane]];
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            c->points[k].normalImpulse = block->points[k].normalImpulse[lane];
            c->points[k].totalNormalImpulse = block->points[k].totalNormalImpulse[lane];
        }
        c->frictionImpulse1 = block->frictionImpulse1[lane];
        c->frictionImpulse2 = block->frictionImpulse2[lane];
        c->twistImpulse = block->twistImpulse[lane];
        c->rollingImpulse = (m3Vec3){block->rollingImpulse[0][lane], block->rollingImpulse[1][lane],
                                     block->rollingImpulse[2][lane]};
    }
}
