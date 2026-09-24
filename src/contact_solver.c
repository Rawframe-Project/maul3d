// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The contact rows and stages. A pass over one constraint loads both
// bodies' velocities, works through its rows and stores the dynamic
// sides back. The solve pass runs the normal rows alone with the soft
// push: the push's motion is not real sliding, and friction solved
// against it would fight it. The relax pass runs the normal rows rigid,
// then twist, rolling and friction, each budgeted by the normal
// impulses of that same pass.

#include "contact_solver.h"

#include "contact_kernel.h"
#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

// Both bodies of a constraint during one pass. A side that is not
// dynamic keeps its velocity: it is never written, and a body shared by
// many constraints of one color must not be.
typedef struct Pair
{
    m3Vec3 vA;
    m3Vec3 wA;
    m3Vec3 vB;
    m3Vec3 wB;
    bool movesA;
    bool movesB;
} Pair;

static Pair LoadPair(const m3World* world, const m3ContactConstraint* c)
{
    Pair p = {world->bodies.linearVelocities[c->bodyA],
              world->bodies.angularVelocities[c->bodyA],
              world->bodies.linearVelocities[c->bodyB],
              world->bodies.angularVelocities[c->bodyB],
              world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody,
              world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody};
    return p;
}

static void StorePair(m3World* world, const m3ContactConstraint* c, const Pair* p)
{
    if (p->movesA)
    {
        world->bodies.linearVelocities[c->bodyA] = p->vA;
        world->bodies.angularVelocities[c->bodyA] = p->wA;
    }
    if (p->movesB)
    {
        world->bodies.linearVelocities[c->bodyB] = p->vB;
        world->bodies.angularVelocities[c->bodyB] = p->wB;
    }
}

// B's point velocity minus A's.
static m3Vec3 RelativeVelocity(const Pair* p, m3Vec3 rA, m3Vec3 rB)
{
    return m3Sub3(m3Add3(p->vB, m3Cross3(p->wB, rB)), m3Add3(p->vA, m3Cross3(p->wA, rA)));
}

// The impulse on B at rB and its opposite on A at rA.
static void Push(Pair* p, const m3ContactConstraint* c, m3Vec3 impulse, m3Vec3 rA, m3Vec3 rB)
{
    if (p->movesA)
    {
        p->vA = m3Sub3(p->vA, m3MulSV3(c->invMassA, impulse));
        p->wA = m3Sub3(p->wA, m3MulMV3(c->invIA, m3Cross3(rA, impulse)));
    }
    if (p->movesB)
    {
        p->vB = m3Add3(p->vB, m3MulSV3(c->invMassB, impulse));
        p->wB = m3Add3(p->wB, m3MulMV3(c->invIB, m3Cross3(rB, impulse)));
    }
}

// An angular impulse on B and its opposite on A.
static void Turn(Pair* p, const m3ContactConstraint* c, m3Vec3 impulse)
{
    if (p->movesA)
    {
        p->wA = m3Sub3(p->wA, m3MulMV3(c->invIA, impulse));
    }
    if (p->movesB)
    {
        p->wB = m3Add3(p->wB, m3MulMV3(c->invIB, impulse));
    }
}

static void WarmStart(Pair* p, const m3ContactConstraint* c)
{
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        const m3ConstraintPoint* cp = &c->points[k];
        Push(p, c, m3MulSV3(cp->normalImpulse, c->normal), cp->rA, cp->rB);
    }
    m3Vec3 f = m3Add3(m3MulSV3(c->frictionImpulse1, c->t1), m3MulSV3(c->frictionImpulse2, c->t2));
    Push(p, c, f, c->originA, c->originB);
    Turn(p, c, m3Add3(m3MulSV3(c->twistImpulse, c->normal), c->rollingImpulse));
}

// The pass's normal impulses: their sum budgets friction and rolling,
// their lever-weighted sum the twist.
typedef struct Budget
{
    m3real normal;
    m3real twist;
} Budget;

// The normal rows. Rows keep their prepare-time anchors; the anchors
// turned by each body's rotation since the step began only measure the
// separation. A separated point lets the bodies close the gap in this
// substep; an overlapping one is pushed out softly, never faster than
// the push speed limit, while solving and held rigid while relaxing.
static Budget NormalRows(const m3World* world, const m3ContactPlan* plan, Pair* p,
                         m3ContactConstraint* c, bool soft)
{
    Budget budget = {0.0f, 0.0f};
    m3Vec3 drift = m3Sub3(plan->deltaPos[c->bodyB], plan->deltaPos[c->bodyA]);
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        m3ConstraintPoint* cp = &c->points[k];
        m3Vec3 turnedA = m3RotateVec3(plan->deltaRot[c->bodyA], cp->rA);
        m3Vec3 turnedB = m3RotateVec3(plan->deltaRot[c->bodyB], cp->rB);
        m3real s = cp->baseSeparation + m3Dot3(m3Add3(drift, m3Sub3(turnedB, turnedA)), c->normal);
        m3real target = 0.0f;
        m3real scale = 1.0f;
        m3real leak = 0.0f;
        if (s > 0.0f)
        {
            target = s * plan->invH;
        }
        else if (soft)
        {
            target = m3MaxF(c->softness.biasRate * s, -world->contactPushMaxSpeed);
            scale = c->softness.massScale;
            leak = c->softness.impulseScale;
        }
        m3real vn = m3Dot3(RelativeVelocity(p, cp->rA, cp->rB), c->normal);
        m3real old = cp->normalImpulse;
        m3real next = m3MaxF(old + (-cp->normalMass * scale * (vn + target) - leak * old), 0.0f);
        cp->normalImpulse = next;
        cp->totalNormalImpulse += next;
        budget.normal += next;
        budget.twist += cp->leverArm * next;
        Push(p, c, m3MulSV3(next - old, c->normal), cp->rA, cp->rB);
    }
    return budget;
}

// Brakes spin about the normal. A single point has no lever, so no
// twist budget.
static void TwistRow(Pair* p, m3ContactConstraint* c, Budget budget)
{
    m3real spin = m3Dot3(c->normal, m3Sub3(p->wB, p->wA));
    m3real bound = c->friction * budget.twist;
    m3real old = c->twistImpulse;
    m3real next = old - c->twistMass * spin;
    next = next < -bound ? -bound : (next > bound ? bound : next);
    c->twistImpulse = next;
    Turn(p, c, m3MulSV3(next - old, c->normal));
}

// Brakes relative rotation, within a ball of the rolling resistance
// times the normal budget. Without it a pile of spheres never stops
// rolling and never sleeps.
static void RollingRow(Pair* p, m3ContactConstraint* c, Budget budget)
{
    m3Vec3 delta = m3MulSV3(-1.0f, m3Solve3(&c->rollingK, m3Sub3(p->wB, p->wA)));
    m3Vec3 next = m3Add3(c->rollingImpulse, delta);
    m3real bound = c->rollingResistance * budget.normal;
    m3real length2 = m3Dot3(next, next);
    if (length2 > bound * bound && length2 > 0.0f)
    {
        next = m3MulSV3(bound / sqrtf(length2), next);
    }
    delta = m3Sub3(next, c->rollingImpulse);
    c->rollingImpulse = next;
    Turn(p, c, delta);
}

// The tangent pair at the manifold center, within the Coulomb circle of
// the normal budget, driving the tangential speed toward the belt speed.
static void FrictionRows(Pair* p, m3ContactConstraint* c, Budget budget)
{
    m3Vec3 v = RelativeVelocity(p, c->originA, c->originB);
    m3real vt1 = m3Dot3(v, c->t1) - c->tangentVelocity1;
    m3real vt2 = m3Dot3(v, c->t2) - c->tangentVelocity2;
    m3real f1 = c->frictionImpulse1 - (c->frictionK11 * vt1 + c->frictionK12 * vt2);
    m3real f2 = c->frictionImpulse2 - (c->frictionK12 * vt1 + c->frictionK22 * vt2);
    m3real bound = c->friction * budget.normal;
    m3real length2 = f1 * f1 + f2 * f2;
    if (length2 > bound * bound)
    {
        m3real length = sqrtf(length2);
        m3real scale = length > 0.0f ? bound / length : 0.0f;
        f1 *= scale;
        f2 *= scale;
    }
    m3Vec3 delta = m3Add3(m3MulSV3(f1 - c->frictionImpulse1, c->t1),
                          m3MulSV3(f2 - c->frictionImpulse2, c->t2));
    c->frictionImpulse1 = f1;
    c->frictionImpulse2 = f2;
    Push(p, c, delta, c->originA, c->originB);
}

static void SolveConstraint(m3World* world, const m3ContactPlan* plan, m3ContactConstraint* c,
                            m3ContactStage stage)
{
    Pair p = LoadPair(world, c);
    if (stage == m3_contactWarmStart)
    {
        WarmStart(&p, c);
    }
    else
    {
        Budget budget = NormalRows(world, plan, &p, c, stage == m3_contactSolve);
        if (stage == m3_contactRelax)
        {
            TwistRow(&p, c, budget);
            if (c->rollingResistance > 0.0f)
            {
                RollingRow(&p, c, budget);
            }
            FrictionRows(&p, c, budget);
        }
    }
    StorePair(world, c, &p);
}

// Bounce: a point that closed faster than the restitution threshold and
// pushed at some pass is driven apart at restitution times its approach.
static void Restitution(m3World* world, m3ContactConstraint* c)
{
    if (c->restitution == 0.0f)
    {
        return;
    }
    Pair p = LoadPair(world, c);
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        m3ConstraintPoint* cp = &c->points[k];
        if (cp->relativeVelocity > -world->restitutionThreshold || cp->totalNormalImpulse == 0.0f)
        {
            continue;
        }
        m3real vn = m3Dot3(RelativeVelocity(&p, cp->rA, cp->rB), c->normal);
        m3real old = cp->normalImpulse;
        m3real next =
            m3MaxF(old + -cp->normalMass * (vn + c->restitution * cp->relativeVelocity), 0.0f);
        cp->normalImpulse = next;
        Push(&p, c, m3MulSV3(next - old, c->normal), cp->rA, cp->rB);
    }
    StorePair(world, c, &p);
}

// At most one hit event per constraint: its fastest-closing point, when
// a side asked for hits and that point pushed.
static void HitEvent(m3World* world, const m3ContactConstraint* c, int32_t shapeA, int32_t shapeB)
{
    int32_t best = -1;
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        const m3ConstraintPoint* cp = &c->points[k];
        if (cp->relativeVelocity < -world->hitEventThreshold && cp->totalNormalImpulse > 0.0f &&
            (best < 0 || cp->relativeVelocity < c->points[best].relativeVelocity))
        {
            best = k;
        }
    }
    if (best < 0)
    {
        return;
    }
    if (world->events.hitEventCount >= world->contacts.pairCapacity)
    {
        world->events.hitEventsDropped += 1;
        return;
    }
    m3ContactHitEvent* e = &world->events.hitEvents[world->events.hitEventCount++];
    e->shapeIdA =
        (m3ShapeId){shapeA + 1, world->idWorld, world->shapes.shapePool.generations[shapeA]};
    e->shapeIdB =
        (m3ShapeId){shapeB + 1, world->idWorld, world->shapes.shapePool.generations[shapeB]};
    m3Transform t = world->bodies.transforms[c->bodyA];
    m3Vec3 center = m3RotateVec3(t.q, world->bodies.localCenters[c->bodyA]);
    m3Vec3 arm = c->points[best].rA;
    e->point =
        (m3Pos3){t.p.x + (double)center.x + (double)arm.x, t.p.y + (double)center.y + (double)arm.y,
                 t.p.z + (double)center.z + (double)arm.z};
    e->normal = c->normal;
    e->approachSpeed = -c->points[best].relativeVelocity;
}

static void Store(m3World* world, const m3ContactConstraint* c)
{
    m3Manifold* manifold = &world->contacts.manifolds[c->manifoldIndex];
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        manifold->points[k].normalImpulse = c->points[k].normalImpulse;
    }
    uint64_t key = world->contacts.pairKeys[c->manifoldIndex];
    int32_t shapeA = (int32_t)(key >> 32);
    int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
    if (world->shapes.shapeHitEvents[shapeA] != 0 || world->shapes.shapeHitEvents[shapeB] != 0)
    {
        HitEvent(world, c, shapeA, shapeB);
    }
    manifold->frictionImpulse =
        m3Add3(m3MulSV3(c->frictionImpulse1, c->t1), m3MulSV3(c->frictionImpulse2, c->t2));
    manifold->twistImpulse = c->twistImpulse;
    manifold->rollingImpulse = c->rollingImpulse;
}

// Packs each color's constraints, in list order, into lane blocks. The
// overflow stays scalar: its constraints share bodies and run in order.
// A dynamic side is always awake (prepare keeps only awake islands'
// contacts), so every such side is colored and no two lanes of a color
// share one.
static void PackBlocks(m3World* world, m3ContactPlan* plan)
{
    const m3SolverColoring* coloring = &plan->coloring;
    int32_t blockCount = 0;
    for (int32_t color = 0; color < M3_GRAPH_COLORS; ++color)
    {
        plan->blockStarts[color] = blockCount;
        int32_t size = coloring->starts[color + 1] - coloring->starts[color];
        blockCount += (size + M3_LANES - 1) / M3_LANES;
    }
    plan->blockStarts[M3_GRAPH_COLORS] = blockCount;
    plan->blocks = NULL;
    if (world->contacts.scalarRows != 0 || blockCount == 0)
    {
        return;
    }
    plan->blocks = (m3ContactBlock*)m3StackAlloc(&world->scratch,
                                                 blockCount * (int32_t)sizeof(m3ContactBlock));
    if (plan->blocks == NULL)
    {
        return; // the scalar rows give the same result
    }
    for (int32_t color = 0; color < M3_GRAPH_COLORS; ++color)
    {
        const int32_t* list = coloring->lists + coloring->starts[color];
        int32_t size = coloring->starts[color + 1] - coloring->starts[color];
        for (int32_t first = 0; first < size; first += M3_LANES)
        {
            m3ContactBlock* block = &plan->blocks[plan->blockStarts[color] + first / M3_LANES];
            memset(block, 0, sizeof(*block));
            block->lanes = size - first < M3_LANES ? size - first : M3_LANES;
            for (int32_t lane = 0; lane < M3_LANES; ++lane)
            {
                if (lane < block->lanes)
                {
                    int32_t index = list[first + lane];
                    m3PackContactLane(world, block, lane, &plan->constraints[index], index);
                }
                else
                {
                    m3PadContactLane(block, lane);
                }
            }
        }
    }
}

bool m3ColorContacts(m3World* world, m3ContactPlan* plan)
{
    int32_t count = plan->count;
    m3SolverColoring* out = &plan->coloring;
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    out->colors = (uint8_t*)m3StackAlloc(&world->scratch, count > 0 ? count : 1);
    uint32_t* used = (uint32_t*)m3StackAlloc(&world->scratch,
                                             maxBody > 0 ? maxBody * (int32_t)sizeof(uint32_t) : 4);
    out->lists =
        (int32_t*)m3StackAlloc(&world->scratch, count > 0 ? count * (int32_t)sizeof(int32_t) : 4);
    if (out->colors == NULL || used == NULL || out->lists == NULL)
    {
        return false;
    }
    memset(used, 0, (size_t)(maxBody > 0 ? maxBody : 1) * sizeof(uint32_t));
    int32_t counts[M3_GRAPH_COLORS + 1];
    memset(counts, 0, sizeof(counts));
    for (int32_t i = 0; i < count; ++i)
    {
        const m3ContactConstraint* c = &plan->constraints[i];
        bool colorsA = world->bodies.types[c->bodyA] == (uint8_t)m3_dynamicBody &&
                       world->bodies.awake[c->bodyA] != 0;
        bool colorsB = world->bodies.types[c->bodyB] == (uint8_t)m3_dynamicBody &&
                       world->bodies.awake[c->bodyB] != 0;
        uint32_t taken = (colorsA ? used[c->bodyA] : 0u) | (colorsB ? used[c->bodyB] : 0u);
        int32_t color = 0;
        while (color < M3_GRAPH_COLORS && (taken & (1u << color)) != 0u)
        {
            color += 1;
        }
        if (color < M3_GRAPH_COLORS)
        {
            used[c->bodyA] |= colorsA ? 1u << color : 0u;
            used[c->bodyB] |= colorsB ? 1u << color : 0u;
        }
        out->colors[i] = (uint8_t)color;
        counts[color] += 1;
    }
    out->starts[0] = 0;
    for (int32_t color = 0; color <= M3_GRAPH_COLORS; ++color)
    {
        out->starts[color + 1] = out->starts[color] + counts[color];
    }
    int32_t fill[M3_GRAPH_COLORS + 1];
    memcpy(fill, out->starts, sizeof(fill));
    for (int32_t i = 0; i < count; ++i)
    {
        M3_ASSERT(world->bodies.types[plan->constraints[i].bodyA] != (uint8_t)m3_dynamicBody ||
                  world->bodies.awake[plan->constraints[i].bodyA] != 0);
        M3_ASSERT(world->bodies.types[plan->constraints[i].bodyB] != (uint8_t)m3_dynamicBody ||
                  world->bodies.awake[plan->constraints[i].bodyB] != 0);
        out->lists[fill[out->colors[i]]++] = i; // ascending within a color
    }
    PackBlocks(world, plan);
    return true;
}

typedef struct StageTask
{
    m3World* world;
    const m3ContactPlan* plan;
    const int32_t* list;
    m3ContactBlock* blocks;
    m3ContactStage stage;
} StageTask;

static void StageRange(int32_t begin, int32_t end, void* context)
{
    StageTask* task = (StageTask*)context;
    for (int32_t k = begin; k < end; ++k)
    {
        SolveConstraint(task->world, task->plan, &task->plan->constraints[task->list[k]],
                        task->stage);
    }
}

static void BlockRange(int32_t begin, int32_t end, void* context)
{
    StageTask* task = (StageTask*)context;
    for (int32_t k = begin; k < end; ++k)
    {
        m3RunContactBlock(task->world, task->plan, &task->blocks[k], task->stage);
    }
}

// One color of one stage: its lane blocks, or its constraints one by
// one, split among the host's workers as it likes.
static void RunColor(m3World* world, const m3ContactPlan* plan, int32_t color, m3ContactStage stage)
{
    int32_t start = plan->coloring.starts[color];
    int32_t size = plan->coloring.starts[color + 1] - start;
    StageTask task = {world, plan, plan->coloring.lists + start, NULL, stage};
    m3TaskFn* fn = StageRange;
    int32_t items = size;
    int32_t minRange = 8;
    if (plan->blocks != NULL && color < M3_GRAPH_COLORS)
    {
        task.blocks = plan->blocks + plan->blockStarts[color];
        fn = BlockRange;
        items = plan->blockStarts[color + 1] - plan->blockStarts[color];
        minRange = 1;
    }
    if (world->enqueueTask != NULL && size >= 16 && color < M3_GRAPH_COLORS)
    {
        void* handle = world->enqueueTask(fn, items, minRange, &task, world->userTaskContext);
        world->finishTask(handle, world->userTaskContext);
    }
    else if (items > 0)
    {
        fn(0, items, &task);
    }
}

// Colors run in order with a barrier between them; the overflow always
// runs serially. Bounce and store run once, serially, in canonical
// order, on the constraints: the blocks hand their impulses back first.
void m3RunContactStage(m3World* world, const m3ContactPlan* plan, m3ContactStage stage)
{
    if (stage == m3_contactRestitution || stage == m3_contactStore)
    {
        if (stage == m3_contactRestitution && plan->blocks != NULL)
        {
            for (int32_t k = 0; k < plan->blockStarts[M3_GRAPH_COLORS]; ++k)
            {
                m3UnpackContactBlock(&plan->blocks[k], plan->constraints);
            }
        }
        for (int32_t i = 0; i < plan->count; ++i)
        {
            if (stage == m3_contactRestitution)
            {
                Restitution(world, &plan->constraints[i]);
            }
            else
            {
                Store(world, &plan->constraints[i]);
            }
        }
        return;
    }
    for (int32_t color = 0; color <= M3_GRAPH_COLORS; ++color)
    {
        RunColor(world, plan, color, stage);
    }
}
