// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The step. Contacts and joints are prepared once at the step's
// starting pose; then each substep integrates velocities, warm starts,
// solves with the soft push, moves the bodies and relaxes without it
// (see SolveSubsteps); after the substeps come the bounce, the
// impulse store and the joint breaks. Constraint errors inside the step
// are measured from each body's accumulated motion since it began, and
// the rows keep their prepare-time Jacobians. Around the solve sit the
// pair update, the events, continuous collision, sleep, the characters
// and the soft bodies.

#include "solver.h"
#include "body.h"
#include "broad_phase.h"
#include "character.h"
#include "contact_solver.h"
#include "continuous.h"
#include "integrate.h"
#include "island.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "narrowphase.h"
#include "softbody.h"
#include "vehicle.h"
#include "water.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

// With w = 2 pi hertz, stiffness k = m w^2 and damping c = 2 m zeta w, an
// implicit Euler step of length h turns the spring into a row that solves
//   J v + (k / (h k + c)) C + (1 / (h (h k + c))) lambda = 0
// for the total impulse lambda. Let a = h w (2 zeta + h w). Then
//   biasRate     = k / (h k + c) = w / (2 zeta + h w)
//   massScale    = a / (1 + a)   (m against m + the lambda term)
//   impulseScale = 1 / (1 + a)   (the lambda term on the accumulated part)
// and a solve adds -m massScale (J v + biasRate C) - impulseScale
// accumulated. Zero hertz is a rigid row with no position feedback.
m3Softness m3MakeSoft(m3real hertz, m3real zeta, m3real h)
{
    if (hertz == 0.0f)
    {
        return (m3Softness){0.0f, 0.0f, 0.0f};
    }
    m3real omega = 2.0f * M3_PI * hertz;
    m3real a1 = 2.0f * zeta + h * omega;
    m3real a2 = h * omega * a1;
    m3real a3 = 1.0f / (1.0f + a2);
    return (m3Softness){omega / a1, a2 * a3, a3};
}

// I_w^-1 = R I_l^-1 R^T, built by applying the operator to the world
// basis vectors. Fixed at prepare like the contact anchors.
m3Mat3 m3WorldInvInertia(const m3World* world, int32_t body)
{
    if (world->bodies.types[body] != (uint8_t)m3_dynamicBody)
    {
        return m3MakeZeroMat3();
    }
    m3Quat q = world->bodies.transforms[body].q;
    m3Mat3 il = world->bodies.invInertiaLocal[body];
    m3Mat3 r;
    r.cx = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f})));
    r.cy = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 1.0f, 0.0f})));
    r.cz = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 0.0f, 1.0f})));
    return r;
}

// Solve J * x = b for a general 3x3 via Cramer's rule. A singular
// Jacobian returns zero, which leaves omega unchanged (the safe step).
m3Vec3 m3Solve3(const m3Mat3* J, m3Vec3 b)
{
    m3Vec3 cxy = m3Cross3(J->cy, J->cz);
    m3real det = m3Dot3(J->cx, cxy);
    if (det == 0.0f)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    m3real inv = 1.0f / det;
    m3Vec3 x;
    x.x = inv * m3Dot3(b, cxy);
    x.y = inv * m3Dot3(J->cx, m3Cross3(b, J->cz));
    x.z = inv * m3Dot3(J->cx, m3Cross3(J->cy, b));
    return x;
}

static void SizeScratchForStep(m3World* world)
{
    // Pre-flight sizing: a starved step is deterministic, size-driven
    // state evolution, and struct sizes are not part of the
    // cross-platform contract. The estimate uses pinned per-item byte
    // budgets chosen to dominate every platform's real sizes, and it
    // runs after the pair scan so the pair count is this step's (a
    // one-step lag let consumption race capacity on pileup spikes, and
    // the winner depended on sizeof). Counts are pure state, so every
    // platform grows on the same tick; the NULL checks that follow are
    // backstops a correct run never reaches.
    {
        int64_t need = 64 * 1024 + 128 * (int64_t)world->bodies.bodyPool.maxIndex +
                       64 * (int64_t)world->shapes.shapePool.maxIndex +
                       1536 * (int64_t)world->contacts.pairCount + 64 * 1024 +
                       1024 * (int64_t)world->joints.jointPool.maxIndex;
        if (need > (int64_t)world->scratch.capacity && world->scratch.capacity < (1 << 28))
        {
            int32_t grown = world->scratch.capacity;
            while ((int64_t)grown < need && grown < (1 << 28))
            {
                grown *= 2;
            }
            m3StackDestroy(&world->scratch);
            world->scratch = m3StackCreate(grown);
        }
    }
}

// Begin-of-step centers of mass and rotations: the sweeps the continuous
// pass needs.
static void CaptureSweepStarts(const m3World* world, m3Pos3* com0, m3Quat* rot0)
{
    int32_t sweepMax = world->bodies.bodyPool.maxIndex;
    for (int32_t i = 0; i < sweepMax; ++i)
    {
        if (world->bodies.bodyPool.alive[i] == 0)
        {
            continue;
        }
        m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[i].q, world->bodies.localCenters[i]);
        com0[i].x = world->bodies.transforms[i].p.x + (double)rlc.x;
        com0[i].y = world->bodies.transforms[i].p.y + (double)rlc.y;
        com0[i].z = world->bodies.transforms[i].p.z + (double)rlc.z;
        rot0[i] = world->bodies.transforms[i].q;
    }
}

static void AdvanceWind(m3World* world, float dt)
{
    // Wind phase: accumulated STATE, so a rollback resumes
    // the exact same gust wave. Wrapped to keep the float precise.
    if (world->windGustHertz > 0.0f)
    {
        world->windPhase += 2.0f * M3_PI * world->windGustHertz * dt;
        if (world->windPhase > 2.0f * M3_PI)
        {
            world->windPhase -= 2.0f * M3_PI * (m3real)(int32_t)(world->windPhase / (2.0f * M3_PI));
        }
    }
}

static void BreakJoints(m3World* world, m3real invH)
{
    // Joint breakage: reactions over threshold destroy the
    // joint and emit the break event, serially in ascending joint
    // order, a pure function of state (like fragmentation: derived
    // transitions never need their own journal op).
    {
        int32_t maxJoint = world->joints.jointPool.maxIndex;
        for (int32_t j = 0; j < maxJoint; ++j)
        {
            if (world->joints.jointPool.alive[j] == 0)
            {
                continue;
            }
            m3real maxForce = world->joints.jointBreak[j].x;
            m3real maxTorque = world->joints.jointBreak[j].y;
            if (maxForce == 0.0f && maxTorque == 0.0f)
            {
                continue;
            }
            m3real force;
            m3real torque;
            m3JointReactionMagnitudes(world, j, invH, &force, &torque);
            if ((maxForce > 0.0f && force > maxForce) || (maxTorque > 0.0f && torque > maxTorque))
            {
                m3JointId id = {j + 1, world->idWorld, world->joints.jointPool.generations[j]};
                m3AppendJointBreakEvent(world, id);
                m3DestroyJointInternal(world, j);
            }
        }
    }
}

// Labels every awake dynamic body with its island root and returns the
// island count (observer data).
static int32_t LabelIslands(m3World* world, const int32_t* islandParent)
{
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    // Island census before the sleep pass retires anyone: awake
    // dynamic union-find roots, an observer count, and the
    // per-body island label the extras draw tints by.
    // Sleeping bodies keep the label of the island they slept in.
    int32_t islands = 0;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodies.bodyPool.alive[i] != 0 &&
            world->bodies.types[i] == (uint8_t)m3_dynamicBody && world->bodies.awake[i] != 0)
        {
            int32_t root = i;
            while (islandParent[root] != root)
            {
                root = islandParent[root];
            }
            world->bodies.bodyIsland[i] = root;
            if (root == i)
            {
                islands += 1;
            }
        }
    }
    return islands;
}

// Everything one step takes from the scratch stack, in the order it takes
// it. The order is part of the stall behavior: a starved step fails at
// the same allocation on every platform.
typedef struct m3StepScratch
{
    m3Pos3* com0; // begin-of-step centers of mass (continuous, sleep, riders)
    m3Quat* rot0;
    int32_t* islandParent;
    m3ContactPlan contacts;
    m3Vec3* deltaPos; // per-body position and rotation drift within the step
    m3Quat* deltaRot;
    int32_t usedColors;
    m3JointConstraint* joints;
    int32_t jointCount;
    int32_t* movers;
    int32_t moverCount;
    m3Buoyancy buoy;
    m3real h;
    m3real invH;
} m3StepScratch;

// A step that starved the scratch stalled harmlessly; the next one
// arrives with double the room. Growth is driven by sizes alone, so twins
// and replays stall and grow on the same ticks.
static void GrowScratchAfterStall(m3World* world)
{
    if (world->scratch.overflow != 0 && world->scratch.capacity < (1 << 28))
    {
        int32_t bigger = world->scratch.capacity * 2;
        m3StackDestroy(&world->scratch);
        world->scratch = m3StackCreate(bigger);
    }
}

// Copies the previous pairs and manifolds aside before the pair scan
// overwrites them; the warm-start carry and the event walk read the copy.
// The buffers belong to the world, so this never allocates.
static int32_t StashPairs(m3World* world)
{
    int32_t oldCount = world->contacts.pairCount;
    if (oldCount > 0)
    {
        memcpy(world->contacts.stashPairKeys, world->contacts.pairKeys,
               (size_t)oldCount * sizeof(uint64_t));
        memcpy(world->contacts.stashManifolds, world->contacts.manifolds,
               (size_t)oldCount * sizeof(m3Manifold));
    }
    return oldCount;
}

// Empties the step's event streams and emits the contact changes.
static void StartEvents(m3World* world, const uint64_t* oldKeys, const m3Manifold* oldManifolds,
                        int32_t oldCount)
{
    m3ResetStepEvents(world);
    m3EmitContactEvents(world, oldKeys, oldManifolds, oldCount);
}

static void* ScratchArray(m3World* world, int32_t count, int32_t elementSize)
{
    return m3StackAlloc(&world->scratch, count > 0 ? count * elementSize : elementSize);
}

// The sweep starts, the island wake pass (a sleeping body touched by an
// awake one joins this step) and the per-body drift accumulators. Returns
// false on a scratch stall.
static bool BeginStep(m3World* world, m3StepScratch* s)
{
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    s->com0 = (m3Pos3*)ScratchArray(world, maxBody, (int32_t)sizeof(m3Pos3));
    s->rot0 = (m3Quat*)ScratchArray(world, maxBody, (int32_t)sizeof(m3Quat));
    if (s->com0 == NULL || s->rot0 == NULL)
    {
        return false; // unreachable after SizeScratchForStep; a backstop
    }
    CaptureSweepStarts(world, s->com0, s->rot0);
    s->islandParent = m3IslandWakePass(world);
    if (s->islandParent == NULL)
    {
        return false;
    }
    s->contacts.constraints = (m3ContactConstraint*)ScratchArray(
        world, world->contacts.pairCount, (int32_t)sizeof(m3ContactConstraint));
    s->deltaPos = (m3Vec3*)ScratchArray(world, maxBody, (int32_t)sizeof(m3Vec3));
    s->deltaRot = (m3Quat*)ScratchArray(world, maxBody, (int32_t)sizeof(m3Quat));
    if (s->contacts.constraints == NULL || s->deltaPos == NULL || s->deltaRot == NULL)
    {
        return false;
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        s->deltaPos[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
        s->deltaRot[i] = m3MakeIdentityQuat();
    }
    return true;
}

// Contact and joint constraints, the graph coloring, the mover list and
// the water field for the substep loop. Returns false on a scratch stall.
static bool PrepareSolve(m3World* world, m3StepScratch* s, float dt, int32_t substeps)
{
    s->h = dt / (m3real)substeps;
    s->invH = s->h > 0.0f ? 1.0f / s->h : 0.0f;
    s->contacts.count = m3PrepareContacts(world, s->contacts.constraints, s->h);
    s->contacts.deltaPos = s->deltaPos;
    s->contacts.deltaRot = s->deltaRot;
    s->contacts.invH = s->invH;
    if (!m3ColorContacts(world, &s->contacts))
    {
        return false;
    }
    s->usedColors = 0;
    for (int32_t c = 0; c < M3_GRAPH_COLORS + 1; ++c)
    {
        if (s->contacts.coloring.starts[c + 1] > s->contacts.coloring.starts[c])
        {
            s->usedColors += 1;
        }
    }
    s->joints = (m3JointConstraint*)ScratchArray(world, world->joints.jointPool.maxIndex,
                                                 (int32_t)sizeof(m3JointConstraint));
    if (s->joints == NULL)
    {
        return false;
    }
    s->jointCount = m3PrepareJoints(world, s->joints, s->h);
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    s->movers = (int32_t*)m3StackAlloc(&world->scratch,
                                       maxBody > 0 ? maxBody * (int32_t)sizeof(int32_t) : 4);
    if (s->movers == NULL)
    {
        return false;
    }
    s->moverCount = m3BuildMovers(world, s->movers, dt);
    m3PrepareBuoyancy(world, s->movers, s->moverCount, &s->buoy);
    return true;
}

// The substeps: forces in; warm start, then the biased pass pushing
// overlap and joint error out (joints first, so contacts see the
// joints' result); positions move; then the relax pass, unbiased,
// removes the velocity the push added.
static void SolveSubsteps(m3World* world, m3StepScratch* s, int32_t substeps)
{
    for (int32_t sub = 0; sub < substeps; ++sub)
    {
        m3IntegrateVelocities(world, s->movers, s->moverCount, &s->buoy, s->h);
        m3WarmStartJoints(world, s->joints, s->jointCount, s->deltaPos, s->deltaRot);
        m3RunContactStage(world, &s->contacts, m3_contactWarmStart);
        m3SolveJoints(world, s->joints, s->jointCount, s->deltaPos, s->deltaRot, s->h, s->invH,
                      true);
        m3RunContactStage(world, &s->contacts, m3_contactSolve);
        m3IntegratePositions(world, s->movers, s->moverCount, s->deltaPos, s->deltaRot, s->h);
        m3SolveJoints(world, s->joints, s->jointCount, s->deltaPos, s->deltaRot, s->h, s->invH,
                      false);
        m3RunContactStage(world, &s->contacts, m3_contactRelax);
    }
}

// After the substeps: host forces are consumed (a force lives for one
// step, and only movers can carry one), restitution runs, impulses are
// stored for the next warm start, the wind advances and overloaded
// joints break.
static void FinishSolve(m3World* world, const m3StepScratch* s, float dt)
{
    for (int32_t m = 0; m < s->moverCount; ++m)
    {
        world->bodies.bodyForce[s->movers[m]] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->bodies.bodyTorque[s->movers[m]] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    m3RunContactStage(world, &s->contacts, m3_contactRestitution);
    m3RunContactStage(world, &s->contacts, m3_contactStore);
    m3StoreJointImpulses(world, s->joints, s->jointCount);
    world->lastInvH = s->invH;
    AdvanceWind(world, dt);
    BreakJoints(world, s->invH);
}

void m3StepInternal(m3World* world, float dt, int32_t substeps)
{
    // The profile is written to the world only when the step completes,
    // so a stalled step keeps the previous one. Observer data only.
    m3Profile prof;
    memset(&prof, 0, sizeof(prof));
    double tStep = m3NowMs();

    GrowScratchAfterStall(world);
    int32_t oldCount = StashPairs(world);
    const uint64_t* oldKeys = world->contacts.stashPairKeys;
    const m3Manifold* oldManifolds = world->contacts.stashManifolds;

    // Vehicle suspension impulses land first, so the narrow phase and the
    // solver see the sprung chassis the way they see gravity.
    double t0 = m3NowMs();
    m3VehicleApplySuspension(world, dt);
    prof.vehicles = (float)(m3NowMs() - t0);

    t0 = m3NowMs();
    m3Result pairsResult = m3UpdatePairs(world);
    prof.broadphase = (float)(m3NowMs() - t0);
    t0 = m3NowMs();
    m3Result contactsResult = pairsResult == m3_success
                                  ? m3UpdateContacts(world, oldKeys, oldManifolds, oldCount)
                                  : pairsResult;
    prof.narrowphase = (float)(m3NowMs() - t0);
    if (pairsResult != m3_success || contactsResult != m3_success)
    {
        return; // a scratch stall (grown next step) or a full pair table:
                // the world stalls, never corrupts
    }

    t0 = m3NowMs();
    StartEvents(world, oldKeys, oldManifolds, oldCount);
    prof.events = (float)(m3NowMs() - t0);

    t0 = m3NowMs();
    SizeScratchForStep(world);
    m3StackReset(&world->scratch);
    m3StepScratch s;
    memset(&s, 0, sizeof(s));
    if (!BeginStep(world, &s))
    {
        return; // scratch stall, grown next step
    }
    prof.prepare = (float)(m3NowMs() - t0);

    t0 = m3NowMs();
    if (!PrepareSolve(world, &s, dt, substeps))
    {
        return; // scratch stall, grown next step
    }
    SolveSubsteps(world, &s, substeps);
    FinishSolve(world, &s, dt);
    prof.solve = (float)(m3NowMs() - t0);

    t0 = m3NowMs();
    if (world->continuousEnabled != 0)
    {
        m3SolveContinuousPhase(world, s.com0, s.rot0);
    }
    prof.continuous = (float)(m3NowMs() - t0);
    int32_t islands = LabelIslands(world, s.islandParent);
    t0 = m3NowMs();
    if (world->sleepEnabled != 0)
    {
        m3IslandSleepPass(world, s.islandParent, s.com0, s.rot0, dt);
    }
    prof.sleep = (float)(m3NowMs() - t0);

    m3EmitMoveEvents(world, s.movers, s.moverCount);
    t0 = m3NowMs();
    m3CharacterCarryRiders(world, s.com0, s.rot0);
    prof.characters = (float)(m3NowMs() - t0);
    t0 = m3NowMs();
    m3SoftBodyPass(world, dt, substeps);
    prof.softBodies = (float)(m3NowMs() - t0);

    world->stepCount += 1;
    prof.step = (float)(m3NowMs() - tStep);
    world->profile = prof;
    world->lastIslandCount = islands;
    world->lastColorCount = s.usedColors;
    world->lastScratchPeak = world->scratch.top;
}

void m3World_Step(m3WorldId worldId, float dt, int32_t substeps)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(dt) || !(dt > 0.0f) || substeps < 1 ||
        substeps > M3_MAX_SUBSTEPS)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    world->contacts.stepVetoCount = 0;
    m3StepInternal(world, dt, substeps);
    if (world->recorder.journalActive != 0)
    {
        // Recording moved BEHIND the execution: nothing can
        // journal during a step, so callback-less streams are
        // byte-identical to the old order, and a step that vetoed
        // contacts writes those keys first. A bare replay (no
        // callback installed) then applies the recorded vetoes and
        // lands on the recorded bits: the tape is self-sufficient.
        if (world->contacts.stepVetoCount > 0)
        {
            m3JournalRecord(world, m3_opStepVetoes, world->contacts.stepVetoKeys,
                            world->contacts.stepVetoCount * (int32_t)sizeof(uint64_t));
        }
        m3OpStep record;
        memset(&record, 0, sizeof(record));
        record.dt = dt;
        record.substeps = substeps;
        m3JournalRecord(world, m3_opStep, &record, (int32_t)sizeof(record));
    }
}
