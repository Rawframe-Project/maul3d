// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Soft bodies: XPBD particle lattices and tetrahedral bodies, their
// creation, anchors and readback. The pass that moves them lives in
// softbody_solver.c.

#include "maul3d/softbody.h"

#include "body.h"
#include "journal.h"
#include "shape.h"
#include "softbody.h"
#include "solver.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

#define M3_SOFTBODY_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3SoftBodyDef) << 8) ^ 9))

m3SoftBodyDef m3DefaultSoftBodyDef(void)
{
    m3SoftBodyDef def;
    memset(&def, 0, sizeof(def));
    def.countX = 4;
    def.countY = 4;
    def.countZ = 4;
    def.spacing = 0.25f;
    def.particleMass = 0.1f;
    def.compliance = 0.0f;
    def.radius = 0.05f;
    def.gravityScale = 1.0f;
    def.bendCompliance = 0.0f;
    def.pressure = 0.0f;
    def.maxDeviation = 0.0f;
    def.internalValue = M3_SOFTBODY_COOKIE;
    return def;
}

int32_t m3SoftBodySlot(const m3World* world, m3SoftBodyId softId)
{
    int32_t index = softId.index1 - 1;
    if (world == NULL || softId.world != world->idWorld ||
        !m3IdPoolValid(&world->softBodies.softPool, index, softId.generation))
    {
        return -1;
    }
    return index;
}

static void AddEdge(m3World* world, int32_t slot, int32_t a, int32_t b, m3real rest)
{
    int32_t e = world->softBodies.softEdgeCount[slot];
    if (e >= M3_SOFTBODY_MAX_EDGES)
    {
        return; // the factory sizes below the cap by construction
    }
    int32_t k = slot * M3_SOFTBODY_MAX_EDGES + e;
    world->softBodies.softEdgeA[k] = (uint16_t)a;
    world->softBodies.softEdgeB[k] = (uint16_t)b;
    world->softBodies.softEdgeRest[k] = rest;
    world->softBodies.softEdgeCount[slot] = e + 1;
}

// Edges a lattice brings: structural edges along the axes and face
// diagonals, plus second-neighbor bend tethers when bending is on.
static int64_t LatticeEdgeCount(const m3SoftBodyDef* def)
{
    int64_t nx = def->countX;
    int64_t ny = def->countY;
    int64_t nz = def->countZ;
    int64_t axes = (nx - 1) * ny * nz + nx * (ny - 1) * nz + nx * ny * (nz - 1);
    int64_t diagonals =
        2 * ((nx - 1) * (ny - 1) * nz + (nx - 1) * ny * (nz - 1) + nx * (ny - 1) * (nz - 1));
    int64_t bend = 0;
    if (def->bendCompliance > 0.0f)
    {
        bend = (nx > 2 ? (nx - 2) * ny * nz : 0) + (ny > 2 ? nx * (ny - 2) * nz : 0) +
               (nz > 2 ? nx * ny * (nz - 2) : 0);
    }
    return axes + diagonals + bend;
}

// Every field check lives here, where the public door and replay both
// pass: replay hands raw journal bytes, and a flipped bit could produce
// NaN positions or an overflowing particle count. The edge count is
// checked before a slot is taken, so a refusal moves no id.
static bool LatticeDefValid(const m3SoftBodyDef* def)
{
    if (def->countX < 1 || def->countY < 1 || def->countZ < 1 ||
        (int64_t)def->countX * def->countY * def->countZ > M3_SOFTBODY_MAX_PARTICLES)
    {
        return false;
    }
    bool pressureFits =
        def->pressure == 0.0f || (def->countX >= 2 && def->countY >= 2 && def->countZ >= 2);
    return m3FinitePos3(def->position) && m3FiniteF(def->spacing) && def->spacing > 0.0f &&
           m3FiniteF(def->particleMass) && def->particleMass > 0.0f && m3FiniteF(def->compliance) &&
           def->compliance >= 0.0f && m3FiniteF(def->radius) && def->radius > 0.0f &&
           m3FiniteF(def->gravityScale) && m3FiniteF(def->bendCompliance) &&
           def->bendCompliance >= 0.0f && m3FiniteF(def->pressure) && def->pressure >= 0.0f &&
           pressureFits && m3FiniteF(def->maxDeviation) && def->maxDeviation >= 0.0f &&
           LatticeEdgeCount(def) <= M3_SOFTBODY_MAX_EDGES;
}

// The per-body scalars a create writes, from a def or a tet recipe.
static void InitSoftSlot(m3World* world, int32_t slot, const m3SoftBodyDef* def,
                         int32_t particleCount)
{
    m3SoftBodies* sb = &world->softBodies;
    sb->softParticleCount[slot] = particleCount;
    sb->softEdgeCount[slot] = 0;
    sb->softAnchorCount[slot] = 0;
    sb->softSoftCount[slot] = 0;
    sb->softCompliance[slot] = def->compliance;
    sb->softRadius[slot] = def->radius;
    sb->softGravityScale[slot] = def->gravityScale;
    sb->softUserData[slot] = def->userData;
    sb->softTetCount[slot] = 0;
    sb->softMaxDeviation[slot] = def->maxDeviation;
}

static void SeedParticle(m3World* world, int32_t k, m3Pos3 p, m3real invMass)
{
    world->softBodies.softPos[k] = p;
    world->softBodies.softPrev[k] = p;
    world->softBodies.softBindPos[k] = p;
    world->softBodies.softInvMass[k] = invMass;
    world->softBodies.softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
}

// Structural edges along the axes, then face diagonals, in one fixed
// nested order.
static void AddLatticeEdges(m3World* world, int32_t slot, int32_t nx, int32_t ny, int32_t nz,
                            m3real s)
{
    m3real d = s * 1.41421356237f;
    int32_t layer = nx * ny;
    for (int32_t z = 0; z < nz; ++z)
    {
        for (int32_t y = 0; y < ny; ++y)
        {
            for (int32_t x = 0; x < nx; ++x)
            {
                int32_t i = x + nx * (y + ny * z);
                bool px = x + 1 < nx;
                bool py = y + 1 < ny;
                bool pz = z + 1 < nz;
                if (px)
                {
                    AddEdge(world, slot, i, i + 1, s);
                }
                if (py)
                {
                    AddEdge(world, slot, i, i + nx, s);
                }
                if (pz)
                {
                    AddEdge(world, slot, i, i + layer, s);
                }
                if (px && py)
                {
                    AddEdge(world, slot, i, i + 1 + nx, d);
                    AddEdge(world, slot, i + 1, i + nx, d);
                }
                if (px && pz)
                {
                    AddEdge(world, slot, i, i + 1 + layer, d);
                    AddEdge(world, slot, i + 1, i + layer, d);
                }
                if (py && pz)
                {
                    AddEdge(world, slot, i, i + nx + layer, d);
                    AddEdge(world, slot, i + nx, i + layer, d);
                }
            }
        }
    }
}

// Second-neighbor bend tethers along each axis, after every structural
// edge so one boundary index splits the two compliances.
static void AddBendTethers(m3World* world, int32_t slot, int32_t nx, int32_t ny, int32_t nz,
                           m3real s)
{
    m3real s2 = 2.0f * s;
    for (int32_t z = 0; z < nz; ++z)
    {
        for (int32_t y = 0; y < ny; ++y)
        {
            for (int32_t x = 0; x < nx; ++x)
            {
                int32_t i = x + nx * (y + ny * z);
                if (x + 2 < nx)
                {
                    AddEdge(world, slot, i, i + 2, s2);
                }
                if (y + 2 < ny)
                {
                    AddEdge(world, slot, i, i + 2 * nx, s2);
                }
                if (z + 2 < nz)
                {
                    AddEdge(world, slot, i, i + 2 * nx * ny, s2);
                }
            }
        }
    }
}

int32_t m3CreateSoftBodyInternal(m3World* world, const m3SoftBodyDef* def)
{
    if (!LatticeDefValid(def))
    {
        return -1;
    }
    int32_t slot = m3IdPoolAlloc(&world->softBodies.softPool);
    if (slot < 0)
    {
        return -1;
    }
    int32_t nx = def->countX;
    int32_t ny = def->countY;
    int32_t nz = def->countZ;
    m3SoftBodies* sb = &world->softBodies;
    InitSoftSlot(world, slot, def, nx * ny * nz);
    sb->softBendCompliance[slot] = def->bendCompliance;
    sb->softDimX[slot] = (uint16_t)nx;
    sb->softDimY[slot] = (uint16_t)ny;
    sb->softDimZ[slot] = (uint16_t)nz;
    sb->softPressure[slot] = def->pressure;
    // The create lattice is a perfect grid: the rest volume is the closed box.
    sb->softRestVolume[slot] = (m3real)(nx - 1) * (m3real)(ny - 1) * (m3real)(nz - 1) *
                               def->spacing * def->spacing * def->spacing;
    m3real invMass = 1.0f / def->particleMass;
    for (int32_t z = 0; z < nz; ++z)
    {
        for (int32_t y = 0; y < ny; ++y)
        {
            for (int32_t x = 0; x < nx; ++x)
            {
                m3Pos3 p = {def->position.x + (double)((m3real)x * def->spacing),
                            def->position.y + (double)((m3real)y * def->spacing),
                            def->position.z + (double)((m3real)z * def->spacing)};
                SeedParticle(world, slot * M3_SOFTBODY_MAX_PARTICLES + x + nx * (y + ny * z), p,
                             invMass);
            }
        }
    }
    AddLatticeEdges(world, slot, nx, ny, nz, def->spacing);
    sb->softBendStart[slot] = sb->softEdgeCount[slot];
    if (def->bendCompliance > 0.0f)
    {
        AddBendTethers(world, slot, nx, ny, nz, def->spacing);
    }
    return slot;
}

// Zeroes every array a soft body wrote, so its slot reads like a fresh one.
static void ClearSoftSlot(m3World* world, int32_t slot)
{
    int32_t count = world->softBodies.softParticleCount[slot];
    for (int32_t i = 0; i < count; ++i)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_PARTICLES + i;
        world->softBodies.softPos[k] = (m3Pos3){0.0, 0.0, 0.0};
        world->softBodies.softPrev[k] = (m3Pos3){0.0, 0.0, 0.0};
        world->softBodies.softInvMass[k] = 0.0f;
        world->softBodies.softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    int32_t edges = world->softBodies.softEdgeCount[slot];
    for (int32_t e = 0; e < edges; ++e)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_EDGES + e;
        world->softBodies.softEdgeA[k] = 0;
        world->softBodies.softEdgeB[k] = 0;
        world->softBodies.softEdgeRest[k] = 0.0f;
    }
    world->softBodies.softSoftCount[slot] = 0;
    for (int32_t a = 0; a < world->softBodies.softAnchorCount[slot]; ++a)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_ANCHORS + a;
        world->softBodies.softAnchorParticle[k] = 0;
        world->softBodies.softAnchorBody[k] = 0;
        world->softBodies.softAnchorGen[k] = 0;
        world->softBodies.softAnchorLocal[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    world->softBodies.softAnchorCount[slot] = 0;
    world->softBodies.softParticleCount[slot] = 0;
    world->softBodies.softEdgeCount[slot] = 0;
    world->softBodies.softCompliance[slot] = 0.0f;
    world->softBodies.softRadius[slot] = 0.0f;
    world->softBodies.softGravityScale[slot] = 0.0f;
    world->softBodies.softUserData[slot] = 0;
    world->softBodies.softBendStart[slot] = 0;
    world->softBodies.softBendCompliance[slot] = 0.0f;
    world->softBodies.softDimX[slot] = 0;
    world->softBodies.softDimY[slot] = 0;
    world->softBodies.softDimZ[slot] = 0;
    world->softBodies.softRestVolume[slot] = 0.0f;
    world->softBodies.softPressure[slot] = 0.0f;
    int32_t tets = world->softBodies.softTetCount[slot];
    for (int32_t t = 0; t < tets; ++t)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_TETS + t;
        world->softBodies.softTetA[k] = 0;
        world->softBodies.softTetB[k] = 0;
        world->softBodies.softTetC[k] = 0;
        world->softBodies.softTetD[k] = 0;
        world->softBodies.softTetRestV6[k] = 0.0f;
    }
    world->softBodies.softTetCount[slot] = 0;
    int32_t bcount = M3_SOFTBODY_MAX_PARTICLES;
    for (int32_t i = 0; i < bcount; ++i)
    {
        world->softBodies.softBindPos[slot * M3_SOFTBODY_MAX_PARTICLES + i] =
            (m3Pos3){0.0, 0.0, 0.0};
    }
    world->softBodies.softMaxDeviation[slot] = 0.0f;
}

void m3DestroySoftBodyInternal(m3World* world, int32_t slot)
{
    ClearSoftSlot(world, slot);
    m3IdPoolFree(&world->softBodies.softPool, slot);
}

// Tet soft bodies: explicit points and tets, edges deduped
// from tet edges in first-touch order, one rigid volume row per
// tet. The full wall lives here: replay hands this raw bytes.
static int32_t TetEdgeSeen(const uint16_t* ea, const uint16_t* eb, int32_t count, uint16_t lo,
                           uint16_t hi)
{
    for (int32_t e = 0; e < count; ++e)
    {
        if (ea[e] == lo && eb[e] == hi)
        {
            return 1;
        }
    }
    return 0;
}

static m3real TetVolume6(const m3Vec3* points, const uint16_t* tet)
{
    m3Vec3 e1 = m3Sub3(points[tet[1]], points[tet[0]]);
    m3Vec3 e2 = m3Sub3(points[tet[2]], points[tet[0]]);
    m3Vec3 e3 = m3Sub3(points[tet[3]], points[tet[0]]);
    return m3Dot3(e1, m3Cross3(e2, e3));
}

// Lattice knobs must sit at their defaults: a tet body has no grid to
// bend or pressurize. Every tet names four distinct points and has a
// positive rest volume, which is its constraint target.
static bool TetDefValid(const m3SoftBodyDef* def, const m3Vec3* points, int32_t pointCount,
                        const uint16_t* tets, int32_t tetCount)
{
    if (pointCount < 4 || pointCount > M3_SOFTBODY_MAX_PARTICLES || tetCount < 1 ||
        tetCount > M3_SOFTBODY_MAX_TETS || !m3FinitePos3(def->position) ||
        !m3FiniteF(def->particleMass) || !(def->particleMass > 0.0f) ||
        !m3FiniteF(def->compliance) || def->compliance < 0.0f || !m3FiniteF(def->radius) ||
        !(def->radius > 0.0f) || !m3FiniteF(def->gravityScale) || def->bendCompliance != 0.0f ||
        def->pressure != 0.0f || !m3FiniteF(def->maxDeviation) || def->maxDeviation < 0.0f)
    {
        return false;
    }
    for (int32_t i = 0; i < pointCount; ++i)
    {
        if (!m3FiniteV3(points[i]))
        {
            return false;
        }
    }
    for (int32_t t = 0; t < tetCount; ++t)
    {
        const uint16_t* v = &tets[4 * t];
        if (v[0] >= pointCount || v[1] >= pointCount || v[2] >= pointCount || v[3] >= pointCount ||
            v[0] == v[1] || v[0] == v[2] || v[0] == v[3] || v[1] == v[2] || v[1] == v[3] ||
            v[2] == v[3] || !(TetVolume6(points, v) > 1.0e-9f))
        {
            return false;
        }
    }
    return true;
}

// The six edges of every tet, deduplicated in first-touch tet order on
// the slot's own edge arrays. False when they outgrow the edge budget.
static bool AddTetEdges(m3World* world, int32_t slot, const m3Vec3* points, const uint16_t* tets,
                        int32_t tetCount)
{
    int32_t ebase = slot * M3_SOFTBODY_MAX_EDGES;
    static const int32_t pairs[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (int32_t t = 0; t < tetCount; ++t)
    {
        for (int32_t e = 0; e < 6; ++e)
        {
            uint16_t va = tets[4 * t + pairs[e][0]];
            uint16_t vb = tets[4 * t + pairs[e][1]];
            uint16_t lo = va < vb ? va : vb;
            uint16_t hi = va < vb ? vb : va;
            int32_t count = world->softBodies.softEdgeCount[slot];
            if (TetEdgeSeen(&world->softBodies.softEdgeA[ebase],
                            &world->softBodies.softEdgeB[ebase], count, lo, hi))
            {
                continue;
            }
            if (count >= M3_SOFTBODY_MAX_EDGES)
            {
                return false;
            }
            m3Vec3 d = m3Sub3(points[hi], points[lo]);
            AddEdge(world, slot, lo, hi, sqrtf(m3Dot3(d, d)));
        }
    }
    return true;
}

int32_t m3CreateSoftBodyTetInternal(m3World* world, const m3SoftBodyDef* def, const m3Vec3* points,
                                    int32_t pointCount, const uint16_t* tets, int32_t tetCount)
{
    if (!TetDefValid(def, points, pointCount, tets, tetCount))
    {
        return -1;
    }
    m3IdPoolMark mark = m3IdPoolMarkNow(&world->softBodies.softPool);
    int32_t slot = m3IdPoolAlloc(&world->softBodies.softPool);
    if (slot < 0)
    {
        return -1;
    }
    m3SoftBodies* sb = &world->softBodies;
    InitSoftSlot(world, slot, def, pointCount);
    sb->softBendStart[slot] = 0;
    sb->softBendCompliance[slot] = 0.0f;
    sb->softDimX[slot] = 0;
    sb->softDimY[slot] = 0;
    sb->softDimZ[slot] = 0;
    sb->softRestVolume[slot] = 0.0f;
    sb->softPressure[slot] = 0.0f;
    m3real invMass = 1.0f / def->particleMass;
    for (int32_t i = 0; i < pointCount; ++i)
    {
        m3Pos3 p = {def->position.x + (double)points[i].x, def->position.y + (double)points[i].y,
                    def->position.z + (double)points[i].z};
        SeedParticle(world, slot * M3_SOFTBODY_MAX_PARTICLES + i, p, invMass);
    }
    if (!AddTetEdges(world, slot, points, tets, tetCount))
    {
        // The refusal leaves the pool as if the create never ran.
        ClearSoftSlot(world, slot);
        m3IdPoolRewind(&world->softBodies.softPool, mark, slot);
        return -1;
    }
    sb->softBendStart[slot] = sb->softEdgeCount[slot];
    sb->softTetCount[slot] = tetCount;
    for (int32_t t = 0; t < tetCount; ++t)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_TETS + t;
        sb->softTetA[k] = tets[4 * t + 0];
        sb->softTetB[k] = tets[4 * t + 1];
        sb->softTetC[k] = tets[4 * t + 2];
        sb->softTetD[k] = tets[4 * t + 3];
        sb->softTetRestV6[k] = TetVolume6(points, &tets[4 * t]); // from the create pose
    }
    return slot;
}

m3SoftBodyId m3CreateSoftBodyTet(m3WorldId worldId, const m3SoftBodyDef* def, const m3Vec3* points,
                                 int32_t pointCount, const uint16_t* tets, int32_t tetCount)
{
    m3SoftBodyId null = {0, 0, 0};
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_SOFTBODY_COOKIE ||
        points == NULL || tets == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return null;
    }
    int32_t slot = m3CreateSoftBodyTetInternal(world, def, points, pointCount, tets, tetCount);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return null;
    }
    m3SoftBodyId id = {slot + 1, world->idWorld, world->softBodies.softPool.generations[slot]};
    if (world->recorder.journalActive != 0)
    {
        m3CreateSoftBodyTetOp head;
        memset(&head, 0, sizeof(head));
        head.def = *def;
        head.pointCount = pointCount;
        head.tetCount = tetCount;
        head.expected = id;
        m3JournalPart parts[3] = {{&head, (int32_t)sizeof(head)},
                                  {points, pointCount * (int32_t)sizeof(m3Vec3)},
                                  {tets, 4 * tetCount * (int32_t)sizeof(uint16_t)}};
        m3JournalRecordParts(world, m3_opCreateSoftBodyTet, parts, 3);
    }
    return id;
}

m3SoftBodyId m3CreateSoftBody(m3WorldId worldId, const m3SoftBodyDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_SOFTBODY_COOKIE)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullSoftBodyId; // field checks live in the internal
    }
    int32_t slot = m3CreateSoftBodyInternal(world, def);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullSoftBodyId;
    }
    m3SoftBodyId id = {slot + 1, world->idWorld, world->softBodies.softPool.generations[slot]};
    if (world->recorder.journalActive != 0)
    {
        m3OpCreateSoftBody record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateSoftBody, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroySoftBody(m3SoftBodyId softId)
{
    m3World* world = m3WorldFromTag(softId.world);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale: the quiet destroy contract
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroySoftBody, &softId, (int32_t)sizeof(softId));
    }
    m3DestroySoftBodyInternal(world, slot);
}

bool m3SoftBody_IsValid(m3SoftBodyId softId)
{
    m3World* world = m3WorldFromTag(softId.world);
    return world != NULL && m3SoftBodySlot(world, softId) >= 0;
}

void m3SoftBody_PinParticle(m3SoftBodyId softId, int32_t particle)
{
    m3World* world = m3WorldFromTag(softId.world);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    if (slot < 0 || particle < 0 || particle >= world->softBodies.softParticleCount[slot])
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale or out of range: a documented no-op
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSoftBodyPin record;
        memset(&record, 0, sizeof(record));
        record.id = softId;
        record.particle = particle;
        m3JournalRecord(world, m3_opSoftBodyPin, &record, (int32_t)sizeof(record));
    }
    m3SoftBodyPinInternal(world, slot, particle);
}

void m3SoftBodyPinInternal(m3World* world, int32_t slot, int32_t particle)
{
    int32_t k = slot * M3_SOFTBODY_MAX_PARTICLES + particle;
    world->softBodies.softInvMass[k] = 0.0f;
    world->softBodies.softPrev[k] = world->softBodies.softPos[k];
}

void m3SoftBodyAnchorInternal(m3World* world, int32_t slot, int32_t particle, int32_t body)
{
    int32_t a = world->softBodies.softAnchorCount[slot];
    int32_t ak = slot * M3_SOFTBODY_MAX_ANCHORS + a;
    int32_t k = slot * M3_SOFTBODY_MAX_PARTICLES + particle;
    const m3Transform* bxf = &world->bodies.transforms[body];
    m3Vec3 rel = {(m3real)(world->softBodies.softPos[k].x - bxf->p.x),
                  (m3real)(world->softBodies.softPos[k].y - bxf->p.y),
                  (m3real)(world->softBodies.softPos[k].z - bxf->p.z)};
    world->softBodies.softAnchorParticle[ak] = particle;
    world->softBodies.softAnchorBody[ak] = body;
    world->softBodies.softAnchorGen[ak] = world->bodies.bodyPool.generations[body];
    world->softBodies.softAnchorLocal[ak] = m3InvRotateVec3(bxf->q, rel);
    world->softBodies.softAnchorCount[slot] = a + 1;
}

void m3SoftBodyAnchorSoftInternal(m3World* world, int32_t slotA, int32_t particleA, int32_t slotB,
                                  int32_t particleB)
{
    // The LOWER slot owns the pin: one canonical home per pair.
    if (slotB < slotA)
    {
        int32_t ts = slotA;
        slotA = slotB;
        slotB = ts;
        int32_t tp = particleA;
        particleA = particleB;
        particleB = tp;
    }
    int32_t a = world->softBodies.softSoftCount[slotA];
    int32_t ak = slotA * M3_SOFTBODY_MAX_ANCHORS + a;
    world->softBodies.softSoftParticleA[ak] = particleA;
    world->softBodies.softSoftSlotB[ak] = slotB;
    world->softBodies.softSoftGenB[ak] = world->softBodies.softPool.generations[slotB];
    world->softBodies.softSoftParticleB[ak] = particleB;
    world->softBodies.softSoftCount[slotA] = a + 1;
}

void m3SoftBody_AnchorToSoft(m3SoftBodyId softIdA, int32_t particleA, m3SoftBodyId softIdB,
                             int32_t particleB)
{
    m3World* world = m3WorldFromTag(softIdA.world);
    int32_t slotA = world != NULL ? m3SoftBodySlot(world, softIdA) : -1;
    int32_t slotB = world != NULL ? m3SoftBodySlot(world, softIdB) : -1;
    if (slotA < 0 || slotB < 0 || slotA == slotB || particleA < 0 || particleB < 0 ||
        particleA >= world->softBodies.softParticleCount[slotA] ||
        particleB >= world->softBodies.softParticleCount[slotB] ||
        world->softBodies.softSoftCount[slotA < slotB ? slotA : slotB] >= M3_SOFTBODY_MAX_ANCHORS)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale, self-pin, out of range, or full: quiet no-op
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSoftBodyAnchorSoft record;
        memset(&record, 0, sizeof(record));
        record.idA = softIdA;
        record.particleA = particleA;
        record.idB = softIdB;
        record.particleB = particleB;
        m3JournalRecord(world, m3_opSoftBodyAnchorSoft, &record, (int32_t)sizeof(record));
    }
    m3SoftBodyAnchorSoftInternal(world, slotA, particleA, slotB, particleB);
}

void m3SoftBody_AnchorParticle(m3SoftBodyId softId, int32_t particle, m3BodyId bodyId)
{
    m3World* world = m3WorldFromTag(softId.world);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    int32_t body = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (slot < 0 || body < 0 || particle < 0 ||
        particle >= world->softBodies.softParticleCount[slot] ||
        world->softBodies.softAnchorCount[slot] >= M3_SOFTBODY_MAX_ANCHORS)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale, out of range, or a full table: quiet no-op
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSoftBodyAnchor record;
        memset(&record, 0, sizeof(record));
        record.id = softId;
        record.particle = particle;
        record.body = bodyId;
        m3JournalRecord(world, m3_opSoftBodyAnchor, &record, (int32_t)sizeof(record));
    }
    m3SoftBodyAnchorInternal(world, slot, particle, body);
}

int32_t m3SoftBody_GetParticleCount(m3SoftBodyId softId)
{
    m3World* world = m3WorldFromTag(softId.world);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    return slot >= 0 ? world->softBodies.softParticleCount[slot] : 0;
}

m3Pos3 m3SoftBody_GetParticlePosition(m3SoftBodyId softId, int32_t particle)
{
    m3World* world = m3WorldFromTag(softId.world);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    if (slot < 0 || particle < 0 || particle >= world->softBodies.softParticleCount[slot])
    {
        m3Refuse(world, m3_errorInvalid);
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return world->softBodies.softPos[slot * M3_SOFTBODY_MAX_PARTICLES + particle];
}
