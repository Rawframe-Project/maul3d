// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Water volumes: creation, destruction, the sleepers they wake and the
// buoyancy field the step applies.

#include "water.h"
#include "body.h"
#include "broad_phase.h"
#include "journal.h"
#include "query.h"
#include "world.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

#define M3_WATER_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3WaterVolumeDef) << 8) ^ 7))

m3WaterVolumeDef m3DefaultWaterVolumeDef(void)
{
    m3WaterVolumeDef def;
    memset(&def, 0, sizeof(def));
    def.hi = (m3Pos3){1.0, 1.0, 1.0};
    def.density = 1000.0f;
    def.linearDrag = 2.0f;
    def.angularDrag = 1.0f;
    def.internalValue = M3_WATER_COOKIE;
    return def;
}

static bool WakeInBoxFn(int32_t shape, void* context)
{
    m3World* world = (m3World*)context;
    int32_t body = world->shapes.shapeBody[shape];
    if (world->bodies.types[body] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, body, 1);
    }
    return true;
}

static void WakeAroundWater(m3World* world, int32_t slot)
{
    // The tide moves things: sleepers touching the volume wake on
    // create AND destroy (without water under it, a sleeper falls).
    double lo[3] = {world->water.waterLo[slot].x, world->water.waterLo[slot].y,
                    world->water.waterLo[slot].z};
    double hi[3] = {world->water.waterHi[slot].x, world->water.waterHi[slot].y,
                    world->water.waterHi[slot].z};
    m3TreeQuery(&world->broadphase.tree, lo, hi, WakeInBoxFn, world);
}

int32_t m3CreateWaterVolumeInternal(m3World* world, const m3WaterVolumeDef* def)
{
    // The full wall, here because replay hands this function
    // raw journal bytes.
    if (!m3FinitePos3(def->lo) || !m3FinitePos3(def->hi) || !(def->hi.x > def->lo.x) ||
        !(def->hi.y > def->lo.y) || !(def->hi.z > def->lo.z) || !m3FiniteF(def->density) ||
        !(def->density > 0.0f) || !m3FiniteF(def->linearDrag) || def->linearDrag < 0.0f ||
        !m3FiniteF(def->angularDrag) || def->angularDrag < 0.0f || !m3FiniteV3(def->flow))
    {
        return -1;
    }
    int32_t slot = m3IdPoolAlloc(&world->water.waterPool);
    if (slot < 0)
    {
        return -1; // all 8 slots taken: loud at the caller
    }
    world->water.waterLo[slot] = def->lo;
    world->water.waterHi[slot] = def->hi;
    world->water.waterDensity[slot] = def->density;
    world->water.waterLinDrag[slot] = def->linearDrag;
    world->water.waterAngDrag[slot] = def->angularDrag;
    world->water.waterFlow[slot] = def->flow;
    WakeAroundWater(world, slot);
    return slot;
}

void m3DestroyWaterVolumeInternal(m3World* world, int32_t slot)
{
    WakeAroundWater(world, slot);
    world->water.waterLo[slot] = (m3Pos3){0.0, 0.0, 0.0};
    world->water.waterHi[slot] = (m3Pos3){0.0, 0.0, 0.0};
    world->water.waterDensity[slot] = 0.0f;
    world->water.waterLinDrag[slot] = 0.0f;
    world->water.waterAngDrag[slot] = 0.0f;
    world->water.waterFlow[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3IdPoolFree(&world->water.waterPool, slot);
}

m3WaterVolumeId m3CreateWaterVolume(m3WorldId worldId, const m3WaterVolumeDef* def)
{
    m3WaterVolumeId null = {0, 0, 0};
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_WATER_COOKIE)
    {
        m3Refuse(world, m3_errorInvalid);
        return null;
    }
    int32_t slot = m3CreateWaterVolumeInternal(world, def);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return null;
    }
    m3WaterVolumeId id = {slot + 1, world->idWorld, world->water.waterPool.generations[slot]};
    if (world->recorder.journalActive != 0)
    {
        m3OpCreateWaterVolume record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateWaterVolume, &record, (int32_t)sizeof(record));
    }
    return id;
}

static int32_t WaterSlot(const m3World* world, m3WaterVolumeId id)
{
    int32_t index = id.index1 - 1;
    if (world == NULL || id.world != world->idWorld ||
        !m3IdPoolValid(&world->water.waterPool, index, id.generation))
    {
        return -1;
    }
    return index;
}

bool m3WaterVolume_IsValid(m3WaterVolumeId id)
{
    m3World* world = m3WorldFromTag(id.world);
    return world != NULL && WaterSlot(world, id) >= 0;
}

void m3DestroyWaterVolume(m3WaterVolumeId id)
{
    m3World* world = m3WorldFromTag(id.world);
    int32_t slot = world != NULL ? WaterSlot(world, id) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyWaterVolume, &id, (int32_t)sizeof(id));
    }
    m3DestroyWaterVolumeInternal(world, slot);
}

// The part of one shape's box inside one water volume: buoyancy
// opposing gravity at the clipped box's centroid (a half-submerged
// crate rights itself, an off-center bite spins it), and the drag and
// flow weighted by the submerged fraction. Returns that fraction.
static float SubmergeShape(const m3World* world, int32_t k, const double slo[3],
                           const double shi[3], const double com[3], m3Buoyancy* b, int32_t m)
{
    const double wlo[3] = {world->water.waterLo[k].x, world->water.waterLo[k].y,
                           world->water.waterLo[k].z};
    const double whi[3] = {world->water.waterHi[k].x, world->water.waterHi[k].y,
                           world->water.waterHi[k].z};
    double clo[3];
    double chi[3];
    for (int32_t a = 0; a < 3; ++a)
    {
        clo[a] = slo[a] > wlo[a] ? slo[a] : wlo[a];
        chi[a] = shi[a] < whi[a] ? shi[a] : whi[a];
    }
    if (chi[0] <= clo[0] || chi[1] <= clo[1] || chi[2] <= clo[2])
    {
        return 0.0f;
    }
    double shapeVol = (shi[0] - slo[0]) * (shi[1] - slo[1]) * (shi[2] - slo[2]);
    double subVol = (chi[0] - clo[0]) * (chi[1] - clo[1]) * (chi[2] - clo[2]);
    float frac = (float)(subVol / shapeVol);
    frac = frac > 1.0f ? 1.0f : frac;
    m3Vec3 f = m3MulSV3(-(float)subVol * world->water.waterDensity[k], world->gravity);
    m3Vec3 r = {(float)(0.5 * (clo[0] + chi[0]) - com[0]),
                (float)(0.5 * (clo[1] + chi[1]) - com[1]),
                (float)(0.5 * (clo[2] + chi[2]) - com[2])};
    b->force[m] = m3Add3(b->force[m], f);
    b->torque[m] = m3Add3(b->torque[m], m3Cross3(r, f));
    b->flow[m] = m3Add3(b->flow[m], m3MulSV3(frac, world->water.waterFlow[k]));
    b->lin[m] += world->water.waterLinDrag[k] * frac;
    b->ang[m] += world->water.waterAngDrag[k] * frac;
    return frac;
}

// One mover's field: every shape against every live volume, the flow
// averaged over the submerged fractions.
static void SubmergeBody(const m3World* world, int32_t i, m3Buoyancy* b, int32_t m)
{
    b->force[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
    b->torque[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
    b->flow[m] = (m3Vec3){0.0f, 0.0f, 0.0f};
    b->lin[m] = 0.0f;
    b->ang[m] = 0.0f;
    if (world->bodies.types[i] != (uint8_t)m3_dynamicBody)
    {
        return;
    }
    m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[i].q, world->bodies.localCenters[i]);
    const double com[3] = {world->bodies.transforms[i].p.x + (double)rlc.x,
                           world->bodies.transforms[i].p.y + (double)rlc.y,
                           world->bodies.transforms[i].p.z + (double)rlc.z};
    float fracSum = 0.0f;
    for (int32_t shape = world->bodies.bodyShapeHead[i]; shape >= 0;
         shape = world->shapes.shapeNext[shape])
    {
        double slo[3];
        double shi[3];
        m3ShapeFatAabb(world, shape, slo, shi);
        if (!((shi[0] - slo[0]) * (shi[1] - slo[1]) * (shi[2] - slo[2]) > 0.0))
        {
            continue; // a plane's infinite box never swims
        }
        for (int32_t k = 0; k < world->water.waterPool.maxIndex; ++k)
        {
            if (world->water.waterPool.alive[k] != 0)
            {
                fracSum += SubmergeShape(world, k, slo, shi, com, b, m);
            }
        }
    }
    if (fracSum > 0.0f)
    {
        b->flow[m] = m3MulSV3(1.0f / fracSum, b->flow[m]);
    }
}

void m3PrepareBuoyancy(m3World* world, const int32_t* movers, int32_t moverCount, m3Buoyancy* b)
{
    memset(b, 0, sizeof(*b));
    for (int32_t k = 0; k < world->water.waterPool.maxIndex; ++k)
    {
        b->active += world->water.waterPool.alive[k];
    }
    if (b->active == 0 || moverCount == 0)
    {
        return;
    }
    b->force = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
    b->torque = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
    b->flow = (m3Vec3*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(m3Vec3));
    b->lin = (float*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(float));
    b->ang = (float*)m3StackAlloc(&world->scratch, moverCount * (int32_t)sizeof(float));
    if (b->force == NULL || b->torque == NULL || b->flow == NULL || b->lin == NULL ||
        b->ang == NULL)
    {
        b->active = 0; // a scratch stall: a dry step
        return;
    }
    for (int32_t m = 0; m < moverCount; ++m)
    {
        SubmergeBody(world, movers[m], b, m);
    }
}
