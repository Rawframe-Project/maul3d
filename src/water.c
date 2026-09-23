// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Water volumes: creation, destruction and the sleepers they wake. The
// buoyancy itself runs in the step.

#include "body.h"
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
    m3WaterVolumeId id = {slot + 1, world->worldIndex0, world->water.waterPool.generations[slot]};
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
    if (world == NULL || id.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->water.waterPool, index, id.generation))
    {
        return -1;
    }
    return index;
}

bool m3WaterVolume_IsValid(m3WaterVolumeId id)
{
    m3World* world = m3WorldFromIndex0(id.world0);
    return world != NULL && WaterSlot(world, id) >= 0;
}

void m3DestroyWaterVolume(m3WaterVolumeId id)
{
    m3World* world = m3WorldFromIndex0(id.world0);
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
