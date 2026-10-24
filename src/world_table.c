// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The table of live worlds.

#include "world_table.h"

#include "misuse.h"

static m3World* s_worlds[M3_MAX_WORLDS];
static uint16_t s_worldGenerations[M3_MAX_WORLDS];

int32_t m3WorldTableFreeSlot(void)
{
    int32_t slot = -1;
    for (int32_t i = 0; i < M3_MAX_WORLDS && slot < 0; ++i)
    {
        slot = s_worlds[i] == NULL ? i : -1;
    }
    return slot;
}

uint16_t m3WorldTableGeneration(int32_t slot)
{
    return s_worldGenerations[slot];
}

void m3WorldTableSet(int32_t slot, m3World* world)
{
    s_worlds[slot] = world;
}

void m3WorldTableRelease(int32_t slot)
{
    s_worlds[slot] = NULL;
    s_worldGenerations[slot] += 1;
}

m3World* m3WorldFromId(m3WorldId worldId)
{
    int32_t index = worldId.index1 - 1;
    if (index < 0 || index >= M3_MAX_WORLDS || s_worlds[index] == NULL ||
        s_worldGenerations[index] != worldId.generation)
    {
        return NULL;
    }
    return s_worlds[index];
}

m3World* m3WorldFromTag(uint16_t tag)
{
    m3World* world = s_worlds[tag & ((1u << M3_WORLD_SLOT_BITS) - 1u)];
    if (world == NULL || world->idWorld != tag)
    {
        m3Refuse(NULL, m3_errorInvalid); // an id of a world that is gone
        return NULL;
    }
    return world;
}
