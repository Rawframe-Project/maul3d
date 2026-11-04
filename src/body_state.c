// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body lookups and wake state.

#include "body_state.h"

#include "broad_phase.h"
#include "misuse.h"

int32_t m3BodySlot(const m3World* world, m3BodyId bodyId)
{
    int32_t index = bodyId.index1 - 1;
    if (world == NULL || !m3IdPoolValid(&world->bodies.bodyPool, index, bodyId.generation))
    {
        return -1;
    }
    return index;
}

bool m3ResolveBody(const m3World* world, m3BodyId body, int32_t* indexOut)
{
    int32_t index = m3BodySlot(world, body);
    *indexOut = index;
    if (index < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    return true;
}

// The step clears the force and torque of every body it moves. A body
// that stops moving before the step drops them here, so a force never
// waits for a later step.
void m3DropForcesInternal(m3World* world, int32_t index)
{
    world->bodies.bodyForce[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodies.bodyTorque[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
}

void m3SetAwakeInternal(m3World* world, int32_t index, int awake)
{
    if (awake)
    {
        world->bodies.awake[index] = 1;
        world->bodies.sleepTimes[index] = 0.0f;
    }
    else
    {
        world->bodies.awake[index] = 0;
        world->bodies.linearVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->bodies.angularVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
        m3DropForcesInternal(world, index); // a sleeping body is not moved
        // Every freeze, API-forced ones included, records its
        // cold pairs or the frozen buffer stops being a pure
        // function of the sleeping state.
        m3FreezeDiscoverPairs(world, index);
    }
}

void m3WakeRegionAabb(m3World* world, const double lo[3], const double hi[3])
{
    // Wake every sleeping dynamic body whose own extent box
    // overlaps the region (the voxel-edit wake idea without the
    // tree: position plus max extent is a fair, deterministic
    // over-approximation). Serial, ascending slots: canonical.
    int32_t maxBody = world->bodies.bodyPool.maxIndex;
    for (int32_t b = 0; b < maxBody; ++b)
    {
        if (world->bodies.bodyPool.alive[b] == 0 ||
            world->bodies.types[b] != (uint8_t)m3_dynamicBody || world->bodies.awake[b] != 0)
        {
            continue;
        }
        double r = (double)world->bodies.maxExtents[b] + (double)M3_AABB_MARGIN;
        const m3Pos3* pp = &world->bodies.transforms[b].p;
        if (pp->x - r <= hi[0] && pp->x + r >= lo[0] && pp->y - r <= hi[1] && pp->y + r >= lo[1] &&
            pp->z - r <= hi[2] && pp->z + r >= lo[2])
        {
            world->bodies.awake[b] = 1;
            world->bodies.sleepTimes[b] = 0.0f;
        }
    }
}

// The lead byte's range of second bytes and its count of bytes in all,
// after Unicode's table of well-formed UTF-8 byte sequences: overlong
// forms, surrogates and code points past U+10FFFF are refused.
static int32_t Utf8Lead(uint8_t lead, uint8_t* lowOut, uint8_t* highOut)
{
    *lowOut = 0x80u;
    *highOut = 0xBFu;
    if (lead >= 0xC2u && lead <= 0xDFu)
    {
        return 2;
    }
    if (lead >= 0xE0u && lead <= 0xEFu)
    {
        *lowOut = lead == 0xE0u ? 0xA0u : 0x80u;
        *highOut = lead == 0xEDu ? 0x9Fu : 0xBFu;
        return 3;
    }
    if (lead >= 0xF0u && lead <= 0xF4u)
    {
        *lowOut = lead == 0xF0u ? 0x90u : 0x80u;
        *highOut = lead == 0xF4u ? 0x8Fu : 0xBFu;
        return 4;
    }
    return 0;
}

bool m3BodyNameValid(const uint8_t* text, int32_t byteCount)
{
    int32_t at = 0;
    while (at < byteCount)
    {
        uint8_t lead = text[at];
        if (lead == 0)
        {
            return false;
        }
        if (lead < 0x80u)
        {
            at += 1;
            continue;
        }
        uint8_t low;
        uint8_t high;
        int32_t length = Utf8Lead(lead, &low, &high);
        if (length == 0 || length > byteCount - at || text[at + 1] < low || text[at + 1] > high)
        {
            return false;
        }
        for (int32_t k = 2; k < length; ++k)
        {
            if (text[at + k] < 0x80u || text[at + k] > 0xBFu)
            {
                return false;
            }
        }
        at += length;
    }
    return true;
}

bool m3BodyNameSlotValid(const uint8_t* slot, int32_t* lengthOut)
{
    int32_t length = 0;
    while (length < M3_BODY_NAME_CAPACITY && slot[length] != 0)
    {
        length += 1;
    }
    *lengthOut = length;
    for (int32_t i = length; i < M3_BODY_NAME_CAPACITY; ++i)
    {
        if (slot[i] != 0)
        {
            return false;
        }
    }
    return length < M3_BODY_NAME_CAPACITY && m3BodyNameValid(slot, length);
}
