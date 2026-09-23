// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bodies: internal declarations.

#ifndef MAUL3D_SRC_BODY_H
#define MAUL3D_SRC_BODY_H

#include "world_internal.h"

// Slot lookup with generation check: -1 for a stale or foreign id.
int32_t m3BodySlot(const m3World* world, m3BodyId bodyId);

void m3ApplyForceInternal(m3World* world, int32_t index, m3Vec3 force);

void m3SetTransformInternal(m3World* world, int32_t index, m3Transform pose);

void m3SetTargetTransformInternal(m3World* world, int32_t index, m3Transform pose);

void m3SetTypeInternal(m3World* world, int32_t index, uint8_t type);

void m3SetEnabledInternal(m3World* world, int32_t index, int enabled);

void m3SetMotionLocksInternal(m3World* world, int32_t index, uint8_t locks);

void m3SetSleepControlsInternal(m3World* world, int32_t index, float threshold, int canSleep);

void m3SetAwakeInternal(m3World* world, int32_t index, int awake);

void m3WakeRegionAabb(m3World* world, const double lo[3], const double hi[3]);

// Wakes a dynamic body; static and kinematic bodies never sleep.
static inline void m3WakeIfDynamic(m3World* world, int32_t body)
{
    if (world->bodies.types[body] == (uint8_t)m3_dynamicBody)
    {
        world->bodies.awake[body] = 1;
        world->bodies.sleepTimes[body] = 0.0f;
    }
}

// bodyLocks bit 6: this body bypasses the angular speed cap (the
// reference allowFastRotation escape hatch). Bits 0..5 stay the
// motion locks; the byte already snapshots and hashes off-default
// as one block, so the flag rides for free.
#define M3_LOCKS_ALLOW_FAST_ROTATION 0x40u

void m3SetAllowFastRotationInternal(m3World* world, int32_t index, int32_t allow);

void m3SetBodyNameInternal(m3World* world, int32_t index, const char* name);

#define M3_SLEEP_VELOCITY_DEFAULT 0.05f

void m3ApplyTorqueInternal(m3World* world, int32_t index, m3Vec3 torque);

void m3ApplyLinearImpulseInternal(m3World* world, int32_t index, m3Vec3 impulse);

void m3ApplyAngularImpulseInternal(m3World* world, int32_t index, m3Vec3 impulse);

void m3ApplyForceAtPointInternal(m3World* world, int32_t index, m3Vec3 force, m3Pos3 point);

void m3ApplyImpulseAtPointInternal(m3World* world, int32_t index, m3Vec3 impulse, m3Pos3 point);

// Internal mutation functions: the ONLY paths that change state. The
// public API validates, journals, then calls these; replay calls them
// directly, so a replayed world takes the identical code path.
int32_t m3CreateBodyInternal(m3World* world, const m3BodyDef* def);

void m3DestroyBodyInternal(m3World* world, int32_t index);

void m3SetLinearVelocityInternal(m3World* world, int32_t index, m3Vec3 velocity);

void m3SetAngularVelocityInternal(m3World* world, int32_t index, m3Vec3 velocity);

#endif // MAUL3D_SRC_BODY_H
