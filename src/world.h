// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World lifecycle, tuning knobs and events: internal declarations.

#ifndef MAUL3D_SRC_WORLD_H
#define MAUL3D_SRC_WORLD_H

#include "world_internal.h"

// Registry lookup: NULL for a stale or null id.
m3World* m3WorldFromId(m3WorldId worldId);

// Registry lookup by slot only (no generation check): body and shape
// ids carry their own generation, so their world reference resolves by
// slot, the Maul2D FromIndex0 pattern.
m3World* m3WorldFromIndex0(uint16_t index0);

// Tuning defaults: the reference values, shared by the def,
// the solver reads, and the off-default hash folds.
#define M3_CONTACT_HERTZ_DEFAULT 30.0f

#define M3_CONTACT_DAMPING_RATIO_DEFAULT 10.0f

#define M3_CONTACT_PUSH_MAX_SPEED_DEFAULT 3.0f

#define M3_RESTITUTION_THRESHOLD_DEFAULT 1.0f

#define M3_MAX_LINEAR_SPEED_DEFAULT 400.0f

// The angular twin keeps the linear cap's philosophy: a
// catastrophe guard far above legal tumbling, not the reference's
// aggressive dt-derived clamp. Hosts wanting the tight reference
// behavior set a low cap and flag their wheels.
#define M3_MAX_ANGULAR_SPEED_DEFAULT 800.0f

#define M3_HIT_EVENT_THRESHOLD_DEFAULT 1.0f

void m3SetHitEventThresholdInternal(m3World* world, float value);

// 8-6 emits through this; capacity jointCapacity, cannot overflow.
void m3AppendJointBreakEvent(m3World* world, m3JointId joint);

void m3SetGravityInternal(m3World* world, m3Vec3 gravity);

void m3RebuildBroadphaseInternal(m3World* world);

int32_t m3CreateWaterVolumeInternal(m3World* world, const m3WaterVolumeDef* def);

void m3DestroyWaterVolumeInternal(m3World* world, int32_t slot);

void m3SetContactTuningInternal(m3World* world, float hertz, float dampingRatio, float pushSpeed);

void m3SetRestitutionThresholdInternal(m3World* world, float value);

void m3SetMaximumLinearSpeedInternal(m3World* world, float value);

void m3SetMaximumAngularSpeedInternal(m3World* world, float value);

void m3EnableSleepingInternal(m3World* world, int32_t on);

void m3EnableContinuousInternal(m3World* world, int32_t on);

void m3SetWindInternal(m3World* world, m3Vec3 dir, float speed, float gustHertz, float gustScale);

#endif // MAUL3D_SRC_WORLD_H
