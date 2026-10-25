// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World lifecycle, tuning knobs and events: internal declarations.

#ifndef MAUL3D_SRC_WORLD_H
#define MAUL3D_SRC_WORLD_H

#include "world_internal.h"

// Tuning defaults, shared by the def,
// the solver reads, and the off-default hash folds.
#define M3_CONTACT_HERTZ_DEFAULT 30.0f

#define M3_CONTACT_DAMPING_RATIO_DEFAULT 10.0f

#define M3_CONTACT_PUSH_MAX_SPEED_DEFAULT 3.0f

#define M3_RESTITUTION_THRESHOLD_DEFAULT 1.0f

#define M3_MAX_LINEAR_SPEED_DEFAULT 400.0f

// The angular twin keeps the linear cap's philosophy: a
// catastrophe guard far above legal tumbling, not a tight clamp
// derived from the step. Hosts wanting a tight clamp set a low cap and
// flag their wheels.
#define M3_MAX_ANGULAR_SPEED_DEFAULT 800.0f

#define M3_HIT_EVENT_THRESHOLD_DEFAULT 1.0f

void m3SetHitEventThresholdInternal(m3World* world, float value);

// Joint breaks emit through this; capacity jointCapacity, cannot overflow.
void m3AppendJointBreakEvent(m3World* world, m3JointId joint);

// Empties every per-step event stream.
void m3ResetStepEvents(m3World* world);

// The step's contact begin and end events: a merge walk of the old and
// new pair lists, both sorted, so events come out in pair order.
void m3EmitContactEvents(m3World* world, const uint64_t* oldKeys, const m3Manifold* oldManifolds,
                         int32_t oldCount);

// One move event per mover, in the movers' ascending order.
void m3EmitMoveEvents(m3World* world, const int32_t* movers, int32_t moverCount);

// What every door of each world setting accepts: the public setters, the
// world def, replay and restore all ask these.
bool m3GravityValid(m3Vec3 gravity);
bool m3ContactTuningValid(float hertz, float dampingRatio, float pushSpeed);
// A world speed cap: positive and within M3_SPEED_LIMIT.
bool m3SpeedCapValid(float value);
bool m3WindValid(m3Vec3 direction, float speed, float gustHertz, float gustScale);
bool m3StepValid(float dt, int32_t substeps);

void m3SetGravityInternal(m3World* world, m3Vec3 gravity);

// False, with the tree untouched, when the allocator refuses the scratch.
bool m3RebuildBroadphaseInternal(m3World* world);

void m3SetContactTuningInternal(m3World* world, float hertz, float dampingRatio, float pushSpeed);

void m3SetRestitutionThresholdInternal(m3World* world, float value);

void m3SetMaximumLinearSpeedInternal(m3World* world, float value);

void m3SetMaximumAngularSpeedInternal(m3World* world, float value);

void m3EnableSleepingInternal(m3World* world, int32_t on);

void m3EnableContinuousInternal(m3World* world, int32_t on);

void m3SetWindInternal(m3World* world, m3Vec3 dir, float speed, float gustHertz, float gustScale);

#endif // MAUL3D_SRC_WORLD_H
