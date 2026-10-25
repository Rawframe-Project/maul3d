// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bodies: internal declarations.

#ifndef MAUL3D_SRC_BODY_H
#define MAUL3D_SRC_BODY_H

#include "world_internal.h"

// What a force, torque or impulse may add, and the velocities a body of a
// type may hold (src/body_motion.c): the public calls and replay refuse
// what these refuse, before anything changes.
bool m3ForceFits(const m3World* world, int32_t index, m3Vec3 force);
bool m3TorqueFits(const m3World* world, int32_t index, m3Vec3 torque);
bool m3ForceAtPointFits(const m3World* world, int32_t index, m3Vec3 force, m3Pos3 point);
bool m3LinearImpulseFits(const m3World* world, int32_t index, m3Vec3 impulse);
bool m3AngularImpulseFits(const m3World* world, int32_t index, m3Vec3 impulse);
bool m3ImpulseAtPointFits(const m3World* world, int32_t index, m3Vec3 impulse, m3Pos3 point);
bool m3BodyVelocityValid(uint8_t type, m3Vec3 velocity);
bool m3BodySpinValid(m3Vec3 velocity);

// The checks the body setters and replay share (src/body.c): a pose
// within M3_POSITION_LIMIT with a near-unit rotation, a servo target within reach of a kinematic
// body, a type change that keeps the velocities within the new type's limit, and a gravity scale or
// damping in range (an m3BodyParam).
bool m3PoseValid(m3Transform pose);
bool m3TargetValid(const m3World* world, int32_t index, m3Transform pose);
// The same for a body of this type at now, as restore reads it.
bool m3TargetFits(uint8_t type, m3Pos3 now, m3Transform pose);
bool m3TypeChangeValid(const m3World* world, int32_t index, int32_t type);
bool m3BodyParamValid(int32_t param, float value);
// A sleep threshold, m/s: finite and not negative.
bool m3SleepThresholdValid(float threshold);

void m3ApplyForceInternal(m3World* world, int32_t index, m3Vec3 force);

void m3SetTransformInternal(m3World* world, int32_t index, m3Transform pose);

void m3SetTargetTransformInternal(m3World* world, int32_t index, m3Transform pose);

void m3SetTypeInternal(m3World* world, int32_t index, uint8_t type);

void m3SetEnabledInternal(m3World* world, int32_t index, int enabled);

void m3SetMotionLocksInternal(m3World* world, int32_t index, uint8_t locks);

void m3SetSleepControlsInternal(m3World* world, int32_t index, float threshold, int canSleep);

// The six lock flags as bits 0..5: linear x, y, z, then angular x, y, z.
uint8_t m3LockBits(m3MotionLocks locks);

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

// Gravity scale and damping (m3BodyParam), the bullet flag and user
// data: the paths the setters and replay share.
void m3SetBodyParamInternal(m3World* world, int32_t index, int32_t param, float value);
void m3SetBulletInternal(m3World* world, int32_t index, int bullet);

#endif // MAUL3D_SRC_BODY_H
