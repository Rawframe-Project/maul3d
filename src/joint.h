// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joints: internal declarations.

#ifndef MAUL3D_SRC_JOINT_H
#define MAUL3D_SRC_JOINT_H

#include "world_internal.h"

#include <math.h>

// What the joint setters and replay accept for a joint's drives
// (src/joint_params.c): a motor speed within M3_SPEED_LIMIT and an
// effort within M3_FORCE_LIMIT, except on a distance joint, whose motor
// fields carry its spring; a spring or steering frequency non-negative,
// positive when enabled, within M3_HERTZ_LIMIT and damping within
// M3_RATIO_LIMIT, on a joint type that drives; a limit range ordered while it is on, or on a
// motor joint, force and torque budgets within M3_FORCE_LIMIT; a motor joint's aim offset within
// M3_LOCAL_LIMIT.
m3Result m3JointMotorCheck(int32_t type, int32_t enable, float speed, float maxEffort);
// The types whose motor slot is a motor: the gear and the motor joint keep
// their own values there.
bool m3JointHasMotor(uint8_t type);
// The types that take limits. The motor joint keeps its force and torque
// budgets, and the distance joint its length range, in the limit slot and
// on the limit record, through their own calls.
bool m3JointHasLimits(uint8_t type);
m3Result m3JointLimitsCheck(int32_t type, int32_t enable, float lower, float upper);
// A distance joint's length range: finite, 0 <= minLength <= maxLength.
bool m3DistanceLengthsFit(float minLength, float maxLength);

// The reach of a motor joint's aim. Its create aims at the gap between
// the anchors in body A's frame. Each body's position is within
// M3_POSITION_LIMIT on every axis and each anchor within M3_LOCAL_LIMIT,
// so the gap is shorter than 2 sqrt(3) (M3_POSITION_LIMIT +
// M3_LOCAL_LIMIT), under 3.5e15. A body's rotation is held only to
// |q|^2 in (0.98, 1.02), and m3RotateVec3's expanded form stretches a
// vector perpendicular to the axis by up to about 4% at |q|^2 = 1.02, so
// each component of the aim in body A's frame stays under about 3.61e15.
// SetOffsets takes an aim within M3_LOCAL_LIMIT.
#define M3_SERVO_AIM_LIMIT 4.0e15f
bool m3JointSpringValid(const m3World* world, int32_t slot, int32_t enable, float hertz,
                        float zeta);
bool m3JointSteerValid(const m3World* world, int32_t slot, int32_t enable, float target,
                       float hertz, float zeta, float maxEffort);
bool m3MotorJointOffsetsValid(const m3World* world, int32_t slot, m3Vec3 linearOffset,
                              m3Quat angularOffset);
// The spring and steering checks for a joint of this type, as restore
// reads it.
bool m3JointSpringFits(uint8_t type, int32_t enable, float hertz, float zeta);
bool m3JointSteerFits(uint8_t type, int32_t enable, float target, float hertz, float zeta,
                      float maxEffort);

void m3JointSetLimitsInternal(m3World* world, int32_t j, int32_t enable, float lower, float upper);

void m3JointSetMotorInternal(m3World* world, int32_t j, int32_t enable, float speed, float effort);

void m3JointSetSteerInternal(m3World* world, int32_t j, int32_t enable, float target, float hertz,
                             float zeta, float maxEffort);

void m3MotorJointSetOffsetsInternal(m3World* world, int32_t j, m3Vec3 linearOffset,
                                    m3Quat angularOffset);

void m3JointEnableConnectedCollisionInternal(m3World* world, int32_t j, int32_t on);

void m3JointSetBreakInternal(m3World* world, int32_t j, float maxForce, float maxTorque);

// Reaction magnitudes from the stored warm rows, the break law's
// input and the readback's source (per-type assembly, documented
// on the public API).
void m3JointReactionMagnitudes(const m3World* world, int32_t j, m3real invH, m3real* outForce,
                               m3real* outTorque);

void m3JointSetSpringInternal(m3World* world, int32_t j, int32_t enable, float hertz, float zeta);

void m3JointSetTargetInternal(m3World* world, int32_t j, float scalar, m3Quat q);

// A drive target the target setters and replay accept for a joint of
// this type: an angle within [-pi, pi] on a revolute joint and a finite
// translation on a prismatic or wheel joint, each with the identity
// rotation, or a rotation within the unit-length band on a spherical
// joint, with a zero scalar.
bool m3JointTargetValid(uint8_t type, float scalar, m3Quat q);

// The gear's spin readback: a body's rotation seen in its
// own gear frame, twisted about that frame's z (the gear axis).
// Shared by the create bake and the solver's drift measurement so
// the two can never disagree.
static inline m3real m3GearSpin(m3Quat q, m3Quat frame)
{
    m3Quat conjF = {-frame.x, -frame.y, -frame.z, frame.w};
    m3Quat inFrame = m3MulQuat(conjF, m3MulQuat(q, frame));
    m3real twist =
        inFrame.w < 0.0f ? m3Atan2(-inFrame.z, -inFrame.w) : m3Atan2(inFrame.z, inFrame.w);
    return 2.0f * twist;
}

// Wrap to (-pi, pi]: spin differences cross the seam every half
// turn and the drift correction must chase the SHORT way. Constant time:
// past three half turns the whole turns come off at once, since one
// step a turn never ends once 2 pi is below the angle's spacing. Within
// three half turns it takes the one step it always did, so every angle a
// gear reaches keeps its bits. An angle too large to carry a phase lands
// on the seam.
static inline m3real m3WrapPi(m3real a)
{
    const m3real turn = 2.0f * M3_PI;
    if (a > 3.0f * M3_PI || a < -3.0f * M3_PI)
    {
        a -= turn * floorf((a + M3_PI) / turn);
    }
    if (a > M3_PI)
    {
        a -= turn;
    }
    else if (a < -M3_PI)
    {
        a += turn;
    }
    return a > M3_PI ? M3_PI : (a < -M3_PI ? -M3_PI : a);
}

int32_t m3JointSlot(const m3World* world, m3JointId jointId);

int32_t m3CreateJointInternal(m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB);

void m3DestroyJointInternal(m3World* world, int32_t index);

#endif // MAUL3D_SRC_JOINT_H
