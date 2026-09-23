// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joints: internal declarations.

#ifndef MAUL3D_SRC_JOINT_H
#define MAUL3D_SRC_JOINT_H

#include "world_internal.h"

void m3JointSetLimitsInternal(m3World* world, int32_t j, int32_t enable, float lower, float upper);

void m3JointSetMotorInternal(m3World* world, int32_t j, int32_t enable, float speed, float effort);

void m3JointSetSteerInternal(m3World* world, int32_t j, int32_t enable, float target, float hertz,
                             float zeta, float maxEffort);

void m3JointSetMotorPoseInternal(m3World* world, int32_t j, m3Vec3 offset, m3Quat rotation);

void m3JointSetCollideInternal(m3World* world, int32_t j, int32_t on);

void m3JointSetBreakInternal(m3World* world, int32_t j, float maxForce, float maxTorque);

// Reaction magnitudes from the stored warm rows, the break law's
// input and the readback's source (per-type assembly, documented
// on the public API).
void m3JointReactionMagnitudes(const m3World* world, int32_t j, m3real invH, m3real* outForce,
                               m3real* outTorque);

void m3JointSetSpringInternal(m3World* world, int32_t j, int32_t enable, float hertz, float zeta);

void m3JointSetTargetInternal(m3World* world, int32_t j, float scalar, m3Quat q);

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
// turn and the drift correction must chase the SHORT way.
static inline m3real m3WrapPi(m3real a)
{
    while (a > M3_PI)
    {
        a -= 2.0f * M3_PI;
    }
    while (a < -M3_PI)
    {
        a += 2.0f * M3_PI;
    }
    return a;
}

int32_t m3JointSlot(const m3World* world, m3JointId jointId);

int32_t m3CreateJointInternal(m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB);

void m3DestroyJointInternal(m3World* world, int32_t index);

#endif // MAUL3D_SRC_JOINT_H
