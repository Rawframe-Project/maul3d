// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver's shape: the per-step constraint, one kind table
// entry per joint type, what each stage hands a kind, and the helpers
// the kinds share. Each kind lives in its own joint_<kind>.c.

#ifndef MAUL3D_SRC_JOINT_SOLVER_H
#define MAUL3D_SRC_JOINT_SOLVER_H

#include "solver.h"
#include "world_internal.h"

// Joint flags (world->joints.jointFlags). The cone bit is the wheel's steer bit.
#define M3_JOINT_LIMIT  1u // limits enabled
#define M3_JOINT_MOTOR  2u // motor enabled
#define M3_JOINT_CONE   4u // spherical cone and twist limit enabled
#define M3_JOINT_STEER  4u // wheel: steering drive enabled
#define M3_JOINT_SPRING 8u // drive spring enabled

typedef struct m3JointConstraint
{
    int32_t joint; // world joint slot (impulses written back at store)
    uint8_t type;
    uint8_t flags; // M3_JOINT_*
    int32_t bodyA;
    int32_t bodyB;
    m3Vec3 rA; // COM-relative anchors, world-rotated at prepare
    m3Vec3 rB;
    m3Vec3 deltaCenter; // cB - cA at prepare
    m3real invMassA;
    m3real invMassB;
    m3Mat3 invIA;
    m3Mat3 invIB;
    m3Softness softness;
    m3Vec3 impulse; // linear point impulse
    // Revolute state (prepared): world joint frames, hinge axis,
    // axial mass, cached perp axes for warm start, extra impulses.
    m3Quat frameQA;
    m3Quat frameQB;
    m3Vec3 rotationAxis;
    m3real axialMass;
    m3Vec3 perpAxisX;
    m3Vec3 perpAxisY;
    m3real perpImpulseX;
    m3real perpImpulseY;
    m3real motorImpulse;
    m3real lowerImpulse;
    m3real upperImpulse;
    m3real motorSpeed;
    m3real maxMotorEffort;
    m3real lowerLimit;
    m3real upperLimit;
    m3Vec3 angularImpulse; // prismatic rotation lock (3 DOF)
    m3Softness springSoft; // distance spring; drive spring
    m3Softness steerSoft;  // wheel strut drive
    m3real steerTarget;    // radians about the strut
    m3real steerBudget;    // 0 = unbudgeted
    m3real restLength;     // distance rest (the upper limit)
    m3real targetScalar;   // drive target: angle or translation
    m3Quat targetQ;        // spherical drive target
    m3Vec3 springImpulseV; // x = scalar rows; xyz = spherical row
    m3Mat3 springK;        // iA + iB, the spherical drive mass
    uint16_t genModes;     // generic 6-DOF packed modes
    m3Vec3 genLinLower;    // generic per-axis limits
    m3Vec3 genLinUpper;
    m3Vec3 genAngLower;
    m3Vec3 genAngUpper;
    // Spherical cone and twist (prepared, the reference layout):
    m3Vec3 swingAxis;
    m3Vec3 twistJacobian; // the flagged row, FD-verified in the tests
    m3real swingMass;
    m3real twistMass;
    m3real coneAngle;
    m3real swingImpulse;
} m3JointConstraint;

// What a kind's prepare receives beyond the common setup already in the
// constraint (bodies, anchors, masses, softness, warm impulses).
typedef struct m3JointFrame
{
    int32_t joint;
    m3real h;
    const m3Transform* xfA;
    const m3Transform* xfB;
    m3Vec3 rlcA; // world-rotated local centers of mass
    m3Vec3 rlcB;
} m3JointFrame;

// Warm start: a kind adds its extra linear and angular impulse; the
// shared tail applies them with the point impulse.
typedef struct m3JointWarmContext
{
    m3Vec3 rA;
    m3Vec3 rB;
    const m3Quat* deltaRot;
    m3Vec3 linearExtra;
    m3Vec3 angularImpulse;
} m3JointWarmContext;

// Solve: the substep's anchors and both bodies' velocities.
typedef struct m3JointSolveContext
{
    m3Vec3 rA;
    m3Vec3 rB;
    m3Vec3 vA;
    m3Vec3 wA;
    m3Vec3 vB;
    m3Vec3 wB;
    const m3Vec3* deltaPos;
    const m3Quat* deltaRot;
    m3real hSub;
    m3real invHSub;
    int useBias;
} m3JointSolveContext;

// A kind's functions. The filter joint never enters the constraint
// array, so its entry is empty.
typedef struct m3JointKind
{
    void (*prepare)(m3World* world, m3JointConstraint* c, const m3JointFrame* f);
    void (*warmStart)(const m3World* world, const m3JointConstraint* c, m3JointWarmContext* w);
    void (*solve)(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s);
} m3JointKind;

extern const m3JointKind m3_sphericalJointKind;
extern const m3JointKind m3_revoluteJointKind;
extern const m3JointKind m3_prismaticJointKind;
extern const m3JointKind m3_fixedJointKind;
extern const m3JointKind m3_distanceJointKind;
extern const m3JointKind m3_genericJointKind;
extern const m3JointKind m3_wheelJointKind;
extern const m3JointKind m3_parallelJointKind;
extern const m3JointKind m3_motorJointKind;
extern const m3JointKind m3_gearJointKind;
extern const m3JointKind m3_pulleyJointKind;

// The step's joint stages, in the order the solver runs them.
int32_t m3PrepareJoints(m3World* world, m3JointConstraint* joints, m3real h);
void m3WarmStartJoints(m3World* world, m3JointConstraint* joints, int32_t count,
                       const m3Quat* deltaRot);
void m3SolveJoints(m3World* world, m3JointConstraint* joints, int32_t count, const m3Vec3* deltaPos,
                   const m3Quat* deltaRot, m3real hSub, m3real invHSub, int useBias);
void m3StoreJointImpulses(m3World* world, m3JointConstraint* joints, int32_t count);

// Shared by the kinds.
void m3PrepareHingeFrame(m3World* world, m3JointConstraint* c, const m3JointFrame* f);
void m3WarmStartHinge(const m3World* world, const m3JointConstraint* c, m3JointWarmContext* w);
void m3WarmStartAngularLock(const m3World* world, const m3JointConstraint* c,
                            m3JointWarmContext* w);
void m3SolveJointPoint(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s,
                       m3Vec3 vA, m3Vec3 wA, m3Vec3 vB, m3Vec3 wB);
m3Vec3 m3JointDeltaQuatToRotation(m3Quat q, m3Quat target);
m3real m3JointTwistAngle(m3Quat q);
m3real m3JointSwingAngle(m3Quat q);
m3Vec3 m3JointQuatToRotationVec(m3Quat relQ);
m3Vec3 m3JointPerpColumn(m3Quat qA, m3Quat relQ, m3Vec3 axis);

#endif // MAUL3D_SRC_JOINT_SOLVER_H
