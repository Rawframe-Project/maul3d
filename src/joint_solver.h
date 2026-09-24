// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver: the per-step constraint every joint gets, the pose
// its rows are built from, and the kind table (one joint_<kind>.c per
// type).

#ifndef MAUL3D_SRC_JOINT_SOLVER_H
#define MAUL3D_SRC_JOINT_SOLVER_H

#include "joint_row.h"
#include "solver.h"
#include "world_internal.h"

// Joint flags (world->joints.jointFlags). The cone bit is the wheel's steer bit.
#define M3_JOINT_LIMIT  1u // limits enabled
#define M3_JOINT_MOTOR  2u // motor enabled
#define M3_JOINT_CONE   4u // spherical cone and twist limit enabled
#define M3_JOINT_STEER  4u // wheel: steering drive enabled
#define M3_JOINT_SPRING 8u // drive spring enabled

// A joint as the solver sees it for one step. The accumulated impulses
// live in the five stored slots, loaded and stored whole; what each slot
// component means is the kind's (see each joint_<kind>.c), and the
// reaction readback in joint_params.c reads the same map.
typedef struct m3JointConstraint
{
    int32_t joint; // world joint slot
    uint8_t type;
    uint8_t flags; // M3_JOINT_*
    int32_t bodyA;
    int32_t bodyB;
    m3Vec3 rA; // anchors from the centers of mass, world-rotated at prepare
    m3Vec3 rB;
    m3Vec3 deltaCenter; // center B minus center A at prepare
    m3real invMassA;
    m3real invMassB;
    m3Mat3 invIA; // world inverse inertia, fixed at prepare
    m3Mat3 invIB;
    m3Quat frameQA; // joint frames in the world at prepare
    m3Quat frameQB;
    m3Softness softness;   // the constraint rows
    m3Softness springSoft; // the drive spring
    m3Softness steerSoft;  // wheel: the steering drive
    m3real motorSpeed;
    m3real maxMotorEffort;
    m3real lowerLimit;
    m3real upperLimit;
    m3real coneAngle;
    m3real target;  // drive target (angle or translation); gear: the phase drift
    m3Quat targetQ; // rotation drive target
    m3Vec3 offset;  // motor: B's target position in A's frame
    m3Vec3 axis;    // distance: the prepare direction, kept for anchors that meet
    m3Vec3 ropeA;   // pulley: each anchor minus its ground point at prepare
    m3Vec3 ropeB;
    m3real ratio;      // gear, pulley
    m3real restLength; // distance, pulley
    m3real steerTarget;
    m3real steerBudget; // 0 = unbudgeted
    uint16_t genModes;  // generic: packed axis modes and the motor axis
    m3Vec3 genLinLower;
    m3Vec3 genLinUpper;
    m3Vec3 genAngLower;
    m3Vec3 genAngUpper;
    m3Vec3 impulse;
    m3Vec3 perpImpulse;
    m3Vec3 limitImpulse;
    m3Vec3 angularImpulse;
    m3Vec3 springImpulse;
} m3JointConstraint;

// What a kind's prepare receives beyond the common setup.
typedef struct m3JointFrame
{
    int32_t joint;
    m3real h;
    const m3Transform* xfA;
    const m3Transform* xfB;
    m3Vec3 rlcA; // world-rotated local centers of mass
    m3Vec3 rlcB;
} m3JointFrame;

// The joint where the substep has taken it.
typedef struct m3JointPose
{
    m3Vec3 armA; // arms turned by each body's rotation since the step began
    m3Vec3 armB;
    m3Vec3 gap;   // anchor B minus anchor A
    m3Vec3 moveA; // how far each center of mass moved since the step began
    m3Vec3 moveB;
    m3Quat turnA; // each body's rotation since the step began
    m3Quat turnB;
} m3JointPose;

// The joint frames now and B's frame relative to A's, taken in the
// hemisphere where its scalar part is not negative.
typedef struct m3JointFrames
{
    m3Quat a;
    m3Quat b;
    m3Quat rel;
} m3JointFrames;

// One solve pass: biased while solving, not while relaxing.
typedef struct m3JointPass
{
    bool biased;
    m3real h;
    m3real invH;
} m3JointPass;

// A kind's functions. The filter joint never enters the constraint
// array, so its entry is empty.
typedef struct m3JointKind
{
    void (*prepare)(m3World* world, m3JointConstraint* c, const m3JointFrame* f);
    void (*warmStart)(const m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b);
    void (*solve)(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                  const m3JointPass* pass);
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
                       const m3Vec3* deltaPos, const m3Quat* deltaRot);
void m3SolveJoints(m3World* world, m3JointConstraint* joints, int32_t count, const m3Vec3* deltaPos,
                   const m3Quat* deltaRot, m3real h, m3real invH, bool biased);
void m3StoreJointImpulses(m3World* world, m3JointConstraint* joints, int32_t count);

// Shared by the kinds.
m3JointFrames m3PoseFrames(const m3JointConstraint* c, const m3JointPose* pose);
m3Vec3 m3FrameAxis(m3Quat frame, int32_t k); // the frame's axis k in the world

// A row along a direction fixed in A: the lever reaches from A's center
// to B's anchor.
m3JointRow m3SlideRow(const m3JointPose* pose, m3Vec3 axis);

// The gradient, with respect to the relative spin wB - wA, of component
// k (x, y or z) of the relative rotation.
m3Vec3 m3RelativeGradient(const m3JointFrames* f, int32_t k);

// The rotation that takes B's frame to where the target puts it, as a
// world rotation vector: its time derivative is wB - wA.
m3Vec3 m3RotationError(const m3JointFrames* f, m3Quat target);

// Both limits of a row whose value is held in [lower, upper]. The upper
// limit runs the row backward, so both impulses are never negative.
void m3SolveLimits(const m3JointRow* row, m3real value, m3real lower, m3real upper, m3Softness soft,
                   m3real* lowerImpulse, m3real* upperImpulse, m3JointBodies* b,
                   const m3JointPass* pass);
void m3WarmStartLimits(const m3JointRow* row, m3real lowerImpulse, m3real upperImpulse,
                       m3JointBodies* b);

// A drive's rigid part and a vector error's bias for the 3x3 blocks.
m3Vec3 m3BlockBias(m3RowDrive unit, m3Vec3 C);

// The rotation vector of a unit quaternion (angle times axis).
m3Vec3 m3RotationVector(m3Quat q);

// The drive on one row: a spring toward the target and a motor toward
// the speed. With both on they share one effort budget, the spring
// spending what the motor's impulse leaves.
void m3SolveDrive(const m3JointRow* row, m3real value, const m3JointConstraint* c, m3JointBodies* b,
                  const m3JointPass* pass, m3real* spring, m3real* motor);

// The two rows that keep the frame z axes together: the gradients of
// the relative rotation's x and y parts, which vanish when aligned.
void m3SolveAlignment(const m3JointFrames* f, m3JointBodies* b, m3Softness soft, bool biased,
                      m3real* first, m3real* second);
void m3WarmStartAlignment(const m3JointFrames* f, m3JointBodies* b, m3real first, m3real second);

// The point block that pins the anchors (impulse).
void m3SolveJointPoint(m3JointConstraint* c, const m3JointPose* pose, m3JointBodies* b,
                       const m3JointPass* pass);

// A component of a vector by index.
m3real* m3Component(m3Vec3* v, int32_t k);

// The stiff softness a joint row gets.
m3Softness m3StiffJointSoftness(m3real h);

#endif // MAUL3D_SRC_JOINT_SOLVER_H
