// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joints: the seven joint types, their defs, limits, motors,
// springs, reaction force readback and breaking.

#ifndef MAUL3D_JOINT_H
#define MAUL3D_JOINT_H

#include "body.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum m3JointType
    {
        m3_sphericalJoint = 0, // ball: pins two body-frame points together
        m3_revoluteJoint = 1,  // hinge: one rotation axis, limits, motor
        m3_prismaticJoint = 2, // slider: one translation axis, limits, motor
        m3_weldJoint = 3,      // full 6-DOF lock at the create pose
        m3_distanceJoint = 4,  // rope/rod: anchor distance in [minLength, maxLength]
        m3_genericJoint = 5,   // 6-DOF: per-axis lock/free/limit + one motor
        m3_wheelJoint = 6,
        /// FILTER: no constraint rows at all; the joint's
        /// whole effect is the connected-pair collision filter
        /// (create with enableConnectedCollision false). Ragdoll sibling
        /// limbs stop grinding without buying a single solver row.
        m3_filterJoint = 7,
        /// PARALLEL: keeps localAxisA on body A parallel to
        /// localAxisB on body B (two angular rows); every
        /// translation and the twist about the shared axis stay
        /// free. Platform linkages without the full revolute.
        m3_parallelJoint = 8,
        /// MOTOR: a servo weld. Drives body B toward a target
        /// pose relative to body A's create-time joint frame: a soft
        /// 3-DOF rotation drive plus a soft 3-DOF translation drive,
        /// each budget-capped. Tune with the joint spring (the
        /// shared stiffness), aim with m3MotorJoint_SetOffsets and
        /// budget with maxForce and maxTorque (0 = uncapped).
        /// Without a spring the servo idles completely free. A
        /// fresh servo aims at its own create pose.
        m3_motorJoint = 9,
        /// GEAR: couples spin so that spinA + ratio * spinB
        /// keeps its create value, where spinX is body X's rotation
        /// about its own axis localAxisX. Positive ratio is the
        /// external mesh (counter-rotation: ratio 2 turns B half as
        /// fast, backwards); negative couples same-direction (belt
        /// or internal ring). MOUNTING CONTRACT (documented): both
        /// bodies must be hinged on a common rigid frame (revolute
        /// or wheel joints to the same chassis); the gear adds no
        /// reaction on that frame and holds no other degree of
        /// freedom. Angle drift is corrected softly, so the mesh
        /// cannot creep under load.
        m3_gearJoint = 10,
        /// PULLEY: a rope from localAnchorA up over the
        /// WORLD point groundAnchorA, across to groundAnchorB and
        /// down to localAnchorB: length1 + ratio * length2 keeps
        /// its create value (ratio > 0). The rope is RIGID both
        /// ways (it can push; pair it with a distance rope when
        /// slack matters). Block and tackle: ratio 2 lifts B twice
        /// as slow with twice the force.
        m3_pulleyJoint = 11,
    } m3JointType;

    /// Per-axis behavior of the generic joint, in the joint frame
    /// built from localAxisA/localAxisB (the frame's z axis).
    typedef enum m3AxisMode
    {
        m3_axisLocked = 0,
        m3_axisFree = 1,
        m3_axisLimited = 2,
    } m3AxisMode;

    /// Build with m3DefaultJointDef; hand-rolled defs are rejected
    /// loudly. Anchors are body-frame points on each body.
    typedef struct m3JointDef
    {
        int32_t type; // m3JointType
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        // Anchors in each body's frame, each component within 1e4 m.
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;
        /// Hinge axis per body frame (unit; revolute only). The two
        /// axes must map to the same world direction at create time
        /// for a well-posed hinge.
        m3Vec3 localAxisA;
        m3Vec3 localAxisB;
        /// Limits and motor share names across joint types: for the
        /// revolute they are angles (radians) and a torque cap, for
        /// the prismatic translations (meters) and a force cap.
        bool enableLimit;
        float lowerLimit;
        float upperLimit;
        bool enableMotor;
        float motorSpeed;     // rad/s or m/s by joint type, within 1e5
        float maxMotorEffort; // non-negative, at most 1e20
        /// Spherical only: the swing cone (the shoulder). Every other
        /// type refuses enableCone. enableLimit doubles as the twist
        /// range for the spherical, using lowerLimit and upperLimit as
        /// twist angles.
        bool enableCone;
        float coneAngle; // radians from the A frame's z-axis
        /// Distance joints hold the anchor distance in [minLength,
        /// maxLength] below and take neither a limit nor a motor:
        /// enableLimit and enableMotor refuse them. Their optional spring
        /// pulls toward length, below. Weld joints ignore axes
        /// and limits entirely: they weld the create-time pose.
        /// Jointed bodies do not collide with each other unless this
        /// is set (the classic chain-fight guard).
        bool enableConnectedCollision;
        /// The generic joint. Modes are m3AxisMode per joint
        /// frame axis; limits are meters (linear) and radians
        /// (angular), used only where the mode is limited. One
        /// motor: genericMotorAxis picks 0..2 (linear) or 3..5
        /// (angular), 255 for none; it drives motorSpeed with the
        /// maxMotorEffort cap on a free or limited axis. At most ONE
        /// angular axis may be limited, and the other two must be
        /// BOTH locked (a hinge with range) or BOTH free (a
        /// twist-limited ball); anything else refuses loudly. The
        /// def's cookie carries its size, so a caller compiled
        /// against another layout is refused instead of misread.
        /// The wheel joint, the OPTIONAL rigid-wheel path for
        /// vehicles (the raycast vehicle stays the default): the
        /// wheel body B slides along a suspension axis fixed in the
        /// chassis A and spins freely about its axle. localAxisA is
        /// the SUSPENSION axis (chassis frame, usually down);
        /// localAxisB is the AXLE (wheel frame, its spin axis). The
        /// axle is captured at create, snapped exactly perpendicular
        /// to the suspension axis; more than a small skew (|dot| >
        /// 0.1) refuses loudly. enableLimit bounds the suspension
        /// translation in meters; enableMotor drives the SPIN
        /// (rad/s, torque cap): the drive axle. The suspension
        /// spring is the joint drive: m3Joint_EnableSpring plus
        /// m3Joint_SetTargetTranslation. m3Joint_GetAngle reads the
        /// spin angle, m3Joint_GetTranslation the suspension travel.
        /// Breakage applies unchanged: a capped axle snaps
        /// deterministically and the wheel body rolls away.
        uint8_t genericLinear[3];
        uint8_t genericAngular[3];
        uint8_t genericMotorAxis;
        float genericLinearLower[3];
        float genericLinearUpper[3];
        float genericAngularLower[3];
        float genericAngularUpper[3];
        /// Gear and pulley: the coupling ratio (gear: any
        /// nonzero within 1e4, sign picks the mesh; pulley: > 0 and at
        /// most 1e4) and the pulley's two fixed WORLD anchor points,
        /// within 1e15 m.
        float ratio;
        m3Pos3 groundAnchorA;
        m3Pos3 groundAnchorB;
        /// The motor joint's force and torque budgets, each in
        /// [0, 1e20], 0 for no cap. Other types ignore them.
        float maxForce;
        float maxTorque;
        /// The distance joint's range, 0 <= minLength <= maxLength meters:
        /// a rod is minLength == maxLength, a rope a range. Other types
        /// ignore them.
        float minLength;
        float maxLength;
        /// The distance joint's spring rest length in meters, finite and
        /// not negative. It need not lie within the range: the range holds
        /// whatever the spring pulls toward. Other types ignore it.
        float length;
        /// The spring, on the types that take one (revolute, prismatic,
        /// spherical, wheel, motor and distance joints): springHertz in
        /// [0, 1e5] and positive when enableSpring is set, the damping
        /// ratio in [0, 1e4]. Every other type refuses enableSpring and
        /// ignores the values.
        bool enableSpring;
        float springHertz;
        float springDampingRatio;
        int32_t internalValue;
    } m3JointDef;

    /// The pinned joint defaults: spherical type, zero anchors, unit
    /// z axes, everything disabled, enableConnectedCollision off, and the
    /// def cookie every create demands.
    M3_API m3JointDef m3DefaultJointDef(void);

    /// Create a joint between two distinct bodies of the same world
    /// (at least one dynamic). Returns m3_errorInvalid for a bad def,
    /// m3_errorLimit for a value past a bound in maul3d/base.h or for a
    /// spherical, revolute, prismatic, weld, wheel or motor joint whose
    /// bodies' lever ratio would pass M3_JOINT_LEVER_LIMIT,
    /// m3_errorStale for a stale body and m3_errorCapacity for an
    /// exhausted pool; *jointOut is the null id on any failure.
    /// Journaled; replay verifies the minted id. Destroying either body
    /// destroys the joint.
    M3_NODISCARD M3_API m3Result m3CreateJoint(m3World* world, const m3JointDef* def,
                                               m3JointId* jointOut);
    M3_NODISCARD M3_API m3Result m3DestroyJoint(m3World* world, m3JointId joint);
    M3_API bool m3Joint_IsValid(const m3World* world, m3JointId joint);
    /// Readback, as in Maul2D. Thread class: reader.
    M3_API m3JointType m3Joint_GetType(const m3World* world, m3JointId joint);
    M3_API m3BodyId m3Joint_GetBodyA(const m3World* world, m3JointId joint);
    M3_API m3BodyId m3Joint_GetBodyB(const m3World* world, m3JointId joint);

    /// Runtime joint control. All journaled; both bodies wake on any
    /// change, and each call zeroes the stored impulse of the rows it
    /// changes, so a stale warm start cannot kick. A refused call changes
    /// nothing and counts one misuse; a getter reads zeros for a stale id,
    /// which counts one misuse, and for a type without that part.
    ///
    /// Limits: angles for revolute joints and the spherical twist, meters
    /// for prismatic and wheel joints. The range is ordered when it is set
    /// and whenever the limit is on. Other joint types take no limits; a
    /// distance joint's range is its length range below.
    M3_API void m3Joint_EnableLimit(m3World* world, m3JointId joint, bool enable);
    M3_API bool m3Joint_IsLimitEnabled(const m3World* world, m3JointId joint);
    M3_API void m3Joint_SetLimits(m3World* world, m3JointId joint, float lower, float upper);
    /// Either output may be null.
    M3_API void m3Joint_GetLimits(const m3World* world, m3JointId joint, float* lowerOut,
                                  float* upperOut);

    /// Motor: only revolute, prismatic, wheel and generic joints take one.
    /// Its speed is within 1e5 rad/s or m/s and its effort in [0, 1e20].
    M3_API void m3Joint_EnableMotor(m3World* world, m3JointId joint, bool enable);
    M3_API bool m3Joint_IsMotorEnabled(const m3World* world, m3JointId joint);
    M3_API void m3Joint_SetMotorSpeed(m3World* world, m3JointId joint, float speed);
    M3_API float m3Joint_GetMotorSpeed(const m3World* world, m3JointId joint);
    M3_API void m3Joint_SetMaxMotorEffort(m3World* world, m3JointId joint, float maxEffort);
    M3_API float m3Joint_GetMaxMotorEffort(const m3World* world, m3JointId joint);

    /// The motor joint's aim: linearOffset is body B's target position
    /// in body A's joint frame, within 1e4 m on every axis, and
    /// angularOffset its target rotation relative to that frame, a unit
    /// quaternion within 10% that is stored normalized. A fresh servo aims
    /// at its create pose. Motor joints only, here and for the budgets:
    /// each setter refuses any other type loudly, each getter reads zeros
    /// for it, and either counts one misuse.
    M3_API void m3MotorJoint_SetOffsets(m3World* world, m3JointId joint, m3Vec3 linearOffset,
                                        m3Quat angularOffset);
    M3_API m3Vec3 m3MotorJoint_GetLinearOffset(const m3World* world, m3JointId joint);
    M3_API m3Quat m3MotorJoint_GetAngularOffset(const m3World* world, m3JointId joint);

    /// The motor joint's budgets: the force its translation drive and the
    /// torque its rotation drive may spend, each in [0, 1e20], 0 for no
    /// cap.
    M3_API void m3MotorJoint_SetMaxForce(m3World* world, m3JointId joint, float maxForce);
    M3_API float m3MotorJoint_GetMaxForce(const m3World* world, m3JointId joint);
    M3_API void m3MotorJoint_SetMaxTorque(m3World* world, m3JointId joint, float maxTorque);
    M3_API float m3MotorJoint_GetMaxTorque(const m3World* world, m3JointId joint);

    /// The distance joint's length range in meters, 0 <= minLength <=
    /// maxLength, finite: a rod is minLength == maxLength, a rope a range.
    /// Distance joints only: the setter refuses any other type loudly, the
    /// getter reads zeros for it, and either counts one misuse. Either
    /// output may be null. Journaled.
    M3_API void m3DistanceJoint_SetLengthRange(m3World* world, m3JointId joint, float minLength,
                                               float maxLength);
    M3_API void m3DistanceJoint_GetLengthRange(const m3World* world, m3JointId joint,
                                               float* minLengthOut, float* maxLengthOut);

    /// The distance joint's spring rest length in meters, finite and not
    /// negative, which the joint spring part pulls toward while it is on.
    /// Distance joints only, as the length range. Journaled.
    M3_API void m3DistanceJoint_SetLength(m3World* world, m3JointId joint, float length);
    M3_API float m3DistanceJoint_GetLength(const m3World* world, m3JointId joint);

    /// Wheel steering: a soft drive about the strut axis toward a target
    /// angle. While steering is enabled, the wheel's frame-x lock becomes
    /// the drive (frame y stays locked), so the axle yaws toward the
    /// target. The target is in radians with |target| <= 1, the
    /// frequency in [0, 1e5] and positive while enabled, the damping
    /// ratio in [0, 1e4] and the torque budget in [0, 1e20], 0 for no
    /// cap. The values hold while steering is off, so set the frequency
    /// before enabling it. Wheel joints only: each setter refuses any
    /// other type loudly, each getter reads 0 for it, and either counts
    /// one misuse. Journaled.
    M3_API void m3WheelJoint_EnableSteering(m3World* world, m3JointId joint, bool enable);
    M3_API bool m3WheelJoint_IsSteeringEnabled(const m3World* world, m3JointId joint);
    M3_API void m3WheelJoint_SetTargetSteeringAngle(m3World* world, m3JointId joint, float radians);
    M3_API float m3WheelJoint_GetTargetSteeringAngle(const m3World* world, m3JointId joint);
    M3_API void m3WheelJoint_SetSteeringHertz(m3World* world, m3JointId joint, float hertz);
    M3_API float m3WheelJoint_GetSteeringHertz(const m3World* world, m3JointId joint);
    M3_API void m3WheelJoint_SetSteeringDampingRatio(m3World* world, m3JointId joint,
                                                     float dampingRatio);
    M3_API float m3WheelJoint_GetSteeringDampingRatio(const m3World* world, m3JointId joint);
    M3_API void m3WheelJoint_SetMaxSteeringTorque(m3World* world, m3JointId joint, float maxTorque);
    M3_API float m3WheelJoint_GetMaxSteeringTorque(const m3World* world, m3JointId joint);

    /// The current strut twist in radians (wheel joints; 0 for
    /// every other type, which counts one misuse, and stale ids).
    M3_API float m3WheelJoint_GetSteeringAngle(const m3World* world, m3JointId joint);

    /// Let (or forbid) the two jointed bodies collide with each
    /// other. Journaled; binds at the next step's pair scan.
    M3_API void m3Joint_EnableConnectedCollision(m3World* world, m3JointId joint, bool flag);
    M3_API bool m3Joint_IsConnectedCollisionEnabled(const m3World* world, m3JointId joint);

    /// Breakage: rollback games need breakage as a deterministic in-step
    /// state transition, not a host poll racing the journal. When
    /// either reaction magnitude exceeds its cap at the end of a
    /// step, the joint destroys itself and emits the joint break
    /// event (see world.h). Zero disables a cap; both zero (the
    /// default) means unbreakable.
    M3_API void m3Joint_SetBreakLimits(m3World* world, m3JointId joint, float maxForce,
                                       float maxTorque);
    /// The break limits m3Joint_SetBreakLimits sets, 0 for a cap that is
    /// off. A stale id reads zeros and counts one misuse; a null world
    /// reads zeros. Either output may be null.
    M3_API void m3Joint_GetBreakLimits(const m3World* world, m3JointId joint, float* maxForceOut,
                                       float* maxTorqueOut);

    /// Reaction readback: MAGNITUDES of the last step's
    /// constraint reactions, assembled per type from the stored
    /// solver rows (linear rows into force, angular rows into
    /// torque; the generic joint reports a conservative sum). Reads
    /// 0 before the first step after a restore (documented
    /// transient).
    M3_API float m3Joint_GetReactionForce(const m3World* world, m3JointId joint);
    M3_API float m3Joint_GetReactionTorque(const m3World* world, m3JointId joint);

    /// Geometry reads: the angle about the hinge of a revolute joint,
    /// or a wheel joint's spin about its axle (radians), and the
    /// translation along a prismatic joint's slide axis or a wheel
    /// joint's suspension axis (meters). Any other type, or a stale
    /// id, reads 0 and counts one misuse.
    M3_API float m3Joint_GetAngle(const m3World* world, m3JointId joint);
    M3_API float m3Joint_GetTranslation(const m3World* world, m3JointId joint);

    /// Position drive: a soft constraint with the given frequency and
    /// damping ratio pulls the joint toward its target. Revolute drives
    /// the hinge angle, prismatic and wheel the translation, spherical the
    /// relative rotation, the motor joint its aim, and the distance joint
    /// the anchor distance toward its length; other types refuse.
    /// The frequency is in [0, 1e5] Hz and positive while the spring is
    /// on, the damping ratio in [0, 1e4]. The spring keeps its values
    /// while it is off.
    M3_API void m3Joint_EnableSpring(m3World* world, m3JointId joint, bool enable);
    M3_API bool m3Joint_IsSpringEnabled(const m3World* world, m3JointId joint);
    M3_API void m3Joint_SetSpringHertz(m3World* world, m3JointId joint, float hertz);
    M3_API float m3Joint_GetSpringHertz(const m3World* world, m3JointId joint);
    M3_API void m3Joint_SetSpringDampingRatio(m3World* world, m3JointId joint, float dampingRatio);
    M3_API float m3Joint_GetSpringDampingRatio(const m3World* world, m3JointId joint);
    M3_API void m3RevoluteJoint_SetTargetAngle(m3World* world, m3JointId joint, float radians);
    M3_API void m3Joint_SetTargetTranslation(m3World* world, m3JointId joint, float meters);
    /// The target must be a unit rotation; garbage refuses loudly by
    /// doing nothing. The target lives in the JOINT FRAMES (frame z
    /// is the create-time local axis): it is the desired rotation of
    /// frame B relative to frame A. Pick
    /// your axes at create time so the frame reads naturally.
    M3_API void m3SphericalJoint_SetTargetRotation(m3World* world, m3JointId joint, m3Quat target);
    /// The swing cone (m3JointDef enableCone and coneAngle), switched and
    /// moved at run time; each takes effect in its own call. The angle is
    /// finite and not negative, and EnableCone refuses to switch the cone
    /// on over a negative angle a def left. Any other joint type, a stale
    /// id or a refused value changes nothing and counts one misuse; the
    /// getters then read false and zero.
    M3_API void m3SphericalJoint_EnableCone(m3World* world, m3JointId joint, bool enable);
    M3_API bool m3SphericalJoint_IsConeEnabled(const m3World* world, m3JointId joint);
    M3_API void m3SphericalJoint_SetConeAngle(m3World* world, m3JointId joint, float angle);
    M3_API float m3SphericalJoint_GetConeAngle(const m3World* world, m3JointId joint);

#ifdef __cplusplus
}
#endif

#endif
