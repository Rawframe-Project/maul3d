// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joints: the twelve joint types, their defs, limits, motors,
// springs, reaction force readback and breaking.

#ifndef MAUL3D_JOINT_H
#define MAUL3D_JOINT_H

#include "body.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef int32_t m3JointType;
    enum
    {
        m3_sphericalJoint = 0, // ball: pins two body-frame points together
        m3_revoluteJoint = 1,  // hinge: one rotation axis, limits, motor
        m3_prismaticJoint = 2, // slider: one translation axis, limits, motor
        m3_weldJoint = 3,      // full 6-DOF lock at the create pose
        m3_distanceJoint = 4,  // rope/rod: anchor distance in [minLength, maxLength]
        m3_genericJoint = 5,   // 6-DOF: per-axis lock/free/limit + one motor
        m3_wheelJoint = 6,     // suspension slide plus a free spinning axle
        m3_filterJoint = 7,    // no rows: only stops the pair colliding
        m3_parallelJoint = 8,  // keeps two body axes parallel
        m3_motorJoint = 9,     // servo weld toward a target pose
        m3_gearJoint = 10,     // couples two bodies' spins at a ratio
        m3_pulleyJoint = 11,   // rope over two fixed world points
    };

    /// Per-axis behavior of the generic joint, in the joint frame
    /// built from localAxisA and localAxisB (the frame's z axis).
    typedef int32_t m3AxisMode;
    enum
    {
        m3_axisLocked = 0,
        m3_axisFree = 1,
        m3_axisLimited = 2,
    };

    /// The generic joint's motorAxis when it has no motor.
#define M3_GENERIC_NO_MOTOR 255

    // Each joint type has its own def, holding only what that type
    // reads. Build one with its m3Default*JointDef and create it with its
    // m3Create*Joint; a hand-rolled def, or another type's, is refused.
    // Anchors are points in each body's frame, each component within
    // 1e4 m. enableConnectedCollision lets the two bodies collide, off by
    // default (the classic chain-fight guard). The spring, on the types
    // that take one, has springHertz in [0, 1e5], positive when
    // enableSpring is set, and springDampingRatio in [0, 1e4]. A motor
    // speed is within 1e5 rad/s or m/s and its effort in [0, 1e20].

    /// A ball: pins two anchors together. The twist about the A frame's
    /// z axis may be limited, and the swing kept within a cone around it.
    typedef struct m3SphericalJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;

        /// The twist axis in each body's frame (unit z by default); the
        /// cone opens around A's.
        m3Vec3 localAxisA;
        m3Vec3 localAxisB;
        bool enableSpring; // pulls toward the target rotation
        float springHertz;
        float springDampingRatio;
        bool enableLimit; // the twist range
        float lowerTwist; // radians, ordered when the limit is on
        float upperTwist;
        bool enableCone;
        float coneAngle; // radians from the A frame's z axis
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultSphericalJointDef sets
    } m3SphericalJointDef;

    /// A hinge: one rotation axis, with a limit, a motor and a spring.
    typedef struct m3RevoluteJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;

        /// The hinge axis in each body's frame. The two must map to the
        /// same world direction at create for a well-posed hinge.
        m3Vec3 localAxisA;
        m3Vec3 localAxisB;
        bool enableSpring; // pulls toward the target angle
        float springHertz;
        float springDampingRatio;
        bool enableMotor;
        float motorSpeed;     // rad/s
        float maxMotorEffort; // the motor's torque budget, N m
        bool enableLimit;
        float lowerAngle; // radians, ordered when the limit is on
        float upperAngle;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultRevoluteJointDef sets
    } m3RevoluteJointDef;

    /// A slider: one translation axis, with a limit, a motor and a spring.
    typedef struct m3PrismaticJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;

        /// The slide axis in each body's frame.
        m3Vec3 localAxisA;
        m3Vec3 localAxisB;
        bool enableSpring; // pulls toward the target translation
        float springHertz;
        float springDampingRatio;
        bool enableMotor;
        float motorSpeed;     // m/s
        float maxMotorEffort; // the motor's force budget, N
        bool enableLimit;
        float lowerTranslation; // meters, ordered when the limit is on
        float upperTranslation;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultPrismaticJointDef sets
    } m3PrismaticJointDef;

    /// A full 6-DOF lock at the create pose.
    typedef struct m3WeldJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultWeldJointDef sets
    } m3WeldJointDef;

    /// A rod or a rope: the anchor distance stays within [minLength,
    /// maxLength], a rod with the two equal, a rope a range. The optional
    /// spring pulls toward length, which need not lie within the range:
    /// the range holds whatever the spring does.
    typedef struct m3DistanceJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;
        float length;    // the spring's rest length, meters, finite and not negative
        float minLength; // meters, 0 <= minLength <= maxLength
        float maxLength;
        bool enableSpring;
        float springHertz;
        float springDampingRatio;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultDistanceJointDef sets
    } m3DistanceJointDef;

    /// A 6-DOF joint: each axis of the joint frame is locked, free or
    /// limited, and one motor may drive a free or limited axis while
    /// enableMotor is on, as every motor follows its switch. Limits are
    /// meters (linear) and radians (angular), ordered on a limited axis.
    /// At most ONE angular axis may be limited, and the other two must be
    /// BOTH locked (a hinge with range) or BOTH free (a twist-limited
    /// ball); anything else refuses loudly.
    typedef struct m3GenericJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;

        /// The joint frame's z axis in each body's frame.
        m3Vec3 localAxisA;
        m3Vec3 localAxisB;
        m3AxisMode linearModes[3];
        m3AxisMode angularModes[3];
        float linearLower[3];
        float linearUpper[3];
        float angularLower[3];
        float angularUpper[3];

        /// 0 to 2 for a linear axis, 3 to 5 for an angular one, or
        /// M3_GENERIC_NO_MOTOR, the default.
        uint8_t motorAxis;
        bool enableMotor;     // drives motorAxis while on
        float motorSpeed;     // m/s or rad/s by axis
        float maxMotorEffort; // the motor's force or torque budget
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultGenericJointDef sets
    } m3GenericJointDef;

    /// The wheel joint, the OPTIONAL rigid-wheel path for vehicles (the
    /// raycast vehicle stays the default): the wheel body B slides along a
    /// suspension axis fixed in the chassis A and spins freely about its
    /// axle. The axle is captured at create, snapped exactly perpendicular
    /// to the suspension axis; more than a small skew (|dot| > 0.1)
    /// refuses loudly. The limit bounds the suspension travel in meters;
    /// the motor drives the SPIN (rad/s, torque budget): the drive axle.
    /// The suspension spring is the joint spring, aimed with
    /// m3Joint_SetTargetTranslation. m3Joint_GetAngle reads the spin
    /// angle, m3Joint_GetTranslation the suspension travel. Breakage
    /// applies unchanged: a capped axle snaps deterministically and the
    /// wheel body rolls away.
    typedef struct m3WheelJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;
        m3Vec3 localAxisA;     // the suspension axis in the chassis frame, usually down
        m3Vec3 localSpinAxisB; // the axle in the wheel's frame, its spin axis
        bool enableSpring;     // the suspension spring
        float springHertz;
        float springDampingRatio;
        bool enableMotor;
        float motorSpeed;     // rad/s of spin
        float maxMotorEffort; // the motor's torque budget, N m
        bool enableLimit;
        float lowerTranslation; // meters of suspension travel, ordered when the limit is on
        float upperTranslation;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultWheelJointDef sets
    } m3WheelJointDef;

    /// No constraint rows at all: the joint only switches collision off
    /// between its two bodies for as long as it lives. Ragdoll sibling
    /// limbs stop grinding without buying a single solver row.
    typedef struct m3FilterJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        int32_t internalValue; // the cookie m3DefaultFilterJointDef sets
    } m3FilterJointDef;

    /// Keeps localAxisA on body A parallel to localAxisB on body B (two
    /// angular rows); every translation and the twist about the shared
    /// axis stay free. Platform linkages without the full revolute.
    typedef struct m3ParallelJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;
        m3Vec3 localAxisA;
        m3Vec3 localAxisB;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultParallelJointDef sets
    } m3ParallelJointDef;

    /// A servo weld. Drives body B toward a target pose relative to body
    /// A's create-time joint frame: a soft 3-DOF rotation drive plus a soft
    /// 3-DOF translation drive, each within its budget. Tune with the
    /// joint spring (the shared stiffness), aim with
    /// m3MotorJoint_SetOffsets. Without a spring the servo idles
    /// completely free. A fresh servo aims at its own create pose.
    typedef struct m3MotorJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;
        float maxForce;  // N, in [0, 1e20], 0 for no cap
        float maxTorque; // N m, in [0, 1e20], 0 for no cap
        bool enableSpring;
        float springHertz;
        float springDampingRatio;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultMotorJointDef sets
    } m3MotorJointDef;

    /// Couples spin so that spinA + ratio * spinB keeps its create value,
    /// where spinX is body X's rotation about its own axis localAxisX. A
    /// positive ratio is the external mesh (counter-rotation: ratio 2
    /// turns B half as fast, backwards); a negative one couples the same
    /// direction (a belt or an internal ring). MOUNTING CONTRACT: both
    /// bodies must be hinged on a common rigid frame (revolute or wheel
    /// joints to the same chassis); the gear adds no reaction on that
    /// frame and holds no other degree of freedom. Angle drift is
    /// corrected softly, so the mesh cannot creep under load.
    typedef struct m3GearJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Vec3 localAxisA;
        m3Vec3 localAxisB;
        float ratio; // nonzero, within 1e4 by magnitude
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultGearJointDef sets
    } m3GearJointDef;

    /// A rope from localAnchorA up over the WORLD point groundAnchorA,
    /// across to groundAnchorB and down to localAnchorB: length1 + ratio *
    /// length2 keeps its create value. The rope is RIGID both ways (it can
    /// push; pair it with a distance rope when slack matters). Block and
    /// tackle: ratio 2 lifts B twice as slow with twice the force.
    typedef struct m3PulleyJointDef
    {
        m3BodyId bodyIdA;
        m3BodyId bodyIdB;
        m3Pos3 groundAnchorA; // world points, within 1e15 m
        m3Pos3 groundAnchorB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;
        float ratio; // positive, at most 1e4
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m3DefaultPulleyJointDef sets
    } m3PulleyJointDef;

    // The pinned defaults: zero anchors, unit z axes (the wheel's
    // suspension points down -y and its axle along z), every part off,
    // enableConnectedCollision off, ratio 1 for the gear and the pulley,
    // no motor axis for the generic joint, and the type's cookie.

    /// Returns a spherical joint def with the defaults: zero anchors, unit z
    /// twist axes, and no spring, twist limit or cone, with the cookie
    /// m3CreateSphericalJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3SphericalJointDef m3DefaultSphericalJointDef(void);

    /// Returns a revolute joint def with the defaults: zero anchors, unit z
    /// hinge axes, and no spring, motor or limit, with the cookie
    /// m3CreateRevoluteJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3RevoluteJointDef m3DefaultRevoluteJointDef(void);

    /// Returns a prismatic joint def with the defaults: zero anchors, unit z
    /// slide axes, and no spring, motor or limit, with the cookie
    /// m3CreatePrismaticJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3PrismaticJointDef m3DefaultPrismaticJointDef(void);

    /// Returns a weld joint def with the defaults: zero anchors, with the
    /// cookie m3CreateWeldJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3WeldJointDef m3DefaultWeldJointDef(void);

    /// Returns a distance joint def with the defaults: zero anchors, a zero
    /// range and rest length, and no spring, with the cookie
    /// m3CreateDistanceJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3DistanceJointDef m3DefaultDistanceJointDef(void);

    /// Returns a generic joint def with the defaults: zero anchors, unit z
    /// axes, every axis locked with zero limits, and no motor axis
    /// (M3_GENERIC_NO_MOTOR), with the cookie m3CreateGenericJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3GenericJointDef m3DefaultGenericJointDef(void);

    /// Returns a wheel joint def with the defaults: zero anchors, a suspension
    /// pointing down -y, an axle along z, and no spring, motor or limit, with
    /// the cookie m3CreateWheelJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3WheelJointDef m3DefaultWheelJointDef(void);

    /// Returns a filter joint def with the defaults: no bodies, with the cookie
    /// m3CreateFilterJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3FilterJointDef m3DefaultFilterJointDef(void);

    /// Returns a parallel joint def with the defaults: unit z axes, with the
    /// cookie m3CreateParallelJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3ParallelJointDef m3DefaultParallelJointDef(void);

    /// Returns a motor joint def with the defaults: zero anchors, no force or
    /// torque budget, and no spring, with the cookie m3CreateMotorJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3MotorJointDef m3DefaultMotorJointDef(void);

    /// Returns a gear joint def with the defaults: unit z axes and a ratio of
    /// 1, with the cookie m3CreateGearJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3GearJointDef m3DefaultGearJointDef(void);

    /// Returns a pulley joint def with the defaults: zero anchors, ground
    /// anchors at the origin and a ratio of 1, with the cookie
    /// m3CreatePulleyJoint checks.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3PulleyJointDef m3DefaultPulleyJointDef(void);

    /// Create a joint between two distinct bodies of the same world (at
    /// least one dynamic). Each call takes only its own type's def: a def
    /// with another type's cookie, or none, is m3_errorInvalid. Returns
    /// m3_errorInvalid for a bad def, m3_errorLimit for a value past a
    /// bound in maul3d/base.h or for a spherical, revolute, prismatic,
    /// weld, wheel or motor joint whose bodies' lever ratio would pass
    /// M3_JOINT_LEVER_LIMIT, m3_errorStale for a stale body and
    /// m3_errorCapacity for an exhausted pool; a refusal other than the
    /// lever ratio and the full pool counts one misuse, and *jointOut is
    /// the null id on any failure. Journaled; replay verifies the minted
    /// id. Destroying either body destroys the joint.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultSphericalJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultSphericalJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, an axis that is not finite, a limit bound that is not
    ///         finite (checked with the limit off too) or bounds out of order
    ///         with the limit on, a cone angle that is not finite (checked with
    ///         the cone off too) or is negative with the cone on, a spring
    ///         hertz or damping ratio that is negative or NaN (checked with the
    ///         spring off too), or a zero hertz with the spring on;
    ///         m3_errorStale for a stale body; m3_errorLimit for an anchor past
    ///         M3_LOCAL_LIMIT, a spring hertz past 1e5 or a damping ratio past
    ///         1e4, infinities included; m3_errorLimit, counting no misuse,
    ///         when the bodies' lever ratio would pass M3_JOINT_LEVER_LIMIT;
    ///         m3_errorCapacity for a full joint pool; m3_errorBusy from host
    ///         code the world is running. Every refusal but a full pool's, the
    ///         lever's and a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateSphericalJoint(m3World* world,
                                                        const m3SphericalJointDef* def,
                                                        m3JointId* jointOut);

    /// Creates a revolute joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultRevoluteJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultRevoluteJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, an axis that is zero or not finite, a limit bound that
    ///         is not finite (checked with the limit off too) or bounds out of
    ///         order with the limit on, a NaN motor speed or a motor effort
    ///         that is negative or NaN (checked with the motor off too), a
    ///         spring hertz or damping ratio that is negative or NaN (checked
    ///         with the spring off too), or a zero hertz with the spring on;
    ///         m3_errorStale for a stale body; m3_errorLimit for an anchor past
    ///         M3_LOCAL_LIMIT, a motor speed past 1e5 or an effort past 1e20, a
    ///         spring hertz past 1e5 or a damping ratio past 1e4, infinities
    ///         included; m3_errorLimit, counting no misuse, when the bodies'
    ///         lever ratio would pass M3_JOINT_LEVER_LIMIT; m3_errorCapacity
    ///         for a full joint pool; m3_errorBusy from host code the world is
    ///         running. Every refusal but a full pool's, the lever's and a null
    ///         world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateRevoluteJoint(m3World* world,
                                                       const m3RevoluteJointDef* def,
                                                       m3JointId* jointOut);

    /// Creates a prismatic joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultPrismaticJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultPrismaticJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, an axis that is zero or not finite, a limit bound that
    ///         is not finite (checked with the limit off too) or bounds out of
    ///         order with the limit on, a NaN motor speed or a motor effort
    ///         that is negative or NaN (checked with the motor off too), a
    ///         spring hertz or damping ratio that is negative or NaN (checked
    ///         with the spring off too), or a zero hertz with the spring on;
    ///         m3_errorStale for a stale body; m3_errorLimit for an anchor past
    ///         M3_LOCAL_LIMIT, a motor speed past 1e5 or an effort past 1e20, a
    ///         spring hertz past 1e5 or a damping ratio past 1e4, infinities
    ///         included; m3_errorLimit, counting no misuse, when the bodies'
    ///         lever ratio would pass M3_JOINT_LEVER_LIMIT; m3_errorCapacity
    ///         for a full joint pool; m3_errorBusy from host code the world is
    ///         running. Every refusal but a full pool's, the lever's and a null
    ///         world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreatePrismaticJoint(m3World* world,
                                                        const m3PrismaticJointDef* def,
                                                        m3JointId* jointOut);

    /// Creates a weld joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultWeldJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultWeldJointDef did not make, a null body id, the same
    ///         body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor; m3_errorStale for a stale body; m3_errorLimit for an
    ///         anchor past M3_LOCAL_LIMIT, infinities included; m3_errorLimit,
    ///         counting no misuse, when the bodies' lever ratio would pass
    ///         M3_JOINT_LEVER_LIMIT; m3_errorCapacity for a full joint pool;
    ///         m3_errorBusy from host code the world is running. Every refusal
    ///         but a full pool's, the lever's and a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateWeldJoint(m3World* world, const m3WeldJointDef* def,
                                                   m3JointId* jointOut);

    /// Creates a distance joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultDistanceJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultDistanceJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, a range bound that is not finite or is negative, a
    ///         minLength above maxLength, a rest length that is not finite or
    ///         is negative, a spring hertz or damping ratio that is negative or
    ///         NaN (checked with the spring off too), or a zero hertz with the
    ///         spring on; m3_errorStale for a stale body; m3_errorLimit for an
    ///         anchor past M3_LOCAL_LIMIT, a spring hertz past 1e5 or a damping
    ///         ratio past 1e4, infinities included; m3_errorCapacity for a full
    ///         joint pool; m3_errorBusy from host code the world is running.
    ///         Every refusal but a full pool's and a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateDistanceJoint(m3World* world,
                                                       const m3DistanceJointDef* def,
                                                       m3JointId* jointOut);

    /// Creates a generic joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultGenericJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultGenericJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, an axis that is zero or not finite, an axis mode out of
    ///         range, a limit bound that is not finite, a limited axis whose
    ///         bounds are out of order, more than one limited angular axis or
    ///         one whose other two are not both locked or both free, a motor
    ///         axis past 5 that is not M3_GENERIC_NO_MOTOR, a motor on a locked
    ///         axis, a NaN motor speed or a motor effort that is negative or
    ///         NaN (checked with the motor off too); m3_errorStale for a stale
    ///         body; m3_errorLimit for an anchor past M3_LOCAL_LIMIT, a motor
    ///         speed past 1e5 or an effort past 1e20, infinities included;
    ///         m3_errorCapacity for a full joint pool; m3_errorBusy from host
    ///         code the world is running. Every refusal but a full pool's and a
    ///         null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateGenericJoint(m3World* world, const m3GenericJointDef* def,
                                                      m3JointId* jointOut);

    /// Creates a wheel joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultWheelJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultWheelJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, an axis that is zero or not finite, an axle more than a
    ///         small skew (|dot| > 0.1) off perpendicular to the suspension, a
    ///         limit bound that is not finite (checked with the limit off too)
    ///         or bounds out of order with the limit on, a NaN motor speed or a
    ///         motor effort that is negative or NaN (checked with the motor off
    ///         too), a spring hertz or damping ratio that is negative or NaN
    ///         (checked with the spring off too), or a zero hertz with the
    ///         spring on; m3_errorStale for a stale body; m3_errorLimit for an
    ///         anchor past M3_LOCAL_LIMIT, a motor speed past 1e5 or an effort
    ///         past 1e20, a spring hertz past 1e5 or a damping ratio past 1e4,
    ///         infinities included; m3_errorLimit, counting no misuse, when the
    ///         bodies' lever ratio would pass M3_JOINT_LEVER_LIMIT;
    ///         m3_errorCapacity for a full joint pool; m3_errorBusy from host
    ///         code the world is running. Every refusal but a full pool's, the
    ///         lever's and a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateWheelJoint(m3World* world, const m3WheelJointDef* def,
                                                    m3JointId* jointOut);

    /// Creates a filter joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultFilterJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultFilterJointDef did not make, a null body id, the
    ///         same body twice, or two bodies of which neither is dynamic;
    ///         m3_errorStale for a stale body; m3_errorCapacity for a full
    ///         joint pool; m3_errorBusy from host code the world is running.
    ///         Every refusal but a full pool's and a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateFilterJoint(m3World* world, const m3FilterJointDef* def,
                                                     m3JointId* jointOut);

    /// Creates a parallel joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultParallelJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultParallelJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, an axis that is zero or not finite; m3_errorStale for a
    ///         stale body; m3_errorLimit for an anchor past M3_LOCAL_LIMIT,
    ///         infinities included; m3_errorCapacity for a full joint pool;
    ///         m3_errorBusy from host code the world is running. Every refusal
    ///         but a full pool's and a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateParallelJoint(m3World* world,
                                                       const m3ParallelJointDef* def,
                                                       m3JointId* jointOut);

    /// Creates a motor joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultMotorJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultMotorJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, a force or torque budget that is negative or NaN, a
    ///         spring hertz or damping ratio that is negative or NaN (checked
    ///         with the spring off too), or a zero hertz with the spring on;
    ///         m3_errorStale for a stale body; m3_errorLimit for an anchor past
    ///         M3_LOCAL_LIMIT, a budget past 1e20, a spring hertz past 1e5 or a
    ///         damping ratio past 1e4, infinities included; m3_errorLimit,
    ///         counting no misuse, when the bodies' lever ratio would pass
    ///         M3_JOINT_LEVER_LIMIT; m3_errorCapacity for a full joint pool;
    ///         m3_errorBusy from host code the world is running. Every refusal
    ///         but a full pool's, the lever's and a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateMotorJoint(m3World* world, const m3MotorJointDef* def,
                                                    m3JointId* jointOut);

    /// Creates a gear joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultGearJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultGearJointDef did not make, a null body id, the same
    ///         body twice, two bodies of which neither is dynamic, an axis that
    ///         is not finite or whose squared length is 1e-8 or less, or a zero
    ///         or NaN ratio; m3_errorStale for a stale body; m3_errorLimit for
    ///         a ratio past 1e4 either way, infinities included;
    ///         m3_errorCapacity for a full joint pool; m3_errorBusy from host
    ///         code the world is running. Every refusal but a full pool's and a
    ///         null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateGearJoint(m3World* world, const m3GearJointDef* def,
                                                   m3JointId* jointOut);

    /// Creates a pulley joint; see m3CreateSphericalJoint for the rules every
    /// create shares.
    ///
    /// @param world     The world.
    /// @param def       The def, from m3DefaultPulleyJointDef.
    /// @param jointOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or jointOut, a
    ///         def m3DefaultPulleyJointDef did not make, a null body id, the
    ///         same body twice, two bodies of which neither is dynamic, a NaN
    ///         anchor, a NaN ground anchor, a ratio that is not positive or is
    ///         NaN, or an anchor whose rope end, at the bodies' create poses,
    ///         lies within 1e-3 m of its pulley; m3_errorStale for a stale
    ///         body; m3_errorLimit for an anchor past M3_LOCAL_LIMIT, a ground
    ///         anchor past M3_POSITION_LIMIT or a ratio past 1e4, infinities
    ///         included; m3_errorCapacity for a full joint pool; m3_errorBusy
    ///         from host code the world is running. Every refusal but a full
    ///         pool's and a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreatePulleyJoint(m3World* world, const m3PulleyJointDef* def,
                                                     m3JointId* jointOut);

    /// Destroys a joint. Journaled.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return m3_success; m3_errorInvalid for a null world or the null id;
    ///         m3_errorStale for a gone joint; m3_errorBusy from host code the
    ///         world is running. Each refusal but a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3DestroyJoint(m3World* world, m3JointId joint);

    /// Returns whether the id names a live joint of this world.
    ///
    /// @param world  The world; null gives false.
    /// @param joint  The id.
    /// @return Whether the joint is alive.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Joint_IsValid(const m3World* world, m3JointId joint);

    /// Returns the joint's type.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The type; m3_sphericalJoint for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3JointType m3Joint_GetType(const m3World* world, m3JointId joint);

    /// Returns the joint's first body.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return Body A's id; the null id for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3BodyId m3Joint_GetBodyA(const m3World* world, m3JointId joint);

    /// Returns the joint's second body.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return Body B's id; the null id for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
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
    /// Switches the limit on or off; the range must be ordered to switch
    /// it on. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not
    /// revolute, prismatic, spherical or wheel, switching the limit on while
    /// its bounds are out of order, or a call from host code the world is
    /// running changes nothing, is not journaled and counts one misuse.
    /// Revolute, prismatic, spherical and wheel joints have a limit. Both
    /// bodies wake, and the stored impulse of the rows it changes is zeroed.
    ///
    /// @param world   The world.
    /// @param joint   The joint.
    /// @param enable  Whether the limit holds.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_EnableLimit(m3World* world, m3JointId joint, bool enable);

    /// Returns whether the limit is on.
    /// Revolute, prismatic, spherical and wheel joints have a limit.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return Whether it is; false, for a type without this part, for a null
    ///         world, and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Joint_IsLimitEnabled(const m3World* world, m3JointId joint);

    /// Sets the limit range; finite, and ordered while the limit is on.
    /// Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not
    /// revolute, prismatic, spherical or wheel, a bound that is not finite, a
    /// lower bound above the upper one (refused with the limit off too), or a
    /// call from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    /// Revolute, prismatic, spherical and wheel joints have a limit. Both
    /// bodies wake, and the stored impulse of the rows it changes is zeroed.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @param lower  The lower bound, radians or meters by type.
    /// @param upper  The upper bound.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_SetLimits(m3World* world, m3JointId joint, float lower, float upper);

    /// Reads the limit range. Either output may be null.
    /// Revolute, prismatic, spherical and wheel joints have a limit.
    ///
    /// @param world     The world.
    /// @param joint     The joint.
    /// @param lowerOut  Receives the lower bound; 0, for a type without this
    ///                  part, for a null world, and for a stale or null id,
    ///                  which counts one misuse.
    /// @param upperOut  Receives the upper bound, as lowerOut.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API void m3Joint_GetLimits(const m3World* world, m3JointId joint, float* lowerOut,
                                  float* upperOut);

    /// Motor: only revolute, prismatic, wheel and generic joints take one.
    /// Its speed is within 1e5 rad/s or m/s and its effort in [0, 1e20].
    /// Switches the motor on or off. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not
    /// revolute, prismatic, wheel or generic, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    /// Revolute, prismatic, wheel and generic joints have a motor. Both bodies
    /// wake, and the stored impulse of the rows it changes is zeroed.
    ///
    /// @param world   The world.
    /// @param joint   The joint.
    /// @param enable  Whether the motor drives.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_EnableMotor(m3World* world, m3JointId joint, bool enable);

    /// Returns whether the motor is on.
    /// Revolute, prismatic, wheel and generic joints have a motor.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return Whether it is; false, for a type without this part, for a null
    ///         world, and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Joint_IsMotorEnabled(const m3World* world, m3JointId joint);

    /// Sets the motor's target speed. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not
    /// revolute, prismatic, wheel or generic, a speed that is NaN or past 1e5
    /// either way, infinities included, or a call from host code the world is
    /// running changes nothing, is not journaled and counts one misuse.
    /// Revolute, prismatic, wheel and generic joints have a motor. Both bodies
    /// wake, and the stored impulse of the rows it changes is zeroed.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @param speed  The speed, rad/s or m/s by type, within 1e5.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_SetMotorSpeed(m3World* world, m3JointId joint, float speed);

    /// Returns the motor's target speed.
    /// Revolute, prismatic, wheel and generic joints have a motor.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The speed; 0, for a type without this part, for a null world,
    ///         and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Joint_GetMotorSpeed(const m3World* world, m3JointId joint);

    /// Sets the motor's force or torque budget. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not
    /// revolute, prismatic, wheel or generic, an effort that is negative, NaN
    /// or past 1e20, infinities included, or a call from host code the world is
    /// running changes nothing, is not journaled and counts one misuse.
    /// Revolute, prismatic, wheel and generic joints have a motor. Both bodies
    /// wake, and the stored impulse of the rows it changes is zeroed.
    ///
    /// @param world      The world.
    /// @param joint      The joint.
    /// @param maxEffort  The budget, N or N m by type, in [0, 1e20].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_SetMaxMotorEffort(m3World* world, m3JointId joint, float maxEffort);

    /// Returns the motor's force or torque budget.
    /// Revolute, prismatic, wheel and generic joints have a motor.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The budget; 0, for a type without this part, for a null world,
    ///         and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Joint_GetMaxMotorEffort(const m3World* world, m3JointId joint);

    /// The motor joint's aim: linearOffset is body B's target position
    /// in body A's joint frame, within 1e4 m on every axis, and
    /// angularOffset its target rotation relative to that frame, a unit
    /// quaternion within 10% that is stored normalized. A fresh servo aims
    /// at its create pose. Motor joints only, here and for the budgets:
    /// each setter refuses any other type loudly, each getter reads zeros
    /// for it, and either counts one misuse.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// motor joint, a linear offset component that is not finite or past 1e4,
    /// an angular offset that is not finite or whose squared length is outside
    /// [0.81, 1.21], or a call from host code the world is running changes
    /// nothing, is not journaled and counts one misuse.
    ///
    /// @param world          The world.
    /// @param joint          The joint.
    /// @param linearOffset   Body B's target position in A's joint frame.
    /// @param angularOffset  B's target rotation, a quaternion within 10% of
    ///                       unit length.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3MotorJoint_SetOffsets(m3World* world, m3JointId joint, m3Vec3 linearOffset,
                                        m3Quat angularOffset);

    /// Returns the motor joint's linear aim.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The offset; zero for a null world, and for a stale or null id or
    ///         another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3MotorJoint_GetLinearOffset(const m3World* world, m3JointId joint);

    /// Returns the motor joint's angular aim, normalized.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The rotation; identity for a null world, and for a stale or null
    ///         id or another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Quat m3MotorJoint_GetAngularOffset(const m3World* world, m3JointId joint);

    /// The motor joint's budgets: the force its translation drive and the
    /// torque its rotation drive may spend, each in [0, 1e20], 0 for no
    /// cap.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// motor joint, a budget that is negative, NaN or past 1e20, infinities
    /// included, or a call from host code the world is running changes nothing,
    /// is not journaled and counts one misuse.
    ///
    /// @param world     The world.
    /// @param joint     The joint.
    /// @param maxForce  The force budget, N, in [0, 1e20].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3MotorJoint_SetMaxForce(m3World* world, m3JointId joint, float maxForce);

    /// Returns the motor joint's force budget.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The budget; 0 for a null world, and for a stale or null id or
    ///         another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3MotorJoint_GetMaxForce(const m3World* world, m3JointId joint);

    /// Sets the motor joint's torque budget. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// motor joint, a budget that is negative, NaN or past 1e20, infinities
    /// included, or a call from host code the world is running changes nothing,
    /// is not journaled and counts one misuse.
    ///
    /// @param world      The world.
    /// @param joint      The joint.
    /// @param maxTorque  The torque budget, N m, in [0, 1e20].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3MotorJoint_SetMaxTorque(m3World* world, m3JointId joint, float maxTorque);

    /// Returns the motor joint's torque budget.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The budget; 0 for a null world, and for a stale or null id or
    ///         another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3MotorJoint_GetMaxTorque(const m3World* world, m3JointId joint);

    /// The distance joint's length range in meters, 0 <= minLength <=
    /// maxLength, finite: a rod is minLength == maxLength, a rope a range.
    /// Distance joints only: the setter refuses any other type loudly, the
    /// getter reads zeros for it, and either counts one misuse. Either
    /// output may be null. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// distance joint, a bound that is not finite, a negative minimum, a
    /// minimum above the maximum, or a call from host code the world is running
    /// changes nothing, is not journaled and counts one misuse.
    ///
    /// @param world      The world.
    /// @param joint      The joint.
    /// @param minLength  The shortest anchor distance, meters.
    /// @param maxLength  The longest, at least minLength.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3DistanceJoint_SetLengthRange(m3World* world, m3JointId joint, float minLength,
                                               float maxLength);

    /// Reads the distance joint's length range. Either output may be null.
    ///
    /// @param world         The world.
    /// @param joint         The joint.
    /// @param minLengthOut  Receives the shortest distance; 0 for a null world,
    ///                      and for a stale or null id or another joint type,
    ///                      which counts one misuse.
    /// @param maxLengthOut  Receives the longest, as minLengthOut.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API void m3DistanceJoint_GetLengthRange(const m3World* world, m3JointId joint,
                                               float* minLengthOut, float* maxLengthOut);

    /// The distance joint's spring rest length in meters, finite and not
    /// negative, which the joint spring part pulls toward while it is on.
    /// Distance joints only, as the length range. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// distance joint, a length that is negative or not finite, or a call from
    /// host code the world is running changes nothing, is not journaled and
    /// counts one misuse.
    ///
    /// @param world   The world.
    /// @param joint   The joint.
    /// @param length  The rest length, meters.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3DistanceJoint_SetLength(m3World* world, m3JointId joint, float length);

    /// Returns the distance joint's spring rest length.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The length, meters; 0 for a null world, and for a stale or null
    ///         id or another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
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
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// wheel joint, switching steering on while its hertz is zero, or a call
    /// from host code the world is running changes nothing, is not journaled
    /// and counts one misuse.
    ///
    /// @param world   The world.
    /// @param joint   The joint.
    /// @param enable  Whether the steering drive runs.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3WheelJoint_EnableSteering(m3World* world, m3JointId joint, bool enable);

    /// Returns whether the steering drive runs.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return Whether it does; false for a null world, and for a stale or null
    ///         id or another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3WheelJoint_IsSteeringEnabled(const m3World* world, m3JointId joint);

    /// Sets the steering target. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// wheel joint, an angle that is NaN or past 1 radian either way, or a call
    /// from host code the world is running changes nothing, is not journaled
    /// and counts one misuse.
    ///
    /// @param world    The world.
    /// @param joint    The joint.
    /// @param radians  The target, within 1 radian either way.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3WheelJoint_SetTargetSteeringAngle(m3World* world, m3JointId joint, float radians);

    /// Returns the steering target.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The target, radians; 0 for a null world, and for a stale or null
    ///         id or another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3WheelJoint_GetTargetSteeringAngle(const m3World* world, m3JointId joint);

    /// Sets the steering drive's frequency. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// wheel joint, a hertz that is negative, NaN or past 1e5, a zero hertz
    /// while steering is on, or a call from host code the world is running
    /// changes nothing, is not journaled and counts one misuse.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @param hertz  The frequency, in [0, 1e5], positive while steering is on.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3WheelJoint_SetSteeringHertz(m3World* world, m3JointId joint, float hertz);

    /// Returns the steering drive's frequency.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The frequency; 0 for a null world, and for a stale or null id or
    ///         another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3WheelJoint_GetSteeringHertz(const m3World* world, m3JointId joint);

    /// Sets the steering drive's damping ratio. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// wheel joint, a ratio that is negative, NaN or past 1e4, or a call from
    /// host code the world is running changes nothing, is not journaled and
    /// counts one misuse.
    ///
    /// @param world         The world.
    /// @param joint         The joint.
    /// @param dampingRatio  The ratio, in [0, 1e4].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3WheelJoint_SetSteeringDampingRatio(m3World* world, m3JointId joint,
                                                     float dampingRatio);

    /// Returns the steering drive's damping ratio.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The ratio; 0 for a null world, and for a stale or null id or
    ///         another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3WheelJoint_GetSteeringDampingRatio(const m3World* world, m3JointId joint);

    /// Sets the steering drive's torque budget. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// wheel joint, a torque that is negative, NaN or past 1e20, or a call from
    /// host code the world is running changes nothing, is not journaled and
    /// counts one misuse.
    ///
    /// @param world      The world.
    /// @param joint      The joint.
    /// @param maxTorque  The budget, N m, in [0, 1e20], 0 for no cap.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3WheelJoint_SetMaxSteeringTorque(m3World* world, m3JointId joint, float maxTorque);

    /// Returns the steering drive's torque budget.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The budget; 0 for a null world, and for a stale or null id or
    ///         another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3WheelJoint_GetMaxSteeringTorque(const m3World* world, m3JointId joint);

    /// The current strut twist in radians (wheel joints; 0 for
    /// every other type, which counts one misuse, and stale ids).
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The twist, radians; 0 for a null world, and for a stale or null
    ///         id or another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3WheelJoint_GetSteeringAngle(const m3World* world, m3JointId joint);

    /// Let (or forbid) the two jointed bodies collide with each
    /// other. Journaled; binds at the next step's pair scan.
    /// A null world does nothing. A stale or null id, or a call from host code
    /// the world is running, changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @param flag   Whether the two bodies collide.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_EnableConnectedCollision(m3World* world, m3JointId joint, bool flag);

    /// Returns whether the two jointed bodies collide.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return Whether they do; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Joint_IsConnectedCollisionEnabled(const m3World* world, m3JointId joint);

    /// Breakage: rollback games need breakage as a deterministic in-step
    /// state transition, not a host poll racing the journal. When
    /// either reaction magnitude exceeds its cap at the end of a
    /// step, the joint destroys itself and emits the joint break
    /// event (see world.h). Zero disables a cap; both zero (the
    /// default) means unbreakable.
    /// Journaled.
    /// A null world does nothing. A stale or null id, a cap that is negative or
    /// not finite, or a call from host code the world is running changes
    /// nothing, is not journaled and counts one misuse.
    ///
    /// @param world      The world.
    /// @param joint      The joint.
    /// @param maxForce   The force cap, N, 0 for none.
    /// @param maxTorque  The torque cap, N m, 0 for none.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_SetBreakLimits(m3World* world, m3JointId joint, float maxForce,
                                       float maxTorque);

    /// The break limits m3Joint_SetBreakLimits sets, 0 for a cap that is
    /// off. A stale id reads zeros and counts one misuse; a null world
    /// reads zeros. Either output may be null.
    ///
    /// @param world         The world.
    /// @param joint         The joint.
    /// @param maxForceOut   Receives the force cap.
    /// @param maxTorqueOut  Receives the torque cap.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API void m3Joint_GetBreakLimits(const m3World* world, m3JointId joint, float* maxForceOut,
                                       float* maxTorqueOut);

    /// Reaction readback: MAGNITUDES of the last step's
    /// constraint reactions, assembled per type from the stored
    /// solver rows (linear rows into force, angular rows into
    /// torque; the generic joint reports a conservative sum). Reads
    /// 0 before the first step after a restore (documented
    /// transient).
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The force magnitude, N; 0 for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Joint_GetReactionForce(const m3World* world, m3JointId joint);

    /// The torque magnitude of the last step's reaction; see above.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The torque magnitude, N m; 0 for a null world, and for a stale
    ///         or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Joint_GetReactionTorque(const m3World* world, m3JointId joint);

    /// Geometry reads: the angle about the hinge of a revolute joint,
    /// or a wheel joint's spin about its axle (radians), and the
    /// translation along a prismatic joint's slide axis or a wheel
    /// joint's suspension axis (meters). Any other type, or a stale
    /// id, reads 0 and counts one misuse.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The angle, radians; 0 for a null world, and for a stale or null
    ///         id or a type without an angle, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Joint_GetAngle(const m3World* world, m3JointId joint);

    /// The translation along the slide or suspension axis; see above.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The translation, meters; 0 for a null world, and for a stale or
    ///         null id or a type without one, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Joint_GetTranslation(const m3World* world, m3JointId joint);

    /// Position drive: a soft constraint with the given frequency and
    /// damping ratio pulls the joint toward its target. Revolute drives
    /// the hinge angle, prismatic and wheel the translation, spherical the
    /// relative rotation, the motor joint its aim, and the distance joint
    /// the anchor distance toward its length; other types refuse.
    /// The frequency is in [0, 1e5] Hz and positive while the spring is
    /// on, the damping ratio in [0, 1e4]. The spring keeps its values
    /// while it is off.
    /// A null world does nothing. A stale or null id, a joint that is not
    /// revolute, prismatic, spherical, wheel, motor or distance, switching the
    /// spring on while its hertz is zero, or a call from host code the world is
    /// running changes nothing, is not journaled and counts one misuse.
    /// Spherical, revolute, prismatic, wheel, motor and distance joints have a
    /// spring. Both bodies wake, and the stored impulse of the rows it changes
    /// is zeroed.
    ///
    /// @param world   The world.
    /// @param joint   The joint.
    /// @param enable  Whether the spring pulls.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_EnableSpring(m3World* world, m3JointId joint, bool enable);

    /// Returns whether the spring is on.
    /// Spherical, revolute, prismatic, wheel, motor and distance joints have a
    /// spring.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return Whether it is; false, for a type without this part, for a null
    ///         world, and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Joint_IsSpringEnabled(const m3World* world, m3JointId joint);

    /// Sets the spring's frequency. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not
    /// revolute, prismatic, spherical, wheel, motor or distance, a hertz that
    /// is negative, NaN or past 1e5, infinities included, a zero hertz while
    /// the spring is on, or a call from host code the world is running changes
    /// nothing, is not journaled and counts one misuse.
    /// Spherical, revolute, prismatic, wheel, motor and distance joints have a
    /// spring. Both bodies wake, and the stored impulse of the rows it changes
    /// is zeroed.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @param hertz  The frequency, in [0, 1e5], positive while the spring is
    ///               on.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_SetSpringHertz(m3World* world, m3JointId joint, float hertz);

    /// Returns the spring's frequency.
    /// Spherical, revolute, prismatic, wheel, motor and distance joints have a
    /// spring.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The frequency; 0, for a type without this part, for a null
    ///         world, and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Joint_GetSpringHertz(const m3World* world, m3JointId joint);

    /// Sets the spring's damping ratio. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not
    /// revolute, prismatic, spherical, wheel, motor or distance, a ratio that
    /// is negative, NaN or past 1e4, infinities included, or a call from host
    /// code the world is running changes nothing, is not journaled and counts
    /// one misuse.
    /// Spherical, revolute, prismatic, wheel, motor and distance joints have a
    /// spring. Both bodies wake, and the stored impulse of the rows it changes
    /// is zeroed.
    ///
    /// @param world         The world.
    /// @param joint         The joint.
    /// @param dampingRatio  The ratio, in [0, 1e4].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_SetSpringDampingRatio(m3World* world, m3JointId joint, float dampingRatio);

    /// Returns the spring's damping ratio.
    /// Spherical, revolute, prismatic, wheel, motor and distance joints have a
    /// spring.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The ratio; 0, for a type without this part, for a null world,
    ///         and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Joint_GetSpringDampingRatio(const m3World* world, m3JointId joint);

    /// Sets the revolute spring's target angle. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// revolute joint, an angle that is not finite or outside [-pi, pi], or a
    /// call from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    ///
    /// @param world    The world.
    /// @param joint    The joint.
    /// @param radians  The target hinge angle, in [-pi, pi].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3RevoluteJoint_SetTargetAngle(m3World* world, m3JointId joint, float radians);

    /// Sets the prismatic or wheel spring's target translation.
    /// Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// prismatic or wheel joint, a translation that is not finite, or a call
    /// from host code the world is running changes nothing, is not journaled
    /// and counts one misuse.
    ///
    /// @param world   The world.
    /// @param joint   The joint.
    /// @param meters  The target translation, finite.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Joint_SetTargetTranslation(m3World* world, m3JointId joint, float meters);

    /// The target must be a unit rotation; garbage refuses loudly by
    /// doing nothing. The target lives in the JOINT FRAMES (frame z
    /// is the create-time local axis): it is the desired rotation of
    /// frame B relative to frame A. Pick
    /// your axes at create time so the frame reads naturally.
    /// Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// spherical joint, a rotation that is not finite or whose squared length
    /// is outside [0.99, 1.01], or a call from host code the world is running
    /// changes nothing, is not journaled and counts one misuse.
    ///
    /// @param world   The world.
    /// @param joint   The joint.
    /// @param target  B's target rotation relative to A, in the joint frames, a
    ///                quaternion whose squared length is within 1 percent of 1.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3SphericalJoint_SetTargetRotation(m3World* world, m3JointId joint, m3Quat target);

    /// The swing cone (m3SphericalJointDef enableCone and coneAngle), switched and
    /// moved at run time; each takes effect in its own call. The angle is
    /// finite and not negative, and EnableCone refuses to switch the cone
    /// on over a negative angle a def left. Any other joint type, a stale
    /// id or a refused value changes nothing and counts one misuse; the
    /// getters then read false and zero.
    /// Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// spherical joint, switching the cone on while its angle is negative, or a
    /// call from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    ///
    /// @param world   The world.
    /// @param joint   The joint.
    /// @param enable  Whether the cone holds.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3SphericalJoint_EnableCone(m3World* world, m3JointId joint, bool enable);

    /// Returns whether the cone holds.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return Whether it does; false for a null world, and for a stale or null
    ///         id or another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3SphericalJoint_IsConeEnabled(const m3World* world, m3JointId joint);

    /// Sets the cone angle. Journaled.
    /// A null world does nothing. A stale or null id, a joint that is not a
    /// spherical joint, an angle that is negative or not finite (refused with
    /// the cone off too), or a call from host code the world is running changes
    /// nothing, is not journaled and counts one misuse.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @param angle  The angle from A's frame z, radians, finite and not
    ///               negative.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3SphericalJoint_SetConeAngle(m3World* world, m3JointId joint, float angle);

    /// Returns the cone angle.
    ///
    /// @param world  The world.
    /// @param joint  The joint.
    /// @return The angle; 0 for a null world, and for a stale or null id or
    ///         another joint type, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3SphericalJoint_GetConeAngle(const m3World* world, m3JointId joint);

#ifdef __cplusplus
}
#endif

#endif
