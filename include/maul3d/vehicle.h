// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Raycast vehicles: each wheel is a suspension ray from the chassis
// with a spring and damper at the contact and tire friction in the
// contact plane. Vehicles are stepped inside m3World_Step in slot order
// and fully snapshotted, so a replayed or rolled-back drive lands on
// identical bits.

#ifndef MAUL3D_VEHICLE_H
#define MAUL3D_VEHICLE_H

#include "world.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define M3_MAX_VEHICLE_WHEELS 8

    typedef struct m3VehicleId
    {
        uint32_t index1;
        uint32_t generation;
    } m3VehicleId;

    typedef struct m3WheelDef
    {
        m3Vec3 anchor;            // suspension attach point, chassis-local
        m3Vec3 direction;         // suspension direction, chassis-local, near
                                  // unit (normalized at create); usually down
        float restLength;         // suspension length at zero load
        float travel;             // max compression from rest
        float springHertz;        // spring frequency; keep well under the
                                  // step rate over two pi (explicit spring)
        float springDampingRatio; // damping ratio
        float radius;             // wheel radius (the cast reaches
                                  // restLength + radius along direction)
        bool steerable;           // turns with the steer command
        bool driven;              // takes the drive force or engine torque
        float brakeShare;         // this wheel's share of brakeForce
    } m3WheelDef;

    typedef struct m3VehicleDef
    {
        m3BodyId chassisId; // a dynamic body; destroying it destroys
                            // the vehicle (the cascade rule)
        int32_t wheelCount;
        m3WheelDef wheels[M3_MAX_VEHICLE_WHEELS];
        float maxSteerAngle;     // radians; full steer command turns
                                 // every steerable wheel this far
        float driveForce;        // newtons per driven wheel at full
                                 // throttle (the flat force model)
        float brakeForce;        // newtons total at full brake, split
                                 // by each wheel's brakeShare
        float tireGrip;          // friction circle scale: tangent
                                 // impulses never exceed grip times
                                 // the wheel's suspension load
        float leanStabilization; // 0 disables (the default). A
                                 // positive gain rolls the chassis
                                 // toward the lean angle the current
                                 // speed and steer imply, critically
                                 // damped; the natural frequency is
                                 // sqrt(gain) rad/s. Meant for
                                 // two-wheelers, which are inverted
                                 // pendulums without it.
        M3_ALIGN8 uint64_t userData;
        int32_t internalValue;
    } m3VehicleDef;

    /// Returns a vehicle def with the defaults: no chassis and no wheels
    /// yet, every wheel slot hanging down 0.4 m with 0.25 m of travel at
    /// 1.5 Hz and a 0.6 damping ratio, a 0.3 m radius and a 0.25 brake
    /// share; a 0.6 rad steer, 800 N of drive, 1600 N of brake, a grip of
    /// 1.5 and no lean stabilization.
    ///
    /// @return The def, with the cookie m3CreateVehicle checks.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3VehicleDef m3DefaultVehicleDef(void);

    /// Creates the vehicle bound to its chassis body. Refuses (null
    /// id) a non-dynamic or stale chassis, a wheel count outside
    /// [1, M3_MAX_VEHICLE_WHEELS], or any non-finite or non-positive
    /// geometry. Each wheel's anchor, rest length and radius lie within
    /// 1e4 m, its radius at least 1e-3 m, its hertz within 1e5 and its
    /// damping ratio and brake share within 1e4; drive and brake force
    /// within 1e20 N, tire grip and lean stabilization within 1e4.
    /// A value past one of these bounds, or a wheel count past
    /// M3_MAX_VEHICLE_WHEELS, returns m3_errorLimit; any other bad
    /// field m3_errorInvalid. Journaled with id verification.
    ///
    /// @param world       The world.
    /// @param def         The def, from m3DefaultVehicleDef.
    /// @param vehicleOut  Receives the new id; any failure leaves the null
    ///                    id there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or
    ///         vehicleOut, a def m3DefaultVehicleDef did not make, a chassis
    ///         that is stale, null or not dynamic, a wheel count below 1, a
    ///         steer angle that is not finite or is negative, a drive force,
    ///         brake force, tire grip or lean stabilization that is negative
    ///         or NaN, or a wheel whose direction is not finite or not within
    ///         10 percent of unit length, whose travel is not positive or
    ///         exceeds its rest length, whose anchor is NaN, whose rest
    ///         length, hertz or radius is not positive or is NaN, or whose
    ///         damping ratio or brake share is negative or NaN; m3_errorLimit
    ///         for a wheel count past M3_MAX_VEHICLE_WHEELS, a drive or brake
    ///         force past 1e20 N, a tire grip or lean stabilization past 1e4,
    ///         or a wheel anchor, rest length or radius past 1e4 m, a radius
    ///         below 1e-3 m, a hertz past 1e5 or a damping ratio or brake share
    ///         past 1e4, infinities included; m3_errorCapacity when the vehicle
    ///         pool is full; m3_errorBusy from host code the world is running.
    ///         Every refusal but a full pool's and a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateVehicle(m3World* world, const m3VehicleDef* def,
                                                 m3VehicleId* vehicleOut);

    /// Destroys a vehicle; its chassis body stays. Journaled.
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle.
    /// @return m3_success; m3_errorInvalid for a null world or the null
    ///         id; m3_errorStale for an id whose vehicle is gone;
    ///         m3_errorBusy from host code the world is running. Each
    ///         refusal but a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3DestroyVehicle(m3World* world, m3VehicleId vehicle);

    /// Returns whether the id names a live vehicle of this world.
    ///
    /// @param world    The world; null gives false.
    /// @param vehicle  The id.
    /// @return Whether the vehicle is alive.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Vehicle_IsValid(const m3World* world, m3VehicleId vehicle);

    /// Last step's suspension compression of one wheel, in meters
    /// from rest length (zero when airborne, stale, or out of
    /// range). Frozen while the chassis sleeps, like the chassis.
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle.
    /// @param wheel    The wheel index, 0 to the wheel count - 1.
    /// @return The compression in meters; 0 while airborne, for a null
    ///         world, and for a stale or null id or an out-of-range wheel,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Vehicle_GetWheelCompression(const m3World* world, m3VehicleId vehicle,
                                               int32_t wheel);

    /// Whether the wheel's suspension cast found ground last step.
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle.
    /// @param wheel    The wheel index, 0 to the wheel count - 1.
    /// @return Whether it touched ground; false for a null world, and for
    ///         a stale or null id or an out-of-range wheel, which counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Vehicle_IsWheelGrounded(const m3World* world, m3VehicleId vehicle, int32_t wheel);

    /// Drive commands: journaled STATE, not per-step
    /// parameters, so replay and rollback hold to the bit. Values
    /// clamp to their ranges (throttle and steer to [-1, 1], brake
    /// to [0, 1]); non-finite commands are hostile no-ops that
    /// never journal. Setting commands wakes the chassis. The
    /// chassis-local +x axis is the vehicle's forward by
    /// convention; steer rotates each steerable wheel's frame
    /// about its suspension axis by steer * maxSteerAngle.
    /// Tank commands: per-side throttle for skid steering.
    /// Driven wheels split by the sign of their chassis-local
    /// anchor z (+z right); each side takes its own throttle in
    /// [-1, 1], the brake rides all wheels. Engaging tank mode
    /// suspends the normal throttle until m3Vehicle_SetCommands is
    /// called again (which disengages it). Drivetrain vehicles
    /// refuse: a gearbox and a skid steer are different machines.
    ///
    /// A refused call changes nothing, is not journaled and counts one
    /// misuse: a stale or null id, a non-finite command, a drivetrain
    /// vehicle, or a call from host code the world is running.
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle.
    /// @param left     The left side's throttle, clamped to [-1, 1].
    /// @param right    The right side's throttle, clamped to [-1, 1].
    /// @param brake    The brake, clamped to [0, 1].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Vehicle_SetTankCommands(m3World* world, m3VehicleId vehicle, float left,
                                          float right, float brake);

    /// Sets the drive commands, as described above, and leaves tank mode.
    /// A refused call changes nothing, is not journaled and counts one
    /// misuse: a stale or null id, a non-finite command, or a call from
    /// host code the world is running.
    ///
    /// @param world     The world.
    /// @param vehicle   The vehicle.
    /// @param throttle  The throttle, clamped to [-1, 1].
    /// @param steer     The steer, clamped to [-1, 1].
    /// @param brake     The brake, clamped to [0, 1].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Vehicle_SetCommands(m3World* world, m3VehicleId vehicle, float throttle,
                                      float steer, float brake);

    /// Accumulated spin angle of one wheel in radians (rendering
    /// state: hosts spin their wheel meshes with it).
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle.
    /// @param wheel    The wheel index, 0 to the wheel count - 1.
    /// @return The spin angle in radians; 0 for a null world, and for a
    ///         stale or null id or an out-of-range wheel, which counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Vehicle_GetWheelSpin(const m3World* world, m3VehicleId vehicle, int32_t wheel);

#define M3_MAX_DRIVETRAIN_CURVE_POINTS 8
#define M3_MAX_DRIVETRAIN_GEARS        6
#define M3_MAX_CLUTCH_STEPS            600

    /// The drivetrain: the flat force model grows an engine.
    /// Torque comes from a pinned curve (control points, linear
    /// interpolation between them: deterministic on every platform
    /// because only +,-,*,/ touch the numbers), multiplied through
    /// the active gear and the final drive, split equally across the
    /// driven wheels (an open differential), and handed to the same
    /// friction circle the flat model uses. The first control point
    /// is the idle line: engine speed never reads below it, so a car
    /// at standstill still has launch torque. Past the last point
    /// the curve extends flat.
    ///
    /// How a drivetrain's differential splits the drive between its
    /// driven wheels.
    typedef int32_t m3DifferentialMode;
    enum
    {
        m3_differentialOpen = 0,        // the equal split
        m3_differentialLimitedSlip = 1, // coupled, clamped to the engine's own force
        m3_differentialLocked = 2,      // coupled without that clamp
    };

    /// With a drivetrain attached, throttle drives only forward
    /// gears; reverse is gear -1 (throttle magnitude, direction from
    /// the gear), gear 0 is neutral. Auto shift manages forward
    /// gears only and waits for the clutch to close between shifts.
    typedef struct m3DrivetrainDef
    {
        int32_t curveCount;                                // 2..8 control points
        float curveRpm[M3_MAX_DRIVETRAIN_CURVE_POINTS];    // strictly ascending,
                                                           // nonnegative
        float curveTorque[M3_MAX_DRIVETRAIN_CURVE_POINTS]; // newton meters at
                                                           // the crank
        int32_t gearCount;                                 // 1..6 forward gears
        float gearRatio[M3_MAX_DRIVETRAIN_GEARS];          // crank turns per
                                                           // wheel turn
        float reverseRatio;                                // positive magnitude; 0 forbids reverse
        float finalDrive;                                  // multiplies every gear
        float shiftUpRpm;                                  // auto shift climbs above this
        float shiftDownRpm;                                // auto shift drops below this
        int32_t clutchSteps; // steps of torque cut per shift, 0..M3_MAX_CLUTCH_STEPS
        bool enableAutoShift;
        /// Differential: m3_differentialOpen (the equal split),
        /// m3_differentialLimitedSlip or m3_differentialLocked. The
        /// other two couple each driven wheel toward the driven mean of
        /// the LAST step's contact speeds with differentialCoupling
        /// newtons per m/s of disparity, at most 1e15; limited slip
        /// additionally clamps the coupling to the engine's own
        /// per-wheel force. One step of lag is the drivetrain's own
        /// convention (engine speed reads wheels the same way).
        m3DifferentialMode differentialMode;
        float differentialCoupling;
        int32_t internalValue;
    } m3DrivetrainDef;

    /// Returns a drivetrain def for a plain small car: a four-point curve
    /// from 120 N m at 1000 rpm to a 200 N m peak at 3000 rpm, five gears
    /// from 3.5 to 0.9, a 3.2 reverse, a 3.7 final drive, automatic shifts
    /// at 5500 and 2000 rpm with 12 clutch steps, and an open
    /// differential.
    ///
    /// @return The def, with the cookie m3Vehicle_SetDrivetrain checks.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3DrivetrainDef m3DefaultDrivetrainDef(void);

    /// Attach a drivetrain (journaled). Replaces the flat driveForce
    /// model for this vehicle; starts in first gear, clutch closed.
    /// Invalid defs are documented no-ops: a def m3DefaultDrivetrainDef
    /// did not make, a curve count outside 2 to M3_MAX_DRIVETRAIN_CURVE_POINTS,
    /// a gear count outside 1 to M3_MAX_DRIVETRAIN_GEARS, clutch steps
    /// outside 0 to M3_MAX_CLUTCH_STEPS, a curve rpm that is not finite,
    /// is negative or does not ascend strictly, a torque that is negative,
    /// NaN or past 1e20 N m, a gear ratio or final drive that is not
    /// positive, is NaN or is past 1e4, a reverse ratio that is negative,
    /// NaN or past 1e4, a shift rpm that is not finite, a negative shift-down rpm or a
    /// shift-up rpm not above it, a differential mode out of range, or a
    /// coupling that is negative, NaN or past 1e15.
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle; a stale or null id is a no-op that
    ///                 counts one misuse.
    /// @param def      The def, from m3DefaultDrivetrainDef. A null or
    ///                 invalid def is a no-op that is not journaled and
    ///                 counts one misuse, as is a call from host code the
    ///                 world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Vehicle_SetDrivetrain(m3World* world, m3VehicleId vehicle,
                                        const m3DrivetrainDef* def);

    /// Select a gear (journaled): -1 reverse, 0 neutral, 1..gearCount.
    /// Opens the clutch for clutchSteps. A no-op without a drivetrain,
    /// out of range, or when reverseRatio is zero and gear is -1.
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle; a stale or null id is a no-op.
    /// @param gear     The gear: -1 reverse, 0 neutral, 1 to gearCount.
    ///                 Each no-op is not journaled and counts one misuse,
    ///                 as does a call from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Vehicle_SetGear(m3World* world, m3VehicleId vehicle, int32_t gear);

    /// Current gear; 0 when no drivetrain is attached.
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle.
    /// @return The gear; 0 without a drivetrain, for a null world, and for
    ///         a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API int32_t m3Vehicle_GetGear(const m3World* world, m3VehicleId vehicle);

    /// Engine speed computed by the last step, idle-floored like the
    /// torque lookup (a tachometer, not raw wheel math).
    ///
    /// @param world    The world.
    /// @param vehicle  The vehicle.
    /// @return The engine speed in rpm; 0 without a drivetrain, for a null
    ///         world, and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Vehicle_GetEngineRpm(const m3World* world, m3VehicleId vehicle);

    static const m3VehicleId m3_nullVehicleId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_VEHICLE_H
