// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Rigid bodies: creation and destruction, motion, forces and
// impulses, mass, sleep, and state readback.

#ifndef MAUL3D_BODY_H
#define MAUL3D_BODY_H

#include "maul3d/world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef int32_t m3BodyType;
    enum
    {
        m3_staticBody = 0,
        m3_kinematicBody = 1, // moved by velocity, immovable by contact
        m3_dynamicBody = 2,
    };

    /// Per-axis motion locks. A locked axis holds still: its velocity is
    /// zeroed every substep, so contacts cannot bank motion on it.
    typedef struct m3MotionLocks
    {
        bool linearX;
        bool linearY;
        bool linearZ;
        bool angularX;
        bool angularY;
        bool angularZ;
    } m3MotionLocks;

    typedef struct m3BodyDef
    {
        m3BodyType type;
        m3Pos3 position;        // world space, double, within 1e15 m
        m3Quat rotation;        // unit quaternion
        m3Vec3 linearVelocity;  // m/s, within 1e5 (kinematic: 1e10; static: zero)
        m3Vec3 angularVelocity; // rad/s, world frame, within 1e5 (static: zero)
        float gravityScale;     // within 1e4
        float linearDamping;
        float angularDamping;
        M3_ALIGN8 uint64_t userData; // opaque, carried unchanged
        /// A high speed body that gets the full continuous pass
        /// against static AND dynamic targets. Every fast
        /// dynamic body already sweeps against statics; the bullet
        /// flag buys the dynamic-target sweep. Bullet versus bullet
        /// is not resolved (a documented limitation).
        bool enableBullet;
        m3MotionLocks motionLocks;
        bool enableSleep;        // false = this body never sleeps
        float sleepThreshold;    // resting speed; zero = the world default
        bool enable;             // false = created outside simulation and queries
        bool enableFastRotation; // may spin past the world's angular cap, up to 1e5 rad/s
        int32_t internalValue;
    } m3BodyDef;

    /// Returns a def with pinned defaults (identity rotation, gravity
    /// scale one, sleep and simulation enabled) and a valid cookie.
    ///
    /// @return A static body def at the origin, with the cookie m3CreateBody
    ///         checks.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3BodyDef m3DefaultBodyDef(void);

    /// Create a body. Returns m3_errorInvalid for an invalid def,
    /// m3_errorLimit for a position, velocity, spin or gravity scale
    /// past its bound in maul3d/base.h, and m3_errorCapacity for an
    /// exhausted body pool; *bodyOut is the null
    /// id on any failure. A shapeless dynamic body has unit mass and zero
    /// inertia until a shape provides the real values. Journaled.
    ///
    /// @param world    The world.
    /// @param def      The def, from m3DefaultBodyDef.
    /// @param bodyOut  Receives the new id; any failure leaves the null id
    ///                 there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or bodyOut, a
    ///         def m3DefaultBodyDef did not make, a type out of range, a
    ///         rotation that is not finite or not near unit, a damping that is
    ///         not finite or is negative, a sleep threshold that is not finite
    ///         or is negative, a static def whose velocity or spin is not
    ///         zero, infinities included, or a NaN position, velocity, spin
    ///         or gravity scale; m3_errorLimit for a position, velocity, spin
    ///         or gravity scale past its bound, infinities included; m3_errorCapacity for
    ///         a full body pool; m3_errorBusy from host code the world is
    ///         running. Every refusal but a full pool's and a null world's
    ///         counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateBody(m3World* world, const m3BodyDef* def,
                                              m3BodyId* bodyOut);

    /// Destroy a body. The id goes stale; the slot recycles FIFO with a
    /// generation bump and retires after 2^32 objects instead of
    /// wrapping. A stale id returns m3_errorStale and the null id
    /// m3_errorInvalid; every destroy of an object a world owns answers
    /// the same way. The body's shapes and joints go with it, and a
    /// vehicle whose chassis it is. Journaled.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return m3_success; m3_errorInvalid for a null world, the null id or a
    ///         character's body; m3_errorStale for a gone body; m3_errorBusy
    ///         from host code the world is running. Each refusal but a null
    ///         world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3DestroyBody(m3World* world, m3BodyId body);

    /// Returns whether the id names a live body of this world.
    ///
    /// @param world  The world; null gives false.
    /// @param body   The id.
    /// @return Whether the body is alive.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Body_IsValid(const m3World* world, m3BodyId body);

    /// Returns the body origin's world position.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The position; the origin for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Pos3 m3Body_GetPosition(const m3World* world, m3BodyId body);

    /// Returns the body's rotation.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The rotation; identity for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Quat m3Body_GetRotation(const m3World* world, m3BodyId body);

    /// Returns the velocity of the body's center of mass, m/s.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The velocity; zero for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Body_GetLinearVelocity(const m3World* world, m3BodyId body);

    /// Returns the body's angular velocity in the world frame, rad/s.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The angular velocity; zero for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Body_GetAngularVelocity(const m3World* world, m3BodyId body);

    /// Returns the host's value the def or m3Body_SetUserData set.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The value; 0 for a null world, and for a stale or null id, which
    ///         counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API uint64_t m3Body_GetUserData(const m3World* world, m3BodyId body);

    /// Returns the body's type.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The type; m3_staticBody for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3BodyType m3Body_GetType(const m3World* world, m3BodyId body);

    // Readback, as in Maul2D. Mass and inertia are about the center of
    // mass, the inertia tensor in the body frame; points and vectors
    // convert between the body frame and world space.

    /// Returns the body's position and rotation together.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The transform; identity at the origin for a null world, and for
    ///         a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Transform m3Body_GetTransform(const m3World* world, m3BodyId body);

    /// Returns the body's mass in kilograms.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The mass; 0 for a static or kinematic body, for a null world,
    ///         and for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Body_GetMass(const m3World* world, m3BodyId body);

    /// Returns the inertia tensor about the center of mass, in the body
    /// frame, kg m^2.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The tensor; zero for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Mat3 m3Body_GetCentralInertia(const m3World* world, m3BodyId body);

    /// Returns the center of mass in the body frame.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The center; zero for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Body_GetLocalCenter(const m3World* world, m3BodyId body);

    /// Returns the center of mass in world space.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The center; the origin for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Pos3 m3Body_GetWorldCenterOfMass(const m3World* world, m3BodyId body);

    /// Converts a point from the body frame to world space.
    ///
    /// @param world       The world.
    /// @param body        The body.
    /// @param localPoint  The point in the body frame.
    /// @return The world point; the origin for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Pos3 m3Body_GetWorldPoint(const m3World* world, m3BodyId body, m3Vec3 localPoint);

    /// Converts a world point to the body frame.
    ///
    /// @param world       The world.
    /// @param body        The body.
    /// @param worldPoint  The point in world space.
    /// @return The body-frame point; zero for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Body_GetLocalPoint(const m3World* world, m3BodyId body, m3Pos3 worldPoint);

    /// Rotates a vector from the body frame to world space.
    ///
    /// @param world        The world.
    /// @param body         The body.
    /// @param localVector  The vector in the body frame.
    /// @return The world vector; zero for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Body_GetWorldVector(const m3World* world, m3BodyId body, m3Vec3 localVector);

    /// Rotates a world vector into the body frame.
    ///
    /// @param world        The world.
    /// @param body         The body.
    /// @param worldVector  The vector in world space.
    /// @return The body-frame vector; zero for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Body_GetLocalVector(const m3World* world, m3BodyId body, m3Vec3 worldVector);

    /// Returns the velocity of the body's material at a world point.
    ///
    /// @param world       The world.
    /// @param body        The body.
    /// @param worldPoint  The point in world space.
    /// @return The velocity, m/s; zero for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Body_GetWorldPointVelocity(const m3World* world, m3BodyId body,
                                               m3Pos3 worldPoint);

    /// Returns the velocity of the body's material at a body-frame point.
    ///
    /// @param world       The world.
    /// @param body        The body.
    /// @param localPoint  The point in the body frame.
    /// @return The velocity, m/s; zero for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Body_GetLocalPointVelocity(const m3World* world, m3BodyId body,
                                               m3Vec3 localPoint);

    /// Returns the body's gravity scale.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The scale; 0 for a null world, and for a stale or null id, which
    ///         counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Body_GetGravityScale(const m3World* world, m3BodyId body);

    /// Returns the body's linear damping.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The damping; 0 for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Body_GetLinearDamping(const m3World* world, m3BodyId body);

    /// Returns the body's angular damping.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The damping; 0 for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Body_GetAngularDamping(const m3World* world, m3BodyId body);

    /// Returns whether the body is a bullet.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return Whether it is; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Body_IsBulletEnabled(const m3World* world, m3BodyId body);
    // Journaled setters for what the def set, as in Maul2D.

    /// Sets the body's gravity scale. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param scale  The scale, finite and within 1e4.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetGravityScale(m3World* world, m3BodyId body, float scale);

    /// Sets the body's linear damping. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world    The world.
    /// @param body     The body.
    /// @param damping  The damping, finite and not negative.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetLinearDamping(m3World* world, m3BodyId body, float damping);

    /// Sets the body's angular damping. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world    The world.
    /// @param body     The body.
    /// @param damping  The damping, finite and not negative.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetAngularDamping(m3World* world, m3BodyId body, float damping);

    /// Turns the bullet flag on or off; see m3BodyDef.enableBullet.
    /// Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param flag   Whether the body is a bullet.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_EnableBullet(m3World* world, m3BodyId body, bool flag);

    /// Sets the host's value, carried unchanged. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param userData  The value.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetUserData(m3World* world, m3BodyId body, uint64_t userData);

    /// Returns the union of the shapes' broadphase bounds, margin
    /// included, or the body origin for a shapeless body.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The bounds; a box at the origin for a null world, and for a
    ///         stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3WorldAabb m3Body_ComputeAabb(const m3World* world, m3BodyId body);

    /// The body's shapes or joints, up to capacity ids; returns the
    /// total, which may exceed capacity, and a NULL array with capacity
    /// 0 measures. A stale body is a refusal. A refusal zeroes the array over its whole capacity; a
    /// success, an empty one included, writes only its entries; a list call only reads, so none is
    /// refused as busy.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param idsOut    Receives up to capacity shape ids, in ascending slot
    ///                  order.
    /// @param capacity  idsOut's length, at least 0.
    /// @return The total number of the body's shapes; 0 on a refusal: a null
    ///         world, a stale or null id, a negative capacity or a null idsOut
    ///         with a positive one, each counting one misuse but a null
    ///         world's.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API int32_t m3Body_GetShapes(const m3World* world, m3BodyId body, m3ShapeId* idsOut,
                                    int32_t capacity);

    /// The body's joints, as m3Body_GetShapes lists its shapes.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param idsOut    Receives up to capacity joint ids, in ascending slot
    ///                  order.
    /// @param capacity  idsOut's length, at least 0.
    /// @return The total number of the body's joints; 0 on a refusal, as
    ///         m3Body_GetShapes.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API int32_t m3Body_GetJoints(const m3World* world, m3BodyId body, m3JointId* idsOut,
                                    int32_t capacity);

    /// Journaled setters: every mutation is a discrete op.
    /// Runtime control, all journaled. SetTransform is the
    /// teleport: the pose lands instantly, velocities stay, and
    /// bodies around BOTH the old and new locations wake so
    /// nothing keeps sleeping under or inside a teleported crate. The
    /// position must lie within 1e15 m of the origin on every axis.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param position  The new origin, within 1e15 m on every axis.
    /// @param rotation  The new rotation, a unit quaternion.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetTransform(m3World* world, m3BodyId body, m3Pos3 position,
                                    m3Quat rotation);

    /// Kinematic servo: velocities are chosen at the next step so
    /// the body lands ON the target pose after that step, then the
    /// target clears. Kinematic bodies only (the correct way to
    /// drive elevators and doors). Journaled. Returns m3_success;
    /// m3_errorLimit for a target more than M3_TARGET_REACH_LIMIT from
    /// where the body is, which the step decides, so it changes nothing
    /// and counts no misuse; m3_errorStale for a stale id,
    /// m3_errorInvalid for the null id, a body that is not kinematic or
    /// a pose that is not finite, normalized and within 1e15 m, and
    /// m3_errorBusy inside a step callback, which change nothing and
    /// count one misuse.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param position  The target origin.
    /// @param rotation  The target rotation, a unit quaternion.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3Body_SetTargetTransform(m3World* world, m3BodyId body,
                                                           m3Pos3 position, m3Quat rotation);

    /// Switch dynamic, kinematic, static at runtime. Mass rebuilds
    /// from shapes when turning dynamic; velocities zero when
    /// turning static; the neighborhood wakes. Journaled. Returns
    /// m3_success; m3_errorLimit when a joint on the body would pass
    /// its lever ratio at the new type's mass, when the body would turn
    /// dynamic with a shape below the mass floors or a total mass past
    /// M3_MASS_LIMIT, or when its linear velocity is past the new
    /// type's limit, which change nothing and count no misuse;
    /// m3_errorStale for a stale id, m3_errorInvalid for the null id, a
    /// type out of range, a character's body, or a body holding a plane,
    /// mesh, height field or voxel chunk that would leave static, and
    /// m3_errorBusy inside a step callback, which change nothing and
    /// count one misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param type   The new type.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3Body_SetType(m3World* world, m3BodyId body, m3BodyType type);

    /// A disabled body vanishes from simulation AND queries without
    /// being destroyed; enabling wakes its neighborhood. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world   The world.
    /// @param body    The body.
    /// @param enable  Whether the body takes part.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_Enable(m3World* world, m3BodyId body, bool enable);

    /// Returns whether the body takes part in simulation and queries.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return Whether it does; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Body_IsEnabled(const m3World* world, m3BodyId body);

    /// Motion locks: locked components re-zero every substep, so 2.5D
    /// scenes and upright enemies stay exact. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param locks  The axes to hold still.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetMotionLocks(m3World* world, m3BodyId body, m3MotionLocks locks);

    /// Returns the body's motion locks.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The locks; none for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3MotionLocks m3Body_GetMotionLocks(const m3World* world, m3BodyId body);

    /// Let this body spin past the world's angular speed cap (for
    /// wheels and other legal fast spinners), up to the 1e5 rad/s the
    /// step holds every body within. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param flag   Whether the body may spin past the world's cap.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_EnableFastRotation(m3World* world, m3BodyId body, bool flag);

    /// Returns whether the body may spin past the world's angular cap.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return Whether it may; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Body_IsFastRotationEnabled(const m3World* world, m3BodyId body);

    /// A body's debug name: UTF-8 of at most M3_MAX_BODY_NAME_BYTES bytes,
    /// with no zero byte. Journaled and carried by snapshots, never part
    /// of the hash (a label moves no matter). A body starts unnamed, and
    /// a byteCount of 0 clears the name. name may be NULL when byteCount
    /// is 0.
    /// Returns m3_success; m3_errorLimit for a byteCount past
    /// M3_MAX_BODY_NAME_BYTES; m3_errorStale for a stale id;
    /// m3_errorInvalid for a NULL world, the null id, a negative
    /// byteCount, a NULL name with a positive byteCount, or bytes that
    /// are not valid UTF-8 or hold a zero byte; m3_errorBusy inside a step
    /// callback. Every refusal changes nothing and counts one misuse.
    ///
    /// @param world      The world.
    /// @param body       The body.
    /// @param name       The name's bytes, no terminator needed.
    /// @param byteCount  The name's length in bytes, 0 to
    ///                   M3_MAX_BODY_NAME_BYTES.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3Body_SetName(m3World* world, m3BodyId body, const char* name,
                                                int32_t byteCount);

    /// Copies a body's name into nameOut, with no terminator, and returns
    /// its byte length, 0 for an unnamed body. The name is written only
    /// when it fits whole: a capacity below its length zeroes nameOut
    /// over its capacity and still returns the length, so a NULL nameOut
    /// with capacity 0 measures, and a capacity of M3_MAX_BODY_NAME_BYTES
    /// always fits. A stale id, a NULL world, a negative capacity or a
    /// NULL nameOut with a positive capacity returns 0 and counts one
    /// misuse; nameOut is then zeroed over its capacity whenever it and
    /// the capacity are valid.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param nameOut   Receives the name's bytes, with no terminator.
    /// @param capacity  nameOut's length in bytes, at least 0.
    /// @return The name's length in bytes, or 0 on a refusal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API int32_t m3Body_GetName(const m3World* world, m3BodyId body, char* nameOut,
                                  int32_t capacity);

    /// Who touches me now: fills up to capacity entries and returns
    /// the total, which may exceed capacity; a NULL array with
    /// capacity 0 measures. A stale body is a refusal. A refusal zeroes the array over its whole
    /// capacity; a success, an empty one included, writes only its entries; a list call only reads,
    /// so none is refused as busy. See m3ContactData in world.h.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param dataOut   Receives up to capacity contacts.
    /// @param capacity  dataOut's length, at least 0.
    /// @return The total number of the body's touching contacts; 0 on a
    ///         refusal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API int32_t m3Body_GetContactData(const m3World* world, m3BodyId body,
                                         m3ContactData* dataOut, int32_t capacity);

    /// Whether this body may fall asleep; turning it off wakes it.
    /// Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param flag   Whether the body may sleep.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_EnableSleep(m3World* world, m3BodyId body, bool flag);

    /// Returns whether the body may fall asleep.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return Whether it may; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Body_IsSleepEnabled(const m3World* world, m3BodyId body);

    /// The speed below which this body counts as resting; zero
    /// restores the world default. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world      The world.
    /// @param body       The body.
    /// @param threshold  The resting speed, m/s, finite and not negative; 0
    ///                   restores the world default.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetSleepThreshold(m3World* world, m3BodyId body, float threshold);

    /// Returns the speed below which the body counts as resting.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return The threshold, m/s; 0 for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Body_GetSleepThreshold(const m3World* world, m3BodyId body);

    /// Returns whether the body is awake.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @return Whether it is; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Body_IsAwake(const m3World* world, m3BodyId body);

    /// SetAwake(true) wakes; SetAwake(false) puts the single body
    /// to sleep and zeroes its velocities. Journaled. Only a dynamic body
    /// takes it: another type is refused.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param awake  Whether the body is awake.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetAwake(m3World* world, m3BodyId body, bool awake);

    // Forces and impulses, journaled like every mutation. Forces and
    // torques ACCUMULATE and act over the next step, then clear; impulses
    // change velocity immediately. Only dynamic bodies respond: static
    // and kinematic targets are refused. The force and torque held over
    // a step stay within 1e20 on every axis, and an impulse may leave the
    // body at most 1e5 m/s or rad/s on every axis; a call past either is
    // refused whole, before anything changes. A nonzero application wakes
    // the body. Points are world-space; an off-center application adds
    // the matching angular part.

    /// Adds a force at the center of mass for the next step. Journaled.
    /// Only a dynamic body takes it: a static or kinematic target is refused.
    /// The force and torque the body holds for the step stay within 1e20 on
    /// every axis: a call that would pass that, or a force that is not
    /// finite, is refused whole.
    /// A stale or null id, a refused target, value or point, or a call
    /// from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param force  The force, newtons.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_ApplyForce(m3World* world, m3BodyId body, m3Vec3 force);

    /// Adds a torque for the next step. Journaled.
    /// Only a dynamic body takes it: a static or kinematic target is refused.
    /// The torque the body holds for the step stays within 1e20 on every
    /// axis: a call that would pass that, or a torque that is not finite,
    /// is refused whole.
    /// A stale or null id, a refused target, value or point, or a call
    /// from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    ///
    /// @param world   The world.
    /// @param body    The body.
    /// @param torque  The torque, N m, in the world frame.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_ApplyTorque(m3World* world, m3BodyId body, m3Vec3 torque);

    /// Changes the velocity at once by impulse over mass. Journaled.
    /// Only a dynamic body takes it: a static or kinematic target is refused.
    /// The impulse may leave the body at most 1e5 m/s and rad/s on every
    /// axis: a call that would pass that, or an impulse that is not
    /// finite, is refused whole.
    /// A stale or null id, a refused target, value or point, or a call
    /// from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    ///
    /// @param world    The world.
    /// @param body     The body.
    /// @param impulse  The impulse, N s, at the center of mass.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_ApplyLinearImpulse(m3World* world, m3BodyId body, m3Vec3 impulse);

    /// Changes the angular velocity at once. Journaled.
    /// Only a dynamic body takes it: a static or kinematic target is refused.
    /// The impulse may leave the body at most 1e5 m/s and rad/s on every
    /// axis: a call that would pass that, or an impulse that is not
    /// finite, is refused whole.
    /// A stale or null id, a refused target, value or point, or a call
    /// from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    ///
    /// @param world    The world.
    /// @param body     The body.
    /// @param impulse  The angular impulse, N m s, in the world frame.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_ApplyAngularImpulse(m3World* world, m3BodyId body, m3Vec3 impulse);

    /// Adds a force at a world point for the next step, with the torque
    /// its arm gives. Journaled.
    /// Only a dynamic body takes it: a static or kinematic target is refused.
    /// The force and torque the body holds for the step stay within 1e20 on
    /// every axis: a call that would pass that, or a force that is not
    /// finite, is refused whole.
    /// A point that is not finite is refused.
    /// A stale or null id, a refused target, value or point, or a call
    /// from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    ///
    /// @param world  The world.
    /// @param body   The body.
    /// @param force  The force, newtons.
    /// @param point  Where it acts, in world space.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_ApplyForceAtPoint(m3World* world, m3BodyId body, m3Vec3 force, m3Pos3 point);

    /// Applies an impulse at a world point, changing both velocities at
    /// once. Journaled.
    /// Only a dynamic body takes it: a static or kinematic target is refused.
    /// The impulse may leave the body at most 1e5 m/s and rad/s on every
    /// axis: a call that would pass that, or an impulse that is not
    /// finite, is refused whole.
    /// A point that is not finite is refused.
    /// A stale or null id, a refused target, value or point, or a call
    /// from host code the world is running changes nothing, is not
    /// journaled and counts one misuse.
    ///
    /// @param world    The world.
    /// @param body     The body.
    /// @param impulse  The impulse, N s.
    /// @param point    Where it acts, in world space.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_ApplyLinearImpulseAtPoint(m3World* world, m3BodyId body, m3Vec3 impulse,
                                                 m3Pos3 point);

    /// Velocities, journaled: within 1e5 m/s on every axis (1e10 m/s
    /// for a kinematic body), and every body's spin within 1e5 rad/s.
    /// Setting a velocity wakes the body. A static body never moves, so it
    /// takes no velocity: the call is refused.
    /// A stale or null id, a static body, a refused value, or a call from
    /// host code the world is running changes nothing, is not journaled and
    /// counts one misuse.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param velocity  The velocity of the center of mass, m/s.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetLinearVelocity(m3World* world, m3BodyId body, m3Vec3 velocity);

    /// Sets the angular velocity in the world frame, within 1e5 rad/s on
    /// every axis, and wakes the body. A static body is refused.
    /// A stale or null id, a static body, a refused value, or a call from
    /// host code the world is running changes nothing, is not journaled and
    /// counts one misuse.
    ///
    /// @param world     The world.
    /// @param body      The body.
    /// @param velocity  The angular velocity, rad/s.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Body_SetAngularVelocity(m3World* world, m3BodyId body, m3Vec3 velocity);

    static const m3BodyId m3_nullBodyId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BODY_H
