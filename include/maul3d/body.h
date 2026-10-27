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

    typedef enum m3BodyType
    {
        m3_staticBody = 0,
        m3_kinematicBody = 1, // moved by velocity, immovable by contact
        m3_dynamicBody = 2,
    } m3BodyType;

    /// Per-axis motion locks. A locked axis holds still: its velocity is
    /// zeroed every substep, so contacts cannot bank motion on it.
    /// World-space bounds, double precision.
    typedef struct m3AabbResult
    {
        m3Pos3 lowerBound;
        m3Pos3 upperBound;
    } m3AabbResult;

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
        int32_t type;           // m3BodyType
        m3Pos3 position;        // world space, double, within 1e15 m
        m3Quat rotation;        // unit quaternion
        m3Vec3 linearVelocity;  // m/s, within 1e5 (kinematic: 1e10)
        m3Vec3 angularVelocity; // rad/s, world frame, within 1e5
        float gravityScale;     // within 1e4
        float linearDamping;
        float angularDamping;
        M3_ALIGN8 uint64_t userData; // opaque, carried unchanged
        /// A high speed body that gets the full continuous pass
        /// against static AND dynamic targets. Every fast
        /// dynamic body already sweeps against statics; the bullet
        /// flag buys the dynamic-target sweep. Bullet versus bullet
        /// is not resolved (a documented limitation).
        bool isBullet;
        m3MotionLocks motionLocks;
        bool enableSleep;        // false = this body never sleeps
        float sleepThreshold;    // resting speed; zero = the world default
        bool isEnabled;          // false = created outside simulation and queries
        bool enableFastRotation; // may spin past the world's angular cap, up to 1e5 rad/s
        int32_t internalValue;
    } m3BodyDef;

    /// Returns a def with pinned defaults (identity rotation, gravity
    /// scale one, sleep and simulation enabled) and a valid cookie.
    M3_API m3BodyDef m3DefaultBodyDef(void);

    /// Create a body. Returns m3_errorInvalid for an invalid def and
    /// m3_errorCapacity for an exhausted body pool; *bodyOut is the null
    /// id on any failure. A shapeless dynamic body has unit mass and zero
    /// inertia until a shape provides the real values.
    M3_NODISCARD M3_API m3Result m3CreateBody(m3World* world, const m3BodyDef* def,
                                              m3BodyId* bodyOut);

    /// Destroy a body. The id goes stale; the slot recycles FIFO with a
    /// generation bump and retires after 2^32 objects instead of
    /// wrapping. A stale id returns m3_errorStale and the null id
    /// m3_errorInvalid; every destroy of an object a world owns answers
    /// the same way.
    M3_NODISCARD M3_API m3Result m3DestroyBody(m3World* world, m3BodyId body);

    M3_API bool m3Body_IsValid(const m3World* world, m3BodyId body);
    M3_API m3Pos3 m3Body_GetPosition(const m3World* world, m3BodyId body);
    M3_API m3Quat m3Body_GetRotation(const m3World* world, m3BodyId body);
    M3_API m3Vec3 m3Body_GetLinearVelocity(const m3World* world, m3BodyId body);
    M3_API m3Vec3 m3Body_GetAngularVelocity(const m3World* world, m3BodyId body);
    M3_API uint64_t m3Body_GetUserData(const m3World* world, m3BodyId body);
    M3_API m3BodyType m3Body_GetType(const m3World* world, m3BodyId body);

    /// Readback, as in Maul2D. Thread class: reader. Mass and inertia
    /// are about the center of mass, the inertia tensor in the body
    /// frame; points and vectors convert between the body frame and
    /// world space. ComputeAabb is the union of the shapes' broadphase
    /// bounds (margin included), or the body origin for a shapeless
    /// body. GetShapes and GetJoints fill up to capacity ids in
    /// ascending slot order and return the total.
    M3_API m3Transform m3Body_GetTransform(const m3World* world, m3BodyId body);
    M3_API float m3Body_GetMass(const m3World* world, m3BodyId body);
    M3_API m3Mat3 m3Body_GetRotationalInertia(const m3World* world, m3BodyId body);
    M3_API m3Vec3 m3Body_GetLocalCenter(const m3World* world, m3BodyId body);
    M3_API m3Pos3 m3Body_GetWorldCenterOfMass(const m3World* world, m3BodyId body);
    M3_API m3Pos3 m3Body_GetWorldPoint(const m3World* world, m3BodyId body, m3Vec3 localPoint);
    M3_API m3Vec3 m3Body_GetLocalPoint(const m3World* world, m3BodyId body, m3Pos3 worldPoint);
    M3_API m3Vec3 m3Body_GetWorldVector(const m3World* world, m3BodyId body, m3Vec3 localVector);
    M3_API m3Vec3 m3Body_GetLocalVector(const m3World* world, m3BodyId body, m3Vec3 worldVector);
    M3_API m3Vec3 m3Body_GetWorldPointVelocity(const m3World* world, m3BodyId body,
                                               m3Pos3 worldPoint);
    M3_API m3Vec3 m3Body_GetLocalPointVelocity(const m3World* world, m3BodyId body,
                                               m3Vec3 localPoint);
    M3_API float m3Body_GetGravityScale(const m3World* world, m3BodyId body);
    M3_API float m3Body_GetLinearDamping(const m3World* world, m3BodyId body);
    M3_API float m3Body_GetAngularDamping(const m3World* world, m3BodyId body);
    M3_API bool m3Body_IsBullet(const m3World* world, m3BodyId body);
    /// Journaled setters for what the def set, as in Maul2D. Damping
    /// must be finite and non-negative, a gravity scale within 1e4.
    /// Thread class: writer.
    M3_API void m3Body_SetGravityScale(m3World* world, m3BodyId body, float scale);
    M3_API void m3Body_SetLinearDamping(m3World* world, m3BodyId body, float damping);
    M3_API void m3Body_SetAngularDamping(m3World* world, m3BodyId body, float damping);
    M3_API void m3Body_SetBullet(m3World* world, m3BodyId body, bool flag);
    M3_API void m3Body_SetUserData(m3World* world, m3BodyId body, uint64_t userData);
    M3_API m3AabbResult m3Body_ComputeAabb(const m3World* world, m3BodyId body);
    M3_API int32_t m3Body_GetShapes(const m3World* world, m3BodyId body, m3ShapeId* ids,
                                    int32_t capacity);
    M3_API int32_t m3Body_GetJoints(const m3World* world, m3BodyId body, m3JointId* ids,
                                    int32_t capacity);

    /// Journaled setters: every mutation is a discrete op.
    /// Runtime control, all journaled. SetTransform is the
    /// teleport: the pose lands instantly, velocities stay, and
    /// bodies around BOTH the old and new locations wake so
    /// nothing keeps sleeping under or inside a teleported crate. The
    /// position must lie within 1e15 m of the origin on every axis.
    M3_API void m3Body_SetTransform(m3World* world, m3BodyId body, m3Pos3 position,
                                    m3Quat rotation);
    /// Kinematic servo: velocities are chosen at the next step so
    /// the body lands ON the target pose after that step, then the
    /// target clears. Kinematic bodies only (the correct way to
    /// drive elevators and doors), with the target within 1e6 m of
    /// where the body is.
    M3_API void m3Body_SetTargetTransform(m3World* world, m3BodyId body, m3Pos3 position,
                                          m3Quat rotation);
    /// Switch dynamic, kinematic, static at runtime. Mass rebuilds
    /// from shapes when turning dynamic; velocities zero when
    /// turning static; the neighborhood wakes. Refused when the linear
    /// velocity is past the new type's limit, when the body would
    /// turn dynamic with a shape below the mass floors or a total mass
    /// past 1e20 kg, when it holds a plane, mesh, height field or
    /// voxel chunk and would leave static, and when a joint on it would
    /// pass its lever ratio at the new type's mass.
    M3_API void m3Body_SetType(m3World* world, m3BodyId body, m3BodyType type);
    /// A disabled body vanishes from simulation AND queries without
    /// being destroyed; enabling wakes its neighborhood. Journaled.
    M3_API void m3Body_Enable(m3World* world, m3BodyId body);
    M3_API void m3Body_Disable(m3World* world, m3BodyId body);
    M3_API bool m3Body_IsEnabled(const m3World* world, m3BodyId body);
    /// Motion locks: locked components re-zero every substep, so 2.5D
    /// scenes and upright enemies stay exact. Journaled.
    M3_API void m3Body_SetMotionLocks(m3World* world, m3BodyId body, m3MotionLocks locks);
    M3_API m3MotionLocks m3Body_GetMotionLocks(const m3World* world, m3BodyId body);
    /// Let this body spin past the world's angular speed cap (for
    /// wheels and other legal fast spinners), up to the 1e5 rad/s the
    /// step holds every body within. Journaled.
    M3_API void m3Body_EnableFastRotation(m3World* world, m3BodyId body, bool flag);
    M3_API bool m3Body_IsFastRotationEnabled(const m3World* world, m3BodyId body);
    /// Debug name, up to 31 bytes plus the terminator; longer names
    /// truncate silently. Journaled and carried by snapshots, never
    /// part of the hash (a label moves no matter). GetName returns
    /// the empty string for unnamed bodies and stale ids.
    M3_API void m3Body_SetName(m3World* world, m3BodyId body, const char* name);
    M3_API const char* m3Body_GetName(const m3World* world, m3BodyId body);
    /// Who touches me now: fills up to capacity entries and
    /// returns the count written. See m3ContactData in world.h.
    M3_API int32_t m3Body_GetContactData(const m3World* world, m3BodyId body, m3ContactData* out,
                                         int32_t capacity);
    /// Whether this body may fall asleep; turning it off wakes it.
    /// Journaled.
    M3_API void m3Body_EnableSleep(m3World* world, m3BodyId body, bool flag);
    M3_API bool m3Body_IsSleepEnabled(const m3World* world, m3BodyId body);
    /// The speed below which this body counts as resting; zero
    /// restores the world default. Journaled.
    M3_API void m3Body_SetSleepThreshold(m3World* world, m3BodyId body, float threshold);
    M3_API float m3Body_GetSleepThreshold(const m3World* world, m3BodyId body);
    M3_API bool m3Body_IsAwake(const m3World* world, m3BodyId body);
    /// SetAwake(true) wakes; SetAwake(false) puts the single body
    /// to sleep and zeroes its velocities.
    M3_API void m3Body_SetAwake(m3World* world, m3BodyId body, bool awake);

    /// Forces and impulses, journaled like every mutation.
    /// Forces and torques ACCUMULATE and act over the next step,
    /// then clear; impulses change velocity immediately. Only
    /// awake-able dynamic bodies respond: static and kinematic
    /// targets are refused. The force and torque held over a step stay
    /// within 1e20 on every axis, and an impulse may leave the body
    /// at most 1e5 m/s or rad/s on every axis; a call past either is
    /// refused whole, before anything changes. A nonzero application
    /// wakes the body. Points are world-space; an off-center
    /// application adds the matching angular part.
    M3_API void m3Body_ApplyForce(m3World* world, m3BodyId body, m3Vec3 force);
    M3_API void m3Body_ApplyTorque(m3World* world, m3BodyId body, m3Vec3 torque);
    M3_API void m3Body_ApplyLinearImpulse(m3World* world, m3BodyId body, m3Vec3 impulse);
    M3_API void m3Body_ApplyAngularImpulse(m3World* world, m3BodyId body, m3Vec3 impulse);
    M3_API void m3Body_ApplyForceAtPoint(m3World* world, m3BodyId body, m3Vec3 force, m3Pos3 point);
    M3_API void m3Body_ApplyLinearImpulseAtPoint(m3World* world, m3BodyId body, m3Vec3 impulse,
                                                 m3Pos3 point);

    /// Velocities, journaled: within 1e5 m/s on every axis (1e10 m/s
    /// for a kinematic body), and every body's spin within 1e5 rad/s.
    M3_API void m3Body_SetLinearVelocity(m3World* world, m3BodyId body, m3Vec3 velocity);
    M3_API void m3Body_SetAngularVelocity(m3World* world, m3BodyId body, m3Vec3 velocity);

    static const m3BodyId m3_nullBodyId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BODY_H
