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
        m3Pos3 position;        // world space, double (hybrid precision)
        m3Quat rotation;        // unit quaternion
        m3Vec3 linearVelocity;  // m/s
        m3Vec3 angularVelocity; // rad/s, world frame
        float gravityScale;
        float linearDamping;
        float angularDamping;
        uint64_t userData; // opaque, carried unchanged
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
        bool enableFastRotation; // may spin past the world's angular speed cap
        int32_t internalValue;
    } m3BodyDef;

    /// Returns a def with pinned defaults (identity rotation, gravity
    /// scale one, sleep and simulation enabled) and a valid cookie.
    M3_API m3BodyDef m3DefaultBodyDef(void);

    /// Create a body. Returns the null id on an invalid def, a stale
    /// world, or an exhausted body pool (loud in debug builds). A
    /// shapeless dynamic body has unit mass and zero inertia until a
    /// shape provides the real values.
    M3_API m3BodyId m3CreateBody(m3WorldId worldId, const m3BodyDef* def);

    /// Destroy a body. The id goes stale; the slot recycles FIFO with a
    /// generation bump and retires instead of wrapping.
    M3_API void m3DestroyBody(m3BodyId bodyId);

    M3_API bool m3Body_IsValid(m3BodyId bodyId);
    M3_API m3Pos3 m3Body_GetPosition(m3BodyId bodyId);
    M3_API m3Quat m3Body_GetRotation(m3BodyId bodyId);
    M3_API m3Vec3 m3Body_GetLinearVelocity(m3BodyId bodyId);
    M3_API m3Vec3 m3Body_GetAngularVelocity(m3BodyId bodyId);
    M3_API uint64_t m3Body_GetUserData(m3BodyId bodyId);
    M3_API m3BodyType m3Body_GetType(m3BodyId bodyId);

    /// Readback, as in Maul2D. Thread class: reader. Mass and inertia
    /// are about the center of mass, the inertia tensor in the body
    /// frame; points and vectors convert between the body frame and
    /// world space. ComputeAabb is the union of the shapes' broadphase
    /// bounds (margin included), or the body origin for a shapeless
    /// body. GetShapes and GetJoints fill up to capacity ids in
    /// ascending slot order and return the total.
    M3_API m3WorldId m3Body_GetWorld(m3BodyId bodyId);
    M3_API m3Transform m3Body_GetTransform(m3BodyId bodyId);
    M3_API float m3Body_GetMass(m3BodyId bodyId);
    M3_API m3Mat3 m3Body_GetRotationalInertia(m3BodyId bodyId);
    M3_API m3Vec3 m3Body_GetLocalCenter(m3BodyId bodyId);
    M3_API m3Pos3 m3Body_GetWorldCenterOfMass(m3BodyId bodyId);
    M3_API m3Pos3 m3Body_GetWorldPoint(m3BodyId bodyId, m3Vec3 localPoint);
    M3_API m3Vec3 m3Body_GetLocalPoint(m3BodyId bodyId, m3Pos3 worldPoint);
    M3_API m3Vec3 m3Body_GetWorldVector(m3BodyId bodyId, m3Vec3 localVector);
    M3_API m3Vec3 m3Body_GetLocalVector(m3BodyId bodyId, m3Vec3 worldVector);
    M3_API m3Vec3 m3Body_GetWorldPointVelocity(m3BodyId bodyId, m3Pos3 worldPoint);
    M3_API m3Vec3 m3Body_GetLocalPointVelocity(m3BodyId bodyId, m3Vec3 localPoint);
    M3_API float m3Body_GetGravityScale(m3BodyId bodyId);
    M3_API float m3Body_GetLinearDamping(m3BodyId bodyId);
    M3_API float m3Body_GetAngularDamping(m3BodyId bodyId);
    M3_API bool m3Body_IsBullet(m3BodyId bodyId);
    /// Journaled setters for what the def set, as in Maul2D. Damping
    /// must be non-negative; every value finite. Thread class: writer.
    M3_API void m3Body_SetGravityScale(m3BodyId bodyId, float scale);
    M3_API void m3Body_SetLinearDamping(m3BodyId bodyId, float damping);
    M3_API void m3Body_SetAngularDamping(m3BodyId bodyId, float damping);
    M3_API void m3Body_SetBullet(m3BodyId bodyId, bool flag);
    M3_API void m3Body_SetUserData(m3BodyId bodyId, uint64_t userData);
    M3_API m3AabbResult m3Body_ComputeAabb(m3BodyId bodyId);
    M3_API int32_t m3Body_GetShapes(m3BodyId bodyId, m3ShapeId* ids, int32_t capacity);
    M3_API int32_t m3Body_GetJoints(m3BodyId bodyId, m3JointId* ids, int32_t capacity);

    /// Journaled setters: every mutation is a discrete op.
    /// Runtime control, all journaled. SetTransform is the
    /// teleport: the pose lands instantly, velocities stay, and
    /// bodies around BOTH the old and new locations wake so
    /// nothing keeps sleeping under or inside a teleported crate.
    M3_API void m3Body_SetTransform(m3BodyId bodyId, m3Pos3 position, m3Quat rotation);
    /// Kinematic servo: velocities are chosen at the next step so
    /// the body lands ON the target pose after that step, then the
    /// target clears. Kinematic bodies only (the correct way to
    /// drive elevators and doors).
    M3_API void m3Body_SetTargetTransform(m3BodyId bodyId, m3Pos3 position, m3Quat rotation);
    /// Switch dynamic, kinematic, static at runtime. Mass rebuilds
    /// from shapes when turning dynamic; velocities zero when
    /// turning static; the neighborhood wakes.
    M3_API void m3Body_SetType(m3BodyId bodyId, m3BodyType type);
    /// A disabled body vanishes from simulation AND queries without
    /// being destroyed; enabling wakes its neighborhood. Journaled.
    M3_API void m3Body_Enable(m3BodyId bodyId);
    M3_API void m3Body_Disable(m3BodyId bodyId);
    M3_API bool m3Body_IsEnabled(m3BodyId bodyId);
    /// Motion locks: locked components re-zero every substep, so 2.5D
    /// scenes and upright enemies stay exact. Journaled.
    M3_API void m3Body_SetMotionLocks(m3BodyId bodyId, m3MotionLocks locks);
    M3_API m3MotionLocks m3Body_GetMotionLocks(m3BodyId bodyId);
    /// Let this body spin past the world's angular speed cap (for
    /// wheels and other legal fast spinners). Journaled.
    M3_API void m3Body_EnableFastRotation(m3BodyId bodyId, bool flag);
    M3_API bool m3Body_IsFastRotationEnabled(m3BodyId bodyId);
    /// Debug name, up to 31 bytes plus the terminator; longer names
    /// truncate silently. Journaled and carried by snapshots, never
    /// part of the hash (a label moves no matter). GetName returns
    /// the empty string for unnamed bodies and stale ids.
    M3_API void m3Body_SetName(m3BodyId bodyId, const char* name);
    M3_API const char* m3Body_GetName(m3BodyId bodyId);
    /// Who touches me now: fills up to capacity entries and
    /// returns the count written. See m3ContactData in world.h.
    M3_API int32_t m3Body_GetContactData(m3BodyId bodyId, m3ContactData* out, int32_t capacity);
    /// Whether this body may fall asleep; turning it off wakes it.
    /// Journaled.
    M3_API void m3Body_EnableSleep(m3BodyId bodyId, bool flag);
    M3_API bool m3Body_IsSleepEnabled(m3BodyId bodyId);
    /// The speed below which this body counts as resting; zero
    /// restores the world default. Journaled.
    M3_API void m3Body_SetSleepThreshold(m3BodyId bodyId, float threshold);
    M3_API float m3Body_GetSleepThreshold(m3BodyId bodyId);
    M3_API bool m3Body_IsAwake(m3BodyId bodyId);
    /// SetAwake(true) wakes; SetAwake(false) puts the single body
    /// to sleep and zeroes its velocities.
    M3_API void m3Body_SetAwake(m3BodyId bodyId, bool awake);

    /// Forces and impulses, journaled like every mutation.
    /// Forces and torques ACCUMULATE and act over the next step,
    /// then clear; impulses change velocity immediately. Only
    /// awake-able dynamic bodies respond: static and kinematic
    /// targets and non-finite values are documented no-ops. A
    /// nonzero application wakes the body. Points are world-space;
    /// an off-center application adds the matching angular part.
    M3_API void m3Body_ApplyForce(m3BodyId bodyId, m3Vec3 force);
    M3_API void m3Body_ApplyTorque(m3BodyId bodyId, m3Vec3 torque);
    M3_API void m3Body_ApplyLinearImpulse(m3BodyId bodyId, m3Vec3 impulse);
    M3_API void m3Body_ApplyAngularImpulse(m3BodyId bodyId, m3Vec3 impulse);
    M3_API void m3Body_ApplyForceAtPoint(m3BodyId bodyId, m3Vec3 force, m3Pos3 point);
    M3_API void m3Body_ApplyLinearImpulseAtPoint(m3BodyId bodyId, m3Vec3 impulse, m3Pos3 point);

    M3_API void m3Body_SetLinearVelocity(m3BodyId bodyId, m3Vec3 velocity);
    M3_API void m3Body_SetAngularVelocity(m3BodyId bodyId, m3Vec3 velocity);

    static const m3BodyId m3_nullBodyId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BODY_H
