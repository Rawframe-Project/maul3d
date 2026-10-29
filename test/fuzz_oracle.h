// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The value oracle for a restored world: every value a caller can read
// back must be one a caller could have set. FuzzValuesHold reads each live
// body, shape and joint, the world and the scene's character through the
// public getters and gives each value to its own setter, which must take
// it without counting a misuse. Where a create may keep a value no setter
// takes (a limit range left unordered while off, a servo's aim at a gap
// past the setter's reach), the oracle checks the create's rule instead.
// The character's own body keeps its type, which SetType refuses for that
// body whatever the type. The oracle changes the world (a setter moves,
// wakes and renames bodies and zeroes warm impulses), so a target runs it
// last.

#ifndef MAUL3D_TEST_FUZZ_ORACLE_H
#define MAUL3D_TEST_FUZZ_ORACLE_H

#include "maul3d/body.h"
#include "maul3d/character.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define FUZZ_ORACLE_CAPACITY 64

// The reach of a motor joint's aim at create, M3_SERVO_AIM_LIMIT in
// src/joint.h: the gap between two anchors on bodies within
// M3_POSITION_LIMIT, under 3.5e15 on every axis in any frame.
#define FUZZ_SERVO_AIM_LIMIT 4.0e15f

static inline uint64_t FuzzMisuse(const m3World* world)
{
    return m3World_GetCounters(world).misuse;
}

static inline void FuzzWorldValues(m3World* world)
{
    m3World_SetGravity(world, m3World_GetGravity(world));
    m3World_EnableSleep(world, m3World_IsSleepEnabled(world));
    m3World_EnableContinuous(world, m3World_IsContinuousEnabled(world));
}

static inline void FuzzShapeValues(m3World* world, m3ShapeId shape)
{
    m3Shape_SetFriction(world, shape, m3Shape_GetFriction(world, shape));
    m3Shape_SetRestitution(world, shape, m3Shape_GetRestitution(world, shape));
    m3Shape_SetRollingResistance(world, shape, m3Shape_GetRollingResistance(world, shape));
    m3Shape_SetDensity(world, shape, m3Shape_GetDensity(world, shape));
    m3Shape_EnableHitEvents(world, shape, m3Shape_IsHitEventsEnabled(world, shape));
    m3Shape_EnablePreSolveEvents(world, shape, m3Shape_IsPreSolveEventsEnabled(world, shape));
    uint64_t category = 0;
    uint64_t mask = 0;
    int32_t group = 0;
    m3Shape_GetFilter(world, shape, &category, &mask, &group);
    m3Shape_SetFilter(world, shape, category, mask, group);
}

// Whether a body is the character's own: the kinematic body the character
// stands on, at the character's position.
static inline bool FuzzCharacterBody(const m3World* world, m3BodyId body, m3CharacterId hero)
{
    if (!m3Character_IsValid(world, hero) || m3Body_GetType(world, body) != m3_kinematicBody)
    {
        return false;
    }
    m3Pos3 at = m3Body_GetPosition(world, body);
    m3Pos3 heroAt = m3Character_GetPosition(world, hero);
    return at.x == heroAt.x && at.y == heroAt.y && at.z == heroAt.z;
}

static inline void FuzzBodyValues(m3World* world, m3BodyId body, m3CharacterId hero)
{
    m3Transform pose = m3Body_GetTransform(world, body);
    m3Body_SetTransform(world, body, pose.p, pose.q);
    if (!FuzzCharacterBody(world, body, hero))
    {
        m3Body_SetType(world, body, m3Body_GetType(world, body));
    }
    m3Body_Enable(world, body, m3Body_IsEnabled(world, body));
    char name[32];
    strncpy(name, m3Body_GetName(world, body), sizeof(name) - 1);
    name[sizeof(name) - 1] = 0;
    m3Body_SetName(world, body, name);
    m3Body_SetLinearVelocity(world, body, m3Body_GetLinearVelocity(world, body));
    m3Body_SetAngularVelocity(world, body, m3Body_GetAngularVelocity(world, body));
    m3Body_SetGravityScale(world, body, m3Body_GetGravityScale(world, body));
    m3Body_SetLinearDamping(world, body, m3Body_GetLinearDamping(world, body));
    m3Body_SetAngularDamping(world, body, m3Body_GetAngularDamping(world, body));
    m3Body_SetSleepThreshold(world, body, m3Body_GetSleepThreshold(world, body));
    m3Body_SetMotionLocks(world, body, m3Body_GetMotionLocks(world, body));
    m3Body_EnableBullet(world, body, m3Body_IsBulletEnabled(world, body));
    m3Body_EnableSleep(world, body, m3Body_IsSleepEnabled(world, body));
    m3Body_EnableFastRotation(world, body, m3Body_IsFastRotationEnabled(world, body));
    // Only a dynamic body sleeps or wakes.
    if (m3Body_GetType(world, body) == m3_dynamicBody)
    {
        m3Body_SetAwake(world, body, m3Body_IsAwake(world, body));
    }
    m3ShapeId shapes[FUZZ_ORACLE_CAPACITY];
    int32_t count = m3Body_GetShapes(world, body, shapes, FUZZ_ORACLE_CAPACITY);
    for (int32_t i = 0; i < count && i < FUZZ_ORACLE_CAPACITY; ++i)
    {
        FuzzShapeValues(world, shapes[i]);
    }
}

// A unit rotation within the 10% the motor joint's setter allows.
static inline bool FuzzUnitQuat(m3Quat q)
{
    float n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return isfinite(n) && n >= 0.81f && n <= 1.21f;
}

// The types each part belongs to, as the joint calls take them.
static inline bool FuzzHasLimits(m3JointType type)
{
    return type == m3_revoluteJoint || type == m3_prismaticJoint || type == m3_sphericalJoint ||
           type == m3_wheelJoint;
}

static inline bool FuzzHasMotor(m3JointType type)
{
    return type == m3_revoluteJoint || type == m3_prismaticJoint || type == m3_wheelJoint ||
           type == m3_genericJoint || type == m3_distanceJoint;
}

static inline bool FuzzHasSpring(m3JointType type)
{
    return type == m3_revoluteJoint || type == m3_prismaticJoint || type == m3_sphericalJoint ||
           type == m3_wheelJoint || type == m3_motorJoint;
}

// Each part's values go in before its switch, as a caller turning the
// part on would set them. False when a value fails the create's own rule
// where the oracle checks that instead.
static inline bool FuzzJointValues(m3World* world, m3JointId joint)
{
    m3JointType type = m3Joint_GetType(world, joint);
    if (FuzzHasLimits(type))
    {
        float lower = 0.0f;
        float upper = 0.0f;
        m3Joint_GetLimits(world, joint, &lower, &upper);
        bool enabled = m3Joint_IsLimitEnabled(world, joint);
        // A def may leave a range unordered while its limit is off; the
        // setter takes only an ordered one.
        if (enabled || lower <= upper)
        {
            m3Joint_SetLimits(world, joint, lower, upper);
        }
        else if (!isfinite(lower) || !isfinite(upper))
        {
            return false;
        }
        m3Joint_EnableLimit(world, joint, enabled);
    }
    if (FuzzHasMotor(type))
    {
        m3Joint_SetMotorSpeed(world, joint, m3Joint_GetMotorSpeed(world, joint));
        m3Joint_SetMaxMotorEffort(world, joint, m3Joint_GetMaxMotorEffort(world, joint));
        m3Joint_EnableMotor(world, joint, m3Joint_IsMotorEnabled(world, joint));
    }
    if (FuzzHasSpring(type))
    {
        m3Joint_SetSpringHertz(world, joint, m3Joint_GetSpringHertz(world, joint));
        m3Joint_SetSpringDampingRatio(world, joint, m3Joint_GetSpringDampingRatio(world, joint));
        m3Joint_EnableSpring(world, joint, m3Joint_IsSpringEnabled(world, joint));
    }
    if (type == m3_distanceJoint)
    {
        float minLength = 0.0f;
        float maxLength = 0.0f;
        m3DistanceJoint_GetLengthRange(world, joint, &minLength, &maxLength);
        m3DistanceJoint_SetLengthRange(world, joint, minLength, maxLength);
    }
    else if (type == m3_motorJoint)
    {
        m3MotorJoint_SetMaxForce(world, joint, m3MotorJoint_GetMaxForce(world, joint));
        m3MotorJoint_SetMaxTorque(world, joint, m3MotorJoint_GetMaxTorque(world, joint));
        // The create aims at the bodies' gap, which may lie past the
        // setter's reach of M3_LOCAL_LIMIT but not past the create's.
        m3Vec3 linear = m3MotorJoint_GetLinearOffset(world, joint);
        m3Quat angular = m3MotorJoint_GetAngularOffset(world, joint);
        if (fabsf(linear.x) <= M3_LOCAL_LIMIT && fabsf(linear.y) <= M3_LOCAL_LIMIT &&
            fabsf(linear.z) <= M3_LOCAL_LIMIT)
        {
            m3MotorJoint_SetOffsets(world, joint, linear, angular);
        }
        else if (!(fabsf(linear.x) <= FUZZ_SERVO_AIM_LIMIT) ||
                 !(fabsf(linear.y) <= FUZZ_SERVO_AIM_LIMIT) ||
                 !(fabsf(linear.z) <= FUZZ_SERVO_AIM_LIMIT) || !FuzzUnitQuat(angular))
        {
            return false;
        }
    }
    else if (type == m3_wheelJoint)
    {
        m3WheelJoint_SetTargetSteeringAngle(world, joint,
                                            m3WheelJoint_GetTargetSteeringAngle(world, joint));
        m3WheelJoint_SetSteeringHertz(world, joint, m3WheelJoint_GetSteeringHertz(world, joint));
        m3WheelJoint_SetSteeringDampingRatio(world, joint,
                                             m3WheelJoint_GetSteeringDampingRatio(world, joint));
        m3WheelJoint_SetMaxSteeringTorque(world, joint,
                                          m3WheelJoint_GetMaxSteeringTorque(world, joint));
        m3WheelJoint_EnableSteering(world, joint, m3WheelJoint_IsSteeringEnabled(world, joint));
    }
    m3Joint_EnableConnectedCollision(world, joint,
                                     m3Joint_IsConnectedCollisionEnabled(world, joint));
    return true;
}

// Whether every value the world holds is one a caller could have set: the
// setters take them all without a misuse, and the values they cannot take
// meet the create's rule. The character id is the scene's; a world without
// it skips it.
static inline bool FuzzValuesHold(m3World* world, m3CharacterId hero)
{
    uint64_t misuse = FuzzMisuse(world);
    FuzzWorldValues(world);
    m3BodyId bodies[FUZZ_ORACLE_CAPACITY];
    int32_t bodyCount = m3World_GetBodies(world, bodies, FUZZ_ORACLE_CAPACITY);
    for (int32_t i = 0; i < bodyCount && i < FUZZ_ORACLE_CAPACITY; ++i)
    {
        FuzzBodyValues(world, bodies[i], hero);
    }
    bool createRules = true;
    m3JointId joints[FUZZ_ORACLE_CAPACITY];
    int32_t jointCount = m3World_GetJoints(world, joints, FUZZ_ORACLE_CAPACITY);
    for (int32_t i = 0; i < jointCount && i < FUZZ_ORACLE_CAPACITY; ++i)
    {
        createRules = FuzzJointValues(world, joints[i]) && createRules;
    }
    if (m3Character_IsValid(world, hero))
    {
        m3real halfHeight = 0.0f;
        m3real radius = 0.0f;
        m3Character_GetStance(world, hero, &halfHeight, &radius);
        // The stand-up veto refuses with m3_errorState and counts no
        // misuse; a value the setter refuses counts one.
        (void)m3Character_SetStance(world, hero, halfHeight, radius);
    }
    return createRules && FuzzMisuse(world) == misuse && bodyCount <= FUZZ_ORACLE_CAPACITY &&
           jointCount <= FUZZ_ORACLE_CAPACITY;
}

#endif
