// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint defs, one per type: their defaults, and the creates that fill
// the full def from them and hand it to m3CreateJointFromDef. A field a
// type's def does not hold keeps the full def's default, so a joint's
// journal record and state are what the one shared def gave.

#include "fpenv.h"
#include "joint.h"
#include "journal.h"

#include <string.h>

// A def's cookie names its type as well as its layout, so one type's def
// handed to another type's create is refused. The tags stay clear of the
// other defs' cookies.
#define M3_TYPED_JOINT_COOKIE(T, type)                                                             \
    ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(T) << 8) ^ (32 + (int32_t)(type))))

static const m3Vec3 s_axisZ = {0.0f, 0.0f, 1.0f};

// Defines the create of one joint type: a worker that fills the full def
// from a def carrying the type's cookie, or passes NULL for any other, and
// the public call around it.
#define M3_JOINT_CREATE(Name, Def, type)                                                           \
    static M3_NOINLINE m3Result Create##Name##Worker(m3World* world, const Def* def,               \
                                                     m3JointId* jointOut)                          \
    {                                                                                              \
        bool own = def != NULL && def->internalValue == M3_TYPED_JOINT_COOKIE(Def, type);          \
        m3JointDef full = m3JointDefaults(type);                                                   \
        if (own)                                                                                   \
        {                                                                                          \
            Name##Full(def, &full);                                                                \
        }                                                                                          \
        return m3CreateJointFromDef(world, own ? &full : NULL, jointOut);                          \
    }                                                                                              \
    m3Result m3Create##Name##Joint(m3World* world, const Def* def, m3JointId* jointOut)            \
    {                                                                                              \
        uint64_t env = m3FloatEnvEnter();                                                          \
        m3Result result = Create##Name##Worker(world, def, jointOut);                              \
        m3FloatEnvLeave(env);                                                                      \
        return result;                                                                             \
    }

m3SphericalJointDef m3DefaultSphericalJointDef(void)
{
    m3SphericalJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = s_axisZ;
    def.localAxisB = s_axisZ;
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3SphericalJointDef, m3_sphericalJoint);
    return def;
}

static void SphericalFull(const m3SphericalJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->localAxisA = def->localAxisA;
    full->localAxisB = def->localAxisB;
    full->enableSpring = def->enableSpring;
    full->springHertz = def->springHertz;
    full->springDampingRatio = def->springDampingRatio;
    full->enableLimit = def->enableLimit;
    full->lowerLimit = def->lowerTwist;
    full->upperLimit = def->upperTwist;
    full->enableCone = def->enableCone;
    full->coneAngle = def->coneAngle;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Spherical, m3SphericalJointDef, m3_sphericalJoint)

m3RevoluteJointDef m3DefaultRevoluteJointDef(void)
{
    m3RevoluteJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = s_axisZ;
    def.localAxisB = s_axisZ;
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3RevoluteJointDef, m3_revoluteJoint);
    return def;
}

static void RevoluteFull(const m3RevoluteJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->localAxisA = def->localAxisA;
    full->localAxisB = def->localAxisB;
    full->enableSpring = def->enableSpring;
    full->springHertz = def->springHertz;
    full->springDampingRatio = def->springDampingRatio;
    full->enableMotor = def->enableMotor;
    full->motorSpeed = def->motorSpeed;
    full->maxMotorEffort = def->maxMotorEffort;
    full->enableLimit = def->enableLimit;
    full->lowerLimit = def->lowerAngle;
    full->upperLimit = def->upperAngle;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Revolute, m3RevoluteJointDef, m3_revoluteJoint)

m3PrismaticJointDef m3DefaultPrismaticJointDef(void)
{
    m3PrismaticJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = s_axisZ;
    def.localAxisB = s_axisZ;
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3PrismaticJointDef, m3_prismaticJoint);
    return def;
}

static void PrismaticFull(const m3PrismaticJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->localAxisA = def->localAxisA;
    full->localAxisB = def->localAxisB;
    full->enableSpring = def->enableSpring;
    full->springHertz = def->springHertz;
    full->springDampingRatio = def->springDampingRatio;
    full->enableMotor = def->enableMotor;
    full->motorSpeed = def->motorSpeed;
    full->maxMotorEffort = def->maxMotorEffort;
    full->enableLimit = def->enableLimit;
    full->lowerLimit = def->lowerTranslation;
    full->upperLimit = def->upperTranslation;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Prismatic, m3PrismaticJointDef, m3_prismaticJoint)

m3WeldJointDef m3DefaultWeldJointDef(void)
{
    m3WeldJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3WeldJointDef, m3_weldJoint);
    return def;
}

static void WeldFull(const m3WeldJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Weld, m3WeldJointDef, m3_weldJoint)

m3DistanceJointDef m3DefaultDistanceJointDef(void)
{
    m3DistanceJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3DistanceJointDef, m3_distanceJoint);
    return def;
}

static void DistanceFull(const m3DistanceJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->length = def->length;
    full->minLength = def->minLength;
    full->maxLength = def->maxLength;
    full->enableSpring = def->enableSpring;
    full->springHertz = def->springHertz;
    full->springDampingRatio = def->springDampingRatio;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Distance, m3DistanceJointDef, m3_distanceJoint)

m3GenericJointDef m3DefaultGenericJointDef(void)
{
    m3GenericJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = s_axisZ;
    def.localAxisB = s_axisZ;
    def.motorAxis = M3_GENERIC_NO_MOTOR;
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3GenericJointDef, m3_genericJoint);
    return def;
}

static void GenericFull(const m3GenericJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->localAxisA = def->localAxisA;
    full->localAxisB = def->localAxisB;
    for (int32_t k = 0; k < 3; ++k)
    {
        full->genericLinear[k] = def->linearModes[k];
        full->genericAngular[k] = def->angularModes[k];
        full->genericLinearLower[k] = def->linearLower[k];
        full->genericLinearUpper[k] = def->linearUpper[k];
        full->genericAngularLower[k] = def->angularLower[k];
        full->genericAngularUpper[k] = def->angularUpper[k];
    }
    full->genericMotorAxis = def->motorAxis;
    full->enableMotor = def->enableMotor;
    full->motorSpeed = def->motorSpeed;
    full->maxMotorEffort = def->maxMotorEffort;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Generic, m3GenericJointDef, m3_genericJoint)

m3WheelJointDef m3DefaultWheelJointDef(void)
{
    m3WheelJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    def.localSpinAxisB = s_axisZ;
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3WheelJointDef, m3_wheelJoint);
    return def;
}

static void WheelFull(const m3WheelJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->localAxisA = def->localAxisA;
    full->localAxisB = def->localSpinAxisB;
    full->enableSpring = def->enableSpring;
    full->springHertz = def->springHertz;
    full->springDampingRatio = def->springDampingRatio;
    full->enableMotor = def->enableMotor;
    full->motorSpeed = def->motorSpeed;
    full->maxMotorEffort = def->maxMotorEffort;
    full->enableLimit = def->enableLimit;
    full->lowerLimit = def->lowerTranslation;
    full->upperLimit = def->upperTranslation;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Wheel, m3WheelJointDef, m3_wheelJoint)

m3FilterJointDef m3DefaultFilterJointDef(void)
{
    m3FilterJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3FilterJointDef, m3_filterJoint);
    return def;
}

static void FilterFull(const m3FilterJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
}

M3_JOINT_CREATE(Filter, m3FilterJointDef, m3_filterJoint)

m3ParallelJointDef m3DefaultParallelJointDef(void)
{
    m3ParallelJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = s_axisZ;
    def.localAxisB = s_axisZ;
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3ParallelJointDef, m3_parallelJoint);
    return def;
}

static void ParallelFull(const m3ParallelJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->localAxisA = def->localAxisA;
    full->localAxisB = def->localAxisB;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Parallel, m3ParallelJointDef, m3_parallelJoint)

m3MotorJointDef m3DefaultMotorJointDef(void)
{
    m3MotorJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3MotorJointDef, m3_motorJoint);
    return def;
}

static void MotorFull(const m3MotorJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->maxForce = def->maxForce;
    full->maxTorque = def->maxTorque;
    full->enableSpring = def->enableSpring;
    full->springHertz = def->springHertz;
    full->springDampingRatio = def->springDampingRatio;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Motor, m3MotorJointDef, m3_motorJoint)

m3GearJointDef m3DefaultGearJointDef(void)
{
    m3GearJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = s_axisZ;
    def.localAxisB = s_axisZ;
    def.ratio = 1.0f;
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3GearJointDef, m3_gearJoint);
    return def;
}

static void GearFull(const m3GearJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->localAxisA = def->localAxisA;
    full->localAxisB = def->localAxisB;
    full->ratio = def->ratio;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Gear, m3GearJointDef, m3_gearJoint)

m3PulleyJointDef m3DefaultPulleyJointDef(void)
{
    m3PulleyJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratio = 1.0f;
    def.internalValue = M3_TYPED_JOINT_COOKIE(m3PulleyJointDef, m3_pulleyJoint);
    return def;
}

static void PulleyFull(const m3PulleyJointDef* def, m3JointDef* full)
{
    full->bodyIdA = def->bodyIdA;
    full->bodyIdB = def->bodyIdB;
    full->groundAnchorA = def->groundAnchorA;
    full->groundAnchorB = def->groundAnchorB;
    full->localAnchorA = def->localAnchorA;
    full->localAnchorB = def->localAnchorB;
    full->ratio = def->ratio;
    full->enableConnectedCollision = def->enableConnectedCollision;
}

M3_JOINT_CREATE(Pulley, m3PulleyJointDef, m3_pulleyJoint)
