// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body readback: the world and frame, mass, points and vectors between
// the body and world frames, velocities at points, bounds, and the
// shapes and joints on a body. Pure readers.

#include "body_state.h"
#include "broad_phase.h"
#include "fpenv.h"
#include "world_internal.h"

static M3_NOINLINE m3Transform BodyGetTransformWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    m3Transform identity = {{0.0, 0.0, 0.0}, {0.0f, 0.0f, 0.0f, 1.0f}};
    return m3ResolveBody(world, body, &index) ? world->bodies.transforms[index] : identity;
}

m3Transform m3Body_GetTransform(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Transform result = BodyGetTransformWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE float BodyGetMassWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index) || world->bodies.invMass[index] <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / world->bodies.invMass[index];
}

float m3Body_GetMass(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    float result = BodyGetMassWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Mat3 BodyGetRotationalInertiaWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.inertiaLocal[index]
                                              : m3MakeZeroMat3();
}

m3Mat3 m3Body_GetRotationalInertia(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Mat3 result = BodyGetRotationalInertiaWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Vec3 BodyGetLocalCenterWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    return m3ResolveBody(world, body, &index) ? world->bodies.localCenters[index] : zero;
}

m3Vec3 m3Body_GetLocalCenter(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = BodyGetLocalCenterWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

// A body-local point in world space: the rotation in float, the offset
// added in double.
static m3Pos3 ToWorld(const m3Transform* xf, m3Vec3 local)
{
    m3Vec3 r = m3RotateVec3(xf->q, local);
    m3Pos3 p = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
    return p;
}

static M3_NOINLINE m3Pos3 BodyGetWorldCenterOfMassWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return ToWorld(&world->bodies.transforms[index], world->bodies.localCenters[index]);
}

m3Pos3 m3Body_GetWorldCenterOfMass(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3Pos3 result = BodyGetWorldCenterOfMassWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Pos3 BodyGetWorldPointWorker(const m3World* world, m3BodyId body,
                                                  m3Vec3 localPoint)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return ToWorld(&world->bodies.transforms[index], localPoint);
}

m3Pos3 m3Body_GetWorldPoint(const m3World* world, m3BodyId body, m3Vec3 localPoint)
{
    uint64_t env = m3FloatEnvEnter();
    m3Pos3 result = BodyGetWorldPointWorker(world, body, localPoint);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Vec3 BodyGetLocalPointWorker(const m3World* world, m3BodyId body,
                                                  m3Pos3 worldPoint)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    const m3Transform* xf = &world->bodies.transforms[index];
    m3Vec3 d = {(m3real)(worldPoint.x - xf->p.x), (m3real)(worldPoint.y - xf->p.y),
                (m3real)(worldPoint.z - xf->p.z)};
    return m3InvRotateVec3(xf->q, d);
}

m3Vec3 m3Body_GetLocalPoint(const m3World* world, m3BodyId body, m3Pos3 worldPoint)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = BodyGetLocalPointWorker(world, body, worldPoint);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Vec3 BodyGetWorldVectorWorker(const m3World* world, m3BodyId body,
                                                   m3Vec3 localVector)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return m3RotateVec3(world->bodies.transforms[index].q, localVector);
}

m3Vec3 m3Body_GetWorldVector(const m3World* world, m3BodyId body, m3Vec3 localVector)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = BodyGetWorldVectorWorker(world, body, localVector);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Vec3 BodyGetLocalVectorWorker(const m3World* world, m3BodyId body,
                                                   m3Vec3 worldVector)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return m3InvRotateVec3(world->bodies.transforms[index].q, worldVector);
}

m3Vec3 m3Body_GetLocalVector(const m3World* world, m3BodyId body, m3Vec3 worldVector)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = BodyGetLocalVectorWorker(world, body, worldVector);
    m3FloatEnvLeave(env);
    return result;
}

// v + w x r, with the arm from the center of mass (one double crossing).
static m3Vec3 PointVelocity(const m3World* world, int32_t index, m3Pos3 worldPoint)
{
    m3Pos3 com = ToWorld(&world->bodies.transforms[index], world->bodies.localCenters[index]);
    m3Vec3 r = {(m3real)(worldPoint.x - com.x), (m3real)(worldPoint.y - com.y),
                (m3real)(worldPoint.z - com.z)};
    return m3Add3(world->bodies.linearVelocities[index],
                  m3Cross3(world->bodies.angularVelocities[index], r));
}

static M3_NOINLINE m3Vec3 BodyGetWorldPointVelocityWorker(const m3World* world, m3BodyId body,
                                                          m3Pos3 worldPoint)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return PointVelocity(world, index, worldPoint);
}

m3Vec3 m3Body_GetWorldPointVelocity(const m3World* world, m3BodyId body, m3Pos3 worldPoint)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = BodyGetWorldPointVelocityWorker(world, body, worldPoint);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Vec3 BodyGetLocalPointVelocityWorker(const m3World* world, m3BodyId body,
                                                          m3Vec3 localPoint)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return PointVelocity(world, index, ToWorld(&world->bodies.transforms[index], localPoint));
}

m3Vec3 m3Body_GetLocalPointVelocity(const m3World* world, m3BodyId body, m3Vec3 localPoint)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = BodyGetLocalPointVelocityWorker(world, body, localPoint);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE float BodyGetGravityScaleWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.gravityScales[index] : 0.0f;
}

float m3Body_GetGravityScale(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    float result = BodyGetGravityScaleWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE float BodyGetLinearDampingWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.linearDamping[index] : 0.0f;
}

float m3Body_GetLinearDamping(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    float result = BodyGetLinearDampingWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE float BodyGetAngularDampingWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) ? world->bodies.angularDamping[index] : 0.0f;
}

float m3Body_GetAngularDamping(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    float result = BodyGetAngularDampingWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

bool m3Body_IsBullet(const m3World* world, m3BodyId body)
{
    int32_t index;
    return m3ResolveBody(world, body, &index) && world->bodies.bulletFlags[index] != 0;
}

static M3_NOINLINE m3AabbResult BodyComputeAabbWorker(const m3World* world, m3BodyId body)
{
    int32_t index;
    m3AabbResult result = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    if (!m3ResolveBody(world, body, &index))
    {
        return result;
    }
    m3Pos3 origin = world->bodies.transforms[index].p;
    result.lowerBound = origin;
    result.upperBound = origin;
    bool first = true;
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        double lo[3];
        double hi[3];
        m3ShapeFatAabb(world, s, lo, hi);
        if (first || lo[0] < result.lowerBound.x)
        {
            result.lowerBound.x = lo[0];
        }
        if (first || lo[1] < result.lowerBound.y)
        {
            result.lowerBound.y = lo[1];
        }
        if (first || lo[2] < result.lowerBound.z)
        {
            result.lowerBound.z = lo[2];
        }
        if (first || hi[0] > result.upperBound.x)
        {
            result.upperBound.x = hi[0];
        }
        if (first || hi[1] > result.upperBound.y)
        {
            result.upperBound.y = hi[1];
        }
        if (first || hi[2] > result.upperBound.z)
        {
            result.upperBound.z = hi[2];
        }
        first = false;
    }
    return result;
}

m3AabbResult m3Body_ComputeAabb(const m3World* world, m3BodyId body)
{
    uint64_t env = m3FloatEnvEnter();
    m3AabbResult result = BodyComputeAabbWorker(world, body);
    m3FloatEnvLeave(env);
    return result;
}

int32_t m3Body_GetShapes(const m3World* world, m3BodyId body, m3ShapeId* ids, int32_t capacity)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t s = 0; s < world->shapes.shapePool.maxIndex; ++s)
    {
        if (world->shapes.shapePool.alive[s] == 0 || world->shapes.shapeBody[s] != index)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m3ShapeId){s + 1, world->shapes.shapePool.generations[s]};
        }
        total += 1;
    }
    return total;
}

int32_t m3Body_GetJoints(const m3World* world, m3BodyId body, m3JointId* ids, int32_t capacity)
{
    int32_t index;
    if (!m3ResolveBody(world, body, &index))
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t j = 0; j < world->joints.jointPool.maxIndex; ++j)
    {
        if (world->joints.jointPool.alive[j] == 0 ||
            (world->joints.jointBodyA[j] != index && world->joints.jointBodyB[j] != index))
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m3JointId){j + 1, world->joints.jointPool.generations[j]};
        }
        total += 1;
    }
    return total;
}
