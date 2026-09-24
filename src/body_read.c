// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body readback: the world and frame, mass, points and vectors between
// the body and world frames, velocities at points, bounds, and the
// shapes and joints on a body. Pure readers.

#include "body.h"
#include "broad_phase.h"
#include "world.h"
#include "world_internal.h"

m3WorldId m3Body_GetWorld(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    m3WorldId id = {0, 0};
    if (world != NULL)
    {
        id = (m3WorldId){(uint16_t)(world->slot + 1), world->generation};
    }
    return id;
}

m3Transform m3Body_GetTransform(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    m3Transform identity = {{0.0, 0.0, 0.0}, {0.0f, 0.0f, 0.0f, 1.0f}};
    return world != NULL ? world->bodies.transforms[index] : identity;
}

float m3Body_GetMass(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL || world->bodies.invMass[index] <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / world->bodies.invMass[index];
}

m3Mat3 m3Body_GetRotationalInertia(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.inertiaLocal[index] : m3MakeZeroMat3();
}

m3Vec3 m3Body_GetLocalCenter(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    return world != NULL ? world->bodies.localCenters[index] : zero;
}

// A body-local point in world space: the rotation in float, the offset
// added in double.
static m3Pos3 ToWorld(const m3Transform* xf, m3Vec3 local)
{
    m3Vec3 r = m3RotateVec3(xf->q, local);
    m3Pos3 p = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
    return p;
}

m3Pos3 m3Body_GetWorldCenterOfMass(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return ToWorld(&world->bodies.transforms[index], world->bodies.localCenters[index]);
}

m3Pos3 m3Body_GetWorldPoint(m3BodyId bodyId, m3Vec3 localPoint)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return ToWorld(&world->bodies.transforms[index], localPoint);
}

m3Vec3 m3Body_GetLocalPoint(m3BodyId bodyId, m3Pos3 worldPoint)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    const m3Transform* xf = &world->bodies.transforms[index];
    m3Vec3 d = {(m3real)(worldPoint.x - xf->p.x), (m3real)(worldPoint.y - xf->p.y),
                (m3real)(worldPoint.z - xf->p.z)};
    return m3InvRotateVec3(xf->q, d);
}

m3Vec3 m3Body_GetWorldVector(m3BodyId bodyId, m3Vec3 localVector)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return m3RotateVec3(world->bodies.transforms[index].q, localVector);
}

m3Vec3 m3Body_GetLocalVector(m3BodyId bodyId, m3Vec3 worldVector)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return m3InvRotateVec3(world->bodies.transforms[index].q, worldVector);
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

m3Vec3 m3Body_GetWorldPointVelocity(m3BodyId bodyId, m3Pos3 worldPoint)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return PointVelocity(world, index, worldPoint);
}

m3Vec3 m3Body_GetLocalPointVelocity(m3BodyId bodyId, m3Vec3 localPoint)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return PointVelocity(world, index, ToWorld(&world->bodies.transforms[index], localPoint));
}

float m3Body_GetGravityScale(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.gravityScales[index] : 0.0f;
}

float m3Body_GetLinearDamping(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.linearDamping[index] : 0.0f;
}

float m3Body_GetAngularDamping(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    return world != NULL ? world->bodies.angularDamping[index] : 0.0f;
}

bool m3Body_IsBullet(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    return world != NULL && world->bodies.bulletFlags[index] != 0;
}

m3AabbResult m3Body_ComputeAabb(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    m3AabbResult result = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    if (world == NULL)
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

int32_t m3Body_GetShapes(m3BodyId bodyId, m3ShapeId* ids, int32_t capacity)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
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
            ids[total] = (m3ShapeId){s + 1, world->idWorld, world->shapes.shapePool.generations[s]};
        }
        total += 1;
    }
    return total;
}

int32_t m3Body_GetJoints(m3BodyId bodyId, m3JointId* ids, int32_t capacity)
{
    int32_t index;
    m3World* world = m3ResolveBody(bodyId, &index);
    if (world == NULL)
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
            ids[total] = (m3JointId){j + 1, world->idWorld, world->joints.jointPool.generations[j]};
        }
        total += 1;
    }
    return total;
}
