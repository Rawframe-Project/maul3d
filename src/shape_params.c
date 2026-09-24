// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape runtime materials and flags: friction, restitution, rolling
// resistance, density, events, pre-solve, mesh materials and surface
// velocity. Public functions validate and journal; replay drives the
// internal setters.

#include "body.h"
#include "broad_phase.h"
#include "hull.h"
#include "journal.h"
#include "manifold.h"
#include "quickhull.h"
#include "shape.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

void m3SetShapeFrictionInternal(m3World* world, int32_t slot, float value)
{
    world->shapes.shapeFriction[slot] = value;
}

void m3SetShapeRestitutionInternal(m3World* world, int32_t slot, float value)
{
    world->shapes.shapeRestitution[slot] = value;
}

void m3SetShapeRollingInternal(m3World* world, int32_t slot, float value)
{
    world->shapes.shapeRollingResistance[slot] = value;
}

void m3SetShapeDensityInternal(m3World* world, int32_t slot, float value, int32_t updateMass)
{
    world->shapes.shapeDensity[slot] = value;
    if (updateMass != 0)
    {
        m3RecomputeMass(world, world->shapes.shapeBody[slot]);
    }
}

// One resolve + one journal + one internal, the body.c pattern.
static m3World* ResolveShape(m3ShapeId shapeId, int32_t* outSlot)
{
    m3World* world = m3WorldFromTag(shapeId.world);
    int32_t slot = world != NULL ? m3ShapeSlot(world, shapeId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return NULL;
    }
    *outSlot = slot;
    return world;
}

static void ShapeScalarOp(m3ShapeId shapeId, int32_t op, float value)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpShapeScalar record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.value = value;
        m3JournalRecord(world, op, &record, (int32_t)sizeof(record));
    }
    if (op == m3_opSetShapeFriction)
    {
        m3SetShapeFrictionInternal(world, slot, value);
    }
    else if (op == m3_opSetShapeRestitution)
    {
        m3SetShapeRestitutionInternal(world, slot, value);
    }
    else
    {
        m3SetShapeRollingInternal(world, slot, value);
    }
}

void m3Shape_SetFriction(m3ShapeId shapeId, float friction)
{
    if (!m3FiniteF(friction) || friction < 0.0f)
    {
        m3Refuse(m3WorldFromTag(shapeId.world), m3_errorInvalid);
        return;
    }
    ShapeScalarOp(shapeId, m3_opSetShapeFriction, friction);
}

float m3Shape_GetFriction(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapes.shapeFriction[slot] : 0.0f;
}

void m3Shape_SetRestitution(m3ShapeId shapeId, float restitution)
{
    if (!m3FiniteF(restitution) || restitution < 0.0f)
    {
        m3Refuse(m3WorldFromTag(shapeId.world), m3_errorInvalid);
        return;
    }
    ShapeScalarOp(shapeId, m3_opSetShapeRestitution, restitution);
}

float m3Shape_GetRestitution(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapes.shapeRestitution[slot] : 0.0f;
}

void m3Shape_SetRollingResistance(m3ShapeId shapeId, float value)
{
    if (!m3FiniteF(value) || value < 0.0f)
    {
        m3Refuse(m3WorldFromTag(shapeId.world), m3_errorInvalid);
        return;
    }
    ShapeScalarOp(shapeId, m3_opSetShapeRolling, value);
}

float m3Shape_GetRollingResistance(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapes.shapeRollingResistance[slot] : 0.0f;
}

void m3Shape_SetDensity(m3ShapeId shapeId, float density, bool updateBodyMass)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL || !m3FiniteF(density) || density <= 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetShapeDensity record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.value = density;
        record.updateMass = updateBodyMass ? 1 : 0;
        m3JournalRecord(world, m3_opSetShapeDensity, &record, (int32_t)sizeof(record));
    }
    m3SetShapeDensityInternal(world, slot, density, updateBodyMass ? 1 : 0);
}

float m3Shape_GetDensity(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapes.shapeDensity[slot] : 0.0f;
}

void m3EnableShapeHitEventsInternal(m3World* world, int32_t slot, int32_t on)
{
    world->shapes.shapeHitEvents[slot] = on != 0 ? 1 : 0;
}

void m3EnableShapePreSolveInternal(m3World* world, int32_t slot, int32_t on)
{
    world->shapes.shapePreSolve[slot] = on != 0 ? 1 : 0;
}

static void ShapeFlagOp(m3ShapeId shapeId, int32_t op, bool flag)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpShapeFlag record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.on = flag ? 1 : 0;
        m3JournalRecord(world, op, &record, (int32_t)sizeof(record));
    }
    if (op == m3_opEnableShapeHitEvents)
    {
        m3EnableShapeHitEventsInternal(world, slot, flag ? 1 : 0);
    }
    else
    {
        m3EnableShapePreSolveInternal(world, slot, flag ? 1 : 0);
    }
}

void m3Shape_EnableHitEvents(m3ShapeId shapeId, bool flag)
{
    ShapeFlagOp(shapeId, m3_opEnableShapeHitEvents, flag);
}

bool m3Shape_IsHitEventsEnabled(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL && world->shapes.shapeHitEvents[slot] != 0;
}

void m3Shape_EnablePreSolve(m3ShapeId shapeId, bool flag)
{
    ShapeFlagOp(shapeId, m3_opEnableShapePreSolve, flag);
}

bool m3Shape_IsPreSolveEnabled(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL && world->shapes.shapePreSolve[slot] != 0;
}

bool m3SetMeshMaterialsInternal(m3World* world, int32_t meshIndex,
                                const m3MeshSurfaceMaterial* materials, int32_t materialCount,
                                const uint8_t* triangleMaterials)
{
    // The full wall, here because replay hands this function
    // raw journal bytes: hostile counts, values, or group indices
    // refuse loudly and paint nothing.
    if (meshIndex < 0 || materialCount < 1 || materialCount > M3_MESH_MAX_MATERIALS)
    {
        return false;
    }
    m3MeshData* mesh = &world->meshes.meshData[meshIndex];
    if (mesh->triangleCount <= 0)
    {
        return false;
    }
    for (int32_t k = 0; k < materialCount; ++k)
    {
        const m3MeshSurfaceMaterial* m = &materials[k];
        if (!m3FiniteF(m->friction) || m->friction < 0.0f || !m3FiniteF(m->restitution) ||
            m->restitution < 0.0f || !m3FiniteF(m->rollingResistance) ||
            m->rollingResistance < 0.0f || !m3FiniteV3(m->surfaceVelocity))
        {
            return false;
        }
    }
    for (int32_t t = 0; t < mesh->triangleCount; ++t)
    {
        if (triangleMaterials[t] >= materialCount)
        {
            return false;
        }
    }
    mesh->materialCount = materialCount;
    memset(mesh->materials, 0, sizeof(mesh->materials));
    memcpy(mesh->materials, materials, (size_t)materialCount * sizeof(m3MeshSurfaceMaterial));
    memcpy(mesh->triMaterials, triangleMaterials, (size_t)mesh->triangleCount);
    return true;
}

void m3Shape_SetMeshMaterials(m3ShapeId shapeId, const m3MeshSurfaceMaterial* materials,
                              int32_t materialCount, const uint8_t* triangleMaterials)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL || materials == NULL || triangleMaterials == NULL ||
        world->shapes.shapeType[slot] != (uint8_t)m3_meshShape)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    int32_t meshIndex = world->shapes.shapeMeshIndex[slot];
    if (!m3SetMeshMaterialsInternal(world, meshIndex, materials, materialCount, triangleMaterials))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // refused: nothing journals
    }
    if (world->recorder.journalActive != 0)
    {
        int32_t triCount = world->meshes.meshData[meshIndex].triangleCount;
        int32_t bytes = (int32_t)sizeof(m3SetMeshMaterialsOp) + triCount;
        uint8_t* payload = (uint8_t*)m3AllocZeroed(bytes);
        if (payload == NULL)
        {
            return;
        }
        m3SetMeshMaterialsOp head;
        memset(&head, 0, sizeof(head));
        head.id = shapeId;
        head.materialCount = materialCount;
        head.triangleCount = triCount;
        memcpy(head.materials, materials, (size_t)materialCount * sizeof(m3MeshSurfaceMaterial));
        memcpy(payload, &head, sizeof(head));
        memcpy(payload + sizeof(head), triangleMaterials, (size_t)triCount);
        m3JournalRecord(world, m3_opSetMeshMaterials, payload, bytes);
        m3Free(payload);
    }
}

void m3SetSurfaceVelocityInternal(m3World* world, int32_t slot, m3Vec3 v)
{
    world->shapes.shapeSurfaceVel[slot] = v;
    int32_t body = world->shapes.shapeBody[slot];
    if (world->bodies.types[body] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, body, 1);
    }
}

void m3Shape_SetSurfaceVelocity(m3ShapeId shapeId, m3Vec3 velocity)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL || !m3FiniteV3(velocity))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetSurfaceVelocity record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetSurfaceVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetSurfaceVelocityInternal(world, slot, velocity);
}

void m3SetFilterInternal(m3World* world, int32_t slot, uint64_t categoryBits, uint64_t maskBits,
                         int32_t groupIndex)
{
    // Whoever this shape touches must notice its allegiance change:
    // awake pairs are filtered again on the next pair update.
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        int32_t a = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if ((a == slot || b == slot) && world->contacts.manifolds[i].pointCount > 0)
        {
            int32_t other = world->shapes.shapeBody[a == slot ? b : a];
            if (world->bodies.types[other] == (uint8_t)m3_dynamicBody)
            {
                m3SetAwakeInternal(world, other, 1);
            }
        }
    }
    int32_t body = world->shapes.shapeBody[slot];
    if (world->bodies.types[body] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, body, 1);
    }
    world->shapes.shapeCategory[slot] = categoryBits;
    world->shapes.shapeMask[slot] = maskBits;
    world->shapes.shapeGroup[slot] = groupIndex;
}

void m3Shape_SetFilter(m3ShapeId shapeId, uint64_t categoryBits, uint64_t maskBits,
                       int32_t groupIndex)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetFilter record;
        memset(&record, 0, sizeof(record));
        record.categoryBits = categoryBits;
        record.maskBits = maskBits;
        record.id = shapeId;
        record.groupIndex = groupIndex;
        m3JournalRecord(world, m3_opSetFilter, &record, (int32_t)sizeof(record));
    }
    m3SetFilterInternal(world, slot, categoryBits, maskBits, groupIndex);
}

void m3Shape_GetFilter(m3ShapeId shapeId, uint64_t* categoryBits, uint64_t* maskBits,
                       int32_t* groupIndex)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (categoryBits != NULL)
    {
        *categoryBits = world != NULL ? world->shapes.shapeCategory[slot] : 0u;
    }
    if (maskBits != NULL)
    {
        *maskBits = world != NULL ? world->shapes.shapeMask[slot] : 0u;
    }
    if (groupIndex != NULL)
    {
        *groupIndex = world != NULL ? world->shapes.shapeGroup[slot] : 0;
    }
}

m3WorldId m3Shape_GetWorld(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    m3WorldId id = {0, 0};
    if (world != NULL)
    {
        id = (m3WorldId){(uint16_t)(world->slot + 1), world->generation};
    }
    return id;
}

m3ShapeType m3Shape_GetType(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? (m3ShapeType)world->shapes.shapeType[slot] : m3_sphereShape;
}

uint64_t m3Shape_GetUserData(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapes.shapeUserData[slot] : 0u;
}

bool m3Shape_IsSensor(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL && world->shapes.shapeSensor[slot] != 0;
}

m3AabbResult m3Shape_GetAabb(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    m3AabbResult result = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    if (world != NULL)
    {
        double lo[3];
        double hi[3];
        m3ShapeFatAabb(world, slot, lo, hi);
        result.lowerBound = (m3Pos3){lo[0], lo[1], lo[2]};
        result.upperBound = (m3Pos3){hi[0], hi[1], hi[2]};
    }
    return result;
}

void m3Shape_SetUserData(m3ShapeId shapeId, uint64_t userData)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpUserData record;
        memset(&record, 0, sizeof(record));
        record.userData = userData;
        record.id = (m3BodyId){shapeId.index1, shapeId.world, shapeId.generation};
        m3JournalRecord(world, m3_opSetShapeUserData, &record, (int32_t)sizeof(record));
    }
    world->shapes.shapeUserData[slot] = userData;
}
