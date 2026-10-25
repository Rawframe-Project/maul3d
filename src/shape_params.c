// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape runtime materials and flags: friction, restitution, rolling
// resistance, density, events, pre-solve, mesh materials and surface
// velocity. Public functions validate and journal; replay drives the
// internal setters.

#include "body_state.h"
#include "broad_phase.h"
#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "shape.h"
#include "shape_state.h"
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

void m3SetShapeDensityInternal(m3World* world, int32_t slot, float value)
{
    world->shapes.shapeDensity[slot] = value;
    m3RecomputeMass(world, world->shapes.shapeBody[slot]);
}

// One resolve + one journal + one internal, the body.c pattern.
static bool ResolveShape(const m3World* world, m3ShapeId id, int32_t* slotOut)
{
    int32_t slot = m3ShapeSlot(world, id);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    *slotOut = slot;
    return true;
}

static void ShapeScalarOp(m3World* world, m3ShapeId shapeId, int32_t op, float value)
{
    int32_t slot;
    if (!ResolveShape(world, shapeId, &slot))
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

static M3_NOINLINE void ShapeSetFrictionWorker(m3World* world, m3ShapeId shape, float friction)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (!m3ShapeScalarValid(m3_opSetShapeFriction, friction))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    ShapeScalarOp(world, shape, m3_opSetShapeFriction, friction);
}

void m3Shape_SetFriction(m3World* world, m3ShapeId shape, float friction)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeSetFrictionWorker(world, shape, friction);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE float ShapeGetFrictionWorker(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) ? world->shapes.shapeFriction[slot] : 0.0f;
}

float m3Shape_GetFriction(const m3World* world, m3ShapeId shape)
{
    uint64_t env = m3FloatEnvEnter();
    float result = ShapeGetFrictionWorker(world, shape);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE void ShapeSetRestitutionWorker(m3World* world, m3ShapeId shape,
                                                  float restitution)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (!m3ShapeScalarValid(m3_opSetShapeRestitution, restitution))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    ShapeScalarOp(world, shape, m3_opSetShapeRestitution, restitution);
}

void m3Shape_SetRestitution(m3World* world, m3ShapeId shape, float restitution)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeSetRestitutionWorker(world, shape, restitution);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE float ShapeGetRestitutionWorker(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) ? world->shapes.shapeRestitution[slot] : 0.0f;
}

float m3Shape_GetRestitution(const m3World* world, m3ShapeId shape)
{
    uint64_t env = m3FloatEnvEnter();
    float result = ShapeGetRestitutionWorker(world, shape);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE void ShapeSetRollingResistanceWorker(m3World* world, m3ShapeId shape,
                                                        float value)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    if (!m3ShapeScalarValid(m3_opSetShapeRolling, value))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    ShapeScalarOp(world, shape, m3_opSetShapeRolling, value);
}

void m3Shape_SetRollingResistance(m3World* world, m3ShapeId shape, float value)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeSetRollingResistanceWorker(world, shape, value);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE float ShapeGetRollingResistanceWorker(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) ? world->shapes.shapeRollingResistance[slot] : 0.0f;
}

float m3Shape_GetRollingResistance(const m3World* world, m3ShapeId shape)
{
    uint64_t env = m3FloatEnvEnter();
    float result = ShapeGetRollingResistanceWorker(world, shape);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE void ShapeSetDensityWorker(m3World* world, m3ShapeId shape, float density)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveShape(world, shape, &slot))
    {
        return;
    }
    if (!m3DensityF(density) || !m3ShapeMassFits(world, slot, world->shapes.shapeType[slot],
                                                 &world->shapes.shapeGeom[slot], density))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetShapeDensity record;
        memset(&record, 0, sizeof(record));
        record.id = shape;
        record.value = density;
        m3JournalRecord(world, m3_opSetShapeDensity, &record, (int32_t)sizeof(record));
    }
    m3SetShapeDensityInternal(world, slot, density);
}

void m3Shape_SetDensity(m3World* world, m3ShapeId shape, float density)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeSetDensityWorker(world, shape, density);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE float ShapeGetDensityWorker(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) ? world->shapes.shapeDensity[slot] : 0.0f;
}

float m3Shape_GetDensity(const m3World* world, m3ShapeId shape)
{
    uint64_t env = m3FloatEnvEnter();
    float result = ShapeGetDensityWorker(world, shape);
    m3FloatEnvLeave(env);
    return result;
}

void m3EnableShapeHitEventsInternal(m3World* world, int32_t slot, int32_t on)
{
    world->shapes.shapeHitEvents[slot] = on != 0 ? 1 : 0;
}

void m3EnableShapePreSolveInternal(m3World* world, int32_t slot, int32_t on)
{
    world->shapes.shapePreSolve[slot] = on != 0 ? 1 : 0;
}

static void ShapeFlagOp(m3World* world, m3ShapeId shapeId, int32_t op, bool flag)
{
    int32_t slot;
    if (!ResolveShape(world, shapeId, &slot))
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

static M3_NOINLINE void ShapeEnableHitEventsWorker(m3World* world, m3ShapeId shape, bool flag)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    ShapeFlagOp(world, shape, m3_opEnableShapeHitEvents, flag);
}

void m3Shape_EnableHitEvents(m3World* world, m3ShapeId shape, bool flag)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeEnableHitEventsWorker(world, shape, flag);
    m3FloatEnvLeave(env);
}

bool m3Shape_IsHitEventsEnabled(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) && world->shapes.shapeHitEvents[slot] != 0;
}

static M3_NOINLINE void ShapeEnablePreSolveWorker(m3World* world, m3ShapeId shape, bool flag)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    ShapeFlagOp(world, shape, m3_opEnableShapePreSolve, flag);
}

void m3Shape_EnablePreSolve(m3World* world, m3ShapeId shape, bool flag)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeEnablePreSolveWorker(world, shape, flag);
    m3FloatEnvLeave(env);
}

bool m3Shape_IsPreSolveEnabled(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) && world->shapes.shapePreSolve[slot] != 0;
}

bool m3MeshMaterialsValid(const m3MeshSurfaceMaterial* materials, int32_t materialCount,
                          const uint8_t* triangleMaterials, int32_t triangleCount)
{
    if (materialCount < 1 || materialCount > M3_MESH_MAX_MATERIALS || triangleCount <= 0)
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
    for (int32_t t = 0; t < triangleCount; ++t)
    {
        if (triangleMaterials[t] >= materialCount)
        {
            return false;
        }
    }
    return true;
}

bool m3SetMeshMaterialsInternal(m3World* world, int32_t meshIndex,
                                const m3MeshSurfaceMaterial* materials, int32_t materialCount,
                                const uint8_t* triangleMaterials)
{
    // The full wall, here because replay hands this function
    // raw journal bytes: hostile counts, values, or group indices
    // refuse loudly and paint nothing.
    if (meshIndex < 0)
    {
        return false;
    }
    m3MeshData* mesh = &world->meshes.meshData[meshIndex];
    if (!m3MeshMaterialsValid(materials, materialCount, triangleMaterials, mesh->triangleCount))
    {
        return false;
    }
    mesh->materialCount = materialCount;
    memset(mesh->materials, 0, sizeof(mesh->materials));
    memcpy(mesh->materials, materials, (size_t)materialCount * sizeof(m3MeshSurfaceMaterial));
    memcpy(mesh->triMaterials, triangleMaterials, (size_t)mesh->triangleCount);
    return true;
}

static M3_NOINLINE void ShapeSetMeshMaterialsWorker(m3World* world, m3ShapeId shape,
                                                    const m3MeshSurfaceMaterial* materials,
                                                    int32_t materialCount,
                                                    const uint8_t* triangleMaterials)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveShape(world, shape, &slot))
    {
        return;
    }
    if (materials == NULL || triangleMaterials == NULL ||
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
        // Recorded in two parts, so no allocation can drop the op from
        // the tape after the materials are applied.
        int32_t triCount = world->meshes.meshData[meshIndex].triangleCount;
        m3SetMeshMaterialsOp head;
        memset(&head, 0, sizeof(head));
        head.id = shape;
        head.materialCount = materialCount;
        head.triangleCount = triCount;
        memcpy(head.materials, materials, (size_t)materialCount * sizeof(m3MeshSurfaceMaterial));
        m3JournalPart parts[2] = {{&head, (int32_t)sizeof(head)}, {triangleMaterials, triCount}};
        m3JournalRecordParts(world, m3_opSetMeshMaterials, parts, 2);
    }
}

void m3Shape_SetMeshMaterials(m3World* world, m3ShapeId shape,
                              const m3MeshSurfaceMaterial* materials, int32_t materialCount,
                              const uint8_t* triangleMaterials)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeSetMeshMaterialsWorker(world, shape, materials, materialCount, triangleMaterials);
    m3FloatEnvLeave(env);
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

static M3_NOINLINE void ShapeSetSurfaceVelocityWorker(m3World* world, m3ShapeId shape,
                                                      m3Vec3 velocity)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveShape(world, shape, &slot))
    {
        return;
    }
    if (!m3SurfaceVelocityValid(velocity))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetSurfaceVelocity record;
        memset(&record, 0, sizeof(record));
        record.id = shape;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetSurfaceVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetSurfaceVelocityInternal(world, slot, velocity);
}

void m3Shape_SetSurfaceVelocity(m3World* world, m3ShapeId shape, m3Vec3 velocity)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeSetSurfaceVelocityWorker(world, shape, velocity);
    m3FloatEnvLeave(env);
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

static M3_NOINLINE void ShapeSetFilterWorker(m3World* world, m3ShapeId shape, uint64_t categoryBits,
                                             uint64_t maskBits, int32_t groupIndex)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveShape(world, shape, &slot))
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetFilter record;
        memset(&record, 0, sizeof(record));
        record.categoryBits = categoryBits;
        record.maskBits = maskBits;
        record.id = shape;
        record.groupIndex = groupIndex;
        m3JournalRecord(world, m3_opSetFilter, &record, (int32_t)sizeof(record));
    }
    m3SetFilterInternal(world, slot, categoryBits, maskBits, groupIndex);
}

void m3Shape_SetFilter(m3World* world, m3ShapeId shape, uint64_t categoryBits, uint64_t maskBits,
                       int32_t groupIndex)
{
    uint64_t env = m3FloatEnvEnter();
    ShapeSetFilterWorker(world, shape, categoryBits, maskBits, groupIndex);
    m3FloatEnvLeave(env);
}

void m3Shape_GetFilter(const m3World* world, m3ShapeId shape, uint64_t* categoryBitsOut,
                       uint64_t* maskBitsOut, int32_t* groupIndexOut)
{
    int32_t slot;
    bool live = ResolveShape(world, shape, &slot);
    if (categoryBitsOut != NULL)
    {
        *categoryBitsOut = live ? world->shapes.shapeCategory[slot] : 0u;
    }
    if (maskBitsOut != NULL)
    {
        *maskBitsOut = live ? world->shapes.shapeMask[slot] : 0u;
    }
    if (groupIndexOut != NULL)
    {
        *groupIndexOut = live ? world->shapes.shapeGroup[slot] : 0;
    }
}

m3ShapeType m3Shape_GetType(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) ? (m3ShapeType)world->shapes.shapeType[slot]
                                             : m3_sphereShape;
}

uint64_t m3Shape_GetUserData(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) ? world->shapes.shapeUserData[slot] : 0u;
}

bool m3Shape_IsSensor(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    return ResolveShape(world, shape, &slot) && world->shapes.shapeSensor[slot] != 0;
}

static M3_NOINLINE m3AabbResult ShapeGetAabbWorker(const m3World* world, m3ShapeId shape)
{
    int32_t slot;
    m3AabbResult result = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    if (ResolveShape(world, shape, &slot))
    {
        double lo[3];
        double hi[3];
        m3ShapeFatAabb(world, slot, lo, hi);
        result.lowerBound = (m3Pos3){lo[0], lo[1], lo[2]};
        result.upperBound = (m3Pos3){hi[0], hi[1], hi[2]};
    }
    return result;
}

m3AabbResult m3Shape_GetAabb(const m3World* world, m3ShapeId shape)
{
    uint64_t env = m3FloatEnvEnter();
    m3AabbResult result = ShapeGetAabbWorker(world, shape);
    m3FloatEnvLeave(env);
    return result;
}

void m3Shape_SetUserData(m3World* world, m3ShapeId shape, uint64_t userData)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot;
    if (!ResolveShape(world, shape, &slot))
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpUserData record;
        memset(&record, 0, sizeof(record));
        record.userData = userData;
        record.id = (m3BodyId){shape.index1, shape.generation};
        m3JournalRecord(world, m3_opSetShapeUserData, &record, (int32_t)sizeof(record));
    }
    world->shapes.shapeUserData[slot] = userData;
}
