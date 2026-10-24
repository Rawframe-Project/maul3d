// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Wrappers the suites build scenes with. Each returns what its call
// produced: the new id (null when the call refused), the bytes written
// (-1 when it refused), or whether it succeeded. A suite that checks a
// status calls the m3 function itself.

#ifndef MAUL3D_TEST_HELPERS_H
#define MAUL3D_TEST_HELPERS_H

#include "maul3d/maul3d.h"

static inline m3World* NewWorld(const m3WorldDef* def)
{
    m3World* world = NULL;
    (void)m3CreateWorld(def, &world);
    return world;
}

static inline m3BodyId NewBody(m3World* world, const m3BodyDef* def)
{
    m3BodyId id = m3_nullBodyId;
    (void)m3CreateBody(world, def, &id);
    return id;
}

static inline m3CharacterId NewCharacter(m3World* world, const m3CharacterDef* def)
{
    m3CharacterId id = m3_nullCharacterId;
    (void)m3CreateCharacter(world, def, &id);
    return id;
}

static inline m3JointId NewJoint(m3World* world, const m3JointDef* def)
{
    m3JointId id = m3_nullJointId;
    (void)m3CreateJoint(world, def, &id);
    return id;
}

static inline m3ShapeId NewSphereShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                       const m3Sphere* sphere)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateSphereShape(world, body, def, sphere, &id);
    return id;
}

static inline m3ShapeId NewPlaneShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                      const m3Plane* plane)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreatePlaneShape(world, body, def, plane, &id);
    return id;
}

static inline m3ShapeId NewBoxShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                    m3Vec3 halfExtents)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateBoxShape(world, body, def, halfExtents, &id);
    return id;
}

static inline m3ShapeId NewCapsuleShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                        const m3Capsule* capsule)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateCapsuleShape(world, body, def, capsule, &id);
    return id;
}

static inline m3ShapeId NewCylinderShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                         const m3Cylinder* cylinder, int32_t segments)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateCylinderShape(world, body, def, cylinder, segments, &id);
    return id;
}

static inline m3ShapeId NewHullShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                     const m3Vec3* points, int32_t count)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateHullShape(world, body, def, points, count, &id);
    return id;
}

static inline m3ShapeId NewMeshShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                     const m3Vec3* vertices, int32_t vertexCount,
                                     const uint16_t* indices, int32_t triangleCount)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateMeshShape(world, body, def, vertices, vertexCount, indices, triangleCount, &id);
    return id;
}

static inline m3ShapeId NewHeightFieldShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                            const float* heights, int32_t nx, int32_t nz,
                                            m3real cellSize)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateHeightFieldShape(world, body, def, heights, nx, nz, cellSize, &id);
    return id;
}

static inline m3ShapeId NewHeightFieldGridShape(m3World* world, m3BodyId body,
                                                const m3ShapeDef* def, const float* heights,
                                                int32_t nx, int32_t nz, m3real cellSize)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateHeightFieldGridShape(world, body, def, heights, nx, nz, cellSize, &id);
    return id;
}

static inline m3ShapeId NewVoxelChunkShape(m3World* world, m3BodyId body, const m3ShapeDef* def,
                                           const uint8_t* voxels, const uint16_t* payload,
                                           m3real cellSize)
{
    m3ShapeId id = m3_nullShapeId;
    (void)m3CreateVoxelChunkShape(world, body, def, voxels, payload, cellSize, &id);
    return id;
}

static inline m3SoftBodyId NewSoftBody(m3World* world, const m3SoftBodyDef* def)
{
    m3SoftBodyId id = m3_nullSoftBodyId;
    (void)m3CreateSoftBody(world, def, &id);
    return id;
}

static inline m3SoftBodyId NewSoftBodyTet(m3World* world, const m3SoftBodyDef* def,
                                          const m3Vec3* points, int32_t pointCount,
                                          const uint16_t* tets, int32_t tetCount)
{
    m3SoftBodyId id = m3_nullSoftBodyId;
    (void)m3CreateSoftBodyTet(world, def, points, pointCount, tets, tetCount, &id);
    return id;
}

static inline m3VehicleId NewVehicle(m3World* world, const m3VehicleDef* def)
{
    m3VehicleId id = m3_nullVehicleId;
    (void)m3CreateVehicle(world, def, &id);
    return id;
}

static inline m3WaterVolumeId NewWaterVolume(m3World* world, const m3WaterVolumeDef* def)
{
    m3WaterVolumeId id = m3_nullWaterVolumeId;
    (void)m3CreateWaterVolume(world, def, &id);
    return id;
}

static inline void DestroyBody(m3World* world, m3BodyId body)
{
    (void)m3DestroyBody(world, body);
}

static inline void DestroyCharacter(m3World* world, m3CharacterId character)
{
    (void)m3DestroyCharacter(world, character);
}

static inline void DestroyJoint(m3World* world, m3JointId joint)
{
    (void)m3DestroyJoint(world, joint);
}

static inline void DestroyShape(m3World* world, m3ShapeId shape)
{
    (void)m3DestroyShape(world, shape);
}

static inline void DestroySoftBody(m3World* world, m3SoftBodyId softBody)
{
    (void)m3DestroySoftBody(world, softBody);
}

static inline void DestroyVehicle(m3World* world, m3VehicleId vehicle)
{
    (void)m3DestroyVehicle(world, vehicle);
}

static inline void DestroyWaterVolume(m3World* world, m3WaterVolumeId volume)
{
    (void)m3DestroyWaterVolume(world, volume);
}

static inline int32_t Snapshot(const m3World* world, void* buffer, int32_t capacity)
{
    int32_t bytes = 0;
    return m3World_Snapshot(world, buffer, capacity, &bytes) == m3_success ? bytes : -1;
}

static inline bool Restore(m3World* world, const void* data, int32_t size)
{
    return m3World_Restore(world, data, size) == m3_success;
}

static inline bool StartJournal(m3World* world, void* buffer, int32_t capacity)
{
    return m3World_StartJournal(world, buffer, capacity) == m3_success;
}

static inline int32_t StopJournal(m3World* world)
{
    int32_t bytes = 0;
    return m3World_StopJournal(world, &bytes) == m3_success ? bytes : -1;
}

static inline bool ReplayJournal(m3World* world, const void* data, int32_t size)
{
    return m3World_ReplayJournal(world, data, size) == m3_success;
}

static inline bool DescribeJournal(const void* journal, int32_t bytes, m3JournalInfo* info)
{
    return m3DescribeJournal(journal, bytes, info) == m3_success;
}

static inline int32_t EncodeReplay(const void* snapshot, int32_t snapshotBytes, const void* journal,
                                   int32_t journalBytes, uint64_t finalHash, void* out,
                                   int32_t capacity)
{
    int32_t bytes = 0;
    m3Result result = m3EncodeReplay(snapshot, snapshotBytes, journal, journalBytes, finalHash, out,
                                     capacity, &bytes);
    return result == m3_success ? bytes : 0;
}

static inline bool DecodeReplay(const void* data, int32_t bytes, m3ReplayView* view)
{
    return m3DecodeReplay(data, bytes, view) == m3_success;
}

static inline bool SetStance(m3World* world, m3CharacterId character, m3real halfHeight,
                             m3real radius)
{
    return m3Character_SetStance(world, character, halfHeight, radius) == m3_success;
}

// Checks that a call refused: it counted one misuse against the world.
// These two expand to CHECK, so a suite includes test_harness.h for them.
#define CHECK_REFUSES(world, call, msg)                                                            \
    do                                                                                             \
    {                                                                                              \
        uint64_t misuseBefore = m3World_GetCounters(world).misuse;                                 \
        call;                                                                                      \
        CHECK(m3World_GetCounters(world).misuse == misuseBefore + 1u, msg);                        \
    } while (0)

// Checks that a call was taken: it counted no misuse.
#define CHECK_ACCEPTS(world, call, msg)                                                            \
    do                                                                                             \
    {                                                                                              \
        uint64_t misuseBefore = m3World_GetCounters(world).misuse;                                 \
        call;                                                                                      \
        CHECK(m3World_GetCounters(world).misuse == misuseBefore, msg);                             \
    } while (0)

#endif // MAUL3D_TEST_HELPERS_H
