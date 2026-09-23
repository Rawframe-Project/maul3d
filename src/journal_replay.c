// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay: one apply function per op (or per family of ops that
// share a payload) and the command table that maps op codes to them.
// Every apply validates its payload as hostile bytes and re-enters the
// same internal functions the public API uses; created ids must match
// the ids the recording minted.

#include "body.h"
#include "character.h"
#include "joint.h"
#include "journal.h"
#include "query.h"
#include "quickhull.h"
#include "shape.h"
#include "softbody.h"
#include "solver.h"
#include "vehicle.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <stddef.h>
#include <string.h>

// One record as the loop found it: the op code, its payload and size.
typedef struct m3ReplayRecord
{
    int32_t op;
    const uint8_t* payload;
    int32_t bytes;
} m3ReplayRecord;

typedef bool m3ReplayApplyFn(m3World* world, const m3ReplayRecord* r);

// Journaled defs are untrusted bytes: a
// flipped bit in an embedded bool field is undefined even to LOAD
// as _Bool, so every replay handler normalizes bool bytes through
// uint8_t before the def is used as its C type. UBSAN convicted
// the raw load on the container fuzzer's first day.
static void NormalizeBoolByte(void* base, size_t offset)
{
    uint8_t* b = (uint8_t*)base + offset;
    *b = *b != 0 ? 1 : 0;
}

static void NormalizeShapeDefBools(m3ShapeDef* def)
{
    NormalizeBoolByte(def, offsetof(m3ShapeDef, isSensor));
    NormalizeBoolByte(def, offsetof(m3ShapeDef, enableHitEvents));
    NormalizeBoolByte(def, offsetof(m3ShapeDef, enablePreSolveEvents));
}

static bool ApplyCreateBody(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3BodyDef def;
        m3BodyId expected;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    NormalizeBoolByte(&record.def, offsetof(m3BodyDef, isBullet));
    int32_t index = m3CreateBodyInternal(world, &record.def);
    // Id determinism: the replayed world must mint the exact
    // id the original minted, or the replay is invalid.
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->bodies.bodyPool.generations[index] != record.expected.generation)
    {
        return false;
    }
    return true;
}

static bool ApplyDestroyBody(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3BodyId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    // Recorded ids carry the ORIGINAL world's slot; replay
    // retargets them to this world (the Maul2D rule).
    id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, id);
    if (index < 0)
    {
        return false;
    }
    m3DestroyBodyInternal(world, index);
    return true;
}

static bool ApplySetLinearVelocity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3BodyId id;
        m3Vec3 v;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3FiniteV3(record.v))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetLinearVelocityInternal(world, index, record.v);
    return true;
}

static bool ApplySetAngularVelocity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3BodyId id;
        m3Vec3 v;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3FiniteV3(record.v))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetAngularVelocityInternal(world, index, record.v);
    return true;
}

static bool ApplyStepVetoes(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    if (bytes <= 0 || (bytes % 8) != 0 || bytes / 8 > world->contacts.pairCapacity)
    {
        return false; // hostile veto list refuses loudly
    }
    memcpy(world->contacts.replayVetoKeys, payload, (size_t)bytes);
    world->contacts.replayVetoCount = bytes / 8;
    return true;
}

static bool ApplyStep(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        float dt;
        int32_t substeps;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (!(record.dt > 0.0f) || record.substeps < 1 || record.substeps > M3_MAX_SUBSTEPS)
    {
        // The upper bound is the 13-4 red-team scar: one
        // flipped bit turned substeps 4 into a billion and a
        // two-second replay into hours. Hostile tapes refuse
        // in proportion to their crime.
        return false;
    }
    m3StepInternal(world, record.dt, record.substeps);
    return true;
}

static bool ApplyCreateShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateShapeOp record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.body.world0 = world->worldIndex0;
    int32_t bodyIndex = m3BodySlot(world, record.body);
    if (bodyIndex < 0)
    {
        return false;
    }
    NormalizeShapeDefBools(&record.def);
    int32_t index = m3CreateShapeInternal(world, bodyIndex, record.type, &record.geom, &record.def,
                                          NULL, NULL, NULL, NULL);
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->shapes.shapePool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for shapes too
    }
    return true;
}

static bool ApplyCreateMeshShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateMeshShapeOp record;
    if (bytes < (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (record.vertexCount < 3 || record.vertexCount > M3_MESH_MAX_VERTS ||
        record.triangleCount < 1 || record.triangleCount > M3_MESH_MAX_TRIS)
    {
        return false;
    }
    int32_t vertexBytes = record.vertexCount * (int32_t)sizeof(m3Vec3);
    int32_t indexBytes = 3 * record.triangleCount * (int32_t)sizeof(uint16_t);
    if (bytes != (int32_t)sizeof(record) + vertexBytes + indexBytes)
    {
        return false;
    }
    record.body.world0 = world->worldIndex0;
    int32_t bodyIndex = m3BodySlot(world, record.body);
    if (bodyIndex < 0)
    {
        return false;
    }
    m3MeshData mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.vertexCount = record.vertexCount;
    mesh.triangleCount = record.triangleCount;
    if (!m3MeshDataAlloc(&mesh))
    {
        return false;
    }
    memcpy(mesh.vertices, (const uint8_t*)payload + sizeof(record), (size_t)vertexBytes);
    memcpy(mesh.indices, (const uint8_t*)payload + sizeof(record) + vertexBytes,
           (size_t)indexBytes);
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    NormalizeShapeDefBools(&record.def);
    // On success the slot owns the arrays; an id mismatch
    // leaves them with the slot too, and the atomic-replay
    // restore reclaims them through the alloc gate.
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_meshShape, &geom,
                                          &record.def, NULL, &mesh, NULL, NULL);
    if (index < 0)
    {
        m3MeshDataFree(&mesh);
        return false;
    }
    if (index + 1 != record.expected.index1 ||
        world->shapes.shapePool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for mesh shapes too
    }
    return true;
}

static bool ApplyCreateJoint(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateJointOp record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableLimit));
    NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableMotor));
    NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableCone));
    NormalizeBoolByte(&record.def, offsetof(m3JointDef, collideConnected));
    record.def.bodyA.world0 = world->worldIndex0;
    record.def.bodyB.world0 = world->worldIndex0;
    int32_t bodyA = m3BodySlot(world, record.def.bodyA);
    int32_t bodyB = m3BodySlot(world, record.def.bodyB);
    if (bodyA < 0 || bodyB < 0)
    {
        return false;
    }
    int32_t index = m3CreateJointInternal(world, &record.def, bodyA, bodyB);
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->joints.jointPool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for joints too
    }
    return true;
}

static bool ApplyDestroyJoint(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3JointId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world0 = world->worldIndex0;
    int32_t index = m3JointSlot(world, id);
    if (index < 0)
    {
        return false;
    }
    m3DestroyJointInternal(world, index);
    return true;
}

static bool ApplyCreateHullShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateHullShapeOp record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.body.world0 = world->worldIndex0;
    int32_t bodyIndex = m3BodySlot(world, record.body);
    if (bodyIndex < 0)
    {
        return false;
    }
    m3HullData rebuilt;
    if (!m3ComputeHull(record.points, record.count, &rebuilt))
    {
        return false; // the recipe must rebuild
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    NormalizeShapeDefBools(&record.def);
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_hullShape, &geom,
                                          &record.def, &rebuilt, NULL, NULL, NULL);
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->shapes.shapePool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for hull shapes too
    }
    return true;
}

static bool ApplyCreateVoxelChunkShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3ShapeDef def;
        m3BodyId body;
        m3ShapeId expected;
        m3real cellSize;
    } record;
    int32_t occBytes = (int32_t)(M3_VOXEL_COUNT / 8);
    int32_t payBytes = (int32_t)(M3_VOXEL_COUNT * sizeof(uint16_t));
    int32_t fillBytes = (int32_t)M3_VOXEL_COUNT;
    if (bytes != (int32_t)sizeof(record) + occBytes + payBytes + fillBytes)
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.body.world0 = world->worldIndex0;
    int32_t bodyIndex = m3BodySlot(world, record.body);
    if (bodyIndex < 0 || !(record.cellSize > 0.0f))
    {
        return false;
    }
    m3VoxelChunkData* chunk = (m3VoxelChunkData*)m3AllocZeroed((int32_t)sizeof(m3VoxelChunkData));
    if (chunk == NULL)
    {
        return false;
    }
    chunk->cellSize = record.cellSize;
    memcpy(chunk->occupancy, (const uint8_t*)payload + sizeof(record), (size_t)occBytes);
    memcpy(chunk->payload, (const uint8_t*)payload + sizeof(record) + occBytes, (size_t)payBytes);
    memcpy(chunk->fill, (const uint8_t*)payload + sizeof(record) + occBytes + payBytes,
           (size_t)fillBytes);
    int32_t filled = 0;
    for (int32_t v = 0; v < M3_VOXEL_COUNT; ++v)
    {
        filled += (chunk->occupancy[v >> 3] >> (v & 7)) & 1;
    }
    chunk->filledCount = filled;
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.s = record.cellSize;
    NormalizeShapeDefBools(&record.def);
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_voxelShape, &geom,
                                          &record.def, NULL, NULL, chunk, NULL);
    m3Free(chunk);
    if (index < 0 || index + 1 != record.expected.index1 ||
        world->shapes.shapePool.generations[index] != record.expected.generation)
    {
        return false; // id determinism holds for voxels too
    }
    return true;
}

static bool ApplyVoxelSet(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3ShapeId id;
        int32_t x, y, z;
        uint16_t payload;
        uint16_t pad;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t shape = m3ShapeSlot(world, record.id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        return false;
    }
    m3VoxelSetInternal(world, shape, record.x, record.y, record.z, record.payload);
    return true;
}

static bool ApplyVoxelClear(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3ShapeId id;
        int32_t x, y, z;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t shape = m3ShapeSlot(world, record.id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        return false;
    }
    m3VoxelClearInternal(world, shape, record.x, record.y, record.z);
    return true;
}

static bool ApplyVoxelSetFill(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3ShapeId id;
        int32_t x, y, z;
        uint8_t fill;
        uint8_t pad[3];
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t shape = m3ShapeSlot(world, record.id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape || record.fill == 0)
    {
        return false;
    }
    m3VoxelSetFillInternal(world, shape, record.x, record.y, record.z, record.fill);
    return true;
}

static bool ApplyVoxelClearBox(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3ShapeId id;
        int32_t lo[3];
        int32_t hi[3];
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t shape = m3ShapeSlot(world, record.id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        return false;
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        if (record.lo[k] < 0 || record.hi[k] >= M3_VOXEL_DIM || record.lo[k] > record.hi[k])
        {
            return false;
        }
    }
    m3VoxelClearBoxInternal(world, shape, record.lo, record.hi);
    return true;
}

static bool ApplyCreateCharacter(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3CharacterDef def;
        m3CharacterId expected;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3CreateCharacterInternal(world, &record.def);
    if (slot < 0 || slot + 1 != record.expected.index1 ||
        world->characters.charPool.generations[slot] != record.expected.generation)
    {
        return false; // id determinism holds for characters too
    }
    return true;
}

static bool ApplyDestroyCharacter(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CharacterId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world0 = world->worldIndex0;
    int32_t slot = m3CharacterSlot(world, id);
    if (slot < 0)
    {
        return false;
    }
    m3DestroyCharacterInternal(world, slot);
    return true;
}

static bool ApplyCharacterMove(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3CharacterId id;
        m3Vec3 translation;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3CharacterSlot(world, record.id);
    if (slot < 0 || !m3FiniteV3(record.translation))
    {
        return false; // hostile bytes fail loudly
    }
    m3CharacterMoveInternal(world, slot, record.translation);
    return true;
}

static bool ApplyCharacterStance(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3CharacterId id;
        m3real halfHeight;
        m3real radius;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3CharacterSlot(world, record.id);
    if (slot < 0 || !m3CharacterStanceInternal(world, slot, record.halfHeight, record.radius))
    {
        // A journaled stance was APPLIED at record time; a
        // replay that cannot re-apply it (fuzzed bytes, a
        // diverged world) fails loudly.
        return false;
    }
    return true;
}

static bool ApplyCreateVehicle(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3VehicleDef def;
        m3VehicleId expected;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        size_t wheelBase = offsetof(m3VehicleDef, wheels) + (size_t)w * sizeof(m3WheelDef);
        NormalizeBoolByte(&record.def, wheelBase + offsetof(m3WheelDef, steerable));
        NormalizeBoolByte(&record.def, wheelBase + offsetof(m3WheelDef, driven));
    }
    record.def.chassis.world0 = world->worldIndex0;
    int32_t slot = m3CreateVehicleInternal(world, &record.def);
    if (slot < 0 || slot + 1 != record.expected.index1 ||
        world->vehicles.vehPool.generations[slot] != record.expected.generation)
    {
        return false; // id determinism holds for vehicles too
    }
    return true;
}

static bool ApplyDestroyVehicle(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3VehicleId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world0 = world->worldIndex0;
    int32_t slot = m3VehicleSlot(world, id);
    if (slot < 0)
    {
        return false;
    }
    m3DestroyVehicleInternal(world, slot);
    return true;
}

static bool ApplyVehicleTankCommands(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3VehicleId id;
        m3real left;
        m3real right;
        m3real brake;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3VehicleSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.left) || !m3FiniteF(record.right) || !m3FiniteF(record.brake))
    {
        return false; // hostile bytes fail loudly
    }
    m3VehicleTankCommandsInternal(world, slot, record.left, record.right, record.brake);
    return true;
}

static bool ApplyVehicleCommands(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3VehicleId id;
        m3real throttle;
        m3real steer;
        m3real brake;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3VehicleSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.throttle) || !m3FiniteF(record.steer) ||
        !m3FiniteF(record.brake))
    {
        return false; // hostile bytes fail loudly
    }
    m3VehicleCommandsInternal(world, slot, record.throttle, record.steer, record.brake);
    return true;
}

static bool ApplyVehicleDrivetrain(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3VehicleId id;
        m3DrivetrainDef def;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    NormalizeBoolByte(&record.def, offsetof(m3DrivetrainDef, autoShift));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3VehicleSlot(world, record.id);
    if (slot < 0 || !m3VehicleDrivetrainInternal(world, slot, &record.def))
    {
        return false; // journaled defs are UNTRUSTED bytes
    }
    return true;
}

static bool ApplyVehicleGear(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3VehicleId id;
        int32_t gear;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3VehicleSlot(world, record.id);
    if (slot < 0 || !m3VehicleGearInternal(world, slot, record.gear))
    {
        return false;
    }
    return true;
}

static bool ApplyCreateSoftBody(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3SoftBodyDef def;
        m3SoftBodyId expected;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3CreateSoftBodyInternal(world, &record.def);
    if (slot < 0 || slot + 1 != record.expected.index1 ||
        world->softBodies.softPool.generations[slot] != record.expected.generation)
    {
        return false; // id determinism holds for soft bodies too
    }
    return true;
}

static bool ApplyDestroySoftBody(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3SoftBodyId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world0 = world->worldIndex0;
    int32_t slot = m3SoftBodySlot(world, id);
    if (slot < 0)
    {
        return false;
    }
    m3DestroySoftBodyInternal(world, slot);
    return true;
}

static bool ApplySoftBodyPin(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3SoftBodyId id;
        int32_t particle;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3SoftBodySlot(world, record.id);
    if (slot < 0 || record.particle < 0 ||
        record.particle >= world->softBodies.softParticleCount[slot])
    {
        return false;
    }
    m3SoftBodyPinInternal(world, slot, record.particle);
    return true;
}

static bool ApplySoftBodyAnchor(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3SoftBodyId id;
        int32_t particle;
        m3BodyId body;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    record.body.world0 = world->worldIndex0;
    int32_t slot = m3SoftBodySlot(world, record.id);
    int32_t body = m3BodySlot(world, record.body);
    if (slot < 0 || body < 0 || record.particle < 0 ||
        record.particle >= world->softBodies.softParticleCount[slot] ||
        world->softBodies.softAnchorCount[slot] >= M3_SOFTBODY_MAX_ANCHORS)
    {
        return false;
    }
    m3SoftBodyAnchorInternal(world, slot, record.particle, body);
    return true;
}

static bool ApplySoftBodyAnchorSoft(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3SoftBodyId idA;
        int32_t particleA;
        m3SoftBodyId idB;
        int32_t particleB;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.idA.world0 = world->worldIndex0;
    record.idB.world0 = world->worldIndex0;
    int32_t slotA = m3SoftBodySlot(world, record.idA);
    int32_t slotB = m3SoftBodySlot(world, record.idB);
    if (slotA < 0 || slotB < 0 || slotA == slotB || record.particleA < 0 || record.particleB < 0 ||
        record.particleA >= world->softBodies.softParticleCount[slotA] ||
        record.particleB >= world->softBodies.softParticleCount[slotB] ||
        world->softBodies.softSoftCount[slotA < slotB ? slotA : slotB] >= M3_SOFTBODY_MAX_ANCHORS)
    {
        // A flipped particle index would become an out of
        // bounds solver read: the full public wall.
        return false;
    }
    m3SoftBodyAnchorSoftInternal(world, slotA, record.particleA, slotB, record.particleB);
    return true;
}

static bool ApplyBodyVector(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    struct
    {
        m3BodyId id;
        m3Vec3 v;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3FiniteV3(record.v))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opApplyForce)
    {
        m3ApplyForceInternal(world, index, record.v);
    }
    else if (op == m3_opApplyTorque)
    {
        m3ApplyTorqueInternal(world, index, record.v);
    }
    else if (op == m3_opApplyLinearImpulse)
    {
        m3ApplyLinearImpulseInternal(world, index, record.v);
    }
    else
    {
        m3ApplyAngularImpulseInternal(world, index, record.v);
    }
    return true;
}

static bool ApplyBodyVectorAtPoint(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    struct
    {
        m3BodyId id;
        m3Vec3 v;
        m3Pos3 p;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0 || !m3FiniteV3(record.v) || !m3FinitePos3(record.p))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opApplyForceAtPoint)
    {
        m3ApplyForceAtPointInternal(world, index, record.v, record.p);
    }
    else
    {
        m3ApplyImpulseAtPointInternal(world, index, record.v, record.p);
    }
    return true;
}

static bool ApplyBodyPose(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    struct
    {
        m3BodyId id;
        m3Transform pose;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    float qq = record.pose.q.x * record.pose.q.x + record.pose.q.y * record.pose.q.y +
               record.pose.q.z * record.pose.q.z + record.pose.q.w * record.pose.q.w;
    if (index < 0 || !m3FinitePos3(record.pose.p) || !m3FiniteQuat(record.pose.q) ||
        !(qq > 0.98f) || !(qq < 1.02f))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opSetTransform)
    {
        m3SetTransformInternal(world, index, record.pose);
    }
    else
    {
        m3SetTargetTransformInternal(world, index, record.pose);
    }
    return true;
}

static bool ApplyBodyByte(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    struct
    {
        m3BodyId id;
        int32_t value;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    if (op == m3_opSetType)
    {
        m3SetTypeInternal(world, index, (uint8_t)record.value);
    }
    else if (op == m3_opSetEnabled)
    {
        m3SetEnabledInternal(world, index, record.value);
    }
    else
    {
        m3SetAwakeInternal(world, index, record.value);
    }
    return true;
}

static bool ApplySetMotionLocks(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3BodyId id;
        uint32_t locks;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    m3SetMotionLocksInternal(world, index, (uint8_t)record.locks);
    return true;
}

static bool ApplySetSleepControls(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3BodyId id;
        float threshold;
        int32_t canSleep;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    m3SetSleepControlsInternal(world, index, record.threshold, record.canSleep);
    return true;
}

static bool ApplyDestroyShape(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3ShapeId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world0 = world->worldIndex0;
    int32_t index = m3ShapeSlot(world, id);
    if (index < 0)
    {
        return false;
    }
    int32_t bodyIndex = world->shapes.shapeBody[index];
    m3DestroyShapeInternal(world, index);
    m3RecomputeMass(world, bodyIndex);
    if (world->bodies.types[bodyIndex] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, bodyIndex, 1);
    }
    return true;
}

static bool ApplySetGravity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3Vec3 gravity;
    if (bytes != (int32_t)sizeof(gravity))
    {
        return false;
    }
    memcpy(&gravity, payload, sizeof(gravity));
    if (!m3FiniteV3(gravity))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetGravityInternal(world, gravity);
    return true;
}

static bool ApplyShapeScalar(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    struct
    {
        m3ShapeId id;
        float value;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.value) || record.value < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opSetShapeFriction)
    {
        m3SetShapeFrictionInternal(world, slot, record.value);
    }
    else if (op == m3_opSetShapeRestitution)
    {
        m3SetShapeRestitutionInternal(world, slot, record.value);
    }
    else
    {
        m3SetShapeRollingInternal(world, slot, record.value);
    }
    return true;
}

static bool ApplySetShapeDensity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3ShapeId id;
        float value;
        int32_t updateMass;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.value) || record.value <= 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetShapeDensityInternal(world, slot, record.value, record.updateMass);
    return true;
}

static bool ApplySetContactTuning(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        float hertz;
        float dampingRatio;
        float pushSpeed;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (!m3FiniteF(record.hertz) || record.hertz <= 0.0f || !m3FiniteF(record.dampingRatio) ||
        record.dampingRatio <= 0.0f || !m3FiniteF(record.pushSpeed) || record.pushSpeed <= 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetContactTuningInternal(world, record.hertz, record.dampingRatio, record.pushSpeed);
    return true;
}

static bool ApplyWorldScalar(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    float value;
    if (bytes != (int32_t)sizeof(value))
    {
        return false;
    }
    memcpy(&value, payload, sizeof(value));
    if (!m3FiniteF(value) || (op == m3_opSetRestitutionThreshold && value < 0.0f) ||
        (op == m3_opSetMaximumLinearSpeed && value <= 0.0f))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opSetRestitutionThreshold)
    {
        m3SetRestitutionThresholdInternal(world, value);
    }
    else
    {
        m3SetMaximumLinearSpeedInternal(world, value);
    }
    return true;
}

static bool ApplySetMaximumAngularSpeed(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    // A new op gets the strict wall: hostile caps (NaN,
    // nonpositive) fail the replay loudly instead of riding
    // into the solver. Op 46 keeps its 8-4 byte contract.
    float value;
    if (bytes != (int32_t)sizeof(value))
    {
        return false;
    }
    memcpy(&value, payload, sizeof(value));
    if (!m3FiniteF(value) || value <= 0.0f)
    {
        return false;
    }
    m3SetMaximumAngularSpeedInternal(world, value);
    return true;
}

static bool ApplySetBodyName(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3BodyId id;
        char name[M3_BODY_NAME_CAPACITY];
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    record.name[M3_BODY_NAME_CAPACITY - 1] = 0;
    m3SetBodyNameInternal(world, index, record.name);
    return true;
}

static bool ApplyCreateSoftBodyTet(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateSoftBodyTetOp head;
    if (bytes < (int32_t)sizeof(head))
    {
        return false;
    }
    memcpy(&head, payload, sizeof(head));
    if (head.pointCount < 4 || head.pointCount > M3_SOFTBODY_MAX_PARTICLES || head.tetCount < 1 ||
        head.tetCount > M3_SOFTBODY_MAX_TETS ||
        bytes != (int32_t)sizeof(head) + head.pointCount * (int32_t)sizeof(m3Vec3) +
                     4 * head.tetCount * (int32_t)sizeof(uint16_t))
    {
        return false;
    }
    // Journal records are byte-packed: typed pointers into the
    // stream are misaligned UB (the replayfile fuzz caught the
    // heightfield twin of this line). Aligned stack copies,
    // bounded by the walls just checked.
    m3Vec3 pts[M3_SOFTBODY_MAX_PARTICLES];
    uint16_t tets[4 * M3_SOFTBODY_MAX_TETS];
    memcpy(pts, (const uint8_t*)payload + sizeof(head), (size_t)head.pointCount * sizeof(m3Vec3));
    memcpy(tets, (const uint8_t*)payload + sizeof(head) + (size_t)head.pointCount * sizeof(m3Vec3),
           (size_t)(4 * head.tetCount) * sizeof(uint16_t));
    int32_t slot =
        m3CreateSoftBodyTetInternal(world, &head.def, pts, head.pointCount, tets, head.tetCount);
    if (slot < 0 || slot + 1 != head.expected.index1 ||
        world->softBodies.softPool.generations[slot] != head.expected.generation)
    {
        return false; // id determinism holds for jelly too
    }
    return true;
}

static bool ApplyCreateHeightFieldGrid(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CreateHeightFieldGridOp head;
    if (bytes < (int32_t)sizeof(head))
    {
        return false;
    }
    memcpy(&head, payload, sizeof(head));
    NormalizeShapeDefBools(&head.def);
    head.body.world0 = world->worldIndex0;
    int32_t bodyIndex = m3BodySlot(world, head.body);
    if (bodyIndex < 0 || world->bodies.types[bodyIndex] != (uint8_t)m3_staticBody || head.nx < 2 ||
        head.nx > M3_HEIGHTFIELD_MAX_DIM || head.nz < 2 || head.nz > M3_HEIGHTFIELD_MAX_DIM ||
        !m3FiniteF(head.cellSize) || !(head.cellSize > 0.0f) ||
        bytes != (int32_t)sizeof(head) + head.nx * head.nz * (int32_t)sizeof(float))
    {
        return false; // hostile grid bytes fail loudly
    }
    // Same alignment law as the tet decode above: the stream
    // is byte-packed, so every sample is read by memcpy.
    const uint8_t* sampleBytes = (const uint8_t*)payload + sizeof(head);
    float mn = 0.0f;
    float mx = 0.0f;
    for (int32_t i = 0; i < head.nx * head.nz; ++i)
    {
        float v;
        memcpy(&v, sampleBytes + (size_t)i * sizeof(float), sizeof(float));
        if (!m3FiniteF(v))
        {
            return false;
        }
        mn = i == 0 ? v : (v < mn ? v : mn);
        mx = i == 0 ? v : (v > mx ? v : mx);
    }
    m3HeightFieldData hf;
    memset(&hf, 0, sizeof(hf));
    hf.nx = head.nx;
    hf.nz = head.nz;
    hf.cellSize = head.cellSize;
    hf.minHeight = mn;
    hf.maxHeight = mx;
    if (!m3HeightFieldDataAlloc(&hf))
    {
        return false;
    }
    memcpy(hf.heights, sampleBytes, (size_t)(head.nx * head.nz) * sizeof(float));
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_heightFieldShape, &geom,
                                          &head.def, NULL, NULL, NULL, &hf);
    if (index < 0)
    {
        m3HeightFieldDataFree(&hf);
        return false;
    }
    if (index + 1 != head.expected.index1 ||
        world->shapes.shapePool.generations[index] != head.expected.generation)
    {
        return false; // id determinism holds for terrain too
    }
    return true;
}

static bool ApplyCreateWaterVolume(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3WaterVolumeDef def;
        m3WaterVolumeId expected;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    int32_t slot = m3CreateWaterVolumeInternal(world, &record.def);
    if (slot < 0 || slot + 1 != record.expected.index1 ||
        world->water.waterPool.generations[slot] != record.expected.generation)
    {
        return false; // id determinism holds for water too
    }
    return true;
}

static bool ApplyDestroyWaterVolume(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3WaterVolumeId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world0 = world->worldIndex0;
    int32_t index = id.index1 - 1;
    if (!m3IdPoolValid(&world->water.waterPool, index, id.generation))
    {
        return false;
    }
    m3DestroyWaterVolumeInternal(world, index);
    return true;
}

static bool ApplyRebuildBroadphase(m3World* world, const m3ReplayRecord* r)
{
    int32_t bytes = r->bytes;
    int32_t zero;
    if (bytes != (int32_t)sizeof(zero))
    {
        return false;
    }
    m3RebuildBroadphaseInternal(world);
    return true;
}

static bool ApplySetMeshMaterials(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3SetMeshMaterialsOp head;
    if (bytes < (int32_t)sizeof(head))
    {
        return false;
    }
    memcpy(&head, payload, sizeof(head));
    head.id.world0 = world->worldIndex0;
    int32_t slot = m3ShapeSlot(world, head.id);
    if (slot < 0 || world->shapes.shapeType[slot] != (uint8_t)m3_meshShape)
    {
        return false;
    }
    int32_t meshIndex = world->shapes.shapeMeshIndex[slot];
    if (meshIndex < 0 || head.triangleCount != world->meshes.meshData[meshIndex].triangleCount ||
        bytes != (int32_t)sizeof(head) + head.triangleCount)
    {
        return false; // the byte array must match THIS mesh
    }
    if (!m3SetMeshMaterialsInternal(world, meshIndex, head.materials, head.materialCount,
                                    (const uint8_t*)payload + sizeof(head)))
    {
        return false; // hostile paint fails loudly
    }
    return true;
}

static bool ApplyJointSetMotorPose(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3JointId id;
        m3Vec3 offset;
        m3Quat rotation;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3JointSlot(world, record.id);
    float q2 = record.rotation.x * record.rotation.x + record.rotation.y * record.rotation.y +
               record.rotation.z * record.rotation.z + record.rotation.w * record.rotation.w;
    if (slot < 0 || world->joints.jointType[slot] != (uint8_t)m3_motorJoint ||
        !m3FiniteV3(record.offset) || !m3FiniteQuat(record.rotation) || q2 < 0.81f || q2 > 1.21f)
    {
        return false; // hostile servo bytes fail loudly
    }
    m3JointSetMotorPoseInternal(world, slot, record.offset, record.rotation);
    return true;
}

static bool ApplyJointSetSteer(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3JointId id;
        int32_t enable;
        float target;
        float hertz;
        float zeta;
        float effort;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || world->joints.jointType[slot] != (uint8_t)m3_wheelJoint ||
        !m3FiniteF(record.target) || m3AbsF(record.target) > 1.0f || !m3FiniteF(record.hertz) ||
        !m3FiniteF(record.zeta) || !m3FiniteF(record.effort) || record.zeta < 0.0f ||
        record.effort < 0.0f || (record.enable != 0 && !(record.hertz > 0.0f)) ||
        (record.enable != 0 && record.enable != 1))
    {
        return false; // hostile steer bytes fail loudly
    }
    m3JointSetSteerInternal(world, slot, record.enable, record.target, record.hertz, record.zeta,
                            record.effort);
    return true;
}

static bool ApplySetShapeGeom(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3ShapeId id;
        uint32_t type;
        m3ShapeGeom geom;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (record.type > 255u)
    {
        return false;
    }
    record.id.world0 = world->worldIndex0;
    int32_t slot = record.id.index1 - 1;
    if (slot < 0 || slot >= world->shapes.shapePool.maxIndex ||
        world->shapes.shapePool.alive[slot] == 0 ||
        world->shapes.shapePool.generations[slot] != record.id.generation)
    {
        return false;
    }
    if (!m3SetShapeGeomInternal(world, slot, (uint8_t)record.type, &record.geom))
    {
        return false; // hostile swaps fail the replay loudly
    }
    return true;
}

static bool ApplySetAllowFastRotation(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3BodyId id;
        uint32_t allow;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (record.allow > 1u)
    {
        return false;
    }
    record.id.world0 = world->worldIndex0;
    int32_t index = m3BodySlot(world, record.id);
    if (index < 0)
    {
        return false;
    }
    m3SetAllowFastRotationInternal(world, index, (int32_t)record.allow);
    return true;
}

static bool ApplyWorldExplode(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3ExplosionDef def;
    if (bytes != (int32_t)sizeof(def))
    {
        return false;
    }
    memcpy(&def, payload, sizeof(def));
    if (!m3WorldExplodeInternal(world, &def))
    {
        return false; // hostile blast fields fail loudly
    }
    return true;
}

static bool ApplyWorldFlag(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    int32_t on;
    if (bytes != (int32_t)sizeof(on))
    {
        return false;
    }
    memcpy(&on, payload, sizeof(on));
    if (op == m3_opEnableSleeping)
    {
        m3EnableSleepingInternal(world, on);
    }
    else
    {
        m3EnableContinuousInternal(world, on);
    }
    return true;
}

static bool ApplySetWind(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3Vec3 dir;
        float speed;
        float gustHertz;
        float gustScale;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    if (!m3FiniteV3(record.dir) || !m3FiniteF(record.speed) || record.speed < 0.0f ||
        !m3FiniteF(record.gustHertz) || record.gustHertz < 0.0f || !m3FiniteF(record.gustScale) ||
        record.gustScale < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetWindInternal(world, record.dir, record.speed, record.gustHertz, record.gustScale);
    return true;
}

static bool ApplySetSurfaceVelocity(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3ShapeId id;
        m3Vec3 v;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0 || !m3FiniteV3(record.v))
    {
        return false; // hostile bytes fail loudly
    }
    m3SetSurfaceVelocityInternal(world, slot, record.v);
    return true;
}

static bool ApplySetHitEventThreshold(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    float value;
    if (bytes != (int32_t)sizeof(value))
    {
        return false;
    }
    memcpy(&value, payload, sizeof(value));
    if (!m3FiniteF(value) || value < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3SetHitEventThresholdInternal(world, value);
    return true;
}

static bool ApplyShapeFlag(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    struct
    {
        m3ShapeId id;
        int32_t on;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3ShapeSlot(world, record.id);
    if (slot < 0)
    {
        return false;
    }
    if (op == m3_opEnableShapeHitEvents)
    {
        m3EnableShapeHitEventsInternal(world, slot, record.on);
    }
    else
    {
        m3EnableShapePreSolveInternal(world, slot, record.on);
    }
    return true;
}

static bool ApplyJointVector(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    int32_t op = r->op;
    struct
    {
        m3JointId id;
        int32_t enable;
        float a;
        float b;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.a) || !m3FiniteF(record.b))
    {
        return false; // hostile bytes fail loudly
    }
    if (op == m3_opJointSetLimits)
    {
        // Mirror the public contract: the motor joint's
        // budgets are independent nonnegatives, every other
        // type wants an ordered range.
        if (world->joints.jointType[slot] == (uint8_t)m3_motorJoint
                ? (record.a < 0.0f || record.b < 0.0f)
                : record.a > record.b)
        {
            return false;
        }
        m3JointSetLimitsInternal(world, slot, record.enable, record.a, record.b);
    }
    else
    {
        m3JointSetMotorInternal(world, slot, record.enable, record.a, record.b);
    }
    return true;
}

static bool ApplyJointSetCollide(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3JointId id;
        int32_t on;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0)
    {
        return false;
    }
    m3JointSetCollideInternal(world, slot, record.on);
    return true;
}

static bool ApplyJointSetBreak(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3JointId id;
        float maxForce;
        float maxTorque;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.maxForce) || !m3FiniteF(record.maxTorque) ||
        record.maxForce < 0.0f || record.maxTorque < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3JointSetBreakInternal(world, slot, record.maxForce, record.maxTorque);
    return true;
}

static bool ApplyJointSetSpring(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3JointId id;
        int32_t enable;
        float hertz;
        float zeta;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.hertz) || record.hertz <= 0.0f || !m3FiniteF(record.zeta) ||
        record.zeta < 0.0f)
    {
        return false; // hostile bytes fail loudly
    }
    m3JointSetSpringInternal(world, slot, record.enable, record.hertz, record.zeta);
    return true;
}

static bool ApplyJointSetTarget(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    struct
    {
        m3JointId id;
        float scalar;
        m3Quat q;
    } record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world0 = world->worldIndex0;
    int32_t slot = m3JointSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.scalar) || !m3FiniteQuat(record.q))
    {
        return false; // hostile bytes fail loudly
    }
    m3JointSetTargetInternal(world, slot, record.scalar, record.q);
    return true;
}

static m3ReplayApplyFn* const s_commands[m3_opCount] = {
    [m3_opCreateBody] = ApplyCreateBody,
    [m3_opDestroyBody] = ApplyDestroyBody,
    [m3_opSetLinearVelocity] = ApplySetLinearVelocity,
    [m3_opSetAngularVelocity] = ApplySetAngularVelocity,
    [m3_opStepVetoes] = ApplyStepVetoes,
    [m3_opStep] = ApplyStep,
    [m3_opCreateShape] = ApplyCreateShape,
    [m3_opCreateMeshShape] = ApplyCreateMeshShape,
    [m3_opCreateJoint] = ApplyCreateJoint,
    [m3_opDestroyJoint] = ApplyDestroyJoint,
    [m3_opCreateHullShape] = ApplyCreateHullShape,
    [m3_opCreateVoxelChunkShape] = ApplyCreateVoxelChunkShape,
    [m3_opVoxelSet] = ApplyVoxelSet,
    [m3_opVoxelClear] = ApplyVoxelClear,
    [m3_opVoxelSetFill] = ApplyVoxelSetFill,
    [m3_opVoxelClearBox] = ApplyVoxelClearBox,
    [m3_opCreateCharacter] = ApplyCreateCharacter,
    [m3_opDestroyCharacter] = ApplyDestroyCharacter,
    [m3_opCharacterMove] = ApplyCharacterMove,
    [m3_opCharacterStance] = ApplyCharacterStance,
    [m3_opCreateVehicle] = ApplyCreateVehicle,
    [m3_opDestroyVehicle] = ApplyDestroyVehicle,
    [m3_opVehicleTankCommands] = ApplyVehicleTankCommands,
    [m3_opVehicleCommands] = ApplyVehicleCommands,
    [m3_opVehicleDrivetrain] = ApplyVehicleDrivetrain,
    [m3_opVehicleGear] = ApplyVehicleGear,
    [m3_opCreateSoftBody] = ApplyCreateSoftBody,
    [m3_opDestroySoftBody] = ApplyDestroySoftBody,
    [m3_opSoftBodyPin] = ApplySoftBodyPin,
    [m3_opSoftBodyAnchor] = ApplySoftBodyAnchor,
    [m3_opSoftBodyAnchorSoft] = ApplySoftBodyAnchorSoft,
    [m3_opApplyForce] = ApplyBodyVector,
    [m3_opApplyTorque] = ApplyBodyVector,
    [m3_opApplyLinearImpulse] = ApplyBodyVector,
    [m3_opApplyAngularImpulse] = ApplyBodyVector,
    [m3_opApplyForceAtPoint] = ApplyBodyVectorAtPoint,
    [m3_opApplyImpulseAtPoint] = ApplyBodyVectorAtPoint,
    [m3_opSetTransform] = ApplyBodyPose,
    [m3_opSetTargetTransform] = ApplyBodyPose,
    [m3_opSetType] = ApplyBodyByte,
    [m3_opSetEnabled] = ApplyBodyByte,
    [m3_opSetAwake] = ApplyBodyByte,
    [m3_opSetMotionLocks] = ApplySetMotionLocks,
    [m3_opSetSleepControls] = ApplySetSleepControls,
    [m3_opDestroyShape] = ApplyDestroyShape,
    [m3_opSetGravity] = ApplySetGravity,
    [m3_opSetShapeFriction] = ApplyShapeScalar,
    [m3_opSetShapeRestitution] = ApplyShapeScalar,
    [m3_opSetShapeRolling] = ApplyShapeScalar,
    [m3_opSetShapeDensity] = ApplySetShapeDensity,
    [m3_opSetContactTuning] = ApplySetContactTuning,
    [m3_opSetRestitutionThreshold] = ApplyWorldScalar,
    [m3_opSetMaximumLinearSpeed] = ApplyWorldScalar,
    [m3_opSetMaximumAngularSpeed] = ApplySetMaximumAngularSpeed,
    [m3_opSetBodyName] = ApplySetBodyName,
    [m3_opCreateSoftBodyTet] = ApplyCreateSoftBodyTet,
    [m3_opCreateHeightFieldGrid] = ApplyCreateHeightFieldGrid,
    [m3_opCreateWaterVolume] = ApplyCreateWaterVolume,
    [m3_opDestroyWaterVolume] = ApplyDestroyWaterVolume,
    [m3_opRebuildBroadphase] = ApplyRebuildBroadphase,
    [m3_opSetMeshMaterials] = ApplySetMeshMaterials,
    [m3_opJointSetMotorPose] = ApplyJointSetMotorPose,
    [m3_opJointSetSteer] = ApplyJointSetSteer,
    [m3_opSetShapeGeom] = ApplySetShapeGeom,
    [m3_opSetAllowFastRotation] = ApplySetAllowFastRotation,
    [m3_opWorldExplode] = ApplyWorldExplode,
    [m3_opEnableSleeping] = ApplyWorldFlag,
    [m3_opEnableContinuous] = ApplyWorldFlag,
    [m3_opSetWind] = ApplySetWind,
    [m3_opSetSurfaceVelocity] = ApplySetSurfaceVelocity,
    [m3_opSetHitEventThreshold] = ApplySetHitEventThreshold,
    [m3_opEnableShapeHitEvents] = ApplyShapeFlag,
    [m3_opEnableShapePreSolve] = ApplyShapeFlag,
    [m3_opJointSetLimits] = ApplyJointVector,
    [m3_opJointSetMotor] = ApplyJointVector,
    [m3_opJointSetCollide] = ApplyJointSetCollide,
    [m3_opJointSetBreak] = ApplyJointSetBreak,
    [m3_opJointSetSpring] = ApplyJointSetSpring,
    [m3_opJointSetTarget] = ApplyJointSetTarget,
};

bool m3JournalReplayApply(m3World* world, const void* data, int32_t size)
{
    const uint8_t* stream = (const uint8_t*)data;
    int32_t cursor = 0;
    while (cursor < size)
    {
        if (cursor + 8 > size)
        {
            return false; // truncated header: reject loudly
        }
        m3ReplayRecord r;
        memcpy(&r.op, stream + cursor, 4);
        memcpy(&r.bytes, stream + cursor + 4, 4);
        cursor += 8;
        if (r.bytes < 0 || cursor + r.bytes > size)
        {
            return false; // truncated payload
        }
        r.payload = stream + cursor;
        cursor += r.bytes;
        if (r.op <= 0 || r.op >= m3_opCount || s_commands[r.op] == NULL)
        {
            return false; // unknown op: reject loudly, never skip
        }
        if (!s_commands[r.op](world, &r))
        {
            return false;
        }
    }
    return cursor == size;
}
