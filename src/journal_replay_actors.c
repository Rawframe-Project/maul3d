// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay for characters, vehicles, soft bodies and water
// volumes.

#include "journal_replay.h"

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

bool m3ReplayCreateCharacter(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpCreateCharacter record;
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

bool m3ReplayDestroyCharacter(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3CharacterId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world = world->idWorld;
    int32_t slot = m3CharacterSlot(world, id);
    if (slot < 0)
    {
        return false;
    }
    m3DestroyCharacterInternal(world, slot);
    return true;
}

bool m3ReplayCharacterMove(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpCharacterMove record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3CharacterSlot(world, record.id);
    if (slot < 0 || !m3FiniteV3(record.translation))
    {
        return false; // hostile bytes fail loudly
    }
    m3CharacterMoveInternal(world, slot, record.translation);
    return true;
}

bool m3ReplayCharacterStance(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpCharacterStance record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
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

bool m3ReplayCreateVehicle(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpCreateVehicle record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        size_t wheelBase = offsetof(m3VehicleDef, wheels) + (size_t)w * sizeof(m3WheelDef);
        m3NormalizeBoolByte(&record.def, wheelBase + offsetof(m3WheelDef, steerable));
        m3NormalizeBoolByte(&record.def, wheelBase + offsetof(m3WheelDef, driven));
    }
    record.def.chassisId.world = world->idWorld;
    int32_t slot = m3CreateVehicleInternal(world, &record.def);
    if (slot < 0 || slot + 1 != record.expected.index1 ||
        world->vehicles.vehPool.generations[slot] != record.expected.generation)
    {
        return false; // id determinism holds for vehicles too
    }
    return true;
}

bool m3ReplayDestroyVehicle(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3VehicleId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world = world->idWorld;
    int32_t slot = m3VehicleSlot(world, id);
    if (slot < 0)
    {
        return false;
    }
    m3DestroyVehicleInternal(world, slot);
    return true;
}

bool m3ReplayVehicleTankCommands(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpVehicleTankCommands record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3VehicleSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.left) || !m3FiniteF(record.right) || !m3FiniteF(record.brake))
    {
        return false; // hostile bytes fail loudly
    }
    m3VehicleTankCommandsInternal(world, slot, record.left, record.right, record.brake);
    return true;
}

bool m3ReplayVehicleCommands(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpVehicleCommands record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3VehicleSlot(world, record.id);
    if (slot < 0 || !m3FiniteF(record.throttle) || !m3FiniteF(record.steer) ||
        !m3FiniteF(record.brake))
    {
        return false; // hostile bytes fail loudly
    }
    m3VehicleCommandsInternal(world, slot, record.throttle, record.steer, record.brake);
    return true;
}

bool m3ReplayVehicleDrivetrain(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpVehicleDrivetrain record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    m3NormalizeBoolByte(&record.def, offsetof(m3DrivetrainDef, autoShift));
    record.id.world = world->idWorld;
    int32_t slot = m3VehicleSlot(world, record.id);
    if (slot < 0 || !m3VehicleDrivetrainInternal(world, slot, &record.def))
    {
        return false; // journaled defs are UNTRUSTED bytes
    }
    return true;
}

bool m3ReplayVehicleGear(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpVehicleGear record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3VehicleSlot(world, record.id);
    if (slot < 0 || !m3VehicleGearInternal(world, slot, record.gear))
    {
        return false;
    }
    return true;
}

bool m3ReplayCreateSoftBody(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpCreateSoftBody record;
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

bool m3ReplayDestroySoftBody(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3SoftBodyId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world = world->idWorld;
    int32_t slot = m3SoftBodySlot(world, id);
    if (slot < 0)
    {
        return false;
    }
    m3DestroySoftBodyInternal(world, slot);
    return true;
}

bool m3ReplaySoftBodyPin(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSoftBodyPin record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    int32_t slot = m3SoftBodySlot(world, record.id);
    if (slot < 0 || record.particle < 0 ||
        record.particle >= world->softBodies.softParticleCount[slot])
    {
        return false;
    }
    m3SoftBodyPinInternal(world, slot, record.particle);
    return true;
}

bool m3ReplaySoftBodyAnchor(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSoftBodyAnchor record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.id.world = world->idWorld;
    record.body.world = world->idWorld;
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

bool m3ReplaySoftBodyAnchorSoft(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpSoftBodyAnchorSoft record;
    if (bytes != (int32_t)sizeof(record))
    {
        return false;
    }
    memcpy(&record, payload, sizeof(record));
    record.idA.world = world->idWorld;
    record.idB.world = world->idWorld;
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

bool m3ReplayCreateSoftBodyTet(m3World* world, const m3ReplayRecord* r)
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

bool m3ReplayCreateWaterVolume(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3OpCreateWaterVolume record;
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

bool m3ReplayDestroyWaterVolume(m3World* world, const m3ReplayRecord* r)
{
    const uint8_t* payload = r->payload;
    int32_t bytes = r->bytes;
    m3WaterVolumeId id;
    if (bytes != (int32_t)sizeof(id))
    {
        return false;
    }
    memcpy(&id, payload, sizeof(id));
    id.world = world->idWorld;
    int32_t index = id.index1 - 1;
    if (!m3IdPoolValid(&world->water.waterPool, index, id.generation))
    {
        return false;
    }
    m3DestroyWaterVolumeInternal(world, index);
    return true;
}
