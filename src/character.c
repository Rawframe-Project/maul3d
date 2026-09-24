// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The character controller core: collide-and-slide over the
// engine's own convex casts, deterministic front to back. The
// character owns a zero-velocity kinematic capsule body: the solver
// never moves it, dynamics collide with it, and every displacement
// is a journaled command that replays and rolls back like any other
// mutation.

#include "maul3d/character.h"

#include "body.h"
#include "character.h"
#include "journal.h"
#include "query.h"
#include "raycast.h"
#include "shape.h"
#include "solver.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

#define M3_CHARACTER_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3CharacterDef) << 8) ^ 7))
#define M3_CHARACTER_SLIDE_ITERATIONS 4

m3CharacterDef m3DefaultCharacterDef(void)
{
    m3CharacterDef def;
    memset(&def, 0, sizeof(def));
    def.radius = 0.4f;
    def.halfHeight = 0.5f;
    def.maxSlopeAngle = 0.8f; // ~46 degrees
    def.snapDistance = 0.3f;
    def.skin = 0.02f;
    def.stepHeight = 0.35f;
    def.mass = 80.0f;
    def.pushMaxMassRatio = 2.0f;
    def.internalValue = M3_CHARACTER_COOKIE;
    return def;
}

int32_t m3CharacterSlot(const m3World* world, m3CharacterId characterId)
{
    int32_t index = characterId.index1 - 1;
    if (world == NULL || characterId.world != world->idWorld ||
        !m3IdPoolValid(&world->characters.charPool, index, characterId.generation))
    {
        return -1;
    }
    return index;
}

int32_t m3CreateCharacterInternal(m3World* world, const m3CharacterDef* def)
{
    // Input checks live here because replay hands this function raw
    // journal bytes: a flipped radius or slope byte must be refused
    // instead of minting a NaN capsule.
    if (!m3FinitePos3(def->position) || !m3FiniteF(def->radius) || !(def->radius > 0.0f) ||
        !m3FiniteF(def->halfHeight) || def->halfHeight < 0.0f || !m3FiniteF(def->maxSlopeAngle) ||
        def->maxSlopeAngle <= 0.0f || def->maxSlopeAngle > 1.5f || !m3FiniteF(def->snapDistance) ||
        def->snapDistance < 0.0f || !m3FiniteF(def->skin) || !(def->skin > 0.0f) ||
        !m3FiniteF(def->stepHeight) || def->stepHeight < 0.0f || !m3FiniteF(def->mass) ||
        !(def->mass > 0.0f) || !m3FiniteF(def->pushMaxMassRatio) || def->pushMaxMassRatio < 0.0f)
    {
        return -1;
    }
    // A character takes a slot, a body and a shape. Room for all three is
    // checked first: a refused create is not journaled, so it must not
    // take and free a slot and move the generations.
    if (!m3IdPoolHasRoom(&world->characters.charPool) ||
        !m3IdPoolHasRoom(&world->bodies.bodyPool) || !m3IdPoolHasRoom(&world->shapes.shapePool))
    {
        return -1;
    }
    int32_t slot = m3IdPoolAlloc(&world->characters.charPool);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.position = def->position;
    int32_t body = m3CreateBodyInternal(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.v = (m3Vec3){0.0f, def->halfHeight, 0.0f};
    geom.v2 = (m3Vec3){0.0f, -def->halfHeight, 0.0f};
    geom.s = def->radius;
    int32_t shape = m3CreateShapeInternal(world, body, (uint8_t)m3_capsuleShape, &geom, &sd,
                                          &(m3ShapeContent){NULL, NULL, NULL, NULL});
    if (shape < 0)
    {
        // Unreachable while the tree holds two nodes per shape slot.
        m3DestroyBodyInternal(world, body);
        m3IdPoolFree(&world->characters.charPool, slot);
        return -1;
    }
    world->characters.charBody[slot] = body;
    world->characters.charRadius[slot] = def->radius;
    world->characters.charHalfHeight[slot] = def->halfHeight;
    world->characters.charCosSlope[slot] = m3ComputeCosSin(def->maxSlopeAngle).c;
    world->characters.charSnap[slot] = def->snapDistance;
    world->characters.charSkin[slot] = def->skin;
    world->characters.charStepHeight[slot] = def->stepHeight;
    world->characters.charMass[slot] = def->mass;
    world->characters.charPushMax[slot] = def->pushMaxMassRatio;
    world->characters.charGrounded[slot] = 0;
    world->characters.charGroundNormal[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->characters.charGroundBody[slot] = -1;
    world->characters.charGroundGen[slot] = 0;
    return slot;
}

void m3DestroyCharacterInternal(m3World* world, int32_t slot)
{
    int32_t body = world->characters.charBody[slot];
    if (body >= 0 && world->bodies.bodyPool.alive[body] != 0)
    {
        m3DestroyBodyInternal(world, body);
    }
    world->characters.charBody[slot] = -1;
    world->characters.charRadius[slot] = 0.0f;
    world->characters.charHalfHeight[slot] = 0.0f;
    world->characters.charCosSlope[slot] = 0.0f;
    world->characters.charSnap[slot] = 0.0f;
    world->characters.charSkin[slot] = 0.0f;
    world->characters.charStepHeight[slot] = 0.0f;
    world->characters.charMass[slot] = 0.0f;
    world->characters.charPushMax[slot] = 0.0f;
    world->characters.charGrounded[slot] = 0;
    world->characters.charGroundNormal[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->characters.charGroundBody[slot] = -1;
    world->characters.charGroundGen[slot] = 0;
    m3IdPoolFree(&world->characters.charPool, slot);
}

// The floor classifier: a ray straight down the capsule
// axis. Capsule casts landing on a stair edge report the CORNER
// normal, which reads as unwalkable; the surface under the axis is
// the truth a walker cares about. The ray starts inside our own
// capsule, and the ray contract (start-inside misses) filters self
// for free.
static bool WalkableBelow(m3World* world, int32_t slot, m3Pos3 center, m3real reach,
                          m3RayCastResult* outHit)
{
    m3real depth = world->characters.charHalfHeight[slot] + world->characters.charRadius[slot] +
                   reach + world->characters.charSkin[slot];
    m3RayCastResult hit = m3RayClosestInternal(world, center, (m3Vec3){0.0f, -depth, 0.0f});
    if (hit.hit && hit.normal.y >= world->characters.charCosSlope[slot])
    {
        *outHit = hit;
        return true;
    }
    return false;
}

// Grounding is recorded WITH the body under the surface: the
// carry pass ferries riders by that body's step motion, and the
// generation guards against a recycled slot impersonating a floor.
static void RecordGround(m3World* world, int32_t slot, const m3RayCastResult* hit)
{
    int32_t shape = hit->shapeId.index1 - 1;
    int32_t under = world->shapes.shapeBody[shape];
    world->characters.charGrounded[slot] = 1;
    world->characters.charGroundNormal[slot] = hit->normal;
    world->characters.charGroundBody[slot] = under;
    world->characters.charGroundGen[slot] = world->bodies.bodyPool.generations[under];
}

static void ClearGround(m3World* world, int32_t slot)
{
    world->characters.charGrounded[slot] = 0;
    world->characters.charGroundNormal[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->characters.charGroundBody[slot] = -1;
    world->characters.charGroundGen[slot] = 0;
}

// One capsule cast from the character's center, excluding its own
// body (the internal caster's ignore hook exists for exactly this).
static m3RayCastResult CharacterCast(m3World* world, int32_t slot, m3Pos3 center,
                                     m3Vec3 translation)
{
    m3Vec3 points[2] = {{0.0f, world->characters.charHalfHeight[slot], 0.0f},
                        {0.0f, -world->characters.charHalfHeight[slot], 0.0f}};
    return m3CastConvexClosestEx(world, center, points, 2, world->characters.charRadius[slot],
                                 translation, world->characters.charBody[slot]);
}

void m3CharacterMoveInternal(m3World* world, int32_t slot, m3Vec3 translation)
{
    int32_t body = world->characters.charBody[slot];
    m3Pos3 pos = world->bodies.transforms[body].p;
    m3Vec3 remaining = translation;
    m3real skin = world->characters.charSkin[slot];
    m3real cosSlope = world->characters.charCosSlope[slot];
    m3real stepHeight = world->characters.charStepHeight[slot];
    int wasDescending = translation.y < 0.0f;
    int wasGrounded = world->characters.charGrounded[slot] != 0;
    ClearGround(world, slot);

    for (int32_t iteration = 0; iteration < M3_CHARACTER_SLIDE_ITERATIONS; ++iteration)
    {
        m3real len2 = m3Dot3(remaining, remaining);
        if (len2 < 1.0e-10f)
        {
            break;
        }
        m3RayCastResult hit = CharacterCast(world, slot, pos, remaining);
        if (!hit.hit)
        {
            pos.x += (double)remaining.x;
            pos.y += (double)remaining.y;
            pos.z += (double)remaining.z;
            break;
        }
        // The stair maneuver: a grounded walker blocked by a
        // steep face tries the classic lift, advance, land triple.
        // Accept only a walkable landing that clears no more than
        // one step height; otherwise the face is a wall and the
        // slide below owns it.
        if (wasGrounded && stepHeight > 0.0f && hit.normal.y < cosSlope &&
            (remaining.x != 0.0f || remaining.z != 0.0f))
        {
            m3RayCastResult up = CharacterCast(world, slot, pos, (m3Vec3){0.0f, stepHeight, 0.0f});
            m3real lift = up.hit ? m3MaxF(up.fraction * stepHeight - skin, 0.0f) : stepHeight;
            m3Pos3 lifted = {pos.x, pos.y + (double)lift, pos.z};
            m3Vec3 horizontal = {remaining.x, 0.0f, remaining.z};
            m3real hLen2 = m3Dot3(horizontal, horizontal);
            m3RayCastResult fwd = CharacterCast(world, slot, lifted, horizontal);
            m3real hLen = sqrtf(hLen2);
            m3real go = fwd.hit ? m3MaxF(fwd.fraction * hLen - skin, 0.0f) : hLen;
            if (go > skin)
            {
                m3real invH = hLen > 0.0f ? 1.0f / hLen : 0.0f;
                m3Pos3 advanced = {lifted.x + (double)(horizontal.x * invH * go), lifted.y,
                                   lifted.z + (double)(horizontal.z * invH * go)};
                m3real reach = lift + stepHeight;
                m3RayCastResult land =
                    CharacterCast(world, slot, advanced, (m3Vec3){0.0f, -reach, 0.0f});
                m3RayCastResult floorHit = {0};
                // Acceptance is the RAY'S alone, and the ray probes
                // HALF A RADIUS AHEAD of the landing axis. A steep
                // ramp rising out of a walkable floor blocks the
                // capsule surface while the axis still hangs over
                // the flat ground behind the crease; an axis ray
                // would bless that landing forever, one step height
                // per tick (the sixty-degree ramp taught this). The
                // forward probe lets the steep face shadow the low
                // floor, so the ramp refuses while stair landings,
                // whose treads carry the probe, keep answering yes.
                // The probe start stays strictly inside our capsule,
                // so the ray's start-inside contract still filters
                // self. And the net rise must fit one step height.
                m3real probeAhead = 0.5f * world->characters.charRadius[slot];
                m3Pos3 probe = {advanced.x + (double)(horizontal.x * invH * probeAhead), advanced.y,
                                advanced.z + (double)(horizontal.z * invH * probeAhead)};
                m3real drop = land.hit ? m3MaxF(land.fraction * reach - skin, 0.0f) : 0.0f;
                double landedY = advanced.y - (double)drop;
                if (land.hit && WalkableBelow(world, slot, probe, reach, &floorHit) &&
                    landedY - pos.y <= (double)(stepHeight + skin))
                {
                    pos = (m3Pos3){advanced.x, landedY, advanced.z};
                    RecordGround(world, slot, &floorHit);
                    // The horizontal budget is spent; vertical intent
                    // (host gravity) dissolves into the landing.
                    m3real spent = hLen > 0.0f ? go * invH : 1.0f;
                    remaining = m3MulSV3(1.0f - spent, horizontal);
                    continue;
                }
            }
        }
        // Advance to the hit, keeping the skin along the direction
        // of travel (never park flush: the next cast would start
        // overlapped and report zero forever). The skin retreat is
        // NOT lost: it returns to the slide budget, or a hundred
        // grounded ticks would each quietly eat a skin of distance.
        m3real len = sqrtf(len2);
        m3real advance = hit.fraction * len - skin;
        if (advance < 0.0f)
        {
            advance = 0.0f;
        }
        m3real inv = 1.0f / len;
        pos.x += (double)(remaining.x * inv * advance);
        pos.y += (double)(remaining.y * inv * advance);
        pos.z += (double)(remaining.z * inv * advance);
        m3Vec3 n = hit.normal;
        if (n.y >= cosSlope)
        {
            RecordGround(world, slot, &hit);
        }
        // The push: a blocked move shoves a dynamic body with
        // impulse = mass * blocked displacement into the surface.
        // Per tick that displacement IS the walker's velocity in
        // tick units, so the momentum flux comes out mass * speed
        // with no dt anywhere. Walkable floors are standing, not
        // pushing (weight is not modeled), and bodies heavier than
        // pushMaxMassRatio * mass are walls by contract.
        {
            int32_t hitShape = hit.shapeId.index1 - 1;
            int32_t hitBody = world->shapes.shapeBody[hitShape];
            m3real blocked = len - advance;
            m3real into = -m3Dot3(remaining, n) * inv;
            if (world->bodies.types[hitBody] == (uint8_t)m3_dynamicBody && n.y < cosSlope &&
                world->bodies.invMass[hitBody] > 0.0f &&
                world->characters.charPushMax[slot] > 0.0f &&
                1.0f <= world->characters.charPushMax[slot] * world->characters.charMass[slot] *
                            world->bodies.invMass[hitBody] &&
                blocked > 0.0f && into > 0.0f)
            {
                m3Vec3 impulse = m3MulSV3(-world->characters.charMass[slot] * blocked * into, n);
                m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[hitBody].q,
                                          world->bodies.localCenters[hitBody]);
                m3Vec3 arm = {(m3real)(hit.point.x - world->bodies.transforms[hitBody].p.x) - rlc.x,
                              (m3real)(hit.point.y - world->bodies.transforms[hitBody].p.y) - rlc.y,
                              (m3real)(hit.point.z - world->bodies.transforms[hitBody].p.z) -
                                  rlc.z};
                world->bodies.linearVelocities[hitBody] =
                    m3Add3(world->bodies.linearVelocities[hitBody],
                           m3MulSV3(world->bodies.invMass[hitBody], impulse));
                world->bodies.angularVelocities[hitBody] =
                    m3Add3(world->bodies.angularVelocities[hitBody],
                           m3MulMV3(m3WorldInvInertia(world, hitBody), m3Cross3(arm, impulse)));
                world->bodies.awake[hitBody] = 1;
                world->bodies.sleepTimes[hitBody] = 0.0f;
            }
        }
        // Slide: everything not actually traveled stays in the
        // budget, minus its component into the surface. A steep
        // face is a WALL: the slide may never mint upward motion
        // out of horizontal intent, or walkers would creep up
        // cliffs one skin at a time.
        m3Vec3 leftover = m3MulSV3(1.0f - advance * inv, remaining);
        remaining = m3Sub3(leftover, m3MulSV3(m3Dot3(leftover, n), n));
        if (n.y < cosSlope && remaining.y > 0.0f && translation.y <= 0.0f)
        {
            remaining.y = 0.0f;
        }
    }

    // Snap to ground: a descending character glues to walkable
    // floor within snapDistance (stairs and edges stay under foot);
    // no floor in reach means airborne.
    if (wasDescending || world->characters.charGrounded[slot] != 0)
    {
        m3RayCastResult down = CharacterCast(
            world, slot, pos, (m3Vec3){0.0f, -world->characters.charSnap[slot], 0.0f});
        m3RayCastResult floorHit;
        memset(&floorHit, 0, sizeof(floorHit));
        if (down.hit &&
            (down.normal.y >= cosSlope ||
             WalkableBelow(world, slot, pos, world->characters.charSnap[slot], &floorHit)))
        {
            m3real drop = down.fraction * world->characters.charSnap[slot] - skin;
            if (drop > 0.0f)
            {
                pos.y -= (double)drop;
            }
            if (down.normal.y >= cosSlope)
            {
                RecordGround(world, slot, &down);
            }
            else
            {
                RecordGround(world, slot, &floorHit);
            }
        }
        else if (!down.hit)
        {
            ClearGround(world, slot);
        }
    }

    // Write the kinematic body's new pose (rotation stays identity:
    // characters translate; facing is the host's render concern).
    world->bodies.transforms[body].p = pos;
    world->bodies.sleepTimes[body] = 0.0f;
}

static void RefreshGroundingCore(m3World* world, int32_t slot)
{
    m3Pos3 pos = world->bodies.transforms[world->characters.charBody[slot]].p;
    m3RayCastResult down =
        CharacterCast(world, slot, pos, (m3Vec3){0.0f, -world->characters.charSnap[slot], 0.0f});
    m3RayCastResult floorHit;
    memset(&floorHit, 0, sizeof(floorHit));
    if (down.hit && down.normal.y >= world->characters.charCosSlope[slot])
    {
        RecordGround(world, slot, &down); // the floor may be a NEW body
    }
    else if (down.hit &&
             WalkableBelow(world, slot, pos, world->characters.charSnap[slot], &floorHit))
    {
        RecordGround(world, slot, &floorHit);
    }
    else
    {
        // The floor is gone: airborne the same step it vanished.
        ClearGround(world, slot);
    }
}

void m3CharacterRefreshGrounding(m3World* world, int32_t slot)
{
    if (world->characters.charGrounded[slot] == 0)
    {
        return; // airborne already tells the truth
    }
    RefreshGroundingCore(world, slot);
}

// Riders: after the step integrates every body, grounded
// characters follow their ground body's rigid motion through the
// regular slide casts, so platforms ferry walkers and walls still
// block them. Serial in slot order: canonical. The begin-of-step
// origin is reconstructed from the continuous pass's COM and
// rotation capture; identical inputs give bit-identical rotation
// math, so an unmoved body compares equal and carries nobody.
void m3CharacterCarryRiders(m3World* world, const m3Pos3* com0, const m3Quat* rot0)
{
    for (int32_t slot = 0; slot < world->characters.charPool.maxIndex; ++slot)
    {
        if (world->characters.charPool.alive[slot] == 0 ||
            world->characters.charGrounded[slot] == 0)
        {
            continue;
        }
        int32_t under = world->characters.charGroundBody[slot];
        if (under < 0 || world->bodies.bodyPool.alive[under] == 0 ||
            world->bodies.bodyPool.generations[under] != world->characters.charGroundGen[slot])
        {
            continue; // the floor body is gone; Move retells the truth
        }
        m3Vec3 rlc0 = m3RotateVec3(rot0[under], world->bodies.localCenters[under]);
        m3Pos3 old = {com0[under].x - (double)rlc0.x, com0[under].y - (double)rlc0.y,
                      com0[under].z - (double)rlc0.z};
        const m3Transform* now = &world->bodies.transforms[under];
        // Bitwise identity is the question here, so memcmp is right.
        // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison)
        if (memcmp(&rot0[under], &now->q, sizeof(m3Quat)) == 0 && old.x == now->p.x &&
            old.y == now->p.y && old.z == now->p.z)
        {
            continue; // the floor did not move this step
        }
        m3Pos3 p0 = world->bodies.transforms[world->characters.charBody[slot]].p;
        m3Vec3 rel = {(m3real)(p0.x - old.x), (m3real)(p0.y - old.y), (m3real)(p0.z - old.z)};
        m3Vec3 out = m3RotateVec3(now->q, m3InvRotateVec3(rot0[under], rel));
        m3Vec3 carry = {(m3real)(now->p.x + (double)out.x - p0.x),
                        (m3real)(now->p.y + (double)out.y - p0.y),
                        (m3real)(now->p.z + (double)out.z - p0.z)};
        m3CharacterMoveInternal(world, slot, carry);
        // A pure sideways carry ends the slide with no floor
        // contact and Move's snap only serves descents; the rider
        // still deserves the grounding truth, so ask the floor
        // classifier directly (it re-grounds within snapDistance
        // or reports that the ride ended in air).
        RefreshGroundingCore(world, slot);
    }
}

m3CharacterId m3CreateCharacter(m3WorldId worldId, const m3CharacterDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_CHARACTER_COOKIE ||
        !m3FinitePos3(def->position) || !m3FiniteF(def->radius) || !(def->radius > 0.0f) ||
        !m3FiniteF(def->halfHeight) || def->halfHeight < 0.0f || !m3FiniteF(def->maxSlopeAngle) ||
        def->maxSlopeAngle <= 0.0f || def->maxSlopeAngle > 1.5f || !m3FiniteF(def->snapDistance) ||
        def->snapDistance < 0.0f || !m3FiniteF(def->skin) || !(def->skin > 0.0f) ||
        !m3FiniteF(def->stepHeight) || def->stepHeight < 0.0f || !m3FiniteF(def->mass) ||
        !(def->mass > 0.0f) || !m3FiniteF(def->pushMaxMassRatio) || def->pushMaxMassRatio < 0.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return m3_nullCharacterId;
    }
    int32_t slot = m3CreateCharacterInternal(world, def);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorCapacity);
        return m3_nullCharacterId;
    }
    m3CharacterId id = {slot + 1, world->idWorld, world->characters.charPool.generations[slot]};
    if (world->recorder.journalActive != 0)
    {
        m3OpCreateCharacter record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateCharacter, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyCharacter(m3CharacterId characterId)
{
    m3World* world = m3WorldFromTag(characterId.world);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale: the quiet destroy contract
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyCharacter, &characterId, (int32_t)sizeof(characterId));
    }
    m3DestroyCharacterInternal(world, slot);
}

bool m3Character_IsValid(m3CharacterId characterId)
{
    m3World* world = m3WorldFromTag(characterId.world);
    return world != NULL && m3CharacterSlot(world, characterId) >= 0;
}

void m3Character_Move(m3CharacterId characterId, m3Vec3 translation)
{
    m3World* world = m3WorldFromTag(characterId.world);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0 || !m3FiniteV3(translation) ||
        !(translation.x >= -M3_CAST_LIMIT && translation.x <= M3_CAST_LIMIT) ||
        !(translation.y >= -M3_CAST_LIMIT && translation.y <= M3_CAST_LIMIT) ||
        !(translation.z >= -M3_CAST_LIMIT && translation.z <= M3_CAST_LIMIT))
    {
        return; // stale id or hostile move (non-finite, or past the
                // caster's float budget): a documented no-op that
                // never journals
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpCharacterMove record;
        memset(&record, 0, sizeof(record));
        record.id = characterId;
        record.translation = translation;
        m3JournalRecord(world, m3_opCharacterMove, &record, (int32_t)sizeof(record));
    }
    m3CharacterMoveInternal(world, slot, translation);
}

m3Pos3 m3Character_GetPosition(m3CharacterId characterId)
{
    m3World* world = m3WorldFromTag(characterId.world);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return world->bodies.transforms[world->characters.charBody[slot]].p;
}

bool m3Character_IsGrounded(m3CharacterId characterId)
{
    m3World* world = m3WorldFromTag(characterId.world);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    return slot >= 0 && world->characters.charGrounded[slot] != 0;
}

m3Vec3 m3Character_GetGroundNormal(m3CharacterId characterId)
{
    m3World* world = m3WorldFromTag(characterId.world);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return world->characters.charGroundNormal[slot];
}

m3BodyId m3Character_GetGroundBody(m3CharacterId characterId)
{
    m3World* world = m3WorldFromTag(characterId.world);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0 || world->characters.charGrounded[slot] == 0)
    {
        return m3_nullBodyId;
    }
    int32_t under = world->characters.charGroundBody[slot];
    if (under < 0 || world->bodies.bodyPool.alive[under] == 0 ||
        world->bodies.bodyPool.generations[under] != world->characters.charGroundGen[slot])
    {
        return m3_nullBodyId; // destroyed or recycled: no impostors
    }
    return (m3BodyId){under + 1, world->idWorld, world->bodies.bodyPool.generations[under]};
}

// The stance change: resize the capsule FEET ANCHORED. The
// veto casts the GROWN capsule at its new pose through the exact
// core every move uses (m3CastConvexClosestEx reports an initial
// overlap as a fraction-zero hit, so a wall against a wider radius
// refuses just like a pressing ceiling); the cast reaches one skin
// upward so the standing gap survives the change. Shrinks skip the
// veto by law: removing volume cannot create contact.
bool m3CharacterStanceInternal(m3World* world, int32_t slot, m3real halfHeight, m3real radius)
{
    if (!m3FiniteF(halfHeight) || !m3FiniteF(radius) || halfHeight < 0.05f || radius < 0.05f ||
        halfHeight > 10.0f || radius > 10.0f)
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // hostile dimensions never touch state (a veto is not misuse)
    }
    int32_t body = world->characters.charBody[slot];
    m3real oldHh = world->characters.charHalfHeight[slot];
    m3real oldR = world->characters.charRadius[slot];
    m3real rise = (halfHeight + radius) - (oldHh + oldR); // feet anchor
    m3Pos3 newCenter = world->bodies.transforms[body].p;
    newCenter.y += (double)rise;
    int grows = halfHeight > oldHh || radius > oldR;
    if (grows)
    {
        m3Vec3 points[2] = {{0.0f, halfHeight, 0.0f}, {0.0f, -halfHeight, 0.0f}};
        m3RayCastResult hit =
            m3CastConvexClosestEx(world, newCenter, points, 2, radius,
                                  (m3Vec3){0.0f, world->characters.charSkin[slot], 0.0f}, body);
        if (hit.hit)
        {
            return false; // the stand-up veto: something presses
        }
    }
    world->characters.charHalfHeight[slot] = halfHeight;
    world->characters.charRadius[slot] = radius;
    int32_t shape = world->bodies.bodyShapeHead[body]; // the capsule is the
                                                       // character's only shape
    world->shapes.shapeGeom[shape].v = (m3Vec3){0.0f, halfHeight, 0.0f};
    world->shapes.shapeGeom[shape].v2 = (m3Vec3){0.0f, -halfHeight, 0.0f};
    world->shapes.shapeGeom[shape].s = radius;
    world->bodies.transforms[body].p = newCenter;
    world->bodies.sleepTimes[body] = 0.0f;
    // The grounded state stays correct through the resize (the same
    // refresh a restore runs).
    RefreshGroundingCore(world, slot);
    return true;
}

bool m3Character_SetStance(m3CharacterId characterId, m3real halfHeight, m3real radius)
{
    m3World* world = m3WorldFromTag(characterId.world);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    if (!m3CharacterStanceInternal(world, slot, halfHeight, radius))
    {
        return false; // vetoed or hostile: never journals
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpCharacterStance record;
        memset(&record, 0, sizeof(record));
        record.id = characterId;
        record.halfHeight = halfHeight;
        record.radius = radius;
        m3JournalRecord(world, m3_opCharacterStance, &record, (int32_t)sizeof(record));
    }
    return true;
}

void m3Character_GetStance(m3CharacterId characterId, m3real* halfHeight, m3real* radius)
{
    m3World* world = m3WorldFromTag(characterId.world);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (halfHeight != NULL)
    {
        *halfHeight = slot >= 0 ? world->characters.charHalfHeight[slot] : 0.0f;
    }
    if (radius != NULL)
    {
        *radius = slot >= 0 ? world->characters.charRadius[slot] : 0.0f;
    }
}
