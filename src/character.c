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
#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "query.h"
#include "raycast.h"
#include "shape.h"
#include "solver_math.h"
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
    if (world == NULL || !m3IdPoolValid(&world->characters.charPool, index, characterId.generation))
    {
        return -1;
    }
    return index;
}

bool m3CharacterMoveValid(m3Vec3 translation)
{
    return m3WithinV3(translation, M3_CAST_LIMIT);
}

int32_t m3CreateCharacterInternal(m3World* world, const m3CharacterDef* def)
{
    // Input checks live here because replay hands this function raw
    // journal bytes: a flipped radius or slope byte must be refused
    // instead of minting a NaN capsule.
    // The capsule is bounded like any capsule shape; the walker's own mass
    // and push ratio are bounded like a body's.
    if (!m3BoundedPos3(def->position) || !(def->radius > 0.0f && def->radius <= M3_LOCAL_LIMIT) ||
        !(def->halfHeight >= 0.0f && def->halfHeight <= M3_LOCAL_LIMIT) ||
        !m3FiniteF(def->maxSlopeAngle) || def->maxSlopeAngle <= 0.0f || def->maxSlopeAngle > 1.5f ||
        !m3FiniteF(def->snapDistance) || def->snapDistance < 0.0f || !m3FiniteF(def->skin) ||
        !(def->skin > 0.0f) || !m3FiniteF(def->stepHeight) || def->stepHeight < 0.0f ||
        !(def->mass >= M3_MASS_FLOOR && def->mass <= M3_MASS_LIMIT) ||
        !(def->pushMaxMassRatio >= 0.0f && def->pushMaxMassRatio <= M3_RATIO_LIMIT))
    {
        return M3_CREATE_INVALID;
    }
    // A character takes a slot, a body and a shape. Room for all three is
    // checked first: a refused create is not journaled, so it must not
    // take and free a slot and move the generations.
    if (!m3IdPoolHasRoom(&world->characters.charPool) ||
        !m3IdPoolHasRoom(&world->bodies.bodyPool) || !m3IdPoolHasRoom(&world->shapes.shapePool))
    {
        return M3_CREATE_FULL;
    }
    int32_t slot = m3IdPoolAlloc(&world->characters.charPool);
    // The slot is live from here, but owns no body until its capsule is on
    // one: a stale body slot would refuse the capsule's create.
    world->characters.charBody[slot] = -1;
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
        return M3_CREATE_FULL;
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
                m3Vec3 rlc = m3RotateVec3(world->bodies.transforms[hitBody].q,
                                          world->bodies.localCenters[hitBody]);
                m3Vec3 arm = {(m3real)(hit.point.x - world->bodies.transforms[hitBody].p.x) - rlc.x,
                              (m3real)(hit.point.y - world->bodies.transforms[hitBody].p.y) - rlc.y,
                              (m3real)(hit.point.z - world->bodies.transforms[hitBody].p.z) -
                                  rlc.z};
                m3real mass = world->characters.charMass[slot];
                m3PushHeld(world, hitBody, -mass * blocked * into, n, arm);
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
    world->bodies.transforms[body].p = m3HoldPosition(pos);
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

static M3_NOINLINE m3Result CreateCharacterWorker(m3World* world, const m3CharacterDef* def,
                                                  m3CharacterId* characterOut)
{
    if (characterOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *characterOut = m3_nullCharacterId;
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    if (world == NULL || def == NULL || def->internalValue != M3_CHARACTER_COOKIE)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    int32_t slot = m3CreateCharacterInternal(world, def);
    if (slot < 0)
    {
        return m3CreateRefusal(world, slot);
    }
    m3CharacterId id = {slot + 1, world->characters.charPool.generations[slot]};
    if (world->recorder.journalActive != 0)
    {
        m3OpCreateCharacter record;
        memset(&record, 0, sizeof(record));
        m3JournalCharacterDef(&record.def, def);
        record.expected = id;
        m3JournalRecord(world, m3_opCreateCharacter, &record, (int32_t)sizeof(record));
    }
    *characterOut = id;
    return m3_success;
}

m3Result m3CreateCharacter(m3World* world, const m3CharacterDef* def, m3CharacterId* characterOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CreateCharacterWorker(world, def, characterOut);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result DestroyCharacterWorker(m3World* world, m3CharacterId character)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    int32_t slot = m3CharacterSlot(world, character);
    if (slot < 0)
    {
        return m3Refuse(world, m3MissingId(character.index1));
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyCharacter, &character, (int32_t)sizeof(character));
    }
    m3DestroyCharacterInternal(world, slot);
    return m3_success;
}

m3Result m3DestroyCharacter(m3World* world, m3CharacterId character)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = DestroyCharacterWorker(world, character);
    m3FloatEnvLeave(env);
    return result;
}

bool m3Character_IsValid(const m3World* world, m3CharacterId character)
{
    return m3CharacterSlot(world, character) >= 0;
}

static M3_NOINLINE void CharacterMoveWorker(m3World* world, m3CharacterId character,
                                            m3Vec3 translation)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot = m3CharacterSlot(world, character);
    if (slot < 0 || !m3CharacterMoveValid(translation))
    {
        // A stale id or a hostile move (non-finite, or past the caster's
        // float budget) changes nothing, never journals and counts once.
        m3Refuse(world, slot < 0 ? m3MissingId(character.index1) : m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpCharacterMove record;
        memset(&record, 0, sizeof(record));
        record.id = character;
        record.translation = translation;
        m3JournalRecord(world, m3_opCharacterMove, &record, (int32_t)sizeof(record));
    }
    m3CharacterMoveInternal(world, slot, translation);
}

void m3Character_Move(m3World* world, m3CharacterId character, m3Vec3 translation)
{
    uint64_t env = m3FloatEnvEnter();
    CharacterMoveWorker(world, character, translation);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE m3Pos3 CharacterGetPositionWorker(const m3World* world, m3CharacterId character)
{
    int32_t slot = m3CharacterSlot(world, character);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return world->bodies.transforms[world->characters.charBody[slot]].p;
}

m3Pos3 m3Character_GetPosition(const m3World* world, m3CharacterId character)
{
    uint64_t env = m3FloatEnvEnter();
    m3Pos3 result = CharacterGetPositionWorker(world, character);
    m3FloatEnvLeave(env);
    return result;
}

bool m3Character_IsGrounded(const m3World* world, m3CharacterId character)
{
    int32_t slot = m3CharacterSlot(world, character);
    if (slot < 0)
    {
        m3Refuse(world, m3MissingId(character.index1));
        return false;
    }
    return world->characters.charGrounded[slot] != 0;
}

static M3_NOINLINE m3Vec3 CharacterGetGroundNormalWorker(const m3World* world,
                                                         m3CharacterId character)
{
    int32_t slot = m3CharacterSlot(world, character);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return world->characters.charGroundNormal[slot];
}

m3Vec3 m3Character_GetGroundNormal(const m3World* world, m3CharacterId character)
{
    uint64_t env = m3FloatEnvEnter();
    m3Vec3 result = CharacterGetGroundNormalWorker(world, character);
    m3FloatEnvLeave(env);
    return result;
}

m3BodyId m3Character_GetGroundBody(const m3World* world, m3CharacterId character)
{
    int32_t slot = m3CharacterSlot(world, character);
    if (slot < 0)
    {
        m3Refuse(world, m3MissingId(character.index1));
        return m3_nullBodyId;
    }
    if (world->characters.charGrounded[slot] == 0)
    {
        return m3_nullBodyId;
    }
    int32_t under = world->characters.charGroundBody[slot];
    if (under < 0 || world->bodies.bodyPool.alive[under] == 0 ||
        world->bodies.bodyPool.generations[under] != world->characters.charGroundGen[slot])
    {
        return m3_nullBodyId; // destroyed or recycled: no impostors
    }
    return (m3BodyId){under + 1, world->bodies.bodyPool.generations[under]};
}

// The stance change: resize the capsule FEET ANCHORED. The
// veto casts the GROWN capsule at its new pose through the exact
// core every move uses (m3CastConvexClosestEx reports an initial
// overlap as a fraction-zero hit, so a wall against a wider radius
// refuses just like a pressing ceiling); the cast reaches one skin
// upward so the standing gap survives the change. Shrinks skip the
// veto by law: removing volume cannot create contact.
m3Result m3CharacterStanceInternal(m3World* world, int32_t slot, m3real halfHeight, m3real radius)
{
    if (!m3FiniteF(halfHeight) || !m3FiniteF(radius) || halfHeight < 0.05f || radius < 0.05f ||
        halfHeight > 10.0f || radius > 10.0f)
    {
        // Hostile dimensions never touch state. The public call counts
        // them as misuse; a replayed tape is data and counts nothing.
        return m3_errorInvalid;
    }
    int32_t body = world->characters.charBody[slot];
    int32_t shape = world->bodies.bodyShapeHead[body]; // the capsule is the
                                                       // character's only shape
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_capsuleShape)
    {
        // Unreachable: no call may destroy or reshape a character's own
        // capsule. Were one to, the stance must still write nothing.
        M3_ASSERT(false);
        return m3_errorState;
    }
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
            return m3_errorState; // the stand-up veto: something presses
        }
    }
    world->characters.charHalfHeight[slot] = halfHeight;
    world->characters.charRadius[slot] = radius;
    world->shapes.shapeGeom[shape].v = (m3Vec3){0.0f, halfHeight, 0.0f};
    world->shapes.shapeGeom[shape].v2 = (m3Vec3){0.0f, -halfHeight, 0.0f};
    world->shapes.shapeGeom[shape].s = radius;
    world->bodies.transforms[body].p = m3HoldPosition(newCenter);
    world->bodies.sleepTimes[body] = 0.0f;
    // The grounded state stays correct through the resize (the same
    // refresh a restore runs).
    RefreshGroundingCore(world, slot);
    return m3_success;
}

static M3_NOINLINE m3Result CharacterSetStanceWorker(m3World* world, m3CharacterId character,
                                                     m3real halfHeight, m3real radius)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    int32_t slot = m3CharacterSlot(world, character);
    if (slot < 0)
    {
        return m3Refuse(world, m3MissingId(character.index1));
    }
    m3Result result = m3CharacterStanceInternal(world, slot, halfHeight, radius);
    if (result == m3_errorInvalid)
    {
        return m3Refuse(world, result); // hostile dimensions
    }
    if (result != m3_success)
    {
        return result; // vetoed: not misuse, and never journals
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpCharacterStance record;
        memset(&record, 0, sizeof(record));
        record.id = character;
        record.halfHeight = halfHeight;
        record.radius = radius;
        m3JournalRecord(world, m3_opCharacterStance, &record, (int32_t)sizeof(record));
    }
    return m3_success;
}

m3Result m3Character_SetStance(m3World* world, m3CharacterId character, m3real halfHeight,
                               m3real radius)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = CharacterSetStanceWorker(world, character, halfHeight, radius);
    m3FloatEnvLeave(env);
    return result;
}

void m3Character_GetStance(const m3World* world, m3CharacterId character, m3real* halfHeightOut,
                           m3real* radiusOut)
{
    int32_t slot = m3CharacterSlot(world, character);
    if (slot < 0)
    {
        m3Refuse(world, m3MissingId(character.index1));
    }
    if (halfHeightOut != NULL)
    {
        *halfHeightOut = slot >= 0 ? world->characters.charHalfHeight[slot] : 0.0f;
    }
    if (radiusOut != NULL)
    {
        *radiusOut = slot >= 0 ? world->characters.charRadius[slot] : 0.0f;
    }
}
