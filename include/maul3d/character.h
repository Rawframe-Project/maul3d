// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The character controller: a kinematic capsule moved by
// collide-and-slide over the engine's convex casts. Its body never
// moves during m3World_Step; every displacement comes from
// m3Character_Move, a journaled command that rollback covers. Gravity
// is the host's job: add it to each move.

#ifndef MAUL3D_CHARACTER_H
#define MAUL3D_CHARACTER_H

#include "world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct m3CharacterId
    {
        uint32_t index1;
        uint32_t generation;
    } m3CharacterId;

    typedef struct m3CharacterDef
    {
        m3Pos3 position;         // the capsule CENTER at spawn, within 1e15 m
        m3real radius;           // positive, at most 1e4 m
        m3real halfHeight;       // segment half length, at most 1e4 m
                                 // (total height = 2 * (halfHeight +
                                 // radius))
        m3real maxSlopeAngle;    // radians from horizontal; steeper
                                 // ground never counts as walkable
        m3real snapDistance;     // how far down a grounded character
                                 // reaches to stay glued on descents
        m3real skin;             // the standing gap casts preserve
        m3real stepHeight;       // the tallest riser a walking
                                 // character mounts in one move
        m3real mass;             // kilograms, 1e-6 to 1e20; a blocked
                                 // move pushes dynamic bodies with
                                 // impulse = mass * blocked
                                 // displacement, which leaves them
                                 // within 1e5 m/s and rad/s
        m3real pushMaxMassRatio; // the heaviest body a push moves,
                                 // as a multiple of mass; heavier
                                 // bodies are walls (zero: never
                                 // push); at most 1e4
        M3_ALIGN8 uint64_t userData;
        int32_t internalValue;
    } m3CharacterDef;

    M3_API m3CharacterDef m3DefaultCharacterDef(void);

    /// Creates the character and its internal kinematic capsule
    /// body (destroyed together). The body collides like any
    /// kinematic: dynamics bounce off it; it walks through nothing.
    /// The character owns the body and the capsule, which queries
    /// report like any shape: m3DestroyShape, m3DestroyBody,
    /// m3Body_SetType, m3Shape_SetSphere, m3Shape_SetCapsule and a
    /// shape create on that body refuse them with m3_errorInvalid.
    /// Destroy the character instead; m3Character_SetStance resizes it.
    M3_NODISCARD M3_API m3Result m3CreateCharacter(m3World* world, const m3CharacterDef* def,
                                                   m3CharacterId* characterOut);
    M3_NODISCARD M3_API m3Result m3DestroyCharacter(m3World* world, m3CharacterId character);
    M3_API bool m3Character_IsValid(const m3World* world, m3CharacterId character);

    /// Collide-and-slide by `translation`, immediately: cast the
    /// capsule, advance to the first hit, slide the remainder along
    /// the surface, repeat (bounded), then snap to walkable ground
    /// within snapDistance when descending. Walkable is measured
    /// against maxSlopeAngle; ceilings and steep walls only ever
    /// block and slide. Journaled; a translation past 1e18 m on any
    /// axis, or not finite, is refused.
    M3_API void m3Character_Move(m3World* world, m3CharacterId character, m3Vec3 translation);

    M3_API m3Pos3 m3Character_GetPosition(const m3World* world, m3CharacterId character);
    M3_API bool m3Character_IsGrounded(const m3World* world, m3CharacterId character);
    /// The last walkable surface normal (zeros while airborne).
    M3_API m3Vec3 m3Character_GetGroundNormal(const m3World* world, m3CharacterId character);
    /// The body under the character's feet (null while airborne or
    /// after that body is destroyed). Each m3World_Step carries
    /// grounded characters along with this body's motion through
    /// the regular slide casts: kinematic platforms, elevators, and
    /// dynamic fragments all ferry their riders.
    M3_API m3BodyId m3Character_GetGroundBody(const m3World* world, m3CharacterId character);

    /// Stance: resize the capsule in place, FEET ANCHORED
    /// (the center moves so the capsule bottom stays level; a mid
    /// air crouch therefore lands shorter, not higher). Shrinking
    /// always applies. Growing runs the stand-up veto: the grown
    /// capsule is cast at its new pose through the same core every
    /// move uses, and ANY contact within the skin (a pressing
    /// ceiling, a wall against a wider radius) refuses the whole
    /// change with m3_errorState and counts no misuse; stance is
    /// untouched. Out-of-range or non-finite dimensions return
    /// m3_errorInvalid, a stale id m3_errorStale. Journaled
    /// only when applied; the veto's refusal is bit-deterministic
    /// so twins and replays refuse together. Stance is state:
    /// snapshotted, hashed, rolled back like everything else.
    M3_NODISCARD M3_API m3Result m3Character_SetStance(m3World* world, m3CharacterId character,
                                                       m3real halfHeight, m3real radius);
    /// Read the live stance (zeros on a stale id).
    M3_API void m3Character_GetStance(const m3World* world, m3CharacterId character,
                                      m3real* halfHeightOut, m3real* radiusOut);

    static const m3CharacterId m3_nullCharacterId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif
