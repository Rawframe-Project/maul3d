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
        m3Pos3 position;        // the capsule CENTER at spawn, within 1e15 m
        float radius;           // positive, at most 1e4 m
        float halfHeight;       // segment half length, at most 1e4 m
                                // (total height = 2 * (halfHeight +
                                // radius))
        float maxSlopeAngle;    // radians from horizontal; steeper
                                // ground never counts as walkable
        float snapDistance;     // how far down a grounded character
                                // reaches to stay glued on descents
        float skin;             // the standing gap casts preserve
        float stepHeight;       // the tallest riser a walking
                                // character mounts in one move
        float mass;             // kilograms, 1e-6 to 1e20; a blocked
                                // move pushes dynamic bodies with
                                // impulse = mass * blocked
                                // displacement, which leaves them
                                // within 1e5 m/s and rad/s
        float pushMaxMassRatio; // the heaviest body a push moves,
                                // as a multiple of mass; heavier
                                // bodies are walls (zero: never
                                // push); at most 1e4
        M3_ALIGN8 uint64_t userData;
        int32_t internalValue;
    } m3CharacterDef;

    /// Returns a character def with the defaults: a 0.4 m radius, a 0.5 m
    /// half height, a 0.8 rad slope limit, a 0.3 m snap, a 0.02 m skin, a
    /// 0.35 m step, 80 kg and a push ratio of 2, at the origin.
    ///
    /// @return The def, with the cookie m3CreateCharacter checks.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3CharacterDef m3DefaultCharacterDef(void);

    /// Creates the character and its internal kinematic capsule
    /// body (destroyed together). The body collides like any
    /// kinematic: dynamics bounce off it; it walks through nothing.
    /// The character owns the body and the capsule, which queries
    /// report like any shape: m3DestroyShape, m3DestroyBody,
    /// m3Body_SetType, m3Shape_SetSphere, m3Shape_SetCapsule and a
    /// shape create on that body refuse them with m3_errorInvalid.
    /// Destroy the character instead; m3Character_SetStance resizes it.
    /// A def value past the bound its field names returns m3_errorLimit,
    /// any other bad field m3_errorInvalid. Journaled.
    ///
    /// @param world         The world.
    /// @param def           The def, from m3DefaultCharacterDef.
    /// @param characterOut  Receives the new id; any failure leaves the
    ///                      null id there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or
    ///         characterOut, a def m3DefaultCharacterDef did not make, a
    ///         slope outside (0, 1.5] rad, a snap distance or step height that
    ///         is negative or not finite, a skin that is not finite or not
    ///         positive, a radius or mass that is not positive or is NaN, a
    ///         half height or push ratio that is negative or NaN, or a NaN
    ///         position; m3_errorLimit for a position past M3_POSITION_LIMIT,
    ///         a radius or half height past M3_LOCAL_LIMIT, a mass outside
    ///         M3_MIN_MASS to M3_MASS_LIMIT or a push ratio past
    ///         M3_RATIO_LIMIT, infinities included; m3_errorCapacity when the
    ///         world has no room for the character, its body or its capsule;
    ///         m3_errorBusy from host code the world is running. Every refusal
    ///         but a full world's and a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateCharacter(m3World* world, const m3CharacterDef* def,
                                                   m3CharacterId* characterOut);

    /// Destroys a character with its body and capsule. Journaled.
    ///
    /// @param world      The world.
    /// @param character  The character.
    /// @return m3_success; m3_errorInvalid for a null world or the null
    ///         id; m3_errorStale for an id whose character is gone;
    ///         m3_errorBusy from host code the world is running. Each
    ///         refusal but a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3DestroyCharacter(m3World* world, m3CharacterId character);

    /// Returns whether the id names a live character of this world.
    ///
    /// @param world      The world; null gives false.
    /// @param character  The id.
    /// @return Whether the character is alive.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Character_IsValid(const m3World* world, m3CharacterId character);

    /// Collide-and-slide by `translation`, immediately: cast the
    /// capsule, advance to the first hit, slide the remainder along
    /// the surface, repeat (bounded), then snap to walkable ground
    /// within snapDistance when descending. Walkable is measured
    /// against maxSlopeAngle; ceilings and steep walls only ever
    /// block and slide. Journaled; a translation past 1e18 m on any
    /// axis, or not finite, is refused.
    ///
    /// @param world        The world.
    /// @param character    The character; a stale or null id moves nothing
    ///                     and counts one misuse.
    /// @param translation  The wanted displacement, meters. A refused one
    ///                     changes nothing, is not journaled and counts
    ///                     one misuse, as does a call from host code the
    ///                     world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Character_Move(m3World* world, m3CharacterId character, m3Vec3 translation);

    /// Returns the capsule's center.
    ///
    /// @param world      The world.
    /// @param character  The character.
    /// @return The center; the origin for a null world, and for a stale
    ///         or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Pos3 m3Character_GetPosition(const m3World* world, m3CharacterId character);

    /// Returns whether the character stands on walkable ground.
    ///
    /// @param world      The world.
    /// @param character  The character.
    /// @return Whether it is grounded; false for a null world, and for a
    ///         stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Character_IsGrounded(const m3World* world, m3CharacterId character);

    /// The last walkable surface normal (zeros while airborne).
    ///
    /// @param world      The world.
    /// @param character  The character.
    /// @return The normal; zeros while airborne, for a null world, and
    ///         for a stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Vec3 m3Character_GetGroundNormal(const m3World* world, m3CharacterId character);
    /// The body under the character's feet (null while airborne or
    /// after that body is destroyed). Each m3World_Step carries
    /// grounded characters along with this body's motion through
    /// the regular slide casts: kinematic platforms, elevators, and
    /// dynamic fragments all ferry their riders.
    ///
    /// @param world      The world.
    /// @param character  The character.
    /// @return The ground body's id; the null id while airborne, once that
    ///         body is gone, for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3BodyId m3Character_GetGroundBody(const m3World* world, m3CharacterId character);

    /// Stance: resize the capsule in place, FEET ANCHORED
    /// (the center moves so the capsule bottom stays level; a mid
    /// air crouch therefore lands shorter, not higher). Shrinking
    /// always applies. Growing runs the stand-up veto: the grown
    /// capsule is cast at its new pose through the same core every
    /// move uses, and ANY contact within the skin (a pressing
    /// ceiling, a wall against a wider radius) refuses the whole
    /// change with m3_errorState and counts no misuse; stance is
    /// untouched. A NaN dimension returns m3_errorInvalid, and one
    /// outside M3_MIN_STANCE to M3_STANCE_LIMIT m3_errorLimit, both
    /// counting one misuse; a stale id returns m3_errorStale. Journaled
    /// only when applied; the veto's refusal is bit-deterministic
    /// so twins and replays refuse together. Stance is state:
    /// snapshotted, hashed, rolled back like everything else.
    ///
    /// @param world       The world.
    /// @param character   The character.
    /// @param halfHeight  The new segment half length, meters.
    /// @param radius      The new radius, meters.
    /// @return m3_success; m3_errorInvalid for a null world, the null id or
    ///         a NaN dimension; m3_errorStale for a gone character;
    ///         m3_errorLimit for a dimension outside M3_MIN_STANCE to
    ///         M3_STANCE_LIMIT; m3_errorState when the stand-up veto
    ///         refuses; m3_errorBusy from host code the world is running.
    ///         Every refusal but the veto's and a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3Character_SetStance(m3World* world, m3CharacterId character,
                                                       float halfHeight, float radius);

    /// Read the live stance (zeros on a stale id).
    ///
    /// @param world          The world.
    /// @param character      The character; a stale or null id counts one
    ///                       misuse.
    /// @param halfHeightOut  Receives the segment half length, or 0 for a
    ///                       missing character; may be null.
    /// @param radiusOut      Receives the radius, or 0 for a missing
    ///                       character; may be null.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API void m3Character_GetStance(const m3World* world, m3CharacterId character,
                                      float* halfHeightOut, float* radiusOut);

    static const m3CharacterId m3_nullCharacterId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif
