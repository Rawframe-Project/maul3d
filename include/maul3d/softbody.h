// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Soft bodies: XPBD particle lattices bound by distance constraints
// and solved in a fixed index order each substep. Rope, cloth and jelly
// come from one factory. Ids are pooled, operations journaled and state
// snapshotted and hashed, so rollback covers them exactly.

#ifndef MAUL3D_SOFTBODY_H
#define MAUL3D_SOFTBODY_H

#include "world.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define M3_MAX_SOFTBODY_PARTICLES  512
#define M3_MAX_SOFTBODY_EDGES      5120
#define M3_MAX_SOFTBODY_ANCHORS    32
#define M3_MAX_SOFTBODY_TETS       1024
#define M3_SOFTBODY_PRESSURE_LIMIT 100.0f

    typedef struct m3SoftBodyId
    {
        uint32_t index1;
        uint32_t generation;
    } m3SoftBodyId;

    typedef struct m3SoftBodyDef
    {
        m3Pos3 position;    // lattice minimum corner
        int32_t countX;     // particles per axis, each >= 1; the
        int32_t countY;     // product must fit
        int32_t countZ;     // M3_MAX_SOFTBODY_PARTICLES
        float spacing;      // rest distance between lattice neighbors, at most 1e4
        float particleMass; // per particle, kilograms
        float compliance;   // XPBD compliance; zero = rigid rods
        float radius;       // particle collision radius
        float gravityScale;
        M3_ALIGN8 uint64_t userData;
        /// Bend resistance: XPBD compliance of the SECOND
        /// NEIGHBOR tethers laid along each lattice axis (three
        /// straight points hold their spacing; a fold shortens it,
        /// and the tether pulls it straight). Zero (the default)
        /// adds no tethers;
        /// smaller positive values bend stiffer. Full Cosserat
        /// twist stays out by design: positions carry no frames.
        float bendCompliance;
        /// Internal pressure: a target volume multiplier for
        /// CLOSED lattices (every axis count >= 2). Zero (default)
        /// is off; 1 holds the create
        /// volume, 2 inflates toward double, up to
        /// M3_SOFTBODY_PRESSURE_LIMIT: a larger value is refused with
        /// m3_errorLimit, and pressure on a lattice that is not closed
        /// with m3_errorInvalid. One global volume constraint over the
        /// surface, solved beside the edges.
        float pressure;
        /// Bind-pose tether: every particle clamps to this
        /// radius around its CREATE position each substep. Zero
        /// (default) is off. The cheap skinned-vertex limit: cloth
        /// on a character cannot explode past its bind pose.
        float maxDeviation;
        int32_t internalValue;
    } m3SoftBodyDef;

    /// Returns a soft body def with the defaults: a 4 x 4 x 4 lattice at
    /// 0.25 m spacing, 0.1 kg particles of radius 0.05 m, rigid rods,
    /// gravity scale 1, and no bend tethers, pressure or bind-pose tether,
    /// at the origin.
    ///
    /// @return The def, with the cookie the creates check.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3SoftBodyDef m3DefaultSoftBodyDef(void);

    /// Creates the lattice with structural edges and face diagonals
    /// in fixed index order, leaving *softBodyOut the null id on any
    /// refusal. Journaled with id verification.
    ///
    /// @param world        The world.
    /// @param def          The def, from m3DefaultSoftBodyDef.
    /// @param softBodyOut  Receives the new id; any failure leaves the null
    ///                     id there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or
    ///         softBodyOut, a def m3DefaultSoftBodyDef did not make, an axis
    ///         count below 1, a NaN position or gravity scale, a spacing,
    ///         particle mass or radius that is not positive or is NaN, a
    ///         compliance, bendCompliance or maxDeviation that is not finite
    ///         or is negative, a pressure that is negative or NaN, or pressure
    ///         on a lattice with an axis count below 2; m3_errorLimit for more
    ///         than M3_MAX_SOFTBODY_PARTICLES particles or
    ///         M3_MAX_SOFTBODY_EDGES edges, a particle mass outside
    ///         M3_MIN_MASS to M3_MASS_LIMIT, a radius or spacing past
    ///         M3_LOCAL_LIMIT, a gravity scale past M3_RATIO_LIMIT, a pressure
    ///         past M3_SOFTBODY_PRESSURE_LIMIT, or a particle past
    ///         M3_POSITION_LIMIT on any axis, infinities included;
    ///         m3_errorCapacity for a full pool; m3_errorBusy from host code
    ///         the world is running. Every refusal but a full pool's and a
    ///         null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateSoftBody(m3World* world, const m3SoftBodyDef* def,
                                                  m3SoftBodyId* softBodyOut);

    /// A tetrahedral soft body: explicit points (world
    /// positions after adding def->position; each point within
    /// M3_LOCAL_LIMIT, 1e4 m, of def->position and each world position
    /// within 1e15 m on every axis, or m3_errorLimit) and tets (four
    /// point indices each, positive volume required). Edges come from
    /// the tet edges, deduplicated in first-touch order, at
    /// def->compliance; every tet holds its create volume rigidly
    /// (the incompressible jelly). The lattice's own fields do not
    /// apply: def->bendCompliance and def->pressure must be 0, or the
    /// create returns m3_errorInvalid, and def->countX/Y/Z and
    /// def->spacing are not read, so any value is accepted. Pins, anchors,
    /// wind, water, explosions, and collision all treat the
    /// particles exactly like lattice particles. Its refusals are
    /// listed below.
    ///
    /// @param world        The world.
    /// @param def          The def, from m3DefaultSoftBodyDef, with the
    ///                     lattice fields at their defaults.
    /// @param points       The points, relative to def->position.
    /// @param pointCount   The number of points, 4 to
    ///                     M3_MAX_SOFTBODY_PARTICLES.
    /// @param tets         Four point indices per tet.
    /// @param tetCount     The number of tets, 1 to M3_MAX_SOFTBODY_TETS.
    /// @param softBodyOut  Receives the new id; any failure leaves the null
    ///                     id there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def, points,
    ///         tets or softBodyOut, a def m3DefaultSoftBodyDef did not make,
    ///         a nonzero bendCompliance or pressure, a NaN position, point or
    ///         gravity scale, a compliance or maxDeviation that is not finite
    ///         or is negative, a particle mass or radius that is not positive
    ///         or is NaN, too few points or tets, or a tet that names a point
    ///         out of range or twice or has no positive volume; m3_errorLimit
    ///         for more than M3_MAX_SOFTBODY_PARTICLES points,
    ///         M3_MAX_SOFTBODY_TETS tets or M3_MAX_SOFTBODY_EDGES edges, a
    ///         point or position past its bound above, or a particle mass,
    ///         radius or gravity scale past its bound, infinities included;
    ///         m3_errorCapacity for a full pool; m3_errorBusy from host code
    ///         the world is running. Every refusal but a full pool's and a
    ///         null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateTetSoftBody(m3World* world, const m3SoftBodyDef* def,
                                                     const m3Vec3* points, int32_t pointCount,
                                                     const uint16_t* tets, int32_t tetCount,
                                                     m3SoftBodyId* softBodyOut);

    /// Destroys a soft body; its anchors and the soft pins that name it go
    /// with it. Journaled.
    ///
    /// @param world     The world.
    /// @param softBody  The soft body.
    /// @return m3_success; m3_errorInvalid for a null world or the null
    ///         id; m3_errorStale for an id whose soft body is gone;
    ///         m3_errorBusy from host code the world is running. Each
    ///         refusal but a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3DestroySoftBody(m3World* world, m3SoftBodyId softBody);

    /// Returns whether the id names a live soft body of this world.
    ///
    /// @param world     The world; null gives false.
    /// @param softBody  The id.
    /// @return Whether the soft body is alive.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3SoftBody_IsValid(const m3World* world, m3SoftBodyId softBody);

    /// Pins one particle in place (inverse mass zero): how a rope
    /// hangs and a flag flies. Journaled. Returns m3_success, or
    /// m3_errorStale for a stale id, m3_errorInvalid for the null id or
    /// an out-of-range particle and m3_errorBusy inside a step
    /// callback, which change nothing and count one misuse. Particle index is
    /// x + countX * (y + countY * z).
    ///
    /// @param world     The world.
    /// @param softBody  The soft body.
    /// @param particle  The particle index, 0 to the particle count - 1.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3SoftBody_PinParticle(m3World* world, m3SoftBodyId softBody,
                                                        int32_t particle);

    /// Anchors one particle to a body at the particle's CURRENT
    /// position, expressed in the body's frame: the particle rides
    /// the body from then on, and the lattice's pull on it lands on
    /// the body as an impulse at the anchor, so the coupling is two-way. Cloth
    /// hangs from beams and jelly rides trucks through this.
    /// Anchoring to a static body is a moving pin; the anchor
    /// RELEASES silently if its body dies. Journaled. Returns
    /// m3_success; m3_errorLimit for a full anchor table
    /// (M3_MAX_SOFTBODY_ANCHORS per soft body), which changes nothing
    /// and counts no misuse; m3_errorStale for a stale id,
    /// m3_errorInvalid for the null id or an out-of-range particle and
    /// m3_errorBusy inside a step callback, which change nothing and
    /// count one misuse.
    ///
    /// @param world     The world.
    /// @param softBody  The soft body.
    /// @param particle  The particle index, 0 to the particle count - 1.
    /// @param body      The body the particle rides; a stale one returns
    ///                  m3_errorStale and the null id m3_errorInvalid, each
    ///                  counting one misuse.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3SoftBody_AnchorParticle(m3World* world, m3SoftBodyId softBody,
                                                           int32_t particle, m3BodyId body);

    /// Pin a particle of one lattice to a particle of ANOTHER
    /// lattice: a position equality split by inverse mass,
    /// solved each substep after soft-vs-soft contact. Released
    /// silently when EITHER lattice dies. Journaled; the pin lives
    /// in the lower slot's table (one canonical home per pair), which
    /// holds M3_MAX_SOFTBODY_ANCHORS. Returns m3_success; m3_errorLimit
    /// for a full table, which changes nothing and counts no misuse;
    /// m3_errorStale for a stale id, m3_errorInvalid for the null id,
    /// the same lattice twice or an out-of-range particle and
    /// m3_errorBusy inside a step callback, which change nothing and
    /// count one misuse.
    ///
    /// @param world      The world.
    /// @param softBodyA  The first soft body.
    /// @param particleA  The first soft body's particle index.
    /// @param softBodyB  The second soft body, not softBodyA.
    /// @param particleB  The second soft body's particle index.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3SoftBody_AnchorToSoftBody(m3World* world, m3SoftBodyId softBodyA,
                                                             int32_t particleA,
                                                             m3SoftBodyId softBodyB,
                                                             int32_t particleB);

    /// Returns how many particles a soft body has.
    ///
    /// @param world     The world.
    /// @param softBody  The soft body.
    /// @return The particle count; 0 for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API int32_t m3SoftBody_GetParticleCount(const m3World* world, m3SoftBodyId softBody);

    /// Returns one particle's position.
    ///
    /// @param world     The world.
    /// @param softBody  The soft body.
    /// @param particle  The particle index, 0 to the particle count - 1.
    /// @return The position; the origin for a null world, and for a stale
    ///         or null id or an out-of-range particle, which counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3Pos3 m3SoftBody_GetParticlePosition(const m3World* world, m3SoftBodyId softBody,
                                                 int32_t particle);

    static const m3SoftBodyId m3_nullSoftBodyId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_SOFTBODY_H
