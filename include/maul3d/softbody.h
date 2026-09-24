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

#define M3_SOFTBODY_MAX_PARTICLES 512
#define M3_SOFTBODY_MAX_EDGES     5120
#define M3_SOFTBODY_MAX_ANCHORS   32
#define M3_SOFTBODY_MAX_TETS      1024

    typedef struct m3SoftBodyId
    {
        int32_t index1;
        uint16_t world;
        uint16_t generation;
    } m3SoftBodyId;

    typedef struct m3SoftBodyDef
    {
        m3Pos3 position;     // lattice minimum corner
        int32_t countX;      // particles per axis, each >= 1; the
        int32_t countY;      // product must fit
        int32_t countZ;      // M3_SOFTBODY_MAX_PARTICLES
        m3real spacing;      // rest distance between lattice neighbors
        m3real particleMass; // per particle, kilograms
        m3real compliance;   // XPBD compliance; zero = rigid rods
        m3real radius;       // particle collision radius
        m3real gravityScale;
        uint64_t userData;
        /// Bend resistance: XPBD compliance of the SECOND
        /// NEIGHBOR tethers laid along each lattice axis (three
        /// straight points hold their spacing; a fold shortens it,
        /// and the tether pulls it straight). Zero (the default)
        /// adds no tethers;
        /// smaller positive values bend stiffer. Full Cosserat
        /// twist stays out by design: positions carry no frames.
        m3real bendCompliance;
        /// Internal pressure: a target volume multiplier for
        /// CLOSED lattices (every axis count >= 2). Zero (default)
        /// is off; 1 holds the create
        /// volume, 2 inflates toward double. One global volume
        /// constraint over the surface, solved beside the edges.
        m3real pressure;
        /// Bind-pose tether: every particle clamps to this
        /// radius around its CREATE position each substep. Zero
        /// (default) is off. The cheap skinned-vertex limit: cloth
        /// on a character cannot explode past its bind pose.
        m3real maxDeviation;
        int32_t internalValue;
    } m3SoftBodyDef;

    M3_API m3SoftBodyDef m3DefaultSoftBodyDef(void);

    /// Creates the lattice with structural edges and face diagonals
    /// in fixed index order. Refuses (null id) hostile counts,
    /// non-finite or non-positive geometry, or a full pool.
    /// Journaled with id verification.
    M3_API m3SoftBodyId m3CreateSoftBody(m3WorldId worldId, const m3SoftBodyDef* def);

    /// A tetrahedral soft body: explicit points (world
    /// positions after adding def->position) and tets (four point
    /// indices each, positive volume required). Edges come from
    /// the tet edges, deduplicated in first-touch order, at
    /// def->compliance; every tet holds its create volume rigidly
    /// (the incompressible jelly). def->countX/Y/Z, spacing,
    /// bendCompliance, and pressure must be left at defaults (the
    /// lattice knobs; hostile mixes refuse loudly). Pins, anchors,
    /// wind, water, explosions, and collision all treat the
    /// particles exactly like lattice particles.
    M3_API m3SoftBodyId m3CreateSoftBodyTet(m3WorldId worldId, const m3SoftBodyDef* def,
                                            const m3Vec3* points, int32_t pointCount,
                                            const uint16_t* tets, int32_t tetCount);
    M3_API void m3DestroySoftBody(m3SoftBodyId softId);
    M3_API bool m3SoftBody_IsValid(m3SoftBodyId softId);

    /// Pins one particle in place (inverse mass zero): how a rope
    /// hangs and a flag flies. Journaled; out-of-range or stale
    /// pins are quiet no-ops. Particle index is
    /// x + countX * (y + countY * z).
    M3_API void m3SoftBody_PinParticle(m3SoftBodyId softId, int32_t particle);

    /// Anchors one particle to a body at the particle's CURRENT
    /// position, expressed in the body's frame: the particle rides
    /// the body from then on, and the lattice's pull on it lands on
    /// the body as an impulse at the anchor, so the coupling is two-way. Cloth
    /// hangs from beams and jelly rides trucks through this.
    /// Anchoring to a static body is a moving pin; the anchor
    /// RELEASES silently if its body dies. Journaled; stale ids,
    /// out-of-range particles, and a full anchor table (32 per
    /// soft body) are quiet no-ops.
    M3_API void m3SoftBody_AnchorParticle(m3SoftBodyId softId, int32_t particle, m3BodyId bodyId);

    /// Pin a particle of one lattice to a particle of ANOTHER
    /// lattice: a position equality split by inverse mass,
    /// solved each substep after soft-vs-soft contact. Released
    /// silently when EITHER lattice dies. Journaled; the pin lives
    /// in the lower slot's table (one canonical home per pair).
    M3_API void m3SoftBody_AnchorToSoft(m3SoftBodyId softIdA, int32_t particleA,
                                        m3SoftBodyId softIdB, int32_t particleB);

    M3_API int32_t m3SoftBody_GetParticleCount(m3SoftBodyId softId);
    M3_API m3Pos3 m3SoftBody_GetParticlePosition(m3SoftBodyId softId, int32_t particle);

    static const m3SoftBodyId m3_nullSoftBodyId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_SOFTBODY_H
