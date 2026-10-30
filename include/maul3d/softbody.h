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
        /// volume, 2 inflates toward double, and no more than
        /// M3_SOFTBODY_PRESSURE_LIMIT is taken. One global volume
        /// constraint over the surface, solved beside the edges.
        float pressure;
        /// Bind-pose tether: every particle clamps to this
        /// radius around its CREATE position each substep. Zero
        /// (default) is off. The cheap skinned-vertex limit: cloth
        /// on a character cannot explode past its bind pose.
        float maxDeviation;
        int32_t internalValue;
    } m3SoftBodyDef;

    M3_API m3SoftBodyDef m3DefaultSoftBodyDef(void);

    /// Creates the lattice with structural edges and face diagonals
    /// in fixed index order. Refuses non-positive counts and NaN,
    /// negative or non-positive geometry with m3_errorInvalid; counts
    /// past M3_MAX_SOFTBODY_PARTICLES or M3_MAX_SOFTBODY_EDGES, a
    /// particle mass outside M3_MIN_MASS to M3_MASS_LIMIT, a radius or
    /// spacing past M3_LOCAL_LIMIT, a gravity scale past M3_RATIO_LIMIT
    /// and a particle past M3_POSITION_LIMIT on any axis with
    /// m3_errorLimit; and a full pool with m3_errorCapacity, leaving
    /// *softBodyOut the null id.
    /// Journaled with id verification.
    M3_NODISCARD M3_API m3Result m3CreateSoftBody(m3World* world, const m3SoftBodyDef* def,
                                                  m3SoftBodyId* softBodyOut);

    /// A tetrahedral soft body: explicit points (world
    /// positions after adding def->position, each within 1e15 m on
    /// every axis) and tets (four point indices each, positive
    /// volume required). Edges come from
    /// the tet edges, deduplicated in first-touch order, at
    /// def->compliance; every tet holds its create volume rigidly
    /// (the incompressible jelly). def->countX/Y/Z, spacing,
    /// bendCompliance, and pressure must be left at defaults (the
    /// lattice knobs; hostile mixes refuse loudly). Pins, anchors,
    /// wind, water, explosions, and collision all treat the
    /// particles exactly like lattice particles. It refuses as
    /// m3CreateSoftBody does, with m3_errorLimit past
    /// M3_MAX_SOFTBODY_TETS tets or M3_MAX_SOFTBODY_EDGES edges.
    M3_NODISCARD M3_API m3Result m3CreateTetSoftBody(m3World* world, const m3SoftBodyDef* def,
                                                     const m3Vec3* points, int32_t pointCount,
                                                     const uint16_t* tets, int32_t tetCount,
                                                     m3SoftBodyId* softBodyOut);
    M3_NODISCARD M3_API m3Result m3DestroySoftBody(m3World* world, m3SoftBodyId softBody);
    M3_API bool m3SoftBody_IsValid(const m3World* world, m3SoftBodyId softBody);

    /// Pins one particle in place (inverse mass zero): how a rope
    /// hangs and a flag flies. Journaled. Returns m3_success, or
    /// m3_errorStale for a stale id, m3_errorInvalid for the null id or
    /// an out-of-range particle and m3_errorBusy inside a step
    /// callback, which change nothing and count one misuse. Particle index is
    /// x + countX * (y + countY * z).
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
    M3_NODISCARD M3_API m3Result m3SoftBody_AnchorToSoft(m3World* world, m3SoftBodyId softBodyA,
                                                         int32_t particleA, m3SoftBodyId softBodyB,
                                                         int32_t particleB);

    M3_API int32_t m3SoftBody_GetParticleCount(const m3World* world, m3SoftBodyId softBody);
    M3_API m3Pos3 m3SoftBody_GetParticlePosition(const m3World* world, m3SoftBodyId softBody,
                                                 int32_t particle);

    static const m3SoftBodyId m3_nullSoftBodyId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_SOFTBODY_H
