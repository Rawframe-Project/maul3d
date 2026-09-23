// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Lattice-to-lattice work in the soft pass: particle contacts between
// different soft bodies and the soft-to-soft pins. Both run once per
// substep in canonical order (lower slot first, ascending particles) and
// keep no state of their own: contacts are transient projections.

#include "softbody.h"
#include "world_internal.h"

#include <math.h>

static void MovePos(m3Pos3* p, m3Vec3 d)
{
    p->x += (double)d.x;
    p->y += (double)d.y;
    p->z += (double)d.z;
}

static m3Vec3 PosDelta(const m3Pos3* a, const m3Pos3* b)
{
    return (m3Vec3){(m3real)(a->x - b->x), (m3real)(a->y - b->y), (m3real)(a->z - b->z)};
}

// The bounds of a lattice's predicted positions.
static void LatticeBounds(const m3World* world, int32_t slot, double lo[3], double hi[3])
{
    int32_t base = slot * M3_SOFTBODY_MAX_PARTICLES;
    lo[0] = lo[1] = lo[2] = 1.0e30;
    hi[0] = hi[1] = hi[2] = -1.0e30;
    for (int32_t i = 0; i < world->softBodies.softParticleCount[slot]; ++i)
    {
        const m3Pos3* p = &world->softBodies.softPos[base + i];
        lo[0] = p->x < lo[0] ? p->x : lo[0];
        lo[1] = p->y < lo[1] ? p->y : lo[1];
        lo[2] = p->z < lo[2] ? p->z : lo[2];
        hi[0] = p->x > hi[0] ? p->x : hi[0];
        hi[1] = p->y > hi[1] ? p->y : hi[1];
        hi[2] = p->z > hi[2] ? p->z : hi[2];
    }
}

// Pushes two overlapping particles apart, split by inverse mass, then
// applies PBD friction with a fixed mix: the tangential motion of this
// substep shrinks by mu times the correction. Dead-centered pairs have
// no deterministic normal and skip.
static void ProjectParticlePair(m3World* world, int32_t ka, int32_t kb, m3real target)
{
    m3SoftBodies* sb = &world->softBodies;
    m3Vec3 d = PosDelta(&sb->softPos[ka], &sb->softPos[kb]);
    m3real dist2 = m3Dot3(d, d);
    m3real wa = sb->softInvMass[ka];
    m3real wb = sb->softInvMass[kb];
    m3real wSum = wa + wb;
    if (dist2 >= target * target || dist2 <= 1.0e-12f || wSum <= 0.0f)
    {
        return;
    }
    m3real dist = sqrtf(dist2);
    m3Vec3 n = m3MulSV3(1.0f / dist, d);
    m3real pen = target - dist;
    MovePos(&sb->softPos[ka], m3MulSV3(pen * wa / wSum, n));
    MovePos(&sb->softPos[kb], m3MulSV3(-pen * wb / wSum, n));
    const m3real mu = 0.5f;
    m3Vec3 velA = PosDelta(&sb->softPos[ka], &sb->softPrev[ka]);
    m3Vec3 velB = PosDelta(&sb->softPos[kb], &sb->softPrev[kb]);
    m3Vec3 rel = m3Sub3(velA, velB);
    m3Vec3 tangential = m3Sub3(rel, m3MulSV3(m3Dot3(rel, n), n));
    m3real tLen = m3Length3(tangential);
    if (tLen > 1.0e-9f)
    {
        m3real budget = mu * pen;
        m3real cut = tLen < budget ? tLen : budget;
        m3Vec3 corr = m3MulSV3(cut / tLen, tangential);
        MovePos(&sb->softPos[ka], m3MulSV3(-wa / wSum, corr));
        MovePos(&sb->softPos[kb], m3MulSV3(wb / wSum, corr));
    }
}

// Particle pairs between different lattices. Self-collision stays out
// on purpose: a box lattice's structure rods hold it apart at these
// scales.
void m3SoftSoftContacts(m3World* world)
{
    m3SoftBodies* sb = &world->softBodies;
    for (int32_t a = 0; a < sb->softPool.maxIndex; ++a)
    {
        if (sb->softPool.alive[a] == 0)
        {
            continue;
        }
        double loA[3];
        double hiA[3];
        LatticeBounds(world, a, loA, hiA);
        for (int32_t b = a + 1; b < sb->softPool.maxIndex; ++b)
        {
            if (sb->softPool.alive[b] == 0)
            {
                continue;
            }
            m3real target = sb->softRadius[a] + sb->softRadius[b];
            double reach = (double)target;
            double loB[3];
            double hiB[3];
            LatticeBounds(world, b, loB, hiB);
            if (loA[0] > hiB[0] + reach || loB[0] > hiA[0] + reach || loA[1] > hiB[1] + reach ||
                loB[1] > hiA[1] + reach || loA[2] > hiB[2] + reach || loB[2] > hiA[2] + reach)
            {
                continue; // lattices out of reach
            }
            for (int32_t i = 0; i < sb->softParticleCount[a]; ++i)
            {
                for (int32_t j = 0; j < sb->softParticleCount[b]; ++j)
                {
                    ProjectParticlePair(world, a * M3_SOFTBODY_MAX_PARTICLES + i,
                                        b * M3_SOFTBODY_MAX_PARTICLES + j, target);
                }
            }
        }
    }
}

// Position equality between two lattices' particles, split by inverse
// mass; the lower slot owns the pin. Either side dying releases it.
void m3SoftSoftAnchors(m3World* world)
{
    m3SoftBodies* sb = &world->softBodies;
    for (int32_t sa = 0; sa < sb->softPool.maxIndex; ++sa)
    {
        for (int32_t a = 0; sb->softPool.alive[sa] != 0 && a < sb->softSoftCount[sa]; ++a)
        {
            int32_t ak = sa * M3_SOFTBODY_MAX_ANCHORS + a;
            int32_t other = sb->softSoftSlotB[ak];
            if (other < 0 || sb->softPool.alive[other] == 0 ||
                sb->softPool.generations[other] != sb->softSoftGenB[ak])
            {
                continue;
            }
            int32_t ka = sa * M3_SOFTBODY_MAX_PARTICLES + sb->softSoftParticleA[ak];
            int32_t kb = other * M3_SOFTBODY_MAX_PARTICLES + sb->softSoftParticleB[ak];
            m3real wa = sb->softInvMass[ka];
            m3real wb = sb->softInvMass[kb];
            m3real wSum = wa + wb;
            if (wSum <= 0.0f)
            {
                continue; // both pinned: nothing to split
            }
            m3Vec3 d = PosDelta(&sb->softPos[kb], &sb->softPos[ka]);
            MovePos(&sb->softPos[ka], m3MulSV3(wa / wSum, d));
            MovePos(&sb->softPos[kb], m3MulSV3(-wb / wSum, d));
        }
    }
}
