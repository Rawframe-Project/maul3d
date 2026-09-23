// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soft body pass, run inside the step: per substep, integrate every
// live particle under gravity, satisfy the distance constraints once in
// fixed edge order (the small-steps XPBD schedule: one Gauss-Seidel sweep
// per substep beats many sweeps per big step), collide against the
// world's planes and shapes, then derive velocities from the position
// delta. Every loop runs in ascending slot, particle and edge order.

#include "body.h"
#include "shape.h"
#include "softbody.h"
#include "solver.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

// Closest point on a triangle to a point (barycentric clamp, the
// textbook regions walk).
static m3Vec3 ClosestOnTriangle(m3Vec3 p, m3Vec3 a, m3Vec3 b, m3Vec3 c)
{
    m3Vec3 ab = m3Sub3(b, a);
    m3Vec3 ac = m3Sub3(c, a);
    m3Vec3 ap = m3Sub3(p, a);
    m3real d1 = m3Dot3(ab, ap);
    m3real d2 = m3Dot3(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        return a;
    }
    m3Vec3 bp = m3Sub3(p, b);
    m3real d3 = m3Dot3(ab, bp);
    m3real d4 = m3Dot3(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
    {
        return b;
    }
    m3real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        m3real v = d1 / (d1 - d3);
        return m3Add3(a, m3MulSV3(v, ab));
    }
    m3Vec3 cp = m3Sub3(p, c);
    m3real d5 = m3Dot3(ab, cp);
    m3real d6 = m3Dot3(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
    {
        return c;
    }
    m3real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        m3real w = d2 / (d2 - d6);
        return m3Add3(a, m3MulSV3(w, ac));
    }
    m3real va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        m3real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return m3Add3(b, m3MulSV3(w, m3Sub3(c, b)));
    }
    m3real denom = 1.0f / (va + vb + vc);
    m3real v = vb * denom;
    m3real w = vc * denom;
    return m3Add3(a, m3Add3(m3MulSV3(v, ab), m3MulSV3(w, ac)));
}

// Apply a projection to particle k: push along n (unit, outward) by
// depth, then the PBD friction rule against the shape's friction:
// tangential motion this substep dies when it is smaller than
// mu times the correction, and shrinks by that budget otherwise.
static void SoftProject(m3World* world, int32_t k, m3Vec3 n, m3real depth, m3real mu, int32_t body,
                        m3real invH)
{
    world->softBodies.softPos[k].x += (double)(depth * n.x);
    world->softBodies.softPos[k].y += (double)(depth * n.y);
    world->softBodies.softPos[k].z += (double)(depth * n.z);
    // Two-way: a particle pushed out of a dynamic body
    // pushes back, the wheel-reaction pattern: the projection is a
    // velocity change of depth over h on the particle's mass,
    // mirrored onto the body at the contact and waking it. Jelly
    // has weight now.
    if (body >= 0 && world->bodies.types[body] == (uint8_t)m3_dynamicBody &&
        world->bodies.invMass[body] > 0.0f && world->softBodies.softInvMass[k] > 0.0f)
    {
        m3real mp = 1.0f / world->softBodies.softInvMass[k];
        m3Vec3 J = m3MulSV3(-depth * invH * mp, n);
        m3Vec3 rlc =
            m3RotateVec3(world->bodies.transforms[body].q, world->bodies.localCenters[body]);
        m3Vec3 arm = {
            (m3real)(world->softBodies.softPos[k].x - world->bodies.transforms[body].p.x) - rlc.x,
            (m3real)(world->softBodies.softPos[k].y - world->bodies.transforms[body].p.y) - rlc.y,
            (m3real)(world->softBodies.softPos[k].z - world->bodies.transforms[body].p.z) - rlc.z};
        world->bodies.linearVelocities[body] =
            m3Add3(world->bodies.linearVelocities[body], m3MulSV3(world->bodies.invMass[body], J));
        world->bodies.angularVelocities[body] =
            m3Add3(world->bodies.angularVelocities[body],
                   m3MulMV3(m3WorldInvInertia(world, body), m3Cross3(arm, J)));
        world->bodies.awake[body] = 1;
        world->bodies.sleepTimes[body] = 0.0f;
    }
    if (mu <= 0.0f)
    {
        return;
    }
    m3Vec3 move = {(m3real)(world->softBodies.softPos[k].x - world->softBodies.softPrev[k].x),
                   (m3real)(world->softBodies.softPos[k].y - world->softBodies.softPrev[k].y),
                   (m3real)(world->softBodies.softPos[k].z - world->softBodies.softPrev[k].z)};
    m3Vec3 tang = m3Sub3(move, m3MulSV3(m3Dot3(move, n), n));
    m3real tl = sqrtf(m3Dot3(tang, tang));
    if (tl < 1.0e-9f)
    {
        return;
    }
    m3real budget = mu * depth;
    m3real scale = tl <= budget ? 1.0f : budget / tl;
    world->softBodies.softPos[k].x -= (double)(tang.x * scale);
    world->softBodies.softPos[k].y -= (double)(tang.y * scale);
    world->softBodies.softPos[k].z -= (double)(tang.z * scale);
}

// One particle against one shape, in the shape body's local frame.
static void SoftCollideParticle(m3World* world, int32_t slot, int32_t k, int32_t shape,
                                uint8_t stype, int32_t body, m3real radius, m3real invH)
{
    (void)slot;
    m3Transform xfS = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS;
    m3real mu = world->shapes.shapeFriction[shape];

    if (stype == (uint8_t)m3_planeShape)
    {
        m3Vec3 n = m3RotateVec3(xf->q, world->shapes.shapeGeom[shape].v);
        m3real offset =
            world->shapes.shapeGeom[shape].s +
            (m3real)((double)n.x * xf->p.x + (double)n.y * xf->p.y + (double)n.z * xf->p.z);
        m3real dist = (m3real)((double)n.x * world->softBodies.softPos[k].x +
                               (double)n.y * world->softBodies.softPos[k].y +
                               (double)n.z * world->softBodies.softPos[k].z) -
                      offset - radius;
        if (dist < 0.0f)
        {
            SoftProject(world, k, n, -dist, mu, body, invH);
        }
        return;
    }

    // Localize the particle into the body frame.
    m3Vec3 rel = {(m3real)(world->softBodies.softPos[k].x - xf->p.x),
                  (m3real)(world->softBodies.softPos[k].y - xf->p.y),
                  (m3real)(world->softBodies.softPos[k].z - xf->p.z)};
    m3Vec3 lp = m3InvRotateVec3(xf->q, rel);

    if (stype == (uint8_t)m3_sphereShape)
    {
        m3Vec3 d = m3Sub3(lp, world->shapes.shapeGeom[shape].v);
        m3real len = sqrtf(m3Dot3(d, d));
        m3real gap = len - world->shapes.shapeGeom[shape].s - radius;
        if (gap < 0.0f && len > 1.0e-9f)
        {
            m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
            SoftProject(world, k, n, -gap, mu, body, invH);
        }
        return;
    }
    if (stype == (uint8_t)m3_capsuleShape)
    {
        m3Vec3 a = world->shapes.shapeGeom[shape].v;
        m3Vec3 ab = m3Sub3(world->shapes.shapeGeom[shape].v2, a);
        m3real t = m3Dot3(m3Sub3(lp, a), ab) / m3MaxF(m3Dot3(ab, ab), 1.0e-12f);
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        m3Vec3 c = m3Add3(a, m3MulSV3(t, ab));
        m3Vec3 d = m3Sub3(lp, c);
        m3real len = sqrtf(m3Dot3(d, d));
        m3real gap = len - world->shapes.shapeGeom[shape].s - radius;
        if (gap < 0.0f && len > 1.0e-9f)
        {
            m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
            SoftProject(world, k, n, -gap, mu, body, invH);
        }
        return;
    }
    if (stype == (uint8_t)m3_hullShape)
    {
        // Boxes intern as hulls, so this branch carries them too.
        // Face planes serve both sides: outside within radius of
        // the closest face projects out; inside pushes along the
        // least penetrated face. Edge and vertex regions round
        // slightly toward the face answer: documented.
        const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[shape]];
        m3real best = -3.4e38f;
        int32_t bestFace = -1;
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            m3real sd = m3Dot3(hull->faceNormals[f], lp) - hull->faceOffsets[f];
            if (sd > best)
            {
                best = sd;
                bestFace = f;
            }
        }
        if (bestFace >= 0)
        {
            m3real gap = best - world->shapes.shapeGeom[shape].s - radius;
            if (gap < 0.0f)
            {
                m3Vec3 n = m3RotateVec3(xf->q, hull->faceNormals[bestFace]);
                SoftProject(world, k, n, -gap, mu, body, invH);
            }
        }
        return;
    }
    if (stype == (uint8_t)m3_voxelShape)
    {
        int32_t vslot = world->shapes.shapeVoxelIndex[shape];
        const m3VoxelChunkData* chunk = &world->voxels.voxelData[vslot];
        m3real cell = chunk->cellSize;
        int32_t cx = m3CellFromF(floorf(lp.x / cell), 2.0e9f);
        int32_t cy = m3CellFromF(floorf(lp.y / cell), 2.0e9f);
        int32_t cz = m3CellFromF(floorf(lp.z / cell), 2.0e9f);
        if (cx >= 0 && cx < 16 && cy >= 0 && cy < 16 && cz >= 0 && cz < 16 &&
            m3VoxelGet(chunk, cx, cy, cz))
        {
            // Deep inside filled voxels: the escape kernel owns it.
            m3Vec3 ln;
            m3real plane;
            if (m3VoxelEscape(world, vslot, lp, &ln, &plane))
            {
                m3real depth = plane - m3Dot3(ln, lp) + radius;
                if (depth > 0.0f)
                {
                    // The escape depth is unbounded (a moving chunk
                    // can hand a particle the middle of a thick
                    // wall), and XPBD turns a one-substep teleport
                    // into real velocity: the jelly probe watched a
                    // particle leave the rampart at 161 m/s. The
                    // rigid world already has the law: depenetration
                    // is a SPEED, capped by contactPushMaxSpeed. The
                    // particle obeys the same knob and walks out
                    // over a few substeps instead of detonating.
                    m3real maxPush = world->contactPushMaxSpeed / invH;
                    depth = depth > maxPush ? maxPush : depth;
                    SoftProject(world, k, m3RotateVec3(xf->q, ln), depth, mu, body, invH);
                }
            }
            return;
        }
        // Near the surface: clamp against the merged boxes the BVH
        // hands back around the particle.
        const m3VoxelSurface* surface = &world->voxels.voxelSurface[vslot];
        m3Vec3 qlo = {lp.x - radius, lp.y - radius, lp.z - radius};
        m3Vec3 qhi = {lp.x + radius, lp.y + radius, lp.z + radius};
        uint16_t boxes[32];
        int32_t nb = m3MeshBvhGather(&surface->bvh, qlo, qhi, boxes);
        nb = nb > 32 ? 32 : nb;
        for (int32_t b = 0; b < nb; ++b)
        {
            m3Vec3 blo;
            m3Vec3 bhi;
            m3VoxelBoxBounds(surface, cell, (int32_t)boxes[b], &blo, &bhi);
            m3Vec3 c = {lp.x < blo.x ? blo.x : (lp.x > bhi.x ? bhi.x : lp.x),
                        lp.y < blo.y ? blo.y : (lp.y > bhi.y ? bhi.y : lp.y),
                        lp.z < blo.z ? blo.z : (lp.z > bhi.z ? bhi.z : lp.z)};
            m3Vec3 d = m3Sub3(lp, c);
            m3real len2 = m3Dot3(d, d);
            if (len2 > 1.0e-12f && len2 < radius * radius)
            {
                m3real len = sqrtf(len2);
                m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
                SoftProject(world, k, n, radius - len, mu, body, invH);
                // Re-localize after the push for the next box.
                m3Vec3 rel2 = {(m3real)(world->softBodies.softPos[k].x - xf->p.x),
                               (m3real)(world->softBodies.softPos[k].y - xf->p.y),
                               (m3real)(world->softBodies.softPos[k].z - xf->p.z)};
                lp = m3InvRotateVec3(xf->q, rel2);
            }
        }
        return;
    }
    if (stype == (uint8_t)m3_heightFieldShape)
    {
        // Native terrain: the mesh recipe over the cell
        // gather, particle-sized window.
        const m3HeightFieldData* hf =
            &world->heightFields.hfData[world->shapes.shapeHfIndex[shape]];
        m3Vec3 hfTris[32][3];
        int32_t nt =
            m3HeightFieldGather(hf, (m3Vec3){lp.x - radius, lp.y - radius, lp.z - radius},
                                (m3Vec3){lp.x + radius, lp.y + radius, lp.z + radius}, hfTris, 32);
        for (int32_t t = 0; t < nt; ++t)
        {
            m3Vec3 cp = ClosestOnTriangle(lp, hfTris[t][0], hfTris[t][1], hfTris[t][2]);
            m3Vec3 d = m3Sub3(lp, cp);
            m3real len2 = m3Dot3(d, d);
            if (len2 > 1.0e-12f && len2 < radius * radius)
            {
                m3real len = sqrtf(len2);
                m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
                SoftProject(world, k, n, radius - len, mu, body, invH);
                m3Vec3 rel2 = {(m3real)(world->softBodies.softPos[k].x - xf->p.x),
                               (m3real)(world->softBodies.softPos[k].y - xf->p.y),
                               (m3real)(world->softBodies.softPos[k].z - xf->p.z)};
                lp = m3InvRotateVec3(xf->q, rel2);
            }
        }
        return;
    }
    if (stype == (uint8_t)m3_meshShape) // mesh-backed terrain interns here
    {
        const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
        const m3MeshBvh* bvh = &world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape]];
        m3Vec3 qlo = {lp.x - radius, lp.y - radius, lp.z - radius};
        m3Vec3 qhi = {lp.x + radius, lp.y + radius, lp.z + radius};
        uint16_t tris[32];
        int32_t nt = m3MeshBvhGather(bvh, qlo, qhi, tris);
        nt = nt > 32 ? 32 : nt;
        for (int32_t t = 0; t < nt; ++t)
        {
            int32_t tri = (int32_t)tris[t];
            m3Vec3 a = mesh->vertices[mesh->indices[3 * tri + 0]];
            m3Vec3 bb2 = mesh->vertices[mesh->indices[3 * tri + 1]];
            m3Vec3 cc = mesh->vertices[mesh->indices[3 * tri + 2]];
            m3Vec3 cp = ClosestOnTriangle(lp, a, bb2, cc);
            m3Vec3 d = m3Sub3(lp, cp);
            m3real len2 = m3Dot3(d, d);
            if (len2 > 1.0e-12f && len2 < radius * radius)
            {
                m3real len = sqrtf(len2);
                m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
                SoftProject(world, k, n, radius - len, mu, body, invH);
                m3Vec3 rel2 = {(m3real)(world->softBodies.softPos[k].x - xf->p.x),
                               (m3real)(world->softBodies.softPos[k].y - xf->p.y),
                               (m3real)(world->softBodies.softPos[k].z - xf->p.z)};
                lp = m3InvRotateVec3(xf->q, rel2);
            }
        }
        return;
    }
}

// The closed-lattice volume and its gradients: six faces
// walked in fixed order, outward winding, signed tet sum against a
// local origin (the first particle: double-safe far from zero).
// dV/da for triangle (a, b, c) is cross(b, c) / 6; the division by
// six is folded once at the call site.
static m3real SoftSurfaceVolume6(m3World* world, int32_t slot, m3Vec3* grads, m3Pos3 origin)
{
    int32_t nx = world->softBodies.softDimX[slot];
    int32_t ny = world->softBodies.softDimY[slot];
    int32_t nz = world->softBodies.softDimZ[slot];
    int32_t base = slot * M3_SOFTBODY_MAX_PARTICLES;
    int32_t count = world->softBodies.softParticleCount[slot];
    for (int32_t i = 0; i < count; ++i)
    {
        grads[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    m3real volume6 = 0.0f;
    for (int32_t face = 0; face < 6; ++face)
    {
        int32_t nu = face < 2 ? ny : nx;
        int32_t nv = face < 4 ? nz : ny;
        for (int32_t v = 0; v + 1 < nv; ++v)
        {
            for (int32_t u = 0; u + 1 < nu; ++u)
            {
                int32_t i00;
                int32_t i10;
                int32_t i01;
                int32_t i11;
                if (face < 2)
                {
                    int32_t x = face == 0 ? 0 : nx - 1;
                    i00 = x + nx * (u + ny * v);
                    i10 = x + nx * ((u + 1) + ny * v);
                    i01 = x + nx * (u + ny * (v + 1));
                    i11 = x + nx * ((u + 1) + ny * (v + 1));
                }
                else if (face < 4)
                {
                    int32_t y = face == 2 ? 0 : ny - 1;
                    i00 = u + nx * (y + ny * v);
                    i10 = (u + 1) + nx * (y + ny * v);
                    i01 = u + nx * (y + ny * (v + 1));
                    i11 = (u + 1) + nx * (y + ny * (v + 1));
                }
                else
                {
                    int32_t z = face == 4 ? 0 : nz - 1;
                    i00 = u + nx * (v + ny * z);
                    i10 = (u + 1) + nx * (v + ny * z);
                    i01 = u + nx * ((v + 1) + ny * z);
                    i11 = (u + 1) + nx * ((v + 1) + ny * z);
                }
                // Outward winding: faces 1 (+x), 2 (-y), 5 (+z) take
                // one diagonal orientation, their mirrors flip.
                int flip = face == 0 || face == 3 || face == 4;
                int32_t t0b = flip ? i01 : i10;
                int32_t t0c = i11;
                int32_t t1b = i11;
                int32_t t1c = flip ? i10 : i01;
                int32_t tris[2][3] = {{i00, t0b, t0c}, {i00, t1b, t1c}};
                for (int32_t t = 0; t < 2; ++t)
                {
                    m3Vec3 pa = {
                        (m3real)(world->softBodies.softPos[base + tris[t][0]].x - origin.x),
                        (m3real)(world->softBodies.softPos[base + tris[t][0]].y - origin.y),
                        (m3real)(world->softBodies.softPos[base + tris[t][0]].z - origin.z)};
                    m3Vec3 pb = {
                        (m3real)(world->softBodies.softPos[base + tris[t][1]].x - origin.x),
                        (m3real)(world->softBodies.softPos[base + tris[t][1]].y - origin.y),
                        (m3real)(world->softBodies.softPos[base + tris[t][1]].z - origin.z)};
                    m3Vec3 pc = {
                        (m3real)(world->softBodies.softPos[base + tris[t][2]].x - origin.x),
                        (m3real)(world->softBodies.softPos[base + tris[t][2]].y - origin.y),
                        (m3real)(world->softBodies.softPos[base + tris[t][2]].z - origin.z)};
                    volume6 += m3Dot3(pa, m3Cross3(pb, pc));
                    grads[tris[t][0]] = m3Add3(grads[tris[t][0]], m3Cross3(pb, pc));
                    grads[tris[t][1]] = m3Add3(grads[tris[t][1]], m3Cross3(pc, pa));
                    grads[tris[t][2]] = m3Add3(grads[tris[t][2]], m3Cross3(pa, pb));
                }
            }
        }
    }
    return volume6;
}

// The soft pass: XPBD small steps over the full shape set.
void m3SoftBodyPass(m3World* world, float dt, int32_t substeps)
{
    if (world->softBodies.softPool.maxIndex == 0)
    {
        return;
    }
    m3real h = dt / (m3real)substeps;
    m3real invH = h > 0.0f ? 1.0f / h : 0.0f;
    int32_t maxShape = world->shapes.shapePool.maxIndex;

    for (int32_t sub = 0; sub < substeps; ++sub)
    {
        for (int32_t slot = 0; slot < world->softBodies.softPool.maxIndex; ++slot)
        {
            if (world->softBodies.softPool.alive[slot] == 0)
            {
                continue;
            }
            int32_t count = world->softBodies.softParticleCount[slot];
            int32_t base = slot * M3_SOFTBODY_MAX_PARTICLES;

            // Anchored particles are driven, not integrated: mark
            // them for this substep (32 max, a cheap bitmask).
            uint8_t anchored[M3_SOFTBODY_MAX_PARTICLES / 8];
            memset(anchored, 0, sizeof(anchored));
            int32_t anchorCount = world->softBodies.softAnchorCount[slot];
            int32_t abase = slot * M3_SOFTBODY_MAX_ANCHORS;
            for (int32_t a = 0; a < anchorCount; ++a)
            {
                // A dead or recycled body releases its anchor: the
                // particle must return to the integrator, or it
                // hangs frozen in the air where its beam died.
                int32_t abody = world->softBodies.softAnchorBody[abase + a];
                if (abody < 0 || world->bodies.bodyPool.alive[abody] == 0 ||
                    world->bodies.bodyPool.generations[abody] !=
                        world->softBodies.softAnchorGen[abase + a])
                {
                    continue;
                }
                int32_t particle = world->softBodies.softAnchorParticle[abase + a];
                anchored[particle >> 3] |= (uint8_t)(1u << (particle & 7));
            }

            // Integrate: velocity from the previous position pair,
            // gravity, then the predicted position.
            m3Vec3 g = m3MulSV3(world->softBodies.softGravityScale[slot], world->gravity);
            // Wind: proportional drag toward the wind
            // velocity, gusted by the accumulated phase. Soft-only
            // by design (rigid bodies already have the force API).
            m3Vec3 windVel = {0.0f, 0.0f, 0.0f};
            m3real windDrag = 0.0f;
            if (world->windSpeed > 0.0f)
            {
                m3real gust = 1.0f + world->windGustScale * m3ComputeCosSin(world->windPhase).s;
                windVel = m3MulSV3(world->windSpeed * gust, world->windDir);
                windDrag = 0.5f; // documented fixed coefficient
            }
            for (int32_t i = 0; i < count; ++i)
            {
                int32_t k = base + i;
                if (world->softBodies.softInvMass[k] == 0.0f ||
                    (anchored[i >> 3] & (uint8_t)(1u << (i & 7))) != 0)
                {
                    world->softBodies.softPrev[k] = world->softBodies.softPos[k];
                    // A blast kick on a pinned particle evaporates:
                    // it must not linger in the hash forever.
                    world->softBodies.softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
                    continue;
                }
                m3Vec3 v = {
                    (m3real)(world->softBodies.softPos[k].x - world->softBodies.softPrev[k].x) *
                        invH,
                    (m3real)(world->softBodies.softPos[k].y - world->softBodies.softPrev[k].y) *
                        invH,
                    (m3real)(world->softBodies.softPos[k].z - world->softBodies.softPrev[k].z) *
                        invH};
                v = m3Add3(v, m3MulSV3(h, g));
                m3Vec3 kick = world->softBodies.softKick[k];
                if (kick.x != 0.0f || kick.y != 0.0f || kick.z != 0.0f)
                {
                    // The pending explosion kick lands exactly once,
                    // on the first substep that integrates it.
                    v = m3Add3(v, kick);
                    world->softBodies.softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
                }
                // The hard speed cap covers particles too: a mutated
                // blast once drove them into
                // float overflow, positions went NaN, and a NaN cell
                // index was undefined behavior. A capped velocity
                // can never outrun the double range.
                m3real pv2 = m3Dot3(v, v);
                m3real pcap = world->maximumLinearSpeed;
                if (pv2 > pcap * pcap)
                {
                    v = m3MulSV3(pcap / sqrtf(pv2), v);
                }
                if (windDrag > 0.0f)
                {
                    v = m3Add3(v, m3MulSV3(h * windDrag, m3Sub3(windVel, v)));
                }
                // Water: a submerged particle trades gravity
                // for buoyancy under the rho-1000 particle
                // convention (density 1000 suspends, denser fluids
                // lift, thinner ones let it sink) and drags toward
                // the flow like wind. First live volume wins,
                // deterministically, and dry worlds never reach the
                // loop (the pool is empty).
                for (int32_t wv = 0; wv < world->water.waterPool.maxIndex; ++wv)
                {
                    if (world->water.waterPool.alive[wv] == 0 ||
                        world->softBodies.softPos[k].x < world->water.waterLo[wv].x ||
                        world->softBodies.softPos[k].x > world->water.waterHi[wv].x ||
                        world->softBodies.softPos[k].y < world->water.waterLo[wv].y ||
                        world->softBodies.softPos[k].y > world->water.waterHi[wv].y ||
                        world->softBodies.softPos[k].z < world->water.waterLo[wv].z ||
                        world->softBodies.softPos[k].z > world->water.waterHi[wv].z)
                    {
                        continue;
                    }
                    v = m3Add3(v,
                               m3MulSV3(-h * world->water.waterDensity[wv] * (1.0f / 1000.0f), g));
                    v = m3Add3(v, m3MulSV3(h * world->water.waterLinDrag[wv],
                                           m3Sub3(world->water.waterFlow[wv], v)));
                    break;
                }
                world->softBodies.softPrev[k] = world->softBodies.softPos[k];
                world->softBodies.softPos[k].x += (double)(v.x * h);
                world->softBodies.softPos[k].y += (double)(v.y * h);
                world->softBodies.softPos[k].z += (double)(v.z * h);
            }

            // One Gauss-Seidel sweep over the edges, fixed order.
            m3real alpha = world->softBodies.softCompliance[slot] * invH * invH;
            m3real bendAlpha = world->softBodies.softBendCompliance[slot] * invH * invH;
            int32_t bendStart = world->softBodies.softBendStart[slot];
            int32_t edges = world->softBodies.softEdgeCount[slot];
            int32_t ebase = slot * M3_SOFTBODY_MAX_EDGES;
            for (int32_t e = 0; e < edges; ++e)
            {
                m3real alphaE = e >= bendStart ? bendAlpha : alpha;
                int32_t ka = base + (int32_t)world->softBodies.softEdgeA[ebase + e];
                int32_t kb = base + (int32_t)world->softBodies.softEdgeB[ebase + e];
                m3real wa = world->softBodies.softInvMass[ka];
                m3real wb = world->softBodies.softInvMass[kb];
                m3real wSum = wa + wb;
                if (wSum == 0.0f)
                {
                    continue;
                }
                m3Vec3 diff = {
                    (m3real)(world->softBodies.softPos[ka].x - world->softBodies.softPos[kb].x),
                    (m3real)(world->softBodies.softPos[ka].y - world->softBodies.softPos[kb].y),
                    (m3real)(world->softBodies.softPos[ka].z - world->softBodies.softPos[kb].z)};
                m3real len = sqrtf(m3Dot3(diff, diff));
                if (len < 1.0e-9f)
                {
                    continue; // coincident: no gradient, no correction
                }
                m3real c = len - world->softBodies.softEdgeRest[ebase + e];
                m3real scale = -c / ((wSum + alphaE) * len);
                world->softBodies.softPos[ka].x += (double)(wa * scale * diff.x);
                world->softBodies.softPos[ka].y += (double)(wa * scale * diff.y);
                world->softBodies.softPos[ka].z += (double)(wa * scale * diff.z);
                world->softBodies.softPos[kb].x -= (double)(wb * scale * diff.x);
                world->softBodies.softPos[kb].y -= (double)(wb * scale * diff.y);
                world->softBodies.softPos[kb].z -= (double)(wb * scale * diff.z);
            }

            // The bind tether: a hard clamp to the create
            // pose radius, BEFORE the volume rows so a crushed tet
            // still restores its volume (the tether is a bound,
            // the volumes are promises; documented order).
            if (world->softBodies.softMaxDeviation[slot] > 0.0f)
            {
                m3real maxDev = world->softBodies.softMaxDeviation[slot];
                for (int32_t i = 0; i < count; ++i)
                {
                    int32_t k2 = base + i;
                    if (world->softBodies.softInvMass[k2] == 0.0f)
                    {
                        continue;
                    }
                    m3Vec3 off = {(m3real)(world->softBodies.softPos[k2].x -
                                           world->softBodies.softBindPos[k2].x),
                                  (m3real)(world->softBodies.softPos[k2].y -
                                           world->softBodies.softBindPos[k2].y),
                                  (m3real)(world->softBodies.softPos[k2].z -
                                           world->softBodies.softBindPos[k2].z)};
                    m3real len2 = m3Dot3(off, off);
                    if (len2 > maxDev * maxDev)
                    {
                        m3real scale = maxDev / sqrtf(len2);
                        world->softBodies.softPos[k2].x =
                            world->softBodies.softBindPos[k2].x + (double)(off.x * scale);
                        world->softBodies.softPos[k2].y =
                            world->softBodies.softBindPos[k2].y + (double)(off.y * scale);
                        world->softBodies.softPos[k2].z =
                            world->softBodies.softBindPos[k2].z + (double)(off.z * scale);
                    }
                }
            }
            // Tet volume rows: one rigid row per tet in
            // fixed order (6V against the rest, the pressure
            // gradients localized to four particles).
            int32_t tetCount2 = world->softBodies.softTetCount[slot];
            for (int32_t t = 0; t < tetCount2; ++t)
            {
                int32_t tk = slot * M3_SOFTBODY_MAX_TETS + t;
                int32_t ia = base + (int32_t)world->softBodies.softTetA[tk];
                int32_t ib = base + (int32_t)world->softBodies.softTetB[tk];
                int32_t ic = base + (int32_t)world->softBodies.softTetC[tk];
                int32_t id2 = base + (int32_t)world->softBodies.softTetD[tk];
                m3Pos3 o = world->softBodies.softPos[ia];
                m3Vec3 pb2 = {(m3real)(world->softBodies.softPos[ib].x - o.x),
                              (m3real)(world->softBodies.softPos[ib].y - o.y),
                              (m3real)(world->softBodies.softPos[ib].z - o.z)};
                m3Vec3 pc2 = {(m3real)(world->softBodies.softPos[ic].x - o.x),
                              (m3real)(world->softBodies.softPos[ic].y - o.y),
                              (m3real)(world->softBodies.softPos[ic].z - o.z)};
                m3Vec3 pd2 = {(m3real)(world->softBodies.softPos[id2].x - o.x),
                              (m3real)(world->softBodies.softPos[id2].y - o.y),
                              (m3real)(world->softBodies.softPos[id2].z - o.z)};
                m3real v6 = m3Dot3(pb2, m3Cross3(pc2, pd2));
                m3real c6 = v6 - world->softBodies.softTetRestV6[tk];
                m3Vec3 gb = m3Cross3(pc2, pd2);
                m3Vec3 gc = m3Cross3(pd2, pb2);
                m3Vec3 gd = m3Cross3(pb2, pc2);
                m3Vec3 ga = m3Neg3(m3Add3(gb, m3Add3(gc, gd)));
                m3real wa = world->softBodies.softInvMass[ia];
                m3real wb = world->softBodies.softInvMass[ib];
                m3real wc = world->softBodies.softInvMass[ic];
                m3real wd = world->softBodies.softInvMass[id2];
                int32_t la = ia - base;
                int32_t lb = ib - base;
                int32_t lc = ic - base;
                int32_t ld = id2 - base;
                if ((anchored[la >> 3] & (uint8_t)(1u << (la & 7))) != 0)
                {
                    wa = 0.0f;
                }
                if ((anchored[lb >> 3] & (uint8_t)(1u << (lb & 7))) != 0)
                {
                    wb = 0.0f;
                }
                if ((anchored[lc >> 3] & (uint8_t)(1u << (lc & 7))) != 0)
                {
                    wc = 0.0f;
                }
                if ((anchored[ld >> 3] & (uint8_t)(1u << (ld & 7))) != 0)
                {
                    wd = 0.0f;
                }
                m3real denom = wa * m3Dot3(ga, ga) + wb * m3Dot3(gb, gb) + wc * m3Dot3(gc, gc) +
                               wd * m3Dot3(gd, gd);
                if (denom > 1.0e-9f)
                {
                    m3real lambda = -c6 / denom;
                    m3Vec3 dp;
                    dp = m3MulSV3(lambda * wa, ga);
                    world->softBodies.softPos[ia].x += (double)dp.x;
                    world->softBodies.softPos[ia].y += (double)dp.y;
                    world->softBodies.softPos[ia].z += (double)dp.z;
                    dp = m3MulSV3(lambda * wb, gb);
                    world->softBodies.softPos[ib].x += (double)dp.x;
                    world->softBodies.softPos[ib].y += (double)dp.y;
                    world->softBodies.softPos[ib].z += (double)dp.z;
                    dp = m3MulSV3(lambda * wc, gc);
                    world->softBodies.softPos[ic].x += (double)dp.x;
                    world->softBodies.softPos[ic].y += (double)dp.y;
                    world->softBodies.softPos[ic].z += (double)dp.z;
                    dp = m3MulSV3(lambda * wd, gd);
                    world->softBodies.softPos[id2].x += (double)dp.x;
                    world->softBodies.softPos[id2].y += (double)dp.y;
                    world->softBodies.softPos[id2].z += (double)dp.z;
                }
            }
            // The pressure row: one global volume constraint
            // per substep, PBD-projected in fixed particle order.
            // Gradients are of 6V, so the projection solves
            // C6 = 6 (V - target) against them directly: the sixes
            // cancel and no epsilon-sensitive division sneaks in.
            if (world->softBodies.softPressure[slot] > 0.0f)
            {
                m3Vec3 grads[M3_SOFTBODY_MAX_PARTICLES];
                m3Pos3 origin = world->softBodies.softPos[base];
                m3real vol6 = SoftSurfaceVolume6(world, slot, grads, origin);
                m3real c6 = vol6 - 6.0f * world->softBodies.softRestVolume[slot] *
                                       world->softBodies.softPressure[slot];
                m3real denom = 0.0f;
                for (int32_t i = 0; i < count; ++i)
                {
                    int pinned = world->softBodies.softInvMass[base + i] == 0.0f ||
                                 (anchored[i >> 3] & (uint8_t)(1u << (i & 7))) != 0;
                    if (!pinned)
                    {
                        denom +=
                            world->softBodies.softInvMass[base + i] * m3Dot3(grads[i], grads[i]);
                    }
                }
                if (denom > 1.0e-9f)
                {
                    m3real lambda = -c6 / denom;
                    for (int32_t i = 0; i < count; ++i)
                    {
                        int pinned = world->softBodies.softInvMass[base + i] == 0.0f ||
                                     (anchored[i >> 3] & (uint8_t)(1u << (i & 7))) != 0;
                        if (!pinned)
                        {
                            m3Vec3 dp = m3MulSV3(lambda * world->softBodies.softInvMass[base + i],
                                                 grads[i]);
                            world->softBodies.softPos[base + i].x += (double)dp.x;
                            world->softBodies.softPos[base + i].y += (double)dp.y;
                            world->softBodies.softPos[base + i].z += (double)dp.z;
                        }
                    }
                }
            }

            // Anchors: snap each anchored particle to its
            // body-frame target; the pull the lattice exerted on it
            // this substep (where the edges dragged it versus where
            // the body says it must be) lands on the body as an
            // impulse at the anchor. A dead or recycled body
            // releases its anchors silently.
            for (int32_t a = 0; a < anchorCount; ++a)
            {
                int32_t ak = abase + a;
                int32_t body = world->softBodies.softAnchorBody[ak];
                if (body < 0 || world->bodies.bodyPool.alive[body] == 0 ||
                    world->bodies.bodyPool.generations[body] != world->softBodies.softAnchorGen[ak])
                {
                    continue; // released
                }
                int32_t particle = world->softBodies.softAnchorParticle[ak];
                int32_t k = base + particle;
                const m3Transform* bxf = &world->bodies.transforms[body];
                m3Vec3 wl = m3RotateVec3(bxf->q, world->softBodies.softAnchorLocal[ak]);
                m3Pos3 target = {bxf->p.x + (double)wl.x, bxf->p.y + (double)wl.y,
                                 bxf->p.z + (double)wl.z};
                m3Vec3 wish = {(m3real)(world->softBodies.softPos[k].x - target.x),
                               (m3real)(world->softBodies.softPos[k].y - target.y),
                               (m3real)(world->softBodies.softPos[k].z - target.z)};
                world->softBodies.softPos[k] = target;
                world->softBodies.softPrev[k] = target;
                if (world->bodies.types[body] == (uint8_t)m3_dynamicBody &&
                    world->bodies.invMass[body] > 0.0f && world->softBodies.softInvMass[k] > 0.0f)
                {
                    m3real mp = 1.0f / world->softBodies.softInvMass[k];
                    m3Vec3 J = m3MulSV3(mp * invH, wish);
                    m3Vec3 rlc = m3RotateVec3(bxf->q, world->bodies.localCenters[body]);
                    m3Vec3 arm = m3Sub3(wl, rlc);
                    world->bodies.linearVelocities[body] =
                        m3Add3(world->bodies.linearVelocities[body],
                               m3MulSV3(world->bodies.invMass[body], J));
                    world->bodies.angularVelocities[body] =
                        m3Add3(world->bodies.angularVelocities[body],
                               m3MulMV3(m3WorldInvInertia(world, body), m3Cross3(arm, J)));
                    world->bodies.awake[body] = 1;
                    world->bodies.sleepTimes[body] = 0.0f;
                }
            }

            // The world's surfaces: every particle projects
            // out of every shape family through the shared local
            // kernels, ascending shape order, friction from the
            // touched shape (the PBD tangential rule). Planes stay
            // world-frame; everything else works in body space.
            m3real radius = world->softBodies.softRadius[slot];
            for (int32_t sShape = 0; sShape < maxShape; ++sShape)
            {
                if (world->shapes.shapePool.alive[sShape] == 0 ||
                    world->shapes.shapeSensor[sShape] != 0)
                {
                    continue;
                }
                uint8_t stype = world->shapes.shapeType[sShape];
                int32_t body = world->shapes.shapeBody[sShape];
                if (world->bodies.bodyEnabled[body] == 0)
                {
                    continue; // disabled bodies are ghosts to lattices too
                }
                for (int32_t i = 0; i < count; ++i)
                {
                    int32_t k = base + i;
                    if (world->softBodies.softInvMass[k] == 0.0f)
                    {
                        continue;
                    }
                    SoftCollideParticle(world, slot, k, sShape, stype, body, radius, invH);
                }
            }
        }

        m3SoftSoftContacts(world);
        m3SoftSoftAnchors(world);
    }
}
