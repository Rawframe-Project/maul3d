// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soft body pass's cull: before testing a soft body's particles
// against a shape, the pass asks whether any particle in the box that
// holds them can reach it. Each test is exact up to a stated rounding
// margin, so a culled pair is one the projection would leave alone and
// the cull changes no result.

#include "softbody_cull.h"

#include <float.h>
#include <math.h>

// The shape kinds a particle can be projected by only within its radius
// of their geometry: a sphere in the shape's local frame, center *center
// and the returned radius, holds that geometry. Every other kind returns
// -1: planes are unbounded, and a hull's face-plane rule can act past
// its bounds near an acute vertex, so hulls take their own test below.
m3real m3SoftShapeBound(const m3World* world, int32_t shape, uint8_t stype, m3Vec3* center)
{
    const m3ShapeGeom* geom = &world->shapes.shapeGeom[shape];
    if (stype == (uint8_t)m3_sphereShape)
    {
        *center = geom->v;
        return geom->s;
    }
    if (stype == (uint8_t)m3_capsuleShape)
    {
        *center = m3MulSV3(0.5f, m3Add3(geom->v, geom->v2));
        return 0.5f * m3Length3(m3Sub3(geom->v2, geom->v)) + geom->s;
    }
    m3Vec3 lo;
    m3Vec3 hi;
    if (stype == (uint8_t)m3_voxelShape)
    {
        m3real side =
            16.0f * world->voxels.voxelData[world->shapes.shapeVoxelIndex[shape]].cellSize;
        lo = (m3Vec3){0.0f, 0.0f, 0.0f};
        hi = (m3Vec3){side, side, side};
    }
    else if (stype == (uint8_t)m3_meshShape)
    {
        // The tree's root frame is the union of every triangle's bounds.
        const m3MeshBvh* bvh = &world->meshes.meshBvh[world->shapes.shapeMeshIndex[shape]];
        lo = bvh->rootLo;
        hi = bvh->rootHi;
    }
    else if (stype == (uint8_t)m3_heightFieldShape)
    {
        const m3HeightFieldData* hf =
            &world->heightFields.hfData[world->shapes.shapeHfIndex[shape]];
        lo = (m3Vec3){0.0f, hf->minHeight, 0.0f};
        hi = (m3Vec3){(m3real)(hf->nx - 1) * hf->cellSize, hf->maxHeight,
                      (m3real)(hf->nz - 1) * hf->cellSize};
    }
    else
    {
        return -1.0f;
    }
    *center = m3MulSV3(0.5f, m3Add3(lo, hi));
    return 0.5f * m3Length3(m3Sub3(hi, lo));
}

// How many float ulps of the magnitudes in play a cull keeps in hand.
// The projection computes in float in the shape's frame: the particle's
// offset from the body rounded to float, a quaternion rotation, then a
// distance or a closest point and a compare. Each step is within a few
// ulps of its operands, and the closest point on a triangle is a clamped
// convex combination of its corners, so the whole chain is within a few
// dozen ulps of the larger of the offset and the shape's local
// coordinates. The cull adds one float rotation. 256 ulps covers both
// several times, so a particle the projection would touch on its
// threshold is never culled by rounding.
#define M3_SOFT_REACH_ULPS 256.0

// Whether no particle inside [lo, hi] can reach the shape: the box is
// farther from the bound's world center than the bound plus the particle
// radius, plus the rounding margin above taken on the magnitudes a
// particle on that threshold brings: the center's offset, the bound and
// the radius. A culled pair is one the projection leaves alone.
bool m3SoftShapeOutOfReach(const m3Transform* xf, m3Vec3 center, m3real bound, m3real radius,
                           const double lo[3], const double hi[3])
{
    m3Vec3 r = m3RotateVec3(xf->q, center);
    double c[3] = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
    double d2 = 0.0;
    for (int32_t a = 0; a < 3; ++a)
    {
        double e = c[a] < lo[a] ? lo[a] - c[a] : (c[a] > hi[a] ? c[a] - hi[a] : 0.0);
        d2 += e * e;
    }
    double reach = (double)bound + (double)radius;
    double scale = (double)m3Length3(center) + reach;
    reach += M3_SOFT_REACH_ULPS * (double)FLT_EPSILON * scale;
    return d2 > reach * reach;
}

// Whether a hull projects no particle inside [lo, hi]. The projection
// acts while the largest of a particle's face-plane distances is below
// s + radius, so one face that has the whole box beyond s + radius
// leaves every particle alone, whatever the hull's shape near its
// vertices. The box is taken relative to the body, so large coordinates
// never meet a float, and the face is tested in world space: the least
// distance over the box is n_w . d - sum |n_w,i| h_i - o_f with n_w the
// rotated normal. The projection computes n_f . lp - o_f in float from a
// particle's offset, within a few dozen ulps of |offset| + |o_f|; the
// cull adds a float rotation of the normal, within a few ulps of
// |d| + sum h_i. No particle in the box is farther than that sum, so the
// margin is M3_SOFT_REACH_ULPS ulps of |d| + sum h_i + |o_f| + s + radius.
bool m3SoftHullOutOfReach(const m3World* world, int32_t shape, const m3Transform* xf, m3real radius,
                          const double lo[3], const double hi[3])
{
    const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[shape]];
    double threshold = (double)world->shapes.shapeGeom[shape].s + (double)radius;
    double d[3] = {0.5 * (lo[0] + hi[0]) - xf->p.x, 0.5 * (lo[1] + hi[1]) - xf->p.y,
                   0.5 * (lo[2] + hi[2]) - xf->p.z};
    double h[3] = {0.5 * (hi[0] - lo[0]), 0.5 * (hi[1] - lo[1]), 0.5 * (hi[2] - lo[2])};
    double scale = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) + h[0] + h[1] + h[2] + threshold;
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3Vec3 n = m3RotateVec3(xf->q, hull->faceNormals[f]);
        double nx = (double)n.x;
        double ny = (double)n.y;
        double nz = (double)n.z;
        double offset = (double)hull->faceOffsets[f];
        double least = nx * d[0] + ny * d[1] + nz * d[2] - fabs(nx) * h[0] - fabs(ny) * h[1] -
                       fabs(nz) * h[2] - offset;
        double margin = M3_SOFT_REACH_ULPS * (double)FLT_EPSILON * (scale + fabs(offset));
        if (least > threshold + margin)
        {
            return true;
        }
    }
    return false;
}

// Grow [lo, hi] to hold p.
void m3SoftGrowBox(double lo[3], double hi[3], m3Pos3 p)
{
    lo[0] = p.x < lo[0] ? p.x : lo[0];
    lo[1] = p.y < lo[1] ? p.y : lo[1];
    lo[2] = p.z < lo[2] ? p.z : lo[2];
    hi[0] = p.x > hi[0] ? p.x : hi[0];
    hi[1] = p.y > hi[1] ? p.y : hi[1];
    hi[2] = p.z > hi[2] ? p.z : hi[2];
}
