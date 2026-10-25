// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape mass properties and the body mass they sum to.

#include "shape.h"
#include "world_internal.h"

#include <float.h>
#include <string.h>

// Invert a symmetric positive-definite 3x3 via the adjugate, writing its
// determinant to *detOut. A singular or non-positive matrix returns zero
// (an unrotatable body).
static m3Mat3 AdjugateInverse(m3Mat3 m, m3real* detOut)
{
    m3real a = m.cx.x;
    m3real b = m.cy.x; // = m.cx.y by symmetry
    m3real c = m.cz.x;
    m3real d = m.cy.y;
    m3real e = m.cz.y;
    m3real f = m.cz.z;
    m3real co00 = d * f - e * e;
    m3real co01 = c * e - b * f;
    m3real co02 = b * e - c * d;
    m3real det = a * co00 + b * co01 + c * co02;
    *detOut = det;
    if (!(det > 0.0f))
    {
        return m3MakeZeroMat3();
    }
    m3real inv = 1.0f / det;
    m3Mat3 r;
    r.cx = (m3Vec3){co00 * inv, co01 * inv, co02 * inv};
    r.cy = (m3Vec3){co01 * inv, (a * f - c * c) * inv, (b * c - a * e) * inv};
    r.cz = (m3Vec3){co02 * inv, (b * c - a * e) * inv, (a * d - b * b) * inv};
    return r;
}

static bool FiniteMat3(m3Mat3 m)
{
    return m3FiniteV3(m.cx) && m3FiniteV3(m.cy) && m3FiniteV3(m.cz);
}

// The adjugate works in the cube of the inertia's scale, which leaves
// float range for a body small or large enough. The direct result stands
// whenever its determinant is a normal float and every entry is finite,
// which covers every ordinary body bit for bit; otherwise the matrix is
// inverted at unit scale and the result rescaled. A body too thin even
// for that gets zero, as a singular one does.
static m3Mat3 InvertSymmetric(m3Mat3 m)
{
    m3real det;
    m3Mat3 direct = AdjugateInverse(m, &det);
    if (det >= FLT_MIN && det <= FLT_MAX && FiniteMat3(direct))
    {
        return direct;
    }
    m3real scale = m3MaxF(m.cx.x, m3MaxF(m.cy.y, m.cz.z));
    if (!(scale > 0.0f) || !(scale <= FLT_MAX))
    {
        return m3MakeZeroMat3();
    }
    m3real k = 1.0f / scale;
    m3Mat3 unit = {m3MulSV3(k, m.cx), m3MulSV3(k, m.cy), m3MulSV3(k, m.cz)};
    m3Mat3 r = AdjugateInverse(unit, &det);
    r = (m3Mat3){m3MulSV3(k, r.cx), m3MulSV3(k, r.cy), m3MulSV3(k, r.cz)};
    return FiniteMat3(r) ? r : m3MakeZeroMat3();
}

// Density-scaled mass, centroid, and centroid inertia of one shape.
// Returns 0 for shapes that carry no mass (planes).
// Compose a shape's mass properties through its compound offset
//: the raw props live in the SHAPE frame; the body wants
// them in ITS frame. c' = p + R c, I' = R I R^T. Identity offsets
// short-circuit: default scenes never enter the rotation.
static void ComposeMassProps(const m3World* world, int32_t s, m3Vec3* com, m3Mat3* inertia)
{
    if (world->shapes.shapeHasOffset[s] == 0)
    {
        return;
    }
    m3Quat q = world->shapes.shapeLocalRot[s];
    *com = m3Add3(world->shapes.shapeLocalPos[s], m3RotateVec3(q, *com));
    m3Mat3 r;
    r.cx = m3RotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f});
    r.cy = m3RotateVec3(q, (m3Vec3){0.0f, 1.0f, 0.0f});
    r.cz = m3RotateVec3(q, (m3Vec3){0.0f, 0.0f, 1.0f});
    // I' = R I R^T, built column by column: first T = I R^T, then
    // I' = R T. Columns of R^T are rows of R.
    m3Mat3 t;
    t.cx = (m3Vec3){inertia->cx.x * r.cx.x + inertia->cy.x * r.cx.y + inertia->cz.x * r.cx.z,
                    inertia->cx.y * r.cx.x + inertia->cy.y * r.cx.y + inertia->cz.y * r.cx.z,
                    inertia->cx.z * r.cx.x + inertia->cy.z * r.cx.y + inertia->cz.z * r.cx.z};
    t.cy = (m3Vec3){inertia->cx.x * r.cy.x + inertia->cy.x * r.cy.y + inertia->cz.x * r.cy.z,
                    inertia->cx.y * r.cy.x + inertia->cy.y * r.cy.y + inertia->cz.y * r.cy.z,
                    inertia->cx.z * r.cy.x + inertia->cy.z * r.cy.y + inertia->cz.z * r.cy.z};
    t.cz = (m3Vec3){inertia->cx.x * r.cz.x + inertia->cy.x * r.cz.y + inertia->cz.x * r.cz.z,
                    inertia->cx.y * r.cz.x + inertia->cy.y * r.cz.y + inertia->cz.y * r.cz.z,
                    inertia->cx.z * r.cz.x + inertia->cy.z * r.cz.y + inertia->cz.z * r.cz.z};
    inertia->cx = (m3Vec3){r.cx.x * t.cx.x + r.cy.x * t.cx.y + r.cz.x * t.cx.z,
                           r.cx.y * t.cx.x + r.cy.y * t.cx.y + r.cz.y * t.cx.z,
                           r.cx.z * t.cx.x + r.cy.z * t.cx.y + r.cz.z * t.cx.z};
    inertia->cy = (m3Vec3){r.cx.x * t.cy.x + r.cy.x * t.cy.y + r.cz.x * t.cy.z,
                           r.cx.y * t.cy.x + r.cy.y * t.cy.y + r.cz.y * t.cy.z,
                           r.cx.z * t.cy.x + r.cy.z * t.cy.y + r.cz.z * t.cy.z};
    inertia->cz = (m3Vec3){r.cx.x * t.cz.x + r.cy.x * t.cz.y + r.cz.x * t.cz.z,
                           r.cx.y * t.cz.x + r.cy.y * t.cz.y + r.cz.y * t.cz.z,
                           r.cx.z * t.cz.x + r.cy.z * t.cz.y + r.cz.z * t.cz.z};
}

// A shape's mass properties from its type, geometry and density, and for
// a hull its interned content.
static int MassPropsOf(uint8_t type, const m3ShapeGeom* geom, const m3HullData* hull, float density,
                       float* massOut, m3Vec3* comOut, m3Mat3* inertiaOut)
{
    if (type == (uint8_t)m3_sphereShape)
    {
        float r = geom->s;
        float m = density * (4.0f / 3.0f) * M3_PI * r * r * r;
        float ic = 0.4f * m * r * r;
        *massOut = m;
        *comOut = geom->v;
        *inertiaOut = m3MakeZeroMat3();
        inertiaOut->cx.x = ic;
        inertiaOut->cy.y = ic;
        inertiaOut->cz.z = ic;
        return 1;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        // Closed form: a cylinder of length L plus two hemispheres.
        // About the COM (the segment midpoint), with u the unit axis:
        //   I = Iperp * Identity + (Iaxial - Iperp) * (u outer u)
        // because t1(x)t1 + t2(x)t2 = Identity - u(x)u for any
        // orthonormal basis {t1, u, t2}. No basis matrix needed and
        // the result is exactly symmetric.
        m3Vec3 p1 = geom->v;
        m3Vec3 p2 = geom->v2;
        float r = geom->s;
        m3Vec3 axis = m3Sub3(p2, p1);
        float length = sqrtf(m3Dot3(axis, axis));
        // The create walls demand length > 0, but a mutated snapshot
        // writes the geometry slab directly and a zero or NaN segment
        // must not mint an inf axis here. With length zero the d
        // term below vanishes and the isotropic (sphere) inertia is
        // exactly right, so any unit stand-in axis is correct.
        m3Vec3 u = length > 0.0f ? m3MulSV3(1.0f / length, axis) : (m3Vec3){1.0f, 0.0f, 0.0f};
        float mCyl = density * M3_PI * r * r * length;
        float mSph = density * (4.0f / 3.0f) * M3_PI * r * r * r;
        float axial = 0.5f * mCyl * r * r + 0.4f * mSph * r * r;
        float perp = mCyl * (length * length / 12.0f + 0.25f * r * r) +
                     mSph * (0.4f * r * r + 0.25f * length * length + 0.375f * length * r);
        *massOut = mCyl + mSph;
        *comOut = m3MulSV3(0.5f, m3Add3(p1, p2));
        m3Mat3 ic2 = m3MakeZeroMat3();
        float d = axial - perp;
        ic2.cx = (m3Vec3){perp + d * u.x * u.x, d * u.x * u.y, d * u.x * u.z};
        ic2.cy = (m3Vec3){d * u.x * u.y, perp + d * u.y * u.y, d * u.y * u.z};
        ic2.cz = (m3Vec3){d * u.x * u.z, d * u.y * u.z, perp + d * u.z * u.z};
        *inertiaOut = ic2;
        return 1;
    }
    if (type == (uint8_t)m3_hullShape)
    {
        *massOut = density * hull->unitMass;
        *comOut = hull->unitCom;
        m3Mat3 ic = hull->unitInertiaCom;
        ic.cx = m3MulSV3(density, ic.cx);
        ic.cy = m3MulSV3(density, ic.cy);
        ic.cz = m3MulSV3(density, ic.cz);
        *inertiaOut = ic;
        return 1;
    }
    return 0;
}

// Shape s's interned hull, or NULL for a shape that holds none.
static const m3HullData* ShapeHull(const m3World* world, int32_t s)
{
    int32_t h = world->shapes.shapeHullIndex[s];
    return h >= 0 ? &world->hulls.hullData[h] : NULL;
}

static int ShapeMassProps(const m3World* world, int32_t s, float* massOut, m3Vec3* comOut,
                          m3Mat3* inertiaOut)
{
    return MassPropsOf(world->shapes.shapeType[s], &world->shapes.shapeGeom[s], ShapeHull(world, s),
                       world->shapes.shapeDensity[s], massOut, comOut, inertiaOut);
}

// Whether every principal moment of a symmetric inertia is at least
// floor: inertia - floor E is positive semidefinite exactly when all
// seven of its principal minors are non-negative. Doubles, no libm, so
// every platform decides alike.
static bool InertiaAtLeast(const m3Mat3* ic, double floor)
{
    double a = (double)ic->cx.x - floor;
    double d = (double)ic->cy.y - floor;
    double f = (double)ic->cz.z - floor;
    double b = (double)ic->cy.x;
    double c = (double)ic->cz.x;
    double e = (double)ic->cz.y;
    double det = a * (d * f - e * e) - b * (b * f - e * c) + c * (b * e - d * c);
    return a >= 0.0 && d >= 0.0 && f >= 0.0 && a * d - b * b >= 0.0 && a * f - c * c >= 0.0 &&
           d * f - e * e >= 0.0 && det >= 0.0; // false for NaN
}

bool m3ShapeMassFloors(uint8_t type, const m3ShapeGeom* geom, const m3HullData* hull, float density,
                       double* total)
{
    float m;
    m3Vec3 c;
    m3Mat3 ic;
    if (MassPropsOf(type, geom, hull, density, &m, &c, &ic) == 0)
    {
        return true;
    }
    *total += (double)m;
    return m >= M3_MASS_FLOOR && InertiaAtLeast(&ic, (double)M3_INERTIA_FLOOR);
}

// The mass floors and ceiling over body's shapes, with slot (or -1 for
// none) taking the given type, geometry and density.
static bool MassFits(const m3World* world, int32_t body, int32_t slot, uint8_t type,
                     const m3ShapeGeom* geom, float density)
{
    double total = 0.0;
    for (int32_t s = world->bodies.bodyShapeHead[body]; s != -1; s = world->shapes.shapeNext[s])
    {
        bool fits = s == slot ? m3ShapeMassFloors(type, geom, ShapeHull(world, s), density, &total)
                              : m3ShapeMassFloors(world->shapes.shapeType[s],
                                                  &world->shapes.shapeGeom[s], ShapeHull(world, s),
                                                  world->shapes.shapeDensity[s], &total);
        if (!fits)
        {
            return false;
        }
    }
    return total <= (double)M3_MASS_LIMIT;
}

bool m3ShapeMassFits(const m3World* world, int32_t slot, uint8_t type, const m3ShapeGeom* geom,
                     float density)
{
    int32_t body = world->shapes.shapeBody[slot];
    return world->bodies.types[body] != (uint8_t)m3_dynamicBody ||
           MassFits(world, body, slot, type, geom, density);
}

bool m3BodyMassFits(const m3World* world, int32_t body)
{
    return MassFits(world, body, -1, 0, NULL, 0.0f);
}

// Adds one shape's inertia about the body center: I += Ic + m (|d|^2 E -
// d d^T), the full parallel axis theorem, with non-negative diagonal
// terms.
static void AddShapeInertia(m3Mat3* inertia, float m, m3Vec3 c, m3Mat3 ic, m3Vec3 center)
{
    m3Vec3 d = m3Sub3(c, center);
    float d2 = m3Dot3(d, d);
    inertia->cx.x += ic.cx.x + m * (d2 - d.x * d.x);
    inertia->cy.y += ic.cy.y + m * (d2 - d.y * d.y);
    inertia->cz.z += ic.cz.z + m * (d2 - d.z * d.z);
    inertia->cy.x += ic.cy.x - m * d.x * d.y;
    inertia->cx.y += ic.cx.y - m * d.x * d.y;
    inertia->cz.x += ic.cz.x - m * d.x * d.z;
    inertia->cx.z += ic.cx.z - m * d.x * d.z;
    inertia->cz.y += ic.cz.y - m * d.y * d.z;
    inertia->cy.z += ic.cy.z - m * d.y * d.z;
}

// The body's center of mass in a shape's own frame, where its geometry
// lives.
static m3Vec3 CenterInShapeFrame(const m3World* world, int32_t s, m3Vec3 bodyCenter)
{
    if (world->shapes.shapeHasOffset[s] == 0)
    {
        return bodyCenter;
    }
    m3Quat q = world->shapes.shapeLocalRot[s];
    m3Quat inverse = {-q.x, -q.y, -q.z, q.w};
    return m3RotateVec3(inverse, m3Sub3(bodyCenter, world->shapes.shapeLocalPos[s]));
}

// Extents drive continuous collision: the smallest is the thinnest
// measure any shape brings (motion past half of it in one step marks the
// body fast), the largest bounds the rotation arc in the sweep advance.
// Distances are rotation invariant, so each shape measures in its frame.
static void BodyExtents(const m3World* world, int32_t bodyIndex, m3Vec3 bodyCenter, float* minOut,
                        float* maxOut)
{
    float minExtent = 1.0e30f;
    float maxExtent = 0.0f;
    for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        uint8_t type = world->shapes.shapeType[s];
        const m3ShapeGeom* g = &world->shapes.shapeGeom[s];
        m3Vec3 center = CenterInShapeFrame(world, s, bodyCenter);
        if (type == (uint8_t)m3_sphereShape)
        {
            m3Vec3 d = m3Sub3(g->v, center);
            minExtent = m3MinF(minExtent, g->s);
            maxExtent = m3MaxF(maxExtent, sqrtf(m3Dot3(d, d)) + g->s);
        }
        else if (type == (uint8_t)m3_capsuleShape)
        {
            m3Vec3 d1 = m3Sub3(g->v, center);
            m3Vec3 d2 = m3Sub3(g->v2, center);
            minExtent = m3MinF(minExtent, g->s);
            maxExtent = m3MaxF(maxExtent, sqrtf(m3MaxF(m3Dot3(d1, d1), m3Dot3(d2, d2))) + g->s);
        }
        else if (type == (uint8_t)m3_hullShape)
        {
            const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[s]];
            for (int32_t f = 0; f < hull->faceCount; ++f)
            {
                minExtent =
                    m3MinF(minExtent, hull->faceOffsets[f] - m3Dot3(hull->faceNormals[f], center));
            }
            for (int32_t v = 0; v < hull->vertexCount; ++v)
            {
                m3Vec3 d = m3Sub3(hull->vertices[v], center);
                maxExtent = m3MaxF(maxExtent, sqrtf(m3Dot3(d, d)));
            }
        }
    }
    *minOut = minExtent;
    *maxOut = maxExtent;
}

// Static, kinematic and shapeless dynamic bodies: no inertia and no
// extents. A shapeless dynamic body keeps unit mass.
static void SetMassless(m3World* world, int32_t bodyIndex, float invMass)
{
    world->bodies.invMass[bodyIndex] = invMass;
    world->bodies.invInertiaLocal[bodyIndex] = m3MakeZeroMat3();
    world->bodies.inertiaLocal[bodyIndex] = m3MakeZeroMat3();
    world->bodies.localCenters[bodyIndex] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodies.minExtents[bodyIndex] = 1.0e30f;
    world->bodies.maxExtents[bodyIndex] = 0.0f;
}

// Two passes: total mass and the mass-weighted center first, then the
// inertia about that center. Every term is non-negative and small, so no
// big-minus-big cancellation can occur.
void m3RecomputeMass(m3World* world, int32_t bodyIndex)
{
    if (world->bodies.types[bodyIndex] != (uint8_t)m3_dynamicBody)
    {
        SetMassless(world, bodyIndex, 0.0f);
        return;
    }
    float mass = 0.0f;
    m3Vec3 center = {0.0f, 0.0f, 0.0f};
    for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        float m;
        m3Vec3 c;
        m3Mat3 ic;
        if (ShapeMassProps(world, s, &m, &c, &ic))
        {
            ComposeMassProps(world, s, &c, &ic);
            mass += m;
            center = m3Add3(center, m3MulSV3(m, c));
        }
    }
    // A mass below the smallest normal float has no finite inverse, so
    // such a body is treated like one whose shapes carry no mass.
    if (!(mass >= FLT_MIN))
    {
        SetMassless(world, bodyIndex, 1.0f);
        return;
    }
    center = m3MulSV3(1.0f / mass, center);
    m3Mat3 inertia = m3MakeZeroMat3();
    for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        float m;
        m3Vec3 c;
        m3Mat3 ic;
        if (ShapeMassProps(world, s, &m, &c, &ic))
        {
            ComposeMassProps(world, s, &c, &ic);
            AddShapeInertia(&inertia, m, c, ic, center);
        }
    }
    world->bodies.invMass[bodyIndex] = 1.0f / mass;
    world->bodies.inertiaLocal[bodyIndex] = inertia; // the gyroscopic solve reads it
    world->bodies.invInertiaLocal[bodyIndex] = InvertSymmetric(inertia);
    BodyExtents(world, bodyIndex, center, &world->bodies.minExtents[bodyIndex],
                &world->bodies.maxExtents[bodyIndex]);
    world->bodies.localCenters[bodyIndex] = center;
}
