// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The narrow phase: one collider per shape-type pair, chosen from a
// [typeA][typeB] table, then the warm-start carry of each pair's old
// impulses by feature id. Pairs are rebuilt in pair order and each
// writes only its own manifold, so any split of the range is safe.

#include "narrowphase.h"
#include "distance.h"
#include "manifold.h"
#include "shape.h"
#include "world_internal.h"

#include <string.h>

typedef void m3CollidePairFn(m3World* world, m3Manifold* fresh, int32_t shapeA, int32_t shapeB);

static void CollideVoxelPair(m3World* world, m3Manifold* fresh, int32_t shapeA, int32_t shapeB)
{
    uint8_t typeA = world->shapes.shapeType[shapeA];
    memset(fresh, 0, sizeof(*fresh));
    int32_t voxelShape = typeA == (uint8_t)m3_voxelShape ? shapeA : shapeB;
    int32_t otherShape = voxelShape == shapeA ? shapeB : shapeA;
    uint8_t ot = world->shapes.shapeType[otherShape];
    if (ot == (uint8_t)m3_sphereShape || ot == (uint8_t)m3_capsuleShape ||
        ot == (uint8_t)m3_hullShape)
    {
        m3CollideVoxelConvex(world, fresh, voxelShape, otherShape, voxelShape == shapeA);
    }
}

static void CollideHeightFieldPair(m3World* world, m3Manifold* fresh, int32_t shapeA,
                                   int32_t shapeB)
{
    uint8_t typeA = world->shapes.shapeType[shapeA];
    memset(fresh, 0, sizeof(*fresh));
    int32_t hfShape = typeA == (uint8_t)m3_heightFieldShape ? shapeA : shapeB;
    int32_t otherShape = hfShape == shapeA ? shapeB : shapeA;
    uint8_t ot = world->shapes.shapeType[otherShape];
    if (ot == (uint8_t)m3_sphereShape || ot == (uint8_t)m3_capsuleShape ||
        ot == (uint8_t)m3_hullShape)
    {
        m3CollideHeightFieldConvex(world, fresh, hfShape, otherShape, hfShape == shapeA);
    }
}

static void CollideMeshPair(m3World* world, m3Manifold* fresh, int32_t shapeA, int32_t shapeB)
{
    uint8_t typeA = world->shapes.shapeType[shapeA];
    memset(fresh, 0, sizeof(*fresh));
    int32_t meshShape = typeA == (uint8_t)m3_meshShape ? shapeA : shapeB;
    int32_t otherShape = meshShape == shapeA ? shapeB : shapeA;
    if (world->shapes.shapeType[otherShape] != (uint8_t)m3_planeShape)
    {
        // Sphere, capsule, and hull all ride the welded
        // per-triangle pipeline.
        m3CollideMeshConvex(world, fresh, meshShape, otherShape, meshShape == shapeA);
    }
}

static void CollidePlaneHullPair(m3World* world, m3Manifold* fresh, int32_t shapeA, int32_t shapeB)
{
    uint8_t typeA = world->shapes.shapeType[shapeA];
    // Plane versus hull: every hull vertex below the margin is a
    // candidate, four of them spread over the patch survive, in
    // ascending vertex order. Feature id = vertex index, so the
    // carry follows each corner across rebuilds.
    memset(fresh, 0, sizeof(*fresh));
    int32_t planeShape = typeA == (uint8_t)m3_planeShape ? shapeA : shapeB;
    int32_t hullShape = planeShape == shapeA ? shapeB : shapeA;
    const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[hullShape]];
    int32_t planeBody = world->shapes.shapeBody[planeShape];
    int32_t hullBody = world->shapes.shapeBody[hullShape];
    m3Transform xfS = m3ShapeWorldTransform(world, hullShape);
    const m3Transform* xf = &xfS;
    m3Vec3 n = world->shapes.shapeGeom[planeShape].v;
    m3real offset = world->shapes.shapeGeom[planeShape].s;

    int32_t candIndex[M3_HULL_MAX_VERTS];
    m3real candSep[M3_HULL_MAX_VERTS];
    double candW[M3_HULL_MAX_VERTS][3];
    int32_t candCount = 0;
    for (int32_t v = 0; v < hull->vertexCount; ++v)
    {
        m3Vec3 r = m3RotateVec3(xf->q, hull->vertices[v]);
        double wx = xf->p.x + (double)r.x;
        double wy = xf->p.y + (double)r.y;
        double wz = xf->p.z + (double)r.z;
        double sep = (double)n.x * wx + (double)n.y * wy + (double)n.z * wz - (double)offset;
        if ((m3real)sep < M3_SPECULATIVE_DISTANCE)
        {
            candIndex[candCount] = v;
            candSep[candCount] = (m3real)sep;
            candW[candCount][0] = wx;
            candW[candCount][1] = wy;
            candW[candCount][2] = wz;
            candCount += 1;
        }
    }
    // Four candidates spread over the patch, in ascending vertex order.
    m3Vec3 candP[M3_HULL_MAX_VERTS];
    for (int32_t c = 0; c < candCount; ++c)
    {
        candP[c] =
            (m3Vec3){(m3real)(candW[c][0] - candW[0][0]), (m3real)(candW[c][1] - candW[0][1]),
                     (m3real)(candW[c][2] - candW[0][2])};
    }
    int32_t kept[M3_MANIFOLD_MAX_POINTS];
    int32_t keptCount = m3ReduceContactPoints(candP, candSep, candCount, n, kept);
    fresh->normal = n; // plane (A) toward hull (B)
    fresh->pointCount = keptCount;
    for (int32_t k = 0; k < keptCount; ++k)
    {
        int32_t c = kept[k];
        double sepD = (double)candSep[c];
        double px = candW[c][0] - (double)n.x * sepD;
        double py = candW[c][1] - (double)n.y * sepD;
        double pz = candW[c][2] - (double)n.z * sepD;
        fresh->points[k].anchorA = m3AnchorFromCom(world, planeBody, px, py, pz);
        fresh->points[k].anchorB =
            m3AnchorFromCom(world, hullBody, candW[c][0], candW[c][1], candW[c][2]);
        fresh->points[k].separation = candSep[c];
        fresh->points[k].id = (uint16_t)candIndex[c];
    }
    if (planeShape != shapeA)
    {
        fresh->normal = m3Neg3(fresh->normal);
        for (int32_t k = 0; k < keptCount; ++k)
        {
            m3Vec3 tmp = fresh->points[k].anchorA;
            fresh->points[k].anchorA = fresh->points[k].anchorB;
            fresh->points[k].anchorB = tmp;
        }
    }
}

static void CollideHullHullPair(m3World* world, m3Manifold* fresh, int32_t shapeA, int32_t shapeB)
{
    // Hull versus hull: the SAT runs in A's frame on a
    // float relative pose (doubles localized here), then the
    // manifold rotates out to world with COM-relative anchors.
    int32_t bodyA = world->shapes.shapeBody[shapeA];
    int32_t bodyB = world->shapes.shapeBody[shapeB];
    m3Transform xfAv = m3ShapeWorldTransform(world, shapeA);
    m3Transform xfBv = m3ShapeWorldTransform(world, shapeB);
    const m3Transform* xfA = &xfAv;
    const m3Transform* xfB = &xfBv;
    m3Quat conjA = {-xfA->q.x, -xfA->q.y, -xfA->q.z, xfA->q.w};
    m3Quat qRel = m3MulQuat(conjA, xfB->q);
    m3Vec3 dp = {(m3real)(xfB->p.x - xfA->p.x), (m3real)(xfB->p.y - xfA->p.y),
                 (m3real)(xfB->p.z - xfA->p.z)};
    m3Vec3 pRel = m3InvRotateVec3(xfA->q, dp);
    *fresh =
        m3CollideHulls(&world->hulls.hullData[world->shapes.shapeHullIndex[shapeA]],
                       &world->hulls.hullData[world->shapes.shapeHullIndex[shapeB]], qRel, pRel);
    if (fresh->pointCount > 0)
    {
        fresh->normal = m3RotateVec3(xfA->q, fresh->normal);
        for (int32_t k = 0; k < fresh->pointCount; ++k)
        {
            // Anchors arrive as positions in A's frame; lift to
            // world, then re-base to each COM.
            m3Vec3 lA = fresh->points[k].anchorA;
            m3Vec3 lB = fresh->points[k].anchorB;
            m3Vec3 rA = m3RotateVec3(xfA->q, lA);
            m3Vec3 rB = m3RotateVec3(xfA->q, lB);
            fresh->points[k].anchorA =
                m3AnchorFromCom(world, bodyA, xfA->p.x + (double)rA.x, xfA->p.y + (double)rA.y,
                                xfA->p.z + (double)rA.z);
            fresh->points[k].anchorB =
                m3AnchorFromCom(world, bodyB, xfA->p.x + (double)rB.x, xfA->p.y + (double)rB.y,
                                xfA->p.z + (double)rB.z);
        }
    }
}

static void CollidePlaneCapsulePair(m3World* world, m3Manifold* fresh, int32_t shapeA,
                                    int32_t shapeB)
{
    uint8_t typeA = world->shapes.shapeType[shapeA];
    // Plane versus capsule: the two cap centers are the only
    // candidates. Both inside the margin means the capsule
    // lies flat and gets the two-point manifold that keeps it
    // from rocking. Feature id = cap index (0 or 1), emitted
    // in cap order (canonical).
    memset(fresh, 0, sizeof(*fresh));
    int32_t planeShape = typeA == (uint8_t)m3_planeShape ? shapeA : shapeB;
    int32_t capShape = planeShape == shapeA ? shapeB : shapeA;
    int32_t planeBody = world->shapes.shapeBody[planeShape];
    int32_t capBody = world->shapes.shapeBody[capShape];
    m3Transform xfS = m3ShapeWorldTransform(world, capShape);
    const m3Transform* xf = &xfS;
    m3Vec3 n = world->shapes.shapeGeom[planeShape].v;
    m3real offset = world->shapes.shapeGeom[planeShape].s;
    m3real radius = world->shapes.shapeGeom[capShape].s;
    m3Vec3 caps[2] = {world->shapes.shapeGeom[capShape].v, world->shapes.shapeGeom[capShape].v2};
    int32_t count = 0;
    for (int32_t k = 0; k < 2; ++k)
    {
        m3Vec3 r = m3RotateVec3(xf->q, caps[k]);
        double wx = xf->p.x + (double)r.x;
        double wy = xf->p.y + (double)r.y;
        double wz = xf->p.z + (double)r.z;
        double centerDist = (double)n.x * wx + (double)n.y * wy + (double)n.z * wz - (double)offset;
        m3real sep = (m3real)centerDist - radius;
        if (sep > M3_SPECULATIVE_DISTANCE)
        {
            continue;
        }
        // anchorA: the cap center projected onto the plane.
        // anchorB: the deepest point of that cap sphere.
        double px = wx - (double)n.x * centerDist;
        double py = wy - (double)n.y * centerDist;
        double pz = wz - (double)n.z * centerDist;
        fresh->points[count].anchorA = m3AnchorFromCom(world, planeBody, px, py, pz);
        fresh->points[count].anchorB =
            m3AnchorFromCom(world, capBody, wx - (double)(n.x * radius),
                            wy - (double)(n.y * radius), wz - (double)(n.z * radius));
        fresh->points[count].separation = sep;
        fresh->points[count].id = (uint16_t)k;
        count += 1;
    }
    fresh->normal = n;
    fresh->pointCount = count;
    if (count > 0 && planeShape != shapeA)
    {
        fresh->normal = m3Neg3(fresh->normal);
        for (int32_t k = 0; k < count; ++k)
        {
            m3Vec3 tmp = fresh->points[k].anchorA;
            fresh->points[k].anchorA = fresh->points[k].anchorB;
            fresh->points[k].anchorB = tmp;
        }
    }
}

static void CollideHullCapsulePair(m3World* world, m3Manifold* fresh, int32_t shapeA,
                                   int32_t shapeB)
{
    uint8_t typeA = world->shapes.shapeType[shapeA];
    // Capsule versus hull: the segment SAT at every depth,
    // never GJK. GJK's one witness cannot hold a lying
    // capsule (it wobbles off the single point), and a deep
    // skewer needs the same SAT axes anyway. Doubles localize
    // into the hull frame, the kernel answers there, and the
    // manifold lifts out with COM anchors like every other
    // pair.
    memset(fresh, 0, sizeof(*fresh));
    int32_t hullShape = typeA == (uint8_t)m3_hullShape ? shapeA : shapeB;
    int32_t capShape = hullShape == shapeA ? shapeB : shapeA;
    int32_t hullBody = world->shapes.shapeBody[hullShape];
    int32_t capBody = world->shapes.shapeBody[capShape];
    m3Transform xfHv = m3ShapeWorldTransform(world, hullShape);
    m3Transform xfCv = m3ShapeWorldTransform(world, capShape);
    const m3Transform* xfH = &xfHv;
    const m3Transform* xfC = &xfCv;
    m3Quat conjH = {-xfH->q.x, -xfH->q.y, -xfH->q.z, xfH->q.w};
    m3Quat qRel = m3MulQuat(conjH, xfC->q);
    m3Vec3 dp = {(m3real)(xfC->p.x - xfH->p.x), (m3real)(xfC->p.y - xfH->p.y),
                 (m3real)(xfC->p.z - xfH->p.z)};
    m3Vec3 pRel = m3InvRotateVec3(xfH->q, dp);
    m3Vec3 s1 = m3Add3(m3RotateVec3(qRel, world->shapes.shapeGeom[capShape].v), pRel);
    m3Vec3 s2 = m3Add3(m3RotateVec3(qRel, world->shapes.shapeGeom[capShape].v2), pRel);
    const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[hullShape]];
    m3Manifold local = m3CollideSegmentHull(hull, s1, s2, world->shapes.shapeGeom[capShape].s);
    if (local.pointCount > 0)
    {
        m3Vec3 nWorld = m3RotateVec3(xfH->q, local.normal); // hull toward capsule
        fresh->normal = hullShape == shapeA ? nWorld : m3Neg3(nWorld);
        fresh->pointCount = local.pointCount;
        for (int32_t k = 0; k < local.pointCount; ++k)
        {
            m3Vec3 rH = m3RotateVec3(xfH->q, local.points[k].anchorA);
            m3Vec3 rC = m3RotateVec3(xfH->q, local.points[k].anchorB);
            m3Vec3 aHull = m3AnchorFromCom(world, hullBody, xfH->p.x + (double)rH.x,
                                           xfH->p.y + (double)rH.y, xfH->p.z + (double)rH.z);
            m3Vec3 aCap = m3AnchorFromCom(world, capBody, xfH->p.x + (double)rC.x,
                                          xfH->p.y + (double)rC.y, xfH->p.z + (double)rC.z);
            fresh->points[k].anchorA = hullShape == shapeA ? aHull : aCap;
            fresh->points[k].anchorB = hullShape == shapeA ? aCap : aHull;
            fresh->points[k].separation = local.points[k].separation;
            fresh->points[k].id = local.points[k].id;
        }
    }
}

static void CollideConvexGjkPair(m3World* world, m3Manifold* fresh, int32_t shapeA, int32_t shapeB)
{
    uint8_t typeA = world->shapes.shapeType[shapeA];
    uint8_t typeB = world->shapes.shapeType[shapeB];
    int hullPair = typeA == (uint8_t)m3_hullShape || typeB == (uint8_t)m3_hullShape;
    // The remaining convex pairs (hull-sphere, capsule-sphere,
    // capsule-capsule): GJK on the cores in A's frame, radii
    // applied analytically, one contact point, feature id 0.
    // Deep core overlap has an exact answer for each family;
    // EPA never became necessary (hull-hull and capsule-hull
    // deep pairs go through their SATs).
    memset(fresh, 0, sizeof(*fresh));
    int32_t bodyA = world->shapes.shapeBody[shapeA];
    int32_t bodyB = world->shapes.shapeBody[shapeB];
    m3Transform xfAv = m3ShapeWorldTransform(world, shapeA);
    m3Transform xfBv = m3ShapeWorldTransform(world, shapeB);
    const m3Transform* xfA = &xfAv;
    const m3Transform* xfB = &xfBv;
    m3Quat conjA = {-xfA->q.x, -xfA->q.y, -xfA->q.z, xfA->q.w};
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.q = m3MulQuat(conjA, xfB->q);
    m3Vec3 dp = {(m3real)(xfB->p.x - xfA->p.x), (m3real)(xfB->p.y - xfA->p.y),
                 (m3real)(xfB->p.z - xfA->p.z)};
    input.p = m3InvRotateVec3(xfA->q, dp);
    m3Vec3 pointsA[2];
    m3Vec3 pointsB[2];
    input.proxyA = m3MakeShapeProxy(world, shapeA, pointsA);
    input.proxyB = m3MakeShapeProxy(world, shapeB, pointsB);
    input.useRadii = false;
    m3DistanceOutput out = m3ShapeDistance(&input);
    m3real rA = input.proxyA.radius;
    m3real rB = input.proxyB.radius;
    m3real sep = out.distance - rA - rB;
    if (sep <= M3_SPECULATIVE_DISTANCE)
    {
        m3Vec3 nLocal;
        m3Vec3 pALocal;
        m3Vec3 pBLocal;
        if (out.distance > 0.0f)
        {
            nLocal = out.normal;
            pALocal = m3Add3(out.pointA, m3MulSV3(rA, nLocal));
            pBLocal = m3Sub3(out.pointB, m3MulSV3(rB, nLocal));
        }
        else if (hullPair)
        {
            // A sphere center inside a hull: the least-deep
            // face is the exact minimum translation.
            int hullIsA = typeA == (uint8_t)m3_hullShape;
            const m3HullData* hull =
                &world->hulls.hullData[world->shapes.shapeHullIndex[hullIsA ? shapeA : shapeB]];
            const m3DistanceProxy* round = hullIsA ? &input.proxyB : &input.proxyA;
            m3Vec3 core = hullIsA ? m3Add3(m3RotateVec3(input.q, round->points[0]), input.p)
                                  : m3InvRotateVec3(input.q, m3Sub3(round->points[0], input.p));
            m3Vec3 nHull; // hull toward sphere, hull frame
            m3real coreSep;
            m3Vec3 onHull;
            m3DeepPointInHull(hull, core, &nHull, &coreSep, &onHull);
            m3real rRound = round->radius;
            sep = coreSep - rRound;
            if (hullIsA)
            {
                nLocal = nHull;
                pALocal = onHull;
                pBLocal = m3Sub3(core, m3MulSV3(rRound, nHull));
            }
            else
            {
                m3Vec3 nA = m3RotateVec3(input.q, nHull);
                nLocal = m3Neg3(nA);
                pALocal =
                    m3Sub3(m3Add3(m3RotateVec3(input.q, core), input.p), m3MulSV3(rRound, nA));
                pBLocal = m3Add3(m3RotateVec3(input.q, onHull), input.p);
            }
        }
        else
        {
            // Sphere and capsule cores meeting exactly (a
            // center on a segment, two segments crossing):
            // measure-zero poses. The mutual perpendicular is
            // the exact axis and the skins overlap by exactly
            // rA + rB along it.
            m3Vec3 axis;
            m3Vec3 cA;
            m3Vec3 cB;
            if (input.proxyA.count == 2 && input.proxyB.count == 2)
            {
                m3Vec3 dirA = m3Sub3(pointsA[1], pointsA[0]);
                m3Vec3 dirB = m3RotateVec3(input.q, m3Sub3(pointsB[1], pointsB[0]));
                axis = m3Cross3(dirA, dirB);
                cA = m3MulSV3(0.5f, m3Add3(pointsA[0], pointsA[1]));
                cB = m3Add3(m3RotateVec3(input.q, m3MulSV3(0.5f, m3Add3(pointsB[0], pointsB[1]))),
                            input.p);
                if (m3Dot3(axis, axis) < 1.0e-10f)
                {
                    axis = m3Sub3(cB, cA); // parallel: center delta
                }
            }
            else
            {
                // Capsule versus sphere: any segment
                // perpendicular works; the tangent basis rule
                // makes the pick bit-stable.
                m3Vec3 dir = input.proxyA.count == 2
                                 ? m3Sub3(pointsA[1], pointsA[0])
                                 : m3RotateVec3(input.q, m3Sub3(pointsB[1], pointsB[0]));
                m3Vec3 t1;
                m3Vec3 t2;
                m3MakeTangentBasis(m3Normalize3(dir), &t1, &t2);
                axis = t1;
                cA = input.proxyA.count == 1 ? pointsA[0]
                                             : m3MulSV3(0.5f, m3Add3(pointsA[0], pointsA[1]));
                cB = input.proxyB.count == 1
                         ? m3Add3(m3RotateVec3(input.q, pointsB[0]), input.p)
                         : m3Add3(m3RotateVec3(input.q,
                                               m3MulSV3(0.5f, m3Add3(pointsB[0], pointsB[1]))),
                                  input.p);
            }
            nLocal = m3Normalize3(axis); // zero falls back to +y
            if (m3Dot3(nLocal, m3Sub3(cB, cA)) < 0.0f)
            {
                nLocal = m3Neg3(nLocal);
            }
            m3Vec3 mid = m3MulSV3(0.5f, m3Add3(cA, cB));
            pALocal = mid;
            pBLocal = mid;
            sep = -(rA + rB);
        }
        fresh->normal = m3RotateVec3(xfA->q, nLocal);
        m3Vec3 wA = m3RotateVec3(xfA->q, pALocal);
        m3Vec3 wB = m3RotateVec3(xfA->q, pBLocal);
        fresh->points[0].anchorA =
            m3AnchorFromCom(world, bodyA, xfA->p.x + (double)wA.x, xfA->p.y + (double)wA.y,
                            xfA->p.z + (double)wA.z);
        fresh->points[0].anchorB =
            m3AnchorFromCom(world, bodyB, xfA->p.x + (double)wB.x, xfA->p.y + (double)wB.y,
                            xfA->p.z + (double)wB.z);
        fresh->points[0].separation = sep;
        fresh->points[0].id = 0;
        fresh->pointCount = 1;
    }
}

static void CollidePlaneSpherePair(m3World* world, m3Manifold* fresh, int32_t shapeA,
                                   int32_t shapeB)
{
    uint8_t typeA = world->shapes.shapeType[shapeA];
    // Canonical orientation: the plane plays A. If the sphere
    // has the lower index the manifold flips on the way out.
    int32_t planeShape = typeA == (uint8_t)m3_planeShape ? shapeA : shapeB;
    int32_t sphereShape = planeShape == shapeA ? shapeB : shapeA;
    m3Vec3 n = world->shapes.shapeGeom[planeShape].v;
    m3real offset = world->shapes.shapeGeom[planeShape].s;
    double cx;
    double cy;
    double cz;
    m3SphereWorldCenter(world, sphereShape, &cx, &cy, &cz);
    double distD = (double)n.x * cx + (double)n.y * cy + (double)n.z * cz - (double)offset;
    *fresh = m3CollidePlaneSphere(n, (m3real)distD, world->shapes.shapeGeom[sphereShape].s);
    if (fresh->pointCount > 0)
    {
        // anchorA: from the plane BODY's origin to the contact
        // point (the sphere's deepest point projected).
        int32_t planeBody = world->shapes.shapeBody[planeShape];
        int32_t ballBody = world->shapes.shapeBody[sphereShape];
        double px = cx - (double)(fresh->normal.x * (m3real)distD);
        double py = cy - (double)(fresh->normal.y * (m3real)distD);
        double pz = cz - (double)(fresh->normal.z * (m3real)distD);
        fresh->points[0].anchorA = m3AnchorFromCom(world, planeBody, px, py, pz);
        fresh->points[0].anchorB =
            m3Add3(m3AnchorFromCom(world, ballBody, cx, cy, cz), fresh->points[0].anchorB);
        if (planeShape != shapeA)
        {
            // Flip to keep the manifold in key order (A = the
            // lower shape index, always).
            m3Vec3 tmp = fresh->points[0].anchorA;
            fresh->points[0].anchorA = fresh->points[0].anchorB;
            fresh->points[0].anchorB = tmp;
            fresh->normal = m3Neg3(fresh->normal);
        }
    }
}

static void CollideSphereSpherePair(m3World* world, m3Manifold* fresh, int32_t shapeA,
                                    int32_t shapeB)
{
    double ax;
    double ay;
    double az;
    double bx;
    double by;
    double bz;
    m3SphereWorldCenter(world, shapeA, &ax, &ay, &az);
    m3SphereWorldCenter(world, shapeB, &bx, &by, &bz);
    m3Vec3 d = {(m3real)(bx - ax), (m3real)(by - ay), (m3real)(bz - az)};
    *fresh =
        m3CollideSpheres(d, world->shapes.shapeGeom[shapeA].s, world->shapes.shapeGeom[shapeB].s);
    if (fresh->pointCount > 0)
    {
        // Kernel anchors are from the sphere CENTERS; re-base
        // them to each body's COM.
        fresh->points[0].anchorA =
            m3Add3(m3AnchorFromCom(world, world->shapes.shapeBody[shapeA], ax, ay, az),
                   fresh->points[0].anchorA);
        fresh->points[0].anchorB =
            m3Add3(m3AnchorFromCom(world, world->shapes.shapeBody[shapeB], bx, by, bz),
                   fresh->points[0].anchorB);
    }
}

// Which collider handles a pair, by the two shape types. The order of
// precedence: voxel, heightfield and mesh pairs route to their own
// pipelines; then plane-hull, hull-hull, plane-capsule and hull-capsule
// have dedicated kernels; any other pair with a hull or a capsule goes
// through GJK on the cores; the rest are plane-sphere and sphere-sphere.
static m3CollidePairFn* const s_colliders[M3_SHAPE_TYPE_COUNT][M3_SHAPE_TYPE_COUNT] = {
    [m3_sphereShape] = {CollideSphereSpherePair, CollidePlaneSpherePair, CollideConvexGjkPair,
                        CollideConvexGjkPair, CollideMeshPair, CollideVoxelPair,
                        CollideHeightFieldPair},
    [m3_planeShape] = {CollidePlaneSpherePair, CollidePlaneSpherePair, CollidePlaneHullPair,
                       CollidePlaneCapsulePair, CollideMeshPair, CollideVoxelPair,
                       CollideHeightFieldPair},
    [m3_hullShape] = {CollideConvexGjkPair, CollidePlaneHullPair, CollideHullHullPair,
                      CollideHullCapsulePair, CollideMeshPair, CollideVoxelPair,
                      CollideHeightFieldPair},
    [m3_capsuleShape] = {CollideConvexGjkPair, CollidePlaneCapsulePair, CollideHullCapsulePair,
                         CollideConvexGjkPair, CollideMeshPair, CollideVoxelPair,
                         CollideHeightFieldPair},
    [m3_meshShape] = {CollideMeshPair, CollideMeshPair, CollideMeshPair, CollideMeshPair,
                      CollideMeshPair, CollideVoxelPair, CollideHeightFieldPair},
    [m3_voxelShape] = {CollideVoxelPair, CollideVoxelPair, CollideVoxelPair, CollideVoxelPair,
                       CollideVoxelPair, CollideVoxelPair, CollideVoxelPair},
    [m3_heightFieldShape] = {CollideHeightFieldPair, CollideHeightFieldPair, CollideHeightFieldPair,
                             CollideHeightFieldPair, CollideHeightFieldPair, CollideVoxelPair,
                             CollideHeightFieldPair},
};

void m3UpdateContactsRange(m3World* world, int32_t start, int32_t end, const uint64_t* oldKeys,
                           const m3Manifold* oldManifolds, int32_t oldCount)
{
    // Rebuild manifolds in pair order, carrying impulses forward by
    // feature id from the caller's stash. The old keys are sorted
    // (canonical order), so the lookup is a binary search. Each pair
    // writes ONLY manifolds[i]: the range is safe under any split.
    for (int32_t i = start; i < end; ++i)
    {
        uint64_t key = world->contacts.pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);

        m3Manifold fresh;
        uint8_t typeA = world->shapes.shapeType[shapeA];
        uint8_t typeB = world->shapes.shapeType[shapeB];
        s_colliders[typeA][typeB](world, &fresh, shapeA, shapeB);

        // Warm-start carry: find the old manifold for this key and
        // match points by feature id.
        if (fresh.pointCount > 0 && oldCount > 0)
        {
            int32_t lo = 0;
            int32_t hi = oldCount - 1;
            while (lo <= hi)
            {
                int32_t mid = (lo + hi) / 2;
                if (oldKeys[mid] == key)
                {
                    const m3Manifold* previous = &oldManifolds[mid];
                    // The central friction payload carries with the
                    // PAIR, not with the points.
                    fresh.frictionImpulse = previous->frictionImpulse;
                    fresh.twistImpulse = previous->twistImpulse;
                    fresh.rollingImpulse = previous->rollingImpulse;
                    for (int32_t k = 0; k < fresh.pointCount; ++k)
                    {
                        for (int32_t o = 0; o < previous->pointCount; ++o)
                        {
                            if (previous->points[o].id == fresh.points[k].id)
                            {
                                fresh.points[k].normalImpulse = previous->points[o].normalImpulse;
                                fresh.points[k].flags |= 1; // persisted
                                break;
                            }
                        }
                    }
                    break;
                }
                if (oldKeys[mid] < key)
                {
                    lo = mid + 1;
                }
                else
                {
                    hi = mid - 1;
                }
            }
        }
        world->contacts.manifolds[i] = fresh;
    }
}

// The task-function shim: the host calls back into the range worker.
typedef struct m3ContactTaskContext
{
    m3World* world;
    const uint64_t* oldKeys;
    const m3Manifold* oldManifolds;
    int32_t oldCount;
} m3ContactTaskContext;

static void ContactTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
    m3ContactTaskContext* ctx = (m3ContactTaskContext*)taskContext;
    m3UpdateContactsRange(ctx->world, startIndex, endIndex, ctx->oldKeys, ctx->oldManifolds,
                          ctx->oldCount);
}

m3Result m3UpdateContacts(m3World* world, const uint64_t* oldKeys, const m3Manifold* oldManifolds,
                          int32_t oldCount)
{
    if (world->enqueueTask != NULL && world->contacts.pairCount > 1)
    {
        m3ContactTaskContext ctx = {world, oldKeys, oldManifolds, oldCount};
        void* task = world->enqueueTask(ContactTask, world->contacts.pairCount, 16, &ctx,
                                        world->userTaskContext);
        world->finishTask(task, world->userTaskContext);
    }
    else
    {
        m3UpdateContactsRange(world, 0, world->contacts.pairCount, oldKeys, oldManifolds, oldCount);
    }
    return m3_success;
}
