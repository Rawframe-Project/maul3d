// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restore's mass checks: the mass floors, the mass limit and every
// joint's lever ratio, asked of the bytes a restore checks before any of
// them lands, through the computation the world itself runs.

#include "snapshot_mass.h"

#include "hull.h"
#include "joint_lever.h"
#include "shape.h"
#include "world_state.h"

#include <stddef.h>
#include <string.h>

// Where each hull slot's content starts in the tail, or -1 for an empty
// slot; TailFits has checked every count and size.
static void HullContentStarts(const m3World* world, const uint8_t* raw, int32_t cursor,
                              int32_t* starts)
{
    for (int32_t h = 0; h < world->shapes.shapeCapacity; ++h)
    {
        int32_t counts[4];
        memcpy(counts, raw + cursor, sizeof(counts));
        cursor += 16;
        starts[h] = counts[0] > 0 ? cursor : -1;
        if (counts[0] > 0)
        {
            cursor += 4 + 2 * (int32_t)sizeof(m3Vec3) + (int32_t)sizeof(m3Mat3) +
                      counts[0] * (int32_t)sizeof(m3Vec3) +
                      counts[1] * ((int32_t)sizeof(m3Vec3) + 4 + 1 + 2) + counts[2] +
                      counts[3] * (int32_t)sizeof(m3HullHalfEdge);
        }
    }
}

// The incoming shape arrays, and the hull records' starts, that the mass
// computation reads; hull holds the unit mass properties of the shape
// read last.
typedef struct IncomingShapes
{
    const uint8_t* next;
    const uint8_t* types;
    const uint8_t* geoms;
    const uint8_t* densities;
    const uint8_t* hullIndices;
    const uint8_t* hasOffset;
    const uint8_t* localPos;
    const uint8_t* localRot;
    const uint8_t* raw;
    const int32_t* starts;
    m3HullData* hull;
} IncomingShapes;

static void ReadIncomingShape(const void* context, int32_t s, m3MassShape* out)
{
    const IncomingShapes* in = (const IncomingShapes*)context;
    int32_t h;
    out->type = in->types[s];
    out->hasOffset = in->hasOffset[s];
    memcpy(&out->geom, in->geoms + (size_t)s * sizeof(m3ShapeGeom), sizeof(out->geom));
    memcpy(&out->density, in->densities + 4 * (size_t)s, sizeof(out->density));
    memcpy(&out->localPos, in->localPos + (size_t)s * sizeof(m3Vec3), sizeof(out->localPos));
    memcpy(&out->localRot, in->localRot + (size_t)s * sizeof(m3Quat), sizeof(out->localRot));
    memcpy(&out->next, in->next + 4 * (size_t)s, sizeof(out->next));
    memcpy(&h, in->hullIndices + 4 * (size_t)s, sizeof(h));
    memset(&in->hull->unitMass, 0, sizeof(in->hull->unitMass));
    memset(&in->hull->unitCom, 0, sizeof(in->hull->unitCom));
    memset(&in->hull->unitInertiaCom, 0, sizeof(in->hull->unitInertiaCom));
    if (h >= 0 && in->starts[h] >= 0)
    {
        const uint8_t* at = in->raw + in->starts[h];
        memcpy(&in->hull->unitMass, at, sizeof(in->hull->unitMass));
        memcpy(&in->hull->unitCom, at + 4, sizeof(in->hull->unitCom));
        memcpy(&in->hull->unitInertiaCom, at + 4 + sizeof(m3Vec3),
               sizeof(in->hull->unitInertiaCom));
    }
    out->hull = h >= 0 ? in->hull : NULL;
}

// Whether every incoming joint keeps its lever ratio at the masses the
// incoming bodies take from their shapes, as restore will compute them.
static bool IncomingLeversFit(m3World* world, const uint8_t* prefix, const IncomingShapes* shapes,
                              bool* fits)
{
#define M3_IN(field) (prefix + m3StateFieldAt(world, offsetof(m3World, field)))
    const uint8_t* bodyAlive = M3_IN(bodies.bodyPool.alive);
    const uint8_t* bodyTypes = M3_IN(bodies.types);
    const uint8_t* heads = M3_IN(bodies.bodyShapeHead);
    const uint8_t* jointAlive = M3_IN(joints.jointPool.alive);
    const uint8_t* jointTypes = M3_IN(joints.jointType);
    const uint8_t* bodiesA = M3_IN(joints.jointBodyA);
    const uint8_t* bodiesB = M3_IN(joints.jointBodyB);
    const uint8_t* localsA = M3_IN(joints.jointLocalA);
    const uint8_t* localsB = M3_IN(joints.jointLocalB);
#undef M3_IN
    int32_t bodyCap = world->bodies.bodyPool.capacity;
    m3LeverBody* levers =
        (m3LeverBody*)m3AllocArray(&world->heap, bodyCap > 0 ? bodyCap : 1, sizeof(m3LeverBody));
    if (levers == NULL)
    {
        return false;
    }
    for (int32_t b = 0; b < bodyCap; ++b)
    {
        m3LeverBody none = {0.0f, m3MakeZeroMat3(), {0.0f, 0.0f, 0.0f}};
        levers[b] = none;
        if (bodyAlive[b] != 0 && bodyTypes[b] == (uint8_t)m3_dynamicBody)
        {
            int32_t head;
            m3Mat3 inertia;
            bool massive;
            memcpy(&head, heads + 4 * (size_t)b, sizeof(head));
            levers[b] = m3DynamicMassOf(ReadIncomingShape, shapes, head, &inertia, &massive);
        }
    }
    *fits = true;
    for (int32_t j = 0; *fits && j < world->joints.jointPool.capacity; ++j)
    {
        if (jointAlive[j] == 0)
        {
            continue;
        }
        int32_t a;
        int32_t b;
        m3Vec3 localA;
        m3Vec3 localB;
        memcpy(&a, bodiesA + 4 * (size_t)j, sizeof(a));
        memcpy(&b, bodiesB + 4 * (size_t)j, sizeof(b));
        memcpy(&localA, localsA + (size_t)j * sizeof(m3Vec3), sizeof(localA));
        memcpy(&localB, localsB + (size_t)j * sizeof(m3Vec3), sizeof(localB));
        *fits = m3LeverFits(jointTypes[j], &levers[a], localA, &levers[b], localB);
    }
    m3Free(&world->heap, levers);
    return true;
}

// Whether every incoming dynamic body's shapes keep the mass floors and
// the body within M3_MASS_LIMIT, the bounds its creates and setters
// kept, through the check they ask, and every joint its lever ratio. The
// links check has made the shape lists, joint bodies and content indices
// safe to follow.
m3Result m3SnapshotMassesFit(m3World* world, const uint8_t* raw, const uint8_t* prefix,
                             int32_t hullCursor)
{
    int32_t* starts = (int32_t*)m3AllocArray(
        &world->heap, world->shapes.shapeCapacity > 0 ? world->shapes.shapeCapacity : 1,
        (int64_t)sizeof(int32_t));
    if (starts == NULL)
    {
        return m3_errorCapacity;
    }
    HullContentStarts(world, raw, hullCursor, starts);
#define M3_IN(field) (prefix + m3StateFieldAt(world, offsetof(m3World, field)))
    const uint8_t* alive = M3_IN(bodies.bodyPool.alive);
    const uint8_t* types = M3_IN(bodies.types);
    const uint8_t* heads = M3_IN(bodies.bodyShapeHead);
    const uint8_t* next = M3_IN(shapes.shapeNext);
    const uint8_t* shapeTypes = M3_IN(shapes.shapeType);
    const uint8_t* geoms = M3_IN(shapes.shapeGeom);
    const uint8_t* densities = M3_IN(shapes.shapeDensity);
    const uint8_t* hullIndices = M3_IN(shapes.shapeHullIndex);
#undef M3_IN
    m3HullData hull; // only the unit mass properties are read
    memset(&hull, 0, sizeof(hull));
    bool fits = true;
    for (int32_t b = 0; fits && b < world->bodies.bodyPool.capacity; ++b)
    {
        if (alive[b] == 0 || types[b] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        double total = 0.0;
        int32_t s;
        memcpy(&s, heads + 4 * (size_t)b, sizeof(s));
        while (fits && s != -1)
        {
            m3ShapeGeom geom;
            float density;
            int32_t h;
            memcpy(&geom, geoms + (size_t)s * sizeof(m3ShapeGeom), sizeof(geom));
            memcpy(&density, densities + 4 * (size_t)s, sizeof(density));
            memcpy(&h, hullIndices + 4 * (size_t)s, sizeof(h));
            // A hull shape names a live slot (the links check); an empty
            // one weighs nothing, below the floor.
            memset(&hull.unitMass, 0, sizeof(hull.unitMass));
            memset(&hull.unitInertiaCom, 0, sizeof(hull.unitInertiaCom));
            if (h >= 0 && starts[h] >= 0)
            {
                const uint8_t* at = raw + starts[h];
                memcpy(&hull.unitMass, at, sizeof(hull.unitMass));
                memcpy(&hull.unitCom, at + 4, sizeof(hull.unitCom));
                memcpy(&hull.unitInertiaCom, at + 4 + sizeof(m3Vec3), sizeof(hull.unitInertiaCom));
            }
            fits = m3ShapeMassFloors(shapeTypes[s], &geom, h >= 0 ? &hull : NULL, density, &total);
            memcpy(&s, next + 4 * (size_t)s, sizeof(s));
        }
        fits = fits && total <= (double)M3_MASS_LIMIT;
    }
    IncomingShapes shapes = {
        next,
        shapeTypes,
        geoms,
        densities,
        hullIndices,
        prefix + m3StateFieldAt(world, offsetof(m3World, shapes.shapeHasOffset)),
        prefix + m3StateFieldAt(world, offsetof(m3World, shapes.shapeLocalPos)),
        prefix + m3StateFieldAt(world, offsetof(m3World, shapes.shapeLocalRot)),
        raw,
        starts,
        &hull};
    bool allocated = !fits || IncomingLeversFit(world, prefix, &shapes, &fits);
    m3Free(&world->heap, starts);
    if (!allocated)
    {
        return m3_errorCapacity;
    }
    return fits ? m3_success : m3_errorInvalid;
}
