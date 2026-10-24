// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape lookups shared by every part that reads shapes: the slot of an
// id, where a shape sits in the world, and the collision filter.

#ifndef MAUL3D_SRC_SHAPE_STATE_H
#define MAUL3D_SRC_SHAPE_STATE_H

#include "world_internal.h"

int32_t m3ShapeSlot(const m3World* world, m3ShapeId shapeId);

// The compound gate: THE one way to read where a shape sits
// in the world. Bodies still move; shapes may ride at an offset.
// Identity short-circuits through the flag so default scenes pay a
// branch and nothing else.
static inline m3Transform m3ShapeWorldTransform(const m3World* world, int32_t shape)
{
    int32_t body = world->shapes.shapeBody[shape];
    m3Transform xf = world->bodies.transforms[body];
    if (world->shapes.shapeHasOffset[shape] != 0)
    {
        m3Vec3 r = m3RotateVec3(xf.q, world->shapes.shapeLocalPos[shape]);
        xf.p.x += (double)r.x;
        xf.p.y += (double)r.y;
        xf.p.z += (double)r.z;
        xf.q = m3MulQuat(xf.q, world->shapes.shapeLocalRot[shape]);
    }
    return xf;
}

// The filter rule, one function for pairs and queries alike.
static inline int m3FilterPass(uint64_t catA, uint64_t maskA, uint64_t catB, uint64_t maskB)
{
    return (catA & maskB) != 0 && (catB & maskA) != 0;
}

#endif
