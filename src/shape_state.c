// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape lookups.

#include "shape_state.h"

int32_t m3ShapeSlot(const m3World* world, m3ShapeId shapeId)
{
    int32_t index = shapeId.index1 - 1;
    if (world == NULL || !m3IdPoolValid(&world->shapes.shapePool, index, shapeId.generation))
    {
        return -1;
    }
    return index;
}
