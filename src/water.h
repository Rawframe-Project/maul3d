// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Water volumes: the buoyancy field the step applies to the bodies in
// them.

#ifndef MAUL3D_SRC_WATER_H
#define MAUL3D_SRC_WATER_H

#include "world_internal.h"

// The per-mover water field of one step; active is zero when no volume
// is alive (or the scratch stalled), and then the arrays are unused.
typedef struct m3Buoyancy
{
    m3Vec3* force;
    m3Vec3* torque;
    m3Vec3* flow;
    float* lin;
    float* ang;
    int32_t active;
} m3Buoyancy;

// The field for the step's movers, from the step-start pose.
void m3PrepareBuoyancy(m3World* world, const int32_t* movers, int32_t moverCount, m3Buoyancy* b);

#endif // MAUL3D_SRC_WATER_H
