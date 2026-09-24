// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug draw: the world describes itself as segments, points and
// triangles through host callbacks. A draw pass only reads simulation
// state; a test hashes the world across a draw to check that.

#ifndef MAUL3D_DRAW_H
#define MAUL3D_DRAW_H

#include "world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// Colors are 0xRRGGBB. The palette is fixed so twin worlds draw
    /// identical streams: awake dynamic 0x4FA3FF, sleeping 0x777788,
    /// static 0x66BB66, kinematic 0xC9A227, sensor 0xAA66CC, contact
    /// 0xFF5544, joint 0xFFFFFF, AABB 0x333344.
    typedef struct m3DebugDraw
    {
        void (*drawSegment)(m3Pos3 p1, m3Pos3 p2, uint32_t color, void* context);
        void (*drawPoint)(m3Pos3 p, m3real size, uint32_t color, void* context);
        void (*drawTriangle)(m3Pos3 a, m3Pos3 b, m3Pos3 c, uint32_t color, void* context);
        void* context;
        bool drawShapes;      // wireframes, through drawSegment
        bool drawSolidShapes; // filled triangles, through drawTriangle
        bool drawContacts;    // manifold points plus normals scaled by impulse
        bool drawJoints;
        bool drawAabbs;     // the broadphase FAT bounds (what the tree sees)
        bool drawSleepTint; // sleeping dynamics take the sleep gray
        bool drawIslands;   // each body's center, colored by its island
        bool drawMassAxes;  // the principal inertia axes at each center of mass
        bool drawTreeBoxes; // the broadphase tree's inner nodes
    } m3DebugDraw;

    /// Draws what the flags ask for through the callbacks given; a
    /// missing callback skips what it would draw. Solid shapes come
    /// first, then wireframes, bounds, contacts, joints and the rest.
    /// Reads simulation state only. Thread class: reader.
    M3_API void m3World_Draw(m3WorldId worldId, const m3DebugDraw* draw);

#ifdef __cplusplus
}
#endif

#endif
