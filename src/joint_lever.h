// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The lever ratio every joint that solves blocks keeps (src/joint_lever.c).

#ifndef MAUL3D_SRC_JOINT_LEVER_H
#define MAUL3D_SRC_JOINT_LEVER_H

#include "world_internal.h"

// The largest lever ratio a joint that solves a point block or a held row
// pair keeps: the mass of its bodies times the squared arm over the moment
// the arm turns them. Past about 5e11 the joint's blocks, solved one after
// another, undo each other and the joint loses its bodies; the limit keeps
// about fifty times below the first such loss measured.
#define M3_JOINT_LEVER_LIMIT 1.0e10

// A body's mass as a lever check reads it: what the step solves with, so
// zero inverse mass and inertia for a body that does not move.
typedef struct m3LeverBody
{
    float invMass;
    m3Mat3 invInertia; // body frame
    m3Vec3 center;     // body frame
} m3LeverBody;

// Whether joints of this type solve a point block or a held row pair.
bool m3JointHasBlocks(uint8_t type);

// The largest eigenvalue of [arm]^T invInertia [arm]: the turn a unit
// impulse at arm gives the body, both in the body's frame.
double m3LeverTurn(const m3Mat3* invInertia, m3Vec3 arm);

// Body b's mass as it stands; none for b < 0.
m3LeverBody m3LeverBodyOf(const m3World* world, int32_t b);

// Whether a joint of this type with anchors localA and localB on bodies of
// these masses keeps the ratio: their turns over their inverse masses, an
// upper bound at every relative orientation of the bodies.
bool m3LeverFits(uint8_t type, const m3LeverBody* a, m3Vec3 localA, const m3LeverBody* b,
                 m3Vec3 localB);

// Whether every joint on body keeps the ratio with body taking the mass
// next.
bool m3BodyJointsFit(const m3World* world, int32_t body, const m3LeverBody* next);

#endif // MAUL3D_SRC_JOINT_LEVER_H
