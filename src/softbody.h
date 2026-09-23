// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Soft bodies: internal declarations.

#ifndef MAUL3D_SRC_SOFTBODY_H
#define MAUL3D_SRC_SOFTBODY_H

#include "world_internal.h"

int32_t m3SoftBodySlot(const m3World* world, m3SoftBodyId softId);

int32_t m3CreateSoftBodyInternal(m3World* world, const m3SoftBodyDef* def);

int32_t m3CreateSoftBodyTetInternal(m3World* world, const m3SoftBodyDef* def, const m3Vec3* points,
                                    int32_t pointCount, const uint16_t* tets, int32_t tetCount);

void m3DestroySoftBodyInternal(m3World* world, int32_t slot);

void m3SoftBodyPinInternal(m3World* world, int32_t slot, int32_t particle);

void m3SoftBodyPass(m3World* world, float dt, int32_t substeps);

// The substep's lattice-to-lattice work (softbody_pairs.c): particle
// contacts between different soft bodies, then the soft-to-soft pins.
void m3SoftSoftContacts(m3World* world);
void m3SoftSoftAnchors(m3World* world);

void m3SoftBodyAnchorInternal(m3World* world, int32_t slot, int32_t particle, int32_t body);

void m3SoftBodyAnchorSoftInternal(m3World* world, int32_t slotA, int32_t particleA, int32_t slotB,
                                  int32_t particleB);

#endif // MAUL3D_SRC_SOFTBODY_H
