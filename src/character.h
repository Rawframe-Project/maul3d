// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Character controllers: internal declarations.

#ifndef MAUL3D_SRC_CHARACTER_H
#define MAUL3D_SRC_CHARACTER_H

#include "world_internal.h"

// Character internals: create/destroy/move without journal
// (replay drives these; public wrappers validate and record).
int32_t m3CreateCharacterInternal(m3World* world, const m3CharacterDef* def);

void m3DestroyCharacterInternal(m3World* world, int32_t slot);

void m3CharacterMoveInternal(m3World* world, int32_t slot, m3Vec3 translation);

bool m3CharacterStanceInternal(m3World* world, int32_t slot, m3real halfHeight, m3real radius);

// Re-evaluate grounding in place (no displacement): the voxel edit
// path calls this so a carved floor drops its tenants the same
// step.
void m3CharacterRefreshGrounding(m3World* world, int32_t slot);

int32_t m3CharacterSlot(const m3World* world, m3CharacterId characterId);

void m3CharacterCarryRiders(m3World* world, const m3Pos3* com0, const m3Quat* rot0);

#endif // MAUL3D_SRC_CHARACTER_H
