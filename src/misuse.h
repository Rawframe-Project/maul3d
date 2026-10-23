// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Refusals of a caller's input against a world: the reason for
// m3LastResult, and the world's misuse counter.

#ifndef MAUL3D_SRC_MISUSE_H
#define MAUL3D_SRC_MISUSE_H

#include "world_internal.h"

// Refuses a caller's input: records the reason for m3LastResult on this
// thread and, for an invalid argument against a live world, counts it in
// m3Counters.misuse. world may be NULL.
void m3Refuse(m3World* world, m3Result reason);
uint64_t m3MisuseCount(const m3World* world);

#endif
