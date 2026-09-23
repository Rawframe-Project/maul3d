// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Raycast vehicles: internal declarations.

#ifndef MAUL3D_SRC_VEHICLE_H
#define MAUL3D_SRC_VEHICLE_H

#include "world_internal.h"

int32_t m3VehicleSlot(const m3World* world, m3VehicleId vehicleId);

int32_t m3CreateVehicleInternal(m3World* world, const m3VehicleDef* def);

void m3DestroyVehicleInternal(m3World* world, int32_t slot);

void m3VehicleApplySuspension(m3World* world, float dt);

void m3VehicleCommandsInternal(m3World* world, int32_t slot, m3real throttle, m3real steer,
                               m3real brake);

void m3VehicleTankCommandsInternal(m3World* world, int32_t slot, m3real left, m3real right,
                                   m3real brake);

bool m3VehicleDrivetrainInternal(m3World* world, int32_t slot, const m3DrivetrainDef* def);

bool m3VehicleGearInternal(m3World* world, int32_t slot, int32_t gear);

#endif // MAUL3D_SRC_VEHICLE_H
