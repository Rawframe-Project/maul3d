// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The vehicle drivetrain: the def, attaching it, gear selection and the
// engine readback. Public functions validate and journal; replay drives
// the internal setters.

#include "maul3d/vehicle.h"

#include "journal.h"
#include "vehicle.h"
#include "world.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

#define M3_DRIVETRAIN_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3DrivetrainDef) << 8) ^ 12))

m3DrivetrainDef m3DefaultDrivetrainDef(void)
{
    // A plain small car: torquey low end (the launch), a peak in the
    // middle, falling off toward the limiter. Ratios descend like
    // every gearbox ever built.
    m3DrivetrainDef def;
    memset(&def, 0, sizeof(def));
    def.curveCount = 4;
    def.curveRpm[0] = 1000.0f;
    def.curveRpm[1] = 3000.0f;
    def.curveRpm[2] = 5000.0f;
    def.curveRpm[3] = 6500.0f;
    def.curveTorque[0] = 120.0f;
    def.curveTorque[1] = 200.0f;
    def.curveTorque[2] = 180.0f;
    def.curveTorque[3] = 90.0f;
    def.gearCount = 5;
    def.gearRatio[0] = 3.5f;
    def.gearRatio[1] = 2.2f;
    def.gearRatio[2] = 1.5f;
    def.gearRatio[3] = 1.1f;
    def.gearRatio[4] = 0.9f;
    def.reverseRatio = 3.2f;
    def.finalDrive = 3.7f;
    def.shiftUpRpm = 5500.0f;
    def.shiftDownRpm = 2000.0f;
    def.clutchSteps = 12;
    def.autoShift = true;
    def.internalValue = M3_DRIVETRAIN_COOKIE;
    return def;
}

// Validation lives here because replay hands this function raw
// journal bytes: every field earns its range or the op fails loud.
bool m3VehicleDrivetrainInternal(m3World* world, int32_t slot, const m3DrivetrainDef* def)
{
    if (def->curveCount < 2 || def->curveCount > M3_DRIVETRAIN_MAX_CURVE || def->gearCount < 1 ||
        def->gearCount > M3_DRIVETRAIN_MAX_GEARS || def->clutchSteps < 0 || def->clutchSteps > 600)
    {
        return false;
    }
    for (int32_t c = 0; c < def->curveCount; ++c)
    {
        if (!m3FiniteF(def->curveRpm[c]) || def->curveRpm[c] < 0.0f ||
            !m3FiniteF(def->curveTorque[c]) || def->curveTorque[c] < 0.0f)
        {
            return false;
        }
        if (c > 0 && def->curveRpm[c] <= def->curveRpm[c - 1])
        {
            return false; // strictly ascending or the interp divides by zero
        }
    }
    for (int32_t g = 0; g < def->gearCount; ++g)
    {
        if (!m3FiniteF(def->gearRatio[g]) || def->gearRatio[g] <= 0.0f)
        {
            return false;
        }
    }
    if (!m3FiniteF(def->reverseRatio) || def->reverseRatio < 0.0f || !m3FiniteF(def->finalDrive) ||
        def->finalDrive <= 0.0f || !m3FiniteF(def->shiftUpRpm) || !m3FiniteF(def->shiftDownRpm) ||
        def->shiftDownRpm < 0.0f || def->shiftUpRpm <= def->shiftDownRpm || def->diffMode < 0 ||
        def->diffMode > 2 || !m3FiniteF(def->diffCouple) || def->diffCouple < 0.0f)
    {
        return false;
    }

    world->vehicles.vehDtActive[slot] = 1;
    world->vehicles.vehDtCurveCount[slot] = def->curveCount;
    for (int32_t c = 0; c < M3_DRIVETRAIN_MAX_CURVE; ++c)
    {
        int32_t inRange = c < def->curveCount;
        world->vehicles.vehDtCurveRpm[slot * M3_DRIVETRAIN_MAX_CURVE + c] =
            inRange ? def->curveRpm[c] : 0.0f;
        world->vehicles.vehDtCurveTorque[slot * M3_DRIVETRAIN_MAX_CURVE + c] =
            inRange ? def->curveTorque[c] : 0.0f;
    }
    world->vehicles.vehDtGearCount[slot] = def->gearCount;
    for (int32_t g = 0; g < M3_DRIVETRAIN_MAX_GEARS; ++g)
    {
        world->vehicles.vehDtGearRatio[slot * M3_DRIVETRAIN_MAX_GEARS + g] =
            g < def->gearCount ? def->gearRatio[g] : 0.0f;
    }
    world->vehicles.vehDtReverse[slot] = def->reverseRatio;
    world->vehicles.vehDtFinal[slot] = def->finalDrive;
    world->vehicles.vehDtDiffMode[slot] = def->diffMode;
    world->vehicles.vehDtDiffCouple[slot] = def->diffCouple;
    world->vehicles.vehDtShiftUp[slot] = def->shiftUpRpm;
    world->vehicles.vehDtShiftDown[slot] = def->shiftDownRpm;
    world->vehicles.vehDtClutchSteps[slot] = def->clutchSteps;
    world->vehicles.vehDtAutoShift[slot] = def->autoShift ? 1 : 0;
    world->vehicles.vehDtGear[slot] = 1;
    world->vehicles.vehDtClutch[slot] = 0;
    world->vehicles.vehDtRpm[slot] = 0.0f;
    int32_t chassis = world->vehicles.vehChassis[slot];
    if (chassis >= 0 && world->bodies.bodyPool.alive[chassis] != 0)
    {
        world->bodies.awake[chassis] = 1;
        world->bodies.sleepTimes[chassis] = 0.0f;
    }
    return true;
}

bool m3VehicleGearInternal(m3World* world, int32_t slot, int32_t gear)
{
    if (world->vehicles.vehDtActive[slot] == 0 || gear < -1 ||
        gear > world->vehicles.vehDtGearCount[slot] ||
        (gear == -1 && world->vehicles.vehDtReverse[slot] == 0.0f))
    {
        return false;
    }
    if ((int8_t)gear != world->vehicles.vehDtGear[slot])
    {
        world->vehicles.vehDtGear[slot] = (int8_t)gear;
        world->vehicles.vehDtClutch[slot] = world->vehicles.vehDtClutchSteps[slot];
    }
    int32_t chassis = world->vehicles.vehChassis[slot];
    if (chassis >= 0 && world->bodies.bodyPool.alive[chassis] != 0)
    {
        world->bodies.awake[chassis] = 1;
        world->bodies.sleepTimes[chassis] = 0.0f;
    }
    return true;
}

void m3Vehicle_SetDrivetrain(m3VehicleId vehicleId, const m3DrivetrainDef* def)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || def == NULL || def->internalValue != M3_DRIVETRAIN_COOKIE)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // stale id or hostile def: a documented no-op
    }
    if (!m3VehicleDrivetrainInternal(world, slot, def))
    {
        m3Refuse(world, m3_errorInvalid);
        return; // invalid fields never journal
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVehicleDrivetrain record;
        memset(&record, 0, sizeof(record));
        record.id = vehicleId;
        record.def = *def;
        m3JournalRecord(world, m3_opVehicleDrivetrain, &record, (int32_t)sizeof(record));
    }
}

void m3Vehicle_SelectGear(m3VehicleId vehicleId, int32_t gear)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (!m3VehicleGearInternal(world, slot, gear))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVehicleGear record;
        memset(&record, 0, sizeof(record));
        record.id = vehicleId;
        record.gear = gear;
        m3JournalRecord(world, m3_opVehicleGear, &record, (int32_t)sizeof(record));
    }
}

int32_t m3Vehicle_GetGear(m3VehicleId vehicleId)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || world->vehicles.vehDtActive[slot] == 0)
    {
        return 0;
    }
    return (int32_t)world->vehicles.vehDtGear[slot];
}

m3real m3Vehicle_GetEngineRpm(m3VehicleId vehicleId)
{
    m3World* world = m3WorldFromTag(vehicleId.world);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || world->vehicles.vehDtActive[slot] == 0)
    {
        return 0.0f;
    }
    return world->vehicles.vehDtRpm[slot];
}
