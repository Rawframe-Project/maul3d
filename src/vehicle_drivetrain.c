// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The vehicle drivetrain: the def, attaching it, gear selection and the
// engine readback. Public functions validate and journal; replay drives
// the internal setters.

#include "fpenv.h"
#include "journal.h"
#include "misuse.h"
#include "vehicle.h"
#include "world_internal.h"

#include "maul3d/vehicle.h"

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

// Validation lives here because replay hands the setter raw journal
// bytes: every field earns its range or the op fails loud. Restore asks
// it of every stored drivetrain.
bool m3DrivetrainDefValid(const m3DrivetrainDef* def)
{
    if (def->curveCount < 2 || def->curveCount > M3_DRIVETRAIN_MAX_CURVE || def->gearCount < 1 ||
        def->gearCount > M3_DRIVETRAIN_MAX_GEARS || def->clutchSteps < 0 || def->clutchSteps > 600)
    {
        return false;
    }
    for (int32_t c = 0; c < def->curveCount; ++c)
    {
        if (!m3FiniteF(def->curveRpm[c]) || def->curveRpm[c] < 0.0f ||
            !(def->curveTorque[c] >= 0.0f && def->curveTorque[c] <= M3_FORCE_LIMIT))
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
        if (!(def->gearRatio[g] > 0.0f && def->gearRatio[g] <= M3_RATIO_LIMIT))
        {
            return false;
        }
    }
    // Torques within M3_FORCE_LIMIT and every ratio within M3_RATIO_LIMIT,
    // so a gear times the final drive stays within its square. The
    // coupling is a gain in N per m/s: at the speed limit it stays within
    // the force limit.
    if (!(def->reverseRatio >= 0.0f && def->reverseRatio <= M3_RATIO_LIMIT) ||
        !(def->finalDrive > 0.0f && def->finalDrive <= M3_RATIO_LIMIT) ||
        !m3FiniteF(def->shiftUpRpm) || !m3FiniteF(def->shiftDownRpm) || def->shiftDownRpm < 0.0f ||
        def->shiftUpRpm <= def->shiftDownRpm || def->diffMode < 0 || def->diffMode > 2 ||
        !(def->diffCouple >= 0.0f && def->diffCouple <= M3_FORCE_LIMIT / M3_SPEED_LIMIT))
    {
        return false;
    }
    return true;
}

bool m3VehicleDrivetrainInternal(m3World* world, int32_t slot, const m3DrivetrainDef* def)
{
    if (!m3DrivetrainDefValid(def))
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

static M3_NOINLINE void VehicleSetDrivetrainWorker(m3World* world, m3VehicleId vehicle,
                                                   const m3DrivetrainDef* def)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot = m3VehicleSlot(world, vehicle);
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
        record.id = vehicle;
        m3JournalDrivetrainDef(&record.def, def);
        m3JournalRecord(world, m3_opVehicleDrivetrain, &record, (int32_t)sizeof(record));
    }
}

void m3Vehicle_SetDrivetrain(m3World* world, m3VehicleId vehicle, const m3DrivetrainDef* def)
{
    uint64_t env = m3FloatEnvEnter();
    VehicleSetDrivetrainWorker(world, vehicle, def);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void VehicleSelectGearWorker(m3World* world, m3VehicleId vehicle, int32_t gear)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t slot = m3VehicleSlot(world, vehicle);
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
        record.id = vehicle;
        record.gear = gear;
        m3JournalRecord(world, m3_opVehicleGear, &record, (int32_t)sizeof(record));
    }
}

void m3Vehicle_SelectGear(m3World* world, m3VehicleId vehicle, int32_t gear)
{
    uint64_t env = m3FloatEnvEnter();
    VehicleSelectGearWorker(world, vehicle, gear);
    m3FloatEnvLeave(env);
}

int32_t m3Vehicle_GetGear(const m3World* world, m3VehicleId vehicle)
{
    int32_t slot = m3VehicleSlot(world, vehicle);
    if (slot < 0)
    {
        m3Refuse(world, m3MissingId(vehicle.index1));
        return 0;
    }
    if (world->vehicles.vehDtActive[slot] == 0)
    {
        return 0; // no drivetrain: neutral and idle
    }
    return (int32_t)world->vehicles.vehDtGear[slot];
}

static M3_NOINLINE m3real VehicleGetEngineRpmWorker(const m3World* world, m3VehicleId vehicle)
{
    int32_t slot = m3VehicleSlot(world, vehicle);
    if (slot < 0)
    {
        m3Refuse(world, m3MissingId(vehicle.index1));
        return 0.0f;
    }
    if (world->vehicles.vehDtActive[slot] == 0)
    {
        return 0.0f; // no drivetrain: neutral and idle
    }
    return world->vehicles.vehDtRpm[slot];
}

m3real m3Vehicle_GetEngineRpm(const m3World* world, m3VehicleId vehicle)
{
    uint64_t env = m3FloatEnvEnter();
    m3real result = VehicleGetEngineRpmWorker(world, vehicle);
    m3FloatEnvLeave(env);
    return result;
}
