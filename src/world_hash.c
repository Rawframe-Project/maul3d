// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world hash: a curated walk over the deterministic state in
// canonical slot order. It hashes what the simulation is, not how it is
// stored: dead slots contribute only their liveness, and optional state
// folds in only when it is present or off its default.

#include "body.h"
#include "joint_solver.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"
#include "world_state.h"

#include <stddef.h>

static uint64_t HashWorldHeader(const m3World* world, uint64_t h)
{
    h = m3Hash64(h, &world->stepCount, 8);
    h = m3Hash64(h, &world->gravity, (int32_t)sizeof(m3Vec3));
    if (world->contactHertz != M3_CONTACT_HERTZ_DEFAULT ||
        world->contactDampingRatio != M3_CONTACT_DAMPING_RATIO_DEFAULT ||
        world->contactPushMaxSpeed != M3_CONTACT_PUSH_MAX_SPEED_DEFAULT ||
        world->restitutionThreshold != M3_RESTITUTION_THRESHOLD_DEFAULT ||
        world->maximumLinearSpeed != M3_MAX_LINEAR_SPEED_DEFAULT || world->sleepEnabled == 0 ||
        world->continuousEnabled == 0 || world->hitEventThreshold != M3_HIT_EVENT_THRESHOLD_DEFAULT)
    {
        // Tuning knobs fold only off their defaults.
        h = m3Hash64(h, &world->contactHertz, 4);
        h = m3Hash64(h, &world->contactDampingRatio, 4);
        h = m3Hash64(h, &world->contactPushMaxSpeed, 4);
        h = m3Hash64(h, &world->restitutionThreshold, 4);
        h = m3Hash64(h, &world->maximumLinearSpeed, 4);
        h = m3Hash64(h, &world->sleepEnabled, 1);
        h = m3Hash64(h, &world->continuousEnabled, 1);
        h = m3Hash64(h, &world->hitEventThreshold, 4);
    }
    if (world->maximumAngularSpeed != M3_MAX_ANGULAR_SPEED_DEFAULT)
    {
        // The angular cap folds in its own block, so the knob block
        // above stays the same size for worlds that change only it.
        h = m3Hash64(h, &world->maximumAngularSpeed, 4);
    }
    if (world->windSpeed != 0.0f)
    {
        // Wind folds only when it blows (additive rule), phase
        // included: the gust wave is trajectory-shaping state.
        h = m3Hash64(h, &world->windDir, (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->windSpeed, 4);
        h = m3Hash64(h, &world->windGustHertz, 4);
        h = m3Hash64(h, &world->windGustScale, 4);
        h = m3Hash64(h, &world->windPhase, 4);
    }
    return h;
}

static uint64_t HashBodies(const m3World* world, uint64_t h)
{
    int32_t maxIndex = world->bodies.bodyPool.maxIndex;
    for (int32_t i = 0; i < maxIndex; ++i)
    {
        uint8_t alive = world->bodies.bodyPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->bodies.transforms[i], (int32_t)sizeof(m3Transform));
        h = m3Hash64(h, &world->bodies.linearVelocities[i], (int32_t)sizeof(m3Vec3));
        if (world->bodies.bodyForce[i].x != 0.0f || world->bodies.bodyForce[i].y != 0.0f ||
            world->bodies.bodyForce[i].z != 0.0f || world->bodies.bodyTorque[i].x != 0.0f ||
            world->bodies.bodyTorque[i].y != 0.0f || world->bodies.bodyTorque[i].z != 0.0f)
        {
            // Pending host forces fold only when present (a mid-step
            // snapshot must carry them).
            h = m3Hash64(h, &world->bodies.bodyForce[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->bodies.bodyTorque[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->bodies.bodyEnabled[i] == 0 || world->bodies.bodyLocks[i] != 0 ||
            world->bodies.bodySleepThreshold[i] != M3_SLEEP_VELOCITY_DEFAULT ||
            world->bodies.bodyCanSleep[i] == 0 || world->bodies.bodyHasTarget[i] != 0)
        {
            // Control state folds only off its defaults.
            h = m3Hash64(h, &world->bodies.bodyEnabled[i], 1);
            h = m3Hash64(h, &world->bodies.bodyLocks[i], 1);
            h = m3Hash64(h, &world->bodies.bodySleepThreshold[i], 4);
            h = m3Hash64(h, &world->bodies.bodyCanSleep[i], 1);
            h = m3Hash64(h, &world->bodies.bodyHasTarget[i], 1);
            h = m3Hash64(h, &world->bodies.bodyTarget[i], (int32_t)sizeof(m3Transform));
        }
        h = m3Hash64(h, &world->bodies.angularVelocities[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->bodies.invMass[i], (int32_t)sizeof(m3real));
        h = m3Hash64(h, &world->bodies.invInertiaLocal[i], (int32_t)sizeof(m3Mat3));
        h = m3Hash64(h, &world->bodies.localCenters[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->bodies.types[i], 1);
        h = m3Hash64(h, &world->bodies.bulletFlags[i], 1);
        h = m3Hash64(h, &world->bodies.awake[i], 1);
        h = m3Hash64(h, &world->bodies.sleepTimes[i], 4);
    }
    return h;
}

static uint64_t HashShapes(const m3World* world, uint64_t h)
{
    int32_t maxShape = world->shapes.shapePool.maxIndex;
    for (int32_t i = 0; i < maxShape; ++i)
    {
        uint8_t alive = world->shapes.shapePool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->shapes.shapeBody[i], 4);
        h = m3Hash64(h, &world->shapes.shapeType[i], 1);
        h = m3Hash64(h, &world->shapes.shapeGeom[i], (int32_t)sizeof(m3ShapeGeom));
        h = m3Hash64(h, &world->shapes.shapeDensity[i], 4);
        h = m3Hash64(h, &world->shapes.shapeFriction[i], 4);
        h = m3Hash64(h, &world->shapes.shapeRestitution[i], 4);
        if (world->shapes.shapeHitEvents[i] != 0 || world->shapes.shapePreSolve[i] != 0)
        {
            // Event flags are observable shape state: they fold
            // off-default (additive rule, seventh use).
            h = m3Hash64(h, &world->shapes.shapeHitEvents[i], 1);
            h = m3Hash64(h, &world->shapes.shapePreSolve[i], 1);
        }
        if (world->shapes.shapeSurfaceVel[i].x != 0.0f ||
            world->shapes.shapeSurfaceVel[i].y != 0.0f ||
            world->shapes.shapeSurfaceVel[i].z != 0.0f)
        {
            h = m3Hash64(h, &world->shapes.shapeSurfaceVel[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->shapes.shapeHasOffset[i] != 0)
        {
            // Compound offsets fold off-identity (additive rule,
            // tenth use).
            h = m3Hash64(h, &world->shapes.shapeLocalPos[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->shapes.shapeLocalRot[i], (int32_t)sizeof(m3Quat));
        }
        if (world->shapes.shapeRollingResistance[i] != 0.0f)
        {
            // Rolling resistance folds only where it is set.
            h = m3Hash64(h, &world->shapes.shapeRollingResistance[i], 4);
        }
        if (world->shapes.shapeCategory[i] != 1ull || world->shapes.shapeMask[i] != ~0ull ||
            world->shapes.shapeGroup[i] != 0)
        {
            // Same rule for filters: default-filtered shapes
            // keep every pre-existing hash still.
            h = m3Hash64(h, &world->shapes.shapeCategory[i], 8);
            h = m3Hash64(h, &world->shapes.shapeMask[i], 8);
            h = m3Hash64(h, &world->shapes.shapeGroup[i], 4);
        }
        h = m3Hash64(h, &world->shapes.shapeHullIndex[i], 4);
        h = m3Hash64(h, &world->shapes.shapeMeshIndex[i], 4);
        h = m3Hash64(h, &world->shapes.shapeSensor[i], 1);
        if (world->shapes.shapeType[i] == (uint8_t)m3_voxelShape)
        {
            // Folded only for the new type: pre-voxel scenes keep
            // their exact hash input set (the golden must not move
            // for worlds that never touch voxels).
            h = m3Hash64(h, &world->shapes.shapeVoxelIndex[i], 4);
        }
    }
    return h;
}

static uint64_t HashCharacters(const m3World* world, uint64_t h)
{
    // Character state: live slots only, the additive-state rule.
    int32_t maxChar = world->characters.charPool.maxIndex;
    for (int32_t i = 0; i < maxChar; ++i)
    {
        uint8_t alive = world->characters.charPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->characters.charBody[i], 4);
        h = m3Hash64(h, &world->characters.charRadius[i], 4);
        h = m3Hash64(h, &world->characters.charHalfHeight[i], 4);
        h = m3Hash64(h, &world->characters.charCosSlope[i], 4);
        h = m3Hash64(h, &world->characters.charSnap[i], 4);
        h = m3Hash64(h, &world->characters.charSkin[i], 4);
        h = m3Hash64(h, &world->characters.charStepHeight[i], 4);
        h = m3Hash64(h, &world->characters.charGrounded[i], 1);
        h = m3Hash64(h, &world->characters.charGroundNormal[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->characters.charMass[i], 4);
        h = m3Hash64(h, &world->characters.charPushMax[i], 4);
        h = m3Hash64(h, &world->characters.charGroundBody[i], 4);
        h = m3Hash64(h, &world->characters.charGroundGen[i], 2);
    }
    return h;
}

// One soft body's edges, body anchors and soft-to-soft anchors.
static uint64_t HashSoftLinks(const m3World* world, uint64_t h, int32_t i)
{
    int32_t ec = world->softBodies.softEdgeCount[i];
    for (int32_t e = 0; e < ec; ++e)
    {
        int32_t k = i * M3_SOFTBODY_MAX_EDGES + e;
        h = m3Hash64(h, &world->softBodies.softEdgeA[k], 2);
        h = m3Hash64(h, &world->softBodies.softEdgeB[k], 2);
        h = m3Hash64(h, &world->softBodies.softEdgeRest[k], 4);
    }
    int32_t ac = world->softBodies.softAnchorCount[i];
    for (int32_t a = 0; a < ac; ++a)
    {
        int32_t k = i * M3_SOFTBODY_MAX_ANCHORS + a;
        h = m3Hash64(h, &world->softBodies.softAnchorParticle[k], 4);
        h = m3Hash64(h, &world->softBodies.softAnchorBody[k], 4);
        h = m3Hash64(h, &world->softBodies.softAnchorGen[k], 2);
        h = m3Hash64(h, &world->softBodies.softAnchorLocal[k], (int32_t)sizeof(m3Vec3));
    }
    // Soft-to-soft anchors fold off-empty (additive rule).
    int32_t sc = world->softBodies.softSoftCount[i];
    for (int32_t a = 0; a < sc; ++a)
    {
        int32_t k = i * M3_SOFTBODY_MAX_ANCHORS + a;
        h = m3Hash64(h, &world->softBodies.softSoftParticleA[k], 4);
        h = m3Hash64(h, &world->softBodies.softSoftSlotB[k], 4);
        h = m3Hash64(h, &world->softBodies.softSoftGenB[k], 2);
        h = m3Hash64(h, &world->softBodies.softSoftParticleB[k], 4);
    }
    return h;
}

static uint64_t HashSoftBodies(const m3World* world, uint64_t h)
{
    for (int32_t i = 0; i < world->softBodies.softPool.maxIndex; ++i)
    {
        if (world->softBodies.softPool.alive[i] == 0)
        {
            continue; // live slots only
        }
        h = m3Hash64(h, &world->softBodies.softParticleCount[i], 4);
        h = m3Hash64(h, &world->softBodies.softCompliance[i], 4);
        if (world->softBodies.softBendCompliance[i] != 0.0f)
        {
            // Bend tethers fold off-default, their own block.
            h = m3Hash64(h, &world->softBodies.softBendStart[i], 4);
            h = m3Hash64(h, &world->softBodies.softBendCompliance[i], 4);
        }
        if (world->softBodies.softPressure[i] != 0.0f)
        {
            // Pressure folds off-default, its own block.
            h = m3Hash64(h, &world->softBodies.softDimX[i], 2);
            h = m3Hash64(h, &world->softBodies.softDimY[i], 2);
            h = m3Hash64(h, &world->softBodies.softDimZ[i], 2);
            h = m3Hash64(h, &world->softBodies.softRestVolume[i], 4);
            h = m3Hash64(h, &world->softBodies.softPressure[i], 4);
        }
        if (world->softBodies.softMaxDeviation[i] != 0.0f)
        {
            // The tether folds off-default, its own block;
            // bind positions join only then (dead weight otherwise).
            h = m3Hash64(h, &world->softBodies.softMaxDeviation[i], 4);
            h = m3Hash64(h, &world->softBodies.softBindPos[i * M3_SOFTBODY_MAX_PARTICLES],
                         world->softBodies.softParticleCount[i] * (int32_t)sizeof(m3Pos3));
        }
        if (world->softBodies.softTetCount[i] > 0)
        {
            // Tets fold off-default, their own block.
            h = m3Hash64(h, &world->softBodies.softTetCount[i], 4);
            int32_t tbase = i * M3_SOFTBODY_MAX_TETS;
            h = m3Hash64(h, &world->softBodies.softTetA[tbase],
                         2 * world->softBodies.softTetCount[i]);
            h = m3Hash64(h, &world->softBodies.softTetB[tbase],
                         2 * world->softBodies.softTetCount[i]);
            h = m3Hash64(h, &world->softBodies.softTetC[tbase],
                         2 * world->softBodies.softTetCount[i]);
            h = m3Hash64(h, &world->softBodies.softTetD[tbase],
                         2 * world->softBodies.softTetCount[i]);
            h = m3Hash64(h, &world->softBodies.softTetRestV6[tbase],
                         4 * world->softBodies.softTetCount[i]);
        }
        h = m3Hash64(h, &world->softBodies.softRadius[i], 4);
        h = m3Hash64(h, &world->softBodies.softGravityScale[i], 4);
        int32_t pc = world->softBodies.softParticleCount[i];
        for (int32_t p = 0; p < pc; ++p)
        {
            int32_t k = i * M3_SOFTBODY_MAX_PARTICLES + p;
            h = m3Hash64(h, &world->softBodies.softPos[k], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->softBodies.softPrev[k], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->softBodies.softInvMass[k], 4);
            if (world->softBodies.softKick[k].x != 0.0f ||
                world->softBodies.softKick[k].y != 0.0f || world->softBodies.softKick[k].z != 0.0f)
            {
                // Pending kicks fold only while they exist:
                // the window between a blast and its next step is
                // real rollback state, everything else is silence.
                h = m3Hash64(h, &world->softBodies.softKick[k], (int32_t)sizeof(m3Vec3));
            }
        }
        h = HashSoftLinks(world, h, i);
    }
    return h;
}

static uint64_t HashDrivetrain(const m3World* world, uint64_t h, int32_t i)
{
    h = m3Hash64(h, &world->vehicles.vehDtCurveCount[i], 4);
    for (int32_t c = 0; c < world->vehicles.vehDtCurveCount[i]; ++c)
    {
        h = m3Hash64(h, &world->vehicles.vehDtCurveRpm[i * M3_DRIVETRAIN_MAX_CURVE + c], 4);
        h = m3Hash64(h, &world->vehicles.vehDtCurveTorque[i * M3_DRIVETRAIN_MAX_CURVE + c], 4);
    }
    h = m3Hash64(h, &world->vehicles.vehDtGearCount[i], 4);
    for (int32_t g = 0; g < world->vehicles.vehDtGearCount[i]; ++g)
    {
        h = m3Hash64(h, &world->vehicles.vehDtGearRatio[i * M3_DRIVETRAIN_MAX_GEARS + g], 4);
    }
    h = m3Hash64(h, &world->vehicles.vehDtReverse[i], 4);
    h = m3Hash64(h, &world->vehicles.vehDtFinal[i], 4);
    if (world->vehicles.vehDtDiffMode[i] != 0)
    {
        // The diff folds only when engaged, wheel
        // speeds included: they steer forces only then.
        h = m3Hash64(h, &world->vehicles.vehDtDiffMode[i], 4);
        h = m3Hash64(h, &world->vehicles.vehDtDiffCouple[i], 4);
        for (int32_t wq = 0; wq < world->vehicles.vehWheelCount[i]; ++wq)
        {
            h = m3Hash64(h, &world->vehicles.vehWheelLon[i * M3_VEHICLE_MAX_WHEELS + wq], 4);
        }
    }
    h = m3Hash64(h, &world->vehicles.vehDtShiftUp[i], 4);
    h = m3Hash64(h, &world->vehicles.vehDtShiftDown[i], 4);
    h = m3Hash64(h, &world->vehicles.vehDtClutchSteps[i], 4);
    h = m3Hash64(h, &world->vehicles.vehDtAutoShift[i], 1);
    h = m3Hash64(h, &world->vehicles.vehDtGear[i], 1);
    h = m3Hash64(h, &world->vehicles.vehDtClutch[i], 4);
    h = m3Hash64(h, &world->vehicles.vehDtRpm[i], 4);
    return h;
}

static uint64_t HashVehicles(const m3World* world, uint64_t h)
{
    for (int32_t i = 0; i < world->vehicles.vehPool.maxIndex; ++i)
    {
        if (world->vehicles.vehPool.alive[i] == 0)
        {
            continue; // live slots only
        }
        h = m3Hash64(h, &world->vehicles.vehChassis[i], 4);
        h = m3Hash64(h, &world->vehicles.vehChassisGen[i], 2);
        h = m3Hash64(h, &world->vehicles.vehWheelCount[i], 4);
        h = m3Hash64(h, &world->vehicles.vehMaxSteer[i], 4);
        h = m3Hash64(h, &world->vehicles.vehDriveForce[i], 4);
        h = m3Hash64(h, &world->vehicles.vehBrakeForce[i], 4);
        h = m3Hash64(h, &world->vehicles.vehTireGrip[i], 4);
        h = m3Hash64(h, &world->vehicles.vehThrottle[i], 4);
        if (world->vehicles.vehTrackMode[i] != 0)
        {
            // Tank mode folds off-default, its own block.
            h = m3Hash64(h, &world->vehicles.vehTrackMode[i], 1);
            h = m3Hash64(h, &world->vehicles.vehTrackLeft[i], 4);
            h = m3Hash64(h, &world->vehicles.vehTrackRight[i], 4);
        }
        if (world->vehicles.vehLeanGain[i] != 0.0f)
        {
            // The lean gain folds off-default, its own block.
            h = m3Hash64(h, &world->vehicles.vehLeanGain[i], 4);
        }
        h = m3Hash64(h, &world->vehicles.vehSteer[i], 4);
        h = m3Hash64(h, &world->vehicles.vehBrake[i], 4);
        for (int32_t w = 0; w < world->vehicles.vehWheelCount[i]; ++w)
        {
            int32_t k = i * M3_VEHICLE_MAX_WHEELS + w;
            h = m3Hash64(h, &world->vehicles.vehWheelAnchor[k], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->vehicles.vehWheelDir[k], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->vehicles.vehWheelRest[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelTravel[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelHertz[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelZeta[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelRadius[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelFlags[k], 1);
            h = m3Hash64(h, &world->vehicles.vehWheelBrake[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelCompression[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelContact[k], 1);
            h = m3Hash64(h, &world->vehicles.vehWheelSpin[k], 4);
        }
        if (world->vehicles.vehDtActive[i] != 0)
        {
            h = HashDrivetrain(world, h, i); // only when attached
        }
    }
    return h;
}

static uint64_t HashVoxels(const m3World* world, uint64_t h)
{
    // Voxel chunk content is simulation state (destruction edits it
    // and rollback must cover it); live slots only, same law as
    // everything above.
    int32_t maxVoxel = world->voxels.voxelPool.maxIndex;
    for (int32_t i = 0; i < maxVoxel; ++i)
    {
        uint8_t alive = world->voxels.voxelPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->voxels.voxelData[i].cellSize, 4);
        h = m3Hash64(h, &world->voxels.voxelData[i].filledCount, 4);
        h = m3Hash64(h, world->voxels.voxelData[i].occupancy,
                     (int32_t)sizeof(world->voxels.voxelData[i].occupancy));
        h = m3Hash64(h, world->voxels.voxelData[i].payload,
                     (int32_t)sizeof(world->voxels.voxelData[i].payload));
        h = m3Hash64(h, world->voxels.voxelData[i].fill,
                     (int32_t)sizeof(world->voxels.voxelData[i].fill));
    }
    return h;
}

static uint64_t HashJoints(const m3World* world, uint64_t h)
{
    int32_t maxJoint = world->joints.jointPool.maxIndex;
    for (int32_t i = 0; i < maxJoint; ++i)
    {
        uint8_t alive = world->joints.jointPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->joints.jointType[i], 1);
        h = m3Hash64(h, &world->joints.jointBodyA[i], 4);
        h = m3Hash64(h, &world->joints.jointBodyB[i], 4);
        h = m3Hash64(h, &world->joints.jointLocalA[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointLocalB[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointImpulse[i], (int32_t)sizeof(m3Vec3));
        if (world->joints.jointBreak[i].x != 0.0f || world->joints.jointBreak[i].y != 0.0f)
        {
            // Break thresholds fold off-default (additive rule,
            // eighth use).
            h = m3Hash64(h, &world->joints.jointBreak[i], (int32_t)sizeof(m3Vec3));
        }
        if ((world->joints.jointFlags[i] & M3_JOINT_SPRING) != 0 ||
            world->joints.jointTargetScalar[i] != 0.0f || world->joints.jointTargetQ[i].x != 0.0f ||
            world->joints.jointTargetQ[i].y != 0.0f || world->joints.jointTargetQ[i].z != 0.0f ||
            world->joints.jointTargetQ[i].w != 1.0f ||
            world->joints.jointSpringImpulse[i].x != 0.0f ||
            world->joints.jointSpringImpulse[i].y != 0.0f ||
            world->joints.jointSpringImpulse[i].z != 0.0f)
        {
            // Drive springs fold off-default (additive rule, ninth
            // use). The impulse joins: it is dynamics state.
            h = m3Hash64(h, &world->joints.jointSpring[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->joints.jointTargetScalar[i], 4);
            h = m3Hash64(h, &world->joints.jointTargetQ[i], (int32_t)sizeof(m3Quat));
            h = m3Hash64(h, &world->joints.jointSpringImpulse[i], (int32_t)sizeof(m3Vec3));
        }
        h = m3Hash64(h, &world->joints.jointPerpImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointLimitImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointAngularImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointFrameQA[i], (int32_t)sizeof(m3Quat));
        h = m3Hash64(h, &world->joints.jointFrameQB[i], (int32_t)sizeof(m3Quat));
        h = m3Hash64(h, &world->joints.jointFlags[i], 1);
        if (world->joints.jointType[i] == (uint8_t)m3_genericJoint)
        {
            // The generic joint's fields fold only for that type.
            h = m3Hash64(h, &world->joints.jointGenericModes[i], 2);
            h = m3Hash64(h, &world->joints.jointGenLinLower[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->joints.jointGenLinUpper[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->joints.jointGenAngLower[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->joints.jointGenAngUpper[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->joints.jointType[i] == (uint8_t)m3_pulleyJoint)
        {
            // The rope's world anchors exist only under a pulley.
            h = m3Hash64(h, &world->joints.jointGroundA[i], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->joints.jointGroundB[i], (int32_t)sizeof(m3Pos3));
        }
    }
    return h;
}

static uint64_t HashWater(const m3World* world, uint64_t h)
{
    // Water volumes: folded ONLY while any volume is alive
    // (the off-default law, in its own block); the pool identity
    // rides along so a destroyed-and-recreated volume moves bits.
    {
        int32_t waterAlive = 0;
        for (int32_t k = 0; k < world->water.waterPool.maxIndex; ++k)
        {
            waterAlive += world->water.waterPool.alive[k];
        }
        if (waterAlive > 0)
        {
            h = m3Hash64(h, world->water.waterLo, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, world->water.waterHi, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, world->water.waterDensity,
                         M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->water.waterLinDrag,
                         M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->water.waterAngDrag,
                         M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->water.waterFlow, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, world->water.waterPool.alive, M3_MAX_WATER_VOLUMES);
        }
    }
    return h;
}

static uint64_t HashContacts(const m3World* world, uint64_t h)
{
    // Pairs and manifolds: warm-start impulses are simulation state
    // (they steer the next solve), so they are part of what the world
    // IS.
    h = m3Hash64(h, &world->contacts.pairCount, 4);
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        h = m3Hash64(h, &world->contacts.pairKeys[i], 8);
        h = m3Hash64(h, &world->contacts.manifolds[i], (int32_t)sizeof(m3Manifold));
    }
    return h;
}

uint64_t m3World_Hash(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    // Dead slots contribute only their liveness byte: destroy zeroes
    // state, but the hash must not depend on that coincidence.
    uint64_t h = HashWorldHeader(world, M3_HASH_INIT);
    h = HashBodies(world, h);
    h = HashShapes(world, h);
    h = HashCharacters(world, h);
    h = HashSoftBodies(world, h);
    h = HashVehicles(world, h);
    h = HashVoxels(world, h);
    h = HashJoints(world, h);
    h = HashWater(world, h);
    h = HashContacts(world, h);
    return h;
}
