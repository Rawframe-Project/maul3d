// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Def validation gate: creation and restore check the same things. Every
// numeric field of every def is poisoned in turn (NaN, both infinities,
// negatives, zero, huge values, unknown enums and counts). A create
// either refuses with one misuse, with m3_errorInvalid for NaN and
// m3_errorInvalid or m3_errorLimit for any other value, or what it built
// survives a snapshot restored into a twin world, before and after two
// steps. A def that creation takes but restore refuses would make a
// world that cannot roll back.
//
// Replay checks the same things too. Each create is also recorded clean,
// the def in the tape is poisoned the same way, and replaying the tape
// must agree with the create: it succeeds where the create did, and
// otherwise refuses with m3_errorInvalid, counts no misuse and leaves the
// world as it was.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/maul3d.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Field
{
    const char* name;
    size_t offset;
    char kind; // 'f' float, 'd' double, 'i' int32
} Field;

#define FIELD(type, member, kind) {#member, offsetof(type, member), kind}
#define COUNT(array)              ((int32_t)(sizeof(array) / sizeof((array)[0])))

static const float s_floats[] = {NAN, INFINITY, -INFINITY, -1.0f, 0.0f, 1.0e30f, -1.0e30f};
static const double s_doubles[] = {(double)NAN, (double)INFINITY, -(double)INFINITY, 1.0e300,
                                   -1.0e300};
static const int32_t s_ints[] = {-1, 0, 99, INT32_MAX, INT32_MIN};

// Every world the probe makes allocates through this host allocator,
// which refuses any block over 64 MB. A poisoned capacity of INT32_MAX
// is a valid request for a world bigger than the host has, and the
// library must answer m3_errorCapacity; without the cap the request
// would reach the C library, which on wasm aborts instead of returning
// NULL. No object create may allocate a block that large.
#define PROBE_BLOCK_LIMIT ((size_t)64 << 20)

static int32_t s_refusedBlocks;

static void* ProbeAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    if (bytes > PROBE_BLOCK_LIMIT)
    {
        s_refusedBlocks += 1;
        return NULL;
    }
    return malloc(bytes);
}

static void ProbeFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    (void)context;
    free(memory);
}

// The world every probe builds in: a floor and two dynamic boxes.
static m3WorldDef s_worldDef;
static m3BodyId s_boxA;
static m3BodyId s_boxB;

static m3World* Scene(void)
{
    m3World* world = NewWorld(&s_worldDef);
    m3BodyDef gd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, NewBody(world, &gd), &sd, &floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    s_boxA = NewBody(world, &bd);
    NewBoxShape(world, s_boxA, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    bd.position = (m3Pos3){2.0, 1.0, 0.0};
    s_boxB = NewBody(world, &bd);
    NewBoxShape(world, s_boxB, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return world;
}

// Whether a snapshot of world restores into a fresh twin.
static bool Restores(const m3World* world)
{
    int32_t capacity = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)capacity);
    m3World* twin = NewWorld(&s_worldDef);
    int32_t bytes = 0;
    bool ok = snap != NULL && twin != NULL &&
              m3World_Snapshot(world, snap, capacity, &bytes) == m3_success &&
              m3World_Restore(twin, snap, bytes) == m3_success;
    m3DestroyWorld(twin);
    free(snap);
    return ok;
}

typedef m3Result Builder(m3World* world, const void* def);

typedef struct Target
{
    const char* what;
    const void* base;
    size_t size;
    Builder* build;
    bool taped; // the create journals its def at the start of its record
} Target;

static void Poison(uint8_t* def, const Field* field, int32_t value)
{
    switch (field->kind)
    {
    case 'f':
        memcpy(def + field->offset, &s_floats[value], sizeof(float));
        break;
    case 'd':
        memcpy(def + field->offset, &s_doubles[value], sizeof(double));
        break;
    default:
        memcpy(def + field->offset, &s_ints[value], sizeof(int32_t));
        break;
    }
}

static int32_t ValueCount(char kind)
{
    return kind == 'f' ? COUNT(s_floats) : (kind == 'd' ? COUNT(s_doubles) : COUNT(s_ints));
}

// Whether a refusal names its poison: NaN is invalid, and any other
// value is invalid or past a bound.
static bool RefusedAs(const Field* field, int32_t value, m3Result result)
{
    bool nan = (field->kind == 'f' && s_floats[value] != s_floats[value]) ||
               (field->kind == 'd' && s_doubles[value] != s_doubles[value]);
    return result == m3_errorInvalid || (!nan && result == m3_errorLimit);
}

static void Report(const char* what, const Field* field, int32_t value, const char* problem)
{
    char message[160];
    snprintf(message, sizeof(message), "%s.%s poisoned with value %d: %s", what, field->name,
             (int)value, problem);
    CHECK(false, message);
}

// Records the clean create, poisons the def in the tape's last record
// (the create) and replays the tape over the scene the create started
// from. Whether the replay agrees with what the poisoned create did.
static bool ReplayAgrees(const Target* target, const Field* field, int32_t value, m3Result created)
{
    static uint8_t tape[65536];
    m3World* world = Scene();
    int32_t capacity = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)capacity);
    int32_t snapBytes = 0;
    bool ok = snap != NULL && m3World_Snapshot(world, snap, capacity, &snapBytes) == m3_success &&
              StartJournal(world, tape, (int32_t)sizeof(tape)) &&
              target->build(world, target->base) == m3_success;
    int32_t tapeBytes = StopJournal(world);
    // The records follow the tape's 16-byte state prefix.
    int32_t last = -1;
    for (int32_t at = 16; ok && at + 8 <= tapeBytes;)
    {
        int32_t payloadBytes;
        memcpy(&payloadBytes, tape + at + 4, sizeof(payloadBytes));
        last = at;
        at += 8 + payloadBytes;
    }
    ok = ok && last >= 0 && m3World_Restore(world, snap, snapBytes) == m3_success;
    if (ok)
    {
        Poison(tape + last + 8, field, value);
        uint64_t hash = m3World_Hash(world);
        uint64_t misuse = m3World_GetCounters(world).misuse;
        m3Result replayed = m3World_ReplayJournal(world, tape, tapeBytes);
        ok = created == m3_success
                 ? replayed == m3_success
                 : replayed == m3_errorFormat && m3World_GetCounters(world).misuse == misuse &&
                       m3World_Hash(world) == hash;
    }
    free(snap);
    m3DestroyWorld(world);
    return ok;
}

static void Probe(const Target* target, const Field* fields, int32_t count)
{
    uint8_t def[2048];
    if (target->size > sizeof(def))
    {
        CHECK(false, "a def fits the probe buffer");
        return;
    }
    const char* what = target->what;
    for (int32_t f = 0; f < count; ++f)
    {
        for (int32_t v = 0; v < ValueCount(fields[f].kind); ++v)
        {
            memcpy(def, target->base, target->size);
            Poison(def, &fields[f], v);
            m3World* world = Scene();
            uint64_t misuse = m3World_GetCounters(world).misuse;
            m3Result result = target->build(world, def);
            if (result == m3_success)
            {
                if (!Restores(world))
                {
                    Report(what, &fields[f], v, "created, but the snapshot does not restore");
                }
                (void)m3World_Step(world, 1.0f / 60.0f, 4);
                (void)m3World_Step(world, 1.0f / 60.0f, 4);
                if (!Restores(world))
                {
                    Report(what, &fields[f], v, "created, but two steps later it does not restore");
                }
            }
            else if (!RefusedAs(&fields[f], v, result) ||
                     m3World_GetCounters(world).misuse != misuse + 1u)
            {
                Report(what, &fields[f], v,
                       "refused without m3_errorInvalid or m3_errorLimit and one misuse");
            }
            m3DestroyWorld(world);
            if (target->taped && !ReplayAgrees(target, &fields[f], v, result))
            {
                Report(what, &fields[f], v, "a tape with it replays unlike the create");
            }
            if (s_refusedBlocks != 0)
            {
                Report(what, &fields[f], v, "a create asked for a block over the probe's limit");
                s_refusedBlocks = 0;
            }
        }
    }
}

static m3Result BuildBody(m3World* world, const void* def)
{
    m3BodyId body;
    return m3CreateBody(world, (const m3BodyDef*)def, &body);
}

static m3Result BuildShape(m3World* world, const void* def)
{
    m3ShapeId shape;
    return m3CreateBoxShape(world, s_boxA, (const m3ShapeDef*)def, (m3Vec3){0.3f, 0.3f, 0.3f},
                            &shape);
}

static m3Result BuildJoint(m3World* world, const void* def)
{
    m3JointId joint;
    return m3CreateJoint(world, (const m3JointDef*)def, &joint);
}

static m3Result BuildCharacter(m3World* world, const void* def)
{
    m3CharacterId character;
    return m3CreateCharacter(world, (const m3CharacterDef*)def, &character);
}

static m3VehicleDef CarDef(void)
{
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = s_boxA;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
        vd.wheels[w].steerable = (w & 1) != 0;
    }
    return vd;
}

static m3Result BuildVehicle(m3World* world, const void* def)
{
    m3VehicleId vehicle;
    return m3CreateVehicle(world, (const m3VehicleDef*)def, &vehicle);
}

static m3Result BuildDrivetrain(m3World* world, const void* def)
{
    m3VehicleDef vd = CarDef();
    m3VehicleId car;
    if (m3CreateVehicle(world, &vd, &car) != m3_success)
    {
        return m3_errorState; // the probe's own car must build
    }
    uint64_t misuse = m3World_GetCounters(world).misuse;
    m3Vehicle_SetDrivetrain(world, car, (const m3DrivetrainDef*)def);
    return m3World_GetCounters(world).misuse == misuse ? m3_success : m3_errorInvalid;
}

static m3Result BuildSoftBody(m3World* world, const void* def)
{
    m3SoftBodyId softBody;
    return m3CreateSoftBody(world, (const m3SoftBodyDef*)def, &softBody);
}

static m3Result BuildWater(m3World* world, const void* def)
{
    m3WaterVolumeId volume;
    return m3CreateWaterVolume(world, (const m3WaterVolumeDef*)def, &volume);
}

static void ProbeBodies(void)
{
    static const Field fields[] = {
        FIELD(m3BodyDef, type, 'i'),
        FIELD(m3BodyDef, position.x, 'd'),
        FIELD(m3BodyDef, rotation.x, 'f'),
        FIELD(m3BodyDef, rotation.w, 'f'),
        FIELD(m3BodyDef, linearVelocity.x, 'f'),
        FIELD(m3BodyDef, angularVelocity.y, 'f'),
        FIELD(m3BodyDef, gravityScale, 'f'),
        FIELD(m3BodyDef, linearDamping, 'f'),
        FIELD(m3BodyDef, angularDamping, 'f'),
        FIELD(m3BodyDef, sleepThreshold, 'f'),
    };
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 4.0, 0.0};
    Target target = {"m3BodyDef", &bd, sizeof(bd), BuildBody, true};
    Probe(&target, fields, COUNT(fields));
}

static void ProbeShapes(void)
{
    static const Field fields[] = {
        FIELD(m3ShapeDef, density, 'f'),           FIELD(m3ShapeDef, friction, 'f'),
        FIELD(m3ShapeDef, restitution, 'f'),       FIELD(m3ShapeDef, groupIndex, 'i'),
        FIELD(m3ShapeDef, rollingResistance, 'f'), FIELD(m3ShapeDef, surfaceVelocity.x, 'f'),
        FIELD(m3ShapeDef, localPosition.x, 'f'),   FIELD(m3ShapeDef, localRotation.x, 'f'),
        FIELD(m3ShapeDef, localRotation.w, 'f'),
    };
    m3ShapeDef sd = m3DefaultShapeDef();
    Target target = {"m3ShapeDef", &sd, sizeof(sd), BuildShape, true};
    Probe(&target, fields, COUNT(fields));
}

static void ProbeJoints(void)
{
    static const Field fields[] = {
        FIELD(m3JointDef, type, 'i'),
        FIELD(m3JointDef, localAnchorA.x, 'f'),
        FIELD(m3JointDef, localAnchorB.y, 'f'),
        FIELD(m3JointDef, localAxisA.x, 'f'),
        FIELD(m3JointDef, localAxisA.y, 'f'),
        FIELD(m3JointDef, localAxisB.z, 'f'),
        FIELD(m3JointDef, lowerLimit, 'f'),
        FIELD(m3JointDef, upperLimit, 'f'),
        FIELD(m3JointDef, motorSpeed, 'f'),
        FIELD(m3JointDef, maxMotorEffort, 'f'),
        FIELD(m3JointDef, coneAngle, 'f'),
        FIELD(m3JointDef, genericLinearLower[0], 'f'),
        FIELD(m3JointDef, genericLinearUpper[0], 'f'),
        FIELD(m3JointDef, genericAngularLower[1], 'f'),
        FIELD(m3JointDef, genericAngularUpper[1], 'f'),
        FIELD(m3JointDef, ratio, 'f'),
        FIELD(m3JointDef, groundAnchorA.y, 'd'),
        FIELD(m3JointDef, groundAnchorB.y, 'd'),
    };
    // Every joint type whose default def joins the two boxes is probed,
    // with its limit, motor and cone on so their fields are read.
    for (int32_t type = m3_sphericalJoint; type <= m3_pulleyJoint; ++type)
    {
        m3JointDef jd = m3DefaultJointDef();
        jd.type = (m3JointType)type;
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.enableLimit = true;
        jd.enableMotor = true;
        jd.enableCone = true;
        jd.groundAnchorA = (m3Pos3){0.0, 4.0, 0.0};
        jd.groundAnchorB = (m3Pos3){2.0, 4.0, 0.0};
        m3World* world = Scene();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        m3Result base = BuildJoint(world, &jd);
        m3DestroyWorld(world);
        if (base != m3_success)
        {
            continue; // the type needs more than its default def
        }
        char what[32];
        snprintf(what, sizeof(what), "m3JointDef(type %d)", (int)type);
        Target target = {what, &jd, sizeof(jd), BuildJoint, true};
        Probe(&target, fields, COUNT(fields));
    }
}

static void ProbeCharacters(void)
{
    static const Field fields[] = {
        FIELD(m3CharacterDef, position.y, 'd'),       FIELD(m3CharacterDef, radius, 'f'),
        FIELD(m3CharacterDef, halfHeight, 'f'),       FIELD(m3CharacterDef, maxSlopeAngle, 'f'),
        FIELD(m3CharacterDef, snapDistance, 'f'),     FIELD(m3CharacterDef, skin, 'f'),
        FIELD(m3CharacterDef, stepHeight, 'f'),       FIELD(m3CharacterDef, mass, 'f'),
        FIELD(m3CharacterDef, pushMaxMassRatio, 'f'),
    };
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){-3.0, 1.0, 0.0};
    Target target = {"m3CharacterDef", &cd, sizeof(cd), BuildCharacter, true};
    Probe(&target, fields, COUNT(fields));
}

static void ProbeVehicles(void)
{
    static const Field fields[] = {
        FIELD(m3VehicleDef, wheelCount, 'i'),
        FIELD(m3VehicleDef, maxSteerAngle, 'f'),
        FIELD(m3VehicleDef, driveForce, 'f'),
        FIELD(m3VehicleDef, brakeForce, 'f'),
        FIELD(m3VehicleDef, tireGrip, 'f'),
        FIELD(m3VehicleDef, leanStabilization, 'f'),
        FIELD(m3VehicleDef, wheels[0].anchor.x, 'f'),
        FIELD(m3VehicleDef, wheels[0].direction.y, 'f'),
        FIELD(m3VehicleDef, wheels[0].restLength, 'f'),
        FIELD(m3VehicleDef, wheels[0].travel, 'f'),
        FIELD(m3VehicleDef, wheels[0].hertz, 'f'),
        FIELD(m3VehicleDef, wheels[0].zeta, 'f'),
        FIELD(m3VehicleDef, wheels[0].radius, 'f'),
        FIELD(m3VehicleDef, wheels[0].brakeShare, 'f'),
    };
    m3World* world = Scene(); // fixes s_boxA, the same id in every scene
    m3DestroyWorld(world);
    m3VehicleDef vd = CarDef();
    Target target = {"m3VehicleDef", &vd, sizeof(vd), BuildVehicle, true};
    Probe(&target, fields, COUNT(fields));
}

static void ProbeDrivetrains(void)
{
    static const Field fields[] = {
        FIELD(m3DrivetrainDef, curveCount, 'i'),     FIELD(m3DrivetrainDef, curveRpm[0], 'f'),
        FIELD(m3DrivetrainDef, curveTorque[1], 'f'), FIELD(m3DrivetrainDef, gearCount, 'i'),
        FIELD(m3DrivetrainDef, gearRatio[0], 'f'),   FIELD(m3DrivetrainDef, reverseRatio, 'f'),
        FIELD(m3DrivetrainDef, finalDrive, 'f'),     FIELD(m3DrivetrainDef, shiftUpRpm, 'f'),
        FIELD(m3DrivetrainDef, shiftDownRpm, 'f'),   FIELD(m3DrivetrainDef, clutchSteps, 'i'),
        FIELD(m3DrivetrainDef, diffMode, 'i'),       FIELD(m3DrivetrainDef, diffCouple, 'f'),
    };
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    // The drivetrain is a setter on a car the probe builds first.
    Target target = {"m3DrivetrainDef", &dd, sizeof(dd), BuildDrivetrain, false};
    Probe(&target, fields, COUNT(fields));
}

static void ProbeSoftBodies(void)
{
    static const Field fields[] = {
        FIELD(m3SoftBodyDef, position.y, 'd'),   FIELD(m3SoftBodyDef, countX, 'i'),
        FIELD(m3SoftBodyDef, countY, 'i'),       FIELD(m3SoftBodyDef, countZ, 'i'),
        FIELD(m3SoftBodyDef, spacing, 'f'),      FIELD(m3SoftBodyDef, particleMass, 'f'),
        FIELD(m3SoftBodyDef, compliance, 'f'),   FIELD(m3SoftBodyDef, radius, 'f'),
        FIELD(m3SoftBodyDef, gravityScale, 'f'), FIELD(m3SoftBodyDef, bendCompliance, 'f'),
        FIELD(m3SoftBodyDef, pressure, 'f'),     FIELD(m3SoftBodyDef, maxDeviation, 'f'),
    };
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){-4.0, 3.0, 0.0};
    Target target = {"m3SoftBodyDef", &sb, sizeof(sb), BuildSoftBody, true};
    Probe(&target, fields, COUNT(fields));
}

static void ProbeWater(void)
{
    static const Field fields[] = {
        FIELD(m3WaterVolumeDef, lo.y, 'd'),        FIELD(m3WaterVolumeDef, hi.y, 'd'),
        FIELD(m3WaterVolumeDef, density, 'f'),     FIELD(m3WaterVolumeDef, linearDrag, 'f'),
        FIELD(m3WaterVolumeDef, angularDrag, 'f'), FIELD(m3WaterVolumeDef, flow.x, 'f'),
    };
    m3WaterVolumeDef wd = m3DefaultWaterVolumeDef();
    wd.lo = (m3Pos3){-5.0, 0.0, -5.0};
    wd.hi = (m3Pos3){5.0, 2.0, 5.0};
    Target target = {"m3WaterVolumeDef", &wd, sizeof(wd), BuildWater, true};
    Probe(&target, fields, COUNT(fields));
}

// The world def has no world to count misuse against: a create refuses
// as RefusedAs says, or with m3_errorCapacity only for a capacity of
// INT32_MAX, the one poison too big to allocate: either the probe's
// allocator or the library's own 2 GB block limit refuses it. A world it
// builds must restore into its twin.
static void ProbeWorlds(void)
{
    static const Field fields[] = {
        FIELD(m3WorldDef, gravity.y, 'f'),           FIELD(m3WorldDef, bodyCapacity, 'i'),
        FIELD(m3WorldDef, shapeCapacity, 'i'),       FIELD(m3WorldDef, meshCapacity, 'i'),
        FIELD(m3WorldDef, jointCapacity, 'i'),       FIELD(m3WorldDef, voxelCapacity, 'i'),
        FIELD(m3WorldDef, characterCapacity, 'i'),   FIELD(m3WorldDef, vehicleCapacity, 'i'),
        FIELD(m3WorldDef, softBodyCapacity, 'i'),    FIELD(m3WorldDef, pairCapacity, 'i'),
        FIELD(m3WorldDef, contactHertz, 'f'),        FIELD(m3WorldDef, contactDampingRatio, 'f'),
        FIELD(m3WorldDef, contactPushMaxSpeed, 'f'), FIELD(m3WorldDef, restitutionThreshold, 'f'),
        FIELD(m3WorldDef, maximumLinearSpeed, 'f'),  FIELD(m3WorldDef, maximumAngularSpeed, 'f'),
        FIELD(m3WorldDef, hitEventThreshold, 'f'),
    };
    for (int32_t f = 0; f < COUNT(fields); ++f)
    {
        for (int32_t v = 0; v < ValueCount(fields[f].kind); ++v)
        {
            m3WorldDef def = s_worldDef;
            Poison((uint8_t*)&def, &fields[f], v);
            m3World* world = NULL;
            bool tooBig = fields[f].kind == 'i' && s_ints[v] == INT32_MAX;
            m3Result result = m3CreateWorld(&def, &world);
            if (result != m3_success)
            {
                if (!RefusedAs(&fields[f], v, result) && !(result == m3_errorCapacity && tooBig))
                {
                    Report("m3WorldDef", &fields[f], v,
                           "refused with neither m3_errorInvalid, m3_errorLimit nor, for "
                           "INT32_MAX, m3_errorCapacity");
                }
                continue;
            }
            m3World* twin = NULL;
            int32_t capacity = m3World_SnapshotSize(world);
            uint8_t* snap = (uint8_t*)malloc((size_t)capacity);
            int32_t bytes = 0;
            bool ok = snap != NULL && m3CreateWorld(&def, &twin) == m3_success &&
                      m3World_Snapshot(world, snap, capacity, &bytes) == m3_success &&
                      m3World_Restore(twin, snap, bytes) == m3_success;
            if (!ok)
            {
                Report("m3WorldDef", &fields[f], v, "created, but the snapshot does not restore");
            }
            free(snap);
            m3DestroyWorld(twin);
            m3DestroyWorld(world);
        }
    }
}

int main(void)
{
    s_worldDef = m3DefaultWorldDef();
    s_worldDef.allocator.alloc = ProbeAlloc;
    s_worldDef.allocator.free = ProbeFree;
    s_worldDef.bodyCapacity = 64;
    s_worldDef.shapeCapacity = 64;
    s_worldDef.jointCapacity = 8;
    s_worldDef.characterCapacity = 2;
    s_worldDef.vehicleCapacity = 2;
    s_worldDef.softBodyCapacity = 2;
    m3DestroyWorld(Scene()); // fixes the scene's ids before any def is built
    ProbeBodies();
    ProbeShapes();
    ProbeJoints();
    ProbeCharacters();
    ProbeVehicles();
    ProbeDrivetrains();
    ProbeSoftBodies();
    ProbeWater();
    ProbeWorlds();
    if (s_failures == 0)
    {
        printf("test_defprobe: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
