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

// Only for the joint create record's layout, to poison a joint def's field
// where its tape holds it. No internal function is called, so the suite
// still runs against the shared library.
#include "../src/journal.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Field
{
    const char* name;
    size_t offset;
    size_t tapeOffset; // where the create's journal record holds it
    char kind;         // 'f' float, 'd' double, 'i' int32, 'b' bool
} Field;

#define FIELD(type, member, kind) {#member, offsetof(type, member), offsetof(type, member), kind}
// A joint def's field, which its tape holds in the full joint def.
#define JOINT_FIELD(type, member, full, kind)                                                      \
    {#member, offsetof(type, member), offsetof(m3JointDef, full), kind}
#define COUNT(array) ((int32_t)(sizeof(array) / sizeof((array)[0])))

static const float s_floats[] = {NAN, INFINITY, -INFINITY, -1.0f, 0.0f, 1.0e30f, -1.0e30f};
static const double s_doubles[] = {(double)NAN, (double)INFINITY, -(double)INFINITY, 1.0e300,
                                   -1.0e300};
static const int32_t s_ints[] = {-1, 0, 99, INT32_MAX, INT32_MIN};
// A bool holds only these: any other byte is not a bool a caller can pass.
static const bool s_bools[] = {false, true};

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
    int64_t capacity = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)capacity);
    m3World* twin = NewWorld(&s_worldDef);
    int64_t bytes = 0;
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

static void Poison(uint8_t* def, size_t offset, char kind, int32_t value)
{
    switch (kind)
    {
    case 'f':
        memcpy(def + offset, &s_floats[value], sizeof(float));
        break;
    case 'd':
        memcpy(def + offset, &s_doubles[value], sizeof(double));
        break;
    case 'b':
        memcpy(def + offset, &s_bools[value], sizeof(bool));
        break;
    default:
        memcpy(def + offset, &s_ints[value], sizeof(int32_t));
        break;
    }
}

static int32_t ValueCount(char kind)
{
    switch (kind)
    {
    case 'f':
        return COUNT(s_floats);
    case 'd':
        return COUNT(s_doubles);
    case 'b':
        return COUNT(s_bools);
    default:
        return COUNT(s_ints);
    }
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
    int64_t capacity = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)capacity);
    int64_t snapBytes = 0;
    bool ok = snap != NULL && m3World_Snapshot(world, snap, capacity, &snapBytes) == m3_success &&
              StartJournal(world, tape, (int32_t)sizeof(tape)) &&
              target->build(world, target->base) == m3_success;
    int64_t tapeBytes = StopJournal(world);
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
        Poison(tape + last + 8, field->tapeOffset, field->kind, value);
        uint64_t hash = m3World_Hash(world);
        uint64_t misuse = m3World_GetCounters(world).misuseCount;
        m3Result replayed = m3World_ReplayJournal(world, tape, tapeBytes);
        ok = created == m3_success
                 ? replayed == m3_success
                 : replayed == m3_errorFormat && m3World_GetCounters(world).misuseCount == misuse &&
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
            Poison(def, fields[f].offset, fields[f].kind, v);
            m3World* world = Scene();
            uint64_t misuse = m3World_GetCounters(world).misuseCount;
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
                     m3World_GetCounters(world).misuseCount != misuse + 1u)
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

#define BUILD_JOINT(Name)                                                                          \
    static m3Result Build##Name##Joint(m3World* world, const void* def)                            \
    {                                                                                              \
        m3JointId joint;                                                                           \
        return m3Create##Name##Joint(world, (const m3##Name##JointDef*)def, &joint);               \
    }
BUILD_JOINT(Spherical)
BUILD_JOINT(Revolute)
BUILD_JOINT(Prismatic)
BUILD_JOINT(Weld)
BUILD_JOINT(Distance)
BUILD_JOINT(Generic)
BUILD_JOINT(Wheel)
BUILD_JOINT(Filter)
BUILD_JOINT(Parallel)
BUILD_JOINT(Motor)
BUILD_JOINT(Gear)
BUILD_JOINT(Pulley)

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
    uint64_t misuse = m3World_GetCounters(world).misuseCount;
    m3Vehicle_SetDrivetrain(world, car, (const m3DrivetrainDef*)def);
    return m3World_GetCounters(world).misuseCount == misuse ? m3_success : m3_errorInvalid;
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

// One joint type's def: the base def must build, or the probe fails
// rather than dropping the type, and then every field is poisoned.
static void ProbeJoint(const char* what, const void* base, size_t size, Builder* build,
                       const Field* fields, int32_t count)
{
    m3World* world = Scene();
    m3Result result = build(world, base);
    m3DestroyWorld(world);
    if (result != m3_success)
    {
        char message[80];
        snprintf(message, sizeof(message), "%s: the base def builds", what);
        CHECK(false, message);
        return;
    }
    Target target = {what, base, size, build, true};
    Probe(&target, fields, count);
}

#define ANCHOR_FIELDS(type)                                                                        \
    JOINT_FIELD(type, localAnchorA.x, localAnchorA.x, 'f'),                                        \
        JOINT_FIELD(type, localAnchorB.y, localAnchorB.y, 'f')
#define AXIS_FIELDS(type)                                                                          \
    JOINT_FIELD(type, localAxisA.x, localAxisA.x, 'f'),                                            \
        JOINT_FIELD(type, localAxisA.y, localAxisA.y, 'f'),                                        \
        JOINT_FIELD(type, localAxisB.z, localAxisB.z, 'f')
#define SPRING_FIELDS(type)                                                                        \
    JOINT_FIELD(type, enableSpring, enableSpring, 'b'),                                            \
        JOINT_FIELD(type, springHertz, springHertz, 'f'),                                          \
        JOINT_FIELD(type, springDampingRatio, springDampingRatio, 'f')
#define MOTOR_FIELDS(type)                                                                         \
    JOINT_FIELD(type, motorSpeed, motorSpeed, 'f'),                                                \
        JOINT_FIELD(type, maxMotorEffort, maxMotorEffort, 'f')

// Every joint type is probed, each with every switch it takes on so the
// fields behind it are read: the limit, the motor, the cone and the
// spring.
static void ProbeJoints(void)
{
    {
        static const Field fields[] = {
            ANCHOR_FIELDS(m3SphericalJointDef),
            AXIS_FIELDS(m3SphericalJointDef),
            JOINT_FIELD(m3SphericalJointDef, lowerTwist, lowerLimit, 'f'),
            JOINT_FIELD(m3SphericalJointDef, upperTwist, upperLimit, 'f'),
            JOINT_FIELD(m3SphericalJointDef, coneAngle, coneAngle, 'f'),
            SPRING_FIELDS(m3SphericalJointDef),
        };
        m3SphericalJointDef jd = m3DefaultSphericalJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.enableLimit = true;
        jd.enableCone = true;
        jd.enableSpring = true;
        jd.springHertz = 2.0f;
        jd.springDampingRatio = 0.5f;
        ProbeJoint("m3SphericalJointDef", &jd, sizeof(jd), BuildSphericalJoint, fields,
                   COUNT(fields));
    }
    {
        static const Field fields[] = {
            ANCHOR_FIELDS(m3RevoluteJointDef),
            AXIS_FIELDS(m3RevoluteJointDef),
            JOINT_FIELD(m3RevoluteJointDef, lowerAngle, lowerLimit, 'f'),
            JOINT_FIELD(m3RevoluteJointDef, upperAngle, upperLimit, 'f'),
            MOTOR_FIELDS(m3RevoluteJointDef),
            SPRING_FIELDS(m3RevoluteJointDef),
        };
        m3RevoluteJointDef jd = m3DefaultRevoluteJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.enableLimit = true;
        jd.enableMotor = true;
        jd.enableSpring = true;
        jd.springHertz = 2.0f;
        jd.springDampingRatio = 0.5f;
        ProbeJoint("m3RevoluteJointDef", &jd, sizeof(jd), BuildRevoluteJoint, fields,
                   COUNT(fields));
    }
    {
        static const Field fields[] = {
            ANCHOR_FIELDS(m3PrismaticJointDef),
            AXIS_FIELDS(m3PrismaticJointDef),
            JOINT_FIELD(m3PrismaticJointDef, lowerTranslation, lowerLimit, 'f'),
            JOINT_FIELD(m3PrismaticJointDef, upperTranslation, upperLimit, 'f'),
            MOTOR_FIELDS(m3PrismaticJointDef),
            SPRING_FIELDS(m3PrismaticJointDef),
        };
        m3PrismaticJointDef jd = m3DefaultPrismaticJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.enableLimit = true;
        jd.enableMotor = true;
        jd.enableSpring = true;
        jd.springHertz = 2.0f;
        jd.springDampingRatio = 0.5f;
        ProbeJoint("m3PrismaticJointDef", &jd, sizeof(jd), BuildPrismaticJoint, fields,
                   COUNT(fields));
    }
    {
        static const Field fields[] = {ANCHOR_FIELDS(m3WeldJointDef)};
        m3WeldJointDef jd = m3DefaultWeldJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        ProbeJoint("m3WeldJointDef", &jd, sizeof(jd), BuildWeldJoint, fields, COUNT(fields));
    }
    {
        static const Field fields[] = {
            ANCHOR_FIELDS(m3DistanceJointDef),
            JOINT_FIELD(m3DistanceJointDef, length, length, 'f'),
            JOINT_FIELD(m3DistanceJointDef, minLength, minLength, 'f'),
            JOINT_FIELD(m3DistanceJointDef, maxLength, maxLength, 'f'),
            SPRING_FIELDS(m3DistanceJointDef),
        };
        m3DistanceJointDef jd = m3DefaultDistanceJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.enableSpring = true;
        jd.springHertz = 2.0f;
        jd.springDampingRatio = 0.5f;
        ProbeJoint("m3DistanceJointDef", &jd, sizeof(jd), BuildDistanceJoint, fields,
                   COUNT(fields));
    }
    {
        static const Field fields[] = {
            ANCHOR_FIELDS(m3GenericJointDef),
            AXIS_FIELDS(m3GenericJointDef),
            JOINT_FIELD(m3GenericJointDef, linearModes[0], genericLinear[0], 'i'),
            JOINT_FIELD(m3GenericJointDef, angularModes[2], genericAngular[2], 'i'),
            JOINT_FIELD(m3GenericJointDef, linearLower[0], genericLinearLower[0], 'f'),
            JOINT_FIELD(m3GenericJointDef, linearUpper[0], genericLinearUpper[0], 'f'),
            JOINT_FIELD(m3GenericJointDef, angularLower[1], genericAngularLower[1], 'f'),
            JOINT_FIELD(m3GenericJointDef, angularUpper[1], genericAngularUpper[1], 'f'),
            JOINT_FIELD(m3GenericJointDef, enableMotor, enableMotor, 'b'),
            MOTOR_FIELDS(m3GenericJointDef),
        };
        m3GenericJointDef jd = m3DefaultGenericJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.enableMotor = true;
        ProbeJoint("m3GenericJointDef", &jd, sizeof(jd), BuildGenericJoint, fields, COUNT(fields));
    }
    {
        static const Field fields[] = {
            ANCHOR_FIELDS(m3WheelJointDef),
            JOINT_FIELD(m3WheelJointDef, localAxisA.x, localAxisA.x, 'f'),
            JOINT_FIELD(m3WheelJointDef, localAxisA.y, localAxisA.y, 'f'),
            JOINT_FIELD(m3WheelJointDef, localSpinAxisB.z, localAxisB.z, 'f'),
            JOINT_FIELD(m3WheelJointDef, lowerTranslation, lowerLimit, 'f'),
            JOINT_FIELD(m3WheelJointDef, upperTranslation, upperLimit, 'f'),
            MOTOR_FIELDS(m3WheelJointDef),
            SPRING_FIELDS(m3WheelJointDef),
        };
        m3WheelJointDef jd = m3DefaultWheelJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};    // strut
        jd.localSpinAxisB = (m3Vec3){0.0f, 0.0f, 1.0f}; // axle
        jd.enableLimit = true;
        jd.enableMotor = true;
        jd.enableSpring = true;
        jd.springHertz = 2.0f;
        jd.springDampingRatio = 0.5f;
        ProbeJoint("m3WheelJointDef", &jd, sizeof(jd), BuildWheelJoint, fields, COUNT(fields));
    }
    {
        // No numeric field: the base def must still build.
        m3FilterJointDef jd = m3DefaultFilterJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        ProbeJoint("m3FilterJointDef", &jd, sizeof(jd), BuildFilterJoint, NULL, 0);
    }
    {
        static const Field fields[] = {ANCHOR_FIELDS(m3ParallelJointDef),
                                       AXIS_FIELDS(m3ParallelJointDef)};
        m3ParallelJointDef jd = m3DefaultParallelJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        ProbeJoint("m3ParallelJointDef", &jd, sizeof(jd), BuildParallelJoint, fields,
                   COUNT(fields));
    }
    {
        static const Field fields[] = {
            ANCHOR_FIELDS(m3MotorJointDef),
            JOINT_FIELD(m3MotorJointDef, maxForce, maxForce, 'f'),
            JOINT_FIELD(m3MotorJointDef, maxTorque, maxTorque, 'f'),
            SPRING_FIELDS(m3MotorJointDef),
        };
        m3MotorJointDef jd = m3DefaultMotorJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.enableSpring = true;
        jd.springHertz = 2.0f;
        jd.springDampingRatio = 0.5f;
        ProbeJoint("m3MotorJointDef", &jd, sizeof(jd), BuildMotorJoint, fields, COUNT(fields));
    }
    {
        static const Field fields[] = {
            AXIS_FIELDS(m3GearJointDef),
            JOINT_FIELD(m3GearJointDef, ratio, ratio, 'f'),
        };
        m3GearJointDef jd = m3DefaultGearJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        ProbeJoint("m3GearJointDef", &jd, sizeof(jd), BuildGearJoint, fields, COUNT(fields));
    }
    {
        static const Field fields[] = {
            ANCHOR_FIELDS(m3PulleyJointDef),
            JOINT_FIELD(m3PulleyJointDef, groundAnchorA.y, groundAnchorA.y, 'd'),
            JOINT_FIELD(m3PulleyJointDef, groundAnchorB.y, groundAnchorB.y, 'd'),
            JOINT_FIELD(m3PulleyJointDef, ratio, ratio, 'f'),
        };
        m3PulleyJointDef jd = m3DefaultPulleyJointDef();
        jd.bodyIdA = s_boxA;
        jd.bodyIdB = s_boxB;
        jd.groundAnchorA = (m3Pos3){0.0, 4.0, 0.0};
        jd.groundAnchorB = (m3Pos3){2.0, 4.0, 0.0};
        ProbeJoint("m3PulleyJointDef", &jd, sizeof(jd), BuildPulleyJoint, fields, COUNT(fields));
    }
}

// A part one joint type takes, written into another type's create record.
typedef struct ForeignPart
{
    const char* what;
    Builder* build;
    const void* base;
    void (*write)(m3JointDef* def);
} ForeignPart;

static void ConeOn(m3JointDef* def)
{
    def->enableCone = true;
    def->coneAngle = 0.5f;
}

static void SpringOn(m3JointDef* def)
{
    def->enableSpring = true;
    def->springHertz = 4.0f;
}

static void LimitOn(m3JointDef* def)
{
    def->enableLimit = true;
    def->lowerLimit = 0.5f;
    def->upperLimit = 2.0f;
}

static void MotorOn(m3JointDef* def)
{
    def->enableMotor = true;
    def->motorSpeed = 1.0f;
}

static void NegativeType(m3JointDef* def)
{
    def->type = -1;
}

// Whether a tape whose last record, a clean create, carries the part is
// refused on replay with m3_errorFormat, counting no misuse and leaving
// the world as it was.
static bool ReplayRefusesPart(const ForeignPart* part)
{
    static uint8_t tape[65536];
    m3World* world = Scene();
    int64_t capacity = m3World_GetSnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)capacity);
    int64_t snapBytes = 0;
    bool ok = snap != NULL && m3World_Snapshot(world, snap, capacity, &snapBytes) == m3_success &&
              StartJournal(world, tape, (int32_t)sizeof(tape)) &&
              part->build(world, part->base) == m3_success;
    int64_t tapeBytes = StopJournal(world);
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
        m3JointDef def;
        memcpy(&def, tape + last + 8, sizeof(def));
        part->write(&def);
        memcpy(tape + last + 8, &def, sizeof(def));
        uint64_t hash = m3World_Hash(world);
        uint64_t misuse = m3World_GetCounters(world).misuseCount;
        ok = m3World_ReplayJournal(world, tape, tapeBytes) == m3_errorFormat &&
             m3World_GetCounters(world).misuseCount == misuse && m3World_Hash(world) == hash;
    }
    free(snap);
    m3DestroyWorld(world);
    return ok;
}

// No type's def can carry another type's part, but a tape carries the full
// def, so replay still refuses each one: a cone on a wheel or a hinge, a
// spring on a weld, a limit or a motor on a distance joint, and a negative
// type.
static void ProbeForeignPartsOnReplay(void)
{
    m3WheelJointDef wheel = m3DefaultWheelJointDef();
    wheel.bodyIdA = s_boxA;
    wheel.bodyIdB = s_boxB;
    m3RevoluteJointDef hinge = m3DefaultRevoluteJointDef();
    hinge.bodyIdA = s_boxA;
    hinge.bodyIdB = s_boxB;
    m3WeldJointDef weld = m3DefaultWeldJointDef();
    weld.bodyIdA = s_boxA;
    weld.bodyIdB = s_boxB;
    m3DistanceJointDef rope = m3DefaultDistanceJointDef();
    rope.bodyIdA = s_boxA;
    rope.bodyIdB = s_boxB;
    rope.maxLength = 2.0f;
    const ForeignPart parts[6] = {
        {"a cone on a wheel", BuildWheelJoint, &wheel, ConeOn},
        {"a cone on a hinge", BuildRevoluteJoint, &hinge, ConeOn},
        {"a spring on a weld", BuildWeldJoint, &weld, SpringOn},
        {"a limit on a distance joint", BuildDistanceJoint, &rope, LimitOn},
        {"a motor on a distance joint", BuildDistanceJoint, &rope, MotorOn},
        {"a negative joint type", BuildWeldJoint, &weld, NegativeType},
    };
    for (int32_t k = 0; k < 6; ++k)
    {
        char message[96];
        snprintf(message, sizeof(message), "a replayed create with %s is refused", parts[k].what);
        CHECK(ReplayRefusesPart(&parts[k]), message);
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
        FIELD(m3VehicleDef, wheels[0].springHertz, 'f'),
        FIELD(m3VehicleDef, wheels[0].springDampingRatio, 'f'),
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
        FIELD(m3WaterVolumeDef, bounds.lowerBound.y, 'd'),
        FIELD(m3WaterVolumeDef, bounds.upperBound.y, 'd'),
        FIELD(m3WaterVolumeDef, density, 'f'),
        FIELD(m3WaterVolumeDef, linearDrag, 'f'),
        FIELD(m3WaterVolumeDef, angularDrag, 'f'),
        FIELD(m3WaterVolumeDef, flow.x, 'f'),
    };
    m3WaterVolumeDef wd = m3DefaultWaterVolumeDef();
    wd.bounds.lowerBound = (m3Pos3){-5.0, 0.0, -5.0};
    wd.bounds.upperBound = (m3Pos3){5.0, 2.0, 5.0};
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
            Poison((uint8_t*)&def, fields[f].offset, fields[f].kind, v);
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
            int64_t capacity = m3World_GetSnapshotSize(world);
            uint8_t* snap = (uint8_t*)malloc((size_t)capacity);
            int64_t bytes = 0;
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
    ProbeForeignPartsOnReplay();
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
