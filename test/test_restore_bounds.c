// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restore's magnitude checks at their edges (src/snapshot_bounds.c). A
// scene holding every record kind is snapshotted; then, one value at a
// time, the snapshot with that value at its limit must restore, and the
// one with the next value past it must be refused with the world left as
// it was.

#include "helpers.h"
#include "snapshot_bounds.h"
#include "state_prefix.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    // The snapshot header ahead of the fixed prefix.
    HEADER_BYTES = M3_SNAPSHOT_HEADER_BYTES
};

// The live slots the cases write into.
typedef struct Scene
{
    m3World* world;
    int32_t ball;    // a dynamic body with one sphere
    int32_t hinge;   // a revolute joint with a motor
    int32_t gear;    // a gear joint
    int32_t pulley;  // a pulley joint
    int32_t servo;   // a motor joint
    int32_t wheel;   // a wheel joint
    int32_t hero;    // a character, on a kinematic body
    int32_t vehicle; // a vehicle with a drivetrain
    int32_t soft;    // a closed lattice with an anchor
    int32_t pool;    // a water volume
    int32_t rail;    // a kinematic body with a box
    int32_t dot;     // a soft body of one particle
    int32_t jelly;   // a soft body of one tet
    int32_t six;     // a generic joint with a limited axis
} Scene;

typedef enum Which
{
    which_world,
    which_ball,
    which_ballShape,
    which_heroBody,
    which_hinge,
    which_gear,
    which_pulley,
    which_servo,
    which_wheel,
    which_hero,
    which_vehicle,
    which_firstWheel,
    which_firstGear,
    which_soft,
    which_particle, // the lattice's second particle; the first is anchored
    which_anchor,
    which_pool,
    which_rail,
    which_edge, // the lattice's first edge
    which_jelly,
    which_tet, // the tet body's tet
    which_six,
} Which;

static int32_t SlotOf(const Scene* s, Which which)
{
    const m3World* w = s->world;
    switch (which)
    {
    case which_world:
        return 0;
    case which_ball:
        return s->ball;
    case which_ballShape:
        return w->bodies.bodyShapeHead[s->ball];
    case which_heroBody:
        return w->characters.charBody[s->hero];
    case which_hinge:
        return s->hinge;
    case which_gear:
        return s->gear;
    case which_pulley:
        return s->pulley;
    case which_servo:
        return s->servo;
    case which_wheel:
        return s->wheel;
    case which_hero:
        return s->hero;
    case which_vehicle:
        return s->vehicle;
    case which_firstWheel:
        return s->vehicle * M3_VEHICLE_MAX_WHEELS;
    case which_firstGear:
        return s->vehicle * M3_DRIVETRAIN_MAX_GEARS;
    case which_soft:
        return s->soft;
    case which_particle:
        return s->soft * M3_SOFTBODY_MAX_PARTICLES + 1;
    case which_anchor:
        return s->soft * M3_SOFTBODY_MAX_ANCHORS;
    case which_pool:
        return s->pool;
    case which_rail:
        return s->rail;
    case which_edge:
        return s->soft * M3_SOFTBODY_MAX_EDGES;
    case which_jelly:
        return s->jelly;
    case which_tet:
        return s->jelly * M3_SOFTBODY_MAX_TETS;
    case which_six:
        return s->six;
    }
    return 0;
}

// Where byte `at` of element `which` of the field at offset sits in a
// snapshot of the scene's world.
static uint8_t* At(const Scene* s, uint8_t* snap, size_t field, size_t width, Which which,
                   size_t at)
{
    int32_t offset = m3StateFieldAt(s->world, field);
    CHECK(offset >= 0, "the field is snapshot state");
    return snap + HEADER_BYTES + (offset < 0 ? 0 : offset) + width * (size_t)SlotOf(s, which) + at;
}

static void PutF32(uint8_t* at, float value)
{
    memcpy(at, &value, sizeof(value));
}

static void PutF64(uint8_t* at, double value)
{
    memcpy(at, &value, sizeof(value));
}

// One float, or one double for a position, written over the snapshot.
typedef struct Value
{
    size_t field;
    size_t width; // the element's size
    Which which;
    size_t at; // the byte within the element
    bool wide; // a double
} Value;

#define F(name)                     offsetof(m3World, name)
#define SCALAR(name, w)             {F(name), sizeof(float), (w), 0, false}
#define COMPONENT(name, type, w, c) {F(name), sizeof(type), (w), sizeof(float) * (size_t)(c), false}
#define POSITION(name, type, w, at) {F(name), sizeof(type), (w), (at), true}

typedef struct Case
{
    const char* name;
    Value value;
    double limit; // restores
    int toward;   // +1 when the values past the limit lie above it, -1 below
} Case;

// The next value past the limit, in the value's own precision.
static double Past(const Case* k)
{
    if (k->value.wide)
    {
        return nextafter(k->limit, k->limit + (double)k->toward * (fabs(k->limit) + 1.0));
    }
    float limit = (float)k->limit;
    return (double)nextafterf(limit, limit + (float)k->toward * (fabsf(limit) + 1.0f));
}

// clang-format off
static const Case s_cases[] = {
    // The world.
    {"gravity", COMPONENT(gravity, m3Vec3, which_world, 1), (double)(-M3_ACCEL_LIMIT), -1},
    {"contact hertz", SCALAR(contactHertz, which_world), (double)(M3_HERTZ_LIMIT), +1},
    {"contact damping", SCALAR(contactDampingRatio, which_world), (double)(M3_RATIO_LIMIT), +1},
    {"push speed", SCALAR(contactPushMaxSpeed, which_world), (double)(M3_SPEED_LIMIT), +1},
    {"restitution threshold", SCALAR(restitutionThreshold, which_world), (double)(0.0), -1},
    {"linear speed cap", SCALAR(maximumLinearSpeed, which_world), (double)(M3_SPEED_LIMIT), +1},
    {"angular speed cap", SCALAR(maximumAngularSpeed, which_world), (double)(M3_SPEED_LIMIT), +1},
    {"hit threshold", SCALAR(hitEventThreshold, which_world), (double)(0.0), -1},
    {"wind speed", SCALAR(windSpeed, which_world), (double)(M3_SPEED_LIMIT), +1},
    {"gust hertz", SCALAR(windGustHertz, which_world), (double)(M3_HERTZ_LIMIT), +1},
    {"gust scale", SCALAR(windGustScale, which_world), (double)(M3_RATIO_LIMIT), +1},
    {"gust phase", SCALAR(windPhase, which_world), (double)(4.0f * M3_PI), +1},
    // Bodies.
    {"dynamic velocity", COMPONENT(bodies.linearVelocities, m3Vec3, which_ball, 0), (double)(M3_SPEED_LIMIT), +1},
    {"kinematic velocity", COMPONENT(bodies.linearVelocities, m3Vec3, which_heroBody, 0), (double)(M3_KINEMATIC_SPEED_LIMIT), +1},
    {"spin", COMPONENT(bodies.angularVelocities, m3Vec3, which_ball, 2), (double)(-M3_SPEED_LIMIT), -1},
    {"gravity scale", SCALAR(bodies.gravityScales, which_ball), (double)(M3_RATIO_LIMIT), +1},
    {"linear damping", SCALAR(bodies.linearDamping, which_ball), (double)(0.0), -1},
    {"angular damping", SCALAR(bodies.angularDamping, which_ball), (double)(0.0), -1},
    {"held force", COMPONENT(bodies.bodyForce, m3Vec3, which_ball, 1), (double)(M3_FORCE_LIMIT), +1},
    {"held torque", COMPONENT(bodies.bodyTorque, m3Vec3, which_ball, 0), (double)(-M3_FORCE_LIMIT), -1},
    {"sleep threshold", SCALAR(bodies.bodySleepThreshold, which_ball), (double)(0.0), -1},
    {"sleep time", SCALAR(bodies.sleepTimes, which_ball), (double)(0.0), -1},
    // Shapes.
    {"density", SCALAR(shapes.shapeDensity, which_ballShape), (double)(M3_DENSITY_LIMIT), +1},
    {"friction", SCALAR(shapes.shapeFriction, which_ballShape), (double)(0.0), -1},
    {"restitution", SCALAR(shapes.shapeRestitution, which_ballShape), (double)(M3_RATIO_LIMIT), +1},
    {"rolling resistance", SCALAR(shapes.shapeRollingResistance, which_ballShape), (double)(0.0), -1},
    {"surface velocity", COMPONENT(shapes.shapeSurfaceVel, m3Vec3, which_ballShape, 0), (double)(M3_SPEED_LIMIT), +1},
    {"shape offset", COMPONENT(shapes.shapeLocalPos, m3Vec3, which_ballShape, 1), (double)(M3_LOCAL_LIMIT), +1},
    // Joints.
    {"anchor A", COMPONENT(joints.jointLocalA, m3Vec3, which_hinge, 0), (double)(M3_LOCAL_LIMIT), +1},
    {"anchor B", COMPONENT(joints.jointLocalB, m3Vec3, which_hinge, 2), (double)(-M3_LOCAL_LIMIT), -1},
    {"break force", COMPONENT(joints.jointBreak, m3Vec3, which_hinge, 0), (double)(0.0), -1},
    {"break torque", COMPONENT(joints.jointBreak, m3Vec3, which_hinge, 1), (double)(0.0), -1},
    {"motor speed", COMPONENT(joints.jointMotor, m3Vec3, which_hinge, 0), (double)(M3_SPEED_LIMIT), +1},
    {"motor effort", COMPONENT(joints.jointMotor, m3Vec3, which_hinge, 1), (double)(M3_FORCE_LIMIT), +1},
    {"spring hertz", COMPONENT(joints.jointSpring, m3Vec3, which_hinge, 0), (double)(M3_HERTZ_LIMIT), +1},
    {"gear ratio", COMPONENT(joints.jointMotor, m3Vec3, which_gear, 2), (double)(M3_RATIO_LIMIT), +1},
    {"gear ratio not zero", COMPONENT(joints.jointMotor, m3Vec3, which_gear, 2), (double)FLT_TRUE_MIN, -1},
    {"pulley ratio", COMPONENT(joints.jointMotor, m3Vec3, which_pulley, 2), (double)(M3_RATIO_LIMIT), +1},
    {"pulley ratio positive", COMPONENT(joints.jointMotor, m3Vec3, which_pulley, 2), (double)FLT_TRUE_MIN, -1},
    {"pulley ground A", POSITION(joints.jointGroundA, m3Pos3, which_pulley, 0), (double)(M3_POSITION_LIMIT), +1},
    {"pulley ground B", POSITION(joints.jointGroundB, m3Pos3, which_pulley, 8), (double)(-M3_POSITION_LIMIT), -1},
    {"servo force budget", COMPONENT(joints.jointLimits, m3Vec3, which_servo, 0), (double)(M3_FORCE_LIMIT), +1},
    {"servo torque budget", COMPONENT(joints.jointLimits, m3Vec3, which_servo, 1), (double)(M3_FORCE_LIMIT), +1},
    {"steer target", COMPONENT(joints.jointMotor, m3Vec3, which_wheel, 2), (double)(1.0), +1},
    {"steer hertz", COMPONENT(joints.jointTargetQ, m3Quat, which_wheel, 0), (double)(M3_HERTZ_LIMIT), +1},
    {"steer damping", COMPONENT(joints.jointTargetQ, m3Quat, which_wheel, 1), (double)(M3_RATIO_LIMIT), +1},
    {"steer effort", COMPONENT(joints.jointTargetQ, m3Quat, which_wheel, 2), (double)(M3_FORCE_LIMIT), +1},
    {"revolute target", SCALAR(joints.jointTargetScalar, which_hinge), (double)(M3_PI), +1},
    {"target on a gear", SCALAR(joints.jointTargetScalar, which_gear), (double)(0.0), +1},
    {"generic linear range", COMPONENT(joints.jointGenLinUpper, m3Vec3, which_six, 0), (double)(-0.5), -1},
    {"generic angular range", COMPONENT(joints.jointGenAngUpper, m3Vec3, which_six, 1), (double)(-0.25), -1},
    // Characters.
    {"character radius", SCALAR(characters.charRadius, which_hero), (double)(M3_LOCAL_LIMIT), +1},
    {"character radius positive", SCALAR(characters.charRadius, which_hero), (double)FLT_TRUE_MIN, -1},
    {"character half height", SCALAR(characters.charHalfHeight, which_hero), (double)(M3_LOCAL_LIMIT), +1},
    {"character mass", SCALAR(characters.charMass, which_hero), (double)(M3_MASS_LIMIT), +1},
    {"character push ratio", SCALAR(characters.charPushMax, which_hero), (double)(M3_RATIO_LIMIT), +1},
    {"slope cosine", SCALAR(characters.charCosSlope, which_hero), (double)(1.0), +1},
    {"slope cosine floor", SCALAR(characters.charCosSlope, which_hero), (double)(0.0), -1},
    // Vehicles.
    {"steer angle", SCALAR(vehicles.vehMaxSteer, which_vehicle), (double)(0.0), -1},
    {"drive force", SCALAR(vehicles.vehDriveForce, which_vehicle), (double)(M3_FORCE_LIMIT), +1},
    {"brake force", SCALAR(vehicles.vehBrakeForce, which_vehicle), (double)(M3_FORCE_LIMIT), +1},
    {"tire grip", SCALAR(vehicles.vehTireGrip, which_vehicle), (double)(M3_RATIO_LIMIT), +1},
    {"lean gain", SCALAR(vehicles.vehLeanGain, which_vehicle), (double)(M3_RATIO_LIMIT), +1},
    {"throttle", SCALAR(vehicles.vehThrottle, which_vehicle), (double)(1.0), +1},
    {"steering", SCALAR(vehicles.vehSteer, which_vehicle), (double)(-1.0), -1},
    {"brake", SCALAR(vehicles.vehBrake, which_vehicle), (double)(1.0), +1},
    {"left track", SCALAR(vehicles.vehTrackLeft, which_vehicle), (double)(1.0), +1},
    {"right track", SCALAR(vehicles.vehTrackRight, which_vehicle), (double)(-1.0), -1},
    {"wheel radius", SCALAR(vehicles.vehWheelRadius, which_firstWheel), (double)(M3_WHEEL_RADIUS_FLOOR), -1},
    {"wheel contact speed", SCALAR(vehicles.vehWheelLon, which_firstWheel), (double)(2.0f * M3_KINEMATIC_SPEED_LIMIT), +1},
    {"wheel compression", SCALAR(vehicles.vehWheelCompression, which_firstWheel), (double)(0.4f), +1},
    {"wheel compression floor", SCALAR(vehicles.vehWheelCompression, which_firstWheel), (double)(0.0), -1},
    {"gear ratio", SCALAR(vehicles.vehDtGearRatio, which_firstGear), (double)(M3_RATIO_LIMIT), +1},
    // Soft bodies.
    {"particle radius", SCALAR(softBodies.softRadius, which_soft), (double)(M3_LOCAL_LIMIT), +1},
    {"particle radius positive", SCALAR(softBodies.softRadius, which_soft), (double)FLT_TRUE_MIN, -1},
    {"particle gravity scale", SCALAR(softBodies.softGravityScale, which_soft), (double)(M3_RATIO_LIMIT), +1},
    {"compliance", SCALAR(softBodies.softCompliance, which_soft), (double)(0.0), -1},
    {"bend compliance", SCALAR(softBodies.softBendCompliance, which_soft), (double)(0.0), -1},
    {"shape deviation", SCALAR(softBodies.softMaxDeviation, which_soft), (double)(0.0), -1},
    {"pressure", SCALAR(softBodies.softPressure, which_soft), (double)(M3_SOFTBODY_MAX_PRESSURE), +1},
    {"particle mass floor", SCALAR(softBodies.softInvMass, which_particle), (double)(1.0f / M3_MASS_FLOOR), +1},
    {"particle mass limit", SCALAR(softBodies.softInvMass, which_particle), (double)(1.0f / M3_MASS_LIMIT), -1},
    {"particle kick", COMPONENT(softBodies.softKick, m3Vec3, which_particle, 0), (double)(M3_SPEED_LIMIT), +1},
    {"particle anchor", COMPONENT(softBodies.softAnchorLocal, m3Vec3, which_anchor, 1), (double)(M3_LOCAL_LIMIT), +1},
    {"edge rest length", SCALAR(softBodies.softEdgeRest, which_edge), (double)(M3_SOFT_EDGE_LIMIT), +1},
    {"edge rest length floor", SCALAR(softBodies.softEdgeRest, which_edge), (double)(0.0), -1},
    {"tet rest volume", SCALAR(softBodies.softTetRestV6, which_tet), (double)(M3_SOFT_TET_LIMIT), +1},
    {"tet rest volume below", SCALAR(softBodies.softTetRestV6, which_tet), (double)(-M3_SOFT_TET_LIMIT), -1},
    {"closed rest volume", SCALAR(softBodies.softRestVolume, which_soft), (double)(M3_SOFT_SHAPE_LIMIT), +1},
    {"closed rest volume floor", SCALAR(softBodies.softRestVolume, which_soft), (double)(0.0), -1},
    {"rest volume of a tet body", SCALAR(softBodies.softRestVolume, which_jelly), (double)(0.0), +1},
    // Positions, past which the step holds every body and particle.
    {"body position", POSITION(bodies.transforms, m3Transform, which_ball, 0), M3_REACH_LIMIT, +1},
    {"particle position", POSITION(softBodies.softPos, m3Pos3, which_particle, 8), -M3_REACH_LIMIT, -1},
    {"particle previous position", POSITION(softBodies.softPrev, m3Pos3, which_particle, 16), M3_REACH_LIMIT, +1},
    {"particle bind position", POSITION(softBodies.softBindPos, m3Pos3, which_particle, 0), M3_REACH_LIMIT, +1},
    // Water.
    {"water low corner", POSITION(water.waterLo, m3Pos3, which_pool, 0), (double)(-M3_POSITION_LIMIT), -1},
    {"water high corner", POSITION(water.waterHi, m3Pos3, which_pool, 16), (double)(M3_POSITION_LIMIT), +1},
    // The pool spans 4 to 6, 0 to 1 and 4 to 6 m: each high corner just
    // above its low one restores, one on it is refused.
    {"water extent x", POSITION(water.waterHi, m3Pos3, which_pool, 0), 4.0 + 0x1p-50, -1},
    {"water extent y", POSITION(water.waterHi, m3Pos3, which_pool, 8), 0x1p-1074, -1},
    {"water extent z", POSITION(water.waterHi, m3Pos3, which_pool, 16), 4.0 + 0x1p-50, -1},
    {"water density", SCALAR(water.waterDensity, which_pool), (double)(M3_DENSITY_LIMIT), +1},
    {"water linear drag", SCALAR(water.waterLinDrag, which_pool), (double)(0.0), -1},
    {"water angular drag", SCALAR(water.waterAngDrag, which_pool), (double)(0.0), -1},
    {"water flow", COMPONENT(water.waterFlow, m3Vec3, which_pool, 2), (double)(M3_SPEED_LIMIT), +1},
};
// clang-format on

static void Put(const Scene* s, uint8_t* snap, const Value* v, double x)
{
    uint8_t* at = At(s, snap, v->field, v->width, v->which, v->at);
    if (v->wide)
    {
        PutF64(at, x);
    }
    else
    {
        PutF32(at, (float)x);
    }
}

// The scene: a ball, a crate on a hinge, a gear, a pulley, a servo, a
// wheel and a generic joint, a kinematic rail, a character, a vehicle
// with a drivetrain, a closed lattice anchored to the crate, a lone
// particle, a tet body, a water volume and a steady wind.
static Scene Build(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.softBodyCapacity = 3;
    Scene s;
    memset(&s, 0, sizeof(s));
    s.world = NewWorld(&def);
    m3World* w = s.world;
    m3World_SetWind(w, (m3Vec3){1.0f, 0.0f, 0.0f}, 1.0f, 0.5f, 0.2f);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(w, &bd);
    (void)NewBoxShape(w, ground, &sd, (m3Vec3){50.0f, 1.0f, 50.0f});
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId crate = NewBody(w, &bd);
    (void)NewBoxShape(w, crate, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    bd.position = (m3Pos3){1.0, 1.5, 0.0};
    m3BodyId ball = NewBody(w, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    (void)NewSphereShape(w, ball, &sd, &sphere);
    bd.position = (m3Pos3){0.0, 0.5, 1.0};
    m3BodyId tire = NewBody(w, &bd);
    m3Sphere small = {{0.0f, 0.0f, 0.0f}, 0.2f};
    (void)NewSphereShape(w, tire, &sd, &small);
    s.ball = ball.index1 - 1;

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = ground;
    jd.bodyIdB = crate;
    jd.localAnchorA = (m3Vec3){0.0f, 2.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){0.0f, 0.5f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.enableMotor = true;
    jd.motorSpeed = 1.0f;
    jd.maxMotorEffort = 5.0f;
    s.hinge = NewJoint(w, &jd).index1 - 1;
    jd = m3DefaultJointDef();
    jd.type = m3_gearJoint;
    jd.bodyIdA = crate;
    jd.bodyIdB = ball;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.ratio = 2.0f;
    s.gear = NewJoint(w, &jd).index1 - 1;
    jd = m3DefaultJointDef();
    jd.type = m3_pulleyJoint;
    jd.bodyIdA = crate;
    jd.bodyIdB = ball;
    jd.groundAnchorA = (m3Pos3){0.0, 4.0, 0.0};
    jd.groundAnchorB = (m3Pos3){1.0, 4.0, 0.0};
    jd.ratio = 1.0f;
    s.pulley = NewJoint(w, &jd).index1 - 1;
    jd = m3DefaultJointDef();
    jd.type = m3_motorJoint;
    jd.bodyIdA = ground;
    jd.bodyIdB = ball;
    s.servo = NewJoint(w, &jd).index1 - 1;
    jd = m3DefaultJointDef();
    jd.type = m3_wheelJoint;
    jd.bodyIdA = crate;
    jd.bodyIdB = tire;
    jd.localAnchorA = (m3Vec3){0.0f, -0.5f, 1.0f};
    jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    m3JointId wheel = NewJoint(w, &jd);
    s.wheel = wheel.index1 - 1;
    m3Joint_SetSteer(w, wheel, true, 0.1f, 5.0f, 0.7f, 10.0f);
    jd = m3DefaultJointDef();
    jd.type = m3_genericJoint;
    jd.bodyIdA = ground;
    jd.bodyIdB = tire;
    jd.localAnchorA = (m3Vec3){0.0f, 0.5f, 1.0f};
    for (int32_t k = 0; k < 3; ++k)
    {
        jd.genericLinear[k] = (uint8_t)m3_axisFree;
        jd.genericAngular[k] = (uint8_t)m3_axisFree;
    }
    jd.genericLinear[0] = (uint8_t)m3_axisLimited;
    jd.genericLinearLower[0] = -0.5f;
    jd.genericLinearUpper[0] = 0.5f;
    jd.genericAngular[1] = (uint8_t)m3_axisLimited;
    jd.genericAngularLower[1] = -0.25f;
    jd.genericAngularUpper[1] = 0.25f;
    s.six = NewJoint(w, &jd).index1 - 1;

    bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.position = (m3Pos3){8.0, 1.0, 0.0};
    m3BodyId rail = NewBody(w, &bd);
    (void)NewBoxShape(w, rail, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    s.rail = rail.index1 - 1;

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.radius = 0.2f;
    cd.halfHeight = 0.3f;
    cd.position = (m3Pos3){-4.0, 1.0, -4.0};
    s.hero = NewCharacter(w, &cd).index1 - 1;

    bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, -6.0};
    m3BodyId chassis = NewBody(w, &bd);
    (void)NewBoxShape(w, chassis, &sd, (m3Vec3){0.8f, 0.2f, 0.4f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 4;
    for (int32_t k = 0; k < 4; ++k)
    {
        vd.wheels[k].anchor =
            (m3Vec3){(k & 1) != 0 ? 0.6f : -0.6f, -0.2f, (k & 2) != 0 ? 0.35f : -0.35f};
        vd.wheels[k].driven = true;
    }
    m3VehicleId car = NewVehicle(w, &vd);
    s.vehicle = car.index1 - 1;
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    m3Vehicle_SetDrivetrain(w, car, &dd);

    m3SoftBodyDef jelly = m3DefaultSoftBodyDef();
    jelly.position = (m3Pos3){-2.0, 2.0, 2.0};
    jelly.countX = 2;
    jelly.countY = 2;
    jelly.countZ = 2;
    jelly.spacing = 0.25f;
    jelly.radius = 0.06f;
    jelly.pressure = 1.0f;
    m3SoftBodyId soft = NewSoftBody(w, &jelly);
    s.soft = soft.index1 - 1;
    m3SoftBody_AnchorParticle(w, soft, 0, crate);
    m3SoftBodyDef single = m3DefaultSoftBodyDef();
    single.position = (m3Pos3){-4.0, 2.0, 4.0};
    single.countX = 1;
    single.countY = 1;
    single.countZ = 1;
    single.radius = 0.06f;
    s.dot = NewSoftBody(w, &single).index1 - 1;
    m3SoftBodyDef tetDef = m3DefaultSoftBodyDef();
    tetDef.position = (m3Pos3){-6.0, 2.0, 4.0};
    tetDef.radius = 0.05f;
    const m3Vec3 corners[4] = {
        {0.0f, 0.0f, 0.0f}, {0.4f, 0.0f, 0.0f}, {0.0f, 0.4f, 0.0f}, {0.0f, 0.0f, 0.4f}};
    const uint16_t tet[4] = {0, 1, 2, 3};
    s.jelly = NewSoftBodyTet(w, &tetDef, corners, 4, tet, 1).index1 - 1;

    m3WaterVolumeDef pool = m3DefaultWaterVolumeDef();
    pool.lo = (m3Pos3){4.0, 0.0, 4.0};
    pool.hi = (m3Pos3){6.0, 1.0, 6.0};
    m3WaterVolumeId volume = NewWaterVolume(w, &pool);
    s.pool = m3WaterVolume_IsValid(w, volume) ? (int32_t)volume.index1 - 1 : -1;
    for (int32_t i = 0; i < 4; ++i)
    {
        m3World_Step(w, 1.0f / 60.0f, 2);
    }
    return s;
}

static void TestBoundsAtTheirEdges(void)
{
    Scene s = Build();
    m3World* world = s.world;
    m3Counters counters = m3World_GetCounters(world);
    CHECK(counters.misuse == 0 && counters.jointCount == 6 && counters.vehicleCount == 1 &&
              counters.softBodyCount == 3 && counters.characterCount == 1 && s.pool >= 0 &&
              world->softBodies.softAnchorCount[s.soft] == 1 &&
              world->vehicles.vehDtActive[s.vehicle] == 1,
          "the scene builds");
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    uint8_t* edited = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && edited != NULL && Snapshot(world, clean, size) == size,
          "snapshot taken");
    if (clean == NULL || edited == NULL)
    {
        free(clean);
        free(edited);
        m3DestroyWorld(world);
        return;
    }
    uint64_t hash = m3World_Hash(world);
    for (size_t c = 0; c < sizeof(s_cases) / sizeof(s_cases[0]); ++c)
    {
        const Case* k = &s_cases[c];
        memcpy(edited, clean, (size_t)size);
        Put(&s, edited, &k->value, k->limit);
        bool atLimit = m3World_Restore(world, edited, size) == m3_success;
        CHECK(Restore(world, clean, size) && m3World_Hash(world) == hash, "back to the scene");
        memcpy(edited, clean, (size_t)size);
        Put(&s, edited, &k->value, Past(k));
        bool pastRefused = m3World_Restore(world, edited, size) == m3_errorInvalid;
        if (!atLimit || !pastRefused)
        {
            printf("  %s: at the limit %s, past it %s\n", k->name,
                   atLimit ? "restores" : "is refused", pastRefused ? "is refused" : "restores");
        }
        CHECK(atLimit, "a value at its limit restores");
        CHECK(pastRefused, "one just past it is refused");
        CHECK(m3World_Hash(world) == hash, "and the world is left as it was");
    }
    CHECK(m3World_GetCounters(world).misuse == 0, "no refusal counted as misuse");
    printf("restore bounds: %d values at and past their limits\n",
           (int)(sizeof(s_cases) / sizeof(s_cases[0])));
    free(edited);
    free(clean);
    m3DestroyWorld(world);
}

// A character's ground normal is none or a contact's unit normal: a unit
// one restores, one a tenth longer is refused with the world left as it
// was.
static void TestGroundNormalLength(void)
{
    Scene s = Build();
    m3World* world = s.world;
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    uint8_t* edited = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && edited != NULL && Snapshot(world, clean, size) == size,
          "snapshot taken");
    if (clean != NULL && edited != NULL)
    {
        uint64_t hash = m3World_Hash(world);
        const Value normal = COMPONENT(characters.charGroundNormal, m3Vec3, which_hero, 1);
        memcpy(edited, clean, (size_t)size);
        Put(&s, edited, &normal, 1.0);
        CHECK(Restore(world, edited, size), "a unit ground normal restores");
        CHECK(Restore(world, clean, size) && m3World_Hash(world) == hash, "back to the scene");
        memcpy(edited, clean, (size_t)size);
        Put(&s, edited, &normal, 1.1);
        CHECK(m3World_Restore(world, edited, size) == m3_errorInvalid, "a longer one is refused");
        CHECK(m3World_Hash(world) == hash && m3World_GetCounters(world).misuse == 0,
              "and the world is left as it was, with no misuse counted");
    }
    free(edited);
    free(clean);
    m3DestroyWorld(world);
}

// A kinematic body at the kinematic limit, a free lattice and a lone
// particle moving outward, all restored just inside the reach limit,
// cross it in one step: the step holds them within it, and the world it
// made snapshots and restores in a second world, which steps on alike.
static void TestStepHoldsWithinReach(void)
{
    Scene s = Build();
    m3World* world = s.world;
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "snapshot taken");
    if (snap == NULL)
    {
        m3DestroyWorld(world);
        return;
    }
    double start = M3_REACH_LIMIT - 1.0e9;
    PutF64(At(&s, snap, F(bodies.transforms), sizeof(m3Transform), which_rail, 0), start);
    PutF32(At(&s, snap, F(bodies.linearVelocities), sizeof(m3Vec3), which_rail, 0),
           M3_KINEMATIC_SPEED_LIMIT);
    int32_t particles = world->softBodies.softParticleCount[s.soft];
    for (int32_t i = 0; i < particles; ++i)
    {
        size_t at = sizeof(m3Pos3) * (size_t)(s.soft * M3_SOFTBODY_MAX_PARTICLES + i);
        PutF64(At(&s, snap, F(softBodies.softPos), 1, which_world, at), M3_REACH_LIMIT - 1024.0);
        PutF64(At(&s, snap, F(softBodies.softPrev), 1, which_world, at), M3_REACH_LIMIT - 1.0e4);
    }
    // The wind stills, so nothing slows the particles but the speed cap.
    PutF32(At(&s, snap, F(windSpeed), sizeof(float), which_world, 0), 0.0f);
    // A lone particle, free of any constraint, at the world's speed cap.
    // The spacing of doubles there is 128 m.
    size_t dot = sizeof(m3Pos3) * (size_t)(s.dot * M3_SOFTBODY_MAX_PARTICLES);
    PutF64(At(&s, snap, F(softBodies.softPos), 1, which_world, dot), M3_REACH_LIMIT - 1024.0);
    PutF64(At(&s, snap, F(softBodies.softPrev), 1, which_world, dot), M3_REACH_LIMIT - 1.0e4);
    // The lattice lets go of the crate, so it moves as a whole.
    uint8_t* anchors = At(&s, snap, F(softBodies.softAnchorCount), sizeof(int32_t), which_soft, 0);
    int32_t none = 0;
    memcpy(anchors, &none, sizeof(none));
    CHECK(Restore(world, snap, size), "a world just inside the reach limit restores");
    // A 10 s step: the particles at the world's speed cap cross too.
    m3World_Step(world, 10.0f, 2);
    double x = world->bodies.transforms[s.rail].p.x;
    CHECK(x <= M3_REACH_LIMIT && x > M3_REACH_LIMIT - 1.0e3,
          "the step holds the body on the limit");
    bool within = true;
    for (int32_t i = 0; i < particles; ++i)
    {
        within = within && world->softBodies.softPos[s.soft * M3_SOFTBODY_MAX_PARTICLES + i].x <=
                               M3_REACH_LIMIT;
    }
    CHECK(within, "every lattice particle stays within it");
    double dotX = world->softBodies.softPos[s.dot * M3_SOFTBODY_MAX_PARTICLES].x;
    CHECK(dotX <= M3_REACH_LIMIT && dotX > M3_REACH_LIMIT - 1.0e3,
          "and the lone particle is held on it");

    Scene twin = Build();
    CHECK(Snapshot(world, snap, size) == size && Restore(twin.world, snap, size),
          "the world the step made restores in a second world");
    CHECK(m3World_Hash(twin.world) == m3World_Hash(world), "to the same state");
    m3World_Step(world, 1.0f, 2);
    m3World_Step(twin.world, 1.0f, 2);
    CHECK(m3World_Hash(twin.world) == m3World_Hash(world), "and both step on alike");
    CHECK(Snapshot(world, snap, size) == size && Restore(twin.world, snap, size),
          "held on the limit, it restores again");
    free(snap);
    m3DestroyWorld(twin.world);
    m3DestroyWorld(world);
}

// The hold's scale alone can round a component a hair past the bound for
// positions a character's repeated moves reach; the clamp after it keeps
// every one within, the direction kept.
static void TestHoldRoundsWithin(void)
{
    const double past[] = {1.9560342718892493e+18, 1.8354988781294495e+18, 1.581204017112003e+18};
    bool within = true;
    for (size_t i = 0; i < sizeof(past) / sizeof(past[0]); ++i)
    {
        m3Pos3 held = m3HoldReach((m3Pos3){past[i], -0.5 * past[i], 0.0});
        within = within && held.x <= M3_REACH_LIMIT && held.x > M3_REACH_LIMIT - 1.0e3 &&
                 fabs(held.y + 0.5 * held.x) <= 1.0e3 && held.z == 0.0;
    }
    CHECK(within, "a held position lands within the bound, direction kept");
    m3Pos3 inside = {M3_REACH_LIMIT, -M3_REACH_LIMIT, 1.0};
    m3Pos3 kept = m3HoldReach(inside);
    CHECK(memcmp(&kept, &inside, sizeof(kept)) == 0, "one within it is unchanged");
}

// A servo target within reach of its kinematic body restores; one just
// past it is refused.
static void TestTargetReach(void)
{
    Scene s = Build();
    m3World* world = s.world;
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    uint8_t* edited = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && edited != NULL && Snapshot(world, clean, size) == size,
          "snapshot taken");
    if (clean == NULL || edited == NULL)
    {
        free(clean);
        free(edited);
        m3DestroyWorld(world);
        return;
    }
    uint64_t hash = m3World_Hash(world);
    const double reach[2] = {M3_SERVO_REACH, nextafter(M3_SERVO_REACH, 2.0 * M3_SERVO_REACH)};
    bool restored[2];
    for (int32_t k = 0; k < 2; ++k)
    {
        memcpy(edited, clean, (size_t)size);
        m3Transform target = world->bodies.transforms[s.rail];
        target.p.x += reach[k];
        *At(&s, edited, F(bodies.bodyHasTarget), 1, which_rail, 0) = 1;
        memcpy(At(&s, edited, F(bodies.bodyTarget), sizeof(m3Transform), which_rail, 0), &target,
               sizeof(target));
        restored[k] = m3World_Restore(world, edited, size) == m3_success;
        CHECK(Restore(world, clean, size) && m3World_Hash(world) == hash, "back to the scene");
    }
    CHECK(restored[0], "a servo target at its reach restores");
    CHECK(!restored[1], "one just past it is refused");
    free(edited);
    free(clean);
    m3DestroyWorld(world);
}

// Twelve boxes at the size and density limits and a sphere of radius 1
// make a body of 9.6e19 kg; the sphere grown to 1e4 m takes it past the
// mass limit, which restore refuses as the create would.
static void TestBodyMassLimit(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3World* world = NewWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId heavy = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = M3_DENSITY_LIMIT;
    for (int32_t i = 0; i < 12; ++i)
    {
        (void)NewBoxShape(world, heavy, &sd,
                          (m3Vec3){M3_LOCAL_LIMIT, M3_LOCAL_LIMIT, M3_LOCAL_LIMIT});
    }
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    m3ShapeId ball = NewSphereShape(world, heavy, &sd, &sphere);
    CHECK(m3Shape_IsValid(world, ball), "the body builds below the mass limit");
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "snapshot taken");
    if (snap == NULL)
    {
        m3DestroyWorld(world);
        return;
    }
    CHECK(Restore(world, snap, size), "it restores");
    uint64_t hash = m3World_Hash(world);
    int32_t at = m3StateFieldAt(world, F(shapes.shapeGeom));
    float grown = M3_LOCAL_LIMIT;
    memcpy(snap + HEADER_BYTES + at + sizeof(m3ShapeGeom) * (size_t)(ball.index1 - 1) +
               offsetof(m3ShapeGeom, s),
           &grown, sizeof(grown));
    CHECK(m3World_Restore(world, snap, size) == m3_errorInvalid,
          "the sphere grown past the mass limit is refused");
    CHECK(m3World_Hash(world) == hash, "and the world is left as it was");
    free(snap);
    m3DestroyWorld(world);
}

// A body turned kinematic has no mass, as one created kinematic, and its
// snapshot restores to the same state.
static void TestTypeChangeMass(void)
{
    Scene s = Build();
    m3World* world = s.world;
    m3BodyId ball = {(uint32_t)s.ball + 1u, world->bodies.bodyPool.generations[s.ball]};
    m3Body_SetType(world, ball, m3_kinematicBody);
    CHECK(m3Body_GetMass(world, ball) == 0.0f, "a body turned kinematic has no mass");
    Scene twin = Build();
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size && Restore(twin.world, snap, size),
          "its world restores in a second world");
    CHECK(m3World_Hash(twin.world) == m3World_Hash(world), "to the same state");
    free(snap);
    m3DestroyWorld(twin.world);
    m3DestroyWorld(world);
}

int main(void)
{
    TestBoundsAtTheirEdges();
    TestGroundNormalLength();
    TestStepHoldsWithinReach();
    TestHoldRoundsWithin();
    TestTargetReach();
    TestBodyMassLimit();
    TestTypeChangeMass();
    if (s_failures == 0)
    {
        printf("test_restore_bounds: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
