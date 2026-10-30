// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The magnitude bounds the API and replay share (src/core.h). Every site
// is driven three ways: its public call accepts a value at the limit; it
// refuses a value just past the limit, counts the refusal once and
// changes nothing; and a journal recorded at the limit, with that one
// value changed to just past it, fails to replay while the unchanged
// journal replays. Replay asks the state a call asks as well.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/character.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "maul3d/vehicle.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The next float toward +inf and toward -inf, for positive x.
static float Up(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    bits += 1u;
    memcpy(&x, &bits, sizeof(x));
    return x;
}

static float Down(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    bits -= 1u;
    memcpy(&x, &bits, sizeof(x));
    return x;
}

typedef struct Scene
{
    m3World* world;
    m3BodyId ground;  // static, a plane
    m3BodyId box;     // dynamic unit cube, 1 kg, at (0, 2, 0)
    m3BodyId arm;     // dynamic sphere, hinged to the ground
    m3BodyId puck;    // dynamic sphere, held by the servo
    m3BodyId cart;    // dynamic box, the wheel joint's chassis
    m3BodyId tire;    // dynamic sphere on the wheel joint
    m3BodyId chassis; // dynamic box, the vehicle's
    m3BodyId kin;     // kinematic sphere at (0, 5, 0)
    m3BodyId spare;   // a site's own body, made in its prepare step
    m3ShapeId boxShape;
    m3ShapeId armShape;
    m3ShapeId ballShape; // the kinematic sphere
    m3ShapeId rodShape;  // a capsule on the ground
    m3JointId hinge;
    m3JointId rope;  // a distance joint with its spring on
    m3JointId servo; // a motor joint
    m3JointId wheel;
    m3JointId socket; // a site's spherical joint, made in its prepare step
    m3VehicleId car;
    m3CharacterId walker;
} Scene;

static m3WorldDef SceneDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 48;
    def.jointCapacity = 16;
    return def;
}

static void BuildScene(Scene* s)
{
    m3WorldDef def = SceneDef();
    m3World* w = NewWorld(&def);
    s->world = w;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    s->ground = NewBody(w, &bd);
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(w, s->ground, &sd, &floor);

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    s->box = NewBody(w, &bd);
    s->boxShape = NewBoxShape(w, s->box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    bd.position = (m3Pos3){3.0, 2.0, 0.0};
    s->arm = NewBody(w, &bd);
    s->armShape = NewSphereShape(w, s->arm, &sd, &ball);
    bd.position = (m3Pos3){-3.0, 2.0, 0.0};
    s->puck = NewBody(w, &bd);
    NewSphereShape(w, s->puck, &sd, &ball);
    bd.position = (m3Pos3){0.0, 2.0, 6.0};
    s->cart = NewBody(w, &bd);
    NewBoxShape(w, s->cart, &sd, (m3Vec3){1.0f, 0.25f, 2.0f});
    bd.position = (m3Pos3){1.5, 1.5, 6.0};
    s->tire = NewBody(w, &bd);
    NewSphereShape(w, s->tire, &sd, &ball);
    bd.position = (m3Pos3){0.0, 2.0, -6.0};
    s->chassis = NewBody(w, &bd);
    NewBoxShape(w, s->chassis, &sd, (m3Vec3){1.0f, 0.25f, 2.0f});
    bd.type = m3_kinematicBody;
    bd.position = (m3Pos3){0.0, 5.0, 0.0};
    s->kin = NewBody(w, &bd);
    s->ballShape = NewSphereShape(w, s->kin, &sd, &ball);
    m3Capsule rod = {{0.0f, 0.0f, -8.0f}, {1.0f, 0.0f, -8.0f}, 0.5f};
    s->rodShape = NewCapsuleShape(w, s->ground, &sd, &rod);
    s->spare = m3_nullBodyId;
    s->socket = m3_nullJointId;

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = s->ground;
    jd.bodyIdB = s->arm;
    jd.localAnchorA = (m3Vec3){3.0f, 2.0f, 0.0f};
    s->hinge = NewJoint(w, &jd);
    jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = s->box;
    jd.bodyIdB = s->arm;
    jd.maxLength = 5.0f;
    jd.length = 2.0f;
    jd.enableSpring = true;
    jd.springHertz = 2.0f;
    jd.springDampingRatio = 0.5f;
    s->rope = NewJoint(w, &jd);
    jd = m3DefaultJointDef();
    jd.type = m3_motorJoint;
    jd.bodyIdA = s->ground;
    jd.bodyIdB = s->puck;
    jd.localAnchorA = (m3Vec3){-3.0f, 2.0f, 0.0f};
    s->servo = NewJoint(w, &jd);
    jd = m3DefaultJointDef();
    jd.type = m3_wheelJoint;
    jd.bodyIdA = s->cart;
    jd.bodyIdB = s->tire;
    jd.localAnchorA = (m3Vec3){1.5f, -0.5f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f};
    s->wheel = NewJoint(w, &jd);

    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = s->chassis;
    vd.wheelCount = 1;
    s->car = NewVehicle(w, &vd);
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){10.0, 1.0, 0.0};
    s->walker = NewCharacter(w, &cd);
}

static bool SceneComplete(const Scene* s)
{
    return m3Shape_IsValid(s->world, s->rodShape) && m3Joint_IsValid(s->world, s->hinge) &&
           m3Joint_IsValid(s->world, s->rope) && m3Joint_IsValid(s->world, s->servo) &&
           m3Joint_IsValid(s->world, s->wheel) && m3Vehicle_IsValid(s->world, s->car) &&
           m3Character_IsValid(s->world, s->walker);
}

static uint64_t Misuse(const m3World* world)
{
    return m3World_GetCounters(world).misuseCount;
}

static const m3Quat s_identity = {0.0f, 0.0f, 0.0f, 1.0f};

// ---- World ----

static void Gravity(Scene* s, float v)
{
    m3World_SetGravity(s->world, (m3Vec3){v, 0.0f, 0.0f});
}

static void ContactHertz(Scene* s, float v)
{
    m3World_SetContactTuning(s->world, v, 1.0f, 3.0f);
}

static void ContactRatio(Scene* s, float v)
{
    m3World_SetContactTuning(s->world, 30.0f, v, 3.0f);
}

static void ContactPush(Scene* s, float v)
{
    m3World_SetContactTuning(s->world, 30.0f, 1.0f, v);
}

static void LinearCap(Scene* s, float v)
{
    m3World_SetMaximumLinearSpeed(s->world, v);
}

static void AngularCap(Scene* s, float v)
{
    m3World_SetMaximumAngularSpeed(s->world, v);
}

static void WindSpeed(Scene* s, float v)
{
    m3World_SetWind(s->world, (m3Vec3){1.0f, 0.0f, 0.0f}, v, 0.0f, 0.0f);
}

static void WindHertz(Scene* s, float v)
{
    m3World_SetWind(s->world, (m3Vec3){1.0f, 0.0f, 0.0f}, 1.0f, v, 0.5f);
}

static void WindScale(Scene* s, float v)
{
    m3World_SetWind(s->world, (m3Vec3){1.0f, 0.0f, 0.0f}, 1.0f, 2.0f, v);
}

static void WindDirection(Scene* s, float v)
{
    m3World_SetWind(s->world, (m3Vec3){v, 0.0f, 0.0f}, 1.0f, 0.0f, 0.0f);
}

static void Step(Scene* s, float v)
{
    (void)m3World_Step(s->world, v, 1);
}

// ---- Bodies ----

static void DynamicVelocity(Scene* s, float v)
{
    m3Body_SetLinearVelocity(s->world, s->box, (m3Vec3){v, 0.0f, 0.0f});
}

static void DynamicSpin(Scene* s, float v)
{
    m3Body_SetAngularVelocity(s->world, s->box, (m3Vec3){0.0f, v, 0.0f});
}

static void KinematicVelocity(Scene* s, float v)
{
    m3Body_SetLinearVelocity(s->world, s->kin, (m3Vec3){v, 0.0f, 0.0f});
}

static void KinematicSpin(Scene* s, float v)
{
    m3Body_SetAngularVelocity(s->world, s->kin, (m3Vec3){0.0f, 0.0f, v});
}

static void DefVelocity(Scene* s, float v)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.linearVelocity = (m3Vec3){0.0f, v, 0.0f};
    NewBody(s->world, &bd);
}

static void DefKinematicSpin(Scene* s, float v)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.angularVelocity = (m3Vec3){v, 0.0f, 0.0f};
    NewBody(s->world, &bd);
}

static void DefPosition(Scene* s, float v)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){(double)v, 0.0, 0.0};
    NewBody(s->world, &bd);
}

static void Teleport(Scene* s, float v)
{
    m3Body_SetTransform(s->world, s->box, (m3Pos3){0.0, 2.0, (double)v}, s_identity);
}

static void DefKinematicVelocity(Scene* s, float v)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.linearVelocity = (m3Vec3){0.0f, 0.0f, v};
    NewBody(s->world, &bd);
}

static void DefGravityScale(Scene* s, float v)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.gravityScale = v;
    NewBody(s->world, &bd);
}

static void GravityScale(Scene* s, float v)
{
    m3Body_SetGravityScale(s->world, s->box, v);
}

static void Force(Scene* s, float v)
{
    m3Body_ApplyForce(s->world, s->box, (m3Vec3){v, 0.0f, 0.0f});
}

static void HeldForce(Scene* s, float v)
{
    (void)v;
    m3Body_ApplyForce(s->world, s->box, (m3Vec3){0.0f, 0.0f, 6.0e19f});
}

static void AddedForce(Scene* s, float v)
{
    m3Body_ApplyForce(s->world, s->box, (m3Vec3){0.0f, 0.0f, v});
}

static void Torque(Scene* s, float v)
{
    m3Body_ApplyTorque(s->world, s->box, (m3Vec3){0.0f, v, 0.0f});
}

// At the center the force alone is bounded; two meters out along x, a
// force along y gives a torque about z of twice it.
static void CenterForce(Scene* s, float v)
{
    m3Body_ApplyForceAtPoint(s->world, s->box, (m3Vec3){0.0f, v, 0.0f}, (m3Pos3){0.0, 2.0, 0.0});
}

static void LeverForce(Scene* s, float v)
{
    m3Body_ApplyForceAtPoint(s->world, s->box, (m3Vec3){0.0f, v, 0.0f}, (m3Pos3){2.0, 2.0, 0.0});
}

// The box weighs 1 kg, so an impulse is the velocity it leaves; its
// inverse inertia is about 6, so an angular impulse leaves six times
// itself.
static void LinearImpulse(Scene* s, float v)
{
    m3Body_ApplyLinearImpulse(s->world, s->box, (m3Vec3){0.0f, 0.0f, v});
}

static void AngularImpulse(Scene* s, float v)
{
    m3Body_ApplyAngularImpulse(s->world, s->box, (m3Vec3){v, 0.0f, 0.0f});
}

static void CenterImpulse(Scene* s, float v)
{
    m3Body_ApplyLinearImpulseAtPoint(s->world, s->box, (m3Vec3){0.0f, v, 0.0f},
                                     (m3Pos3){0.0, 2.0, 0.0});
}

static void LeverImpulse(Scene* s, float v)
{
    m3Body_ApplyLinearImpulseAtPoint(s->world, s->box, (m3Vec3){0.0f, v, 0.0f},
                                     (m3Pos3){1.0, 2.0, 0.0});
}

static void Target(Scene* s, float v)
{
    m3Body_SetTargetTransform(s->world, s->kin, (m3Pos3){(double)v, 5.0, 0.0}, s_identity);
}

static void SleepThreshold(Scene* s, float v)
{
    m3Body_SetSleepThreshold(s->world, s->box, v);
}

static void SpeedKinematic(Scene* s, float v)
{
    m3Body_SetLinearVelocity(s->world, s->kin, (m3Vec3){0.0f, 0.0f, v});
}

static void TurnDynamic(Scene* s, float v)
{
    (void)v;
    m3Body_SetType(s->world, s->kin, m3_dynamicBody);
}

static void SpeckKinematic(Scene* s, float v)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.position = (m3Pos3){5.0, 5.0, 5.0};
    s->spare = NewBody(s->world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere speck = {{0.0f, 0.0f, 0.0f}, v};
    NewSphereShape(s->world, s->spare, &sd, &speck);
}

static void SpareDynamic(Scene* s, float v)
{
    (void)v;
    m3Body_SetType(s->world, s->spare, m3_dynamicBody);
}

// ---- Shapes ----

static void EmptyDynamic(Scene* s, float v)
{
    (void)v;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){5.0, 5.0, -5.0};
    s->spare = NewBody(s->world, &bd);
}

// Density 1: a sphere reaches the mass floor at a radius of about
// 6.2e-3 m, with its inertia ten times the floor.
static void LightSphere(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, v};
    NewSphereShape(s->world, s->spare, &sd, &sphere);
}

// Density 1e5: a sphere reaches the inertia floor at a radius of about
// 3.6e-4 m, with its mass ten times the floor.
static void ThinSphere(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 1.0e5f;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, v};
    NewSphereShape(s->world, s->spare, &sd, &sphere);
}

static void Reshape(Scene* s, float v)
{
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, v};
    m3Shape_SetSphere(s->world, s->armShape, &sphere);
}

// The box's unit volume makes its density its mass.
static void Density(Scene* s, float v)
{
    m3Shape_SetDensity(s->world, s->boxShape, v);
}

// Twelve boxes of 8e18 kg leave 4e18 kg of the body's mass limit, a
// thirteenth box at density 5e5.
static void HeavyBody(Scene* s, float v)
{
    EmptyDynamic(s, v);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 1.0e6f;
    for (int32_t i = 0; i < 12; ++i)
    {
        NewBoxShape(s->world, s->spare, &sd, (m3Vec3){1.0e4f, 1.0e4f, 1.0e4f});
    }
}

static void HeavyBox(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = v;
    NewBoxShape(s->world, s->spare, &sd, (m3Vec3){1.0e4f, 1.0e4f, 1.0e4f});
}

static void DefDensity(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = v;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    NewSphereShape(s->world, s->box, &sd, &sphere);
}

static void ReshapeRadius(Scene* s, float v)
{
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, v};
    m3Shape_SetSphere(s->world, s->ballShape, &sphere);
}

static void ReshapeCapsule(Scene* s, float v)
{
    m3Capsule capsule = {{0.0f, 0.0f, 0.0f}, {v, 0.0f, 0.0f}, 0.5f};
    m3Shape_SetCapsule(s->world, s->rodShape, &capsule);
}

static void Restitution(Scene* s, float v)
{
    m3Shape_SetRestitution(s->world, s->boxShape, v);
}

static void SurfaceVelocity(Scene* s, float v)
{
    m3Shape_SetSurfaceVelocity(s->world, s->boxShape, (m3Vec3){0.0f, 0.0f, v});
}

static void DefRestitution(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.restitution = v;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(s->world, s->ground, &sd, &sphere);
}

static void DefSurfaceVelocity(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.surfaceVelocity = (m3Vec3){v, 0.0f, 0.0f};
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    NewSphereShape(s->world, s->ground, &sd, &sphere);
}

static void SphereRadius(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, v};
    NewSphereShape(s->world, s->ground, &sd, &sphere);
}

static void SphereCenter(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, v, 0.0f}, 1.0f};
    NewSphereShape(s->world, s->ground, &sd, &sphere);
}

static void CapsuleRadius(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, v};
    NewCapsuleShape(s->world, s->ground, &sd, &capsule);
}

static void CapsulePoint(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, v}, 1.0f};
    NewCapsuleShape(s->world, s->ground, &sd, &capsule);
}

static void BoxExtent(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    NewBoxShape(s->world, s->ground, &sd, (m3Vec3){1.0f, v, 1.0f});
}

// One point of a hull cloud on the ground, v along z; the others keep the
// cloud from being too thin to build.
static void HullPoint(Scene* s, float v)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Vec3 cloud[4] = {
        {0.0f, 0.0f, 0.0f}, {4000.0f, 0.0f, 0.0f}, {0.0f, 4000.0f, 0.0f}, {0.0f, 0.0f, v}};
    NewHullShape(s->world, s->ground, &sd, cloud, 4);
}

// ---- Joints ----

static void MotorSpeed(Scene* s, float v)
{
    m3Joint_SetMotorSpeed(s->world, s->hinge, v);
}

static void MotorEffort(Scene* s, float v)
{
    m3Joint_SetMaxMotorEffort(s->world, s->hinge, v);
}

// A distance joint's spring is the joint spring part.
static void RopeHertz(Scene* s, float v)
{
    m3Joint_SetSpringHertz(s->world, s->rope, v);
}

static void RopeRatio(Scene* s, float v)
{
    m3Joint_SetSpringDampingRatio(s->world, s->rope, v);
}

static void SpringHertz(Scene* s, float v)
{
    m3Joint_SetSpringHertz(s->world, s->hinge, v);
}

static void SpringRatio(Scene* s, float v)
{
    m3Joint_SetSpringDampingRatio(s->world, s->hinge, v);
}

static void SteerHertz(Scene* s, float v)
{
    m3WheelJoint_SetSteeringHertz(s->world, s->wheel, v);
}

static void SteerRatio(Scene* s, float v)
{
    m3WheelJoint_SetSteeringDampingRatio(s->world, s->wheel, v);
}

static void SteerTorque(Scene* s, float v)
{
    m3WheelJoint_SetMaxSteeringTorque(s->world, s->wheel, v);
}

static void ServoAim(Scene* s, float v)
{
    m3MotorJoint_SetOffsets(s->world, s->servo, (m3Vec3){0.0f, v, 0.0f}, s_identity);
}

static void HingeTarget(Scene* s, float v)
{
    m3RevoluteJoint_SetTargetAngle(s->world, s->hinge, v);
}

static void Socket(Scene* s, float v)
{
    (void)v;
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyIdA = s->ground;
    jd.bodyIdB = s->box;
    jd.localAnchorA = (m3Vec3){0.0f, 2.0f, 0.0f};
    s->socket = NewJoint(s->world, &jd);
}

// A rotation's w beside z = 0.6: its squared length leaves the unit
// band, [0.99, 1.01], between 0.806 and 0.807.
static void SocketTarget(Scene* s, float v)
{
    m3SphericalJoint_SetTargetRotation(s->world, s->socket, (m3Quat){0.0f, 0.0f, 0.6f, v});
}

static void ServoForce(Scene* s, float v)
{
    m3MotorJoint_SetMaxForce(s->world, s->servo, v);
}

static void ServoTorque(Scene* s, float v)
{
    m3MotorJoint_SetMaxTorque(s->world, s->servo, v);
}

static void DefMotorSpeed(Scene* s, float v)
{
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = s->ground;
    jd.bodyIdB = s->puck;
    jd.localAnchorA = (m3Vec3){-3.0f, 2.0f, 0.0f};
    jd.enableMotor = true;
    jd.motorSpeed = v;
    jd.maxMotorEffort = 10.0f;
    NewJoint(s->world, &jd);
}

static void DefRopeRatio(Scene* s, float v)
{
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = s->box;
    jd.bodyIdB = s->puck;
    jd.maxLength = 5.0f;
    jd.enableSpring = true;
    jd.springHertz = 2.0f;
    jd.springDampingRatio = v;
    NewJoint(s->world, &jd);
}

static void DefServoForce(Scene* s, float v)
{
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_motorJoint;
    jd.bodyIdA = s->ground;
    jd.bodyIdB = s->box;
    jd.localAnchorA = (m3Vec3){0.0f, 2.0f, 0.0f};
    jd.maxForce = v;
    NewJoint(s->world, &jd);
}

static void GearRatio(Scene* s, float v)
{
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_gearJoint;
    jd.bodyIdA = s->arm;
    jd.bodyIdB = s->puck;
    jd.ratio = v;
    NewJoint(s->world, &jd);
}

static void PulleyRatio(Scene* s, float v)
{
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_pulleyJoint;
    jd.bodyIdA = s->box;
    jd.bodyIdB = s->puck;
    jd.groundAnchorA = (m3Pos3){0.0, 6.0, 0.0};
    jd.groundAnchorB = (m3Pos3){-3.0, 6.0, 0.0};
    jd.ratio = v;
    NewJoint(s->world, &jd);
}

// ---- Vehicles ----

static m3VehicleDef CarDef(const Scene* s)
{
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = s->cart;
    vd.wheelCount = 1;
    return vd;
}

static void WheelRadius(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.wheels[0].radius = v;
    NewVehicle(s->world, &vd);
}

static void WheelHertz(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.wheels[0].springHertz = v;
    NewVehicle(s->world, &vd);
}

static void WheelRatio(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.wheels[0].springDampingRatio = v;
    NewVehicle(s->world, &vd);
}

static void WheelAnchor(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.wheels[0].anchor = (m3Vec3){v, 0.0f, 0.0f};
    NewVehicle(s->world, &vd);
}

static void WheelRest(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.wheels[0].restLength = v;
    NewVehicle(s->world, &vd);
}

static void WheelBrake(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.wheels[0].brakeShare = v;
    NewVehicle(s->world, &vd);
}

static void DriveForce(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.driveForce = v;
    NewVehicle(s->world, &vd);
}

static void BrakeForce(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.brakeForce = v;
    NewVehicle(s->world, &vd);
}

static void TireGrip(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.tireGrip = v;
    NewVehicle(s->world, &vd);
}

static void Lean(Scene* s, float v)
{
    m3VehicleDef vd = CarDef(s);
    vd.leanStabilization = v;
    NewVehicle(s->world, &vd);
}

static void CurveTorque(Scene* s, float v)
{
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    dd.curveTorque[1] = v;
    m3Vehicle_SetDrivetrain(s->world, s->car, &dd);
}

static void GearStep(Scene* s, float v)
{
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    dd.gearRatio[0] = v;
    m3Vehicle_SetDrivetrain(s->world, s->car, &dd);
}

static void FinalDrive(Scene* s, float v)
{
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    dd.finalDrive = v;
    m3Vehicle_SetDrivetrain(s->world, s->car, &dd);
}

static void ReverseRatio(Scene* s, float v)
{
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    dd.reverseRatio = v;
    m3Vehicle_SetDrivetrain(s->world, s->car, &dd);
}

static void DiffCouple(Scene* s, float v)
{
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    dd.diffMode = 2;
    dd.diffCouple = v;
    m3Vehicle_SetDrivetrain(s->world, s->car, &dd);
}

// ---- Characters, soft bodies, water, explosions ----

static m3CharacterDef WalkerDef(void)
{
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){20.0, 1.0, 0.0};
    return cd;
}

static void WalkerPosition(Scene* s, float v)
{
    m3CharacterDef cd = WalkerDef();
    cd.position = (m3Pos3){(double)v, 1.0, 0.0};
    NewCharacter(s->world, &cd);
}

static void WalkerMass(Scene* s, float v)
{
    m3CharacterDef cd = WalkerDef();
    cd.mass = v;
    NewCharacter(s->world, &cd);
}

static void WalkerPush(Scene* s, float v)
{
    m3CharacterDef cd = WalkerDef();
    cd.pushMaxMassRatio = v;
    NewCharacter(s->world, &cd);
}

static void WalkerRadius(Scene* s, float v)
{
    m3CharacterDef cd = WalkerDef();
    cd.radius = v;
    NewCharacter(s->world, &cd);
}

static void WalkerHeight(Scene* s, float v)
{
    m3CharacterDef cd = WalkerDef();
    cd.halfHeight = v;
    NewCharacter(s->world, &cd);
}

static void Walk(Scene* s, float v)
{
    m3Character_Move(s->world, s->walker, (m3Vec3){0.0f, 0.0f, v});
}

static void ParticleMass(Scene* s, float v)
{
    m3SoftBodyDef def = m3DefaultSoftBodyDef();
    def.particleMass = v;
    NewSoftBody(s->world, &def);
}

static void LatticeSpacing(Scene* s, float v)
{
    m3SoftBodyDef def = m3DefaultSoftBodyDef();
    def.spacing = v;
    NewSoftBody(s->world, &def);
}

static void ParticleRadius(Scene* s, float v)
{
    m3SoftBodyDef def = m3DefaultSoftBodyDef();
    def.radius = v;
    NewSoftBody(s->world, &def);
}

static void ParticleGravity(Scene* s, float v)
{
    m3SoftBodyDef def = m3DefaultSoftBodyDef();
    def.gravityScale = v;
    NewSoftBody(s->world, &def);
}

static void WaterDensity(Scene* s, float v)
{
    m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
    def.density = v;
    NewWaterVolume(s->world, &def);
}

static void WaterCorner(Scene* s, float v)
{
    m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
    def.bounds.upperBound = (m3Pos3){1.0, 1.0, (double)v};
    NewWaterVolume(s->world, &def);
}

static void WaterFlow(Scene* s, float v)
{
    m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
    def.flow = (m3Vec3){0.0f, 0.0f, v};
    NewWaterVolume(s->world, &def);
}

static void Blast(Scene* s, float v)
{
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 1.0, 0.0};
    def.impulsePerArea = v;
    m3World_Explode(s->world, &def);
}

static void BlastPosition(Scene* s, float v)
{
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, (double)v, 0.0};
    def.impulsePerArea = 1.0f;
    m3World_Explode(s->world, &def);
}

static void BlastSoftPush(Scene* s, float v)
{
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 1.0, 0.0};
    def.impulsePerArea = 1.0f;
    def.softPush = v;
    m3World_Explode(s->world, &def);
}

// ---- The driver ----

typedef void SiteFunction(Scene* s, float value);

typedef struct Site
{
    const char* name;
    SiteFunction* prepare; // earlier calls of the site, or NULL; they accept both values
    SiteFunction* apply;   // the call the limit refuses
    float ok;              // at the limit, or just inside it where rounding sits
    float past;            // just past the limit
    int32_t width;         // how the journal stores the value: 4 a float, 8 a double
} Site;

static uint8_t s_journal[1 << 20];

// Replaces the one occurrence of from's bytes in the journal with to's.
static bool Patch(int64_t bytes, float from, float to, int32_t width)
{
    uint8_t a[8];
    uint8_t b[8];
    if (width == 8)
    {
        double fromWide = (double)from;
        double toWide = (double)to;
        memcpy(a, &fromWide, 8);
        memcpy(b, &toWide, 8);
    }
    else
    {
        memcpy(a, &from, 4);
        memcpy(b, &to, 4);
    }
    int32_t at = -1;
    for (int32_t i = 0; i + width <= bytes; ++i)
    {
        if (memcmp(s_journal + i, a, (size_t)width) == 0)
        {
            if (at >= 0)
            {
                return false;
            }
            at = i;
        }
    }
    if (at < 0)
    {
        return false;
    }
    memcpy(s_journal + at, b, (size_t)width);
    return true;
}

static bool ReplaysOnto(const uint8_t* snapshot, int64_t snapshotBytes, int64_t journalBytes)
{
    m3WorldDef def = SceneDef();
    m3World* world = NewWorld(&def);
    bool ok =
        Restore(world, snapshot, snapshotBytes) && ReplayJournal(world, s_journal, journalBytes);
    m3DestroyWorld(world);
    return ok;
}

static void CheckSite(const Site* site)
{
    char what[192];
    Scene s;

    BuildScene(&s);
    snprintf(what, sizeof(what), "%s: the scene builds", site->name);
    CHECK(SceneComplete(&s), what);
    uint64_t misuse = Misuse(s.world);
    if (site->prepare != NULL)
    {
        site->prepare(&s, site->ok);
    }
    site->apply(&s, site->ok);
    snprintf(what, sizeof(what), "%s: accepted at the limit", site->name);
    CHECK(Misuse(s.world) == misuse, what);
    m3DestroyWorld(s.world);

    BuildScene(&s);
    misuse = Misuse(s.world);
    if (site->prepare != NULL)
    {
        site->prepare(&s, site->past);
    }
    snprintf(what, sizeof(what), "%s: the earlier calls accept the value past the limit",
             site->name);
    CHECK(Misuse(s.world) == misuse, what);
    uint64_t hash = m3World_Hash(s.world);
    site->apply(&s, site->past);
    snprintf(what, sizeof(what), "%s: refused just past the limit, counted once", site->name);
    CHECK(Misuse(s.world) == misuse + 1, what);
    snprintf(what, sizeof(what), "%s: the refusal changes nothing", site->name);
    CHECK(m3World_Hash(s.world) == hash, what);
    m3DestroyWorld(s.world);

    BuildScene(&s);
    int64_t snapshotBytes = m3World_GetSnapshotSize(s.world);
    uint8_t* snapshot = (uint8_t*)malloc((size_t)snapshotBytes);
    Snapshot(s.world, snapshot, snapshotBytes);
    StartJournal(s.world, s_journal, (int32_t)sizeof(s_journal));
    if (site->prepare != NULL)
    {
        site->prepare(&s, site->ok);
    }
    site->apply(&s, site->ok);
    int64_t journalBytes = StopJournal(s.world);
    m3DestroyWorld(s.world);
    snprintf(what, sizeof(what), "%s: the journal replays as recorded", site->name);
    CHECK(ReplaysOnto(snapshot, snapshotBytes, journalBytes), what);
    snprintf(what, sizeof(what), "%s: the journal holds the value once", site->name);
    CHECK(Patch(journalBytes, site->ok, site->past, site->width), what);
    snprintf(what, sizeof(what), "%s: replay refuses the value past the limit", site->name);
    CHECK(!ReplaysOnto(snapshot, snapshotBytes, journalBytes), what);
    free(snapshot);
}

static void TestSites(void)
{
    const Site sites[] = {
        // The world.
        {"gravity", NULL, Gravity, 1.0e4f, Up(1.0e4f), 4},
        {"contact hertz", NULL, ContactHertz, 1.0e5f, Up(1.0e5f), 4},
        {"contact damping ratio", NULL, ContactRatio, 1.0e4f, Up(1.0e4f), 4},
        {"contact push speed", NULL, ContactPush, 1.0e5f, Up(1.0e5f), 4},
        {"linear speed cap", NULL, LinearCap, 1.0e5f, Up(1.0e5f), 4},
        {"angular speed cap", NULL, AngularCap, 1.0e5f, Up(1.0e5f), 4},
        {"wind speed", NULL, WindSpeed, 1.0e5f, Up(1.0e5f), 4},
        {"wind gust hertz", NULL, WindHertz, 1.0e5f, Up(1.0e5f), 4},
        {"wind gust scale", NULL, WindScale, 1.0e4f, Up(1.0e4f), 4},
        {"wind direction", NULL, WindDirection, 1.0049f, 1.0051f, 4},
        {"longest step", NULL, Step, 10.0f, Up(10.0f), 4},
        {"shortest step", NULL, Step, 1.0e-4f, Down(1.0e-4f), 4},
        // Bodies.
        {"dynamic velocity", NULL, DynamicVelocity, 1.0e5f, Up(1.0e5f), 4},
        {"dynamic spin", NULL, DynamicSpin, 1.0e5f, Up(1.0e5f), 4},
        {"kinematic velocity", NULL, KinematicVelocity, 1.0e10f, Up(1.0e10f), 4},
        {"kinematic spin", NULL, KinematicSpin, 1.0e5f, Up(1.0e5f), 4},
        {"def velocity", NULL, DefVelocity, 1.0e5f, Up(1.0e5f), 4},
        {"def kinematic spin", NULL, DefKinematicSpin, 1.0e5f, Up(1.0e5f), 4},
        {"def kinematic velocity", NULL, DefKinematicVelocity, 1.0e10f, Up(1.0e10f), 4},
        {"def position", NULL, DefPosition, 1.0e15f, Up(1.0e15f), 8},
        {"transform position", NULL, Teleport, 1.0e15f, Up(1.0e15f), 8},
        {"def gravity scale", NULL, DefGravityScale, 1.0e4f, Up(1.0e4f), 4},
        {"gravity scale", NULL, GravityScale, 1.0e4f, Up(1.0e4f), 4},
        {"force", NULL, Force, 1.0e20f, Up(1.0e20f), 4},
        {"accumulated force", HeldForce, AddedForce, 3.99e19f, 4.01e19f, 4},
        {"torque", NULL, Torque, 1.0e20f, Up(1.0e20f), 4},
        {"force at the center", NULL, CenterForce, 1.0e20f, Up(1.0e20f), 4},
        {"force on a lever", NULL, LeverForce, 5.0e19f, Up(5.0e19f), 4},
        {"linear impulse", NULL, LinearImpulse, 1.0e5f, Up(1.0e5f), 4},
        {"angular impulse", NULL, AngularImpulse, 16650.0f, 16684.0f, 4},
        {"impulse at the center", NULL, CenterImpulse, 1.0e5f, Up(1.0e5f), 4},
        {"impulse on a lever", NULL, LeverImpulse, 16650.0f, 16684.0f, 4},
        {"servo target", NULL, Target, 1.0e6f, Up(1.0e6f), 8},
        {"type change speed", SpeedKinematic, TurnDynamic, 1.0e5f, Up(1.0e5f), 4},
        {"sleep threshold sign", NULL, SleepThreshold, 0.375f, -0.375f, 4},
        {"type change mass", SpeckKinematic, SpareDynamic, 6.25e-3f, 6.15e-3f, 4},
        // Shapes.
        {"shape mass floor", EmptyDynamic, LightSphere, 6.25e-3f, 6.15e-3f, 4},
        {"shape inertia floor", EmptyDynamic, ThinSphere, 3.65e-4f, 3.55e-4f, 4},
        {"reshape mass floor", NULL, Reshape, 6.25e-3f, 6.15e-3f, 4},
        {"density mass floor", NULL, Density, 1.001e-6f, 0.999e-6f, 4},
        {"body mass limit", HeavyBody, HeavyBox, 4.99e5f, 5.01e5f, 4},
        {"def density", NULL, DefDensity, 1.0e6f, Up(1.0e6f), 4},
        {"density", NULL, Density, 1.0e6f, Up(1.0e6f), 4},
        {"reshape radius", NULL, ReshapeRadius, 1.0e4f, Up(1.0e4f), 4},
        {"reshape capsule point", NULL, ReshapeCapsule, 1.0e4f, Up(1.0e4f), 4},
        {"restitution", NULL, Restitution, 1.0e4f, Up(1.0e4f), 4},
        {"surface velocity", NULL, SurfaceVelocity, 1.0e5f, Up(1.0e5f), 4},
        {"def restitution", NULL, DefRestitution, 1.0e4f, Up(1.0e4f), 4},
        {"def surface velocity", NULL, DefSurfaceVelocity, 1.0e5f, Up(1.0e5f), 4},
        {"sphere radius", NULL, SphereRadius, 1.0e4f, Up(1.0e4f), 4},
        {"sphere center", NULL, SphereCenter, 1.0e4f, Up(1.0e4f), 4},
        {"capsule radius", NULL, CapsuleRadius, 1.0e4f, Up(1.0e4f), 4},
        {"capsule point", NULL, CapsulePoint, 1.0e4f, Up(1.0e4f), 4},
        {"box extent", NULL, BoxExtent, 1.0e4f, Up(1.0e4f), 4},
        {"hull point", NULL, HullPoint, 1.0e4f, Up(1.0e4f), 4},
        // Joints.
        {"motor speed", NULL, MotorSpeed, 1.0e5f, Up(1.0e5f), 4},
        {"motor effort", NULL, MotorEffort, 1.0e20f, Up(1.0e20f), 4},
        {"rope spring hertz", NULL, RopeHertz, 1.0e5f, Up(1.0e5f), 4},
        {"rope spring damping", NULL, RopeRatio, 1.0e4f, Up(1.0e4f), 4},
        {"spring hertz", NULL, SpringHertz, 1.0e5f, Up(1.0e5f), 4},
        {"spring damping", NULL, SpringRatio, 1.0e4f, Up(1.0e4f), 4},
        {"steer hertz", NULL, SteerHertz, 1.0e5f, Up(1.0e5f), 4},
        {"steer damping", NULL, SteerRatio, 1.0e4f, Up(1.0e4f), 4},
        {"steer torque", NULL, SteerTorque, 1.0e20f, Up(1.0e20f), 4},
        {"servo aim", NULL, ServoAim, 1.0e4f, Up(1.0e4f), 4},
        {"hinge target angle", NULL, HingeTarget, M3_PI, Up(M3_PI), 4},
        {"socket target rotation", Socket, SocketTarget, 0.806f, 0.807f, 4},
        {"servo force budget", NULL, ServoForce, 1.0e20f, Up(1.0e20f), 4},
        {"servo torque budget", NULL, ServoTorque, 1.0e20f, Up(1.0e20f), 4},
        {"def motor speed", NULL, DefMotorSpeed, 1.0e5f, Up(1.0e5f), 4},
        {"def rope damping", NULL, DefRopeRatio, 1.0e4f, Up(1.0e4f), 4},
        {"def servo budget", NULL, DefServoForce, 1.0e20f, Up(1.0e20f), 4},
        {"gear ratio", NULL, GearRatio, 1.0e4f, Up(1.0e4f), 4},
        {"pulley ratio", NULL, PulleyRatio, 1.0e4f, Up(1.0e4f), 4},
        // Vehicles.
        {"wheel radius floor", NULL, WheelRadius, 1.0e-3f, Down(1.0e-3f), 4},
        {"wheel radius", NULL, WheelRadius, 1.0e4f, Up(1.0e4f), 4},
        {"wheel hertz", NULL, WheelHertz, 1.0e5f, Up(1.0e5f), 4},
        {"wheel damping", NULL, WheelRatio, 1.0e4f, Up(1.0e4f), 4},
        {"wheel anchor", NULL, WheelAnchor, 1.0e4f, Up(1.0e4f), 4},
        {"wheel rest length", NULL, WheelRest, 1.0e4f, Up(1.0e4f), 4},
        {"wheel brake share", NULL, WheelBrake, 1.0e4f, Up(1.0e4f), 4},
        {"drive force", NULL, DriveForce, 1.0e20f, Up(1.0e20f), 4},
        {"brake force", NULL, BrakeForce, 1.0e20f, Up(1.0e20f), 4},
        {"tire grip", NULL, TireGrip, 1.0e4f, Up(1.0e4f), 4},
        {"lean stabilization", NULL, Lean, 1.0e4f, Up(1.0e4f), 4},
        {"engine torque", NULL, CurveTorque, 1.0e20f, Up(1.0e20f), 4},
        {"gear", NULL, GearStep, 1.0e4f, Up(1.0e4f), 4},
        {"final drive", NULL, FinalDrive, 1.0e4f, Up(1.0e4f), 4},
        {"reverse ratio", NULL, ReverseRatio, 1.0e4f, Up(1.0e4f), 4},
        {"differential coupling", NULL, DiffCouple, 1.0e15f, Up(1.0e15f), 4},
        // Characters, soft bodies, water, explosions.
        {"walker position", NULL, WalkerPosition, 1.0e15f, Up(1.0e15f), 8},
        {"walker mass", NULL, WalkerMass, 1.0e20f, Up(1.0e20f), 4},
        {"walker mass floor", NULL, WalkerMass, 1.0e-6f, Down(1.0e-6f), 4},
        {"walker push ratio", NULL, WalkerPush, 1.0e4f, Up(1.0e4f), 4},
        {"walker radius", NULL, WalkerRadius, 1.0e4f, Up(1.0e4f), 4},
        {"walker half height", NULL, WalkerHeight, 1.0e4f, Up(1.0e4f), 4},
        {"walker move", NULL, Walk, 1.0e18f, Up(1.0e18f), 4},
        {"particle mass", NULL, ParticleMass, 1.0e20f, Up(1.0e20f), 4},
        {"particle mass floor", NULL, ParticleMass, 1.0e-6f, Down(1.0e-6f), 4},
        {"lattice spacing", NULL, LatticeSpacing, 1.0e4f, Up(1.0e4f), 4},
        {"particle radius", NULL, ParticleRadius, 1.0e4f, Up(1.0e4f), 4},
        {"particle gravity scale", NULL, ParticleGravity, 1.0e4f, Up(1.0e4f), 4},
        {"water density", NULL, WaterDensity, 1.0e6f, Up(1.0e6f), 4},
        {"water corner", NULL, WaterCorner, 1.0e15f, Up(1.0e15f), 8},
        {"water flow", NULL, WaterFlow, 1.0e5f, Up(1.0e5f), 4},
        {"blast impulse", NULL, Blast, 1.0e10f, Up(1.0e10f), 4},
        {"blast position", NULL, BlastPosition, 1.0e15f, Up(1.0e15f), 8},
        {"blast soft push", NULL, BlastSoftPush, 1.0e4f, Up(1.0e4f), 4},
    };
    for (size_t i = 0; i < sizeof(sites) / sizeof(sites[0]); ++i)
    {
        CheckSite(&sites[i]);
    }
}

// The world def takes the world setters' limits.
static void TestWorldDef(void)
{
    m3WorldDef def = SceneDef();
    def.gravity = (m3Vec3){0.0f, -1.0e4f, 0.0f};
    m3World* world = NewWorld(&def);
    CHECK(world != NULL, "a world def accepts gravity at the limit");
    m3DestroyWorld(world);
    def.gravity = (m3Vec3){0.0f, -Up(1.0e4f), 0.0f};
    CHECK(NewWorld(&def) == NULL, "a world def refuses gravity past the limit");
    def = SceneDef();
    def.contactHertz = Up(1.0e5f);
    CHECK(NewWorld(&def) == NULL, "a world def refuses contact hertz past the limit");
    def = SceneDef();
    def.contactDampingRatio = Up(1.0e4f);
    CHECK(NewWorld(&def) == NULL, "a world def refuses a damping ratio past the limit");
    def = SceneDef();
    def.contactPushMaxSpeed = Up(1.0e5f);
    CHECK(NewWorld(&def) == NULL, "a world def refuses a push speed past the limit");
    def = SceneDef();
    def.maximumLinearSpeed = Up(1.0e5f);
    CHECK(NewWorld(&def) == NULL, "a world def refuses a linear speed cap past the limit");
    def = SceneDef();
    def.maximumAngularSpeed = Up(1.0e5f);
    CHECK(NewWorld(&def) == NULL, "a world def refuses an angular speed cap past the limit");
}

// A spring or steering frequency is never negative, and is positive while
// enabled, on a distance joint's spring as on every other.
static void TestFrequencySign(void)
{
    Scene s;
    BuildScene(&s);
    uint64_t misuse = Misuse(s.world);
    m3Joint_EnableSpring(s.world, s.rope, false);
    m3Joint_SetSpringHertz(s.world, s.rope, 0.0f);
    m3WheelJoint_EnableSteering(s.world, s.wheel, false);
    m3WheelJoint_SetSteeringHertz(s.world, s.wheel, 0.0f);
    m3Joint_SetSpringHertz(s.world, s.hinge, 0.0f);
    CHECK(Misuse(s.world) == misuse, "a disabled spring or steer takes a zero frequency");
    m3Joint_EnableSpring(s.world, s.rope, true);
    CHECK(Misuse(s.world) == misuse + 1, "an enabled distance spring needs a positive frequency");
    m3Joint_SetSpringHertz(s.world, s.rope, -1.0f);
    CHECK(Misuse(s.world) == misuse + 2, "a distance spring frequency is never negative");
    m3WheelJoint_SetSteeringHertz(s.world, s.wheel, -1.0f);
    CHECK(Misuse(s.world) == misuse + 3, "a steering frequency is never negative");
    m3Joint_EnableSpring(s.world, s.hinge, true);
    CHECK(Misuse(s.world) == misuse + 4, "an enabled joint spring needs a positive frequency");
    m3Joint_SetSpringHertz(s.world, s.hinge, -1.0f);
    CHECK(Misuse(s.world) == misuse + 5, "a joint spring frequency is never negative");
    m3DestroyWorld(s.world);
}

// Turning static is always allowed, and static-only scenery stays static.
static void TestTypeChanges(void)
{
    Scene s;
    BuildScene(&s);
    uint64_t misuse = Misuse(s.world);
    m3Body_SetLinearVelocity(s.world, s.kin, (m3Vec3){1.0e10f, 0.0f, 0.0f});
    m3Body_SetType(s.world, s.kin, m3_staticBody);
    CHECK(Misuse(s.world) == misuse, "a fast kinematic body may turn static");
    m3Body_SetType(s.world, s.ground, m3_kinematicBody);
    CHECK(Misuse(s.world) == misuse + 1, "a body holding a plane stays static");
    m3DestroyWorld(s.world);
}

// Shapes on bodies that are not dynamic carry no mass, so the mass floors
// do not apply to them.
static void TestMassFloorsDynamicOnly(void)
{
    Scene s;
    BuildScene(&s);
    uint64_t misuse = Misuse(s.world);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere speck = {{0.0f, 0.0f, 0.0f}, 1.0e-3f};
    m3ShapeId a = NewSphereShape(s.world, s.ground, &sd, &speck);
    m3ShapeId b = NewSphereShape(s.world, s.kin, &sd, &speck);
    CHECK(m3Shape_IsValid(s.world, a) && m3Shape_IsValid(s.world, b) && Misuse(s.world) == misuse,
          "static and kinematic bodies take shapes under the mass floors");
    m3DestroyWorld(s.world);
}

// Replay asks the state the public calls ask, not only the values: a
// journal recorded where a call is accepted refuses to replay onto a world
// where that call is refused, and replays onto the one it was recorded on.
typedef void StateFunction(Scene* s);

static void TankCommands(Scene* s)
{
    m3Vehicle_SetTankCommands(s->world, s->car, 0.5f, 0.25f, 0.0f);
}

static void Gearbox(Scene* s)
{
    m3DrivetrainDef dd = m3DefaultDrivetrainDef();
    m3Vehicle_SetDrivetrain(s->world, s->car, &dd);
}

static void PutToSleep(Scene* s)
{
    m3Body_SetAwake(s->world, s->box, false);
}

static void BoxKinematic(Scene* s)
{
    m3Body_SetType(s->world, s->box, m3_kinematicBody);
}

static void CheckReplayState(const char* name, StateFunction* call, StateFunction* refusing)
{
    char what[192];
    Scene s;
    BuildScene(&s);
    int64_t snapshotBytes = m3World_GetSnapshotSize(s.world);
    uint8_t* snapshot = (uint8_t*)malloc((size_t)snapshotBytes);
    Snapshot(s.world, snapshot, snapshotBytes);
    uint64_t misuse = Misuse(s.world);
    StartJournal(s.world, s_journal, (int32_t)sizeof(s_journal));
    call(&s);
    int64_t journalBytes = StopJournal(s.world);
    snprintf(what, sizeof(what), "%s: the call is accepted", name);
    CHECK(Misuse(s.world) == misuse, what);
    m3DestroyWorld(s.world);
    snprintf(what, sizeof(what), "%s: the journal replays as recorded", name);
    CHECK(ReplaysOnto(snapshot, snapshotBytes, journalBytes), what);

    BuildScene(&s);
    refusing(&s);
    misuse = Misuse(s.world);
    snapshotBytes = m3World_GetSnapshotSize(s.world);
    uint8_t* changed = (uint8_t*)realloc(snapshot, (size_t)snapshotBytes);
    CHECK(changed != NULL, "the snapshot buffer grows");
    Snapshot(s.world, changed, snapshotBytes);
    call(&s);
    snprintf(what, sizeof(what), "%s: the call is refused once the state changes", name);
    CHECK(Misuse(s.world) == misuse + 1, what);
    m3DestroyWorld(s.world);
    snprintf(what, sizeof(what), "%s: replay refuses it onto that state", name);
    CHECK(!ReplaysOnto(changed, snapshotBytes, journalBytes), what);
    free(changed);
}

static void TestReplayState(void)
{
    CheckReplayState("skid steer on a gearbox", TankCommands, Gearbox);
    CheckReplayState("sleep on a body that is not dynamic", PutToSleep, BoxKinematic);
}

int main(void)
{
    TestSites();
    TestWorldDef();
    TestFrequencySign();
    TestTypeChanges();
    TestMassFloorsDynamicOnly();
    TestReplayState();
    if (s_failures == 0)
    {
        printf("test_bounds: all green\n");
        return 0;
    }
    printf("test_bounds: %d failure(s)\n", s_failures);
    return 1;
}
