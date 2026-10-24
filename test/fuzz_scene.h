// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world the fuzz targets and their seeds share: small pools, so a
// restored or replayed world stays cheap to snapshot and to step, and one
// of each kind of object, so the seed snapshot and tape hold every block
// and nearly every record a hostile input can reach. FuzzSession makes the
// journaled calls a session adds: pushes, setters, an edit, a blast, a
// destroy and steps. The seeds come from these functions on every build,
// so they always match the format the targets read.

#ifndef MAUL3D_TEST_FUZZ_SCENE_H
#define MAUL3D_TEST_FUZZ_SCENE_H

#include "helpers.h"

#include "maul3d/character.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "maul3d/vehicle.h"

#include <string.h>

#define FUZZ_DT 1.0f / 60.0f

static inline m3WorldDef FuzzWorldDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    def.meshCapacity = 2;
    def.jointCapacity = 8;
    def.voxelCapacity = 1;
    def.characterCapacity = 1;
    def.vehicleCapacity = 1;
    def.softBodyCapacity = 2;
    return def;
}

typedef struct FuzzScene
{
    m3BodyId crate;
    m3BodyId ball;
    m3ShapeId voxels;
    m3JointId hinge;
} FuzzScene;

static inline m3BodyId FuzzBody(m3World* world, m3BodyType type, double x, double y, double z)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = type;
    bd.position = (m3Pos3){x, y, z};
    return NewBody(world, &bd);
}

static inline void FuzzBuild(m3World* world, FuzzScene* scene)
{
    memset(scene, 0, sizeof(*scene));
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId ground = FuzzBody(world, m3_staticBody, 0.0, 0.0, 0.0);
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    NewPlaneShape(world, ground, &sd, &floor);

    scene->crate = FuzzBody(world, m3_dynamicBody, 0.0, 1.0, 0.0);
    NewBoxShape(world, scene->crate, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    scene->ball = FuzzBody(world, m3_dynamicBody, 1.0, 1.5, 0.0);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    NewSphereShape(world, scene->ball, &sd, &ball);
    m3Capsule rod = {{-0.3f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, 0.15f};
    NewCapsuleShape(world, FuzzBody(world, m3_dynamicBody, 2.0, 1.0, 0.0), &sd, &rod);
    m3Cylinder can = {{0.0f, -0.3f, 0.0f}, {0.0f, 0.3f, 0.0f}, 0.3f};
    NewCylinderShape(world, FuzzBody(world, m3_dynamicBody, 3.0, 1.0, 0.0), &sd, &can, 8);
    static const m3Vec3 rock[5] = {{-0.3f, 0.0f, -0.3f},
                                   {0.3f, 0.0f, -0.3f},
                                   {0.3f, 0.0f, 0.3f},
                                   {-0.3f, 0.0f, 0.3f},
                                   {0.0f, 0.5f, 0.0f}};
    NewHullShape(world, FuzzBody(world, m3_dynamicBody, 4.0, 1.0, 0.0), &sd, rock, 5);
    FuzzBody(world, m3_kinematicBody, 5.0, 1.0, 0.0);

    static const m3Vec3 corners[4] = {
        {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, 1.0f}, {-1.0f, 0.0f, 1.0f}};
    static const uint16_t quad[6] = {0, 2, 1, 0, 3, 2};
    NewMeshShape(world, FuzzBody(world, m3_staticBody, 0.0, 0.2, 3.0), &sd, corners, 4, quad, 2);
    static const float bumps[9] = {0.0f, 0.1f, 0.0f, 0.1f, 0.3f, 0.1f, 0.0f, 0.1f, 0.0f};
    NewHeightFieldGridShape(world, FuzzBody(world, m3_staticBody, 3.0, 0.0, 3.0), &sd, bumps, 3, 3,
                            1.0f);
    // A 2 x 2 x 2 block on the chunk's base layer, so a clearing edit
    // fractures what it cuts loose.
    static uint8_t cells[16 * 16 * 16];
    memset(cells, 0, sizeof(cells));
    for (int32_t i = 0; i < 8; ++i)
    {
        cells[(i & 1) + 16 * (((i >> 1) & 1) + 16 * ((i >> 2) & 1))] = 1;
    }
    scene->voxels = NewVoxelChunkShape(world, FuzzBody(world, m3_staticBody, -3.0, 0.0, 0.0), &sd,
                                       cells, NULL, 0.25f);

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyIdA = scene->crate;
    jd.bodyIdB = scene->ball;
    jd.enableLimit = true;
    jd.lowerLimit = 0.5f;
    jd.upperLimit = 2.0f;
    NewJoint(world, &jd);
    jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = ground;
    jd.bodyIdB = scene->crate;
    jd.localAnchorA = (m3Vec3){0.0f, 1.5f, 0.0f};
    jd.localAnchorB = (m3Vec3){0.0f, 0.5f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.enableMotor = true;
    jd.motorSpeed = 1.0f;
    jd.maxMotorEffort = 5.0f;
    scene->hinge = NewJoint(world, &jd);

    m3BodyId chassis = FuzzBody(world, m3_dynamicBody, 0.0, 1.0, -4.0);
    NewBoxShape(world, chassis, &sd, (m3Vec3){0.8f, 0.2f, 0.4f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassisId = chassis;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.6f : -0.6f, -0.2f, (w & 2) != 0 ? 0.35f : -0.35f};
        vd.wheels[w].driven = true;
    }
    NewVehicle(world, &vd);

    m3SoftBodyDef rope = m3DefaultSoftBodyDef();
    rope.position = (m3Pos3){-1.0, 2.0, 2.0};
    rope.countX = 4;
    rope.countY = 1;
    rope.countZ = 1;
    rope.spacing = 0.25f;
    rope.radius = 0.06f;
    NewSoftBody(world, &rope);
    m3SoftBodyDef jelly = m3DefaultSoftBodyDef();
    jelly.position = (m3Pos3){2.0, 1.0, 2.0};
    jelly.compliance = 1.0e-4f;
    jelly.radius = 0.1f;
    static const m3Vec3 points[4] = {
        {0.0f, 0.0f, 0.0f}, {0.4f, 0.0f, 0.0f}, {0.0f, 0.4f, 0.0f}, {0.0f, 0.0f, 0.4f}};
    static const uint16_t tets[4] = {0, 1, 2, 3};
    NewSoftBodyTet(world, &jelly, points, 4, tets, 1);

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.radius = 0.2f;
    cd.halfHeight = 0.3f;
    cd.position = (m3Pos3){-2.0, 1.0, -2.0};
    NewCharacter(world, &cd);

    m3WaterVolumeDef pool = m3DefaultWaterVolumeDef();
    pool.lo = (m3Pos3){1.0, 0.0, 1.0};
    pool.hi = (m3Pos3){3.0, 1.0, 3.0};
    NewWaterVolume(world, &pool);
}

static inline void FuzzStep(m3World* world, int32_t steps)
{
    for (int32_t i = 0; i < steps; ++i)
    {
        m3World_Step(world, FUZZ_DT, 2);
    }
}

// A short session on the built scene: pushes and setters, a voxel edit,
// a blast, a destroy and steps between them.
static inline void FuzzSession(m3World* world, const FuzzScene* scene)
{
    FuzzStep(world, 4);
    m3Body_SetLinearVelocity(world, scene->crate, (m3Vec3){0.5f, 0.0f, 0.0f});
    m3Body_ApplyLinearImpulse(world, scene->ball, (m3Vec3){0.0f, 0.5f, 0.2f});
    m3World_SetGravity(world, (m3Vec3){0.0f, -9.0f, 0.5f});
    FuzzStep(world, 4);
    m3VoxelChunk_ClearVoxel(world, scene->voxels, 0, 0, 0);
    m3ExplosionDef blast = m3DefaultExplosionDef();
    blast.position = (m3Pos3){1.0, 0.5, 0.0};
    blast.radius = 1.0f;
    blast.impulsePerArea = 2.0f;
    m3World_Explode(world, &blast);
    FuzzStep(world, 4);
    DestroyJoint(world, scene->hinge);
    FuzzStep(world, 4);
}

#endif
