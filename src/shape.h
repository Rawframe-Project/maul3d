// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: internal declarations.

#ifndef MAUL3D_SRC_SHAPE_H
#define MAUL3D_SRC_SHAPE_H

#include "world_internal.h"

void m3EnableShapeHitEventsInternal(m3World* world, int32_t slot, int32_t on);

void m3EnableShapePreSolveInternal(m3World* world, int32_t slot, int32_t on);

// The compound gate: THE one way to read where a shape sits
// in the world. Bodies still move; shapes may ride at an offset.
// Identity short-circuits through the flag so default scenes pay a
// branch and nothing else.
static inline m3Transform m3ShapeWorldTransform(const m3World* world, int32_t shape)
{
    int32_t body = world->shapes.shapeBody[shape];
    m3Transform xf = world->bodies.transforms[body];
    if (world->shapes.shapeHasOffset[shape] != 0)
    {
        m3Vec3 r = m3RotateVec3(xf.q, world->shapes.shapeLocalPos[shape]);
        xf.p.x += (double)r.x;
        xf.p.y += (double)r.y;
        xf.p.z += (double)r.z;
        xf.q = m3MulQuat(xf.q, world->shapes.shapeLocalRot[shape]);
    }
    return xf;
}

void m3SetShapeFrictionInternal(m3World* world, int32_t slot, float value);

void m3SetShapeRestitutionInternal(m3World* world, int32_t slot, float value);

void m3SetShapeRollingInternal(m3World* world, int32_t slot, float value);

void m3SetShapeDensityInternal(m3World* world, int32_t slot, float value, int32_t updateMass);

bool m3SetShapeGeomInternal(m3World* world, int32_t slot, uint8_t type, const m3ShapeGeom* geom);

// The material wall lives in the internal (replay hands it raw
// bytes): count 1..8, finite entries, nonnegative frictions and
// resistances, every triangle byte < count.
bool m3SetMeshMaterialsInternal(m3World* world, int32_t meshIndex,
                                const m3MeshSurfaceMaterial* materials, int32_t materialCount,
                                const uint8_t* triangleMaterials);

// The filter rule, one function for pairs and queries alike.
static inline int m3FilterPass(uint64_t catA, uint64_t maskA, uint64_t catB, uint64_t maskB)
{
    return (catA & maskB) != 0 && (catB & maskA) != 0;
}

// Content a shape create hands over, built before the create: a hull
// from QuickHull, or staged mesh, voxel or height field content. All NULL
// for the plain geometry kinds.
typedef struct m3ShapeContent
{
    const m3HullData* hull;
    const m3MeshData* mesh;
    const m3VoxelChunkData* voxels;
    const m3HeightFieldData* heightField;
} m3ShapeContent;

// Materials and the compound pose of a def, and the geometry of the
// plain shape kinds. The internal create checks both; public doors check
// them first to refuse bad input as invalid rather than as capacity.
bool m3ShapeDefValid(const m3ShapeDef* def);
bool m3PlainGeomValid(uint8_t type, const m3ShapeGeom* geom);

int32_t m3CreateShapeInternal(m3World* world, int32_t bodyIndex, uint8_t type,
                              const m3ShapeGeom* geom, const m3ShapeDef* def,
                              const m3ShapeContent* content);

void m3DestroyShapeInternal(m3World* world, int32_t index);

// Rebuilds the list of live plane shapes (world->shapes.planeShapes).
void m3RebuildPlaneList(m3World* world);

void m3RecomputeMass(m3World* world, int32_t bodyIndex);

int32_t m3ShapeSlot(const m3World* world, m3ShapeId shapeId);

void m3SetSurfaceVelocityInternal(m3World* world, int32_t slot, m3Vec3 v);
void m3SetFilterInternal(m3World* world, int32_t slot, uint64_t categoryBits, uint64_t maskBits,
                         int32_t groupIndex);

#endif // MAUL3D_SRC_SHAPE_H
