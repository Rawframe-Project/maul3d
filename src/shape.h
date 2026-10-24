// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: internal declarations.

#ifndef MAUL3D_SRC_SHAPE_H
#define MAUL3D_SRC_SHAPE_H

#include "world_internal.h"

void m3EnableShapeHitEventsInternal(m3World* world, int32_t slot, int32_t on);

void m3EnableShapePreSolveInternal(m3World* world, int32_t slot, int32_t on);

void m3SetShapeFrictionInternal(m3World* world, int32_t slot, float value);

void m3SetShapeRestitutionInternal(m3World* world, int32_t slot, float value);

void m3SetShapeRollingInternal(m3World* world, int32_t slot, float value);

void m3SetShapeDensityInternal(m3World* world, int32_t slot, float value, int32_t updateMass);

bool m3SetShapeGeomInternal(m3World* world, int32_t slot, uint8_t type, const m3ShapeGeom* geom);

// The material wall: count 1..8, finite entries, nonnegative frictions,
// restitutions and resistances, every triangle byte < count. Set, replay
// and restore all check paint through it.
bool m3MeshMaterialsValid(const m3MeshSurfaceMaterial* materials, int32_t materialCount,
                          const uint8_t* triangleMaterials, int32_t triangleCount);

// Applies paint that passes the wall (replay hands it raw bytes).
bool m3SetMeshMaterialsInternal(m3World* world, int32_t meshIndex,
                                const m3MeshSurfaceMaterial* materials, int32_t materialCount,
                                const uint8_t* triangleMaterials);

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

void m3SetSurfaceVelocityInternal(m3World* world, int32_t slot, m3Vec3 v);
void m3SetFilterInternal(m3World* world, int32_t slot, uint64_t categoryBits, uint64_t maskBits,
                         int32_t groupIndex);

#endif // MAUL3D_SRC_SHAPE_H
