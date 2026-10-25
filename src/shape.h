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

void m3SetShapeDensityInternal(m3World* world, int32_t slot, float value);

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

// Whether shape slot's body, if dynamic, would keep every mass-bearing
// shape at M3_MASS_FLOOR and M3_INERTIA_FLOOR or more and its total mass
// within M3_MASS_LIMIT, with slot given this type, geometry and density
// (src/shape_mass.c). Create, reshape and density changes ask it first;
// only a dynamic body's mass reaches the solver.
bool m3ShapeMassFits(const m3World* world, int32_t slot, uint8_t type, const m3ShapeGeom* geom,
                     float density);

// The same bounds over body's shapes as they stand, asked when a body
// turns dynamic.
bool m3BodyMassFits(const m3World* world, int32_t body);

// Whether one shape of this type, geometry, density and hull content (NULL
// for a shape with none) bears no mass or weighs at least M3_MASS_FLOOR
// with every principal moment at least M3_INERTIA_FLOOR; adds its mass to
// *total. The checks above and restore's ask it.
bool m3ShapeMassFloors(uint8_t type, const m3ShapeGeom* geom, const m3HullData* hull, float density,
                       double* total);

// A material scalar a shape setter or replay may write: friction and
// rolling resistance non-negative, restitution within M3_RATIO_LIMIT
// (op is the setter's journal op).
bool m3ShapeScalarValid(int32_t op, float value);
bool m3SurfaceVelocityValid(m3Vec3 velocity);

void m3SetSurfaceVelocityInternal(m3World* world, int32_t slot, m3Vec3 v);
void m3SetFilterInternal(m3World* world, int32_t slot, uint64_t categoryBits, uint64_t maskBits,
                         int32_t groupIndex);

#endif // MAUL3D_SRC_SHAPE_H
