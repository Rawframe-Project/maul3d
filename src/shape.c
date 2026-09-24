// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: the internal create and destroy, geometry updates and the
// body shape lists. Same law as bodies: public functions validate and
// journal, internal functions mutate, replay drives the internals.

#include "shape.h"
#include "body.h"
#include "broad_phase.h"
#include "hull.h"
#include "journal.h"
#include "manifold.h"
#include "quickhull.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

int32_t m3ShapeSlot(const m3World* world, m3ShapeId shapeId)
{
    int32_t index = shapeId.index1 - 1;
    if (world == NULL || shapeId.world != world->idWorld ||
        !m3IdPoolValid(&world->shapes.shapePool, index, shapeId.generation))
    {
        return -1;
    }
    return index;
}

m3QueryFilter m3DefaultQueryFilter(void)
{
    m3QueryFilter f;
    f.categoryBits = ~0ull;
    f.maskBits = ~0ull;
    return f;
}

m3ShapeDef m3DefaultShapeDef(void)
{
    m3ShapeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 1.0f;
    def.friction = 0.6f;
    def.restitution = 0.0f;
    def.categoryBits = 1ull;
    def.maskBits = ~0ull;
    def.groupIndex = 0;
    def.localRotation = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    def.internalValue = M3_SHAPE_COOKIE;
    return def;
}

// Materials and the compound pose, checked on every create door.
bool m3ShapeDefValid(const m3ShapeDef* def)
{
    m3Quat q = def->localRotation;
    float rotLen2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return m3FiniteV3(def->localPosition) && m3FiniteQuat(q) && rotLen2 >= 0.99f &&
           rotLen2 <= 1.01f && m3FiniteF(def->density) && def->density > 0.0f &&
           m3FiniteF(def->friction) && def->friction >= 0.0f && m3FiniteF(def->restitution) &&
           def->restitution >= 0.0f && m3FiniteF(def->rollingResistance) &&
           def->rollingResistance >= 0.0f && m3FiniteV3(def->surfaceVelocity);
}

// The plain m3ShapeGeom door (journal op 4): spheres, planes, capsules
// and box hulls, whose half extents ride in geom.v. A zero-height
// capsule is legal: it is a documented character stance.
bool m3PlainGeomValid(uint8_t type, const m3ShapeGeom* geom)
{
    switch (type)
    {
    case m3_sphereShape:
        return m3FiniteV3(geom->v) && m3FiniteF(geom->s) && geom->s > 0.0f;
    case m3_planeShape:
        return m3FiniteV3(geom->v) && m3FiniteF(geom->s) && m3Dot3(geom->v, geom->v) > 1.0e-12f;
    case m3_capsuleShape:
        return m3FiniteV3(geom->v) && m3FiniteV3(geom->v2) && m3FiniteF(geom->s) && geom->s > 0.0f;
    case m3_hullShape:
        return m3FiniteV3(geom->v) && geom->v.x > 0.0f && geom->v.y > 0.0f && geom->v.z > 0.0f;
    default:
        return false;
    }
}

static void WriteShapeSlot(m3World* world, int32_t index, int32_t bodyIndex, uint8_t type,
                           const m3ShapeGeom* geom, const m3ShapeDef* def)
{
    m3Shapes* sh = &world->shapes;
    m3Vec3 p = def->localPosition;
    m3Quat q = def->localRotation;
    sh->shapeBody[index] = bodyIndex;
    sh->shapeType[index] = type;
    sh->shapeGeom[index] = *geom;
    sh->shapeDensity[index] = def->density;
    sh->shapeFriction[index] = def->friction;
    sh->shapeRestitution[index] = def->restitution;
    sh->shapeRollingResistance[index] = def->rollingResistance;
    sh->shapeCategory[index] = def->categoryBits;
    sh->shapeMask[index] = def->maskBits;
    sh->shapeGroup[index] = def->groupIndex;
    sh->shapeUserData[index] = def->userData;
    sh->shapeSensor[index] = def->isSensor ? 1 : 0;
    sh->shapeHitEvents[index] = def->enableHitEvents ? 1 : 0;
    sh->shapePreSolve[index] = def->enablePreSolveEvents ? 1 : 0;
    sh->shapeSurfaceVel[index] = def->surfaceVelocity;
    sh->shapeLocalPos[index] = p;
    sh->shapeLocalRot[index] = q;
    sh->shapeHasOffset[index] = (p.x != 0.0f || p.y != 0.0f || p.z != 0.0f || q.x != 0.0f ||
                                 q.y != 0.0f || q.z != 0.0f || q.w != 1.0f)
                                    ? 1
                                    : 0;
    sh->shapeHullIndex[index] = -1;
    sh->shapeMeshIndex[index] = -1;
    sh->shapeHfIndex[index] = -1;
    sh->shapeVoxelIndex[index] = -1;
    // Push onto the body's list head; replay recreates in the same order.
    sh->shapeNext[index] = world->bodies.bodyShapeHead[bodyIndex];
    world->bodies.bodyShapeHead[bodyIndex] = index;
}

// Content a create hands over: an interned hull (boxes rebuild from the
// journaled half extents, general hulls arrive from QuickHull), or a
// fresh mesh, height field or voxel slot. Meshes are big and authored,
// so they are never deduplicated. The mesh BVH and the voxel surface are
// derived here from the content.
static bool AttachShapeContent(m3World* world, int32_t index, const m3ShapeContent* content)
{
    m3Shapes* sh = &world->shapes;
    switch (sh->shapeType[index])
    {
    case m3_hullShape:
    {
        m3HullData box;
        if (content->hull == NULL)
        {
            m3BuildBoxHull(&box, sh->shapeGeom[index].v);
        }
        sh->shapeHullIndex[index] =
            m3InternHull(world, content->hull != NULL ? content->hull : &box);
        return sh->shapeHullIndex[index] >= 0;
    }
    case m3_meshShape:
    {
        int32_t m = m3IdPoolAlloc(&world->meshes.meshPool);
        if (m < 0)
        {
            return false;
        }
        world->meshes.meshData[m] = *content->mesh;
        int32_t* scratch = (int32_t*)m3AllocArray(
            m3MeshEdgeScratchCount(&world->meshes.meshData[m]), (int64_t)sizeof(int32_t));
        if (scratch == NULL)
        {
            // The caller keeps its content.
            memset(&world->meshes.meshData[m], 0, sizeof(m3MeshData));
            m3IdPoolFree(&world->meshes.meshPool, m);
            return false;
        }
        m3BakeMeshEdgeFlags(&world->meshes.meshData[m], scratch);
        m3Free(scratch);
        m3MeshBvhBuild(&world->meshes.meshBvh[m], &world->meshes.meshData[m]);
        world->meshes.meshRefCounts[m] = 1;
        sh->shapeMeshIndex[index] = m;
        return true;
    }
    case m3_heightFieldShape:
    {
        int32_t h = m3IdPoolAlloc(&world->heightFields.hfPool);
        if (h < 0)
        {
            return false;
        }
        world->heightFields.hfData[h] = *content->heightField; // the sample block moves in
        world->heightFields.hfRefCounts[h] = 1;
        sh->shapeHfIndex[index] = h;
        return true;
    }
    case m3_voxelShape:
    {
        int32_t v = m3IdPoolAlloc(&world->voxels.voxelPool);
        if (v < 0)
        {
            return false;
        }
        world->voxels.voxelData[v] = *content->voxels;
        m3VoxelSurfaceBuild(&world->voxels.voxelSurface[v], &world->voxels.voxelData[v]);
        world->voxels.voxelRefCounts[v] = 1;
        sh->shapeVoxelIndex[index] = v;
        world->voxels.voxelShape[v] = index;
        // A new chunk can weld to existing ones and change their border
        // coverage too.
        m3VoxelRebuildLinks(world);
        m3VoxelCoverageRefreshAround(world, v);
        return true;
    }
    default:
        return true;
    }
}

static void UnlinkShape(m3World* world, int32_t index)
{
    int32_t* cursor = &world->bodies.bodyShapeHead[world->shapes.shapeBody[index]];
    while (*cursor != -1)
    {
        if (*cursor == index)
        {
            *cursor = world->shapes.shapeNext[index];
            break;
        }
        cursor = &world->shapes.shapeNext[*cursor];
    }
    world->shapes.shapeNext[index] = -1;
    world->shapes.shapeBody[index] = -1;
}

// Rebuilds voxel links and every chunk's coverage after a chunk vanished:
// its neighbors' border faces just became exposed.
static void RefreshAllVoxelCoverage(m3World* world)
{
    m3VoxelRebuildLinks(world);
    for (int32_t v = 0; v < world->voxels.voxelPool.maxIndex; ++v)
    {
        if (world->voxels.voxelPool.alive[v] != 0)
        {
            m3VoxelCoverageBuild(world, v);
        }
    }
}

// Releases a destroyed shape's content: the interned hull and the mesh,
// height field and voxel slots it owns.
static void ReleaseShapeContent(m3World* world, int32_t index)
{
    m3Shapes* sh = &world->shapes;
    m3ReleaseHull(world, sh->shapeHullIndex[index]);
    int32_t m = sh->shapeMeshIndex[index];
    if (m >= 0 && --world->meshes.meshRefCounts[m] == 0)
    {
        m3MeshDataFree(&world->meshes.meshData[m]);
        m3MeshBvhFree(&world->meshes.meshBvh[m]);
        m3IdPoolFree(&world->meshes.meshPool, m);
    }
    int32_t h = sh->shapeHfIndex[index];
    if (h >= 0 && --world->heightFields.hfRefCounts[h] == 0)
    {
        m3HeightFieldDataFree(&world->heightFields.hfData[h]);
        m3IdPoolFree(&world->heightFields.hfPool, h);
    }
    int32_t v = sh->shapeVoxelIndex[index];
    if (v >= 0 && --world->voxels.voxelRefCounts[v] == 0)
    {
        memset(&world->voxels.voxelData[v], 0, sizeof(m3VoxelChunkData));
        m3MeshBvhFree(&world->voxels.voxelSurface[v].bvh);
        memset(&world->voxels.voxelSurface[v], 0, sizeof(m3VoxelSurface));
        world->voxels.voxelShape[v] = -1;
        m3IdPoolFree(&world->voxels.voxelPool, v);
        RefreshAllVoxelCoverage(world);
    }
    sh->shapeHullIndex[index] = -1;
    sh->shapeMeshIndex[index] = -1;
    sh->shapeHfIndex[index] = -1;
    sh->shapeVoxelIndex[index] = -1;
}

// The pools a shape create allocates from, marked before it starts.
typedef struct ShapeCreateMarks
{
    m3IdPoolMark shape;
    m3IdPoolMark hull;
    m3IdPoolMark mesh;
    m3IdPoolMark heightField;
    m3IdPoolMark voxel;
} ShapeCreateMarks;

// Undoes a create that failed after taking its slot: every pool rewinds
// to its mark, and staged mesh and height field content stays the
// caller's to free.
static void UndoShapeCreate(m3World* world, int32_t index, const ShapeCreateMarks* marks)
{
    m3Shapes* sh = &world->shapes;
    int32_t hull = sh->shapeHullIndex[index];
    if (hull >= 0 && --world->hulls.hullRefCounts[hull] == 0)
    {
        memset(&world->hulls.hullData[hull], 0, sizeof(m3HullData));
        m3IdPoolRewind(&world->hulls.hullPool, marks->hull, hull);
    }
    int32_t m = sh->shapeMeshIndex[index];
    if (m >= 0)
    {
        m3MeshBvhFree(&world->meshes.meshBvh[m]);
        memset(&world->meshes.meshData[m], 0, sizeof(m3MeshData));
        world->meshes.meshRefCounts[m] = 0;
        m3IdPoolRewind(&world->meshes.meshPool, marks->mesh, m);
    }
    int32_t h = sh->shapeHfIndex[index];
    if (h >= 0)
    {
        memset(&world->heightFields.hfData[h], 0, sizeof(m3HeightFieldData));
        world->heightFields.hfRefCounts[h] = 0;
        m3IdPoolRewind(&world->heightFields.hfPool, marks->heightField, h);
    }
    int32_t v = sh->shapeVoxelIndex[index];
    if (v >= 0)
    {
        memset(&world->voxels.voxelData[v], 0, sizeof(m3VoxelChunkData));
        m3MeshBvhFree(&world->voxels.voxelSurface[v].bvh);
        memset(&world->voxels.voxelSurface[v], 0, sizeof(m3VoxelSurface));
        world->voxels.voxelShape[v] = -1;
        world->voxels.voxelRefCounts[v] = 0;
        m3IdPoolRewind(&world->voxels.voxelPool, marks->voxel, v);
        RefreshAllVoxelCoverage(world);
    }
    sh->shapeHullIndex[index] = -1;
    sh->shapeMeshIndex[index] = -1;
    sh->shapeHfIndex[index] = -1;
    sh->shapeVoxelIndex[index] = -1;
    UnlinkShape(world, index);
    m3IdPoolRewind(&world->shapes.shapePool, marks->shape, index);
}

// Input checks live here because replay hands this function raw journal
// bytes. Content that arrives prebuilt (hull, mesh, voxel, height field)
// validated in its own decode path. A failure leaves every pool as it
// was, and staged mesh and height field content is still the caller's.
int32_t m3CreateShapeInternal(m3World* world, int32_t bodyIndex, uint8_t type,
                              const m3ShapeGeom* geom, const m3ShapeDef* def,
                              const m3ShapeContent* content)
{
    bool prebuilt = content->hull != NULL || content->mesh != NULL ||
                    content->heightField != NULL || content->voxels != NULL;
    if (!m3ShapeDefValid(def) || (!prebuilt && !m3PlainGeomValid(type, geom)))
    {
        return -1;
    }
    ShapeCreateMarks marks = {
        m3IdPoolMarkNow(&world->shapes.shapePool), m3IdPoolMarkNow(&world->hulls.hullPool),
        m3IdPoolMarkNow(&world->meshes.meshPool), m3IdPoolMarkNow(&world->heightFields.hfPool),
        m3IdPoolMarkNow(&world->voxels.voxelPool)};
    int32_t index = m3IdPoolAlloc(&world->shapes.shapePool);
    if (index < 0)
    {
        return -1;
    }
    WriteShapeSlot(world, index, bodyIndex, type, geom, def);
    bool ok = AttachShapeContent(world, index, content);
    // Infinite planes stay out of the tree and take the dedicated pair pass.
    world->broadphase.proxyIds[index] = M3_TREE_NULL;
    if (ok && type != (uint8_t)m3_planeShape)
    {
        double lo[3];
        double hi[3];
        m3ShapeFatAabb(world, index, lo, hi);
        world->broadphase.proxyIds[index] =
            m3TreeInsert(&world->broadphase.tree, lo, hi, index, m3ProxyMask(world, index));
        ok = world->broadphase.proxyIds[index] != M3_TREE_NULL;
        world->broadphase.moved[index] = 1;
    }
    if (!ok)
    {
        UndoShapeCreate(world, index, &marks);
        return -1;
    }
    m3RecomputeMass(world, bodyIndex);
    if (type == (uint8_t)m3_planeShape)
    {
        m3RebuildPlaneList(world);
    }
    return index;
}

void m3RebuildPlaneList(m3World* world)
{
    m3Shapes* sh = &world->shapes;
    sh->planeCount = 0;
    for (int32_t s = 0; s < sh->shapePool.maxIndex; ++s)
    {
        if (sh->shapePool.alive[s] != 0 && sh->shapeType[s] == (uint8_t)m3_planeShape)
        {
            sh->planeShapes[sh->planeCount++] = s;
        }
    }
}

void m3DestroyShapeInternal(m3World* world, int32_t index)
{
    int32_t bodyIndex = world->shapes.shapeBody[index];
    bool plane = world->shapes.shapeType[index] == (uint8_t)m3_planeShape;
    UnlinkShape(world, index);
    if (world->broadphase.proxyIds[index] != M3_TREE_NULL)
    {
        m3TreeRemove(&world->broadphase.tree, world->broadphase.proxyIds[index]);
        world->broadphase.proxyIds[index] = M3_TREE_NULL;
        world->broadphase.moved[index] = 1;
    }
    ReleaseShapeContent(world, index);
    m3Shapes* sh = &world->shapes;
    sh->shapeType[index] = 0;
    sh->shapeGeom[index] = (m3ShapeGeom){{0.0f, 0.0f, 0.0f}, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f};
    sh->shapeDensity[index] = 0.0f;
    sh->shapeFriction[index] = 0.0f;
    sh->shapeRestitution[index] = 0.0f;
    sh->shapeRollingResistance[index] = 0.0f;
    sh->shapeCategory[index] = 0;
    sh->shapeMask[index] = 0;
    sh->shapeGroup[index] = 0;
    sh->shapeUserData[index] = 0;
    sh->shapeSensor[index] = 0;
    sh->shapeHitEvents[index] = 0;
    sh->shapePreSolve[index] = 0;
    sh->shapeSurfaceVel[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    sh->shapeLocalPos[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    sh->shapeLocalRot[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    sh->shapeHasOffset[index] = 0;
    m3IdPoolFree(&world->shapes.shapePool, index);
    m3RecomputeMass(world, bodyIndex);
    if (plane)
    {
        m3RebuildPlaneList(world);
    }
}

bool m3SetShapeGeomInternal(m3World* world, int32_t slot, uint8_t type, const m3ShapeGeom* geom)
{
    // The validation wall: both doors pass through here.
    // Only geometry that LIVES in m3ShapeGeom swaps (sphere and
    // capsule, conversions included); interned slabs are immutable.
    uint8_t current = world->shapes.shapeType[slot];
    if (current != (uint8_t)m3_sphereShape && current != (uint8_t)m3_capsuleShape)
    {
        return false;
    }
    if (type == (uint8_t)m3_sphereShape)
    {
        if (!m3FiniteV3(geom->v) || !m3FiniteF(geom->s) || !(geom->s > 0.0f))
        {
            return false;
        }
    }
    else if (type == (uint8_t)m3_capsuleShape)
    {
        if (!m3FiniteV3(geom->v) || !m3FiniteV3(geom->v2) || !m3FiniteF(geom->s) ||
            !(geom->s > 0.0f))
        {
            return false;
        }
        m3Vec3 axis = m3Sub3(geom->v2, geom->v);
        if (!(m3Dot3(axis, axis) > 0.0f))
        {
            return false; // a zero-length capsule is a sphere
        }
    }
    else
    {
        return false;
    }
    // Wake around BOTH silhouettes: a shrink frees what leaned on
    // the old bounds, a grow disturbs what sits inside the new.
    double oldLo[3];
    double oldHi[3];
    m3ShapeFatAabb(world, slot, oldLo, oldHi);
    world->shapes.shapeType[slot] = type;
    m3ShapeGeom fresh = *geom;
    fresh.s2 = 0.0f;
    world->shapes.shapeGeom[slot] = fresh;
    double newLo[3];
    double newHi[3];
    m3ShapeFatAabb(world, slot, newLo, newHi);
    double lo[3] = {newLo[0] < oldLo[0] ? newLo[0] : oldLo[0],
                    newLo[1] < oldLo[1] ? newLo[1] : oldLo[1],
                    newLo[2] < oldLo[2] ? newLo[2] : oldLo[2]};
    double hi[3] = {newHi[0] > oldHi[0] ? newHi[0] : oldHi[0],
                    newHi[1] > oldHi[1] ? newHi[1] : oldHi[1],
                    newHi[2] > oldHi[2] ? newHi[2] : oldHi[2]};
    int32_t body = world->shapes.shapeBody[slot];
    m3RecomputeMass(world, body);
    m3WakeRegionAabb(world, lo, hi);
    world->bodies.awake[body] = 1;
    world->bodies.sleepTimes[body] = 0.0f;
    return true;
}

static bool SetShapeGeomPublic(m3ShapeId shapeId, uint8_t type, const m3ShapeGeom* geom)
{
    m3World* world = m3WorldFromTag(shapeId.world);
    if (world == NULL)
    {
        return false;
    }
    int32_t slot = shapeId.index1 - 1;
    if (slot < 0 || slot >= world->shapes.shapePool.maxIndex ||
        world->shapes.shapePool.alive[slot] == 0 ||
        world->shapes.shapePool.generations[slot] != shapeId.generation)
    {
        return false;
    }
    if (!m3SetShapeGeomInternal(world, slot, type, geom))
    {
        return false; // refused swaps journal nothing
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSetShapeGeom record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.type = type;
        record.geom = world->shapes.shapeGeom[slot];
        m3JournalRecord(world, m3_opSetShapeGeom, &record, (int32_t)sizeof(record));
    }
    return true;
}

bool m3Shape_SetSphere(m3ShapeId shapeId, const m3Sphere* sphere)
{
    if (sphere == NULL)
    {
        m3Refuse(m3WorldFromTag(shapeId.world), m3_errorInvalid);
        return false;
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.v = sphere->center;
    geom.s = sphere->radius;
    return SetShapeGeomPublic(shapeId, (uint8_t)m3_sphereShape, &geom);
}

bool m3Shape_SetCapsule(m3ShapeId shapeId, const m3Capsule* capsule)
{
    if (capsule == NULL)
    {
        m3Refuse(m3WorldFromTag(shapeId.world), m3_errorInvalid);
        return false;
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.v = capsule->point1;
    geom.s = capsule->radius;
    geom.v2 = capsule->point2;
    return SetShapeGeomPublic(shapeId, (uint8_t)m3_capsuleShape, &geom);
}

bool m3Shape_IsValid(m3ShapeId shapeId)
{
    m3World* world = m3WorldFromTag(shapeId.world);
    return world != NULL && m3ShapeSlot(world, shapeId) >= 0;
}

m3BodyId m3Shape_GetBody(m3ShapeId shapeId)
{
    m3BodyId null = {0, 0, 0};
    m3World* world = m3WorldFromTag(shapeId.world);
    int32_t slot = world != NULL ? m3ShapeSlot(world, shapeId) : -1;
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return null;
    }
    int32_t body = world->shapes.shapeBody[slot];
    m3BodyId id = {body + 1, world->idWorld, world->bodies.bodyPool.generations[body]};
    return id;
}

void m3DestroyShape(m3ShapeId shapeId)
{
    m3World* world = m3WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? m3ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyShape, &shapeId, (int32_t)sizeof(shapeId));
    }
    int32_t bodyIndex = world->shapes.shapeBody[index];
    m3DestroyShapeInternal(world, index);
    m3RecomputeMass(world, bodyIndex);
    if (world->bodies.types[bodyIndex] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, bodyIndex, 1);
    }
}
