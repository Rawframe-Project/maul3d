// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The snapshot walk and the size measures taken with it: one walk sizes,
// writes and reads every byte after the header, so a measure and a walk
// cannot disagree.

#include "snapshot_walk.h"

#include "state_prefix.h"
#include "world_internal.h"
#include "world_state.h"

#include <string.h>

// Little-endian only until a big-endian CI cell exists to prove the
// swap path; the walker is the single place a swap would live.
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "maul3d snapshots are little-endian only (add the swap in m3WalkBlocks first)"
#endif

static void Block(m3Walk* walk, void* ptr, int32_t bytes)
{
    if (walk->mode == m3_walkWrite)
    {
        memcpy(walk->out + walk->cursor, ptr, (size_t)bytes);
    }
    else if (walk->mode == m3_walkRead)
    {
        memcpy(ptr, walk->in + walk->cursor, (size_t)bytes);
    }
    walk->cursor += bytes;
}

// Counts land first; the read pass sizes the slot through the alloc gate
// before its content arrives. The BVH stays derived and rebuilds after
// restore. A material-free mesh writes zeros for its material table.
bool m3WalkMesh(m3Heap* heap, m3Walk* walk, m3MeshData* mesh)
{
    Block(walk, &mesh->vertexCount, 4);
    Block(walk, &mesh->triangleCount, 4);
    if (walk->mode == m3_walkRead && !m3MeshDataAlloc(heap, mesh))
    {
        return false; // corrupt counts or no memory
    }
    if (mesh->triangleCount == 0)
    {
        return true;
    }
    Block(walk, mesh->vertices, mesh->vertexCount * (int32_t)sizeof(m3Vec3));
    Block(walk, mesh->indices, 3 * mesh->triangleCount * (int32_t)sizeof(uint16_t));
    Block(walk, mesh->edgeFlags, mesh->triangleCount);
    Block(walk, &mesh->materialCount, 4);
    if (walk->mode == m3_walkRead &&
        (mesh->materialCount < 0 || mesh->materialCount > M3_MESH_MAX_MATERIALS))
    {
        return false;
    }
    Block(walk, mesh->materials, (int32_t)sizeof(mesh->materials));
    Block(walk, mesh->triMaterials, mesh->triangleCount);
    return true;
}

static bool HullCountsValid(const m3HullData* hull)
{
    return hull->vertexCount >= 0 && hull->vertexCount <= M3_HULL_MAX_VERTS &&
           hull->faceCount >= 0 && hull->faceCount <= M3_HULL_MAX_FACES && hull->indexCount >= 0 &&
           hull->indexCount <= M3_HULL_MAX_FACE_INDICES && hull->edgeCount >= 0 &&
           hull->edgeCount <= M3_HULL_MAX_HALF_EDGES &&
           (hull->vertexCount != 0 || (hull->faceCount | hull->indexCount | hull->edgeCount) == 0);
}

// Only the used prefix of each hull array travels; an empty slot costs
// 16 bytes. The read pass wipes a lived-in slot so the unused tail is
// canonical zeros. An empty slot already is (create fills prefixes,
// release wipes), and wiping every slot made restore five times slower
// than snapshot.
bool m3WalkHull(m3Walk* walk, m3HullData* hull)
{
    int32_t hadContent = hull->vertexCount; // read before the counts land
    Block(walk, &hull->vertexCount, 4);
    Block(walk, &hull->faceCount, 4);
    Block(walk, &hull->indexCount, 4);
    Block(walk, &hull->edgeCount, 4);
    if (walk->mode == m3_walkRead)
    {
        if (!HullCountsValid(hull))
        {
            return false;
        }
        if (hadContent > 0)
        {
            int32_t vc = hull->vertexCount;
            int32_t fc = hull->faceCount;
            int32_t ic = hull->indexCount;
            int32_t ec = hull->edgeCount;
            memset(hull, 0, sizeof(*hull));
            hull->vertexCount = vc;
            hull->faceCount = fc;
            hull->indexCount = ic;
            hull->edgeCount = ec;
        }
    }
    if (hull->vertexCount > 0)
    {
        Block(walk, &hull->unitMass, 4);
        Block(walk, &hull->unitCom, (int32_t)sizeof(m3Vec3));
        Block(walk, &hull->unitInertiaCom, (int32_t)sizeof(m3Mat3));
        Block(walk, &hull->center, (int32_t)sizeof(m3Vec3));
        Block(walk, hull->vertices, hull->vertexCount * (int32_t)sizeof(m3Vec3));
        Block(walk, hull->faceNormals, hull->faceCount * (int32_t)sizeof(m3Vec3));
        Block(walk, hull->faceOffsets, hull->faceCount * (int32_t)sizeof(m3real));
        Block(walk, hull->faceVertCounts, hull->faceCount);
        Block(walk, hull->faceVertStart, hull->faceCount * (int32_t)sizeof(uint16_t));
        Block(walk, hull->faceIndices, hull->indexCount);
        Block(walk, hull->edges, hull->edgeCount * (int32_t)sizeof(m3HullHalfEdge));
    }
    return true;
}

// Counts first, then the baked extremes and the raw samples; the read
// pass refuses hostile counts and sizes through the alloc gate.
bool m3WalkHeightField(m3Heap* heap, m3Walk* walk, m3HeightFieldData* hf)
{
    Block(walk, &hf->nx, 4);
    Block(walk, &hf->nz, 4);
    Block(walk, &hf->cellSize, 4);
    if (walk->mode == m3_walkRead)
    {
        bool valid = hf->nx >= 0 && hf->nx <= M3_HEIGHTFIELD_MAX_DIM && hf->nz >= 0 &&
                     hf->nz <= M3_HEIGHTFIELD_MAX_DIM && (hf->nx == 0) == (hf->nz == 0) &&
                     (hf->nx == 0 || (hf->nx >= 2 && hf->nz >= 2));
        if (!valid || !m3HeightFieldDataAlloc(heap, hf))
        {
            return false;
        }
    }
    if (hf->nx > 0)
    {
        Block(walk, &hf->minHeight, 4);
        Block(walk, &hf->maxHeight, 4);
        Block(walk, hf->heights, hf->nx * hf->nz * (int32_t)sizeof(float));
    }
    return true;
}

// The snapshot byte stream: the state table's fixed prefix, then the
// variable-size per-slot content (meshes, hulls, heightfields) last, so
// the prefix stays state-independent and restore can pre-validate sizes
// straight from the buffer. Returns the byte total, or -1 when a read
// meets corrupt counts or runs out of memory.
int64_t m3WalkBlocks(m3World* world, uint8_t* out, const uint8_t* in, m3WalkMode mode,
                     int includeMesh)
{
    m3Walk walk = {out, in, mode, 0};
    walk.cursor = m3StateWalk(world, out, in,
                              mode == m3_walkWrite  ? 0
                              : mode == m3_walkRead ? 1
                                                    : 2);
    bool ok = true;
    for (int32_t m = 0; includeMesh && ok && m < world->meshes.meshCapacity; ++m)
    {
        ok = m3WalkMesh(&world->heap, &walk, &world->meshes.meshData[m]);
    }
    for (int32_t i = 0; includeMesh && ok && i < world->shapes.shapeCapacity; ++i)
    {
        ok = m3WalkHull(&walk, &world->hulls.hullData[i]);
    }
    for (int32_t i = 0; includeMesh && ok && i < world->shapes.shapeCapacity; ++i)
    {
        ok = m3WalkHeightField(&world->heap, &walk, &world->heightFields.hfData[i]);
    }
    return ok ? walk.cursor : -1;
}

// The walk for the readers: measure and write only read the world, which
// the walk takes writable because a restore reads into it.
int64_t m3WalkOut(const m3World* world, uint8_t* out, m3WalkMode mode, int includeMesh)
{
    M3_ASSERT(mode != m3_walkRead);
    return m3WalkBlocks((m3World*)world, out, NULL, mode, includeMesh);
}

bool m3SnapshotWithinBound(int64_t bytes)
{
    return bytes <= M3_SNAPSHOT_BOUND;
}

int64_t m3SnapshotBytes(const m3World* world)
{
    int64_t size = (int64_t)M3_SNAPSHOT_HEADER_BYTES + m3WalkOut(world, NULL, m3_walkMeasure, 1);
    M3_ASSERT(m3SnapshotWithinBound(size)); // the world_state.h invariant
    return size;
}

// The most bytes one mesh slot and one shape slot take in the tail, the
// walk's own measure of the largest counts its checks let in. A shape
// slot carries a hull and an empty height field: height field content is
// counted as it is created, not here.
static int64_t MeshSlotMostBytes(void)
{
    m3MeshData mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.vertexCount = M3_MESH_MAX_VERTS;
    mesh.triangleCount = M3_MESH_MAX_TRIS;
    m3Walk walk = {NULL, NULL, m3_walkMeasure, 0};
    m3WalkMesh(NULL, &walk, &mesh);
    return walk.cursor;
}

static int64_t ShapeSlotMostBytes(void)
{
    m3HullData hull;
    memset(&hull, 0, sizeof(hull));
    hull.vertexCount = M3_HULL_MAX_VERTS;
    hull.faceCount = M3_HULL_MAX_FACES;
    hull.indexCount = M3_HULL_MAX_FACE_INDICES;
    hull.edgeCount = M3_HULL_MAX_HALF_EDGES;
    m3HeightFieldData empty;
    memset(&empty, 0, sizeof(empty));
    m3Walk walk = {NULL, NULL, m3_walkMeasure, 0};
    m3WalkHull(&walk, &hull);
    m3WalkHeightField(NULL, &walk, &empty);
    return walk.cursor;
}

bool m3SnapshotFitsCapacities(const m3World* world)
{
    int64_t most = (int64_t)M3_SNAPSHOT_HEADER_BYTES + m3StateSnapshotBytes(world) +
                   (int64_t)world->meshes.meshCapacity * MeshSlotMostBytes() +
                   (int64_t)world->shapes.shapeCapacity * ShapeSlotMostBytes();
    return m3SnapshotWithinBound(most);
}

bool m3SnapshotTakesHeightField(const m3World* world, int32_t nx, int32_t nz)
{
    // What the slot's walk adds over the empty slot it fills.
    m3HeightFieldData hf;
    memset(&hf, 0, sizeof(hf));
    m3Walk empty = {NULL, NULL, m3_walkMeasure, 0};
    m3WalkHeightField(NULL, &empty, &hf);
    hf.nx = nx;
    hf.nz = nz;
    m3Walk filled = {NULL, NULL, m3_walkMeasure, 0};
    m3WalkHeightField(NULL, &filled, &hf);
    return m3SnapshotWithinBound(m3SnapshotBytes(world) + filled.cursor - empty.cursor);
}
