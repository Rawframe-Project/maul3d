// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Snapshot v1: the portable half of the rollback spine. The format is
// a padding-free little-endian header plus the persistent arrays as
// canonical field blocks, walked by ONE function in ONE order (the
// Maul2D single-source-of-truth rule: the size IS the walk). This is
// the deliberate opposite of a build-locked raw world image: no
// pointers, no layout hash, no SIMD width anywhere in the bytes. The
// header's config hash covers exactly the things that change the
// MEANING of the bytes (engine version, solver revision, precision,
// FP policy) and restore refuses a mismatch loudly.

#include "snapshot.h"

#include "fpenv.h"
#include "hull.h"
#include "misuse.h"
#include "shape.h"
#include "snapshot_links.h"
#include "state_prefix.h"
#include "voxel_coverage.h"
#include "world_internal.h"
#include "world_state.h"

#include <stddef.h>
#include <string.h>

// Little-endian only until a big-endian CI cell exists to prove the
// swap path; the walker is the single place a swap would live.
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "maul3d snapshots are little-endian only (add the swap in WalkBlocks first)"
#endif

// The math types are canonical field data only because they are
// provably padding-free; a change here is a format version bump.
_Static_assert(sizeof(m3Vec3) == 12, "m3Vec3 must be padding-free");
_Static_assert(sizeof(m3Quat) == 16, "m3Quat must be padding-free");
_Static_assert(sizeof(m3Pos3) == 24, "m3Pos3 must be padding-free");
_Static_assert(sizeof(m3Transform) == 40, "m3Transform must be padding-free");
_Static_assert(sizeof(m3Mat3) == 36, "m3Mat3 must be padding-free");

// The header says which release, which build and which world shape wrote
// the bytes; everything else, pool cursors included, is the state table's.
typedef struct m3SnapshotHeader
{
    m3StatePrefix prefix;
    uint64_t configHash;
    int32_t capacities[8];
} m3SnapshotHeader;

_Static_assert(sizeof(m3SnapshotHeader) == M3_SNAPSHOT_HEADER_BYTES,
               "snapshot header must be padding-free");

static void WorldCapacities(const m3World* world, int32_t capacities[8])
{
    capacities[0] = world->bodies.bodyCapacity;
    capacities[1] = world->shapes.shapeCapacity;
    capacities[2] = world->meshes.meshCapacity;
    capacities[3] = world->joints.jointCapacity;
    capacities[4] = world->voxels.voxelCapacity;
    capacities[5] = world->characters.characterCapacity;
    capacities[6] = world->vehicles.vehicleCapacity;
    capacities[7] = world->softBodies.softBodyCapacity;
}

static uint64_t ConfigHash(void)
{
    // Everything that changes what the serialized bytes MEAN within one
    // release, and nothing that does not (SIMD backend and worker count
    // are deliberately absent: the format is portable across them). The
    // release itself is the prefix's abiVersion.
    uint64_t h = M3_HASH_INIT;
    int32_t version = (int32_t)M3_SNAPSHOT_VERSION;
    int32_t solverRev = M3_SOLVER_REV;
    int32_t realSize = (int32_t)sizeof(m3real);
    int32_t posSize = (int32_t)sizeof(double);
    const char* fpPolicy = "contract-off;no-fast-math;no-fuse-4wide";
    h = m3Hash64(h, &version, 4);
    h = m3Hash64(h, &solverRev, 4);
    h = m3Hash64(h, &realSize, 4);
    h = m3Hash64(h, &posSize, 4);
    h = m3Hash64(h, fpPolicy, (int32_t)strlen(fpPolicy));
    return h;
}

typedef enum m3WalkMode
{
    m3_walkMeasure = 0,
    m3_walkWrite = 1,
    m3_walkRead = 2,
} m3WalkMode;

// A cursor over the snapshot byte stream. Measure only counts; write
// and read stream against the caller's buffer at the running offset.
typedef struct m3Walk
{
    uint8_t* out;
    const uint8_t* in;
    m3WalkMode mode;
    int32_t cursor;
} m3Walk;

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
static bool WalkMesh(m3Heap* heap, m3Walk* walk, m3MeshData* mesh)
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
static bool WalkHull(m3Walk* walk, m3HullData* hull)
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
static bool WalkHeightField(m3Heap* heap, m3Walk* walk, m3HeightFieldData* hf)
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
static int32_t WalkBlocks(m3World* world, uint8_t* out, const uint8_t* in, m3WalkMode mode,
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
        ok = WalkMesh(&world->heap, &walk, &world->meshes.meshData[m]);
    }
    for (int32_t i = 0; includeMesh && ok && i < world->shapes.shapeCapacity; ++i)
    {
        ok = WalkHull(&walk, &world->hulls.hullData[i]);
    }
    for (int32_t i = 0; includeMesh && ok && i < world->shapes.shapeCapacity; ++i)
    {
        ok = WalkHeightField(&world->heap, &walk, &world->heightFields.hfData[i]);
    }
    return ok ? walk.cursor : -1;
}

// The walk for the readers: measure and write only read the world, which
// the walk takes writable because a restore reads into it.
static int32_t WalkOut(const m3World* world, uint8_t* out, m3WalkMode mode, int includeMesh)
{
    M3_ASSERT(mode != m3_walkRead);
    return WalkBlocks((m3World*)world, out, NULL, mode, includeMesh);
}

static M3_NOINLINE int32_t WorldSnapshotSizeWorker(const m3World* world)
{
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1;
    }
    return (int32_t)sizeof(m3SnapshotHeader) + WalkOut(world, NULL, m3_walkMeasure, 1);
}

int32_t m3World_SnapshotSize(const m3World* world)
{
    uint64_t env = m3FloatEnvEnter();
    int32_t result = WorldSnapshotSizeWorker(world);
    m3FloatEnvLeave(env);
    return result;
}

static M3_NOINLINE m3Result WorldSnapshotWorker(const m3World* world, void* buffer,
                                                int32_t capacity, int32_t* bytesOut)
{
    if (world == NULL || buffer == NULL || bytesOut == NULL)
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    *bytesOut = 0;
    int32_t size = (int32_t)sizeof(m3SnapshotHeader) + WalkOut(world, NULL, m3_walkMeasure, 1);
    if (capacity < size)
    {
        // A buffer sized with m3World_SnapshotSize always fits, so a short
        // one is the caller's bug as well as a capacity refusal.
        m3Refuse(world, m3_errorInvalid);
        return m3_errorCapacity;
    }

    m3SnapshotHeader header;
    memset(&header, 0, sizeof(header));
    header.prefix =
        m3MakeStatePrefix(M3_SNAPSHOT_MAGIC, M3_SNAPSHOT_VERSION, M3_SNAPSHOT_HEADER_BYTES);
    header.configHash = ConfigHash();
    WorldCapacities(world, header.capacities);

    uint8_t* bytes = (uint8_t*)buffer;
    memcpy(bytes, &header, sizeof(header));
    WalkOut(world, bytes + sizeof(header), m3_walkWrite, 1);
    *bytesOut = size;
    return m3_success;
}

m3Result m3World_Snapshot(const m3World* world, void* buffer, int32_t capacity, int32_t* bytesOut)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = WorldSnapshotWorker(world, buffer, capacity, bytesOut);
    m3FloatEnvLeave(env);
    return result;
}

static bool BytesZero(const uint8_t* bytes, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (bytes[i] != 0)
        {
            return false;
        }
    }
    return true;
}

// The rules a mesh create and its paint enforce, read from one slot's
// content in the buffer: finite vertices, indices into them, and paint
// that passes the material wall. The BVH build and every contact index
// the vertices through these, and contacts mix the paint into friction.
static bool MeshContentFits(const uint8_t* content, int32_t vc, int32_t tc)
{
    for (int32_t i = 0; i < vc; ++i)
    {
        m3Vec3 vertex;
        memcpy(&vertex, content + (size_t)i * sizeof(m3Vec3), sizeof(vertex));
        if (!m3FiniteV3(vertex))
        {
            return false;
        }
    }
    const uint8_t* indices = content + (size_t)vc * sizeof(m3Vec3);
    for (int32_t i = 0; i < 3 * tc; ++i)
    {
        uint16_t index;
        memcpy(&index, indices + 2 * (size_t)i, sizeof(index));
        if (index >= (uint32_t)vc)
        {
            return false;
        }
    }
    // Paint as m3Shape_SetMeshMaterials leaves it: the same wall, with the
    // table entries past the count zero, or no paint at all with every
    // entry and group byte zero.
    const uint8_t* paint = indices + 7 * (size_t)tc;
    int32_t materialCount;
    memcpy(&materialCount, paint, sizeof(materialCount));
    const uint8_t* table = paint + sizeof(materialCount);
    m3MeshSurfaceMaterial materials[M3_MESH_MAX_MATERIALS];
    memcpy(materials, table, sizeof(materials));
    const uint8_t* groups = table + sizeof(materials);
    if (materialCount != 0 && !m3MeshMaterialsValid(materials, materialCount, groups, tc))
    {
        return false;
    }
    size_t used = (size_t)materialCount * sizeof(m3MeshSurfaceMaterial);
    if (!BytesZero(table + used, sizeof(materials) - used))
    {
        return false;
    }
    return materialCount != 0 || BytesZero(groups, (size_t)tc);
}

// The variable-size tail (mesh, hull and heightfield content) parsed
// straight from the buffer: every count within its cap and every section
// inside the buffer, before any byte lands.
// Each tail parser walks its section of the buffer from cursor and
// returns where it ends, or -1 when a count is out of range or a section
// runs past the buffer.
static int32_t MeshTailEnd(const m3World* world, const uint8_t* raw, int32_t size, int32_t cursor)
{
    for (int32_t m = 0; m < world->meshes.meshCapacity; ++m)
    {
        if (size - cursor < 8)
        {
            return -1;
        }
        int32_t vc;
        int32_t tc;
        memcpy(&vc, raw + cursor, 4);
        memcpy(&tc, raw + cursor + 4, 4);
        cursor += 8;
        if (vc < 0 || vc > M3_MESH_MAX_VERTS || tc < 0 || tc > M3_MESH_MAX_TRIS ||
            (vc == 0) != (tc == 0))
        {
            return -1; // corrupt counts refuse before any write
        }
        if (tc > 0)
        {
            // Content plus the material section: the
            // count word, the fixed eight-entry table, and a group
            // byte per triangle.
            int64_t content = (int64_t)vc * (int64_t)sizeof(m3Vec3) + 6LL * tc + (int64_t)tc + 4 +
                              (int64_t)(M3_MESH_MAX_MATERIALS * sizeof(m3MeshSurfaceMaterial)) +
                              (int64_t)tc;
            if ((int64_t)size - cursor < content)
            {
                return -1;
            }
            if (!MeshContentFits(raw + cursor, vc, tc))
            {
                return -1;
            }
            cursor += (int32_t)content;
        }
    }
    return cursor;
}

static int32_t HullTailEnd(const m3World* world, const uint8_t* raw, int32_t size, int32_t cursor)
{
    // The hull tail: same pre-validation, four counts per
    // slot, content only where vertices exist.
    for (int32_t hIdx = 0; hIdx < world->shapes.shapeCapacity; ++hIdx)
    {
        if (size - cursor < 16)
        {
            return -1;
        }
        int32_t vc;
        int32_t fc;
        int32_t ic;
        int32_t ec;
        memcpy(&vc, raw + cursor, 4);
        memcpy(&fc, raw + cursor + 4, 4);
        memcpy(&ic, raw + cursor + 8, 4);
        memcpy(&ec, raw + cursor + 12, 4);
        cursor += 16;
        if (vc < 0 || vc > M3_HULL_MAX_VERTS || fc < 0 || fc > M3_HULL_MAX_FACES || ic < 0 ||
            ic > M3_HULL_MAX_FACE_INDICES || ec < 0 || ec > M3_HULL_MAX_HALF_EDGES ||
            (vc == 0 && (fc | ic | ec) != 0))
        {
            return -1; // corrupt counts refuse before any write
        }
        if (vc > 0)
        {
            int64_t content = 4 + 2LL * (int64_t)sizeof(m3Vec3) + (int64_t)sizeof(m3Mat3) +
                              (int64_t)vc * (int64_t)sizeof(m3Vec3) +
                              (int64_t)fc * ((int64_t)sizeof(m3Vec3) + 4 + 1 + 2) + (int64_t)ic +
                              (int64_t)ec * (int64_t)sizeof(m3HullHalfEdge);
            if ((int64_t)size - cursor < content)
            {
                return -1;
            }
            // The content lands only if it is a hull the builders make.
            m3HullData incoming;
            memset(&incoming, 0, sizeof(incoming));
            m3Walk walk = {NULL, raw, m3_walkRead, cursor - 16};
            if (!WalkHull(&walk, &incoming) || !m3HullDataValid(&incoming))
            {
                return -1;
            }
            cursor += (int32_t)content;
        }
    }
    return cursor;
}

static int32_t HeightFieldTailEnd(const m3World* world, const uint8_t* raw, int32_t size,
                                  int32_t cursor)
{
    // The heightfield tail: three count words per slot,
    // samples only where a grid lives.
    for (int32_t hfIdx = 0; hfIdx < world->shapes.shapeCapacity; ++hfIdx)
    {
        if (size - cursor < 12)
        {
            return -1;
        }
        int32_t hnx;
        int32_t hnz;
        float hcell;
        memcpy(&hnx, raw + cursor, 4);
        memcpy(&hnz, raw + cursor + 4, 4);
        memcpy(&hcell, raw + cursor + 8, 4);
        cursor += 12;
        if (hnx < 0 || hnx > M3_HEIGHTFIELD_MAX_DIM || hnz < 0 || hnz > M3_HEIGHTFIELD_MAX_DIM ||
            (hnx == 0) != (hnz == 0) || (hnx > 0 && (hnx < 2 || hnz < 2)))
        {
            return -1;
        }
        if (hnx == 0)
        {
            if (hcell != 0.0f)
            {
                return -1; // an empty slot is all zeros
            }
            continue;
        }
        int64_t content = 8 + (int64_t)hnx * hnz * 4; // min/max + samples
        if ((int64_t)size - cursor < content)
        {
            return -1;
        }
        // The create door's wall, and the extremes it bakes from the
        // samples.
        float storedMin;
        float storedMax;
        memcpy(&storedMin, raw + cursor, 4);
        memcpy(&storedMax, raw + cursor + 4, 4);
        float lo;
        float hi;
        if (!m3HeightFieldSamplesValid(hnx, hnz, hcell, raw + cursor + 8, &lo, &hi) ||
            storedMin != lo || storedMax != hi)
        {
            return -1;
        }
        cursor += (int32_t)content;
    }
    return cursor;
}

static bool TailFits(const m3World* world, const uint8_t* raw, int32_t size, int32_t cursor)
{
    cursor = MeshTailEnd(world, raw, size, cursor);
    cursor = cursor < 0 ? -1 : HullTailEnd(world, raw, size, cursor);
    cursor = cursor < 0 ? -1 : HeightFieldTailEnd(world, raw, size, cursor);
    return cursor == size;
}

// Events are transient observers: a restore clears them.
static void ClearEvents(m3World* world)
{
    world->events.beginEventCount = 0;
    world->events.endEventCount = 0;
    world->events.sensorBeginEventCount = 0;
    world->events.sensorEndEventCount = 0;
    world->events.fragmentEventCount = 0;
    world->contacts.stepVetoCount = 0;
    world->contacts.replayVetoCount = 0;
    world->events.fragmentRecipeCount = 0;
    world->events.fragmentDropped = 0;
    world->events.hitEventCount = 0;
    world->events.hitEventsDropped = 0;
    world->events.moveEventCount = 0;
    world->joints.jointBreakEventCount = 0;
    world->lastInvH = 0.0f; // readback reads 0 until the next step
}

// Frees the stage's arrays, not what they hold.
static void FreeStageArrays(m3World* world, m3RestoreStage* stage)
{
    m3Free(&world->heap, stage->meshes);
    m3Free(&world->heap, stage->meshBvhs);
    m3Free(&world->heap, stage->heightFields);
    m3Free(&world->heap, (void*)stage->voxelTrees);
    memset(stage, 0, sizeof(*stage));
}

void m3RestoreDrop(m3World* world, m3RestoreStage* stage)
{
    m3Heap* heap = &world->heap;
    for (int32_t m = 0; stage->meshes != NULL && m < world->meshes.meshCapacity; ++m)
    {
        m3MeshDataFree(heap, &stage->meshes[m]);
    }
    for (int32_t m = 0; stage->meshBvhs != NULL && m < world->meshes.meshCapacity; ++m)
    {
        m3MeshBvhFree(heap, &stage->meshBvhs[m]);
    }
    for (int32_t h = 0; stage->heightFields != NULL && h < world->shapes.shapeCapacity; ++h)
    {
        m3HeightFieldDataFree(heap, &stage->heightFields[h]);
    }
    for (int32_t v = 0; stage->voxelTrees != NULL && v < world->voxels.voxelCapacity; ++v)
    {
        m3Free(heap, stage->voxelTrees[v]);
    }
    FreeStageArrays(world, stage);
}

// A zeroed array of count elements, or NULL when refused; no elements
// need no memory.
static bool StageArray(m3Heap* heap, void** out, int32_t count, int32_t elementSize)
{
    *out = count > 0 ? m3AllocZeroed(heap, count * elementSize) : NULL;
    return count <= 0 || *out != NULL;
}

// Whether every live incoming chunk's surface stays within the box
// limit, as every chunk this build lands does. Each is copied out first,
// since the buffer need not be aligned for a chunk.
static bool VoxelSurfacesFit(m3World* world, const uint8_t* raw)
{
    const uint8_t* prefix = raw + sizeof(m3SnapshotHeader);
    const uint8_t* alive =
        prefix + m3StateFieldAt(world, offsetof(m3World, voxels.voxelPool.alive));
    const uint8_t* chunks = prefix + m3StateFieldAt(world, offsetof(m3World, voxels.voxelData));
    for (int32_t v = 0; v < world->voxels.voxelCapacity; ++v)
    {
        if (alive[v] == 0)
        {
            continue;
        }
        memcpy(world->voxels.voxelStage, chunks + (size_t)v * sizeof(m3VoxelChunkData),
               sizeof(m3VoxelChunkData));
        if (!m3VoxelSurfaceFits(world, world->voxels.voxelStage))
        {
            return false;
        }
    }
    return true;
}

// Where each hull slot's content starts in the tail, or -1 for an empty
// slot; TailFits has checked every count and size.
static void HullContentStarts(const m3World* world, const uint8_t* raw, int32_t cursor,
                              int32_t* starts)
{
    for (int32_t h = 0; h < world->shapes.shapeCapacity; ++h)
    {
        int32_t counts[4];
        memcpy(counts, raw + cursor, sizeof(counts));
        cursor += 16;
        starts[h] = counts[0] > 0 ? cursor : -1;
        if (counts[0] > 0)
        {
            cursor += 4 + 2 * (int32_t)sizeof(m3Vec3) + (int32_t)sizeof(m3Mat3) +
                      counts[0] * (int32_t)sizeof(m3Vec3) +
                      counts[1] * ((int32_t)sizeof(m3Vec3) + 4 + 1 + 2) + counts[2] +
                      counts[3] * (int32_t)sizeof(m3HullHalfEdge);
        }
    }
}

// Whether every incoming dynamic body's shapes keep the mass floors and
// the body within M3_MASS_LIMIT, the bounds its creates and setters
// kept, through the check they ask. The links check has made the shape
// lists and content indices safe to follow.
static m3Result IncomingMassesFit(m3World* world, const uint8_t* raw, int32_t hullCursor)
{
    int32_t* starts = (int32_t*)m3AllocArray(
        &world->heap, world->shapes.shapeCapacity > 0 ? world->shapes.shapeCapacity : 1,
        (int64_t)sizeof(int32_t));
    if (starts == NULL)
    {
        return m3_errorCapacity;
    }
    HullContentStarts(world, raw, hullCursor, starts);
    const uint8_t* prefix = raw + sizeof(m3SnapshotHeader);
#define M3_IN(field) (prefix + m3StateFieldAt(world, offsetof(m3World, field)))
    const uint8_t* alive = M3_IN(bodies.bodyPool.alive);
    const uint8_t* types = M3_IN(bodies.types);
    const uint8_t* heads = M3_IN(bodies.bodyShapeHead);
    const uint8_t* next = M3_IN(shapes.shapeNext);
    const uint8_t* shapeTypes = M3_IN(shapes.shapeType);
    const uint8_t* geoms = M3_IN(shapes.shapeGeom);
    const uint8_t* densities = M3_IN(shapes.shapeDensity);
    const uint8_t* hullIndices = M3_IN(shapes.shapeHullIndex);
#undef M3_IN
    m3HullData hull; // only the unit mass properties are read
    memset(&hull, 0, sizeof(hull));
    bool fits = true;
    for (int32_t b = 0; fits && b < world->bodies.bodyPool.capacity; ++b)
    {
        if (alive[b] == 0 || types[b] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        double total = 0.0;
        int32_t s;
        memcpy(&s, heads + 4 * (size_t)b, sizeof(s));
        while (fits && s != -1)
        {
            m3ShapeGeom geom;
            float density;
            int32_t h;
            memcpy(&geom, geoms + (size_t)s * sizeof(m3ShapeGeom), sizeof(geom));
            memcpy(&density, densities + 4 * (size_t)s, sizeof(density));
            memcpy(&h, hullIndices + 4 * (size_t)s, sizeof(h));
            // A hull shape names a live slot (the links check); an empty
            // one weighs nothing, below the floor.
            memset(&hull.unitMass, 0, sizeof(hull.unitMass));
            memset(&hull.unitInertiaCom, 0, sizeof(hull.unitInertiaCom));
            if (h >= 0 && starts[h] >= 0)
            {
                const uint8_t* at = raw + starts[h];
                memcpy(&hull.unitMass, at, sizeof(hull.unitMass));
                memcpy(&hull.unitCom, at + 4, sizeof(hull.unitCom));
                memcpy(&hull.unitInertiaCom, at + 4 + sizeof(m3Vec3), sizeof(hull.unitInertiaCom));
            }
            fits = m3ShapeMassFloors(shapeTypes[s], &geom, h >= 0 ? &hull : NULL, density, &total);
            memcpy(&s, next + 4 * (size_t)s, sizeof(s));
        }
        fits = fits && total <= (double)M3_MASS_LIMIT;
    }
    m3Free(&world->heap, starts);
    return fits ? m3_success : m3_errorInvalid;
}

// Fills the stage from a buffer whose every byte has been checked, so a
// false return means only that the allocator refused.
static bool PrepareRestore(m3World* world, const uint8_t* raw, int32_t size, int32_t fixed,
                           m3RestoreStage* stage)
{
    m3Heap* heap = &world->heap;
    int32_t meshCap = world->meshes.meshCapacity;
    int32_t shapeCap = world->shapes.shapeCapacity;
    int32_t voxelCap = world->voxels.voxelCapacity;
    memset(stage, 0, sizeof(*stage));
    if (!StageArray(heap, (void**)&stage->meshes, meshCap, (int32_t)sizeof(m3MeshData)) ||
        !StageArray(heap, (void**)&stage->meshBvhs, meshCap, (int32_t)sizeof(m3MeshBvh)) ||
        !StageArray(heap, (void**)&stage->heightFields, shapeCap,
                    (int32_t)sizeof(m3HeightFieldData)) ||
        !StageArray(heap, (void**)&stage->voxelTrees, voxelCap, (int32_t)sizeof(void*)))
    {
        return false;
    }
    m3Walk walk = {NULL, raw, m3_walkRead, fixed};
    for (int32_t m = 0; m < meshCap; ++m)
    {
        if (!WalkMesh(heap, &walk, &stage->meshes[m]))
        {
            return false;
        }
    }
    stage->hullCursor = walk.cursor;
    walk.cursor = HullTailEnd(world, raw, size, walk.cursor);
    for (int32_t h = 0; h < shapeCap; ++h)
    {
        if (!WalkHeightField(heap, &walk, &stage->heightFields[h]))
        {
            return false;
        }
    }
    // The derived trees follow the incoming pools, read from the prefix.
    const uint8_t* prefix = raw + sizeof(m3SnapshotHeader);
    const uint8_t* meshAlive =
        prefix + m3StateFieldAt(world, offsetof(m3World, meshes.meshPool.alive));
    for (int32_t m = 0; m < meshCap; ++m)
    {
        if (meshAlive[m] != 0 && !m3MeshBvhBuild(heap, &stage->meshBvhs[m], &stage->meshes[m]))
        {
            return false;
        }
    }
    const uint8_t* voxelAlive =
        prefix + m3StateFieldAt(world, offsetof(m3World, voxels.voxelPool.alive));
    for (int32_t v = 0; v < voxelCap; ++v)
    {
        if (voxelAlive[v] != 0 && world->voxels.voxelTrees[v] == NULL)
        {
            stage->voxelTrees[v] = m3AllocZeroed(heap, M3_VOXEL_TREE_BYTES);
            if (stage->voxelTrees[v] == NULL)
            {
                return false;
            }
        }
    }
    return true;
}

// Lands the checked buffer and the whole stage; allocates nothing. The
// frozen-pair buffer came from a pair list the restore replaced, so the
// next update requeries the whole tree. Mesh BVHs and voxel surfaces are
// pure functions of the restored content, so they match the ones create
// built; voxel surfaces rebuild here, into storage the world keeps for
// them.
void m3RestoreCommit(m3World* world, m3RestoreStage* stage)
{
    const uint8_t* raw = stage->data;
    m3Heap* heap = &world->heap;
    ClearEvents(world);
    m3StateWalk(world, NULL, raw + sizeof(m3SnapshotHeader), 1);
    m3Walk walk = {NULL, raw, m3_walkRead, stage->hullCursor};
    for (int32_t i = 0; i < world->shapes.shapeCapacity; ++i)
    {
        bool landed = WalkHull(&walk, &world->hulls.hullData[i]);
        M3_ASSERT(landed); // the counts were checked before the stage
        (void)landed;
    }
    // Mass, center, inertia and extents are a function of each body's
    // type and shapes, rebuilt by the function every change to those
    // calls, so they match the source world's bit for bit.
    for (int32_t b = 0; b < world->bodies.bodyPool.maxIndex; ++b)
    {
        if (world->bodies.bodyPool.alive[b] != 0)
        {
            m3RecomputeMass(world, b);
        }
    }
    // A stage array is NULL only for a pool of capacity zero, which the
    // loops then never enter.
    for (int32_t m = 0;
         stage->meshes != NULL && stage->meshBvhs != NULL && m < world->meshes.meshCapacity; ++m)
    {
        m3MeshDataFree(heap, &world->meshes.meshData[m]);
        world->meshes.meshData[m] = stage->meshes[m];
        m3MeshBvhFree(heap, &world->meshes.meshBvh[m]);
        world->meshes.meshBvh[m] = stage->meshBvhs[m];
    }
    for (int32_t h = 0; stage->heightFields != NULL && h < world->shapes.shapeCapacity; ++h)
    {
        m3HeightFieldDataFree(heap, &world->heightFields.hfData[h]);
        world->heightFields.hfData[h] = stage->heightFields[h];
    }
    for (int32_t v = 0; stage->voxelTrees != NULL && v < world->voxels.voxelCapacity; ++v)
    {
        if (stage->voxelTrees[v] != NULL)
        {
            world->voxels.voxelTrees[v] = stage->voxelTrees[v]; // the slot had none
        }
    }
    FreeStageArrays(world, stage);
    for (int32_t v = 0; v < world->voxels.voxelCapacity; ++v)
    {
        if (world->voxels.voxelPool.alive[v] != 0)
        {
            m3VoxelSurfaceBuild(world, v);
        }
        else
        {
            memset(&world->voxels.voxelSurface[v], 0, sizeof(m3VoxelSurface));
        }
    }
    world->contacts.sleepingPairCount = 0;
    world->contacts.pairsFullQuery = 1;
    world->broadphase.candidatesFresh = 0;
    m3RebuildPlaneList(world);
    for (int32_t v = 0; v < world->voxels.voxelPool.maxIndex; ++v)
    {
        world->voxels.voxelShape[v] = -1;
    }
    for (int32_t s = 0; s < world->shapes.shapePool.maxIndex; ++s)
    {
        if (world->shapes.shapePool.alive[s] != 0 && world->shapes.shapeVoxelIndex[s] >= 0)
        {
            world->voxels.voxelShape[world->shapes.shapeVoxelIndex[s]] = s;
        }
    }
    m3VoxelRebuildLinks(world);
    for (int32_t v = 0; v < world->voxels.voxelPool.maxIndex; ++v)
    {
        if (world->voxels.voxelPool.alive[v] != 0)
        {
            m3VoxelCoverageBuild(world, v);
        }
    }
}

m3Result m3RestorePrepare(m3World* world, const void* data, int32_t size, m3RestoreStage* stage)
{
    memset(stage, 0, sizeof(*stage));
    if (data == NULL)
    {
        return m3_errorInvalid;
    }
    m3Result prefix = m3CheckStatePrefix(data, size, M3_SNAPSHOT_MAGIC, M3_SNAPSHOT_VERSION,
                                         M3_SNAPSHOT_HEADER_BYTES);
    if (prefix != m3_success)
    {
        return prefix;
    }
    m3SnapshotHeader header;
    memcpy(&header, data, sizeof(header));
    if (header.configHash != ConfigHash())
    {
        return m3_errorVersion; // another build of this release
    }
    int32_t capacities[8];
    WorldCapacities(world, capacities);
    if (memcmp(header.capacities, capacities, sizeof(capacities)) != 0)
    {
        return m3_errorInvalid; // another world shape
    }
    // Every byte is checked before any lands, so a refusal is atomic: the
    // fixed prefix against its table entries (indices and cursors in
    // range, flags that are flags, finite floats), then the variable
    // tail parsed straight from the buffer.
    int32_t fixed =
        (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure, 0);
    const uint8_t* raw = (const uint8_t*)data;
    if (size < fixed || !m3StateValidate(world, raw + sizeof(m3SnapshotHeader)) ||
        !TailFits(world, raw, size, fixed) || !VoxelSurfacesFit(world, raw))
    {
        return m3_errorInvalid;
    }
    // Then the structures as wholes, which the block checks make safe to
    // walk: what a step follows must be what the engine could have written.
    m3Result links = m3SnapshotLinksCheck(world, raw + sizeof(m3SnapshotHeader));
    if (links != m3_success)
    {
        return links;
    }
    m3Result masses = IncomingMassesFit(world, raw, MeshTailEnd(world, raw, size, fixed));
    if (masses != m3_success)
    {
        return masses;
    }
    if (!PrepareRestore(world, raw, size, fixed, stage))
    {
        m3RestoreDrop(world, stage);
        return m3_errorCapacity; // the world is as it was
    }
    stage->data = raw;
    return m3_success;
}

static M3_NOINLINE m3Result WorldRestoreWorker(m3World* world, const void* data, int32_t size)
{
    if (m3WorldBusy(world))
    {
        return m3Refuse(world, m3_errorState);
    }
    // A snapshot is untrusted data: bytes that fail a check, a short or
    // truncated buffer included, are refused with m3_errorInvalid, bytes
    // another release or build wrote with m3_errorVersion, and neither
    // counts misuse. Only arguments a correct program never passes do.
    if (world == NULL || size < 0 || (data == NULL && size > 0))
    {
        return m3Refuse(world, m3_errorInvalid);
    }
    m3RestoreStage stage;
    m3Result prepared = m3RestorePrepare(world, data, size, &stage);
    if (prepared != m3_success)
    {
        return prepared;
    }
    m3RestoreCommit(world, &stage);
    return m3_success;
}

m3Result m3World_Restore(m3World* world, const void* data, int32_t size)
{
    uint64_t env = m3FloatEnvEnter();
    m3Result result = WorldRestoreWorker(world, data, size);
    m3FloatEnvLeave(env);
    return result;
}
