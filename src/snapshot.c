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

#include "body.h"
#include "joint_solver.h"
#include "shape.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"
#include "world_state.h"

#include <string.h>

// Little-endian only until a big-endian CI cell exists to prove the
// swap path; the walker is the single place a swap would live.
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "maul3d snapshot v1 is little-endian only (add the swap in WalkBlocks first)"
#endif

#define M3_SNAPSHOT_MAGIC   0x4D33534Eu // 'M3SN'
#define M3_SNAPSHOT_VERSION 1u

// The math types are canonical field data only because they are
// provably padding-free; a change here is a format version bump.
_Static_assert(sizeof(m3Vec3) == 12, "m3Vec3 must be padding-free");
_Static_assert(sizeof(m3Quat) == 16, "m3Quat must be padding-free");
_Static_assert(sizeof(m3Pos3) == 24, "m3Pos3 must be padding-free");
_Static_assert(sizeof(m3Transform) == 40, "m3Transform must be padding-free");
_Static_assert(sizeof(m3Mat3) == 36, "m3Mat3 must be padding-free");

// The header says which build and which world shape wrote the bytes;
// everything else, pool cursors included, is the state table's.
typedef struct m3SnapshotHeader
{
    uint32_t magic;
    uint32_t formatVersion;
    uint64_t configHash;
    int32_t capacities[8];
} m3SnapshotHeader;

_Static_assert(sizeof(m3SnapshotHeader) == 48, "snapshot header must be padding-free");

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
    // Everything that changes what the serialized bytes MEAN, and
    // nothing that does not (SIMD backend and worker count are
    // deliberately absent: the format is portable across them).
    uint64_t h = M3_HASH_INIT;
    int32_t version = m3GetVersion();
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
static bool WalkMesh(m3Walk* walk, m3MeshData* mesh)
{
    Block(walk, &mesh->vertexCount, 4);
    Block(walk, &mesh->triangleCount, 4);
    if (walk->mode == m3_walkRead && !m3MeshDataAlloc(mesh))
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
static bool WalkHeightField(m3Walk* walk, m3HeightFieldData* hf)
{
    Block(walk, &hf->nx, 4);
    Block(walk, &hf->nz, 4);
    Block(walk, &hf->cellSize, 4);
    if (walk->mode == m3_walkRead)
    {
        bool valid = hf->nx >= 0 && hf->nx <= M3_HEIGHTFIELD_MAX_DIM && hf->nz >= 0 &&
                     hf->nz <= M3_HEIGHTFIELD_MAX_DIM && (hf->nx == 0) == (hf->nz == 0) &&
                     (hf->nx == 0 || (hf->nx >= 2 && hf->nz >= 2));
        if (!valid || !m3HeightFieldDataAlloc(hf))
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
        ok = WalkMesh(&walk, &world->meshes.meshData[m]);
    }
    for (int32_t i = 0; includeMesh && ok && i < world->shapes.shapeCapacity; ++i)
    {
        ok = WalkHull(&walk, &world->hulls.hullData[i]);
    }
    for (int32_t i = 0; includeMesh && ok && i < world->shapes.shapeCapacity; ++i)
    {
        ok = WalkHeightField(&walk, &world->heightFields.hfData[i]);
    }
    return ok ? walk.cursor : -1;
}

int32_t m3World_SnapshotSize(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1;
    }
    return (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure, 1);
}

int32_t m3World_Snapshot(m3WorldId worldId, void* out, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || out == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1;
    }
    int32_t size =
        (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure, 1);
    if (capacity < size)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1; // loud: the caller sized with m3World_SnapshotSize
    }

    m3SnapshotHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = M3_SNAPSHOT_MAGIC;
    header.formatVersion = M3_SNAPSHOT_VERSION;
    header.configHash = ConfigHash();
    WorldCapacities(world, header.capacities);

    uint8_t* bytes = (uint8_t*)out;
    memcpy(bytes, &header, sizeof(header));
    WalkBlocks(world, bytes + sizeof(header), NULL, m3_walkWrite, 1);
    return size;
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
        (void)hcell;
        if (hnx > 0)
        {
            int64_t content = 8 + (int64_t)hnx * hnz * 4; // min/max + samples
            if ((int64_t)size - cursor < content)
            {
                return -1;
            }
            cursor += (int32_t)content;
        }
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

// Derived data follows content. The frozen-pair buffer came from a pair
// list the restore replaced, so the next update requeries the whole
// tree. Mesh BVHs and voxel surfaces are pure functions of the restored
// content, so they rebuild byte-identical to the ones create built.
static void RebuildDerived(m3World* world)
{
    world->contacts.sleepingPairCount = 0;
    world->contacts.pairsFullQuery = 1;
    world->broadphase.candidatesFresh = 0;
    m3RebuildPlaneList(world);
    for (int32_t m = 0; m < world->meshes.meshPool.maxIndex; ++m)
    {
        if (world->meshes.meshPool.alive[m] != 0)
        {
            m3MeshBvhBuild(&world->meshes.meshBvh[m], &world->meshes.meshData[m]);
        }
    }
    for (int32_t v = 0; v < world->voxels.voxelPool.maxIndex; ++v)
    {
        world->voxels.voxelShape[v] = -1;
        if (world->voxels.voxelPool.alive[v] != 0)
        {
            m3VoxelSurfaceBuild(&world->voxels.voxelSurface[v], &world->voxels.voxelData[v]);
        }
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

bool m3World_Restore(m3WorldId worldId, const void* data, int32_t size)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || data == NULL || size < (int32_t)sizeof(m3SnapshotHeader))
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    m3SnapshotHeader header;
    memcpy(&header, data, sizeof(header));
    int32_t capacities[8];
    WorldCapacities(world, capacities);
    if (header.magic != M3_SNAPSHOT_MAGIC || header.formatVersion != M3_SNAPSHOT_VERSION ||
        header.configHash != ConfigHash() ||
        memcmp(header.capacities, capacities, sizeof(capacities)) != 0)
    {
        m3Refuse(world, m3_errorConfig); // another build or another world shape
        return false;
    }
    // Every byte is checked before any lands, so a refusal is atomic: the
    // fixed prefix against its table entries (indices and cursors in
    // range, flags that are flags, finite floats), then the variable
    // tail parsed straight from the buffer.
    int32_t fixed =
        (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure, 0);
    const uint8_t* raw = (const uint8_t*)data;
    if (size < fixed || !m3StateValidate(world, raw + sizeof(m3SnapshotHeader)) ||
        !TailFits(world, raw, size, fixed))
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    ClearEvents(world);
    if (WalkBlocks(world, NULL, raw + sizeof(header), m3_walkRead, 1) < 0)
    {
        // The pre-parse leaves only exhausted memory in the alloc gate.
        m3Refuse(world, m3_errorCapacity);
        return false;
    }
    RebuildDerived(world);
    return true;
}
