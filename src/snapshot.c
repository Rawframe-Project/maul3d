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

typedef struct m3SnapshotHeader
{
    uint32_t magic;
    uint32_t formatVersion;
    uint64_t configHash;
    uint64_t stepCount;
    int32_t bodyCapacity;
    int32_t maxIndex;
    int32_t freeHead;
    int32_t freeCount;
    int32_t retiredCount;
    int32_t shapeCapacity;
    int32_t shapeMaxIndex;
    int32_t shapeFreeHead;
    int32_t shapeFreeCount;
    int32_t shapeRetiredCount;
    int32_t pairCount;
    int32_t treeRoot;
    int32_t treeFreeList;
    int32_t hullMaxIndex;
    int32_t hullFreeHead;
    int32_t hullFreeCount;
    int32_t hullRetiredCount;
    int32_t meshCapacity;
    int32_t meshMaxIndex;
    int32_t meshFreeHead;
    int32_t meshFreeCount;
    int32_t meshRetiredCount;
    int32_t jointCapacity;
    int32_t jointMaxIndex;
    int32_t jointFreeHead;
    int32_t jointFreeCount;
    int32_t jointRetiredCount;
    int32_t voxelCapacity;
    int32_t voxelMaxIndex;
    int32_t voxelFreeHead;
    int32_t voxelFreeCount;
    int32_t voxelRetiredCount;
    int32_t charCapacity;
    int32_t charMaxIndex;
    int32_t charFreeHead;
    int32_t charFreeCount;
    int32_t charRetiredCount;
    int32_t reserved[2]; // keeps the 8-byte-aligned header padding-free
    m3Vec3 gravity;
} m3SnapshotHeader;

_Static_assert(sizeof(m3SnapshotHeader) == 192, "snapshot header must be padding-free");

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

// The snapshot byte stream: the state table's fixed prefix, then the
// variable-size per-slot content (meshes, hulls, heightfields). Measure
// returns the byte total; write and read stream against the caller's
// buffer at the running offset.
static int32_t WalkBlocks(m3World* world, uint8_t* out, const uint8_t* in, m3WalkMode mode,
                          int includeMesh)
{
    int32_t cursor = 0;

#define M3_BLOCK(ptr, bytes)                                                                       \
    do                                                                                             \
    {                                                                                              \
        int32_t n = (int32_t)(bytes);                                                              \
        if (mode == m3_walkWrite)                                                                  \
        {                                                                                          \
            memcpy(out + cursor, (ptr), (size_t)n);                                                \
        }                                                                                          \
        else if (mode == m3_walkRead)                                                              \
        {                                                                                          \
            memcpy((ptr), in + cursor, (size_t)n);                                                 \
        }                                                                                          \
        cursor += n;                                                                               \
    } while (0)

    // The fixed prefix: every snapshot array, in table order.
    cursor = m3StateWalk(world, out, in, mode == m3_walkWrite ? 0 : mode == m3_walkRead ? 1 : 2);

    // Mesh content LAST: the only variable-size section, so
    // the fixed prefix above stays state-independent and the
    // restore can pre-validate sizes straight from the buffer.
    // Counts land first; the read pass sizes the slot through the
    // alloc gate before its content arrives. The BVH stays derived
    // and rebuilds after restore.
    if (includeMesh)
    {
        for (int32_t m = 0; m < world->meshCapacity; ++m)
        {
            m3MeshData* mesh = &world->meshData[m];
            M3_BLOCK(&mesh->vertexCount, 4);
            M3_BLOCK(&mesh->triangleCount, 4);
            if (mode == m3_walkRead)
            {
                if (!m3MeshDataAlloc(mesh))
                {
                    return -1; // corrupt counts or no memory: refuse
                }
            }
            if (mesh->triangleCount > 0)
            {
                M3_BLOCK(mesh->vertices, mesh->vertexCount * (int32_t)sizeof(m3Vec3));
                M3_BLOCK(mesh->indices, 3 * mesh->triangleCount * (int32_t)sizeof(uint16_t));
                M3_BLOCK(mesh->edgeFlags, mesh->triangleCount);
                // Material groups: the count, the fixed
                // table, and one group byte per triangle. A
                // material-free mesh writes zeros throughout.
                M3_BLOCK(&mesh->materialCount, 4);
                if (mode == m3_walkRead &&
                    (mesh->materialCount < 0 || mesh->materialCount > M3_MESH_MAX_MATERIALS))
                {
                    return -1; // corrupt count: refuse
                }
                M3_BLOCK(mesh->materials, (int32_t)sizeof(mesh->materials));
                M3_BLOCK(mesh->triMaterials, mesh->triangleCount);
            }
        }
        // Hull content, count-derived: counts land first,
        // the read pass validates them against the compile-time
        // caps and zeroes the slab so the unused tail is canonical
        // (a released slot already zeroes itself; this guards the
        // restore-over-a-lived-in-world path). Only the used
        // prefixes of each array travel; an empty slot costs 16
        // bytes instead of 5808.
        for (int32_t hIdx = 0; hIdx < world->shapeCapacity; ++hIdx)
        {
            m3HullData* hull = &world->hullData[hIdx];
            // The old count, read BEFORE the incoming counts land:
            // an empty slab is already canonical zeros (create
            // fills prefixes, release wipes), so only a lived-in
            // slot pays the wipe below. The unconditional memset
            // was 47 MB per restore at 8k shape slots and restore
            // ran five times slower than snapshot because of it.
            int32_t hadContent = hull->vertexCount;
            M3_BLOCK(&hull->vertexCount, 4);
            M3_BLOCK(&hull->faceCount, 4);
            M3_BLOCK(&hull->indexCount, 4);
            M3_BLOCK(&hull->edgeCount, 4);
            if (mode == m3_walkRead)
            {
                if (hull->vertexCount < 0 || hull->vertexCount > M3_HULL_MAX_VERTS ||
                    hull->faceCount < 0 || hull->faceCount > M3_HULL_MAX_FACES ||
                    hull->indexCount < 0 || hull->indexCount > M3_HULL_MAX_FACE_INDICES ||
                    hull->edgeCount < 0 || hull->edgeCount > M3_HULL_MAX_HALF_EDGES ||
                    (hull->vertexCount == 0 &&
                     (hull->faceCount | hull->indexCount | hull->edgeCount) != 0))
                {
                    return -1; // corrupt counts: refuse
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
                M3_BLOCK(&hull->unitMass, 4);
                M3_BLOCK(&hull->unitCom, (int32_t)sizeof(m3Vec3));
                M3_BLOCK(&hull->unitInertiaCom, (int32_t)sizeof(m3Mat3));
                M3_BLOCK(&hull->center, (int32_t)sizeof(m3Vec3));
                M3_BLOCK(hull->vertices, hull->vertexCount * (int32_t)sizeof(m3Vec3));
                M3_BLOCK(hull->faceNormals, hull->faceCount * (int32_t)sizeof(m3Vec3));
                M3_BLOCK(hull->faceOffsets, hull->faceCount * (int32_t)sizeof(m3real));
                M3_BLOCK(hull->faceVertCounts, hull->faceCount);
                M3_BLOCK(hull->faceVertStart, hull->faceCount * (int32_t)sizeof(uint16_t));
                M3_BLOCK(hull->faceIndices, hull->indexCount);
                M3_BLOCK(hull->edges, hull->edgeCount * (int32_t)sizeof(m3HullHalfEdge));
            }
        }
        // Native heightfield content: counts first, then the
        // baked extremes and the raw samples; the read pass sizes
        // through the alloc gate and refuses hostile counts.
        for (int32_t hfIdx = 0; hfIdx < world->shapeCapacity; ++hfIdx)
        {
            m3HeightFieldData* hf = &world->hfData[hfIdx];
            M3_BLOCK(&hf->nx, 4);
            M3_BLOCK(&hf->nz, 4);
            M3_BLOCK(&hf->cellSize, 4);
            if (mode == m3_walkRead)
            {
                if (hf->nx < 0 || hf->nx > M3_HEIGHTFIELD_MAX_DIM || hf->nz < 0 ||
                    hf->nz > M3_HEIGHTFIELD_MAX_DIM || (hf->nx == 0) != (hf->nz == 0) ||
                    (hf->nx > 0 && (hf->nx < 2 || hf->nz < 2)))
                {
                    return -1;
                }
                if (!m3HeightFieldDataAlloc(hf))
                {
                    return -1;
                }
            }
            if (hf->nx > 0)
            {
                M3_BLOCK(&hf->minHeight, 4);
                M3_BLOCK(&hf->maxHeight, 4);
                M3_BLOCK(hf->heights, hf->nx * hf->nz * (int32_t)sizeof(float));
            }
        }
    }

#undef M3_BLOCK
    return cursor;
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
    header.stepCount = world->stepCount;
    header.bodyCapacity = world->bodyCapacity;
    header.maxIndex = world->bodyPool.maxIndex;
    header.freeHead = world->bodyPool.freeHead;
    header.freeCount = world->bodyPool.freeCount;
    header.retiredCount = world->bodyPool.retiredCount;
    header.shapeCapacity = world->shapeCapacity;
    header.shapeMaxIndex = world->shapePool.maxIndex;
    header.shapeFreeHead = world->shapePool.freeHead;
    header.shapeFreeCount = world->shapePool.freeCount;
    header.shapeRetiredCount = world->shapePool.retiredCount;
    header.pairCount = world->pairCount;
    header.treeRoot = world->tree.root;
    header.treeFreeList = world->tree.freeList;
    header.hullMaxIndex = world->hullPool.maxIndex;
    header.hullFreeHead = world->hullPool.freeHead;
    header.hullFreeCount = world->hullPool.freeCount;
    header.hullRetiredCount = world->hullPool.retiredCount;
    header.reserved[0] = 0;
    header.reserved[1] = 0;
    header.meshCapacity = world->meshCapacity;
    header.jointCapacity = world->jointCapacity;
    header.jointMaxIndex = world->jointPool.maxIndex;
    header.jointFreeHead = world->jointPool.freeHead;
    header.jointFreeCount = world->jointPool.freeCount;
    header.jointRetiredCount = world->jointPool.retiredCount;
    header.charCapacity = world->characterCapacity;
    header.charMaxIndex = world->charPool.maxIndex;
    header.charFreeHead = world->charPool.freeHead;
    header.charFreeCount = world->charPool.freeCount;
    header.charRetiredCount = world->charPool.retiredCount;
    header.voxelCapacity = world->voxelCapacity;
    header.voxelMaxIndex = world->voxelPool.maxIndex;
    header.voxelFreeHead = world->voxelPool.freeHead;
    header.voxelFreeCount = world->voxelPool.freeCount;
    header.voxelRetiredCount = world->voxelPool.retiredCount;
    header.meshMaxIndex = world->meshPool.maxIndex;
    header.meshFreeHead = world->meshPool.freeHead;
    header.meshFreeCount = world->meshPool.freeCount;
    header.meshRetiredCount = world->meshPool.retiredCount;
    header.gravity = world->gravity;

    uint8_t* bytes = (uint8_t*)out;
    memcpy(bytes, &header, sizeof(header));
    WalkBlocks(world, bytes + sizeof(header), NULL, m3_walkWrite, 1);
    return size;
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
    if (header.magic != M3_SNAPSHOT_MAGIC || header.formatVersion != M3_SNAPSHOT_VERSION ||
        header.configHash != ConfigHash() || header.bodyCapacity != world->bodyCapacity ||
        header.shapeCapacity != world->shapeCapacity ||
        header.meshCapacity != world->meshCapacity ||
        header.jointCapacity != world->jointCapacity ||
        header.voxelCapacity != world->voxelCapacity ||
        header.charCapacity != world->characterCapacity)
    {
        m3Refuse(world, m3_errorInvalid);
        // Wrong world shape or wrong build semantics: refuse loudly,
        // never a partial restore.
        return false;
    }
    // Pool cursor walls (20-2 cure): a flipped header bit made
    // jointPool.maxIndex outrun its capacity and the island pass
    // read past the alive array (ASAN, the fuzz gate). Every
    // cursor is ranged BEFORE any byte lands.
#define M3_POOL_SANE(mx, fh, fc, rc, cap)                                                          \
    ((mx) >= 0 && (mx) <= (cap) && (fh) >= -1 && (fh) < (cap) && (fc) >= 0 && (fc) <= (cap) &&     \
     (rc) >= 0 && (rc) <= (cap))
    if (!M3_POOL_SANE(header.maxIndex, header.freeHead, header.freeCount, header.retiredCount,
                      world->bodyCapacity) ||
        !M3_POOL_SANE(header.shapeMaxIndex, header.shapeFreeHead, header.shapeFreeCount,
                      header.shapeRetiredCount, world->shapeCapacity) ||
        !M3_POOL_SANE(header.hullMaxIndex, header.hullFreeHead, header.hullFreeCount,
                      header.hullRetiredCount, world->shapeCapacity) ||
        !M3_POOL_SANE(header.jointMaxIndex, header.jointFreeHead, header.jointFreeCount,
                      header.jointRetiredCount, world->jointCapacity) ||
        !M3_POOL_SANE(header.meshMaxIndex, header.meshFreeHead, header.meshFreeCount,
                      header.meshRetiredCount, world->meshCapacity) ||
        !M3_POOL_SANE(header.voxelMaxIndex, header.voxelFreeHead, header.voxelFreeCount,
                      header.voxelRetiredCount, world->voxelCapacity) ||
        !M3_POOL_SANE(header.charMaxIndex, header.charFreeHead, header.charFreeCount,
                      header.charRetiredCount, world->characterCapacity))
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // hostile cursors refuse before any write
    }
#undef M3_POOL_SANE
    if (header.pairCount < 0 || header.pairCount > world->pairCapacity ||
        header.treeRoot < M3_TREE_NULL || header.treeRoot >= world->tree.capacity ||
        header.treeFreeList < M3_TREE_NULL || header.treeFreeList >= world->tree.capacity)
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // the pair list and the tree index arrays directly
    }
    // Two-phase size validation: the fixed prefix is
    // state-independent, and the variable mesh tail is parsed
    // straight from the buffer BEFORE any byte lands in the world,
    // so refusal stays atomic.
    int32_t fixed =
        (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure, 0);
    int32_t cursor = fixed;
    const uint8_t* raw = (const uint8_t*)data;
    for (int32_t m = 0; m < world->meshCapacity; ++m)
    {
        if (size - cursor < 8)
        {
            return false;
        }
        int32_t vc;
        int32_t tc;
        memcpy(&vc, raw + cursor, 4);
        memcpy(&tc, raw + cursor + 4, 4);
        cursor += 8;
        if (vc < 0 || vc > M3_MESH_MAX_VERTS || tc < 0 || tc > M3_MESH_MAX_TRIS ||
            (vc == 0) != (tc == 0))
        {
            return false; // corrupt counts refuse before any write
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
                return false;
            }
            cursor += (int32_t)content;
        }
    }
    // The hull tail: same pre-validation, four counts per
    // slot, content only where vertices exist.
    for (int32_t hIdx = 0; hIdx < world->shapeCapacity; ++hIdx)
    {
        if (size - cursor < 16)
        {
            return false;
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
            return false; // corrupt counts refuse before any write
        }
        if (vc > 0)
        {
            int64_t content = 4 + 2LL * (int64_t)sizeof(m3Vec3) + (int64_t)sizeof(m3Mat3) +
                              (int64_t)vc * (int64_t)sizeof(m3Vec3) +
                              (int64_t)fc * ((int64_t)sizeof(m3Vec3) + 4 + 1 + 2) + (int64_t)ic +
                              (int64_t)ec * (int64_t)sizeof(m3HullHalfEdge);
            if ((int64_t)size - cursor < content)
            {
                return false;
            }
            cursor += (int32_t)content;
        }
    }
    // The heightfield tail: three count words per slot,
    // samples only where a grid lives.
    for (int32_t hfIdx = 0; hfIdx < world->shapeCapacity; ++hfIdx)
    {
        if (size - cursor < 12)
        {
            return false;
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
            return false;
        }
        (void)hcell;
        if (hnx > 0)
        {
            int64_t content = 8 + (int64_t)hnx * hnz * 4; // min/max + samples
            if ((int64_t)size - cursor < content)
            {
                return false;
            }
            cursor += (int32_t)content;
        }
    }
    if (size != cursor)
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }

    world->stepCount = header.stepCount;
    world->gravity = header.gravity;
    world->bodyPool.maxIndex = header.maxIndex;
    world->bodyPool.freeHead = header.freeHead;
    world->bodyPool.freeCount = header.freeCount;
    world->bodyPool.retiredCount = header.retiredCount;
    world->shapePool.maxIndex = header.shapeMaxIndex;
    world->shapePool.freeHead = header.shapeFreeHead;
    world->shapePool.freeCount = header.shapeFreeCount;
    world->shapePool.retiredCount = header.shapeRetiredCount;
    world->pairCount = header.pairCount;
    world->tree.root = header.treeRoot;
    world->tree.freeList = header.treeFreeList;
    world->jointPool.maxIndex = header.jointMaxIndex;
    world->jointPool.freeHead = header.jointFreeHead;
    world->jointPool.freeCount = header.jointFreeCount;
    world->jointPool.retiredCount = header.jointRetiredCount;
    world->charPool.maxIndex = header.charMaxIndex;
    world->charPool.freeHead = header.charFreeHead;
    world->charPool.freeCount = header.charFreeCount;
    world->charPool.retiredCount = header.charRetiredCount;
    world->voxelPool.maxIndex = header.voxelMaxIndex;
    world->voxelPool.freeHead = header.voxelFreeHead;
    world->voxelPool.freeCount = header.voxelFreeCount;
    world->voxelPool.retiredCount = header.voxelRetiredCount;
    world->meshPool.maxIndex = header.meshMaxIndex;
    world->meshPool.freeHead = header.meshFreeHead;
    world->meshPool.freeCount = header.meshFreeCount;
    world->meshPool.retiredCount = header.meshRetiredCount;
    // Events are transient observers: a restore clears them.
    world->beginEventCount = 0;
    world->endEventCount = 0;
    world->sensorBeginEventCount = 0;
    world->sensorEndEventCount = 0;
    world->fragmentEventCount = 0;
    world->stepVetoCount = 0;
    world->replayVetoCount = 0;
    world->fragmentRecipeCount = 0;
    world->fragmentDropped = 0;
    world->hitEventCount = 0;
    world->hitEventsDropped = 0;
    world->moveEventCount = 0;
    world->jointBreakEventCount = 0;
    world->lastInvH = 0.0f; // readback reads 0 until the next step
    world->hullPool.maxIndex = header.hullMaxIndex;
    world->hullPool.freeHead = header.hullFreeHead;
    world->hullPool.freeCount = header.hullFreeCount;
    world->hullPool.retiredCount = header.hullRetiredCount;
    if (WalkBlocks(world, NULL, (const uint8_t*)data + sizeof(header), m3_walkRead, 1) < 0)
    {
        return false; // the pre-parse above makes this unreachable,
                      // except for exhausted memory in the alloc gate
    }
    // The frozen-pair buffer is derived from a pair list this
    // restore just invalidated; the next update must requery the
    // whole tree once and re-harvest.
    world->sleepingPairCount = 0;
    world->pairsFullQuery = 1;
    // Derived data follows content: the per-mesh BVHs are rebuilt
    // from the restored triangle sets (a pure function, so the tree
    // a restore produces is byte-identical to the one the original
    // create produced).
    for (int32_t m = 0; m < world->meshPool.maxIndex; ++m)
    {
        if (world->meshPool.alive[m] != 0)
        {
            m3MeshBvhBuild(&world->meshBvh[m], &world->meshData[m]);
        }
    }
    for (int32_t v = 0; v < world->voxelPool.maxIndex; ++v)
    {
        world->voxelShape[v] = -1;
        if (world->voxelPool.alive[v] != 0)
        {
            m3VoxelSurfaceBuild(&world->voxelSurface[v], &world->voxelData[v]);
        }
    }
    for (int32_t s2 = 0; s2 < world->shapePool.maxIndex; ++s2)
    {
        if (world->shapePool.alive[s2] != 0 && world->shapeVoxelIndex[s2] >= 0)
        {
            world->voxelShape[world->shapeVoxelIndex[s2]] = s2;
        }
    }
    m3VoxelRebuildLinks(world);
    for (int32_t v = 0; v < world->voxelPool.maxIndex; ++v)
    {
        if (world->voxelPool.alive[v] != 0)
        {
            m3VoxelCoverageBuild(world, v);
        }
    }
    return true;
}

uint64_t m3World_Hash(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return 0;
    }
    // Curated deterministic state in canonical slot order: what the
    // simulation IS, not how it is stored. Dead slots contribute only
    // their liveness byte (destroy zeroes state, but the hash must not
    // depend on that coincidence).
    uint64_t h = M3_HASH_INIT;
    h = m3Hash64(h, &world->stepCount, 8);
    h = m3Hash64(h, &world->gravity, (int32_t)sizeof(m3Vec3));
    if (world->contactHertz != M3_CONTACT_HERTZ_DEFAULT ||
        world->contactDampingRatio != M3_CONTACT_DAMPING_RATIO_DEFAULT ||
        world->contactPushMaxSpeed != M3_CONTACT_PUSH_MAX_SPEED_DEFAULT ||
        world->restitutionThreshold != M3_RESTITUTION_THRESHOLD_DEFAULT ||
        world->maximumLinearSpeed != M3_MAX_LINEAR_SPEED_DEFAULT || world->sleepEnabled == 0 ||
        world->continuousEnabled == 0 || world->hitEventThreshold != M3_HIT_EVENT_THRESHOLD_DEFAULT)
    {
        // Additive-state golden rule, sixth use: tuning knobs fold
        // only off their defaults.
        h = m3Hash64(h, &world->contactHertz, 4);
        h = m3Hash64(h, &world->contactDampingRatio, 4);
        h = m3Hash64(h, &world->contactPushMaxSpeed, 4);
        h = m3Hash64(h, &world->restitutionThreshold, 4);
        h = m3Hash64(h, &world->maximumLinearSpeed, 4);
        h = m3Hash64(h, &world->sleepEnabled, 1);
        h = m3Hash64(h, &world->continuousEnabled, 1);
        h = m3Hash64(h, &world->hitEventThreshold, 4);
    }
    if (world->maximumAngularSpeed != M3_MAX_ANGULAR_SPEED_DEFAULT)
    {
        // The angular cap folds in its OWN block, not the 8-4
        // knob block above: appending there would move the hash of
        // every world already off-default on an older knob.
        h = m3Hash64(h, &world->maximumAngularSpeed, 4);
    }
    if (world->windSpeed != 0.0f)
    {
        // Wind folds only when it blows (additive rule), phase
        // included: the gust wave is trajectory-shaping state.
        h = m3Hash64(h, &world->windDir, (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->windSpeed, 4);
        h = m3Hash64(h, &world->windGustHertz, 4);
        h = m3Hash64(h, &world->windGustScale, 4);
        h = m3Hash64(h, &world->windPhase, 4);
    }
    int32_t maxIndex = world->bodyPool.maxIndex;
    for (int32_t i = 0; i < maxIndex; ++i)
    {
        uint8_t alive = world->bodyPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->transforms[i], (int32_t)sizeof(m3Transform));
        h = m3Hash64(h, &world->linearVelocities[i], (int32_t)sizeof(m3Vec3));
        if (world->bodyForce[i].x != 0.0f || world->bodyForce[i].y != 0.0f ||
            world->bodyForce[i].z != 0.0f || world->bodyTorque[i].x != 0.0f ||
            world->bodyTorque[i].y != 0.0f || world->bodyTorque[i].z != 0.0f)
        {
            // Additive-state golden rule: pending host forces fold
            // only when present (a mid-step snapshot must carry
            // them; every force-free scene keeps its hash).
            h = m3Hash64(h, &world->bodyForce[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->bodyTorque[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->bodyEnabled[i] == 0 || world->bodyLocks[i] != 0 ||
            world->bodySleepThreshold[i] != M3_SLEEP_VELOCITY_DEFAULT ||
            world->bodyCanSleep[i] == 0 || world->bodyHasTarget[i] != 0)
        {
            // Additive-state golden rule, fifth use: control state
            // folds only off its defaults.
            h = m3Hash64(h, &world->bodyEnabled[i], 1);
            h = m3Hash64(h, &world->bodyLocks[i], 1);
            h = m3Hash64(h, &world->bodySleepThreshold[i], 4);
            h = m3Hash64(h, &world->bodyCanSleep[i], 1);
            h = m3Hash64(h, &world->bodyHasTarget[i], 1);
            h = m3Hash64(h, &world->bodyTarget[i], (int32_t)sizeof(m3Transform));
        }
        h = m3Hash64(h, &world->angularVelocities[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->invMass[i], (int32_t)sizeof(m3real));
        h = m3Hash64(h, &world->invInertiaLocal[i], (int32_t)sizeof(m3Mat3));
        h = m3Hash64(h, &world->localCenters[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->types[i], 1);
        h = m3Hash64(h, &world->bulletFlags[i], 1);
        h = m3Hash64(h, &world->awake[i], 1);
        h = m3Hash64(h, &world->sleepTimes[i], 4);
    }
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t i = 0; i < maxShape; ++i)
    {
        uint8_t alive = world->shapePool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->shapeBody[i], 4);
        h = m3Hash64(h, &world->shapeType[i], 1);
        h = m3Hash64(h, &world->shapeGeom[i], (int32_t)sizeof(m3ShapeGeom));
        h = m3Hash64(h, &world->shapeDensity[i], 4);
        h = m3Hash64(h, &world->shapeFriction[i], 4);
        h = m3Hash64(h, &world->shapeRestitution[i], 4);
        if (world->shapeHitEvents[i] != 0 || world->shapePreSolve[i] != 0)
        {
            // Event flags are observable shape state: they fold
            // off-default (additive rule, seventh use).
            h = m3Hash64(h, &world->shapeHitEvents[i], 1);
            h = m3Hash64(h, &world->shapePreSolve[i], 1);
        }
        if (world->shapeSurfaceVel[i].x != 0.0f || world->shapeSurfaceVel[i].y != 0.0f ||
            world->shapeSurfaceVel[i].z != 0.0f)
        {
            h = m3Hash64(h, &world->shapeSurfaceVel[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->shapeHasOffset[i] != 0)
        {
            // Compound offsets fold off-identity (additive rule,
            // tenth use).
            h = m3Hash64(h, &world->shapeLocalPos[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->shapeLocalRot[i], (int32_t)sizeof(m3Quat));
        }
        if (world->shapeRollingResistance[i] != 0.0f)
        {
            // The additive-state golden rule: the new field folds
            // only where it is set, so every pre-existing scene
            // (rolling resistance zero everywhere) keeps its hash.
            h = m3Hash64(h, &world->shapeRollingResistance[i], 4);
        }
        if (world->shapeCategory[i] != 1ull || world->shapeMask[i] != ~0ull ||
            world->shapeGroup[i] != 0)
        {
            // Same rule for filters: default-filtered shapes
            // keep every pre-existing hash still.
            h = m3Hash64(h, &world->shapeCategory[i], 8);
            h = m3Hash64(h, &world->shapeMask[i], 8);
            h = m3Hash64(h, &world->shapeGroup[i], 4);
        }
        h = m3Hash64(h, &world->shapeHullIndex[i], 4);
        h = m3Hash64(h, &world->shapeMeshIndex[i], 4);
        h = m3Hash64(h, &world->shapeSensor[i], 1);
        if (world->shapeType[i] == (uint8_t)m3_voxelShape)
        {
            // Folded only for the new type: pre-voxel scenes keep
            // their exact hash input set (the golden must not move
            // for worlds that never touch voxels).
            h = m3Hash64(h, &world->shapeVoxelIndex[i], 4);
        }
    }

    // Character state: live slots only, the additive-state rule.
    int32_t maxChar = world->charPool.maxIndex;
    for (int32_t i = 0; i < maxChar; ++i)
    {
        uint8_t alive = world->charPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->charBody[i], 4);
        h = m3Hash64(h, &world->charRadius[i], 4);
        h = m3Hash64(h, &world->charHalfHeight[i], 4);
        h = m3Hash64(h, &world->charCosSlope[i], 4);
        h = m3Hash64(h, &world->charSnap[i], 4);
        h = m3Hash64(h, &world->charSkin[i], 4);
        h = m3Hash64(h, &world->charStepHeight[i], 4);
        h = m3Hash64(h, &world->charGrounded[i], 1);
        h = m3Hash64(h, &world->charGroundNormal[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->charMass[i], 4);
        h = m3Hash64(h, &world->charPushMax[i], 4);
        h = m3Hash64(h, &world->charGroundBody[i], 4);
        h = m3Hash64(h, &world->charGroundGen[i], 2);
    }
    for (int32_t i = 0; i < world->softPool.maxIndex; ++i)
    {
        if (world->softPool.alive[i] == 0)
        {
            continue; // the additive-state golden rule: live slots only
        }
        h = m3Hash64(h, &world->softParticleCount[i], 4);
        h = m3Hash64(h, &world->softCompliance[i], 4);
        if (world->softBendCompliance[i] != 0.0f)
        {
            // Bend tethers fold off-default, their own block.
            h = m3Hash64(h, &world->softBendStart[i], 4);
            h = m3Hash64(h, &world->softBendCompliance[i], 4);
        }
        if (world->softPressure[i] != 0.0f)
        {
            // Pressure folds off-default, its own block.
            h = m3Hash64(h, &world->softDimX[i], 2);
            h = m3Hash64(h, &world->softDimY[i], 2);
            h = m3Hash64(h, &world->softDimZ[i], 2);
            h = m3Hash64(h, &world->softRestVolume[i], 4);
            h = m3Hash64(h, &world->softPressure[i], 4);
        }
        if (world->softMaxDeviation[i] != 0.0f)
        {
            // The tether folds off-default, its own block;
            // bind positions join only then (dead weight otherwise).
            h = m3Hash64(h, &world->softMaxDeviation[i], 4);
            h = m3Hash64(h, &world->softBindPos[i * M3_SOFTBODY_MAX_PARTICLES],
                         world->softParticleCount[i] * (int32_t)sizeof(m3Pos3));
        }
        if (world->softTetCount[i] > 0)
        {
            // Tets fold off-default, their own block.
            h = m3Hash64(h, &world->softTetCount[i], 4);
            int32_t tbase = i * M3_SOFTBODY_MAX_TETS;
            h = m3Hash64(h, &world->softTetA[tbase], 2 * world->softTetCount[i]);
            h = m3Hash64(h, &world->softTetB[tbase], 2 * world->softTetCount[i]);
            h = m3Hash64(h, &world->softTetC[tbase], 2 * world->softTetCount[i]);
            h = m3Hash64(h, &world->softTetD[tbase], 2 * world->softTetCount[i]);
            h = m3Hash64(h, &world->softTetRestV6[tbase], 4 * world->softTetCount[i]);
        }
        h = m3Hash64(h, &world->softRadius[i], 4);
        h = m3Hash64(h, &world->softGravityScale[i], 4);
        int32_t pc = world->softParticleCount[i];
        for (int32_t p = 0; p < pc; ++p)
        {
            int32_t k = i * M3_SOFTBODY_MAX_PARTICLES + p;
            h = m3Hash64(h, &world->softPos[k], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->softPrev[k], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->softInvMass[k], 4);
            if (world->softKick[k].x != 0.0f || world->softKick[k].y != 0.0f ||
                world->softKick[k].z != 0.0f)
            {
                // Pending kicks fold only while they exist:
                // the window between a blast and its next step is
                // real rollback state, everything else is silence.
                h = m3Hash64(h, &world->softKick[k], (int32_t)sizeof(m3Vec3));
            }
        }
        int32_t ec = world->softEdgeCount[i];
        for (int32_t e = 0; e < ec; ++e)
        {
            int32_t k = i * M3_SOFTBODY_MAX_EDGES + e;
            h = m3Hash64(h, &world->softEdgeA[k], 2);
            h = m3Hash64(h, &world->softEdgeB[k], 2);
            h = m3Hash64(h, &world->softEdgeRest[k], 4);
        }
        int32_t ac = world->softAnchorCount[i];
        for (int32_t a = 0; a < ac; ++a)
        {
            int32_t k = i * M3_SOFTBODY_MAX_ANCHORS + a;
            h = m3Hash64(h, &world->softAnchorParticle[k], 4);
            h = m3Hash64(h, &world->softAnchorBody[k], 4);
            h = m3Hash64(h, &world->softAnchorGen[k], 2);
            h = m3Hash64(h, &world->softAnchorLocal[k], (int32_t)sizeof(m3Vec3));
        }
        // Soft-to-soft anchors fold off-empty (additive rule).
        int32_t sc = world->softSoftCount[i];
        for (int32_t a = 0; a < sc; ++a)
        {
            int32_t k = i * M3_SOFTBODY_MAX_ANCHORS + a;
            h = m3Hash64(h, &world->softSoftParticleA[k], 4);
            h = m3Hash64(h, &world->softSoftSlotB[k], 4);
            h = m3Hash64(h, &world->softSoftGenB[k], 2);
            h = m3Hash64(h, &world->softSoftParticleB[k], 4);
        }
    }
    for (int32_t i = 0; i < world->vehPool.maxIndex; ++i)
    {
        if (world->vehPool.alive[i] == 0)
        {
            continue; // the additive-state golden rule: live slots only
        }
        h = m3Hash64(h, &world->vehChassis[i], 4);
        h = m3Hash64(h, &world->vehChassisGen[i], 2);
        h = m3Hash64(h, &world->vehWheelCount[i], 4);
        h = m3Hash64(h, &world->vehMaxSteer[i], 4);
        h = m3Hash64(h, &world->vehDriveForce[i], 4);
        h = m3Hash64(h, &world->vehBrakeForce[i], 4);
        h = m3Hash64(h, &world->vehTireGrip[i], 4);
        h = m3Hash64(h, &world->vehThrottle[i], 4);
        if (world->vehTrackMode[i] != 0)
        {
            // Tank mode folds off-default, its own block.
            h = m3Hash64(h, &world->vehTrackMode[i], 1);
            h = m3Hash64(h, &world->vehTrackLeft[i], 4);
            h = m3Hash64(h, &world->vehTrackRight[i], 4);
        }
        if (world->vehLeanGain[i] != 0.0f)
        {
            // The lean gain folds off-default, its own block.
            h = m3Hash64(h, &world->vehLeanGain[i], 4);
        }
        h = m3Hash64(h, &world->vehSteer[i], 4);
        h = m3Hash64(h, &world->vehBrake[i], 4);
        for (int32_t w = 0; w < world->vehWheelCount[i]; ++w)
        {
            int32_t k = i * M3_VEHICLE_MAX_WHEELS + w;
            h = m3Hash64(h, &world->vehWheelAnchor[k], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->vehWheelDir[k], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->vehWheelRest[k], 4);
            h = m3Hash64(h, &world->vehWheelTravel[k], 4);
            h = m3Hash64(h, &world->vehWheelHertz[k], 4);
            h = m3Hash64(h, &world->vehWheelZeta[k], 4);
            h = m3Hash64(h, &world->vehWheelRadius[k], 4);
            h = m3Hash64(h, &world->vehWheelFlags[k], 1);
            h = m3Hash64(h, &world->vehWheelBrake[k], 4);
            h = m3Hash64(h, &world->vehWheelCompression[k], 4);
            h = m3Hash64(h, &world->vehWheelContact[k], 1);
            h = m3Hash64(h, &world->vehWheelSpin[k], 4);
        }
        if (world->vehDtActive[i] != 0)
        {
            // The drivetrain hashes only when attached: vehicles on
            // the flat force model keep their pre-12-1 hashes (the
            // additive-state golden rule).
            h = m3Hash64(h, &world->vehDtCurveCount[i], 4);
            for (int32_t c = 0; c < world->vehDtCurveCount[i]; ++c)
            {
                h = m3Hash64(h, &world->vehDtCurveRpm[i * M3_DRIVETRAIN_MAX_CURVE + c], 4);
                h = m3Hash64(h, &world->vehDtCurveTorque[i * M3_DRIVETRAIN_MAX_CURVE + c], 4);
            }
            h = m3Hash64(h, &world->vehDtGearCount[i], 4);
            for (int32_t g = 0; g < world->vehDtGearCount[i]; ++g)
            {
                h = m3Hash64(h, &world->vehDtGearRatio[i * M3_DRIVETRAIN_MAX_GEARS + g], 4);
            }
            h = m3Hash64(h, &world->vehDtReverse[i], 4);
            h = m3Hash64(h, &world->vehDtFinal[i], 4);
            if (world->vehDtDiffMode[i] != 0)
            {
                // The diff folds only when engaged, wheel
                // speeds included: they steer forces only then.
                h = m3Hash64(h, &world->vehDtDiffMode[i], 4);
                h = m3Hash64(h, &world->vehDtDiffCouple[i], 4);
                for (int32_t wq = 0; wq < world->vehWheelCount[i]; ++wq)
                {
                    h = m3Hash64(h, &world->vehWheelLon[i * M3_VEHICLE_MAX_WHEELS + wq], 4);
                }
            }
            h = m3Hash64(h, &world->vehDtShiftUp[i], 4);
            h = m3Hash64(h, &world->vehDtShiftDown[i], 4);
            h = m3Hash64(h, &world->vehDtClutchSteps[i], 4);
            h = m3Hash64(h, &world->vehDtAutoShift[i], 1);
            h = m3Hash64(h, &world->vehDtGear[i], 1);
            h = m3Hash64(h, &world->vehDtClutch[i], 4);
            h = m3Hash64(h, &world->vehDtRpm[i], 4);
        }
    }

    // Voxel chunk content is simulation state (destruction edits it
    // and rollback must cover it); live slots only, same law as
    // everything above.
    int32_t maxVoxel = world->voxelPool.maxIndex;
    for (int32_t i = 0; i < maxVoxel; ++i)
    {
        uint8_t alive = world->voxelPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->voxelData[i].cellSize, 4);
        h = m3Hash64(h, &world->voxelData[i].filledCount, 4);
        h = m3Hash64(h, world->voxelData[i].occupancy,
                     (int32_t)sizeof(world->voxelData[i].occupancy));
        h = m3Hash64(h, world->voxelData[i].payload, (int32_t)sizeof(world->voxelData[i].payload));
        h = m3Hash64(h, world->voxelData[i].fill, (int32_t)sizeof(world->voxelData[i].fill));
    }
    int32_t maxJoint = world->jointPool.maxIndex;
    for (int32_t i = 0; i < maxJoint; ++i)
    {
        uint8_t alive = world->jointPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->jointType[i], 1);
        h = m3Hash64(h, &world->jointBodyA[i], 4);
        h = m3Hash64(h, &world->jointBodyB[i], 4);
        h = m3Hash64(h, &world->jointLocalA[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointLocalB[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointImpulse[i], (int32_t)sizeof(m3Vec3));
        if (world->jointBreak[i].x != 0.0f || world->jointBreak[i].y != 0.0f)
        {
            // Break thresholds fold off-default (additive rule,
            // eighth use).
            h = m3Hash64(h, &world->jointBreak[i], (int32_t)sizeof(m3Vec3));
        }
        if ((world->jointFlags[i] & 8) != 0 || world->jointTargetScalar[i] != 0.0f ||
            world->jointTargetQ[i].x != 0.0f || world->jointTargetQ[i].y != 0.0f ||
            world->jointTargetQ[i].z != 0.0f || world->jointTargetQ[i].w != 1.0f ||
            world->jointSpringImpulse[i].x != 0.0f || world->jointSpringImpulse[i].y != 0.0f ||
            world->jointSpringImpulse[i].z != 0.0f)
        {
            // Drive springs fold off-default (additive rule, ninth
            // use). The impulse joins: it is dynamics state.
            h = m3Hash64(h, &world->jointSpring[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->jointTargetScalar[i], 4);
            h = m3Hash64(h, &world->jointTargetQ[i], (int32_t)sizeof(m3Quat));
            h = m3Hash64(h, &world->jointSpringImpulse[i], (int32_t)sizeof(m3Vec3));
        }
        h = m3Hash64(h, &world->jointPerpImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointLimitImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointAngularImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointFrameQA[i], (int32_t)sizeof(m3Quat));
        h = m3Hash64(h, &world->jointFrameQB[i], (int32_t)sizeof(m3Quat));
        h = m3Hash64(h, &world->jointFlags[i], 1);
        if (world->jointType[i] == (uint8_t)m3_genericJoint)
        {
            // Additive-state golden rule: new fields fold only for
            // the new type.
            h = m3Hash64(h, &world->jointGenericModes[i], 2);
            h = m3Hash64(h, &world->jointGenLinLower[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->jointGenLinUpper[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->jointGenAngLower[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->jointGenAngUpper[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->jointType[i] == (uint8_t)m3_pulleyJoint)
        {
            // The same golden rule, tenth use: the rope's world
            // anchors exist only under a pulley.
            h = m3Hash64(h, &world->jointGroundA[i], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->jointGroundB[i], (int32_t)sizeof(m3Pos3));
        }
    }

    // Water volumes: folded ONLY while any volume is alive
    // (the off-default law, in its own block); the pool identity
    // rides along so a destroyed-and-recreated volume moves bits.
    {
        int32_t waterAlive = 0;
        for (int32_t k = 0; k < world->waterPool.maxIndex; ++k)
        {
            waterAlive += world->waterPool.alive[k];
        }
        if (waterAlive > 0)
        {
            h = m3Hash64(h, world->waterLo, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, world->waterHi, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, world->waterDensity, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->waterLinDrag, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->waterAngDrag, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->waterFlow, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, world->waterPool.alive, M3_MAX_WATER_VOLUMES);
        }
    }

    // Pairs and manifolds: warm-start impulses are simulation state
    // (they steer the next solve), so they are part of what the world
    // IS.
    h = m3Hash64(h, &world->pairCount, 4);
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        h = m3Hash64(h, &world->pairKeys[i], 8);
        h = m3Hash64(h, &world->manifolds[i], (int32_t)sizeof(m3Manifold));
    }
    return h;
}
