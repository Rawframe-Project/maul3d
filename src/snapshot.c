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
        for (int32_t m = 0; m < world->meshes.meshCapacity; ++m)
        {
            m3MeshData* mesh = &world->meshes.meshData[m];
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
        for (int32_t hIdx = 0; hIdx < world->shapes.shapeCapacity; ++hIdx)
        {
            m3HullData* hull = &world->hulls.hullData[hIdx];
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
        for (int32_t hfIdx = 0; hfIdx < world->shapes.shapeCapacity; ++hfIdx)
        {
            m3HeightFieldData* hf = &world->heightFields.hfData[hfIdx];
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
    header.bodyCapacity = world->bodies.bodyCapacity;
    header.maxIndex = world->bodies.bodyPool.maxIndex;
    header.freeHead = world->bodies.bodyPool.freeHead;
    header.freeCount = world->bodies.bodyPool.freeCount;
    header.retiredCount = world->bodies.bodyPool.retiredCount;
    header.shapeCapacity = world->shapes.shapeCapacity;
    header.shapeMaxIndex = world->shapes.shapePool.maxIndex;
    header.shapeFreeHead = world->shapes.shapePool.freeHead;
    header.shapeFreeCount = world->shapes.shapePool.freeCount;
    header.shapeRetiredCount = world->shapes.shapePool.retiredCount;
    header.pairCount = world->contacts.pairCount;
    header.treeRoot = world->broadphase.tree.root;
    header.treeFreeList = world->broadphase.tree.freeList;
    header.hullMaxIndex = world->hulls.hullPool.maxIndex;
    header.hullFreeHead = world->hulls.hullPool.freeHead;
    header.hullFreeCount = world->hulls.hullPool.freeCount;
    header.hullRetiredCount = world->hulls.hullPool.retiredCount;
    header.reserved[0] = 0;
    header.reserved[1] = 0;
    header.meshCapacity = world->meshes.meshCapacity;
    header.jointCapacity = world->joints.jointCapacity;
    header.jointMaxIndex = world->joints.jointPool.maxIndex;
    header.jointFreeHead = world->joints.jointPool.freeHead;
    header.jointFreeCount = world->joints.jointPool.freeCount;
    header.jointRetiredCount = world->joints.jointPool.retiredCount;
    header.charCapacity = world->characters.characterCapacity;
    header.charMaxIndex = world->characters.charPool.maxIndex;
    header.charFreeHead = world->characters.charPool.freeHead;
    header.charFreeCount = world->characters.charPool.freeCount;
    header.charRetiredCount = world->characters.charPool.retiredCount;
    header.voxelCapacity = world->voxels.voxelCapacity;
    header.voxelMaxIndex = world->voxels.voxelPool.maxIndex;
    header.voxelFreeHead = world->voxels.voxelPool.freeHead;
    header.voxelFreeCount = world->voxels.voxelPool.freeCount;
    header.voxelRetiredCount = world->voxels.voxelPool.retiredCount;
    header.meshMaxIndex = world->meshes.meshPool.maxIndex;
    header.meshFreeHead = world->meshes.meshPool.freeHead;
    header.meshFreeCount = world->meshes.meshPool.freeCount;
    header.meshRetiredCount = world->meshes.meshPool.retiredCount;
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
        header.configHash != ConfigHash() || header.bodyCapacity != world->bodies.bodyCapacity ||
        header.shapeCapacity != world->shapes.shapeCapacity ||
        header.meshCapacity != world->meshes.meshCapacity ||
        header.jointCapacity != world->joints.jointCapacity ||
        header.voxelCapacity != world->voxels.voxelCapacity ||
        header.charCapacity != world->characters.characterCapacity)
    {
        m3Refuse(world, m3_errorInvalid);
        // Wrong world shape or wrong build semantics: refuse loudly,
        // never a partial restore.
        return false;
    }
    // Pool cursor walls: a flipped header bit made
    // jointPool.maxIndex outrun its capacity and the island pass
    // read past the alive array (ASAN, the fuzz gate). Every
    // cursor is ranged BEFORE any byte lands.
#define M3_POOL_SANE(mx, fh, fc, rc, cap)                                                          \
    ((mx) >= 0 && (mx) <= (cap) && (fh) >= -1 && (fh) < (cap) && (fc) >= 0 && (fc) <= (cap) &&     \
     (rc) >= 0 && (rc) <= (cap))
    if (!M3_POOL_SANE(header.maxIndex, header.freeHead, header.freeCount, header.retiredCount,
                      world->bodies.bodyCapacity) ||
        !M3_POOL_SANE(header.shapeMaxIndex, header.shapeFreeHead, header.shapeFreeCount,
                      header.shapeRetiredCount, world->shapes.shapeCapacity) ||
        !M3_POOL_SANE(header.hullMaxIndex, header.hullFreeHead, header.hullFreeCount,
                      header.hullRetiredCount, world->shapes.shapeCapacity) ||
        !M3_POOL_SANE(header.jointMaxIndex, header.jointFreeHead, header.jointFreeCount,
                      header.jointRetiredCount, world->joints.jointCapacity) ||
        !M3_POOL_SANE(header.meshMaxIndex, header.meshFreeHead, header.meshFreeCount,
                      header.meshRetiredCount, world->meshes.meshCapacity) ||
        !M3_POOL_SANE(header.voxelMaxIndex, header.voxelFreeHead, header.voxelFreeCount,
                      header.voxelRetiredCount, world->voxels.voxelCapacity) ||
        !M3_POOL_SANE(header.charMaxIndex, header.charFreeHead, header.charFreeCount,
                      header.charRetiredCount, world->characters.characterCapacity))
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // hostile cursors refuse before any write
    }
#undef M3_POOL_SANE
    if (header.pairCount < 0 || header.pairCount > world->contacts.pairCapacity ||
        header.treeRoot < M3_TREE_NULL || header.treeRoot >= world->broadphase.tree.capacity ||
        header.treeFreeList < M3_TREE_NULL ||
        header.treeFreeList >= world->broadphase.tree.capacity)
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
    for (int32_t m = 0; m < world->meshes.meshCapacity; ++m)
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
    for (int32_t hIdx = 0; hIdx < world->shapes.shapeCapacity; ++hIdx)
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
    for (int32_t hfIdx = 0; hfIdx < world->shapes.shapeCapacity; ++hfIdx)
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
    world->bodies.bodyPool.maxIndex = header.maxIndex;
    world->bodies.bodyPool.freeHead = header.freeHead;
    world->bodies.bodyPool.freeCount = header.freeCount;
    world->bodies.bodyPool.retiredCount = header.retiredCount;
    world->shapes.shapePool.maxIndex = header.shapeMaxIndex;
    world->shapes.shapePool.freeHead = header.shapeFreeHead;
    world->shapes.shapePool.freeCount = header.shapeFreeCount;
    world->shapes.shapePool.retiredCount = header.shapeRetiredCount;
    world->contacts.pairCount = header.pairCount;
    world->broadphase.tree.root = header.treeRoot;
    world->broadphase.tree.freeList = header.treeFreeList;
    world->joints.jointPool.maxIndex = header.jointMaxIndex;
    world->joints.jointPool.freeHead = header.jointFreeHead;
    world->joints.jointPool.freeCount = header.jointFreeCount;
    world->joints.jointPool.retiredCount = header.jointRetiredCount;
    world->characters.charPool.maxIndex = header.charMaxIndex;
    world->characters.charPool.freeHead = header.charFreeHead;
    world->characters.charPool.freeCount = header.charFreeCount;
    world->characters.charPool.retiredCount = header.charRetiredCount;
    world->voxels.voxelPool.maxIndex = header.voxelMaxIndex;
    world->voxels.voxelPool.freeHead = header.voxelFreeHead;
    world->voxels.voxelPool.freeCount = header.voxelFreeCount;
    world->voxels.voxelPool.retiredCount = header.voxelRetiredCount;
    world->meshes.meshPool.maxIndex = header.meshMaxIndex;
    world->meshes.meshPool.freeHead = header.meshFreeHead;
    world->meshes.meshPool.freeCount = header.meshFreeCount;
    world->meshes.meshPool.retiredCount = header.meshRetiredCount;
    // Events are transient observers: a restore clears them.
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
    world->hulls.hullPool.maxIndex = header.hullMaxIndex;
    world->hulls.hullPool.freeHead = header.hullFreeHead;
    world->hulls.hullPool.freeCount = header.hullFreeCount;
    world->hulls.hullPool.retiredCount = header.hullRetiredCount;
    if (WalkBlocks(world, NULL, (const uint8_t*)data + sizeof(header), m3_walkRead, 1) < 0)
    {
        return false; // the pre-parse above makes this unreachable,
                      // except for exhausted memory in the alloc gate
    }
    // The frozen-pair buffer is derived from a pair list this
    // restore just invalidated; the next update must requery the
    // whole tree once and re-harvest.
    world->contacts.sleepingPairCount = 0;
    world->contacts.pairsFullQuery = 1;
    // Derived data follows content: the per-mesh BVHs are rebuilt
    // from the restored triangle sets (a pure function, so the tree
    // a restore produces is byte-identical to the one the original
    // create produced).
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
    for (int32_t s2 = 0; s2 < world->shapes.shapePool.maxIndex; ++s2)
    {
        if (world->shapes.shapePool.alive[s2] != 0 && world->shapes.shapeVoxelIndex[s2] >= 0)
        {
            world->voxels.voxelShape[world->shapes.shapeVoxelIndex[s2]] = s2;
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
        // Tuning knobs fold only off their defaults.
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
        // The angular cap folds in its own block, so the knob block
        // above stays the same size for worlds that change only it.
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
    int32_t maxIndex = world->bodies.bodyPool.maxIndex;
    for (int32_t i = 0; i < maxIndex; ++i)
    {
        uint8_t alive = world->bodies.bodyPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->bodies.transforms[i], (int32_t)sizeof(m3Transform));
        h = m3Hash64(h, &world->bodies.linearVelocities[i], (int32_t)sizeof(m3Vec3));
        if (world->bodies.bodyForce[i].x != 0.0f || world->bodies.bodyForce[i].y != 0.0f ||
            world->bodies.bodyForce[i].z != 0.0f || world->bodies.bodyTorque[i].x != 0.0f ||
            world->bodies.bodyTorque[i].y != 0.0f || world->bodies.bodyTorque[i].z != 0.0f)
        {
            // Pending host forces fold only when present (a mid-step
            // snapshot must carry them).
            h = m3Hash64(h, &world->bodies.bodyForce[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->bodies.bodyTorque[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->bodies.bodyEnabled[i] == 0 || world->bodies.bodyLocks[i] != 0 ||
            world->bodies.bodySleepThreshold[i] != M3_SLEEP_VELOCITY_DEFAULT ||
            world->bodies.bodyCanSleep[i] == 0 || world->bodies.bodyHasTarget[i] != 0)
        {
            // Control state folds only off its defaults.
            h = m3Hash64(h, &world->bodies.bodyEnabled[i], 1);
            h = m3Hash64(h, &world->bodies.bodyLocks[i], 1);
            h = m3Hash64(h, &world->bodies.bodySleepThreshold[i], 4);
            h = m3Hash64(h, &world->bodies.bodyCanSleep[i], 1);
            h = m3Hash64(h, &world->bodies.bodyHasTarget[i], 1);
            h = m3Hash64(h, &world->bodies.bodyTarget[i], (int32_t)sizeof(m3Transform));
        }
        h = m3Hash64(h, &world->bodies.angularVelocities[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->bodies.invMass[i], (int32_t)sizeof(m3real));
        h = m3Hash64(h, &world->bodies.invInertiaLocal[i], (int32_t)sizeof(m3Mat3));
        h = m3Hash64(h, &world->bodies.localCenters[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->bodies.types[i], 1);
        h = m3Hash64(h, &world->bodies.bulletFlags[i], 1);
        h = m3Hash64(h, &world->bodies.awake[i], 1);
        h = m3Hash64(h, &world->bodies.sleepTimes[i], 4);
    }
    int32_t maxShape = world->shapes.shapePool.maxIndex;
    for (int32_t i = 0; i < maxShape; ++i)
    {
        uint8_t alive = world->shapes.shapePool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->shapes.shapeBody[i], 4);
        h = m3Hash64(h, &world->shapes.shapeType[i], 1);
        h = m3Hash64(h, &world->shapes.shapeGeom[i], (int32_t)sizeof(m3ShapeGeom));
        h = m3Hash64(h, &world->shapes.shapeDensity[i], 4);
        h = m3Hash64(h, &world->shapes.shapeFriction[i], 4);
        h = m3Hash64(h, &world->shapes.shapeRestitution[i], 4);
        if (world->shapes.shapeHitEvents[i] != 0 || world->shapes.shapePreSolve[i] != 0)
        {
            // Event flags are observable shape state: they fold
            // off-default (additive rule, seventh use).
            h = m3Hash64(h, &world->shapes.shapeHitEvents[i], 1);
            h = m3Hash64(h, &world->shapes.shapePreSolve[i], 1);
        }
        if (world->shapes.shapeSurfaceVel[i].x != 0.0f ||
            world->shapes.shapeSurfaceVel[i].y != 0.0f ||
            world->shapes.shapeSurfaceVel[i].z != 0.0f)
        {
            h = m3Hash64(h, &world->shapes.shapeSurfaceVel[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->shapes.shapeHasOffset[i] != 0)
        {
            // Compound offsets fold off-identity (additive rule,
            // tenth use).
            h = m3Hash64(h, &world->shapes.shapeLocalPos[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->shapes.shapeLocalRot[i], (int32_t)sizeof(m3Quat));
        }
        if (world->shapes.shapeRollingResistance[i] != 0.0f)
        {
            // Rolling resistance folds only where it is set.
            h = m3Hash64(h, &world->shapes.shapeRollingResistance[i], 4);
        }
        if (world->shapes.shapeCategory[i] != 1ull || world->shapes.shapeMask[i] != ~0ull ||
            world->shapes.shapeGroup[i] != 0)
        {
            // Same rule for filters: default-filtered shapes
            // keep every pre-existing hash still.
            h = m3Hash64(h, &world->shapes.shapeCategory[i], 8);
            h = m3Hash64(h, &world->shapes.shapeMask[i], 8);
            h = m3Hash64(h, &world->shapes.shapeGroup[i], 4);
        }
        h = m3Hash64(h, &world->shapes.shapeHullIndex[i], 4);
        h = m3Hash64(h, &world->shapes.shapeMeshIndex[i], 4);
        h = m3Hash64(h, &world->shapes.shapeSensor[i], 1);
        if (world->shapes.shapeType[i] == (uint8_t)m3_voxelShape)
        {
            // Folded only for the new type: pre-voxel scenes keep
            // their exact hash input set (the golden must not move
            // for worlds that never touch voxels).
            h = m3Hash64(h, &world->shapes.shapeVoxelIndex[i], 4);
        }
    }

    // Character state: live slots only, the additive-state rule.
    int32_t maxChar = world->characters.charPool.maxIndex;
    for (int32_t i = 0; i < maxChar; ++i)
    {
        uint8_t alive = world->characters.charPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->characters.charBody[i], 4);
        h = m3Hash64(h, &world->characters.charRadius[i], 4);
        h = m3Hash64(h, &world->characters.charHalfHeight[i], 4);
        h = m3Hash64(h, &world->characters.charCosSlope[i], 4);
        h = m3Hash64(h, &world->characters.charSnap[i], 4);
        h = m3Hash64(h, &world->characters.charSkin[i], 4);
        h = m3Hash64(h, &world->characters.charStepHeight[i], 4);
        h = m3Hash64(h, &world->characters.charGrounded[i], 1);
        h = m3Hash64(h, &world->characters.charGroundNormal[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->characters.charMass[i], 4);
        h = m3Hash64(h, &world->characters.charPushMax[i], 4);
        h = m3Hash64(h, &world->characters.charGroundBody[i], 4);
        h = m3Hash64(h, &world->characters.charGroundGen[i], 2);
    }
    for (int32_t i = 0; i < world->softBodies.softPool.maxIndex; ++i)
    {
        if (world->softBodies.softPool.alive[i] == 0)
        {
            continue; // live slots only
        }
        h = m3Hash64(h, &world->softBodies.softParticleCount[i], 4);
        h = m3Hash64(h, &world->softBodies.softCompliance[i], 4);
        if (world->softBodies.softBendCompliance[i] != 0.0f)
        {
            // Bend tethers fold off-default, their own block.
            h = m3Hash64(h, &world->softBodies.softBendStart[i], 4);
            h = m3Hash64(h, &world->softBodies.softBendCompliance[i], 4);
        }
        if (world->softBodies.softPressure[i] != 0.0f)
        {
            // Pressure folds off-default, its own block.
            h = m3Hash64(h, &world->softBodies.softDimX[i], 2);
            h = m3Hash64(h, &world->softBodies.softDimY[i], 2);
            h = m3Hash64(h, &world->softBodies.softDimZ[i], 2);
            h = m3Hash64(h, &world->softBodies.softRestVolume[i], 4);
            h = m3Hash64(h, &world->softBodies.softPressure[i], 4);
        }
        if (world->softBodies.softMaxDeviation[i] != 0.0f)
        {
            // The tether folds off-default, its own block;
            // bind positions join only then (dead weight otherwise).
            h = m3Hash64(h, &world->softBodies.softMaxDeviation[i], 4);
            h = m3Hash64(h, &world->softBodies.softBindPos[i * M3_SOFTBODY_MAX_PARTICLES],
                         world->softBodies.softParticleCount[i] * (int32_t)sizeof(m3Pos3));
        }
        if (world->softBodies.softTetCount[i] > 0)
        {
            // Tets fold off-default, their own block.
            h = m3Hash64(h, &world->softBodies.softTetCount[i], 4);
            int32_t tbase = i * M3_SOFTBODY_MAX_TETS;
            h = m3Hash64(h, &world->softBodies.softTetA[tbase],
                         2 * world->softBodies.softTetCount[i]);
            h = m3Hash64(h, &world->softBodies.softTetB[tbase],
                         2 * world->softBodies.softTetCount[i]);
            h = m3Hash64(h, &world->softBodies.softTetC[tbase],
                         2 * world->softBodies.softTetCount[i]);
            h = m3Hash64(h, &world->softBodies.softTetD[tbase],
                         2 * world->softBodies.softTetCount[i]);
            h = m3Hash64(h, &world->softBodies.softTetRestV6[tbase],
                         4 * world->softBodies.softTetCount[i]);
        }
        h = m3Hash64(h, &world->softBodies.softRadius[i], 4);
        h = m3Hash64(h, &world->softBodies.softGravityScale[i], 4);
        int32_t pc = world->softBodies.softParticleCount[i];
        for (int32_t p = 0; p < pc; ++p)
        {
            int32_t k = i * M3_SOFTBODY_MAX_PARTICLES + p;
            h = m3Hash64(h, &world->softBodies.softPos[k], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->softBodies.softPrev[k], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->softBodies.softInvMass[k], 4);
            if (world->softBodies.softKick[k].x != 0.0f ||
                world->softBodies.softKick[k].y != 0.0f || world->softBodies.softKick[k].z != 0.0f)
            {
                // Pending kicks fold only while they exist:
                // the window between a blast and its next step is
                // real rollback state, everything else is silence.
                h = m3Hash64(h, &world->softBodies.softKick[k], (int32_t)sizeof(m3Vec3));
            }
        }
        int32_t ec = world->softBodies.softEdgeCount[i];
        for (int32_t e = 0; e < ec; ++e)
        {
            int32_t k = i * M3_SOFTBODY_MAX_EDGES + e;
            h = m3Hash64(h, &world->softBodies.softEdgeA[k], 2);
            h = m3Hash64(h, &world->softBodies.softEdgeB[k], 2);
            h = m3Hash64(h, &world->softBodies.softEdgeRest[k], 4);
        }
        int32_t ac = world->softBodies.softAnchorCount[i];
        for (int32_t a = 0; a < ac; ++a)
        {
            int32_t k = i * M3_SOFTBODY_MAX_ANCHORS + a;
            h = m3Hash64(h, &world->softBodies.softAnchorParticle[k], 4);
            h = m3Hash64(h, &world->softBodies.softAnchorBody[k], 4);
            h = m3Hash64(h, &world->softBodies.softAnchorGen[k], 2);
            h = m3Hash64(h, &world->softBodies.softAnchorLocal[k], (int32_t)sizeof(m3Vec3));
        }
        // Soft-to-soft anchors fold off-empty (additive rule).
        int32_t sc = world->softBodies.softSoftCount[i];
        for (int32_t a = 0; a < sc; ++a)
        {
            int32_t k = i * M3_SOFTBODY_MAX_ANCHORS + a;
            h = m3Hash64(h, &world->softBodies.softSoftParticleA[k], 4);
            h = m3Hash64(h, &world->softBodies.softSoftSlotB[k], 4);
            h = m3Hash64(h, &world->softBodies.softSoftGenB[k], 2);
            h = m3Hash64(h, &world->softBodies.softSoftParticleB[k], 4);
        }
    }
    for (int32_t i = 0; i < world->vehicles.vehPool.maxIndex; ++i)
    {
        if (world->vehicles.vehPool.alive[i] == 0)
        {
            continue; // live slots only
        }
        h = m3Hash64(h, &world->vehicles.vehChassis[i], 4);
        h = m3Hash64(h, &world->vehicles.vehChassisGen[i], 2);
        h = m3Hash64(h, &world->vehicles.vehWheelCount[i], 4);
        h = m3Hash64(h, &world->vehicles.vehMaxSteer[i], 4);
        h = m3Hash64(h, &world->vehicles.vehDriveForce[i], 4);
        h = m3Hash64(h, &world->vehicles.vehBrakeForce[i], 4);
        h = m3Hash64(h, &world->vehicles.vehTireGrip[i], 4);
        h = m3Hash64(h, &world->vehicles.vehThrottle[i], 4);
        if (world->vehicles.vehTrackMode[i] != 0)
        {
            // Tank mode folds off-default, its own block.
            h = m3Hash64(h, &world->vehicles.vehTrackMode[i], 1);
            h = m3Hash64(h, &world->vehicles.vehTrackLeft[i], 4);
            h = m3Hash64(h, &world->vehicles.vehTrackRight[i], 4);
        }
        if (world->vehicles.vehLeanGain[i] != 0.0f)
        {
            // The lean gain folds off-default, its own block.
            h = m3Hash64(h, &world->vehicles.vehLeanGain[i], 4);
        }
        h = m3Hash64(h, &world->vehicles.vehSteer[i], 4);
        h = m3Hash64(h, &world->vehicles.vehBrake[i], 4);
        for (int32_t w = 0; w < world->vehicles.vehWheelCount[i]; ++w)
        {
            int32_t k = i * M3_VEHICLE_MAX_WHEELS + w;
            h = m3Hash64(h, &world->vehicles.vehWheelAnchor[k], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->vehicles.vehWheelDir[k], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->vehicles.vehWheelRest[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelTravel[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelHertz[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelZeta[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelRadius[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelFlags[k], 1);
            h = m3Hash64(h, &world->vehicles.vehWheelBrake[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelCompression[k], 4);
            h = m3Hash64(h, &world->vehicles.vehWheelContact[k], 1);
            h = m3Hash64(h, &world->vehicles.vehWheelSpin[k], 4);
        }
        if (world->vehicles.vehDtActive[i] != 0)
        {
            // The drivetrain hashes only when attached.
            h = m3Hash64(h, &world->vehicles.vehDtCurveCount[i], 4);
            for (int32_t c = 0; c < world->vehicles.vehDtCurveCount[i]; ++c)
            {
                h = m3Hash64(h, &world->vehicles.vehDtCurveRpm[i * M3_DRIVETRAIN_MAX_CURVE + c], 4);
                h = m3Hash64(h, &world->vehicles.vehDtCurveTorque[i * M3_DRIVETRAIN_MAX_CURVE + c],
                             4);
            }
            h = m3Hash64(h, &world->vehicles.vehDtGearCount[i], 4);
            for (int32_t g = 0; g < world->vehicles.vehDtGearCount[i]; ++g)
            {
                h = m3Hash64(h, &world->vehicles.vehDtGearRatio[i * M3_DRIVETRAIN_MAX_GEARS + g],
                             4);
            }
            h = m3Hash64(h, &world->vehicles.vehDtReverse[i], 4);
            h = m3Hash64(h, &world->vehicles.vehDtFinal[i], 4);
            if (world->vehicles.vehDtDiffMode[i] != 0)
            {
                // The diff folds only when engaged, wheel
                // speeds included: they steer forces only then.
                h = m3Hash64(h, &world->vehicles.vehDtDiffMode[i], 4);
                h = m3Hash64(h, &world->vehicles.vehDtDiffCouple[i], 4);
                for (int32_t wq = 0; wq < world->vehicles.vehWheelCount[i]; ++wq)
                {
                    h = m3Hash64(h, &world->vehicles.vehWheelLon[i * M3_VEHICLE_MAX_WHEELS + wq],
                                 4);
                }
            }
            h = m3Hash64(h, &world->vehicles.vehDtShiftUp[i], 4);
            h = m3Hash64(h, &world->vehicles.vehDtShiftDown[i], 4);
            h = m3Hash64(h, &world->vehicles.vehDtClutchSteps[i], 4);
            h = m3Hash64(h, &world->vehicles.vehDtAutoShift[i], 1);
            h = m3Hash64(h, &world->vehicles.vehDtGear[i], 1);
            h = m3Hash64(h, &world->vehicles.vehDtClutch[i], 4);
            h = m3Hash64(h, &world->vehicles.vehDtRpm[i], 4);
        }
    }

    // Voxel chunk content is simulation state (destruction edits it
    // and rollback must cover it); live slots only, same law as
    // everything above.
    int32_t maxVoxel = world->voxels.voxelPool.maxIndex;
    for (int32_t i = 0; i < maxVoxel; ++i)
    {
        uint8_t alive = world->voxels.voxelPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->voxels.voxelData[i].cellSize, 4);
        h = m3Hash64(h, &world->voxels.voxelData[i].filledCount, 4);
        h = m3Hash64(h, world->voxels.voxelData[i].occupancy,
                     (int32_t)sizeof(world->voxels.voxelData[i].occupancy));
        h = m3Hash64(h, world->voxels.voxelData[i].payload,
                     (int32_t)sizeof(world->voxels.voxelData[i].payload));
        h = m3Hash64(h, world->voxels.voxelData[i].fill,
                     (int32_t)sizeof(world->voxels.voxelData[i].fill));
    }
    int32_t maxJoint = world->joints.jointPool.maxIndex;
    for (int32_t i = 0; i < maxJoint; ++i)
    {
        uint8_t alive = world->joints.jointPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->joints.jointType[i], 1);
        h = m3Hash64(h, &world->joints.jointBodyA[i], 4);
        h = m3Hash64(h, &world->joints.jointBodyB[i], 4);
        h = m3Hash64(h, &world->joints.jointLocalA[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointLocalB[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointImpulse[i], (int32_t)sizeof(m3Vec3));
        if (world->joints.jointBreak[i].x != 0.0f || world->joints.jointBreak[i].y != 0.0f)
        {
            // Break thresholds fold off-default (additive rule,
            // eighth use).
            h = m3Hash64(h, &world->joints.jointBreak[i], (int32_t)sizeof(m3Vec3));
        }
        if ((world->joints.jointFlags[i] & M3_JOINT_SPRING) != 0 ||
            world->joints.jointTargetScalar[i] != 0.0f || world->joints.jointTargetQ[i].x != 0.0f ||
            world->joints.jointTargetQ[i].y != 0.0f || world->joints.jointTargetQ[i].z != 0.0f ||
            world->joints.jointTargetQ[i].w != 1.0f ||
            world->joints.jointSpringImpulse[i].x != 0.0f ||
            world->joints.jointSpringImpulse[i].y != 0.0f ||
            world->joints.jointSpringImpulse[i].z != 0.0f)
        {
            // Drive springs fold off-default (additive rule, ninth
            // use). The impulse joins: it is dynamics state.
            h = m3Hash64(h, &world->joints.jointSpring[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->joints.jointTargetScalar[i], 4);
            h = m3Hash64(h, &world->joints.jointTargetQ[i], (int32_t)sizeof(m3Quat));
            h = m3Hash64(h, &world->joints.jointSpringImpulse[i], (int32_t)sizeof(m3Vec3));
        }
        h = m3Hash64(h, &world->joints.jointPerpImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointLimitImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointAngularImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->joints.jointFrameQA[i], (int32_t)sizeof(m3Quat));
        h = m3Hash64(h, &world->joints.jointFrameQB[i], (int32_t)sizeof(m3Quat));
        h = m3Hash64(h, &world->joints.jointFlags[i], 1);
        if (world->joints.jointType[i] == (uint8_t)m3_genericJoint)
        {
            // The generic joint's fields fold only for that type.
            h = m3Hash64(h, &world->joints.jointGenericModes[i], 2);
            h = m3Hash64(h, &world->joints.jointGenLinLower[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->joints.jointGenLinUpper[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->joints.jointGenAngLower[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->joints.jointGenAngUpper[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->joints.jointType[i] == (uint8_t)m3_pulleyJoint)
        {
            // The rope's world anchors exist only under a pulley.
            h = m3Hash64(h, &world->joints.jointGroundA[i], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->joints.jointGroundB[i], (int32_t)sizeof(m3Pos3));
        }
    }

    // Water volumes: folded ONLY while any volume is alive
    // (the off-default law, in its own block); the pool identity
    // rides along so a destroyed-and-recreated volume moves bits.
    {
        int32_t waterAlive = 0;
        for (int32_t k = 0; k < world->water.waterPool.maxIndex; ++k)
        {
            waterAlive += world->water.waterPool.alive[k];
        }
        if (waterAlive > 0)
        {
            h = m3Hash64(h, world->water.waterLo, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, world->water.waterHi, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, world->water.waterDensity,
                         M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->water.waterLinDrag,
                         M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->water.waterAngDrag,
                         M3_MAX_WATER_VOLUMES * (int32_t)sizeof(float));
            h = m3Hash64(h, world->water.waterFlow, M3_MAX_WATER_VOLUMES * (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, world->water.waterPool.alive, M3_MAX_WATER_VOLUMES);
        }
    }

    // Pairs and manifolds: warm-start impulses are simulation state
    // (they steer the next solve), so they are part of what the world
    // IS.
    h = m3Hash64(h, &world->contacts.pairCount, 4);
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        h = m3Hash64(h, &world->contacts.pairKeys[i], 8);
        h = m3Hash64(h, &world->contacts.manifolds[i], (int32_t)sizeof(m3Manifold));
    }
    return h;
}
