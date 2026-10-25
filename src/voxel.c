// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The voxel chunk: the destruction niche's foundation. State
// is a dense 16x16x16 occupancy bitset plus a uint16 payload per
// voxel (one padding-free snapshot block per chunk slot). The
// collision surface is DERIVED data under the standing law: a
// deterministic greedy sweep merges filled voxels into maximal
// boxes, a BVH goes over the boxes, and both are rebuilt as pure
// functions of the grid wherever content lands (create, journal
// replay, restore), so twin worlds agree byte for byte. Boxes are
// axis-aligned in the chunk frame BY CONSTRUCTION, which is what
// makes the sphere kernel in manifold.c an exact clamp instead of
// an iteration.

#include "voxel.h"

#include "body_state.h"
#include "character.h"
#include "fpenv.h"
#include "hull.h"
#include "journal.h"
#include "misuse.h"
#include "shape_state.h"
#include "voxel_coverage.h"
#include "world_internal.h"

#include <string.h>

bool m3VoxelGet(const m3VoxelChunkData* chunk, int32_t x, int32_t y, int32_t z)
{
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    return (chunk->occupancy[v >> 3] & (uint8_t)(1u << (v & 7))) != 0;
}

static void VoxelSet(m3VoxelChunkData* chunk, int32_t v)
{
    chunk->occupancy[v >> 3] |= (uint8_t)(1u << (v & 7));
}

// Pack a caller-friendly grid (one byte per voxel, zero = empty)
// into chunk state. Returns the filled count.
int32_t m3VoxelPack(m3VoxelChunkData* chunk, const uint8_t* voxels, const uint16_t* payload,
                    m3real cellSize)
{
    memset(chunk, 0, sizeof(*chunk));
    chunk->cellSize = cellSize;
    int32_t filled = 0;
    for (int32_t v = 0; v < M3_VOXEL_COUNT; ++v)
    {
        if (voxels[v] != 0)
        {
            VoxelSet(chunk, v);
            chunk->payload[v] = payload != NULL ? payload[v] : 0;
            chunk->fill[v] = 255; // whole voxels at birth; SetFill wears them down
            filled += 1;
        }
    }
    chunk->filledCount = filled;
    return filled;
}

// Counts the occupancy bits. Every occupancy edit rebuilds the surface, so
// the build checks the edit's filled count against them. Release builds
// compile the check out, and the helper with it.
#ifndef NDEBUG
static int32_t FilledVoxels(const m3VoxelChunkData* chunk)
{
    int32_t filled = 0;
    for (int32_t v = 0; v < M3_VOXEL_COUNT; ++v)
    {
        filled += (chunk->occupancy[v >> 3] >> (v & 7)) & 1;
    }
    return filled;
}
#define ASSERT_FILLED_COUNT(chunk) M3_ASSERT(FilledVoxels(chunk) == (chunk)->filledCount)
#else
#define ASSERT_FILLED_COUNT(chunk) ((void)(chunk))
#endif

// The midphase over the merged boxes, built into the slot's own tree
// storage.
static void BuildSurfaceTree(m3World* world, int32_t slot, m3VoxelSurface* surface, m3real cell)
{
    if (surface->boxCount == 0)
    {
        return; // an empty chunk keeps the empty tree, which gathers nothing
    }
    // Bounds per merged box, in chunk-frame meters.
    m3Vec3 los[M3_VOXEL_MAX_BOXES];
    m3Vec3 his[M3_VOXEL_MAX_BOXES];
    for (int32_t b = 0; b < surface->boxCount; ++b)
    {
        los[b] = (m3Vec3){(m3real)surface->boxLo[b][0] * cell, (m3real)surface->boxLo[b][1] * cell,
                          (m3real)surface->boxLo[b][2] * cell};
        his[b] = (m3Vec3){(m3real)(surface->boxHi[b][0] + 1) * cell,
                          (m3real)(surface->boxHi[b][1] + 1) * cell,
                          (m3real)(surface->boxHi[b][2] + 1) * cell};
    }
    uint8_t* tree = (uint8_t*)world->voxels.voxelTrees[slot];
    M3_ASSERT(tree != NULL); // taken when the chunk landed
    surface->bvh.nodes = (m3MeshBvhNode*)tree;
    surface->bvh.order = (uint16_t*)(tree + 2 * M3_VOXEL_MAX_BOXES * sizeof(m3MeshBvhNode));
    m3MeshBvhBuildIn(&surface->bvh, world->voxels.voxelTreeScratch, los, his, surface->boxCount);
}

// The greedy merge: scan in canonical order (x fastest, then y,
// then z); from each unclaimed filled voxel grow a run along x,
// widen it along y, then deepen it along z, claiming as it goes.
// Deterministic by the fixed scan and growth order; maximal in the
// greedy sense (not globally optimal, which is NP-hard and
// unnecessary).
//
// Writes the boxes to surface unless it is NULL, and returns how many
// there are, or -1 as soon as one more would pass limit, with no box
// written past it.
static int32_t MergeBoxes(const m3VoxelChunkData* chunk, m3VoxelSurface* surface, int32_t limit)
{
    int32_t count = 0;
    uint8_t claimed[M3_VOXEL_COUNT];
    memset(claimed, 0, sizeof(claimed));

    for (int32_t z = 0; z < M3_VOXEL_DIM; ++z)
    {
        for (int32_t y = 0; y < M3_VOXEL_DIM; ++y)
        {
            for (int32_t x = 0; x < M3_VOXEL_DIM; ++x)
            {
                int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
                if (claimed[v] != 0 || !m3VoxelGet(chunk, x, y, z))
                {
                    continue;
                }
                // Grow the run along x.
                int32_t x2 = x;
                while (x2 + 1 < M3_VOXEL_DIM && !claimed[v + (x2 - x) + 1] &&
                       m3VoxelGet(chunk, x2 + 1, y, z))
                {
                    x2 += 1;
                }
                // Widen along y: every column of the candidate row
                // must be filled and unclaimed.
                int32_t y2 = y;
                while (y2 + 1 < M3_VOXEL_DIM)
                {
                    bool rowOk = true;
                    for (int32_t xi = x; xi <= x2; ++xi)
                    {
                        int32_t vi = xi + M3_VOXEL_DIM * ((y2 + 1) + M3_VOXEL_DIM * z);
                        if (claimed[vi] != 0 || !m3VoxelGet(chunk, xi, y2 + 1, z))
                        {
                            rowOk = false;
                            break;
                        }
                    }
                    if (!rowOk)
                    {
                        break;
                    }
                    y2 += 1;
                }
                // Deepen along z: every cell of the candidate slab.
                int32_t z2 = z;
                while (z2 + 1 < M3_VOXEL_DIM)
                {
                    bool slabOk = true;
                    for (int32_t yi = y; yi <= y2 && slabOk; ++yi)
                    {
                        for (int32_t xi = x; xi <= x2; ++xi)
                        {
                            int32_t vi = xi + M3_VOXEL_DIM * (yi + M3_VOXEL_DIM * (z2 + 1));
                            if (claimed[vi] != 0 || !m3VoxelGet(chunk, xi, yi, z2 + 1))
                            {
                                slabOk = false;
                                break;
                            }
                        }
                    }
                    if (!slabOk)
                    {
                        break;
                    }
                    z2 += 1;
                }
                // Claim and emit.
                for (int32_t zi = z; zi <= z2; ++zi)
                {
                    for (int32_t yi = y; yi <= y2; ++yi)
                    {
                        for (int32_t xi = x; xi <= x2; ++xi)
                        {
                            claimed[xi + M3_VOXEL_DIM * (yi + M3_VOXEL_DIM * zi)] = 1;
                        }
                    }
                }
                if (count == limit)
                {
                    return -1;
                }
                if (surface != NULL)
                {
                    surface->boxLo[count][0] = (uint8_t)x;
                    surface->boxLo[count][1] = (uint8_t)y;
                    surface->boxLo[count][2] = (uint8_t)z;
                    surface->boxHi[count][0] = (uint8_t)x2;
                    surface->boxHi[count][1] = (uint8_t)y2;
                    surface->boxHi[count][2] = (uint8_t)z2;
                }
                count += 1;
            }
        }
    }

    return count;
}

bool m3VoxelSurfaceFits(const m3World* world, const m3VoxelChunkData* chunk)
{
    return MergeBoxes(chunk, NULL, world->voxels.voxelBoxLimit) >= 0;
}

bool m3VoxelTreeReserve(m3World* world, int32_t slot)
{
    if (world->voxels.voxelTrees[slot] == NULL)
    {
        world->voxels.voxelTrees[slot] = m3AllocZeroed(&world->heap, M3_VOXEL_TREE_BYTES);
    }
    return world->voxels.voxelTrees[slot] != NULL;
}

void m3VoxelSurfaceBuild(m3World* world, int32_t slot)
{
    const m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
    m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
    ASSERT_FILLED_COUNT(chunk);
    memset(surface, 0, sizeof(*surface));
    // The limit never passes the storage, so no build writes past it; a
    // grid that would pass the limit never lands (m3VoxelSurfaceFits).
    int32_t limit = world->voxels.voxelBoxLimit;
    int32_t count = MergeBoxes(chunk, surface, limit);
    M3_ASSERT(count >= 0);
    surface->boxCount = count >= 0 ? count : limit;
    BuildSurfaceTree(world, slot, surface, chunk->cellSize);
}

// Chunk-frame bounds of one merged box.
void m3VoxelBoxBounds(const m3VoxelSurface* surface, m3real cellSize, int32_t box, m3Vec3* lo,
                      m3Vec3* hi)
{
    *lo = (m3Vec3){(m3real)surface->boxLo[box][0] * cellSize,
                   (m3real)surface->boxLo[box][1] * cellSize,
                   (m3real)surface->boxLo[box][2] * cellSize};
    *hi = (m3Vec3){(m3real)(surface->boxHi[box][0] + 1) * cellSize,
                   (m3real)(surface->boxHi[box][1] + 1) * cellSize,
                   (m3real)(surface->boxHi[box][2] + 1) * cellSize};
}

// A merged box as hull data in the CHUNK frame (m3BuildBoxHull
// makes an origin-centered box; translate the vertices and shift
// the face plane offsets by the same center).
void m3VoxelBoxHull(const m3VoxelSurface* surface, m3real cellSize, int32_t box, m3HullData* out)
{
    m3Vec3 lo;
    m3Vec3 hi;
    m3VoxelBoxBounds(surface, cellSize, box, &lo, &hi);
    m3Vec3 half = m3MulSV3(0.5f, m3Sub3(hi, lo));
    m3Vec3 center = m3MulSV3(0.5f, m3Add3(lo, hi));
    m3BuildBoxHull(out, half);
    for (int32_t v = 0; v < out->vertexCount; ++v)
    {
        out->vertices[v] = m3Add3(out->vertices[v], center);
    }
    for (int32_t f = 0; f < out->faceCount; ++f)
    {
        out->faceOffsets[f] += m3Dot3(out->faceNormals[f], center);
    }
    out->center = m3Add3(out->center, center);
}

// ---------------------------------------------------------------
// Edits: deterministic state transitions.

// Wake every dynamic body whose fat bounds touch the edited region
// (world frame): a disturbance is a disturbance even for sleepers.
typedef struct m3VoxelWakeContext
{
    m3World* world;
} m3VoxelWakeContext;

static bool VoxelWakeCallback(int32_t shape, void* userContext)
{
    m3World* world = ((m3VoxelWakeContext*)userContext)->world;
    int32_t body = world->shapes.shapeBody[shape];
    m3WakeIfDynamic(world, body);
    return true;
}

static void VoxelWakeRegion(m3World* world, int32_t shape, const int32_t lo[3], const int32_t hi[3])
{
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3real cell = world->voxels.voxelData[slot].cellSize;
    int32_t body = world->shapes.shapeBody[shape];
    const m3Transform* xf = &world->bodies.transforms[body];
    // The region's eight corners in the chunk frame, rotated out,
    // padded by the speculative margin so grazing sleepers wake too.
    double wlo[3] = {1.0e30, 1.0e30, 1.0e30};
    double whi[3] = {-1.0e30, -1.0e30, -1.0e30};
    for (int32_t c = 0; c < 8; ++c)
    {
        m3Vec3 corner = {(m3real)((c & 1) != 0 ? hi[0] + 1 : lo[0]) * cell,
                         (m3real)((c & 2) != 0 ? hi[1] + 1 : lo[1]) * cell,
                         (m3real)((c & 4) != 0 ? hi[2] + 1 : lo[2]) * cell};
        m3Vec3 r = m3RotateVec3(xf->q, corner);
        double p[3] = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
        for (int32_t k = 0; k < 3; ++k)
        {
            wlo[k] = p[k] < wlo[k] ? p[k] : wlo[k];
            whi[k] = p[k] > whi[k] ? p[k] : whi[k];
        }
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        wlo[k] -= (double)M3_AABB_MARGIN;
        whi[k] += (double)M3_AABB_MARGIN;
    }
    m3VoxelWakeContext ctx = {world};
    m3TreeQuery(&world->broadphase.tree, wlo, whi, VoxelWakeCallback, &ctx);

    // Characters standing over the edited region lose their ground
    // THIS step if it vanished: the destruction interplay is
    // a contract, not a next-frame coincidence.
    for (int32_t c = 0; c < world->characters.charPool.maxIndex; ++c)
    {
        if (world->characters.charPool.alive[c] == 0 || world->characters.charGrounded[c] == 0)
        {
            continue;
        }
        const m3Pos3* p = &world->bodies.transforms[world->characters.charBody[c]].p;
        m3real reachDown = world->characters.charHalfHeight[c] + world->characters.charRadius[c] +
                           world->characters.charSnap[c] + M3_AABB_MARGIN;
        m3real reachSide = world->characters.charRadius[c] + M3_AABB_MARGIN;
        if (p->x + (double)reachSide < wlo[0] || p->x - (double)reachSide > whi[0] ||
            p->z + (double)reachSide < wlo[2] || p->z - (double)reachSide > whi[2] ||
            p->y - (double)reachDown > whi[1] || p->y + (double)reachDown < wlo[1])
        {
            continue;
        }
        m3CharacterRefreshGrounding(world, c);
    }

    // A parked car hovers on wheel RAYS: its chassis bounds may sit
    // well above the carved region and the tree wake would leave
    // the sleeper floating on vanished floor. Any wheel ray
    // overlapping the region wakes the chassis; the next
    // suspension pass reads the new surface the same step.
    for (int32_t v = 0; v < world->vehicles.vehPool.maxIndex; ++v)
    {
        if (world->vehicles.vehPool.alive[v] == 0)
        {
            continue;
        }
        int32_t chassis = world->vehicles.vehChassis[v];
        if (chassis < 0 || world->bodies.bodyPool.alive[chassis] == 0 ||
            world->bodies.bodyPool.generations[chassis] != world->vehicles.vehChassisGen[v] ||
            world->bodies.awake[chassis] != 0)
        {
            continue;
        }
        const m3Transform* cxf = &world->bodies.transforms[chassis];
        for (int32_t w = 0; w < world->vehicles.vehWheelCount[v]; ++w)
        {
            int32_t k = v * M3_VEHICLE_MAX_WHEELS + w;
            m3Vec3 anchorR = m3RotateVec3(cxf->q, world->vehicles.vehWheelAnchor[k]);
            double ax = cxf->p.x + (double)anchorR.x;
            double ay = cxf->p.y + (double)anchorR.y;
            double az = cxf->p.z + (double)anchorR.z;
            m3Vec3 dir = m3RotateVec3(cxf->q, world->vehicles.vehWheelDir[k]);
            m3real reach = world->vehicles.vehWheelRest[k] + world->vehicles.vehWheelRadius[k];
            double ex = ax + (double)(dir.x * reach);
            double ey = ay + (double)(dir.y * reach);
            double ez = az + (double)(dir.z * reach);
            double m = (double)M3_AABB_MARGIN;
            double slo[3] = {(ax < ex ? ax : ex) - m, (ay < ey ? ay : ey) - m,
                             (az < ez ? az : ez) - m};
            double shi[3] = {(ax > ex ? ax : ex) + m, (ay > ey ? ay : ey) + m,
                             (az > ez ? az : ez) + m};
            if (slo[0] <= whi[0] && shi[0] >= wlo[0] && slo[1] <= whi[1] && shi[1] >= wlo[1] &&
                slo[2] <= whi[2] && shi[2] >= wlo[2])
            {
                world->bodies.awake[chassis] = 1;
                world->bodies.sleepTimes[chassis] = 0.0f;
                break;
            }
        }
    }
}

bool m3VoxelCoordsValid(int32_t x, int32_t y, int32_t z)
{
    return x >= 0 && x < M3_VOXEL_DIM && y >= 0 && y < M3_VOXEL_DIM && z >= 0 && z < M3_VOXEL_DIM;
}

// An edit lands in the world's staging copy of the chunk first: one whose
// surface would pass the box limit is refused before anything changes,
// counted and journaled by no one. Otherwise the copy lands, and the
// surface, the fracture sweep and the seam coverage follow it.
static m3VoxelChunkData* StageChunk(m3World* world, int32_t slot)
{
    m3VoxelChunkData* stage = world->voxels.voxelStage;
    *stage = world->voxels.voxelData[slot];
    return stage;
}

static bool LandStaged(m3World* world, int32_t shape, int32_t slot, bool fracture)
{
    if (!m3VoxelSurfaceFits(world, world->voxels.voxelStage))
    {
        world->voxels.voxelEditsRefused += 1;
        return false;
    }
    world->voxels.voxelData[slot] = *world->voxels.voxelStage;
    m3VoxelSurfaceBuild(world, slot);
    if (fracture)
    {
        m3VoxelFractureSweep(world, shape);
    }
    m3VoxelCoverageRefreshAround(world, slot);
    return true;
}

static void ClearCell(m3VoxelChunkData* chunk, int32_t x, int32_t y, int32_t z)
{
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
    chunk->payload[v] = 0;
    chunk->fill[v] = 0;
}

bool m3VoxelSetInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z,
                        uint16_t payload)
{
    M3_ASSERT(m3VoxelCoordsValid(x, y, z));
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    if (m3VoxelGet(&world->voxels.voxelData[slot], x, y, z))
    {
        // Payload only: the surface is a pure function of occupancy and
        // cell size, never of payload, so it stays.
        world->voxels.voxelData[slot].payload[v] = payload;
        world->voxels.voxelData[slot].fill[v] = 255; // a set voxel is a whole voxel
    }
    else
    {
        m3VoxelChunkData* chunk = StageChunk(world, slot);
        chunk->occupancy[v >> 3] |= (uint8_t)(1u << (v & 7));
        chunk->payload[v] = payload;
        chunk->fill[v] = 255;
        chunk->filledCount += 1;
        // No fracture sweep: adding a voxel can only CONNECT islands.
        if (!LandStaged(world, shape, slot, false))
        {
            return false;
        }
    }
    int32_t lo[3] = {x, y, z};
    VoxelWakeRegion(world, shape, lo, lo);
    return true;
}

bool m3VoxelClearInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z)
{
    M3_ASSERT(m3VoxelCoordsValid(x, y, z));
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    if (m3VoxelGet(&world->voxels.voxelData[slot], x, y, z))
    {
        m3VoxelChunkData* chunk = StageChunk(world, slot);
        ClearCell(chunk, x, y, z);
        chunk->filledCount -= 1;
        if (!LandStaged(world, shape, slot, true))
        {
            return false;
        }
    }
    int32_t lo[3] = {x, y, z};
    VoxelWakeRegion(world, shape, lo, lo);
    return true;
}

int32_t m3VoxelClearBoxInternal(m3World* world, int32_t shape, const int32_t lo[3],
                                const int32_t hi[3])
{
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = StageChunk(world, slot);
    int32_t cleared = 0;
    for (int32_t z = lo[2]; z <= hi[2]; ++z)
    {
        for (int32_t y = lo[1]; y <= hi[1]; ++y)
        {
            for (int32_t x = lo[0]; x <= hi[0]; ++x)
            {
                if (m3VoxelGet(chunk, x, y, z))
                {
                    ClearCell(chunk, x, y, z);
                    cleared += 1;
                }
            }
        }
    }
    if (cleared > 0)
    {
        chunk->filledCount -= cleared;
        if (!LandStaged(world, shape, slot, true))
        {
            return -1;
        }
    }
    VoxelWakeRegion(world, shape, lo, hi);
    return cleared;
}

bool m3VoxelSetFillInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z,
                            uint8_t fill)
{
    M3_ASSERT(m3VoxelCoordsValid(x, y, z));
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
    if (!m3VoxelGet(chunk, x, y, z))
    {
        return false; // fill describes an occupied voxel, nothing else
    }
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    chunk->fill[v] = fill;
    // Mass bookkeeping only: no surface rebuild (geometry is
    // untouched), no wake (a static chunk's mass moves nothing
    // dynamic until fracture delivers it).
    return true;
}

int32_t m3VoxelCarveSphereInternal(m3World* world, int32_t shape, m3Vec3 center, m3real radius)
{
    // The explosion bite: clear every cell whose center lies
    // inside the sphere (center in the CHUNK frame), then ONE
    // surface rebuild and ONE fracture sweep for the whole bite,
    // the ClearBox economy. Cell (x,y,z) spans [x, x+1) * cellSize.
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = StageChunk(world, slot);
    m3real cell = chunk->cellSize;
    int32_t lo[3];
    int32_t hi[3];
    m3real c[3] = {center.x, center.y, center.z};
    for (int32_t a = 0; a < 3; ++a)
    {
        m3real f0 = (c[a] - radius) / cell;
        m3real f1 = (c[a] + radius) / cell;
        int32_t i0 = m3CellFromF(floorf(f0), 2.0e9f);
        int32_t i1 = m3CellFromF(floorf(f1), -2.0e9f);
        lo[a] = i0 < 0 ? 0 : (i0 > M3_VOXEL_DIM - 1 ? M3_VOXEL_DIM - 1 : i0);
        hi[a] = i1 < 0 ? 0 : (i1 > M3_VOXEL_DIM - 1 ? M3_VOXEL_DIM - 1 : i1);
    }
    int32_t cleared = 0;
    m3real r2 = radius * radius;
    for (int32_t z = lo[2]; z <= hi[2]; ++z)
    {
        for (int32_t y = lo[1]; y <= hi[1]; ++y)
        {
            for (int32_t x = lo[0]; x <= hi[0]; ++x)
            {
                if (!m3VoxelGet(chunk, x, y, z))
                {
                    continue;
                }
                m3real dx = ((m3real)x + 0.5f) * cell - center.x;
                m3real dy = ((m3real)y + 0.5f) * cell - center.y;
                m3real dz = ((m3real)z + 0.5f) * cell - center.z;
                if (dx * dx + dy * dy + dz * dz > r2)
                {
                    continue;
                }
                ClearCell(chunk, x, y, z);
                cleared += 1;
            }
        }
    }
    if (cleared > 0)
    {
        chunk->filledCount -= cleared;
        if (!LandStaged(world, shape, slot, true))
        {
            return -1;
        }
    }
    VoxelWakeRegion(world, shape, lo, hi);
    return cleared;
}

// Public entries: validate, journal, apply (the command pattern).
static bool ResolveVoxelShape(const m3World* world, m3ShapeId id, int32_t* shapeOut)
{
    int32_t shape = m3ShapeSlot(world, id);
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // stale, or not a voxel chunk: contract
    }
    *shapeOut = shape;
    return true;
}

static M3_NOINLINE void VoxelChunkSetVoxelWorker(m3World* world, m3ShapeId shape, int32_t x,
                                                 int32_t y, int32_t z, uint16_t payload)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!ResolveVoxelShape(world, shape, &index))
    {
        return;
    }
    if (!m3VoxelCoordsValid(x, y, z))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (!m3VoxelSetInternal(world, index, x, y, z, payload))
    {
        return; // the surface would pass the box limit: counted, nothing changed
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVoxelSet record;
        memset(&record, 0, sizeof(record));
        record.id = shape;
        record.x = x;
        record.y = y;
        record.z = z;
        record.payload = payload;
        m3JournalRecord(world, m3_opVoxelSet, &record, (int32_t)sizeof(record));
    }
}

void m3VoxelChunk_SetVoxel(m3World* world, m3ShapeId shape, int32_t x, int32_t y, int32_t z,
                           uint16_t payload)
{
    uint64_t env = m3FloatEnvEnter();
    VoxelChunkSetVoxelWorker(world, shape, x, y, z, payload);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void VoxelChunkClearVoxelWorker(m3World* world, m3ShapeId shape, int32_t x,
                                                   int32_t y, int32_t z)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!ResolveVoxelShape(world, shape, &index))
    {
        return;
    }
    if (!m3VoxelCoordsValid(x, y, z))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (!m3VoxelClearInternal(world, index, x, y, z))
    {
        return; // the surface would pass the box limit: counted, nothing changed
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVoxelClear record;
        memset(&record, 0, sizeof(record));
        record.id = shape;
        record.x = x;
        record.y = y;
        record.z = z;
        m3JournalRecord(world, m3_opVoxelClear, &record, (int32_t)sizeof(record));
    }
}

void m3VoxelChunk_ClearVoxel(m3World* world, m3ShapeId shape, int32_t x, int32_t y, int32_t z)
{
    uint64_t env = m3FloatEnvEnter();
    VoxelChunkClearVoxelWorker(world, shape, x, y, z);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE void VoxelChunkSetFillWorker(m3World* world, m3ShapeId shape, int32_t x,
                                                int32_t y, int32_t z, uint8_t fill)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return;
    }
    int32_t index;
    if (!ResolveVoxelShape(world, shape, &index))
    {
        return;
    }
    if (!m3VoxelCoordsValid(x, y, z) || fill == 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return; // fill zero is a clear in disguise: refused
    }
    int32_t slot = world->shapes.shapeVoxelIndex[index];
    if (!m3VoxelGet(&world->voxels.voxelData[slot], x, y, z))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVoxelSetFill record;
        memset(&record, 0, sizeof(record));
        record.id = shape;
        record.x = x;
        record.y = y;
        record.z = z;
        record.fill = fill;
        m3JournalRecord(world, m3_opVoxelSetFill, &record, (int32_t)sizeof(record));
    }
    m3VoxelSetFillInternal(world, index, x, y, z, fill);
}

void m3VoxelChunk_SetFill(m3World* world, m3ShapeId shape, int32_t x, int32_t y, int32_t z,
                          uint8_t fill)
{
    uint64_t env = m3FloatEnvEnter();
    VoxelChunkSetFillWorker(world, shape, x, y, z, fill);
    m3FloatEnvLeave(env);
}

static M3_NOINLINE int32_t VoxelChunkClearBoxWorker(m3World* world, m3ShapeId shape,
                                                    const int32_t lo[3], const int32_t hi[3])
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorState);
        return -1;
    }
    int32_t index;
    if (!ResolveVoxelShape(world, shape, &index))
    {
        return -1;
    }
    if (lo == NULL || hi == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1;
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        if (lo[k] < 0 || hi[k] >= M3_VOXEL_DIM || lo[k] > hi[k])
        {
            m3Refuse(world, m3_errorInvalid);
            return -1; // a region out of the chunk or inverted
        }
    }
    int32_t cleared = m3VoxelClearBoxInternal(world, index, lo, hi);
    if (cleared < 0)
    {
        return -1; // the surface would pass the box limit: counted, nothing changed
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVoxelClearBox record;
        memset(&record, 0, sizeof(record));
        record.id = shape;
        for (int32_t k = 0; k < 3; ++k)
        {
            record.lo[k] = lo[k];
            record.hi[k] = hi[k];
        }
        m3JournalRecord(world, m3_opVoxelClearBox, &record, (int32_t)sizeof(record));
    }
    return cleared;
}

int32_t m3VoxelChunk_ClearBox(m3World* world, m3ShapeId shape, const int32_t lo[3],
                              const int32_t hi[3])
{
    uint64_t env = m3FloatEnvEnter();
    int32_t result = VoxelChunkClearBoxWorker(world, shape, lo, hi);
    m3FloatEnvLeave(env);
    return result;
}

// ---------------------------------------------------------------
// Seam welding.

// A hull from raw chunk-frame bounds (the welded collision path
// extends covered faces before building, so the SAT never sees an
// interior face as a candidate).
void m3VoxelBoundsHull(m3Vec3 lo, m3Vec3 hi, m3HullData* out)
{
    m3Vec3 half = m3MulSV3(0.5f, m3Sub3(hi, lo));
    m3Vec3 center = m3MulSV3(0.5f, m3Add3(lo, hi));
    m3BuildBoxHull(out, half);
    for (int32_t v = 0; v < out->vertexCount; ++v)
    {
        out->vertices[v] = m3Add3(out->vertices[v], center);
    }
    for (int32_t f = 0; f < out->faceCount; ++f)
    {
        out->faceOffsets[f] += m3Dot3(out->faceNormals[f], center);
    }
    out->center = m3Add3(out->center, center);
}
