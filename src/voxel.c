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
#include "body.h"
#include "character.h"
#include "hull.h"
#include "journal.h"
#include "shape.h"
#include "world.h"
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

// The greedy merge: scan in canonical order (x fastest, then y,
// then z); from each unclaimed filled voxel grow a run along x,
// widen it along y, then deepen it along z, claiming as it goes.
// Deterministic by the fixed scan and growth order; maximal in the
// greedy sense (not globally optimal, which is NP-hard and
// unnecessary).
void m3VoxelSurfaceBuild(m3VoxelSurface* surface, const m3VoxelChunkData* chunk)
{
    // The embedded BVH owns heap arrays now: release them
    // before the wipe or the rebuild leaks the previous build.
    // CONTRACT: the surface must be zeroed or a previous build;
    // garbage pointers here are the caller's crash.
    m3MeshBvhFree(&surface->bvh);
    memset(surface, 0, sizeof(*surface));
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
                int32_t b = surface->boxCount;
                M3_ASSERT(b < M3_VOXEL_MAX_BOXES); // 2048 = the exact
                                                   // isolated-voxel
                                                   // worst case
                surface->boxLo[b][0] = (uint8_t)x;
                surface->boxLo[b][1] = (uint8_t)y;
                surface->boxLo[b][2] = (uint8_t)z;
                surface->boxHi[b][0] = (uint8_t)x2;
                surface->boxHi[b][1] = (uint8_t)y2;
                surface->boxHi[b][2] = (uint8_t)z2;
                surface->boxCount = b + 1;
            }
        }
    }

    // The midphase: bounds per merged box, in chunk-frame meters.
    m3Vec3 los[M3_VOXEL_MAX_BOXES];
    m3Vec3 his[M3_VOXEL_MAX_BOXES];
    m3real cell = chunk->cellSize;
    for (int32_t b = 0; b < surface->boxCount; ++b)
    {
        los[b] = (m3Vec3){(m3real)surface->boxLo[b][0] * cell, (m3real)surface->boxLo[b][1] * cell,
                          (m3real)surface->boxLo[b][2] * cell};
        his[b] = (m3Vec3){(m3real)(surface->boxHi[b][0] + 1) * cell,
                          (m3real)(surface->boxHi[b][1] + 1) * cell,
                          (m3real)(surface->boxHi[b][2] + 1) * cell};
    }
    m3MeshBvhBuildBounds(&surface->bvh, los, his, surface->boxCount);
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

static bool VoxelCoordsValid(int32_t x, int32_t y, int32_t z)
{
    return x >= 0 && x < M3_VOXEL_DIM && y >= 0 && y < M3_VOXEL_DIM && z >= 0 && z < M3_VOXEL_DIM;
}

bool m3VoxelSetInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z,
                        uint16_t payload)
{
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    bool wasFilled = m3VoxelGet(chunk, x, y, z);
    chunk->occupancy[v >> 3] |= (uint8_t)(1u << (v & 7));
    chunk->payload[v] = payload;
    chunk->fill[v] = 255; // a set voxel is a whole voxel
    if (!wasFilled)
    {
        chunk->filledCount += 1;
        // Occupancy changed: the surface is stale. A payload-only
        // set falls through (the surface is a pure function of
        // occupancy and cell size, never of payload). No fracture
        // sweep here: adding a voxel can only CONNECT islands.
        m3VoxelSurfaceBuild(&world->voxels.voxelSurface[slot], chunk);
        m3VoxelCoverageRefreshAround(world, slot);
    }
    int32_t lo[3] = {x, y, z};
    VoxelWakeRegion(world, shape, lo, lo);
    return true;
}

bool m3VoxelClearInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z)
{
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    if (m3VoxelGet(chunk, x, y, z))
    {
        chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
        chunk->payload[v] = 0;
        chunk->fill[v] = 0;
        chunk->filledCount -= 1;
        m3VoxelSurfaceBuild(&world->voxels.voxelSurface[slot], chunk);
        m3VoxelFractureSweep(world, shape);
        m3VoxelCoverageRefreshAround(world, slot);
    }
    int32_t lo[3] = {x, y, z};
    VoxelWakeRegion(world, shape, lo, lo);
    return true;
}

int32_t m3VoxelClearBoxInternal(m3World* world, int32_t shape, const int32_t lo[3],
                                const int32_t hi[3])
{
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
    int32_t cleared = 0;
    for (int32_t z = lo[2]; z <= hi[2]; ++z)
    {
        for (int32_t y = lo[1]; y <= hi[1]; ++y)
        {
            for (int32_t x = lo[0]; x <= hi[0]; ++x)
            {
                if (m3VoxelGet(chunk, x, y, z))
                {
                    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
                    chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
                    chunk->payload[v] = 0;
                    chunk->fill[v] = 0;
                    cleared += 1;
                }
            }
        }
    }
    if (cleared > 0)
    {
        chunk->filledCount -= cleared;
        m3VoxelSurfaceBuild(&world->voxels.voxelSurface[slot], chunk);
        m3VoxelFractureSweep(world, shape);
        m3VoxelCoverageRefreshAround(world, slot);
    }
    VoxelWakeRegion(world, shape, lo, hi);
    return cleared;
}

bool m3VoxelSetFillInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z,
                            uint8_t fill)
{
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
    m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
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
                int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
                chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
                chunk->payload[v] = 0;
                chunk->fill[v] = 0;
                cleared += 1;
            }
        }
    }
    if (cleared > 0)
    {
        chunk->filledCount -= cleared;
        m3VoxelSurfaceBuild(&world->voxels.voxelSurface[slot], chunk);
        m3VoxelFractureSweep(world, shape);
        m3VoxelCoverageRefreshAround(world, slot);
    }
    VoxelWakeRegion(world, shape, lo, hi);
    return cleared;
}

// Public entries: validate, journal, apply (the command pattern).
static m3World* ResolveVoxelShape(m3ShapeId shapeId, int32_t* shapeOut)
{
    m3World* world = m3WorldFromTag(shapeId.world);
    int32_t shape = world != NULL ? m3ShapeSlot(world, shapeId) : -1;
    if (shape < 0 || world->shapes.shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        m3Refuse(world, m3_errorInvalid);
        return NULL; // stale, foreign, or not a voxel chunk: contract
    }
    *shapeOut = shape;
    return world;
}

bool m3VoxelChunk_SetVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z, uint16_t payload)
{
    int32_t shape;
    m3World* world = ResolveVoxelShape(shapeId, &shape);
    if (world == NULL || !VoxelCoordsValid(x, y, z))
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVoxelSet record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.x = x;
        record.y = y;
        record.z = z;
        record.payload = payload;
        m3JournalRecord(world, m3_opVoxelSet, &record, (int32_t)sizeof(record));
    }
    return m3VoxelSetInternal(world, shape, x, y, z, payload);
}

bool m3VoxelChunk_ClearVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z)
{
    int32_t shape;
    m3World* world = ResolveVoxelShape(shapeId, &shape);
    if (world == NULL || !VoxelCoordsValid(x, y, z))
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVoxelClear record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.x = x;
        record.y = y;
        record.z = z;
        m3JournalRecord(world, m3_opVoxelClear, &record, (int32_t)sizeof(record));
    }
    return m3VoxelClearInternal(world, shape, x, y, z);
}

bool m3VoxelChunk_SetFill(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z, uint8_t fill)
{
    int32_t shape;
    m3World* world = ResolveVoxelShape(shapeId, &shape);
    if (world == NULL || !VoxelCoordsValid(x, y, z) || fill == 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false; // fill zero is a clear in disguise: refused
    }
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    if (!m3VoxelGet(&world->voxels.voxelData[slot], x, y, z))
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVoxelSetFill record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.x = x;
        record.y = y;
        record.z = z;
        record.fill = fill;
        m3JournalRecord(world, m3_opVoxelSetFill, &record, (int32_t)sizeof(record));
    }
    return m3VoxelSetFillInternal(world, shape, x, y, z, fill);
}

int32_t m3VoxelChunk_ClearBox(m3ShapeId shapeId, const int32_t lo[3], const int32_t hi[3])
{
    int32_t shape;
    m3World* world = ResolveVoxelShape(shapeId, &shape);
    if (world == NULL || lo == NULL || hi == NULL)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1;
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        if (lo[k] < 0 || hi[k] >= M3_VOXEL_DIM || lo[k] > hi[k])
        {
            return -1; // bad region: contract, loud
        }
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpVoxelClearBox record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        for (int32_t k = 0; k < 3; ++k)
        {
            record.lo[k] = lo[k];
            record.hi[k] = hi[k];
        }
        m3JournalRecord(world, m3_opVoxelClearBox, &record, (int32_t)sizeof(record));
    }
    return m3VoxelClearBoxInternal(world, shape, lo, hi);
}

// ---------------------------------------------------------------
// Seam welding.

void m3VoxelRebuildLinks(m3World* world)
{
    int32_t cap = world->voxels.voxelCapacity;
    for (int32_t i = 0; i < cap * 6; ++i)
    {
        world->voxels.voxelNeighbors[i] = -1;
    }
    static const m3Quat identity = {0.0f, 0.0f, 0.0f, 1.0f};
    int32_t maxSlot = world->voxels.voxelPool.maxIndex;
    for (int32_t a = 0; a < maxSlot; ++a)
    {
        if (world->voxels.voxelPool.alive[a] == 0)
        {
            continue;
        }
        int32_t bodyA = world->shapes.shapeBody[world->voxels.voxelShape[a]];
        const m3Transform* xfA = &world->bodies.transforms[bodyA];
        // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison): bitwise identity
        if (memcmp(&xfA->q, &identity, sizeof(m3Quat)) != 0)
        {
            continue; // the welding contract wants grid alignment
        }
        double extentA = (double)((m3real)M3_VOXEL_DIM * world->voxels.voxelData[a].cellSize);
        for (int32_t b = 0; b < maxSlot; ++b)
        {
            if (b == a || world->voxels.voxelPool.alive[b] == 0 ||
                world->voxels.voxelData[b].cellSize != world->voxels.voxelData[a].cellSize)
            {
                continue;
            }
            int32_t bodyB = world->shapes.shapeBody[world->voxels.voxelShape[b]];
            const m3Transform* xfB = &world->bodies.transforms[bodyB];
            // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison): bitwise identity
            if (memcmp(&xfB->q, &identity, sizeof(m3Quat)) != 0)
            {
                continue;
            }
            double dx = xfB->p.x - xfA->p.x;
            double dy = xfB->p.y - xfA->p.y;
            double dz = xfB->p.z - xfA->p.z;
            // Exact adjacency on exactly one axis (the contract).
            if (dy == 0.0 && dz == 0.0 && dx == extentA)
            {
                world->voxels.voxelNeighbors[a * 6 + 1] = b; // +x
            }
            else if (dy == 0.0 && dz == 0.0 && dx == -extentA)
            {
                world->voxels.voxelNeighbors[a * 6 + 0] = b; // -x
            }
            else if (dx == 0.0 && dz == 0.0 && dy == extentA)
            {
                world->voxels.voxelNeighbors[a * 6 + 3] = b; // +y
            }
            else if (dx == 0.0 && dz == 0.0 && dy == -extentA)
            {
                world->voxels.voxelNeighbors[a * 6 + 2] = b; // -y
            }
            else if (dx == 0.0 && dy == 0.0 && dz == extentA)
            {
                world->voxels.voxelNeighbors[a * 6 + 5] = b; // +z
            }
            else if (dx == 0.0 && dy == 0.0 && dz == -extentA)
            {
                world->voxels.voxelNeighbors[a * 6 + 4] = b; // -z
            }
        }
    }
}

// Is the axis-aligned layer just OUTSIDE face `face` of the box
// fully filled? Looks into the slot's own grid, or across the weld
// into the neighbor's border layer when the box touches the chunk
// boundary.
static bool VoxelFaceCovered(const m3World* world, int32_t slot, const uint8_t lo[3],
                             const uint8_t hi[3], int32_t face)
{
    static const int32_t axisOf[6] = {0, 0, 1, 1, 2, 2};
    static const int32_t signOf[6] = {-1, 1, -1, 1, -1, 1};
    int32_t axis = axisOf[face];
    int32_t sign = signOf[face];
    int32_t layer = sign < 0 ? (int32_t)lo[axis] - 1 : (int32_t)hi[axis] + 1;
    const m3VoxelChunkData* grid = &world->voxels.voxelData[slot];
    if (layer < 0 || layer >= M3_VOXEL_DIM)
    {
        int32_t neighbor = world->voxels.voxelNeighbors[slot * 6 + face];
        if (neighbor < 0)
        {
            return false; // no weld: the face is exposed to the world
        }
        grid = &world->voxels.voxelData[neighbor];
        layer = sign < 0 ? M3_VOXEL_DIM - 1 : 0; // the mirrored border
    }
    int32_t u = (axis + 1) % 3;
    int32_t v = (axis + 2) % 3;
    for (int32_t i = lo[u]; i <= hi[u]; ++i)
    {
        for (int32_t j = lo[v]; j <= hi[v]; ++j)
        {
            int32_t c[3];
            c[axis] = layer;
            c[u] = i;
            c[v] = j;
            if (!m3VoxelGet(grid, c[0], c[1], c[2]))
            {
                return false;
            }
        }
    }
    return true;
}

void m3VoxelCoverageBuild(m3World* world, int32_t slot)
{
    m3VoxelSurface* surface = &world->voxels.voxelSurface[slot];
    for (int32_t b = 0; b < surface->boxCount; ++b)
    {
        uint8_t covered = 0;
        for (int32_t face = 0; face < 6; ++face)
        {
            if (VoxelFaceCovered(world, slot, surface->boxLo[b], surface->boxHi[b], face))
            {
                covered |= (uint8_t)(1u << face);
            }
        }
        surface->boxCovered[b] = covered;
    }
}

void m3VoxelCoverageRefreshAround(m3World* world, int32_t slot)
{
    m3VoxelCoverageBuild(world, slot);
    for (int32_t face = 0; face < 6; ++face)
    {
        int32_t neighbor = world->voxels.voxelNeighbors[slot * 6 + face];
        if (neighbor >= 0)
        {
            // The neighbor's border coverage reads THIS grid.
            m3VoxelCoverageBuild(world, neighbor);
        }
    }
}

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
