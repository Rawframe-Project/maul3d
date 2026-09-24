// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Voxel fracture: connectivity after an edit and the fragment events it
// emits, and the escape direction for a point buried in a chunk.

#include "body.h"
#include "character.h"
#include "hull.h"
#include "journal.h"
#include "shape.h"
#include "voxel.h"
#include "world.h"
#include "world_internal.h"

#include <string.h>

// Deterministic flood fill: seeds scan in canonical linear order,
// the frontier grows with a fixed six-neighbor order, so island
// labels and event order are pure functions of the grid.
void m3VoxelFractureSweep(m3World* world, int32_t shape)
{
    int32_t slot = world->shapes.shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
    if (chunk->filledCount == 0)
    {
        return;
    }
    static const int32_t offsets[6][3] = {{-1, 0, 0}, {1, 0, 0},  {0, -1, 0},
                                          {0, 1, 0},  {0, 0, -1}, {0, 0, 1}};
    uint8_t visited[M3_VOXEL_COUNT];
    memset(visited, 0, sizeof(visited));
    uint16_t stack[M3_VOXEL_COUNT];
    uint16_t island[M3_VOXEL_COUNT];

    bool removedAny = false;
    for (int32_t seed = 0; seed < M3_VOXEL_COUNT; ++seed)
    {
        int32_t sx = seed % M3_VOXEL_DIM;
        int32_t sy = (seed / M3_VOXEL_DIM) % M3_VOXEL_DIM;
        int32_t sz = seed / (M3_VOXEL_DIM * M3_VOXEL_DIM);
        if (visited[seed] != 0 || !m3VoxelGet(chunk, sx, sy, sz))
        {
            continue;
        }
        // Collect the island.
        int32_t top = 0;
        int32_t count = 0;
        bool anchored = false;
        stack[top++] = (uint16_t)seed;
        visited[seed] = 1;
        while (top > 0)
        {
            uint16_t v = stack[--top];
            island[count++] = v;
            int32_t x = v % M3_VOXEL_DIM;
            int32_t y = (v / M3_VOXEL_DIM) % M3_VOXEL_DIM;
            int32_t z = v / (M3_VOXEL_DIM * M3_VOXEL_DIM);
            if (y == 0)
            {
                anchored = true; // the base layer is the anchor
            }
            for (int32_t n = 0; n < 6; ++n)
            {
                int32_t nx = x + offsets[n][0];
                int32_t ny = y + offsets[n][1];
                int32_t nz = z + offsets[n][2];
                if (nx < 0 || nx >= M3_VOXEL_DIM || ny < 0 || ny >= M3_VOXEL_DIM || nz < 0 ||
                    nz >= M3_VOXEL_DIM)
                {
                    continue;
                }
                int32_t nv = nx + M3_VOXEL_DIM * (ny + M3_VOXEL_DIM * nz);
                if (visited[nv] == 0 && m3VoxelGet(chunk, nx, ny, nz))
                {
                    visited[nv] = 1;
                    stack[top++] = (uint16_t)nv;
                }
            }
        }
        if (anchored)
        {
            continue; // grounded islands stand
        }

        // An unanchored island: remove it from the grid (part of the
        // edit's state transition) and emit the event.
        m3real cell = chunk->cellSize;
        m3Vec3 com = {0.0f, 0.0f, 0.0f};
        m3Vec3 comEq = {0.0f, 0.0f, 0.0f}; // unweighted twin (see below)
        int64_t fillSum = 0;
        uint8_t lo[3] = {255, 255, 255};
        uint8_t hi[3] = {0, 0, 0};
        for (int32_t k = 0; k < count; ++k)
        {
            uint16_t v = island[k];
            int32_t x = v % M3_VOXEL_DIM;
            int32_t y = (v / M3_VOXEL_DIM) % M3_VOXEL_DIM;
            int32_t z = v / (M3_VOXEL_DIM * M3_VOXEL_DIM);
            m3real w = (m3real)chunk->fill[v]; // fill-weighted
            fillSum += (int64_t)chunk->fill[v];
            chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
            chunk->payload[v] = 0;
            chunk->fill[v] = 0;
            m3Vec3 center = {((m3real)x + 0.5f) * cell, ((m3real)y + 0.5f) * cell,
                             ((m3real)z + 0.5f) * cell};
            com = m3Add3(com, m3MulSV3(w, center));
            comEq = m3Add3(comEq, center);
            lo[0] = x < lo[0] ? (uint8_t)x : lo[0];
            lo[1] = y < lo[1] ? (uint8_t)y : lo[1];
            lo[2] = z < lo[2] ? (uint8_t)z : lo[2];
            hi[0] = x > hi[0] ? (uint8_t)x : hi[0];
            hi[1] = y > hi[1] ? (uint8_t)y : hi[1];
            hi[2] = z > hi[2] ? (uint8_t)z : hi[2];
        }
        chunk->filledCount -= count;
        removedAny = true;
        if (fillSum > 0)
        {
            com = m3MulSV3(1.0f / (m3real)fillSum, com);
        }
        else
        {
            // A legitimate stream never gets here (occupied implies
            // fill >= 1), but a mutated snapshot can set occupancy
            // bits over zeroed fill bytes, and 1/0 would mint a NaN
            // center that infects the whole simulation. The equal
            // weight center is the fallback (count >= 1 in
            // this branch by construction).
            com = m3MulSV3(1.0f / (m3real)count, comEq);
        }

        if (world->events.fragmentEventCount >= M3_FRAGMENT_EVENT_CAP)
        {
            world->events.fragmentDropped += 1; // loud: the state moved, the
                                                // event did not fit
            continue;
        }
        m3FragmentEvent* ev = &world->events.fragmentEvents[world->events.fragmentEventCount];
        memset(ev, 0, sizeof(*ev));
        ev->chunkShapeId =
            (m3ShapeId){shape + 1, world->idWorld, world->shapes.shapePool.generations[shape]};
        ev->voxelCount = count;
        if (world->events.fragmentRecipeCount + count <= M3_FRAGMENT_RECIPE_CAP)
        {
            ev->recipeStart = world->events.fragmentRecipeCount;
            ev->recipeCount = count;
            for (int32_t k = 0; k < count; ++k)
            {
                world->events.fragmentRecipe[world->events.fragmentRecipeCount + k] = island[k];
            }
            world->events.fragmentRecipeCount += count;
        }
        else
        {
            ev->recipeStart = -1; // recipe overflow: bounds and count
                                  // still describe the island, loudly
            ev->recipeCount = 0;
        }
        ev->comChunk = com;
        int32_t body = world->shapes.shapeBody[shape];
        const m3Transform* xf = &world->bodies.transforms[body];
        m3Vec3 r = m3RotateVec3(xf->q, com);
        ev->comWorld =
            (m3Pos3){xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
        ev->mass = ((m3real)fillSum / 255.0f) * cell * cell * cell;
        for (int32_t k = 0; k < 3; ++k)
        {
            ev->boundsLo[k] = lo[k];
            ev->boundsHi[k] = hi[k];
        }
        world->events.fragmentEventCount += 1;
    }

    if (removedAny)
    {
        // The surface follows the grid, always (the derived law).
        m3VoxelSurfaceBuild(&world->voxels.voxelSurface[slot], chunk);
        m3VoxelCoverageRefreshAround(world, slot);
    }
}

// Interior depenetration: a body center INSIDE the solid
// (spawn mistakes, monster impulses) escapes through the nearest
// exposed face found by a deterministic BFS over the grid (fixed
// seed, fixed neighbor order). Returns false when the point is not
// inside solid. The out parameters name the exit: a chunk-frame
// axis normal and the exit plane coordinate along it.
bool m3VoxelEscape(const m3World* world, int32_t slot, m3Vec3 localPoint, m3Vec3* outNormal,
                   m3real* outPlane)
{
    const m3VoxelChunkData* chunk = &world->voxels.voxelData[slot];
    m3real cell = chunk->cellSize;
    int32_t sx = (int32_t)(localPoint.x / cell);
    int32_t sy = (int32_t)(localPoint.y / cell);
    int32_t sz = (int32_t)(localPoint.z / cell);
    if (localPoint.x < 0.0f || localPoint.y < 0.0f || localPoint.z < 0.0f || sx >= M3_VOXEL_DIM ||
        sy >= M3_VOXEL_DIM || sz >= M3_VOXEL_DIM || !m3VoxelGet(chunk, sx, sy, sz))
    {
        return false;
    }
    static const int32_t offsets[6][3] = {{-1, 0, 0}, {1, 0, 0},  {0, -1, 0},
                                          {0, 1, 0},  {0, 0, -1}, {0, 0, 1}};
    uint8_t visited[M3_VOXEL_COUNT];
    memset(visited, 0, sizeof(visited));
    uint16_t queue[M3_VOXEL_COUNT];
    int32_t head = 0;
    int32_t tail = 0;
    int32_t start = sx + M3_VOXEL_DIM * (sy + M3_VOXEL_DIM * sz);
    queue[tail++] = (uint16_t)start;
    visited[start] = 1;
    while (head < tail)
    {
        uint16_t v = queue[head++];
        int32_t x = v % M3_VOXEL_DIM;
        int32_t y = (v / M3_VOXEL_DIM) % M3_VOXEL_DIM;
        int32_t z = v / (M3_VOXEL_DIM * M3_VOXEL_DIM);
        for (int32_t n = 0; n < 6; ++n)
        {
            int32_t nx = x + offsets[n][0];
            int32_t ny = y + offsets[n][1];
            int32_t nz = z + offsets[n][2];
            bool exit = false;
            if (nx < 0 || nx >= M3_VOXEL_DIM || ny < 0 || ny >= M3_VOXEL_DIM || nz < 0 ||
                nz >= M3_VOXEL_DIM)
            {
                // The chunk boundary: an exit unless a welded
                // neighbor continues the solid there.
                int32_t link = world->voxels.voxelNeighbors[slot * 6 + n];
                if (link < 0)
                {
                    exit = true;
                }
                else
                {
                    int32_t mx = (nx + M3_VOXEL_DIM) % M3_VOXEL_DIM;
                    int32_t my = (ny + M3_VOXEL_DIM) % M3_VOXEL_DIM;
                    int32_t mz = (nz + M3_VOXEL_DIM) % M3_VOXEL_DIM;
                    exit = !m3VoxelGet(&world->voxels.voxelData[link], mx, my, mz);
                }
            }
            else if (!m3VoxelGet(chunk, nx, ny, nz))
            {
                exit = true;
            }
            if (exit)
            {
                int32_t axis = n / 2;
                m3real sign = (n & 1) != 0 ? 1.0f : -1.0f;
                *outNormal = (m3Vec3){0.0f, 0.0f, 0.0f};
                int32_t faceCell = (n & 1) != 0 ? (axis == 0 ? x + 1 : (axis == 1 ? y + 1 : z + 1))
                                                : (axis == 0 ? x : (axis == 1 ? y : z));
                if (axis == 0)
                {
                    outNormal->x = sign;
                }
                else if (axis == 1)
                {
                    outNormal->y = sign;
                }
                else
                {
                    outNormal->z = sign;
                }
                *outPlane = (m3real)faceCell * cell;
                return true;
            }
            if (nx < 0 || nx >= M3_VOXEL_DIM || ny < 0 || ny >= M3_VOXEL_DIM || nz < 0 ||
                nz >= M3_VOXEL_DIM)
            {
                continue; // solid continues into a welded neighbor: no exit this way
            }
            int32_t nv = nx + M3_VOXEL_DIM * (ny + M3_VOXEL_DIM * nz);
            if (visited[nv] == 0)
            {
                visited[nv] = 1;
                queue[tail++] = (uint16_t)nv;
            }
        }
    }
    return false; // a chunk with no exposed face anywhere: welded
                  // solid on all sides; the neighbor owns the escape
}
