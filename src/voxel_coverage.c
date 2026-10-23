// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Voxel seams: chunk links and face coverage.

#include "voxel_coverage.h"

#include <string.h>

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
