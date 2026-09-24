// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Broadphase v2: the fat-AABB dynamic tree behind the same module
// contract as the 2a scan (pairs out in canonical ascending key
// order). Planes are infinite, so they stay out of the tree and take
// a dedicated pass; spheres refresh their proxies in shape order,
// query the tree, and every emitted key list is sorted once at the
// end (unique keys, so any correct sort yields the identical result).
// The 2a brute-force scan stays below as the referee: on any scene
// both paths must produce the same list, and a test holds that gate.

#include "broad_phase.h"

#include "shape.h"
#include "world_internal.h"
#include <string.h>

#include <stdlib.h>

// A moved leaf is reinserted this far beyond its fresh bounds, so a
// slow body stays inside it for many steps. Only the tree shape and
// the candidate list see this margin; pairs are decided on the fresh
// bounds, so it moves no result.
#define M3_TREE_MARGIN 0.1

typedef struct m3Aabb3d
{
    double lo[3];
    double hi[3];
} m3Aabb3d;

static m3Aabb3d SphereAabb(const m3World* world, int32_t shape)
{
    m3Transform xfS = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS;

    if (world->shapes.shapeType[shape] == (uint8_t)m3_hullShape)
    {
        // Hull bounds: rotate every vertex, min and max in double.
        const m3HullData* hull = &world->hulls.hullData[world->shapes.shapeHullIndex[shape]];
        m3Aabb3d box = {{1.0e30, 1.0e30, 1.0e30}, {-1.0e30, -1.0e30, -1.0e30}};
        for (int32_t v = 0; v < hull->vertexCount; ++v)
        {
            m3Vec3 r = m3RotateVec3(xf->q, hull->vertices[v]);
            double p[3] = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
            for (int32_t k = 0; k < 3; ++k)
            {
                box.lo[k] = p[k] < box.lo[k] ? p[k] : box.lo[k];
                box.hi[k] = p[k] > box.hi[k] ? p[k] : box.hi[k];
            }
        }
        for (int32_t k = 0; k < 3; ++k)
        {
            box.lo[k] -= (double)M3_AABB_MARGIN;
            box.hi[k] += (double)M3_AABB_MARGIN;
        }
        return box;
    }

    if (world->shapes.shapeType[shape] == (uint8_t)m3_heightFieldShape)
    {
        // Native grid bounds: the same box the fat-AABB
        // branch uses, minus the fat margin plus the tight one.
        const m3HeightFieldData* hf =
            &world->heightFields.hfData[world->shapes.shapeHfIndex[shape]];
        m3Aabb3d box;
        box.lo[0] = xf->p.x;
        box.lo[1] = xf->p.y + (double)hf->minHeight;
        box.lo[2] = xf->p.z;
        box.hi[0] = xf->p.x + (double)((float)(hf->nx - 1) * hf->cellSize);
        box.hi[1] = xf->p.y + (double)hf->maxHeight;
        box.hi[2] = xf->p.z + (double)((float)(hf->nz - 1) * hf->cellSize);
        for (int32_t k = 0; k < 3; ++k)
        {
            box.lo[k] -= (double)M3_AABB_MARGIN;
            box.hi[k] += (double)M3_AABB_MARGIN;
        }
        return box;
    }

    if (world->shapes.shapeType[shape] == (uint8_t)m3_voxelShape)
    {
        // Voxel chunk bounds: the eight corners of the chunk box
        // [0, dim * cell]^3 in the body frame, rotated out.
        m3real extent = (m3real)M3_VOXEL_DIM * world->shapes.shapeGeom[shape].s;
        m3Aabb3d box = {{1.0e30, 1.0e30, 1.0e30}, {-1.0e30, -1.0e30, -1.0e30}};
        for (int32_t c = 0; c < 8; ++c)
        {
            m3Vec3 corner = {(c & 1) != 0 ? extent : 0.0f, (c & 2) != 0 ? extent : 0.0f,
                             (c & 4) != 0 ? extent : 0.0f};
            m3Vec3 r = m3RotateVec3(xf->q, corner);
            double p[3] = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
            for (int32_t k = 0; k < 3; ++k)
            {
                box.lo[k] = p[k] < box.lo[k] ? p[k] : box.lo[k];
                box.hi[k] = p[k] > box.hi[k] ? p[k] : box.hi[k];
            }
        }
        for (int32_t k = 0; k < 3; ++k)
        {
            box.lo[k] -= (double)M3_AABB_MARGIN;
            box.hi[k] += (double)M3_AABB_MARGIN;
        }
        return box;
    }

    if (world->shapes.shapeType[shape] == (uint8_t)m3_meshShape)
    {
        // Mesh bounds: min and max over all vertices in double.
        const m3MeshData* mesh = &world->meshes.meshData[world->shapes.shapeMeshIndex[shape]];
        m3Aabb3d box = {{1.0e30, 1.0e30, 1.0e30}, {-1.0e30, -1.0e30, -1.0e30}};
        for (int32_t v = 0; v < mesh->vertexCount; ++v)
        {
            m3Vec3 r = m3RotateVec3(xf->q, mesh->vertices[v]);
            double p[3] = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
            for (int32_t k = 0; k < 3; ++k)
            {
                box.lo[k] = p[k] < box.lo[k] ? p[k] : box.lo[k];
                box.hi[k] = p[k] > box.hi[k] ? p[k] : box.hi[k];
            }
        }
        for (int32_t k = 0; k < 3; ++k)
        {
            box.lo[k] -= (double)M3_AABB_MARGIN;
            box.hi[k] += (double)M3_AABB_MARGIN;
        }
        return box;
    }

    if (world->shapes.shapeType[shape] == (uint8_t)m3_capsuleShape)
    {
        // Capsule bounds: the two cap-center spheres, min and max in
        // double, same fattening as everything else.
        m3Vec3 r1 = m3RotateVec3(xf->q, world->shapes.shapeGeom[shape].v);
        m3Vec3 r2 = m3RotateVec3(xf->q, world->shapes.shapeGeom[shape].v2);
        double c1[3] = {xf->p.x + (double)r1.x, xf->p.y + (double)r1.y, xf->p.z + (double)r1.z};
        double c2[3] = {xf->p.x + (double)r2.x, xf->p.y + (double)r2.y, xf->p.z + (double)r2.z};
        double fat = (double)(world->shapes.shapeGeom[shape].s + M3_AABB_MARGIN);
        m3Aabb3d box;
        for (int32_t k = 0; k < 3; ++k)
        {
            box.lo[k] = (c1[k] < c2[k] ? c1[k] : c2[k]) - fat;
            box.hi[k] = (c1[k] > c2[k] ? c1[k] : c2[k]) + fat;
        }
        return box;
    }

    m3Vec3 local = world->shapes.shapeGeom[shape].v;
    m3Vec3 r = m3RotateVec3(xf->q, local);
    double cx = xf->p.x + (double)r.x;
    double cy = xf->p.y + (double)r.y;
    double cz = xf->p.z + (double)r.z;
    double fat = (double)(world->shapes.shapeGeom[shape].s + M3_AABB_MARGIN);
    m3Aabb3d box = {{cx - fat, cy - fat, cz - fat}, {cx + fat, cy + fat, cz + fat}};
    return box;
}

uint32_t m3ProxyMask(const m3World* world, int32_t shape)
{
    uint8_t bodyType = world->bodies.types[world->shapes.shapeBody[shape]];
    uint8_t type = world->shapes.shapeType[shape];
    uint32_t kind =
        bodyType == (uint8_t)m3_staticBody
            ? M3_PROXY_STATIC
            : (bodyType == (uint8_t)m3_kinematicBody ? M3_PROXY_KINEMATIC : M3_PROXY_DYNAMIC);
    bool surface = type == (uint8_t)m3_meshShape || type == (uint8_t)m3_voxelShape;
    return kind | (surface ? M3_PROXY_SURFACE : 0u);
}

void m3ShapeFatAabb(const m3World* world, int32_t shape, double lo[3], double hi[3])
{
    if (world->shapes.shapeType[shape] == (uint8_t)m3_heightFieldShape)
    {
        // The native grid: min corner at the body origin,
        // the sample extremes baked at create; the mesh margin.
        const m3HeightFieldData* hf =
            &world->heightFields.hfData[world->shapes.shapeHfIndex[shape]];
        const m3Transform* xf = &world->bodies.transforms[world->shapes.shapeBody[shape]];
        double margin = 0.1;
        lo[0] = xf->p.x - margin;
        lo[1] = xf->p.y + (double)hf->minHeight - margin;
        lo[2] = xf->p.z - margin;
        hi[0] = xf->p.x + (double)((float)(hf->nx - 1) * hf->cellSize) + margin;
        hi[1] = xf->p.y + (double)hf->maxHeight + margin;
        hi[2] = xf->p.z + (double)((float)(hf->nz - 1) * hf->cellSize) + margin;
        return;
    }
    m3Aabb3d box = SphereAabb(world, shape);
    lo[0] = box.lo[0];
    lo[1] = box.lo[1];
    lo[2] = box.lo[2];
    hi[0] = box.hi[0];
    hi[1] = box.hi[1];
    hi[2] = box.hi[2];
}

static int Overlap(const m3Aabb3d* a, const m3Aabb3d* b)
{
    return a->lo[0] <= b->hi[0] && b->lo[0] <= a->hi[0] && a->lo[1] <= b->hi[1] &&
           b->lo[1] <= a->hi[1] && a->lo[2] <= b->hi[2] && b->lo[2] <= a->hi[2];
}

// Fresh bounds of one shape for this step, memoized: a sleeping shape
// skipped the prefill and fills on first touch. Same function, same
// inputs, same bits as the prefill would have written; a scratch stall
// (no cache) computes directly.
static m3Aabb3d FreshBounds(const m3World* world, m3Aabb3d* cache, uint8_t* cacheValid,
                            int32_t shape)
{
    if (cache == NULL)
    {
        return SphereAabb(world, shape);
    }
    if (cacheValid[shape] == 0)
    {
        cache[shape] = SphereAabb(world, shape);
        cacheValid[shape] = 1;
    }
    return cache[shape];
}

// Shared pair filter: no self pairs, no static-static pairs.
static int PairAllowed(const m3World* world, int32_t i, int32_t j)
{
    int32_t bodyI = world->shapes.shapeBody[i];
    int32_t bodyJ = world->shapes.shapeBody[j];
    if (bodyI == bodyJ)
    {
        return 0;
    }
    if (world->bodies.bodyEnabled[bodyI] == 0 || world->bodies.bodyEnabled[bodyJ] == 0)
    {
        return 0; // disabled bodies vanish
    }
    if (world->bodies.types[bodyI] == (uint8_t)m3_staticBody &&
        world->bodies.types[bodyJ] == (uint8_t)m3_staticBody)
    {
        return 0;
    }

    // Sensors do not sense other sensors.
    if (world->shapes.shapeSensor[i] != 0 && world->shapes.shapeSensor[j] != 0)
    {
        return 0;
    }
    // Filters: a shared nonzero group overrides the bits,
    // positive forcing and negative forbidding; otherwise each
    // category must land in the other's mask.
    int32_t gi = world->shapes.shapeGroup[i];
    if (gi != 0 && gi == world->shapes.shapeGroup[j])
    {
        if (gi < 0)
        {
            return 0;
        }
    }
    else if (!m3FilterPass(world->shapes.shapeCategory[i], world->shapes.shapeMask[i],
                           world->shapes.shapeCategory[j], world->shapes.shapeMask[j]))
    {
        return 0;
    }
    // Jointed bodies skip contact unless the joint says otherwise
    // so chained links do not fight. Walk the shorter list.
    for (int32_t jt = world->joints.bodyJointHead[bodyI]; jt != -1;
         jt = world->joints.jointBodyA[jt] == bodyI ? world->joints.jointNextA[jt]
                                                    : world->joints.jointNextB[jt])
    {
        int32_t other = world->joints.jointBodyA[jt] == bodyI ? world->joints.jointBodyB[jt]
                                                              : world->joints.jointBodyA[jt];
        if (other == bodyJ && world->joints.jointCollide[jt] == 0)
        {
            return 0;
        }
    }
    return 1;
}

// A pair is hot when either endpoint is an awake dynamic
// body. Cold pairs (sleeping-sleeping, static-sleeping, plane-
// sleeping) cannot change until something wakes, so they ride the
// frozen buffer instead of being re-discovered every step.
static int PairHot(const m3World* world, int32_t i, int32_t j)
{
    // Kinematic bodies (characters, platforms) move without ever
    // sleeping: they are hot whenever awake, or a walker would glide
    // through a sleeping crate it had never met (the first cut said
    // "dynamic" here and five suites said otherwise).
    int32_t bodyI = world->shapes.shapeBody[i];
    int32_t bodyJ = world->shapes.shapeBody[j];
    if (world->bodies.types[bodyI] != (uint8_t)m3_staticBody && world->bodies.awake[bodyI] != 0)
    {
        return 1;
    }
    return world->bodies.types[bodyJ] != (uint8_t)m3_staticBody && world->bodies.awake[bodyJ] != 0;
}

static int EmitPair(m3World* world, int32_t i, int32_t j)
{
    if (world->contacts.pairCount == world->contacts.pairCapacity)
    {
        return 0; // loud at the caller
    }
    uint64_t key =
        i < j ? (((uint64_t)i << 32) | (uint64_t)j) : (((uint64_t)j << 32) | (uint64_t)i);
    world->contacts.pairKeys[world->contacts.pairCount] = key;
    world->contacts.pairCount += 1;
    return 1;
}

static int CompareKeys(const void* a, const void* b)
{
    uint64_t ka = *(const uint64_t*)a;
    uint64_t kb = *(const uint64_t*)b;
    return ka < kb ? -1 : (ka > kb ? 1 : 0);
}

typedef struct m3QueryCtx
{
    m3World* world;
    m3Aabb3d* cache;     // per-step fresh bounds
    uint8_t* cacheValid; // sleepers fill lazily on first hit
    m3Aabb3d selfBounds;
    int32_t self;
    int32_t overflow;
} m3QueryCtx;

static bool QueryHit(int32_t other, void* context)
{
    m3QueryCtx* ctx = (m3QueryCtx*)context;
    if (other == ctx->self)
    {
        return true;
    }
    // Emit each pair once. Both-awake pairs use the larger-index
    // rule (both directions get queried, one emits). Static and
    // sleeping shapes never query, so their awake partner emits
    // from EITHER side; the referee caught pair (sleeper, awake)
    // with the sleeper on the smaller index silently vanishing.
    if (other < ctx->self)
    {
        int32_t otherBody = ctx->world->shapes.shapeBody[other];
        int32_t otherQueries = ctx->world->bodies.types[otherBody] != (uint8_t)m3_staticBody &&
                               ctx->world->bodies.awake[otherBody] != 0;
        if (otherQueries)
        {
            return true; // the other side owns this emit
        }
    }
    if (!PairAllowed(ctx->world, ctx->self, other))
    {
        return true;
    }
    if (!PairHot(ctx->world, ctx->self, other))
    {
        return true; // cold: rides the frozen buffer
    }
    // The stored leaf bounds are stale-but-containing (a leaf only
    // moves when its fresh bounds escape), so the tree can return a
    // SUPERSET of the true fat overlaps. Re-test with fresh bounds so
    // the pair list equals the brute-force referee STRUCTURALLY, not
    // by luck.
    m3Aabb3d fresh = FreshBounds(ctx->world, ctx->cache, ctx->cacheValid, other);
    if (!Overlap(&ctx->selfBounds, &fresh))
    {
        return true;
    }
    if (!EmitPair(ctx->world, ctx->self, other))
    {
        ctx->overflow = 1;
        return false;
    }
    return true;
}

// The direct tree pass: each awake shape queries the tree; the j > i
// rule emits every overlap exactly once. A static shape only ever
// pairs hot with an awake one, which finds it from its side.
static m3Result QueryAwakeShapes(m3World* world, m3Aabb3d* cache, uint8_t* cacheValid)
{
    for (int32_t i = 0; i < world->shapes.shapePool.maxIndex; ++i)
    {
        if (world->shapes.shapePool.alive[i] == 0 || world->broadphase.proxyIds[i] == M3_TREE_NULL)
        {
            continue;
        }
        if (world->bodies.types[world->shapes.shapeBody[i]] == (uint8_t)m3_staticBody ||
            world->bodies.awake[world->shapes.shapeBody[i]] == 0)
        {
            continue; // a static or frozen shape discovers nothing new
        }
        m3QueryCtx ctx;
        ctx.world = world;
        ctx.cache = cache;
        ctx.cacheValid = cacheValid;
        ctx.selfBounds = FreshBounds(world, cache, cacheValid, i);
        ctx.self = i;
        ctx.overflow = 0;
        m3TreeQuery(&world->broadphase.tree, ctx.selfBounds.lo, ctx.selfBounds.hi, QueryHit, &ctx);
        if (ctx.overflow != 0)
        {
            return m3_errorCapacity;
        }
    }
    return m3_success;
}

typedef struct m3CandidateCtx
{
    m3World* world;
    int32_t self;
    int32_t overflow;
} m3CandidateCtx;

static bool CandidateHit(int32_t other, void* context)
{
    m3CandidateCtx* ctx = (m3CandidateCtx*)context;
    m3World* world = ctx->world;
    m3Broadphase* bp = &world->broadphase;
    if (other == ctx->self || (bp->moved[other] != 0 && other < ctx->self))
    {
        return true; // itself, or a pair the other mover already added
    }
    if (world->bodies.types[world->shapes.shapeBody[other]] == (uint8_t)m3_staticBody &&
        world->bodies.types[world->shapes.shapeBody[ctx->self]] == (uint8_t)m3_staticBody)
    {
        return true;
    }
    if (bp->candidateCount == world->contacts.pairCapacity)
    {
        ctx->overflow = 1;
        return false;
    }
    int32_t i = ctx->self;
    uint64_t key = i < other ? (((uint64_t)i << 32) | (uint64_t)other)
                             : (((uint64_t)other << 32) | (uint64_t)i);
    bp->candidateKeys[bp->candidateCount++] = key;
    return true;
}

// Brings the candidate list up to date: every pair of tree leaves that
// overlap, static-static aside. A leaf changes only when it moves, so
// only pairs touching a moved leaf are dropped and requeried, and a
// body resting inside its leaf costs no query at all. Returns 0 when
// the list does not fit; the next update then starts over.
static int UpdateCandidates(m3World* world)
{
    m3Broadphase* bp = &world->broadphase;
    int32_t maxShape = world->shapes.shapePool.maxIndex;
    if (bp->candidatesFresh == 0)
    {
        bp->candidateCount = 0;
        memset(bp->moved, 1, (size_t)world->shapes.shapeCapacity);
    }
    int32_t kept = 0;
    for (int32_t k = 0; k < bp->candidateCount; ++k)
    {
        uint64_t key = bp->candidateKeys[k];
        if (bp->moved[key >> 32] == 0 && bp->moved[key & 0xFFFFFFFFu] == 0)
        {
            bp->candidateKeys[kept++] = key;
        }
    }
    bp->candidateCount = kept;
    int fits = 1;
    for (int32_t i = 0; i < maxShape && fits; ++i)
    {
        if (bp->moved[i] == 0 || world->shapes.shapePool.alive[i] == 0 ||
            bp->proxyIds[i] == M3_TREE_NULL)
        {
            continue;
        }
        const m3TreeNode* leaf = &bp->tree.nodes[bp->proxyIds[i]];
        uint32_t targets = (leaf->mask & M3_PROXY_STATIC) != 0
                               ? (M3_PROXY_KINEMATIC | M3_PROXY_DYNAMIC)
                               : 0xFFFFFFFFu;
        m3CandidateCtx ctx = {world, i, 0};
        m3TreeQueryMask(&bp->tree, leaf->lo, leaf->hi, targets, CandidateHit, &ctx);
        fits = ctx.overflow == 0;
    }
    memset(bp->moved, 0, (size_t)world->shapes.shapeCapacity);
    bp->candidatesFresh = (uint8_t)fits;
    return fits;
}

// Emits the hot candidates whose fresh bounds overlap: the same pairs
// the direct pass finds.
static int EmitCandidates(m3World* world, m3Aabb3d* cache, uint8_t* cacheValid)
{
    const m3Broadphase* bp = &world->broadphase;
    for (int32_t k = 0; k < bp->candidateCount; ++k)
    {
        int32_t i = (int32_t)(bp->candidateKeys[k] >> 32);
        int32_t j = (int32_t)(bp->candidateKeys[k] & 0xFFFFFFFFu);
        // A destroy marks its shape moved, so no dead shape survives.
        M3_ASSERT(world->shapes.shapePool.alive[i] != 0 && world->shapes.shapePool.alive[j] != 0);
        if (!PairHot(world, i, j) || !PairAllowed(world, i, j))
        {
            continue; // cold pairs ride the frozen buffer
        }
        m3Aabb3d a = FreshBounds(world, cache, cacheValid, i);
        m3Aabb3d b = FreshBounds(world, cache, cacheValid, j);
        if (Overlap(&a, &b) && !EmitPair(world, i, j))
        {
            return 0;
        }
    }
    return 1;
}

typedef struct m3FreezeCtx
{
    m3World* world;
    m3Aabb3d selfBounds;
    int32_t self;
    int32_t overflow;
} m3FreezeCtx;

static bool FreezeHit(int32_t other, void* context)
{
    m3FreezeCtx* ctx = (m3FreezeCtx*)context;
    if (other == ctx->self || !PairAllowed(ctx->world, ctx->self, other))
    {
        return true;
    }
    m3Aabb3d fresh = SphereAabb(ctx->world, other);
    if (!Overlap(&ctx->selfBounds, &fresh))
    {
        return true;
    }
    m3World* world = ctx->world;
    if (world->contacts.sleepingPairCount == world->contacts.pairCapacity)
    {
        ctx->overflow = 1;
        return false;
    }
    int32_t i = ctx->self;
    uint64_t key = i < other ? (((uint64_t)i << 32) | (uint64_t)other)
                             : (((uint64_t)other << 32) | (uint64_t)i);
    world->contacts.sleepingPairKeys[world->contacts.sleepingPairCount++] = key;
    return true;
}

// The freeze step can CREATE tight overlaps (the solver pushes two
// bodies together in the very step their island falls asleep) that
// were never in any pair list. The sleep pass calls this for every
// body it just froze so the frozen buffer holds the true overlap
// set; the harvest dedupes. Without it the replay scrubber's seeks
// diverged: a restored world's full query saw pairs the linear run
// had never discovered.
void m3FreezeDiscoverPairs(m3World* world, int32_t body)
{
    for (int32_t sh = world->bodies.bodyShapeHead[body]; sh >= 0; sh = world->shapes.shapeNext[sh])
    {
        if (world->shapes.shapePool.alive[sh] == 0)
        {
            continue;
        }
        if (world->broadphase.proxyIds[sh] != M3_TREE_NULL)
        {
            m3FreezeCtx ctx;
            ctx.world = world;
            ctx.self = sh;
            ctx.selfBounds = SphereAabb(world, sh);
            ctx.overflow = 0;
            m3TreeQuery(&world->broadphase.tree, ctx.selfBounds.lo, ctx.selfBounds.hi, FreezeHit,
                        &ctx);
        }
        // Planes live outside the tree and pair unconditionally: a
        // frozen body keeps its ground pair through the buffer.
        for (int32_t k = 0; k < world->shapes.planeCount; ++k)
        {
            int32_t p2 = world->shapes.planeShapes[k];
            if (!PairAllowed(world, p2, sh))
            {
                continue;
            }
            if (world->contacts.sleepingPairCount == world->contacts.pairCapacity)
            {
                return;
            }
            uint64_t key = p2 < sh ? (((uint64_t)p2 << 32) | (uint64_t)sh)
                                   : (((uint64_t)sh << 32) | (uint64_t)p2);
            world->contacts.sleepingPairKeys[world->contacts.sleepingPairCount++] = key;
        }
    }
    world->contacts.frozenDirty = 1;
}

m3Result m3UpdatePairs(m3World* world)
{
    if (world->contacts.pairsFullQuery != 0)
    {
        // A restore invalidated the buffer: rebuild it as the pure
        // function it is of the CURRENT sleeping state, by running
        // the same discovery every freeze runs. Equality with the
        // linear run is by construction, not by bookkeeping.
        world->contacts.sleepingPairCount = 0;
        for (int32_t b = 0; b < world->bodies.bodyPool.maxIndex; ++b)
        {
            if (world->bodies.bodyPool.alive[b] != 0 &&
                world->bodies.types[b] != (uint8_t)m3_staticBody && world->bodies.awake[b] == 0)
            {
                m3FreezeDiscoverPairs(world, b);
            }
        }
        world->contacts.pairsFullQuery = 0;
    }
    if (world->contacts.frozenDirty != 0)
    {
        // Discoveries append unsorted and may duplicate (both sides
        // of a pair can freeze in different events); one canonical
        // sort plus unique restores the invariant. Rare: only steps
        // with freeze events pay it.
        qsort(world->contacts.sleepingPairKeys, (size_t)world->contacts.sleepingPairCount,
              sizeof(uint64_t), CompareKeys);
        int32_t w = 0;
        for (int32_t k = 0; k < world->contacts.sleepingPairCount; ++k)
        {
            if (w == 0 ||
                world->contacts.sleepingPairKeys[k] != world->contacts.sleepingPairKeys[w - 1])
            {
                world->contacts.sleepingPairKeys[w++] = world->contacts.sleepingPairKeys[k];
            }
        }
        world->contacts.sleepingPairCount = w;
        world->contacts.frozenDirty = 0;
    }
    world->contacts.pairCount = 0;
    int32_t maxShape = world->shapes.shapePool.maxIndex;

    // Fresh bounds, once per shape per step: the refresh,
    // the self query, and every hit re-test read this cache. Pure
    // memoization: the values are what the old per-call computes
    // produced, so the pair set cannot move by a bit. A scratch
    // stall falls back to the direct computes, same values.
    m3Aabb3d* cache =
        (m3Aabb3d*)m3StackAlloc(&world->scratch, maxShape > 0 ? maxShape * (int32_t)sizeof(m3Aabb3d)
                                                              : (int32_t)sizeof(m3Aabb3d));
    uint8_t* cacheValid = (uint8_t*)m3StackAlloc(&world->scratch, maxShape > 0 ? maxShape : 1);
    if (cache == NULL || cacheValid == NULL)
    {
        cache = NULL; // both or neither: the fallback path stays whole
        cacheValid = NULL;
    }
    if (cache != NULL)
    {
        memset(cacheValid, 0, (size_t)(maxShape > 0 ? maxShape : 1));
        for (int32_t i = 0; i < maxShape; ++i)
        {
            // A sleeping body's shape has not moved since its
            // island froze; skip the prefill (lazy on first hit) and
            // the whole refresh walk below skips it too.
            if (world->shapes.shapePool.alive[i] != 0 &&
                world->broadphase.proxyIds[i] != M3_TREE_NULL &&
                world->bodies.awake[world->shapes.shapeBody[i]] != 0)
            {
                cache[i] = SphereAabb(world, i);
                cacheValid[i] = 1;
            }
        }
    }

    // Refresh proxies in shape order: a leaf moves only when its tight
    // bounds escape the fat bounds, so the tree shape (and therefore
    // everything downstream) is a pure function of the op history.
    for (int32_t i = 0; i < maxShape; ++i)
    {
        if (world->shapes.shapePool.alive[i] == 0 ||
            world->broadphase.proxyIds[i] == M3_TREE_NULL ||
            world->bodies.awake[world->shapes.shapeBody[i]] == 0)
        {
            // A frozen body cannot escape its own fat leaf: the
            // refresh was a no-op for it every step it slept.
            continue;
        }
        m3Aabb3d tight = cache != NULL ? cache[i] : SphereAabb(world, i);
        if (!m3TreeContains(&world->broadphase.tree, world->broadphase.proxyIds[i], tight.lo,
                            tight.hi))
        {
            // Reinsert FAT, like creation does. The first draft
            // reinserted the tight box, which meant every moving
            // shape escaped its own proxy again the very next step
            // (a remove, an insert, and a rebalance per shape per
            // step: forty percent of a profiled step), and, worse,
            // approaching bodies could not pair until their exact
            // boxes touched: the speculative contact window the
            // solver is built around silently vanished for every
            // shape that had ever moved.
            m3Aabb3d fat = tight;
            for (int32_t k = 0; k < 3; ++k)
            {
                fat.lo[k] -= (double)M3_TREE_MARGIN;
                fat.hi[k] += (double)M3_TREE_MARGIN;
            }
            m3TreeMove(&world->broadphase.tree, world->broadphase.proxyIds[i], fat.lo, fat.hi);
            world->broadphase.moved[i] = 1;
        }
    }

    // Plane pass: infinite shapes pair with every allowed sphere.
    for (int32_t k = 0; k < world->shapes.planeCount; ++k)
    {
        int32_t p = world->shapes.planeShapes[k];
        for (int32_t s = 0; s < maxShape; ++s)
        {
            if (world->shapes.shapePool.alive[s] == 0 ||
                world->shapes.shapeType[s] == (uint8_t)m3_planeShape || !PairAllowed(world, p, s))
            {
                continue;
            }
            if (!PairHot(world, p, s))
            {
                continue; // cold: rides the frozen buffer
            }
            if (!EmitPair(world, p, s))
            {
                return m3_errorCapacity;
            }
        }
    }

    // Tree pass: the candidate list when it fits, else every awake
    // shape queries the tree directly. Both yield the same pairs.
    if (UpdateCandidates(world))
    {
        if (!EmitCandidates(world, cache, cacheValid))
        {
            return m3_errorCapacity;
        }
    }
    else if (QueryAwakeShapes(world, cache, cacheValid) != m3_success)
    {
        return m3_errorCapacity;
    }

    // Merge the frozen buffer: only pairs that are STILL
    // cold and alive emit; the same walk compacts the buffer, so a
    // wake or a destroy needs no bookkeeping anywhere else. The
    // fresh passes emit only hot pairs, so the union is
    // duplicate-free.
    {
        int32_t w = 0;
        for (int32_t k = 0; k < world->contacts.sleepingPairCount; ++k)
        {
            uint64_t key = world->contacts.sleepingPairKeys[k];
            int32_t i = (int32_t)(key >> 32);
            int32_t j = (int32_t)(key & 0xFFFFFFFFu);
            if (world->shapes.shapePool.alive[i] == 0 || world->shapes.shapePool.alive[j] == 0 ||
                PairHot(world, i, j))
            {
                continue;
            }
            world->contacts.sleepingPairKeys[w++] = key;
            if (world->contacts.pairCount == world->contacts.pairCapacity)
            {
                return m3_errorCapacity;
            }
            world->contacts.pairKeys[world->contacts.pairCount++] = key;
        }
        world->contacts.sleepingPairCount = w;
    }

    // One sort restores the canonical ascending order. Keys are
    // unique, so the result is independent of the sort implementation.
    qsort(world->contacts.pairKeys, (size_t)world->contacts.pairCount, sizeof(uint64_t),
          CompareKeys);
    return m3_success;
}

// The brute-force scan: the result the tree must match.
m3Result m3UpdatePairsBruteForce(m3World* world)
{
    world->contacts.pairCount = 0;
    int32_t maxShape = world->shapes.shapePool.maxIndex;
    for (int32_t i = 0; i < maxShape; ++i)
    {
        if (world->shapes.shapePool.alive[i] == 0)
        {
            continue;
        }
        for (int32_t j = i + 1; j < maxShape; ++j)
        {
            if (world->shapes.shapePool.alive[j] == 0 || !PairAllowed(world, i, j))
            {
                continue;
            }
            uint8_t typeI = world->shapes.shapeType[i];
            uint8_t typeJ = world->shapes.shapeType[j];
            int candidate;
            if (typeI == (uint8_t)m3_planeShape || typeJ == (uint8_t)m3_planeShape)
            {
                candidate = typeI != typeJ;
            }
            else
            {
                m3Aabb3d a = SphereAabb(world, i);
                m3Aabb3d b = SphereAabb(world, j);
                candidate = Overlap(&a, &b);
            }
            if (!candidate)
            {
                continue;
            }
            if (!EmitPair(world, i, j))
            {
                return m3_errorCapacity;
            }
        }
    }
    return m3_success;
}
