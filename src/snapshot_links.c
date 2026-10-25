// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A snapshot is untrusted data: it comes from another machine in a
// rollback session and from replay files other people share. Each block
// is checked on its own first (world_state.c): indices in range, flags
// that are flags, floats that are finite. This file checks what no single
// block can show: that the incoming structures are ones the engine could
// have written, so that a step on the restored world follows no broken
// link. The id pools, the body-to-shape and body-to-joint lists, the
// broadphase tree, the pair keys and the shared content each shape names
// are validated as wholes, never rebuilt: their shape and order are
// state, and a rebuild would change what a restored world resimulates.
// The per-record values are checked in snapshot_values.c.
//
// Every walk is bounded by a capacity, so a crafted cycle ends the walk
// instead of the process.

#include "snapshot_links.h"

#include "allocator.h"
#include "snapshot_bounds.h"
#include "snapshot_values.h"

#include <stddef.h>
#include <string.h>

static m3TreeNode Node(const uint8_t* nodes, int32_t i)
{
    m3TreeNode node;
    memcpy(&node, nodes + (size_t)i * sizeof(m3TreeNode), sizeof(node));
    return node;
}

// --- Id pools ------------------------------------------------------------

typedef struct PoolFields
{
    size_t generations;
    size_t alive;
    size_t freeQueue;
    size_t maxIndex;
    size_t freeHead;
    size_t freeCount;
    size_t retiredCount;
} PoolFields;

// The offsets of one pool's fields. The pool argument names a field path,
// so it cannot be parenthesized.
// NOLINTBEGIN(bugprone-macro-parentheses)
#define M3_POOL_FIELDS(pool)                                                                       \
    (PoolFields)                                                                                   \
    {                                                                                              \
        offsetof(m3World, pool.generations), offsetof(m3World, pool.alive),                        \
            offsetof(m3World, pool.freeQueue), offsetof(m3World, pool.maxIndex),                   \
            offsetof(m3World, pool.freeHead), offsetof(m3World, pool.freeCount),                   \
            offsetof(m3World, pool.retiredCount)                                                   \
    }
// NOLINTEND(bugprone-macro-parentheses)

// A pool the allocator could have left: no slot at or past maxIndex is
// alive; the free ring holds freeCount distinct dead slots below
// maxIndex; every other dead slot below it is retired, with its
// generation spent, and there are retiredCount of them.
static bool PoolValid(const m3Incoming* inc, int32_t capacity, PoolFields f)
{
    const uint8_t* alive = m3InField(inc, f.alive);
    const uint8_t* generations = m3InField(inc, f.generations);
    const uint8_t* queue = m3InField(inc, f.freeQueue);
    int32_t maxIndex = m3InI32(m3InField(inc, f.maxIndex), 0);
    int32_t freeHead = m3InI32(m3InField(inc, f.freeHead), 0);
    int32_t freeCount = m3InI32(m3InField(inc, f.freeCount), 0);
    int32_t retiredCount = m3InI32(m3InField(inc, f.retiredCount), 0);
    if (capacity == 0)
    {
        return maxIndex == 0 && freeCount == 0 && retiredCount == 0;
    }
    int32_t* queued = inc->scratch;
    memset(queued, 0, (size_t)capacity * sizeof(int32_t));
    int32_t dead = 0;
    for (int32_t i = 0; i < capacity; ++i)
    {
        if (alive[i] != 0 && i >= maxIndex)
        {
            return false;
        }
        dead += alive[i] == 0 && i < maxIndex ? 1 : 0;
    }
    if (freeCount + retiredCount != dead)
    {
        return false;
    }
    for (int32_t k = 0; k < freeCount; ++k)
    {
        int32_t slot = m3InI32(queue, (int32_t)(((int64_t)freeHead + k) % capacity));
        if (slot < 0 || slot >= maxIndex || alive[slot] != 0 || queued[slot] != 0)
        {
            return false;
        }
        queued[slot] = 1;
    }
    for (int32_t i = 0; i < maxIndex; ++i)
    {
        if (alive[i] == 0 && queued[i] == 0 && m3InU32(generations, i) != M3_LAST_GENERATION)
        {
            return false; // dead, not queued, so retired
        }
    }
    return true;
}

static bool PoolsValid(const m3Incoming* inc)
{
    const m3World* w = inc->world;
    return PoolValid(inc, w->bodies.bodyPool.capacity, M3_POOL_FIELDS(bodies.bodyPool)) &&
           PoolValid(inc, w->shapes.shapePool.capacity, M3_POOL_FIELDS(shapes.shapePool)) &&
           PoolValid(inc, w->hulls.hullPool.capacity, M3_POOL_FIELDS(hulls.hullPool)) &&
           PoolValid(inc, w->voxels.voxelPool.capacity, M3_POOL_FIELDS(voxels.voxelPool)) &&
           PoolValid(inc, w->heightFields.hfPool.capacity, M3_POOL_FIELDS(heightFields.hfPool)) &&
           PoolValid(inc, w->characters.charPool.capacity, M3_POOL_FIELDS(characters.charPool)) &&
           PoolValid(inc, w->vehicles.vehPool.capacity, M3_POOL_FIELDS(vehicles.vehPool)) &&
           PoolValid(inc, w->softBodies.softPool.capacity, M3_POOL_FIELDS(softBodies.softPool)) &&
           PoolValid(inc, w->meshes.meshPool.capacity, M3_POOL_FIELDS(meshes.meshPool)) &&
           PoolValid(inc, w->water.waterPool.capacity, M3_POOL_FIELDS(water.waterPool)) &&
           PoolValid(inc, w->joints.jointPool.capacity, M3_POOL_FIELDS(joints.jointPool));
}

// --- Shapes and their content --------------------------------------------

// The content slot a shape of this type names, and which pool it is in:
// hulls (boxes, point clouds and cylinders), meshes, voxel chunks and
// height fields; spheres, capsules and planes name none.
typedef enum ContentKind
{
    content_none,
    content_hull,
    content_mesh,
    content_voxel,
    content_heightField,
} ContentKind;

static ContentKind ShapeContent(uint8_t type)
{
    switch (type)
    {
    case m3_hullShape:
        return content_hull;
    case m3_meshShape:
        return content_mesh;
    case m3_voxelShape:
        return content_voxel;
    case m3_heightFieldShape:
        return content_heightField;
    default:
        return content_none;
    }
}

typedef struct ContentFields
{
    size_t index;     // the shape's index into the pool
    size_t refCounts; // per slot
    size_t alive;     // the pool's alive flags
    int32_t capacity;
} ContentFields;

static ContentFields Content(const m3World* w, ContentKind kind)
{
    switch (kind)
    {
    case content_hull:
        return (ContentFields){offsetof(m3World, shapes.shapeHullIndex),
                               offsetof(m3World, hulls.hullRefCounts),
                               offsetof(m3World, hulls.hullPool.alive), w->hulls.hullPool.capacity};
    case content_mesh:
        return (ContentFields){
            offsetof(m3World, shapes.shapeMeshIndex), offsetof(m3World, meshes.meshRefCounts),
            offsetof(m3World, meshes.meshPool.alive), w->meshes.meshPool.capacity};
    case content_voxel:
        return (ContentFields){
            offsetof(m3World, shapes.shapeVoxelIndex), offsetof(m3World, voxels.voxelRefCounts),
            offsetof(m3World, voxels.voxelPool.alive), w->voxels.voxelPool.capacity};
    default:
        return (ContentFields){
            offsetof(m3World, shapes.shapeHfIndex), offsetof(m3World, heightFields.hfRefCounts),
            offsetof(m3World, heightFields.hfPool.alive), w->heightFields.hfPool.capacity};
    }
}

// Every live shape names the content its type uses and no other, in a
// live slot; every slot's stored reference count is the number of live
// shapes that name it, and a slot is alive exactly when that is nonzero.
static bool ContentValid(const m3Incoming* inc)
{
    const m3World* w = inc->world;
    int32_t shapes = w->shapes.shapePool.capacity;
    const uint8_t* shapeAlive = m3InField(inc, offsetof(m3World, shapes.shapePool.alive));
    const uint8_t* shapeType = m3InField(inc, offsetof(m3World, shapes.shapeType));
    for (int32_t kind = content_hull; kind <= content_heightField; ++kind)
    {
        ContentFields f = Content(w, (ContentKind)kind);
        const uint8_t* index = m3InField(inc, f.index);
        const uint8_t* slotAlive = m3InField(inc, f.alive);
        const uint8_t* refCounts = m3InField(inc, f.refCounts);
        int32_t* counts = inc->scratch;
        memset(counts, 0, (size_t)f.capacity * sizeof(int32_t));
        for (int32_t s = 0; s < shapes; ++s)
        {
            int32_t slot = m3InI32(index, s);
            bool uses = shapeAlive[s] != 0 && ShapeContent(shapeType[s]) == (ContentKind)kind;
            if (!uses)
            {
                if (shapeAlive[s] != 0 && slot != -1)
                {
                    return false; // a live shape names content its type does not use
                }
                continue;
            }
            if (slot < 0 || slot >= f.capacity || slotAlive[slot] == 0)
            {
                return false;
            }
            counts[slot] += 1;
        }
        for (int32_t slot = 0; slot < f.capacity; ++slot)
        {
            if (m3InI32(refCounts, slot) != counts[slot] ||
                (slotAlive[slot] != 0) != (counts[slot] > 0))
            {
                return false;
            }
        }
    }
    return true;
}

// Each live body's shape list holds exactly the live shapes whose body it
// is, once each, and ends; a dead body's list is empty.
static bool ShapeListsValid(const m3Incoming* inc)
{
    const m3World* w = inc->world;
    int32_t bodies = w->bodies.bodyPool.capacity;
    int32_t shapes = w->shapes.shapePool.capacity;
    const uint8_t* bodyAlive = m3InField(inc, offsetof(m3World, bodies.bodyPool.alive));
    const uint8_t* shapeAlive = m3InField(inc, offsetof(m3World, shapes.shapePool.alive));
    const uint8_t* head = m3InField(inc, offsetof(m3World, bodies.bodyShapeHead));
    const uint8_t* next = m3InField(inc, offsetof(m3World, shapes.shapeNext));
    const uint8_t* shapeBody = m3InField(inc, offsetof(m3World, shapes.shapeBody));
    int32_t liveShapes = 0;
    for (int32_t s = 0; s < shapes; ++s)
    {
        if (shapeAlive[s] == 0)
        {
            continue;
        }
        int32_t b = m3InI32(shapeBody, s);
        if (b < 0 || bodyAlive[b] == 0)
        {
            return false;
        }
        liveShapes += 1;
    }
    int32_t listed = 0;
    for (int32_t b = 0; b < bodies; ++b)
    {
        int32_t s = m3InI32(head, b);
        if (bodyAlive[b] == 0)
        {
            if (s != -1)
            {
                return false;
            }
            continue;
        }
        // A list longer than the pool repeats a shape: a cycle.
        for (int32_t steps = 0; s != -1; ++steps, s = m3InI32(next, s))
        {
            if (steps == shapes || shapeAlive[s] == 0 || m3InI32(shapeBody, s) != b)
            {
                return false;
            }
            listed += 1;
        }
    }
    // Each list holds only its own body's shapes and none twice, so the
    // totals agree only when every live shape is listed.
    return listed == liveShapes;
}

// --- Joints ----------------------------------------------------------------

// Every live joint joins two different live bodies, and each live body's
// joint list (threaded through nextA where the body is A, nextB where it
// is B) holds exactly its joints, once each, and ends; a dead body's list
// is empty.
static bool JointListsValid(const m3Incoming* inc)
{
    const m3World* w = inc->world;
    int32_t bodies = w->bodies.bodyPool.capacity;
    int32_t joints = w->joints.jointPool.capacity;
    const uint8_t* bodyAlive = m3InField(inc, offsetof(m3World, bodies.bodyPool.alive));
    const uint8_t* jointAlive = m3InField(inc, offsetof(m3World, joints.jointPool.alive));
    const uint8_t* head = m3InField(inc, offsetof(m3World, joints.bodyJointHead));
    const uint8_t* nextA = m3InField(inc, offsetof(m3World, joints.jointNextA));
    const uint8_t* nextB = m3InField(inc, offsetof(m3World, joints.jointNextB));
    const uint8_t* bodyA = m3InField(inc, offsetof(m3World, joints.jointBodyA));
    const uint8_t* bodyB = m3InField(inc, offsetof(m3World, joints.jointBodyB));
    int32_t liveJoints = 0;
    for (int32_t j = 0; j < joints; ++j)
    {
        if (jointAlive[j] == 0)
        {
            continue;
        }
        int32_t a = m3InI32(bodyA, j);
        int32_t b = m3InI32(bodyB, j);
        if (a < 0 || b < 0 || a == b || bodyAlive[a] == 0 || bodyAlive[b] == 0)
        {
            return false;
        }
        liveJoints += 1;
    }
    int32_t listed = 0;
    for (int32_t b = 0; b < bodies; ++b)
    {
        int32_t j = m3InI32(head, b);
        if (bodyAlive[b] == 0)
        {
            if (j != -1)
            {
                return false;
            }
            continue;
        }
        for (int32_t steps = 0; j != -1; ++steps)
        {
            if (steps == joints || jointAlive[j] == 0 ||
                (m3InI32(bodyA, j) != b && m3InI32(bodyB, j) != b))
            {
                return false;
            }
            listed += 1;
            j = m3InI32(bodyA, j) == b ? m3InI32(nextA, j) : m3InI32(nextB, j);
        }
    }
    return listed == 2 * liveJoints;
}

// --- The broadphase tree -------------------------------------------------

// A box the tree could hold: no axis inverted.
static bool Ordered(const m3TreeNode* n)
{
    return n->lo[0] <= n->hi[0] && n->lo[1] <= n->hi[1] && n->lo[2] <= n->hi[2];
}

static bool Contains(const m3TreeNode* outer, const m3TreeNode* inner)
{
    for (int32_t k = 0; k < 3; ++k)
    {
        if (!(outer->lo[k] <= inner->lo[k] && inner->hi[k] <= outer->hi[k]))
        {
            return false;
        }
    }
    return true;
}

// The free chain: exactly capacity - nodeCount free nodes, linked through
// parent, ending at the null node. The block checks put every link, the
// root and the free list head in [-1, capacity), so each step is a read
// inside the pool.
static bool FreeChainValid(const uint8_t* nodes, int32_t capacity, int32_t freeList,
                           int32_t nodeCount)
{
    int32_t length = 0;
    for (int32_t id = freeList; id != M3_TREE_NULL; id = Node(nodes, id).parent)
    {
        if (length == capacity - nodeCount || Node(nodes, id).height != -1)
        {
            return false;
        }
        length += 1;
    }
    return length == capacity - nodeCount;
}

// The tree from the root: links both ways, the height rule, AVL balance,
// box containment and kind masks, as m3TreeValidate checks a live tree,
// and boxes with no axis inverted;
// every leaf the proxy of the live, non-plane shape it names. Walked with
// a stack instead of recursion: balance bounds a valid tree's depth, and
// a deeper one is refused before the stack could fill.
static bool TreeValid(const m3Incoming* inc, const uint8_t* nodes, int32_t capacity, int32_t root,
                      int32_t nodeCount, int32_t* leavesOut)
{
    const uint8_t* shapeAlive = m3InField(inc, offsetof(m3World, shapes.shapePool.alive));
    const uint8_t* shapeType = m3InField(inc, offsetof(m3World, shapes.shapeType));
    const uint8_t* proxies = m3InField(inc, offsetof(m3World, broadphase.proxyIds));
    int32_t shapes = inc->world->shapes.shapePool.capacity;
    *leavesOut = 0;
    if (root == M3_TREE_NULL)
    {
        return nodeCount == 0;
    }
    if (Node(nodes, root).parent != M3_TREE_NULL)
    {
        return false;
    }
    int32_t stack[M3_TREE_STACK_CAPACITY];
    int32_t depth = 0;
    stack[depth++] = root;
    int32_t visited = 0;
    while (depth > 0)
    {
        int32_t id = stack[--depth];
        m3TreeNode n = Node(nodes, id);
        if (++visited > nodeCount || !Ordered(&n))
        {
            return false;
        }
        if (n.height == 0)
        {
            int32_t s = n.userData;
            if (n.child1 != M3_TREE_NULL || n.child2 != M3_TREE_NULL || s < 0 || s >= shapes ||
                shapeAlive[s] == 0 || shapeType[s] == (uint8_t)m3_planeShape ||
                m3InI32(proxies, s) != id)
            {
                return false;
            }
            *leavesOut += 1;
            continue;
        }
        int32_t a = n.child1;
        int32_t b = n.child2;
        if (a < 0 || a >= capacity || b < 0 || b >= capacity || a == b || n.userData != -1)
        {
            return false;
        }
        m3TreeNode na = Node(nodes, a);
        m3TreeNode nb = Node(nodes, b);
        // In 64 bits: an incoming height can be any int32. The differences
        // below are of heights already known not to be negative.
        int64_t tallest = na.height > nb.height ? na.height : nb.height;
        if (na.parent != id || nb.parent != id || na.height < 0 || nb.height < 0 ||
            (int64_t)n.height != 1 + tallest || na.height - nb.height > 1 ||
            nb.height - na.height > 1 || !Contains(&n, &na) || !Contains(&n, &nb) ||
            n.mask != (na.mask | nb.mask))
        {
            return false;
        }
        if (depth + 2 > M3_TREE_STACK_CAPACITY)
        {
            return false; // deeper than any balanced tree in the pool
        }
        stack[depth++] = a;
        stack[depth++] = b;
    }
    return nodeCount == 2 * *leavesOut - 1;
}

// The tree and its free chain partition the node pool, and the tree's
// leaves are exactly the live, non-plane shapes' proxies: a live
// non-plane shape names a leaf, and a plane or a dead shape names none.
static bool BroadphaseValid(const m3Incoming* inc)
{
    const m3World* w = inc->world;
    int32_t capacity = w->broadphase.tree.capacity;
    const uint8_t* nodes = m3InField(inc, offsetof(m3World, broadphase.tree.nodes));
    int32_t root = m3InI32(m3InField(inc, offsetof(m3World, broadphase.tree.root)), 0);
    int32_t freeList = m3InI32(m3InField(inc, offsetof(m3World, broadphase.tree.freeList)), 0);
    int32_t nodeCount = m3InI32(m3InField(inc, offsetof(m3World, broadphase.tree.nodeCount)), 0);
    if (!FreeChainValid(nodes, capacity, freeList, nodeCount))
    {
        return false;
    }
    int32_t leaves = 0;
    if (!TreeValid(inc, nodes, capacity, root, nodeCount, &leaves))
    {
        return false;
    }
    const uint8_t* shapeAlive = m3InField(inc, offsetof(m3World, shapes.shapePool.alive));
    const uint8_t* shapeType = m3InField(inc, offsetof(m3World, shapes.shapeType));
    const uint8_t* proxies = m3InField(inc, offsetof(m3World, broadphase.proxyIds));
    int32_t proxied = 0;
    for (int32_t s = 0; s < w->shapes.shapePool.capacity; ++s)
    {
        bool wants = shapeAlive[s] != 0 && shapeType[s] != (uint8_t)m3_planeShape;
        int32_t proxy = m3InI32(proxies, s);
        if (wants != (proxy != M3_TREE_NULL))
        {
            return false;
        }
        proxied += wants ? 1 : 0;
    }
    // Each leaf names a distinct shape whose proxy is that leaf, so equal
    // counts mean every proxy is a leaf of the tree.
    return proxied == leaves;
}

// --- Contact pairs ---------------------------------------------------------

// The pair keys ascend strictly, each with the lower index first: the
// step stashes them as the old list and merges it with the new one for
// the begin and end events and the warm starts. That is all a step reads
// from them. A key may name a dead or reused shape, or shapes whose leaves
// no longer overlap: a destroy, a create or a teleport changes the shapes
// at once, while the keys are rebuilt only by the next step, so a snapshot
// taken between them holds such keys. The block checks put both halves of
// every key below the shape capacity and the count within the pair
// capacity.
static bool PairsValid(const m3Incoming* inc)
{
    const uint8_t* keys = m3InField(inc, offsetof(m3World, contacts.pairKeys));
    int32_t count = m3InI32(m3InField(inc, offsetof(m3World, contacts.pairCount)), 0);
    for (int32_t i = 0; i < count; ++i)
    {
        uint64_t key = m3InU64(keys, i);
        if ((i > 0 && key <= m3InU64(keys, i - 1)) || (key >> 32) >= (key & 0xFFFFFFFFu))
        {
            return false;
        }
    }
    return true;
}

// --- The whole ---------------------------------------------------------------

static int32_t LargestCapacity(const m3World* w)
{
    int32_t capacities[] = {
        w->bodies.bodyPool.capacity,     w->shapes.shapePool.capacity,
        w->joints.jointPool.capacity,    w->meshes.meshPool.capacity,
        w->voxels.voxelPool.capacity,    w->characters.charPool.capacity,
        w->vehicles.vehPool.capacity,    w->softBodies.softPool.capacity,
        w->water.waterPool.capacity,     w->hulls.hullPool.capacity,
        w->heightFields.hfPool.capacity,
    };
    int32_t largest = 1;
    for (size_t i = 0; i < sizeof(capacities) / sizeof(capacities[0]); ++i)
    {
        largest = capacities[i] > largest ? capacities[i] : largest;
    }
    return largest;
}

m3Result m3SnapshotLinksCheck(m3World* world, const uint8_t* in)
{
    int32_t largest = LargestCapacity(world);
    m3Incoming inc = {world, in, (int32_t*)m3AllocArray(&world->heap, largest, sizeof(int32_t))};
    if (inc.scratch == NULL)
    {
        return m3_errorCapacity;
    }
    bool valid = PoolsValid(&inc) && ContentValid(&inc) && ShapeListsValid(&inc) &&
                 JointListsValid(&inc) && BroadphaseValid(&inc) && PairsValid(&inc) &&
                 m3SnapshotValuesValid(&inc) && m3SnapshotBoundsValid(&inc);
    m3Free(&world->heap, inc.scratch);
    return valid ? m3_success : m3_errorInvalid;
}
