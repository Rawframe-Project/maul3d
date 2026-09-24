// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase tree: a bounding volume hierarchy over fat leaf boxes
// in double precision, kept height balanced as an AVL tree so a query
// stack of a fixed size always suffices. Double bounds keep the tree
// exact far from the origin, where world positions are double too.
//
// Insertion descends from the root toward the child whose box grows
// least by the new leaf, pairs the leaf with the node on that path that
// adds the least total surface area to the tree, then walks back up
// refitting boxes and restoring the AVL height rule with single and
// double rotations. A move detaches a leaf and reattaches it through the
// same descent, reusing its node and its old parent node, so a proxy id
// never changes while the shape lives. A rebuild splits the leaves top
// down at the median centroid along the widest axis.
//
// Every decision is a comparison of double sums and products, so the
// tree shape is a pure function of the operation history.

#include "dynamic_tree.h"

#include "allocator.h"

#include <string.h>

static void Union(const m3TreeNode* a, const double lo[3], const double hi[3], double outLo[3],
                  double outHi[3])
{
    for (int32_t k = 0; k < 3; ++k)
    {
        outLo[k] = a->lo[k] < lo[k] ? a->lo[k] : lo[k];
        outHi[k] = a->hi[k] > hi[k] ? a->hi[k] : hi[k];
    }
}

// Half the surface area: the factor two never changes a comparison.
static double HalfArea(const double lo[3], const double hi[3])
{
    double dx = hi[0] - lo[0];
    double dy = hi[1] - lo[1];
    double dz = hi[2] - lo[2];
    return dx * dy + dy * dz + dz * dx;
}

static double NodeArea(const m3TreeNode* node)
{
    return HalfArea(node->lo, node->hi);
}

// The half area of a node's box grown by another box.
static double UnionArea(const m3TreeNode* node, const double lo[3], const double hi[3])
{
    double ulo[3];
    double uhi[3];
    Union(node, lo, hi, ulo, uhi);
    return HalfArea(ulo, uhi);
}

static bool Overlap(const double aLo[3], const double aHi[3], const double bLo[3],
                    const double bHi[3])
{
    return aLo[0] <= bHi[0] && bLo[0] <= aHi[0] && aLo[1] <= bHi[1] && bLo[1] <= aHi[1] &&
           aLo[2] <= bHi[2] && bLo[2] <= aHi[2];
}

static int32_t MaxHeight(const m3TreeNode* nodes, int32_t a, int32_t b)
{
    return nodes[a].height > nodes[b].height ? nodes[a].height : nodes[b].height;
}

// Puts every node on the free chain in ascending order.
static void ResetNodes(m3Tree* tree)
{
    memset(tree->nodes, 0, (size_t)tree->capacity * sizeof(m3TreeNode));
    for (int32_t i = 0; i < tree->capacity; ++i)
    {
        tree->nodes[i].parent = i + 1 < tree->capacity ? i + 1 : M3_TREE_NULL;
        tree->nodes[i].height = -1;
        tree->nodes[i].child1 = M3_TREE_NULL;
        tree->nodes[i].child2 = M3_TREE_NULL;
    }
    tree->freeList = tree->capacity > 0 ? 0 : M3_TREE_NULL;
    tree->root = M3_TREE_NULL;
    tree->nodeCount = 0;
}

m3Tree m3TreeCreate(int32_t capacity)
{
    m3Tree tree;
    memset(&tree, 0, sizeof(tree));
    tree.root = M3_TREE_NULL;
    tree.freeList = M3_TREE_NULL;
    M3_ALLOC(tree.nodes, capacity, m3TreeNode);
    if (tree.nodes != NULL)
    {
        tree.capacity = capacity; // out of memory leaves capacity 0 for the caller
        ResetNodes(&tree);
    }
    return tree;
}

void m3TreeDestroy(m3Tree* tree)
{
    m3Free(tree->nodes);
    memset(tree, 0, sizeof(*tree));
    tree->root = M3_TREE_NULL;
    tree->freeList = M3_TREE_NULL;
}

static int32_t TakeNode(m3Tree* tree)
{
    int32_t id = tree->freeList;
    m3TreeNode* node = tree->nodes + id;
    tree->freeList = node->parent;
    tree->nodeCount += 1;
    memset(node, 0, sizeof(*node));
    node->parent = M3_TREE_NULL;
    node->child1 = M3_TREE_NULL;
    node->child2 = M3_TREE_NULL;
    node->userData = -1;
    return id;
}

static void GiveNode(m3Tree* tree, int32_t id)
{
    m3TreeNode* node = tree->nodes + id;
    memset(node, 0, sizeof(*node));
    node->parent = tree->freeList;
    node->child1 = M3_TREE_NULL;
    node->child2 = M3_TREE_NULL;
    node->height = -1;
    tree->freeList = id;
    tree->nodeCount -= 1;
}

// Points the parent of old (or the root) at replacement.
static void Relink(m3Tree* tree, int32_t parent, int32_t old, int32_t replacement)
{
    tree->nodes[replacement].parent = parent;
    if (parent == M3_TREE_NULL)
    {
        tree->root = replacement;
    }
    else if (tree->nodes[parent].child1 == old)
    {
        tree->nodes[parent].child1 = replacement;
    }
    else
    {
        tree->nodes[parent].child2 = replacement;
    }
}

static void Refit(m3TreeNode* nodes, int32_t id)
{
    m3TreeNode* node = nodes + id;
    Union(nodes + node->child1, nodes[node->child2].lo, nodes[node->child2].hi, node->lo, node->hi);
    node->height = 1 + MaxHeight(nodes, node->child1, node->child2);
    node->mask = nodes[node->child1].mask | nodes[node->child2].mask;
}

// Lifts the child on the given side (1 or 2) of top into its place. The
// lifted node keeps its own outer child and adopts top on the vacated
// side; top adopts the lifted node's inner child. Returns the new root
// of the subtree.
static int32_t Rotate(m3Tree* tree, int32_t top, int32_t side)
{
    m3TreeNode* nodes = tree->nodes;
    int32_t up = side == 1 ? nodes[top].child1 : nodes[top].child2;
    int32_t inner = side == 1 ? nodes[up].child2 : nodes[up].child1;
    Relink(tree, nodes[top].parent, top, up);
    if (side == 1)
    {
        nodes[top].child1 = inner;
        nodes[up].child2 = top;
    }
    else
    {
        nodes[top].child2 = inner;
        nodes[up].child1 = top;
    }
    nodes[inner].parent = top;
    nodes[top].parent = up;
    Refit(nodes, top);
    Refit(nodes, up);
    return up;
}

// Restores the AVL rule at node, whose two children are balanced subtrees
// of any heights. The taller child rises; when its inner grandchild is
// the taller one, that grandchild rises first so the rotation cannot
// leave the imbalance on the other side. When the heights were more than
// two apart the demoted node can still lean, so it is balanced in turn
// and the new top checked again: in effect an AVL join, which lets an
// insertion pair a leaf with a subtree of any height. Returns the root
// of the balanced subtree.
static int32_t Rebalance(m3Tree* tree, int32_t node)
{
    m3TreeNode* nodes = tree->nodes;
    for (;;)
    {
        int32_t a = nodes[node].child1;
        int32_t b = nodes[node].child2;
        int32_t skew = nodes[b].height - nodes[a].height;
        if (skew >= -1 && skew <= 1)
        {
            return node;
        }
        int32_t side = skew > 0 ? 2 : 1;
        int32_t tall = side == 1 ? a : b;
        int32_t outer = side == 1 ? nodes[tall].child1 : nodes[tall].child2;
        int32_t inner = side == 1 ? nodes[tall].child2 : nodes[tall].child1;
        if (nodes[inner].height > nodes[outer].height)
        {
            Rotate(tree, tall, side == 1 ? 2 : 1);
        }
        int32_t up = Rotate(tree, node, side);
        Rebalance(tree, node);
        Refit(nodes, up);
        node = up;
    }
}

// Refits boxes and heights from node up to the root, rebalancing on the
// way.
static void RepairUpward(m3Tree* tree, int32_t node)
{
    while (node != M3_TREE_NULL)
    {
        Refit(tree->nodes, node);
        node = Rebalance(tree, node);
        node = tree->nodes[node].parent;
    }
}

// The node a new box becomes the sibling of. Pairing box with node X adds
// a junction of area |X u box| and grows every ancestor A of X by
// |A u box| - |A|, so the added area of the whole tree is
//
//     cost(X) = |X u box| + (growth of the ancestors of X).
//
// The descent follows the child that grows least (ties to the smaller
// union, then to the first child) and keeps the cheapest node it meets.
// It stops once the growth inherited so far plus |box|, the least any
// deeper junction can cost, reaches the best cost found.
static int32_t PickSibling(const m3Tree* tree, const double lo[3], const double hi[3])
{
    const m3TreeNode* nodes = tree->nodes;
    double boxArea = HalfArea(lo, hi);
    int32_t node = tree->root;
    int32_t best = node;
    double bestCost = UnionArea(nodes + node, lo, hi);
    double inherited = 0.0;
    while (nodes[node].height > 0)
    {
        inherited += UnionArea(nodes + node, lo, hi) - NodeArea(nodes + node);
        if (inherited + boxArea >= bestCost)
        {
            break;
        }
        int32_t a = nodes[node].child1;
        int32_t b = nodes[node].child2;
        double areaA = UnionArea(nodes + a, lo, hi);
        double areaB = UnionArea(nodes + b, lo, hi);
        double growA = areaA - NodeArea(nodes + a);
        double growB = areaB - NodeArea(nodes + b);
        bool takeB = growB < growA || (growB == growA && areaB < areaA);
        node = takeB ? b : a;
        double cost = inherited + (takeB ? areaB : areaA);
        if (cost < bestCost)
        {
            best = node;
            bestCost = cost;
        }
    }
    return best;
}

// Hangs leaf in the tree under junction, a free interior node.
static void Attach(m3Tree* tree, int32_t leaf, int32_t junction)
{
    m3TreeNode* nodes = tree->nodes;
    if (tree->root == M3_TREE_NULL)
    {
        tree->root = leaf;
        nodes[leaf].parent = M3_TREE_NULL;
        return;
    }
    int32_t sibling = PickSibling(tree, nodes[leaf].lo, nodes[leaf].hi);
    Relink(tree, nodes[sibling].parent, sibling, junction);
    nodes[junction].child1 = sibling;
    nodes[junction].child2 = leaf;
    nodes[sibling].parent = junction;
    nodes[leaf].parent = junction;
    RepairUpward(tree, junction);
}

// Takes leaf out of the tree and returns its old parent node, which is
// no longer linked (M3_TREE_NULL when the leaf was the root).
static int32_t Detach(m3Tree* tree, int32_t leaf)
{
    m3TreeNode* nodes = tree->nodes;
    int32_t parent = nodes[leaf].parent;
    nodes[leaf].parent = M3_TREE_NULL;
    if (parent == M3_TREE_NULL)
    {
        tree->root = M3_TREE_NULL;
        return M3_TREE_NULL;
    }
    int32_t sibling = nodes[parent].child1 == leaf ? nodes[parent].child2 : nodes[parent].child1;
    int32_t grandParent = nodes[parent].parent;
    Relink(tree, grandParent, parent, sibling);
    RepairUpward(tree, grandParent);
    return parent;
}

static void SetBounds(m3TreeNode* node, const double lo[3], const double hi[3])
{
    for (int32_t k = 0; k < 3; ++k)
    {
        node->lo[k] = lo[k];
        node->hi[k] = hi[k];
    }
}

int32_t m3TreeInsert(m3Tree* tree, const double lo[3], const double hi[3], int32_t userData,
                     uint32_t mask)
{
    // A non-empty tree needs a junction node besides the leaf; both are
    // taken up front so a full pool refuses with the tree untouched.
    bool empty = tree->root == M3_TREE_NULL;
    if (tree->nodeCount + (empty ? 1 : 2) > tree->capacity)
    {
        return M3_TREE_NULL;
    }
    int32_t leaf = TakeNode(tree);
    SetBounds(tree->nodes + leaf, lo, hi);
    tree->nodes[leaf].userData = userData;
    tree->nodes[leaf].height = 0;
    tree->nodes[leaf].mask = mask;
    Attach(tree, leaf, empty ? M3_TREE_NULL : TakeNode(tree));
    return leaf;
}

void m3TreeRemove(m3Tree* tree, int32_t nodeId)
{
    int32_t junction = Detach(tree, nodeId);
    if (junction != M3_TREE_NULL)
    {
        GiveNode(tree, junction);
    }
    GiveNode(tree, nodeId);
}

void m3TreeMove(m3Tree* tree, int32_t nodeId, const double lo[3], const double hi[3])
{
    int32_t junction = Detach(tree, nodeId);
    SetBounds(tree->nodes + nodeId, lo, hi);
    Attach(tree, nodeId, junction);
}

void m3TreeSetMask(m3Tree* tree, int32_t nodeId, uint32_t mask)
{
    tree->nodes[nodeId].mask = mask;
    for (int32_t id = tree->nodes[nodeId].parent; id != M3_TREE_NULL; id = tree->nodes[id].parent)
    {
        m3TreeNode* node = tree->nodes + id;
        uint32_t merged = tree->nodes[node->child1].mask | tree->nodes[node->child2].mask;
        if (merged == node->mask)
        {
            break; // nothing above can change
        }
        node->mask = merged;
    }
}

bool m3TreeContains(const m3Tree* tree, int32_t nodeId, const double lo[3], const double hi[3])
{
    const m3TreeNode* node = tree->nodes + nodeId;
    return node->lo[0] <= lo[0] && node->lo[1] <= lo[1] && node->lo[2] <= lo[2] &&
           hi[0] <= node->hi[0] && hi[1] <= node->hi[1] && hi[2] <= node->hi[2];
}

// Depth first, first child first. The stack holds at most one pending
// sibling per level plus the node in hand, and an AVL tree over any
// int32 node count is under 64 levels deep.
void m3TreeQueryMask(const m3Tree* tree, const double lo[3], const double hi[3], uint32_t mask,
                     m3TreeQueryFn fn, void* context)
{
    int32_t stack[M3_TREE_STACK_CAPACITY];
    int32_t top = 0;
    if (tree->root != M3_TREE_NULL)
    {
        stack[top++] = tree->root;
    }
    while (top > 0)
    {
        const m3TreeNode* node = tree->nodes + stack[--top];
        if ((node->mask & mask) == 0 || !Overlap(node->lo, node->hi, lo, hi))
        {
            continue;
        }
        if (node->height == 0)
        {
            if (!fn(node->userData, context))
            {
                return;
            }
            continue;
        }
        M3_ASSERT(top + 2 <= M3_TREE_STACK_CAPACITY);
        stack[top++] = node->child2;
        stack[top++] = node->child1;
    }
}

void m3TreeQuery(const m3Tree* tree, const double lo[3], const double hi[3], m3TreeQueryFn fn,
                 void* context)
{
    m3TreeQueryMask(tree, lo, hi, 0xFFFFFFFFu, fn, context);
}

// Checks one subtree and returns its leaf count, or -1 on any breach:
// links both ways, the height rule, AVL balance and box containment.
static int32_t CheckSubtree(const m3Tree* tree, int32_t id)
{
    const m3TreeNode* nodes = tree->nodes;
    const m3TreeNode* n = nodes + id;
    if (n->height == 0)
    {
        return n->child1 == M3_TREE_NULL && n->child2 == M3_TREE_NULL ? 1 : -1;
    }
    int32_t a = n->child1;
    int32_t b = n->child2;
    if (a < 0 || a >= tree->capacity || b < 0 || b >= tree->capacity || nodes[a].parent != id ||
        nodes[b].parent != id || n->height != 1 + MaxHeight(nodes, a, b) ||
        nodes[a].height - nodes[b].height > 1 || nodes[b].height - nodes[a].height > 1 ||
        !m3TreeContains(tree, id, nodes[a].lo, nodes[a].hi) ||
        !m3TreeContains(tree, id, nodes[b].lo, nodes[b].hi) ||
        n->mask != (nodes[a].mask | nodes[b].mask))
    {
        return -1;
    }
    int32_t leavesA = CheckSubtree(tree, a);
    int32_t leavesB = CheckSubtree(tree, b);
    return leavesA < 0 || leavesB < 0 ? -1 : leavesA + leavesB;
}

bool m3TreeValidate(const m3Tree* tree)
{
    if (tree->root == M3_TREE_NULL)
    {
        return tree->nodeCount == 0;
    }
    if (tree->nodes[tree->root].parent != M3_TREE_NULL)
    {
        return false;
    }
    int32_t leaves = CheckSubtree(tree, tree->root);
    return leaves > 0 && tree->nodeCount == 2 * leaves - 1;
}

// --- Whole-tree rebuild ----------------------------------------------

typedef struct RebuildInput
{
    const double (*los)[3];
    const double (*his)[3];
    const int32_t* userDatas;
    const uint32_t* masks;
    int32_t* order;   // input indices, sorted range by range
    int32_t* scratch; // merge buffer, as long as order
    int32_t* outNodes;
    int32_t axis;
} RebuildInput;

// Twice the centroid on the current axis: the factor never changes an
// ordering.
static double CentroidKey(const RebuildInput* in, int32_t input)
{
    return in->los[input][in->axis] + in->his[input][in->axis];
}

// Stable merge sort of order[s, e) by centroid, ties by input index.
static void SortRange(RebuildInput* in, int32_t s, int32_t e)
{
    if (e - s < 2)
    {
        return;
    }
    int32_t m = s + (e - s) / 2;
    SortRange(in, s, m);
    SortRange(in, m, e);
    int32_t i = s;
    int32_t j = m;
    for (int32_t k = s; k < e; ++k)
    {
        bool takeRight =
            i >= m || (j < e && (CentroidKey(in, in->order[j]) < CentroidKey(in, in->order[i]) ||
                                 (CentroidKey(in, in->order[j]) == CentroidKey(in, in->order[i]) &&
                                  in->order[j] < in->order[i])));
        in->scratch[k] = takeRight ? in->order[j++] : in->order[i++];
    }
    memcpy(in->order + s, in->scratch + s, (size_t)(e - s) * sizeof(int32_t));
}

// The axis along which the centroids of order[s, e) spread widest; ties
// go to the lower axis.
static int32_t WidestAxis(RebuildInput* in, int32_t s, int32_t e)
{
    double lo[3];
    double hi[3];
    for (int32_t k = 0; k < 3; ++k)
    {
        in->axis = k;
        lo[k] = CentroidKey(in, in->order[s]);
        hi[k] = lo[k];
        for (int32_t i = s + 1; i < e; ++i)
        {
            double c = CentroidKey(in, in->order[i]);
            lo[k] = c < lo[k] ? c : lo[k];
            hi[k] = c > hi[k] ? c : hi[k];
        }
    }
    int32_t axis = 0;
    for (int32_t k = 1; k < 3; ++k)
    {
        axis = hi[k] - lo[k] > hi[axis] - lo[axis] ? k : axis;
    }
    return axis;
}

// Builds the subtree over order[s, e): one leaf, or two halves split at
// the median of the sorted centroids. Halves differ by at most one leaf,
// so the result satisfies the AVL rule.
static int32_t BuildRange(m3Tree* tree, RebuildInput* in, int32_t s, int32_t e)
{
    int32_t id = TakeNode(tree);
    if (e - s == 1)
    {
        int32_t input = in->order[s];
        SetBounds(tree->nodes + id, in->los[input], in->his[input]);
        tree->nodes[id].userData = in->userDatas[input];
        tree->nodes[id].mask = in->masks[input];
        in->outNodes[input] = id;
        return id;
    }
    in->axis = WidestAxis(in, s, e);
    SortRange(in, s, e);
    int32_t mid = s + (e - s) / 2;
    int32_t a = BuildRange(tree, in, s, mid);
    int32_t b = BuildRange(tree, in, mid, e);
    tree->nodes[id].child1 = a;
    tree->nodes[id].child2 = b;
    tree->nodes[a].parent = id;
    tree->nodes[b].parent = id;
    Refit(tree->nodes, id);
    return id;
}

bool m3TreeRebuild(m3Tree* tree, const double (*los)[3], const double (*his)[3],
                   const int32_t* userDatas, const uint32_t* masks, int32_t count,
                   int32_t* outNodes)
{
    if (count < 0 || (count > 0 && 2 * count - 1 > tree->capacity))
    {
        return false; // a tree over n leaves needs 2n - 1 nodes
    }
    int32_t* order = (int32_t*)m3AllocZeroed(2 * count * (int32_t)sizeof(int32_t) + 1);
    if (order == NULL)
    {
        return false; // the old tree stays untouched
    }
    // Every node goes back on the free chain in ascending order, so the
    // rebuilt layout is a pure function of the input list.
    ResetNodes(tree);
    for (int32_t i = 0; i < count; ++i)
    {
        order[i] = i;
    }
    RebuildInput in = {los, his, userDatas, masks, order, order + count, outNodes, 0};
    if (count > 0)
    {
        tree->root = BuildRange(tree, &in, 0, count);
    }
    m3Free(order);
    return true;
}
