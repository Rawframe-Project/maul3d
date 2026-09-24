// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase tree over fat leaf boxes, AVL balanced, in double
// bounds because world positions are double and a float tree degrades
// far from the origin. The tree is snapshot state: its shape is a pure
// function of the operation history, and restoring it byte for byte
// keeps rollback exact.

#ifndef MAUL3D_SRC_DYNAMIC_TREE_H
#define MAUL3D_SRC_DYNAMIC_TREE_H

#include "maul3d/base.h"

#define M3_TREE_NULL (-1)

typedef struct m3TreeNode
{
    double lo[3];
    double hi[3];
    int32_t parent; // the free-list next when the node is free
    int32_t child1;
    int32_t child2;
    int32_t userData; // the shape index on leaves, -1 on interior nodes
    int32_t height;   // -1 = free, 0 = leaf
    uint32_t mask;    // the leaf's kind bits; for a subtree, all of its leaves' OR'd
} m3TreeNode;

_Static_assert(sizeof(m3TreeNode) == 72, "tree node must be padding-free");

typedef struct m3Tree
{
    m3TreeNode* nodes; // fixed capacity, allocated by the world
    int32_t capacity;
    int32_t root;      // M3_TREE_NULL when empty
    int32_t freeList;  // head of the free chain
    int32_t nodeCount; // nodes in the tree
    int32_t pad;
} m3Tree;

// Deep enough for any AVL tree over an int32 node count.
#define M3_TREE_STACK_CAPACITY 64

m3Tree m3TreeCreate(int32_t capacity);
void m3TreeDestroy(m3Tree* tree);

/// Insert a leaf with fat bounds. Returns the node id, or M3_TREE_NULL
/// when the pool is exhausted (loud at the caller).
int32_t m3TreeInsert(m3Tree* tree, const double lo[3], const double hi[3], int32_t userData,
                     uint32_t mask);
void m3TreeRemove(m3Tree* tree, int32_t nodeId);

/// Moves a leaf to new fat bounds. The node id stays the same, and a move
/// never needs a free node.
void m3TreeMove(m3Tree* tree, int32_t nodeId, const double lo[3], const double hi[3]);

/// Changes a leaf's kind bits.
void m3TreeSetMask(m3Tree* tree, int32_t nodeId, uint32_t mask);

/// True when the leaf's fat bounds still contain the given tight
/// bounds (no move needed).
bool m3TreeContains(const m3Tree* tree, int32_t nodeId, const double lo[3], const double hi[3]);

/// Query every leaf overlapping the bounds, in deterministic stack
/// order; the callback returns false to stop early.
typedef bool (*m3TreeQueryFn)(int32_t userData, void* context);
void m3TreeQuery(const m3Tree* tree, const double lo[3], const double hi[3], m3TreeQueryFn fn,
                 void* context);

/// The same, visiting only leaves whose kind bits meet mask; a subtree
/// without such a leaf is skipped whole.
void m3TreeQueryMask(const m3Tree* tree, const double lo[3], const double hi[3], uint32_t mask,
                     m3TreeQueryFn fn, void* context);

/// Rebuilds the whole tree top down from leaf bounds and payloads given
/// in canonical order: a median split along the axis where the centroids
/// spread widest (ties keep the input order), so the shape is a pure
/// function of the input list. Writes each input's new leaf id to
/// outNodes. Returns false, with the old tree untouched, when count
/// exceeds the capacity or memory runs out.
bool m3TreeRebuild(m3Tree* tree, const double (*los)[3], const double (*his)[3],
                   const int32_t* userDatas, const uint32_t* masks, int32_t count,
                   int32_t* outNodes);

/// Test oracle: links both ways, heights, AVL balance, box containment
/// and the node count.
bool m3TreeValidate(const m3Tree* tree);

#endif // MAUL3D_SRC_DYNAMIC_TREE_H
