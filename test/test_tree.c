// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase tree against a brute-force oracle: thousands of mixed
// inserts, moves and removes keep it a valid AVL tree, moves keep node
// ids, queries match a scan of every box, and a rebuild keeps the same
// leaves.

#include "dynamic_tree.h"
#include "test_harness.h"

#include <stdio.h>
#include <stdlib.h>

// Deterministic test generator (splitmix64): platform rand() differs.
static uint64_t s_state = 0x2545F4914F6CDD1DULL;
static uint64_t NextRandom(void)
{
    s_state += 0x9E3779B97F4A7C15ULL;
    uint64_t z = s_state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

static double RandomIn(double lo, double hi)
{
    return lo + (double)(NextRandom() >> 11) * (1.0 / 9007199254740992.0) * (hi - lo);
}

typedef struct Box
{
    double lo[3];
    double hi[3];
} Box;

static Box RandomBox(void)
{
    Box b;
    for (int32_t k = 0; k < 3; ++k)
    {
        b.lo[k] = RandomIn(-50.0, 50.0);
        b.hi[k] = b.lo[k] + RandomIn(0.1, 4.0);
    }
    if (NextRandom() % 16 == 0)
    {
        b.hi[0] = b.lo[0] + 80.0; // a wide floor-like box
    }
    return b;
}

static bool Overlaps(const Box* a, const Box* b)
{
    return a->lo[0] <= b->hi[0] && b->lo[0] <= a->hi[0] && a->lo[1] <= b->hi[1] &&
           b->lo[1] <= a->hi[1] && a->lo[2] <= b->hi[2] && b->lo[2] <= a->hi[2];
}

static bool CountHit(int32_t userData, void* context)
{
    (void)userData;
    *(int32_t*)context += 1;
    return true;
}

enum
{
    N = 300
};

static Box s_boxes[N];
static int32_t s_proxies[N];

static int32_t QueryMismatches(const m3Tree* tree, int32_t queries)
{
    int32_t mismatches = 0;
    for (int32_t q = 0; q < queries; ++q)
    {
        Box query = RandomBox();
        int32_t hits = 0;
        m3TreeQuery(tree, query.lo, query.hi, CountHit, &hits);
        int32_t brute = 0;
        for (int32_t i = 0; i < N; ++i)
        {
            brute += s_proxies[i] != M3_TREE_NULL && Overlaps(&s_boxes[i], &query) ? 1 : 0;
        }
        mismatches += hits != brute ? 1 : 0;
    }
    return mismatches;
}

static void TestChurn(m3Tree* tree)
{
    bool valid = true;
    bool stable = true;
    for (int32_t op = 0; op < 6000; ++op)
    {
        int32_t i = (int32_t)(NextRandom() % N);
        Box box = RandomBox();
        if (s_proxies[i] == M3_TREE_NULL)
        {
            s_proxies[i] = m3TreeInsert(tree, box.lo, box.hi, i);
            s_boxes[i] = box;
        }
        else if (NextRandom() % 3 == 0)
        {
            m3TreeRemove(tree, s_proxies[i]);
            s_proxies[i] = M3_TREE_NULL;
        }
        else
        {
            m3TreeMove(tree, s_proxies[i], box.lo, box.hi);
            stable = stable && tree->nodes[s_proxies[i]].userData == i &&
                     tree->nodes[s_proxies[i]].height == 0;
            s_boxes[i] = box;
        }
        valid = valid && m3TreeValidate(tree);
    }
    CHECK(valid, "the tree stays a valid AVL tree through the churn");
    CHECK(stable, "moves keep node ids");
    CHECK(QueryMismatches(tree, 50) == 0, "queries match brute force");
}

static void TestRebuild(m3Tree* tree)
{
    double los[N][3];
    double his[N][3];
    int32_t users[N];
    int32_t out[N];
    int32_t count = 0;
    for (int32_t i = 0; i < N; ++i)
    {
        if (s_proxies[i] != M3_TREE_NULL)
        {
            for (int32_t k = 0; k < 3; ++k)
            {
                los[count][k] = s_boxes[i].lo[k];
                his[count][k] = s_boxes[i].hi[k];
            }
            users[count++] = i;
        }
    }
    CHECK(
        m3TreeRebuild(tree, (const double (*)[3])los, (const double (*)[3])his, users, count, out),
        "the rebuild fits");
    for (int32_t j = 0; j < count; ++j)
    {
        s_proxies[users[j]] = out[j];
    }
    CHECK(m3TreeValidate(tree), "the rebuilt tree is a valid AVL tree");
    CHECK(QueryMismatches(tree, 50) == 0, "rebuilt queries match brute force");
}

int main(void)
{
    m3Tree tree = m3TreeCreate(2 * N);
    for (int32_t i = 0; i < N; ++i)
    {
        s_proxies[i] = M3_TREE_NULL;
    }
    TestChurn(&tree);
    TestRebuild(&tree);
    TestChurn(&tree);
    m3TreeDestroy(&tree);
    if (s_failures == 0)
    {
        printf("test_tree: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
