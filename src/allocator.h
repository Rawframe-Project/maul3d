// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The three memory lifetimes, kept structurally apart because mixing
// them is the classic mistake that defeats rollback:
//   1. Persistent rollback state: per-array M3_ALLOC allocations, every
//      one registered in the world's single snapshot walker.
//   2. Per-step scratch: the m3Stack below, reset every step, never
//      snapshotted, never aborts on exhaustion (loud failure instead).
//   3. Immutable geometry and config: interned, referenced, not copied.
// The m3IdPool is the slot allocator behind every generation-tagged
// handle: FIFO recycling with a generation bump, so a stale id can
// always be detected and slots are reused in a canonical order.

#ifndef MAUL3D_SRC_ALLOCATOR_H
#define MAUL3D_SRC_ALLOCATOR_H

#include "core.h"

#include "maul3d/base.h"

#include <stddef.h>

// The allocator a world keeps (the def's, or the C library's when the
// def leaves it zeroed) and the world's own count of the blocks it took
// and gave back. Every block the library asks for comes from a heap.
typedef struct m3Heap
{
    m3Allocator allocator;
    int64_t allocCalls;
    int64_t freeCalls;
    int64_t refusedCalls; // blocks the host's allocator refused
} m3Heap;

// The alignment of every block the library asks for, and the size of
// the header in front of it that remembers the block's size, so the
// host's free receives the size its alloc was asked for.
#define M3_ALLOC_ALIGNMENT ((size_t)_Alignof(max_align_t))

// A heap for the requested allocator: the C library stands in for a
// zeroed one. A half-set allocator (one function without the other) is
// the caller's def error and is checked before this.
m3Heap m3HeapMake(const m3Allocator* requested);

// Whether an allocator is usable: both functions or neither.
bool m3AllocatorValid(const m3Allocator* allocator);

// Persistent arrays. The field is NULL when the count is not positive,
// the byte size overflows, or the allocator refuses; callers check it.
// The snapshot walker, not this macro, is the source of truth for what
// state is.
#define M3_ALLOC(heap, field, count, type)                                                         \
    do                                                                                             \
    {                                                                                              \
        (field) = (type*)m3AllocArray((heap), (int64_t)(count), (int64_t)sizeof(type));            \
    } while (0)

// Zeroed memory, or NULL when bytes is not positive or the allocator
// refuses. Running out of memory is a refusal, never an assert.
void* m3AllocZeroed(m3Heap* heap, int32_t bytes);
// Zeroed array of count elements, or NULL when count or elementBytes is
// not positive, when count * elementBytes does not fit the allocator's
// 32-bit byte count, or when the allocator refuses.
void* m3AllocArray(m3Heap* heap, int64_t count, int64_t elementBytes);
// Returns a block to the heap that gave it. NULL is a no-op.
void m3Free(m3Heap* heap, void* memory);

/// Per-step scratch: a bump allocator with 16-byte alignment. Reset
/// once per step. Exhaustion returns NULL and latches the overflow
/// flag; the step propagates m3_errorCapacity and the world grows the
/// stack between steps. Never aborts.
typedef struct m3Stack
{
    uint8_t* base;
    int32_t capacity;
    int32_t top;
    int32_t overflow; // latched until the next reset
} m3Stack;

m3Stack m3StackCreate(m3Heap* heap, int32_t capacity);
void m3StackDestroy(m3Heap* heap, m3Stack* stack);
void* m3StackAlloc(m3Stack* stack, int32_t size);
void m3StackReset(m3Stack* stack);

/// A slot names objects under generations 0 to M3_LAST_GENERATION, then
/// retires, so a stale id never names a later object in it: a slot makes
/// 2^32 objects over a world's life.
#define M3_LAST_GENERATION UINT32_MAX

/// Slot allocator for generation-tagged handles. FIFO free queue (a
/// freed slot is reused last, in a canonical order) with a generation
/// bump on free, so stale handles fail validation instead of aliasing
/// a recycled slot. A slot retires at M3_LAST_GENERATION instead of
/// wrapping (the Maul2D rule).
typedef struct m3IdPool
{
    int32_t capacity;
    int32_t maxIndex; // high-water mark of ever-allocated slots
    uint32_t* generations;
    uint8_t* alive;
    int32_t* freeQueue; // FIFO ring
    int32_t freeHead;
    int32_t freeCount;
    int32_t retiredCount;
} m3IdPool;

m3IdPool m3IdPoolCreate(m3Heap* heap, int32_t capacity);
void m3IdPoolDestroy(m3Heap* heap, m3IdPool* pool);
/// Returns a slot index, or -1 when the pool is exhausted (loud
/// failure at the caller, never a hidden growth or an abort).
int32_t m3IdPoolAlloc(m3IdPool* pool);
void m3IdPoolFree(m3IdPool* pool, int32_t index);
int m3IdPoolValid(const m3IdPool* pool, int32_t index, uint32_t generation);

/// True when the next m3IdPoolAlloc will succeed.
static inline bool m3IdPoolHasRoom(const m3IdPool* pool)
{
    return pool->freeCount > 0 || pool->maxIndex < pool->capacity;
}

/// A pool's cursors before an allocation. A create that fails after
/// taking a slot rewinds to its mark instead of freeing the slot, so the
/// refusal leaves the pool, generations included, exactly as it was: a
/// refused create is not journaled and must not move any later id.
typedef struct m3IdPoolMark
{
    int32_t maxIndex;
    int32_t freeHead;
    int32_t freeCount;
} m3IdPoolMark;

m3IdPoolMark m3IdPoolMarkNow(const m3IdPool* pool);
/// Undoes the one allocation made since the mark; index is the slot it
/// returned.
void m3IdPoolRewind(m3IdPool* pool, m3IdPoolMark mark, int32_t index);

#endif // MAUL3D_SRC_ALLOCATOR_H
