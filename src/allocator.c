// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "allocator.h"

#include "fpenv.h"

#include <stdlib.h>
#include <string.h>

// src/allocator.c is the one file that reaches the C library's
// allocation functions; everything else asks a world's heap.
static void* CLibraryAlloc(size_t size, size_t alignment, void* context)
{
    (void)context;
    (void)alignment; // malloc's blocks are aligned for any type
    return malloc(size);
}

static void CLibraryFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    free(memory);
}

bool m3AllocatorValid(const m3Allocator* allocator)
{
    return (allocator->alloc == NULL) == (allocator->free == NULL);
}

m3Heap m3HeapMake(const m3Allocator* requested)
{
    m3Heap heap;
    memset(&heap, 0, sizeof(heap));
    if (requested != NULL && requested->alloc != NULL && requested->free != NULL)
    {
        heap.allocator = *requested;
    }
    else
    {
        heap.allocator.alloc = CLibraryAlloc;
        heap.allocator.free = CLibraryFree;
    }
    return heap;
}

// The allocator is host code: it runs in the host's floating-point
// environment, through a helper whose body is only that call.
static M3_NOINLINE void* HostAlloc(const m3Allocator* allocator, size_t bytes)
{
    m3FloatHost host = m3FloatHostEnter();
    void* memory = allocator->alloc(bytes, M3_ALLOC_ALIGNMENT, allocator->context);
    m3FloatHostLeave(host);
    return memory;
}

static M3_NOINLINE void HostFree(const m3Allocator* allocator, void* memory, size_t bytes)
{
    m3FloatHost host = m3FloatHostEnter();
    allocator->free(memory, bytes, M3_ALLOC_ALIGNMENT, allocator->context);
    m3FloatHostLeave(host);
}

// Every block starts with a header of M3_ALLOC_ALIGNMENT bytes holding
// the full size the allocator was asked for, so the block the caller
// sees keeps the alignment and the free hands back the same size.
void* m3AllocZeroed(m3Heap* heap, int32_t bytes)
{
    if (bytes <= 0)
    {
        M3_ASSERT(false);
        return NULL;
    }
    size_t total = (size_t)bytes + M3_ALLOC_ALIGNMENT;
    uint8_t* block = (uint8_t*)HostAlloc(&heap->allocator, total);
    if (block == NULL)
    {
        heap->refusedCalls += 1;
        return NULL;
    }
    memset(block, 0, total);
    memcpy(block, &total, sizeof(total));
    heap->allocCalls += 1;
    return block + M3_ALLOC_ALIGNMENT;
}

void* m3AllocArray(m3Heap* heap, int64_t count, int64_t elementBytes)
{
    if (count <= 0 || elementBytes <= 0 || count > (int64_t)INT32_MAX / elementBytes)
    {
        return NULL;
    }
    return m3AllocZeroed(heap, (int32_t)(count * elementBytes));
}

void m3Free(m3Heap* heap, void* memory)
{
    if (memory == NULL)
    {
        return;
    }
    uint8_t* block = (uint8_t*)memory - M3_ALLOC_ALIGNMENT;
    size_t total;
    memcpy(&total, block, sizeof(total));
    heap->freeCalls += 1;
    HostFree(&heap->allocator, block, total);
}

m3Stack m3StackCreate(m3Heap* heap, int32_t capacity)
{
    m3Stack stack;
    memset(&stack, 0, sizeof(stack));
    if (capacity > 0)
    {
        stack.base = (uint8_t*)m3AllocZeroed(heap, capacity);
        stack.capacity = stack.base != NULL ? capacity : 0;
    }
    return stack;
}

void m3StackDestroy(m3Heap* heap, m3Stack* stack)
{
    m3Free(heap, stack->base);
    memset(stack, 0, sizeof(*stack));
}

void* m3StackAlloc(m3Stack* stack, int32_t size)
{
    if (size <= 0)
    {
        M3_ASSERT(false);
        return NULL;
    }
    // 16-byte alignment keeps every scratch block SIMD-safe.
    int32_t aligned = (size + 15) & ~15;
    if (stack->top + aligned > stack->capacity)
    {
        // Loud failure: latch the flag, return NULL, keep running. The
        // caller propagates m3_errorCapacity and the world can grow the
        // stack between steps.
        stack->overflow = 1;
        return NULL;
    }
    void* memory = stack->base + stack->top;
    stack->top += aligned;
    return memory;
}

void m3StackReset(m3Stack* stack)
{
    // The step must read the overflow flag before resetting; reset
    // starts the next step clean.
    stack->top = 0;
    stack->overflow = 0;
}

m3IdPool m3IdPoolCreate(m3Heap* heap, int32_t capacity)
{
    m3IdPool pool;
    memset(&pool, 0, sizeof(pool));
    if (capacity <= 0)
    {
        M3_ASSERT(false);
        return pool;
    }
    M3_ALLOC(heap, pool.generations, capacity, uint32_t);
    M3_ALLOC(heap, pool.alive, capacity, uint8_t);
    M3_ALLOC(heap, pool.freeQueue, capacity, int32_t);
    if (pool.generations == NULL || pool.alive == NULL || pool.freeQueue == NULL)
    {
        // Out of memory: an empty pool (capacity 0) tells the caller.
        m3IdPoolDestroy(heap, &pool);
        return pool;
    }
    pool.capacity = capacity;
    return pool;
}

void m3IdPoolDestroy(m3Heap* heap, m3IdPool* pool)
{
    m3Free(heap, pool->generations);
    m3Free(heap, pool->alive);
    m3Free(heap, pool->freeQueue);
    memset(pool, 0, sizeof(*pool));
}

int32_t m3IdPoolAlloc(m3IdPool* pool)
{
    // Recycled slots first, in FIFO order (canonical reuse), then fresh
    // slots up to capacity.
    if (pool->freeCount > 0)
    {
        int32_t index = pool->freeQueue[pool->freeHead];
        pool->freeHead = (pool->freeHead + 1) % pool->capacity;
        pool->freeCount -= 1;
        pool->alive[index] = 1;
        return index;
    }
    if (pool->maxIndex < pool->capacity)
    {
        int32_t index = pool->maxIndex;
        pool->maxIndex += 1;
        pool->alive[index] = 1;
        return index;
    }
    return -1; // exhausted: the caller fails loudly
}

void m3IdPoolFree(m3IdPool* pool, int32_t index)
{
    if (index < 0 || index >= pool->capacity || pool->alive[index] == 0)
    {
        M3_ASSERT(false);
        return;
    }
    pool->alive[index] = 0;
    if (pool->generations[index] == M3_LAST_GENERATION)
    {
        // Retire instead of wrapping: a wrapped generation could make a
        // very stale handle validate again.
        pool->retiredCount += 1;
        return;
    }
    pool->generations[index] += 1;
    pool->freeQueue[(pool->freeHead + pool->freeCount) % pool->capacity] = index;
    pool->freeCount += 1;
}

m3IdPoolMark m3IdPoolMarkNow(const m3IdPool* pool)
{
    m3IdPoolMark mark = {pool->maxIndex, pool->freeHead, pool->freeCount};
    return mark;
}

void m3IdPoolRewind(m3IdPool* pool, m3IdPoolMark mark, int32_t index)
{
    if (index >= 0 && index < pool->capacity)
    {
        pool->alive[index] = 0;
    }
    pool->maxIndex = mark.maxIndex;
    pool->freeHead = mark.freeHead;
    pool->freeCount = mark.freeCount;
}

int m3IdPoolValid(const m3IdPool* pool, int32_t index, uint32_t generation)
{
    return index >= 0 && index < pool->capacity && pool->alive[index] != 0 &&
           pool->generations[index] == generation;
}
