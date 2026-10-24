// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The task dispatch: the host's hooks run in the host's floating-point
// environment, and every range they run enters the library's.

#include "parallel.h"

#include "fpenv.h"

// One dispatch's body and context, which every range the host runs
// shares.
typedef struct TaskCall
{
    m3TaskFn* fn;
    void* context;
} TaskCall;

typedef struct TaskRange
{
    const TaskCall* call;
    int32_t start;
    int32_t end;
} TaskRange;

static M3_NOINLINE void RunRange(void* context)
{
    const TaskRange* range = (const TaskRange*)context;
    range->call->fn(range->start, range->end, range->call->context);
}

// What the host's hooks run, on a thread in any floating-point mode: the
// range inside the library's environment.
static void RunInEnvironment(int32_t start, int32_t end, void* context)
{
    TaskRange range = {(const TaskCall*)context, start, end};
    m3FloatEnvRun(RunRange, &range);
}

// The hooks are host code: they run in the host's environment, each
// through a helper whose body is only that call.
static M3_NOINLINE void* Enqueue(const m3World* world, TaskCall* call, int32_t itemCount,
                                 int32_t minRange)
{
    m3FloatHost host = m3FloatHostEnter();
    void* task =
        world->enqueueTask(RunInEnvironment, itemCount, minRange, call, world->userTaskContext);
    m3FloatHostLeave(host);
    return task;
}

static M3_NOINLINE void Finish(const m3World* world, void* task)
{
    m3FloatHost host = m3FloatHostEnter();
    world->finishTask(task, world->userTaskContext);
    m3FloatHostLeave(host);
}

void m3RunTasks(m3World* world, m3TaskFn* fn, void* ctx, int32_t itemCount, int32_t minRange)
{
    TaskCall call = {fn, ctx};
    Finish(world, Enqueue(world, &call, itemCount, minRange));
}
