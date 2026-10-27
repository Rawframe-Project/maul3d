// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Writes the fuzz targets' seeds from the shared scene, so they always
// match the formats the targets read:
//
//   fuzz_seeds <dir>
//
// writes <dir>/fuzz_snapshot/ and <dir>/fuzz_restore_step/ (the same
// snapshots), <dir>/fuzz_journal/ and <dir>/fuzz_replay/: snapshots of the
// scene before and after its session, the tapes of the build alone, of the
// build and the session and of a call aimed at the character's own
// capsule, and the replay container of the whole session from an empty
// world.

#include "fuzz_scene.h"
#include "journal.h"
#include "state_prefix.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul3d/replay.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#define MakeDir(path) _mkdir(path)
#else
#include <sys/stat.h>
#define MakeDir(path) mkdir(path, 0755)
#endif

enum
{
    TAPE_BYTES = 1 << 20
};

static void Fail(const char* what)
{
    fprintf(stderr, "fuzz_seeds: %s\n", what);
    exit(1);
}

static void WriteSeed(const char* dir, const char* target, const char* name, const void* bytes,
                      int32_t size)
{
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, target);
    MakeDir(path);
    snprintf(path, sizeof(path), "%s/%s/%s", dir, target, name);
    FILE* file = fopen(path, "wb");
    if (file == NULL || fwrite(bytes, 1, (size_t)size, file) != (size_t)size || fclose(file) != 0)
    {
        Fail(path);
    }
}

static uint8_t* SnapshotOf(const m3World* world, int32_t* bytesOut)
{
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* bytes = (uint8_t*)malloc((size_t)size);
    int32_t written = 0;
    if (bytes == NULL || m3World_Snapshot(world, bytes, size, &written) != m3_success)
    {
        Fail("a snapshot");
    }
    *bytesOut = written;
    return bytes;
}

static void WriteSnapshot(const char* dir, const char* name, const m3World* world)
{
    int32_t bytes = 0;
    uint8_t* snapshot = SnapshotOf(world, &bytes);
    WriteSeed(dir, "fuzz_snapshot", name, snapshot, bytes);
    WriteSeed(dir, "fuzz_restore_step", name, snapshot, bytes);
    free(snapshot);
}

// The scene's snapshot with its first soft body's first edge at a rest
// length far past any create gives, which restore refuses: a seed for the
// restore-then-step target at the edge of the soft bodies' rest values.
static void WriteFarRestSeed(const char* dir, const m3World* world)
{
    int32_t bytes = 0;
    uint8_t* snapshot = SnapshotOf(world, &bytes);
    int32_t slot = 0;
    while (slot < world->softBodies.softPool.maxIndex &&
           world->softBodies.softPool.alive[slot] == 0)
    {
        slot += 1;
    }
    int32_t at = m3StateFieldAt(world, offsetof(m3World, softBodies.softEdgeRest));
    if (slot >= world->softBodies.softPool.maxIndex || world->softBodies.softEdgeCount[slot] < 1 ||
        at < 0)
    {
        Fail("a soft body with an edge");
    }
    const float far = -1.0e29f;
    memcpy(snapshot + M3_SNAPSHOT_HEADER_BYTES + at +
               sizeof(float) * (size_t)slot * M3_SOFTBODY_MAX_EDGES,
           &far, sizeof(far));
    m3WorldDef def = FuzzWorldDef();
    m3World* check = NewWorld(&def);
    if (check == NULL || m3World_Restore(check, snapshot, bytes) != m3_errorInvalid)
    {
        Fail("the far rest length's refusal");
    }
    m3DestroyWorld(check);
    WriteSeed(dir, "fuzz_restore_step", "far_rest", snapshot, bytes);
    free(snapshot);
}

static m3World* FreshWorld(void)
{
    m3WorldDef def = FuzzWorldDef();
    m3World* world = NewWorld(&def);
    if (world == NULL)
    {
        Fail("the world");
    }
    return world;
}

// The scene, whole: every create succeeds and the world holds what
// fuzz_scene.h intends, kind by kind, or the seeds would silently thin.
static void BuildScene(m3World* world, FuzzScene* scene)
{
    if (!FuzzBuild(world, scene))
    {
        fprintf(stderr, "fuzz_seeds: %d of the scene's creates refused\n", scene->failures);
        exit(1);
    }
    if (!FuzzCountsHold(world, scene))
    {
        m3Counters c = m3World_GetCounters(world);
        fprintf(stderr,
                "fuzz_seeds: the scene holds %d bodies, %d shapes, %d joints, %d vehicles, %d soft "
                "bodies, %d characters, %d voxel chunks and %d meshes, not what fuzz_scene.h "
                "intends\n",
                c.bodyCount, c.shapeCount, c.jointCount, c.vehicleCount, c.softBodyCount,
                c.characterCount, c.voxelChunkCount, c.meshCount);
        exit(1);
    }
}

// What FuzzSession records, kind by kind. A refused call records
// nothing, so a session call that starts refusing would thin the tape
// silently; each kind's records past the build's must reach its floor.
typedef struct SessionOp
{
    int32_t op;
    int32_t floor;
    const char* name;
} SessionOp;

static const SessionOp s_sessionOps[] = {
    {m3_opStep, 16, "steps"},
    {m3_opSetLinearVelocity, 1, "velocity sets"},
    {m3_opApplyLinearImpulse, 1, "impulses"},
    {m3_opSetGravity, 1, "gravity sets"},
    {m3_opVoxelClear, 1, "voxel clears"},
    {m3_opWorldExplode, 1, "blasts"},
    {m3_opDestroyJoint, 1, "joint destroys"},
};

// The records of one kind on a tape, by the [i32 op][i32 size][payload]
// walk m3DescribeJournal makes, which must agree with its counts.
static int32_t CountOps(const uint8_t* tape, int32_t bytes, int32_t op)
{
    m3JournalInfo info;
    if (m3DescribeJournal(tape, bytes, &info) != m3_success)
    {
        Fail("a tape's framing");
    }
    int32_t records = 0;
    int32_t steps = 0;
    int32_t matches = 0;
    for (int32_t cursor = M3_JOURNAL_HEADER_BYTES; cursor < bytes;)
    {
        int32_t kind;
        int32_t payload;
        memcpy(&kind, tape + cursor, 4);
        memcpy(&payload, tape + cursor + 4, 4);
        records += 1;
        steps += kind == m3_opStep ? 1 : 0;
        matches += kind == op ? 1 : 0;
        cursor += 8 + payload;
    }
    if (records != info.opCount || steps != info.stepCount)
    {
        Fail("a tape walk that disagrees with m3DescribeJournal");
    }
    return matches;
}

static void SessionOpsHold(const uint8_t* build, int32_t buildBytes, const uint8_t* session,
                           int32_t sessionBytes)
{
    for (size_t i = 0; i < sizeof(s_sessionOps) / sizeof(s_sessionOps[0]); ++i)
    {
        const SessionOp* kind = &s_sessionOps[i];
        int32_t added =
            CountOps(session, sessionBytes, kind->op) - CountOps(build, buildBytes, kind->op);
        if (added < kind->floor)
        {
            fprintf(stderr, "fuzz_seeds: the session records %d %s, not the %d FuzzSession makes\n",
                    added, kind->name, kind->floor);
            exit(1);
        }
    }
}

// A tape aimed at the character's own capsule: the build, then a destroy
// recorded on the ball's sphere and pointed at the capsule, then a stance
// change. Replay must refuse it as the public call does. The calls are
// recorded on a tape of their own, whose records follow the build's.
static void WriteOwnedSeed(const char* dir, uint8_t* tape)
{
    uint8_t* calls = (uint8_t*)malloc(TAPE_BYTES);
    if (calls == NULL)
    {
        Fail("the owned seed's call tape");
    }
    m3World* world = FreshWorld();
    FuzzScene scene;
    int32_t buildBytes = 0;
    if (m3World_StartJournal(world, tape, TAPE_BYTES) != m3_success)
    {
        Fail("the owned seed's journal");
    }
    BuildScene(world, &scene);
    if (m3World_StopJournal(world, &buildBytes) != m3_success)
    {
        Fail("the owned seed's build");
    }
    m3ShapeId sphere;
    m3RayCastResult hit = m3World_CastRayClosest(
        world, (m3Pos3){-2.0, 4.0, -2.0}, (m3Vec3){0.0f, -3.0f, 0.0f}, m3DefaultQueryFilter());
    if (m3Body_GetShapes(world, scene.ball, &sphere, 1) != 1 || !hit.hit ||
        m3World_StartJournal(world, calls, TAPE_BYTES - buildBytes) != m3_success)
    {
        Fail("the owned seed's targets");
    }
    int32_t callBytes = 0;
    if (m3DestroyShape(world, sphere) != m3_success ||
        m3Character_SetStance(world, scene.hero, 0.25f, 0.2f) != m3_success ||
        m3World_StopJournal(world, &callBytes) != m3_success)
    {
        Fail("the owned seed's calls");
    }
    // The call tape's records, past its prefix, after the build's.
    callBytes -= M3_JOURNAL_HEADER_BYTES;
    memcpy(tape + buildBytes, calls + M3_JOURNAL_HEADER_BYTES, (size_t)callBytes);
    free(calls);
    int32_t found = 0;
    for (int32_t at = buildBytes; at + (int32_t)sizeof(sphere) <= buildBytes + callBytes; ++at)
    {
        if (memcmp(tape + at, &sphere, sizeof(sphere)) == 0)
        {
            memcpy(tape + at, &hit.shapeId, sizeof(hit.shapeId));
            found += 1;
        }
    }
    if (found != 1)
    {
        Fail("the owned seed's retarget");
    }
    WriteSeed(dir, "fuzz_journal", "owned", tape, buildBytes + callBytes);
    m3DestroyWorld(world);
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "usage: fuzz_seeds <dir>\n");
        return 2;
    }
    const char* dir = argv[1];
    MakeDir(dir);
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    if (tape == NULL)
    {
        Fail("the tape buffer");
    }

    // The build alone, recorded from an empty world.
    m3World* world = FreshWorld();
    int32_t emptyBytes = 0;
    uint8_t* empty = SnapshotOf(world, &emptyBytes);
    FuzzScene scene;
    int32_t tapeBytes = 0;
    if (m3World_StartJournal(world, tape, TAPE_BYTES) != m3_success)
    {
        Fail("the build's journal");
    }
    BuildScene(world, &scene);
    if (m3World_StopJournal(world, &tapeBytes) != m3_success)
    {
        Fail("the build's tape");
    }
    WriteSeed(dir, "fuzz_journal", "build", tape, tapeBytes);
    uint8_t* buildTape = (uint8_t*)malloc((size_t)tapeBytes);
    if (buildTape == NULL)
    {
        Fail("the build's tape copy");
    }
    memcpy(buildTape, tape, (size_t)tapeBytes);
    int32_t buildBytes = tapeBytes;
    WriteSnapshot(dir, "built", world);
    m3DestroyWorld(world);

    // The build and the session, recorded from an empty world, as a tape
    // and as the replay container the tools write.
    world = FreshWorld();
    if (m3World_StartJournal(world, tape, TAPE_BYTES) != m3_success)
    {
        Fail("the session's journal");
    }
    BuildScene(world, &scene);
    FuzzSession(world, &scene);
    if (m3World_StopJournal(world, &tapeBytes) != m3_success)
    {
        Fail("the session's tape");
    }
    SessionOpsHold(buildTape, buildBytes, tape, tapeBytes);
    WriteSeed(dir, "fuzz_journal", "session", tape, tapeBytes);
    WriteSnapshot(dir, "session", world);
    WriteFarRestSeed(dir, world);
    int32_t replaySize = m3GetEncodedReplaySize(emptyBytes, tapeBytes);
    uint8_t* replay = (uint8_t*)malloc((size_t)replaySize);
    int32_t replayBytes = 0;
    if (replay == NULL || m3EncodeReplay(empty, emptyBytes, tape, tapeBytes, m3World_Hash(world),
                                         replay, replaySize, &replayBytes) != m3_success)
    {
        Fail("the replay container");
    }
    WriteSeed(dir, "fuzz_replay", "session", replay, replayBytes);
    WriteOwnedSeed(dir, tape);
    printf("fuzz_seeds: snapshot %d bytes, tape %d bytes, replay %d bytes\n", emptyBytes, tapeBytes,
           replayBytes);
    m3DestroyWorld(world);
    free(replay);
    free(empty);
    free(buildTape);
    free(tape);
    return 0;
}
