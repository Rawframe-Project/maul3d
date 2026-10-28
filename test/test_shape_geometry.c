// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape geometry and positions at their edges. Replay and restore take a
// shape's geometry only in the form its create leaves it (src/shape.c,
// m3ShapeGeomCheck): a field its type does not use is zero, and a plane's
// normal is unit length. A plane's offset and every soft particle a
// create places are within M3_POSITION_LIMIT like every position, and
// the step keeps every particle within it.

#include "helpers.h"
#include "journal.h"
#include "state_prefix.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    TAPE_BYTES = 1 << 16
};

static m3World* DefaultWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    return NewWorld(&def);
}

// The largest float offset within the position limit, and the next one.
static float NearOffset(void)
{
    return (float)M3_POSITION_LIMIT;
}

static float FarOffset(void)
{
    return nextafterf((float)M3_POSITION_LIMIT, INFINITY);
}

static m3BodyId StaticBody(m3World* world)
{
    m3BodyDef bd = m3DefaultBodyDef();
    return NewBody(world, &bd);
}

// A plane at the largest offset within the limit is taken, on either
// side; one a float further is refused with one misuse and the null id.
static void TestPlaneOffsetAtTheLimit(void)
{
    m3World* world = DefaultWorld();
    m3BodyId ground = StaticBody(world);
    m3ShapeDef sd = m3DefaultShapeDef();
    const float sides[2] = {1.0f, -1.0f};
    for (int32_t i = 0; i < 2; ++i)
    {
        m3Plane near = {{0.0f, 1.0f, 0.0f}, sides[i] * NearOffset()};
        m3ShapeId id = m3_nullShapeId;
        CHECK_ACCEPTS(world, (void)m3CreatePlaneShape(world, ground, &sd, &near, &id),
                      "a plane at the limit counts no misuse");
        CHECK(id.index1 != 0, "a plane at the limit is created");
        m3Plane far = {{0.0f, 1.0f, 0.0f}, sides[i] * FarOffset()};
        m3Result result = m3_success;
        id = m3_nullShapeId;
        CHECK_REFUSES(world, result = m3CreatePlaneShape(world, ground, &sd, &far, &id),
                      "a plane past the limit counts one misuse");
        CHECK(result == m3_errorLimit && id.index1 == 0, "and is refused with the null id");
    }
    m3DestroyWorld(world);
}

// The geometry field a case writes, by its byte offset in m3ShapeGeom.
typedef struct GeomCase
{
    const char* name;
    int32_t shape; // which of Build's shapes
    size_t at;
    float value;
} GeomCase;

enum
{
    shape_box,
    shape_sphere,
    shape_plane,
    shape_capsule,
    // Created through m3_opCreateShape; the kinds below have their own ops.
    shape_plainCount,
    shape_mesh = shape_plainCount,
    shape_heightField,
    shape_voxel,
    shape_count
};

static const GeomCase s_geomCases[] = {
    {"a box's radius", shape_box, offsetof(m3ShapeGeom, s), 1.0f},
    {"a box's second point", shape_box, offsetof(m3ShapeGeom, v2) + offsetof(m3Vec3, y), 1.0f},
    {"a box's reserved field", shape_box, offsetof(m3ShapeGeom, s2), 1.0f},
    {"a sphere's second point", shape_sphere, offsetof(m3ShapeGeom, v2), 1.0f},
    {"a sphere's reserved field", shape_sphere, offsetof(m3ShapeGeom, s2), 1.0f},
    {"a plane's normal length", shape_plane, offsetof(m3ShapeGeom, v) + offsetof(m3Vec3, y), 2.0f},
    {"a plane's offset", shape_plane, offsetof(m3ShapeGeom, s), 2.0e15f},
    {"a plane's second point", shape_plane, offsetof(m3ShapeGeom, v2) + offsetof(m3Vec3, z), 1.0f},
    {"a capsule's reserved field", shape_capsule, offsetof(m3ShapeGeom, s2), 1.0f},
};

// Shapes whose content arrives through their own create: restore only.
static const GeomCase s_contentCases[] = {
    {"a mesh's radius", shape_mesh, offsetof(m3ShapeGeom, s), 1.0f},
    {"a mesh's first point", shape_mesh, offsetof(m3ShapeGeom, v) + offsetof(m3Vec3, x), 1.0f},
    {"a height field's second point", shape_heightField, offsetof(m3ShapeGeom, v2), 1.0f},
    {"a height field's reserved field", shape_heightField, offsetof(m3ShapeGeom, s2), 1.0f},
    {"a voxel chunk's first point", shape_voxel, offsetof(m3ShapeGeom, v) + offsetof(m3Vec3, y),
     1.0f},
    {"a voxel chunk's reserved field", shape_voxel, offsetof(m3ShapeGeom, s2), 1.0f},
};

// A box and a sphere on a dynamic body; a plane, a capsule, a mesh, a
// height field and a voxel chunk on a static one; shapes[i] receives each
// shape's id.
static void Build(m3World* world, m3ShapeId* shapes)
{
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId crate = NewBody(world, &bd);
    shapes[shape_box] = NewBoxShape(world, crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3Sphere sphere = {{0.0f, 1.0f, 0.0f}, 0.25f};
    shapes[shape_sphere] = NewSphereShape(world, crate, &sd, &sphere);
    m3BodyId ground = StaticBody(world);
    m3Plane plane = {{0.0f, 1.0f, 0.0f}, 0.0f};
    shapes[shape_plane] = NewPlaneShape(world, ground, &sd, &plane);
    m3Capsule capsule = {{4.0f, 1.0f, 0.0f}, {6.0f, 1.0f, 0.0f}, 0.5f};
    shapes[shape_capsule] = NewCapsuleShape(world, ground, &sd, &capsule);
    static const m3Vec3 quad[4] = {
        {-2.0f, 0.0f, 8.0f}, {2.0f, 0.0f, 8.0f}, {2.0f, 0.0f, 12.0f}, {-2.0f, 0.0f, 12.0f}};
    static const uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    shapes[shape_mesh] = NewMeshShape(world, ground, &sd, quad, 4, tris, 2);
    static const float heights[16] = {0.0f};
    shapes[shape_heightField] = NewHeightFieldGridShape(world, ground, &sd, heights, 4, 4, 1.0f);
    static uint8_t voxels[M3_VOXEL_COUNT];
    voxels[0] = 1;
    shapes[shape_voxel] = NewVoxelChunkShape(world, ground, &sd, voxels, NULL, 0.5f);
}

// A tape that builds the shapes replays; the same tape with one unused
// geometry field set, a plane normal of another length or a plane offset
// past the limit in a shape's create is refused.
static void TestReplayTakesCreatedGeometry(void)
{
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    uint8_t* edited = (uint8_t*)malloc(TAPE_BYTES);
    m3World* world = DefaultWorld();
    CHECK(tape != NULL && edited != NULL && StartJournal(world, tape, TAPE_BYTES),
          "the journal starts");
    if (tape == NULL || edited == NULL)
    {
        free(tape);
        free(edited);
        m3DestroyWorld(world);
        return;
    }
    m3ShapeId shapes[shape_count];
    Build(world, shapes);
    int32_t bytes = StopJournal(world);
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);

    m3World* again = DefaultWorld();
    CHECK(ReplayJournal(again, tape, bytes) && m3World_Hash(again) == hash, "the tape replays");
    m3DestroyWorld(again);

    // The shape creates on the tape, in create order.
    int32_t creates[shape_plainCount];
    int32_t found = 0;
    for (int32_t at = M3_JOURNAL_HEADER_BYTES; at + 8 <= bytes;)
    {
        int32_t op;
        int32_t size;
        memcpy(&op, tape + at, sizeof(op));
        memcpy(&size, tape + at + 4, sizeof(size));
        if (op == (int32_t)m3_opCreateShape && found < shape_plainCount)
        {
            creates[found] = at + 8;
            found += 1;
        }
        at += 8 + size;
    }
    CHECK(found == shape_plainCount, "every plain shape's create is on the tape");
    for (size_t c = 0;
         found == shape_plainCount && c < sizeof(s_geomCases) / sizeof(s_geomCases[0]); ++c)
    {
        const GeomCase* k = &s_geomCases[c];
        memcpy(edited, tape, (size_t)bytes);
        memcpy(edited + creates[k->shape] + offsetof(m3CreateShapeOp, geom) + k->at, &k->value,
               sizeof(k->value));
        m3World* refused = DefaultWorld();
        CHECK(m3World_ReplayJournal(refused, edited, bytes) == m3_errorFormat, k->name);
        m3DestroyWorld(refused);
    }
    free(edited);
    free(tape);
}

// A snapshot of the shapes restores; the same snapshot with one geometry
// field changed as above, or one unused field of a mesh, a height field or
// a voxel chunk set, is refused, and the world is left as it was.
static void TestRestoreTakesCreatedGeometry(void)
{
    m3World* world = DefaultWorld();
    m3ShapeId shapes[shape_count];
    Build(world, shapes);
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    uint8_t* edited = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && edited != NULL && Snapshot(world, snap, size) == size, "snapshot taken");
    int64_t field = m3StateFieldAt(world, offsetof(m3World, shapes.shapeGeom));
    CHECK(field >= 0, "shape geometry is snapshot state");
    if (snap == NULL || edited == NULL || field < 0)
    {
        free(snap);
        free(edited);
        m3DestroyWorld(world);
        return;
    }
    bool built = true;
    for (int32_t i = 0; i < shape_count; ++i)
    {
        built = built && shapes[i].index1 != 0;
    }
    CHECK(built, "every shape is created");
    CHECK(world->shapes.shapeType[shapes[shape_heightField].index1 - 1] ==
              (uint8_t)m3_heightFieldShape,
          "the grid is a height field shape");
    uint64_t hash = m3World_Hash(world);
    CHECK(Restore(world, snap, size) && m3World_Hash(world) == hash, "the snapshot restores");
    const size_t plain = sizeof(s_geomCases) / sizeof(s_geomCases[0]);
    const size_t content = sizeof(s_contentCases) / sizeof(s_contentCases[0]);
    for (size_t c = 0; c < plain + content; ++c)
    {
        const GeomCase* k = c < plain ? &s_geomCases[c] : &s_contentCases[c - plain];
        size_t slot = (size_t)(shapes[k->shape].index1 - 1);
        memcpy(edited, snap, (size_t)size);
        memcpy(edited + M3_SNAPSHOT_HEADER_BYTES + (size_t)field + slot * sizeof(m3ShapeGeom) +
                   k->at,
               &k->value, sizeof(k->value));
        CHECK(m3World_Restore(world, edited, size) == m3_errorFormat, k->name);
        CHECK(m3World_Hash(world) == hash, "the refused restore leaves the world as it was");
    }
    free(edited);
    free(snap);
    m3DestroyWorld(world);
}

// A lattice whose far corner lands on the limit is taken; one whose far
// corner lands a double past it is refused. So is a tet body with a point
// past it. The spacing of doubles at 1e15 is 0.125 m.
static void TestSoftCreatesWithinTheLimit(void)
{
    m3World* world = DefaultWorld();
    m3SoftBodyDef lattice = m3DefaultSoftBodyDef();
    lattice.countX = 3;
    lattice.countY = 1;
    lattice.countZ = 1;
    lattice.spacing = 1.0f;
    lattice.position = (m3Pos3){M3_POSITION_LIMIT - 2.0, 0.0, 0.0};
    m3SoftBodyId id = m3_nullSoftBodyId;
    CHECK_ACCEPTS(world, (void)m3CreateSoftBody(world, &lattice, &id),
                  "a lattice ending on the limit counts no misuse");
    CHECK(id.index1 != 0, "and is created");
    lattice.position.x = M3_POSITION_LIMIT - 1.875;
    m3Result result = m3_success;
    CHECK_REFUSES(world, result = m3CreateSoftBody(world, &lattice, &id),
                  "a lattice ending past the limit counts one misuse");
    CHECK(result == m3_errorLimit && id.index1 == 0, "and is refused with the null id");

    m3SoftBodyDef jelly = m3DefaultSoftBodyDef();
    m3Vec3 points[4] = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    uint16_t tet[4] = {0, 1, 2, 3};
    jelly.position = (m3Pos3){0.0, 0.0, -M3_POSITION_LIMIT};
    CHECK_ACCEPTS(world, (void)m3CreateSoftBodyTet(world, &jelly, points, 4, tet, 1, &id),
                  "a tet body on the limit counts no misuse");
    CHECK(id.index1 != 0, "and is created");
    jelly.position = (m3Pos3){0.0, M3_POSITION_LIMIT - 0.875, 0.0};
    CHECK_REFUSES(world, result = m3CreateSoftBodyTet(world, &jelly, points, 4, tet, 1, &id),
                  "a tet body with a point past the limit counts one misuse");
    CHECK(result == m3_errorLimit && id.index1 == 0, "and is refused with the null id");
    m3DestroyWorld(world);
}

static bool ParticlesWithin(const m3World* world, m3SoftBodyId body)
{
    bool within = true;
    for (int32_t i = 0; i < m3SoftBody_GetParticleCount(world, body); ++i)
    {
        m3Pos3 p = m3SoftBody_GetParticlePosition(world, body, i);
        within = within && m3BoundedPos3(p);
    }
    return within;
}

// A lattice and a tet body lie under a plane at the largest offset the
// limit takes, so the plane lifts them onto the limit. Every particle
// stays within it on every step after, and the world snapshots, restores
// and steps on alike in a second world.
static void TestParticlesStayWithinTheLimit(void)
{
    m3World* world = DefaultWorld();
    m3SoftBodyId lattice = m3_nullSoftBodyId;
    m3SoftBodyId jelly = m3_nullSoftBodyId;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane plane = {{0.0f, 1.0f, 0.0f}, NearOffset()};
    CHECK(NewPlaneShape(world, StaticBody(world), &sd, &plane).index1 != 0,
          "the plane at the limit is created");
    m3SoftBodyDef def = m3DefaultSoftBodyDef();
    def.countX = 3;
    def.countY = 3;
    def.countZ = 3;
    lattice = NewSoftBody(world, &def);
    m3Vec3 points[4] = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    uint16_t tet[4] = {0, 1, 2, 3};
    m3SoftBodyDef jellyDef = m3DefaultSoftBodyDef();
    jellyDef.position = (m3Pos3){10.0, 0.0, 0.0};
    jelly = NewSoftBodyTet(world, &jellyDef, points, 4, tet, 1);
    CHECK(lattice.index1 != 0 && jelly.index1 != 0, "both soft bodies are created");
    bool within = true;
    bool lifted = true;
    for (int32_t step = 0; step < 8; ++step)
    {
        (void)m3World_Step(world, 1.0f / 60.0f, 4);
        within = within && ParticlesWithin(world, lattice) && ParticlesWithin(world, jelly);
    }
    for (int32_t i = 0; i < m3SoftBody_GetParticleCount(world, jelly); ++i)
    {
        lifted = lifted && m3SoftBody_GetParticlePosition(world, jelly, i).y > 1.0e14;
    }
    CHECK(within, "every particle stays within the limit");
    CHECK(lifted, "the plane lifts the particles toward it");

    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    m3World* twin = DefaultWorld();
    CHECK(snap != NULL && Snapshot(world, snap, size) == size && Restore(twin, snap, size),
          "the world the step made restores in a second world");
    (void)m3World_Step(world, 1.0f / 60.0f, 4);
    (void)m3World_Step(twin, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(twin) == m3World_Hash(world), "and both step on alike");
    free(snap);
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

// Whether a static shape stays usable: finite bounds, a ray at it that
// hits at a finite point with a unit normal, and a snapshot that
// restores.
static bool StaysFinite(m3World* world, m3ShapeId shape, m3Pos3 origin, m3Vec3 ray)
{
    m3AabbResult box = m3Shape_GetAabb(world, shape);
    const double c[6] = {box.lowerBound.x, box.lowerBound.y, box.lowerBound.z,
                         box.upperBound.x, box.upperBound.y, box.upperBound.z};
    bool finite = true;
    for (int32_t k = 0; k < 6; ++k)
    {
        finite = finite && c[k] - c[k] == 0.0;
    }
    m3RayCastResult hit = m3World_CastRayClosest(world, origin, ray, m3DefaultQueryFilter());
    finite = finite && hit.hit && hit.point.x - hit.point.x == 0.0 &&
             hit.point.y - hit.point.y == 0.0 && hit.point.z - hit.point.z == 0.0 &&
             isfinite(hit.fraction) && fabsf(m3Dot3(hit.normal, hit.normal) - 1.0f) <= 1.0e-3f;
    int32_t size = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    bool restores =
        snap != NULL && Snapshot(world, snap, size) == size && Restore(world, snap, size);
    free(snap);
    return finite && restores;
}

// Static geometry within M3_GEOMETRY_LIMIT, on cells within M3_CELL_LIMIT,
// is taken and stays finite through its bounds, a ray and a restore. A
// vertex, sample or cell past its bound is refused with m3_errorLimit
// and one misuse, a NaN with m3_errorInvalid. Unbounded, a grid of cells
// near FLT_MAX had infinite bounds and a snapshot its world could not
// restore, and a mesh of 1e20 m gave rays a non-finite hit.
static void TestStaticGeometryAtTheLimit(void)
{
    m3World* world = DefaultWorld();
    m3BodyId ground = StaticBody(world);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId id = m3_nullShapeId;
    m3Result result = m3_success;

    static float heights[M3_MAX_HEIGHTFIELD_SIZE * M3_MAX_HEIGHTFIELD_SIZE];
    for (int32_t i = 0; i < M3_MAX_HEIGHTFIELD_SIZE * M3_MAX_HEIGHTFIELD_SIZE; ++i)
    {
        heights[i] = (i & 1) != 0 ? M3_GEOMETRY_LIMIT : -M3_GEOMETRY_LIMIT;
    }
    CHECK_ACCEPTS(
        world,
        result = m3CreateHeightFieldGridShape(world, ground, &sd, heights, M3_MAX_HEIGHTFIELD_SIZE,
                                              M3_MAX_HEIGHTFIELD_SIZE, M3_CELL_LIMIT, &id),
        "a grid at both limits counts no misuse");
    const double cell = (double)M3_CELL_LIMIT;
    const double reach = (double)M3_GEOMETRY_LIMIT;
    CHECK(result == m3_success &&
              StaysFinite(world, id, (m3Pos3){10.0 * cell, 2.0 * reach, 20.0 * cell},
                          (m3Vec3){0.0f, -4.0f * M3_GEOMETRY_LIMIT, 0.0f}),
          "a grid at both limits stays finite");
    const float cells[3] = {2.0f * M3_CELL_LIMIT, 3.0e38f, INFINITY};
    for (int32_t i = 0; i < 3; ++i)
    {
        CHECK_REFUSES(
            world,
            result = m3CreateHeightFieldGridShape(world, ground, &sd, heights, 4, 4, cells[i], &id),
            "a grid cell past the limit counts one misuse");
        CHECK(result == m3_errorLimit, "a grid cell past M3_CELL_LIMIT is refused as limit");
        CHECK_REFUSES(
            world,
            result = m3CreateHeightFieldShape(world, ground, &sd, heights, 4, 4, cells[i], &id),
            "a mesh height field cell past the limit counts one misuse");
        CHECK(result == m3_errorLimit,
              "a mesh height field cell past M3_CELL_LIMIT is refused as limit");
    }
    heights[5] = 2.0f * M3_GEOMETRY_LIMIT;
    CHECK_REFUSES(
        world, result = m3CreateHeightFieldGridShape(world, ground, &sd, heights, 4, 4, 1.0f, &id),
        "a far sample counts one misuse");
    CHECK(result == m3_errorLimit, "a sample past M3_GEOMETRY_LIMIT is refused as limit");
    heights[5] = NAN;
    CHECK_REFUSES(
        world, result = m3CreateHeightFieldGridShape(world, ground, &sd, heights, 4, 4, 1.0f, &id),
        "a NaN sample counts one misuse");
    CHECK(result == m3_errorInvalid, "a NaN sample is invalid");

    const float g = M3_GEOMETRY_LIMIT;
    m3Vec3 vertices[3] = {{-g, 0.0f, -g}, {g, 0.0f, -g}, {0.0f, 0.0f, g}};
    const uint16_t triangle[3] = {0, 2, 1};
    CHECK_ACCEPTS(world,
                  result = m3CreateMeshShape(world, ground, &sd, vertices, 3, triangle, 1, &id),
                  "a mesh at the limit counts no misuse");
    CHECK(result == m3_success && StaysFinite(world, id, (m3Pos3){1.0, reach, 1.0},
                                              (m3Vec3){0.0f, -2.0f * M3_GEOMETRY_LIMIT, 0.0f}),
          "a mesh at the limit stays finite");
    vertices[1].x = 2.0f * M3_GEOMETRY_LIMIT;
    CHECK_REFUSES(world,
                  result = m3CreateMeshShape(world, ground, &sd, vertices, 3, triangle, 1, &id),
                  "a far vertex counts one misuse");
    CHECK(result == m3_errorLimit, "a vertex past M3_GEOMETRY_LIMIT is refused as limit");

    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 1, sizeof(voxels));
    m3BodyId block = StaticBody(world);
    CHECK_ACCEPTS(world,
                  result =
                      m3CreateVoxelChunkShape(world, block, &sd, voxels, NULL, M3_CELL_LIMIT, &id),
                  "a voxel chunk at the limit counts no misuse");
    CHECK(result == m3_success && StaysFinite(world, id, (m3Pos3){-cell, 1.5 * cell, 1.5 * cell},
                                              (m3Vec3){2.0f * M3_CELL_LIMIT, 0.0f, 0.0f}),
          "a voxel chunk at the limit stays finite");
    CHECK_REFUSES(world,
                  result = m3CreateVoxelChunkShape(world, block, &sd, voxels, NULL, 3.0e37f, &id),
                  "a voxel cell past the limit counts one misuse");
    CHECK(result == m3_errorLimit, "a voxel cell past M3_CELL_LIMIT is refused as limit");
    m3DestroyWorld(world);
}

int main(void)
{
    TestPlaneOffsetAtTheLimit();
    TestReplayTakesCreatedGeometry();
    TestRestoreTakesCreatedGeometry();
    TestSoftCreatesWithinTheLimit();
    TestParticlesStayWithinTheLimit();
    TestStaticGeometryAtTheLimit();
    if (s_failures == 0)
    {
        printf("test_shape_geometry: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
