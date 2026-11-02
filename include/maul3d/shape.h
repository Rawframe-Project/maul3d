// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: geometry, materials, filters, compounds, meshes,
// heightfields and voxel chunks.

#ifndef MAUL3D_SHAPE_H
#define MAUL3D_SHAPE_H

#include "maul3d/body.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef int32_t m3ShapeType;
    enum
    {
        m3_sphereShape = 0,
        m3_planeShape = 1,       // static bodies only: an infinite half-space
        m3_hullShape = 2,        // convex hull: boxes and point clouds
        m3_capsuleShape = 3,     // a segment with a radius
        m3_meshShape = 4,        // static triangle soup
        m3_voxelShape = 5,       // dense 16^3 voxel chunk
        m3_heightFieldShape = 6, // native grid terrain
    };

    typedef struct m3Sphere
    {
        m3Vec3 center; // body frame, within 1e4 m
        float radius;  // positive, at most 1e4 m
    } m3Sphere;

    /// Half-space: points p with dot(normal, p) <= offset are solid,
    /// p in the body's frame as for every shape, so the plane moves with
    /// its body's pose and the shape's local transform.
    /// A native plane, instead of a huge box or a height field.
    typedef struct m3Plane
    {
        m3Vec3 normal; // normalized on create
        float offset;
    } m3Plane;

    typedef struct m3ShapeDef
    {
        /// kg/m^3, positive and at most 1e6. On a dynamic body every
        /// shape that carries mass weighs at least 1e-6 kg with every
        /// principal moment at least 1e-12 kg m^2, and the body at most
        /// 1e20 kg in all; a create past them is refused.
        float density;
        float friction;    // finite, non-negative
        float restitution; // non-negative, at most 1e4
        /// Collision filtering. A pair collides when each
        /// side's category intersects the other's mask, unless a
        /// shared nonzero groupIndex overrides: positive forces
        /// collision, negative forbids it.
        /// Defaults: category 1, mask all bits, group 0.
        M3_ALIGN8 uint64_t categoryBits;
        M3_ALIGN8 uint64_t maskBits;
        int32_t groupIndex;

        /// Rolling resistance: a dimensionless material knob
        /// braking relative rotation at contacts, mixed by maximum
        /// across the pair and scaled by the pair's extent. Zero (the default) rolls free; around
        /// 0.01 to 0.1 reads as soft ground. Without it a sphere
        /// pile literally never stops rolling or sleeping.
        float rollingResistance;

        /// Conveyor velocity of the surface, world frame: contacts
        /// drive the tangential speed toward it. Zero by default;
        /// within 1e5 m/s on every axis.
        m3Vec3 surfaceVelocity;
        M3_ALIGN8 uint64_t userData;

        /// A sensor detects overlap and fires its own begin and end
        /// events but never produces contact response: bodies pass
        /// through, bullets do not stop, sleepers are not woken.
        /// Sensors do not sense other sensors.
        bool isSensor;
        bool enableHitEvents;      // default false: streams cost
        bool enablePreSolveEvents; // default false: veto calls cost
        /// Compound offset: the shape's transform relative to
        /// its body. Identity by default; a near-unit rotation is
        /// demanded and each offset component stays within 1e4 m;
        /// garbage refuses at create.
        m3Vec3 localPosition;
        m3Quat localRotation;
        int32_t internalValue;
    } m3ShapeDef;

    /// Returns a def with pinned defaults (density 1, friction 0.6,
    /// restitution 0) and a valid cookie.
    ///
    /// @return The def, with the cookie the creates check.
    /// @par Thread safety
    /// Safe from any thread.
    M3_API m3ShapeDef m3DefaultShapeDef(void);

    /// Create a sphere on a body and recompute the body's mass from
    /// its shapes (sphere mass = density * 4/3 pi r^3, inertia
    /// 0.4 m r^2). Returns m3_errorInvalid for an invalid def or
    /// geometry, m3_errorLimit for a value past a bound in
    /// maul3d/base.h, a mass outside the mass floors and limit, or one
    /// that would carry a joint on the body past its lever ratio,
    /// m3_errorStale for a stale body and m3_errorCapacity for an
    /// exhausted pool; every shape create leaves
    /// *shapeOut the null id on any failure.
    /// Journaled. Every refusal but a full pool's, a lever's and a null
    /// world's counts one misuse.
    ///
    /// @param world     The world.
    /// @param body      The body the shape joins.
    /// @param def       The def, from m3DefaultShapeDef.
    /// @param sphere    The sphere, in the shape's frame.
    /// @param shapeOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, a null sphere, a radius that is not positive or is
    ///         NaN, or a NaN center; m3_errorLimit for a value past its bound
    ///         in maul3d/base.h, a mass outside the floors and M3_MASS_LIMIT,
    ///         or a joint or contact lever the shape would carry past its
    ///         ratio, infinities included; m3_errorCapacity for a full shape
    ///         pool; m3_errorStale for a stale body; m3_errorBusy from host
    ///         code the world is running. Every refusal but a full pool's, an
    ///         allocator's, a lever's and a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateSphereShape(m3World* world, m3BodyId body,
                                                     const m3ShapeDef* def, const m3Sphere* sphere,
                                                     m3ShapeId* shapeOut);

    /// Create a half-space on a STATIC body (a plane on a dynamic body
    /// returns m3_errorInvalid). The normal is normalized on create, and
    /// the offset is within M3_POSITION_LIMIT like every position: a
    /// farther plane returns m3_errorLimit.
    /// Journaled.
    ///
    /// @param world     The world.
    /// @param body      The body the shape joins.
    /// @param def       The def, from m3DefaultShapeDef.
    /// @param plane     The half-space, in the body's frame.
    /// @param shapeOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, a null plane, a zero or non-finite normal, a NaN
    ///         offset, or a body that is not static; m3_errorLimit for a value
    ///         past its bound in maul3d/base.h, a mass outside the floors and
    ///         M3_MASS_LIMIT, or a joint or contact lever the shape would carry
    ///         past its ratio, an offset past M3_POSITION_LIMIT included,
    ///         infinities too; m3_errorCapacity for a full shape pool;
    ///         m3_errorStale for a stale body; m3_errorBusy from host code the
    ///         world is running. Every refusal but a full pool's, an
    ///         allocator's, a lever's and a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreatePlaneShape(m3World* world, m3BodyId body,
                                                    const m3ShapeDef* def, const m3Plane* plane,
                                                    m3ShapeId* shapeOut);

    /// Create a box (a convex hull with analytic mass properties:
    /// m = 8 rho hx hy hz, I = m/3 diag(hy^2+hz^2, ...) about the
    /// center). Half extents are positive and at most 1e4. A box is a hull: it collides,
    /// casts and sweeps as one.
    /// Journaled.
    ///
    /// @param world        The world.
    /// @param body         The body the shape joins.
    /// @param def          The def, from m3DefaultShapeDef.
    /// @param halfExtents  The half extents, each positive and at most 1e4 m.
    /// @param shapeOut     Receives the new id; any failure leaves the null id
    ///                     there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, or a half extent that is not positive or is NaN;
    ///         m3_errorLimit for a value past its bound in maul3d/base.h, a
    ///         mass outside the floors and M3_MASS_LIMIT, or a joint or contact
    ///         lever the shape would carry past its ratio, infinities included;
    ///         m3_errorCapacity for a full shape pool; m3_errorStale for a
    ///         stale body; m3_errorBusy from host code the world is running.
    ///         Every refusal but a full pool's, an allocator's, a lever's and a
    ///         null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateBoxShape(m3World* world, m3BodyId body,
                                                  const m3ShapeDef* def, m3Vec3 halfExtents,
                                                  m3ShapeId* shapeOut);

    /// A capsule: the segment p1-p2 swept by a radius. Mass is the
    /// closed-form cylinder plus two hemispheres about the midpoint.
    typedef struct m3Capsule
    {
        m3Vec3 point1; // body frame, within 1e4 m
        m3Vec3 point2; // body frame, within 1e4 m
        float radius;  // positive, at most 1e4 m
    } m3Capsule;

    /// A capsule: the segment point1..point2 inflated by radius. A
    /// zero-length segment is refused (that shape is a sphere; use
    /// m3CreateSphereShape).
    /// Journaled.
    ///
    /// @param world     The world.
    /// @param body      The body the shape joins.
    /// @param def       The def, from m3DefaultShapeDef.
    /// @param capsule   The capsule, in the shape's frame.
    /// @param shapeOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, a null capsule, a NaN point, a radius that is not
    ///         positive or is NaN, or a zero-length segment; m3_errorLimit for
    ///         a value past its bound in maul3d/base.h, a mass outside the
    ///         floors and M3_MASS_LIMIT, or a joint or contact lever the shape
    ///         would carry past its ratio, infinities included;
    ///         m3_errorCapacity for a full shape pool; m3_errorStale for a
    ///         stale body; m3_errorBusy from host code the world is running.
    ///         Every refusal but a full pool's, an allocator's, a lever's and a
    ///         null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateCapsuleShape(m3World* world, m3BodyId body,
                                                      const m3ShapeDef* def,
                                                      const m3Capsule* capsule,
                                                      m3ShapeId* shapeOut);

    /// A faceted cylinder: the factory builds a
    /// 2N-vertex prism through the interned hull path (N = segments,
    /// clamped to [3, 32] so 2N fits the 64-vertex hull), so mass,
    /// contacts, casts, carving, and CCD all inherit exact hull math
    /// for the prism. The documented trade: the side is an N-gon,
    /// not a curve; 24 segments roll visually smooth at meter
    /// scales. Degenerate axes and radii are refused like every
    /// hull.
    typedef struct m3Cylinder
    {
        m3Vec3 point1; // center of one cap
        m3Vec3 point2; // center of the other
        float radius;
    } m3Cylinder;

    /// Journaled.
    ///
    /// @param world     The world.
    /// @param body      The body the shape joins.
    /// @param def       The def, from m3DefaultShapeDef.
    /// @param cylinder  The cylinder, in the shape's frame.
    /// @param segments  The side's facet count, clamped to [3, 32].
    /// @param shapeOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, a null cylinder, bad geometry or a zero-length axis;
    ///         m3_errorLimit for a value past its bound in maul3d/base.h, a
    ///         mass outside the floors and M3_MASS_LIMIT, or a joint or contact
    ///         lever the shape would carry past its ratio, infinities included;
    ///         m3_errorCapacity for a full shape pool; m3_errorStale for a
    ///         stale body; m3_errorBusy from host code the world is running.
    ///         Every refusal but a full pool's, an allocator's, a lever's and a
    ///         null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateCylinderShape(m3World* world, m3BodyId body,
                                                       const m3ShapeDef* def,
                                                       const m3Cylinder* cylinder, int32_t segments,
                                                       m3ShapeId* shapeOut);

    /// Runtime geometry replacement: swap a sphere or capsule
    /// shape's geometry in place; conversions between the two are
    /// legal. Hulls, meshes, voxels, and planes refuse loudly
    /// (interned slabs are immutable by law). Journaled; mass,
    /// extents, and the broadphase follow, and sleepers around both
    /// the old and the new bounds wake. The new geometry takes the
    /// create's bounds. Returns m3_success; m3_errorLimit when the
    /// body's total mass would pass M3_MASS_LIMIT or a joint on it its
    /// lever ratio, which changes nothing and counts no misuse;
    /// m3_errorStale for a stale id, m3_errorInvalid for the null id, a
    /// shape of another kind, a character's capsule or bad geometry,
    /// m3_errorLimit for geometry past a bound or a shape below the mass
    /// floors, and m3_errorBusy inside a step callback, which change
    /// nothing and count one misuse.
    ///
    /// @param world   The world.
    /// @param shape   The shape.
    /// @param sphere  The new sphere; null is refused as invalid.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3Shape_SetSphere(m3World* world, m3ShapeId shape,
                                                   const m3Sphere* sphere);

    /// Replaces a sphere or capsule shape's geometry with a capsule, as
    /// m3Shape_SetSphere does with a sphere.
    ///
    /// @param world    The world.
    /// @param shape    The shape.
    /// @param capsule  The new capsule; null is refused as invalid.
    /// @return m3_success, or a refusal as m3Shape_SetSphere lists.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3Shape_SetCapsule(m3World* world, m3ShapeId shape,
                                                    const m3Capsule* capsule);

    /// A convex hull built from a point cloud (QuickHull, coplanar
    /// faces merged, mass integrated). Between 4 and M3_MAX_HULL_POINTS
    /// points, each coordinate within M3_LOCAL_LIMIT of the body origin. A point within 512
    /// times the builder's tolerance (16 FLT_EPSILON of the cloud's
    /// summed extents) of an earlier point is merged into it, so the
    /// hull may lose detail up to about 1e-3 of the summed extents.
    /// Degenerate clouds (coplanar, collinear, or fewer than four
    /// points once merged) return m3_errorInvalid, and a cloud of more
    /// than M3_MAX_HULL_POINTS points m3_errorLimit. An allocator that
    /// refuses the builder's working memory returns m3_errorCapacity,
    /// with the world as it was. A hull keeps at most 64 vertices; the
    /// builder stops adding points there.
    /// Journaled.
    ///
    /// @param world     The world.
    /// @param body      The body the shape joins.
    /// @param def       The def, from m3DefaultShapeDef.
    /// @param points    The cloud, in the shape's frame.
    /// @param count     The number of points, 4 to M3_MAX_HULL_POINTS.
    /// @param shapeOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, or a degenerate cloud; m3_errorLimit for a value past
    ///         its bound in maul3d/base.h, a mass outside the floors and
    ///         M3_MASS_LIMIT, or a joint or contact lever the shape would carry
    ///         past its ratio, a cloud of more than M3_MAX_HULL_POINTS points
    ///         included, infinities too; m3_errorCapacity for a full shape pool
    ///         or an allocator that refuses the builder's memory; m3_errorStale
    ///         for a stale body; m3_errorBusy from host code the world is
    ///         running. Every refusal but a full pool's, an allocator's, a
    ///         lever's and a null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateHullShape(m3World* world, m3BodyId body,
                                                   const m3ShapeDef* def, const m3Vec3* points,
                                                   int32_t count, m3ShapeId* shapeOut);

    /// A static triangle mesh (level geometry, voxel chunk surfaces).
    /// Static bodies only; up to M3_MAX_MESH_VERTICES vertices and
    /// M3_MAX_MESH_TRIANGLES triangles per mesh, three indices each,
    /// counter-clockwise from outside: contacts cull the back side.
    /// Every vertex lies within M3_GEOMETRY_LIMIT of the body origin.
    /// Ghost collisions on shared edges are filtered by feature welding.
    /// A dynamic body, an index past the vertices or a NaN vertex returns
    /// m3_errorInvalid; a count or a vertex past its bound m3_errorLimit.
    /// Journaled.
    ///
    /// @param world          The world.
    /// @param body           The body the shape joins.
    /// @param def            The def, from m3DefaultShapeDef.
    /// @param vertices       The vertices, in the shape's frame.
    /// @param vertexCount    The number of vertices, 3 to M3_MAX_MESH_VERTICES.
    /// @param indices        Three vertex indices per triangle.
    /// @param triangleCount  The number of triangles, 1 to
    ///                       M3_MAX_MESH_TRIANGLES.
    /// @param shapeOut       Receives the new id; any failure leaves the null
    ///                       id there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, null vertices or indices, fewer than 3 vertices or 1
    ///         triangle, an index past the vertices, a NaN vertex, or a body
    ///         that is not static; m3_errorLimit for a value past its bound in
    ///         maul3d/base.h, a mass outside the floors and M3_MASS_LIMIT, or a
    ///         joint or contact lever the shape would carry past its ratio, a
    ///         count or a vertex past its bound included, infinities too;
    ///         m3_errorCapacity for a full shape pool or an allocator that
    ///         refuses the mesh; m3_errorStale for a stale body; m3_errorBusy
    ///         from host code the world is running. Every refusal but a full
    ///         pool's, an allocator's, a lever's and a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateMeshShape(m3World* world, m3BodyId body,
                                                   const m3ShapeDef* def, const m3Vec3* vertices,
                                                   int32_t vertexCount, const uint16_t* indices,
                                                   int32_t triangleCount, m3ShapeId* shapeOut);

    /// A per-triangle surface material for mesh shapes: the
    /// struck triangle's entry replaces the SHAPE's friction,
    /// restitution, and rolling resistance in the contact mix, and
    /// its surface velocity feeds the conveyor row (a moving
    /// walkway per triangle). All finite, frictions and resistances
    /// nonnegative.
    typedef struct m3MeshSurfaceMaterial
    {
        float friction;
        float restitution;
        float rollingResistance;
        m3Vec3 surfaceVelocity; // world frame, the conveyor tangent
    } m3MeshSurfaceMaterial;

    /// Paint material groups onto a mesh shape: up to 8 entries and
    /// one group index byte per triangle (triangleMaterials, length
    /// = the mesh's triangle count, every byte < materialCount).
    /// Journaled; refuses loudly on a non-mesh shape, hostile
    /// values, or out-of-range group bytes. A mesh without painted
    /// materials keeps using its shape material everywhere.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world              The world.
    /// @param shape              The shape.
    /// @param materials          The materials, materialCount of them.
    /// @param materialCount      The number of materials, 1 to
    ///                           M3_MAX_MESH_MATERIALS.
    /// @param triangleMaterials  One material index per triangle.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3MeshShape_SetMaterials(m3World* world, m3ShapeId shape,
                                         const m3MeshSurfaceMaterial* materials,
                                         int32_t materialCount, const uint8_t* triangleMaterials);

    /// A heightfield chunk: an nx by nz grid of heights (row-major,
    /// x fastest) spaced `cellSize` apart in x and z, triangulated
    /// into a static mesh through the same welded triangle path.
    /// Grid limits per chunk: 2 to M3_MAX_HEIGHTFIELD_MESH_SIZE in each
    /// direction; larger terrain tiles as chunks, the voxel-world model.
    /// The cell size is positive and within M3_CELL_LIMIT, and every
    /// height within M3_GEOMETRY_LIMIT; past either is m3_errorLimit.
    /// The grid's minimum corner sits at the body origin.
    /// Journaled.
    ///
    /// @param world     The world.
    /// @param body      The body the shape joins.
    /// @param def       The def, from m3DefaultShapeDef.
    /// @param heights   nx * nz heights, row-major with x fastest.
    /// @param nx        The samples along x, 2 to M3_MAX_HEIGHTFIELD_MESH_SIZE.
    /// @param nz        The samples along z, 2 to M3_MAX_HEIGHTFIELD_MESH_SIZE.
    /// @param cellSize  The spacing, meters, positive and within M3_CELL_LIMIT.
    /// @param shapeOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, null heights, fewer than 2 samples along an axis, or
    ///         a body that is not static; m3_errorLimit for a value past its
    ///         bound in maul3d/base.h, a mass outside the floors and
    ///         M3_MASS_LIMIT, or a joint or contact lever the shape would carry
    ///         past its ratio, a grid, cell size or height past its bound
    ///         included, infinities too; m3_errorCapacity for a full shape pool
    ///         or an allocator that refuses the mesh; m3_errorStale for a stale
    ///         body; m3_errorBusy from host code the world is running. Every
    ///         refusal but a full pool's, an allocator's, a lever's and a null
    ///         world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateHeightFieldMeshShape(m3World* world, m3BodyId body,
                                                              const m3ShapeDef* def,
                                                              const float* heights, int32_t nx,
                                                              int32_t nz, float cellSize,
                                                              m3ShapeId* shapeOut);

    /// The NATIVE grid heightfield: the same nx-by-nz grid
    /// contract as above, but stored as raw heights (four bytes per
    /// sample) instead of triangulated into a mesh: the low-memory
    /// terrain path. Grid limits per chunk: 2 to
    /// M3_MAX_HEIGHTFIELD_SIZE in each direction; larger terrain tiles
    /// as chunks. The cell size and the heights are bounded as above.
    /// Static bodies only; the minimum corner sits at the body origin.
    /// Rays, contacts, overlaps, shape casts, movers and continuous
    /// collision all see its triangles.
    /// Journaled.
    ///
    /// @param world     The world.
    /// @param body      The body the shape joins.
    /// @param def       The def, from m3DefaultShapeDef.
    /// @param heights   nx * nz heights, row-major with x fastest.
    /// @param nx        The samples along x, 2 to M3_MAX_HEIGHTFIELD_SIZE.
    /// @param nz        The samples along z, 2 to M3_MAX_HEIGHTFIELD_SIZE.
    /// @param cellSize  The spacing, meters, positive and within M3_CELL_LIMIT.
    /// @param shapeOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, null heights, fewer than 2 samples along an axis, or
    ///         a body that is not static; m3_errorLimit for a value past its
    ///         bound in maul3d/base.h, a mass outside the floors and
    ///         M3_MASS_LIMIT, or a joint or contact lever the shape would carry
    ///         past its ratio, a grid, cell size or height past its bound
    ///         included, infinities too; m3_errorCapacity for a full shape pool
    ///         or an allocator that refuses the samples; m3_errorStale for a
    ///         stale body; m3_errorBusy from host code the world is running.
    ///         Every refusal but a full pool's, an allocator's, a lever's and a
    ///         null world's counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateHeightFieldShape(m3World* world, m3BodyId body,
                                                          const m3ShapeDef* def,
                                                          const float* heights, int32_t nx,
                                                          int32_t nz, float cellSize,
                                                          m3ShapeId* shapeOut);

    /// A voxel chunk: a dense 16x16x16 grid anchored at the body
    /// origin, cells of `cellSize` meters, positive and within
    /// M3_CELL_LIMIT. `voxels` is one byte per
    /// voxel (zero = empty), x fastest then y then z; `payload` is
    /// an optional uint16 per voxel (material, health, type),
    /// carried as state and hashed. Static bodies only; sensors
    /// refused (sensor volumes are convex by contract); an empty
    /// grid refused. Solid voxels are CLOSED volumes: point-inside
    /// answers true inside filled cells. Rays, contacts, shape
    /// casts, and continuous collision all see voxel chunks; fast
    /// bodies and bullets sweep the merged surface, so a
    /// one-voxel-thick wall stops a bullet, and a wall whose voxel
    /// was cleared last step is a REAL hole.
    ///
    /// Seam welding: chunks weld automatically when both bodies
    /// carry the exact identity rotation, cell sizes match, and
    /// world positions differ by exactly one chunk extent along one
    /// axis (grid-laid level geometry). Welded borders are interior
    /// geometry and never produce contact features, so bodies roll
    /// and slide across chunk boundaries without seam impulses.
    /// Rotated or misaligned chunks still collide correctly; they
    /// just do not weld.
    /// Journaled.
    ///
    /// @param world     The world.
    /// @param body      The body the shape joins.
    /// @param def       The def, from m3DefaultShapeDef.
    /// @param voxels    4096 occupancy bytes, x fastest then y then z.
    /// @param payload   4096 payload values in the same order, or null for
    ///                  zeros.
    /// @param cellSize  The cell size, meters, positive and within
    ///                  M3_CELL_LIMIT.
    /// @param shapeOut  Receives the new id; any failure leaves the null id
    ///                  there when it is not null.
    /// @return m3_success; m3_errorInvalid for a null world, def or shapeOut,
    ///         the null body id, a def m3DefaultShapeDef did not make or a bad
    ///         def field, null voxels, a body that is not static, a sensor def
    ///         or an empty grid; m3_errorLimit for a value past its bound in
    ///         maul3d/base.h, a mass outside the floors and M3_MASS_LIMIT, or a
    ///         joint or contact lever the shape would carry past its ratio, a
    ///         cell size past M3_CELL_LIMIT included, infinities too;
    ///         m3_errorCapacity for a full shape pool, an allocator that
    ///         refuses the chunk, or a grid whose surface passes
    ///         M3_MAX_VOXEL_SURFACE_BOXES; m3_errorStale for a stale body;
    ///         m3_errorBusy from host code the world is running. Every refusal
    ///         but a full pool's, an allocator's, a lever's and a null world's
    ///         counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3CreateVoxelChunkShape(m3World* world, m3BodyId body,
                                                         const m3ShapeDef* def,
                                                         const uint8_t* voxels,
                                                         const uint16_t* payload, float cellSize,
                                                         m3ShapeId* shapeOut);

    /// Voxel edits: deterministic state transitions, journaled
    /// and replayed like every other mutation, fully inside the
    /// rollback delta. The collision surface rebuilds as a pure
    /// function of the grid after any occupancy change (a
    /// payload-only set touches no geometry), and dynamic bodies
    /// whose bounds touch the edited region are woken (a floor
    /// vanishing under a sleeper is a disturbance). Coordinates are
    /// voxel indices in [0, 15]; out-of-range and inverted regions
    /// refuse loudly. A chunk edited down to empty stays a valid
    /// shape that collides with nothing (the natural end state of
    /// destruction).
    ///
    /// Each edit returns m3_success when it landed; m3_errorStale for a
    /// stale id, m3_errorInvalid for the null id, a non-voxel shape or
    /// a bad argument, and m3_errorBusy inside a step callback, each
    /// counting one misuse. A chunk's surface holds at most
    /// M3_MAX_VOXEL_SURFACE_BOXES merged boxes; the slot reserved that
    /// much when its first chunk was created, so no edit asks the
    /// allocator. An edit whose surface would pass the bound returns
    /// m3_errorLimit: it changes nothing, journals nothing and counts
    /// no misuse, only m3Counters.voxelEditsRefused, which also counts
    /// the carves m3World_Explode loses to the bound. A create whose
    /// grid passes it is refused with m3_errorCapacity.
    ///
    /// The anchor convention: an island is anchored through the
    /// CHUNK's y = 0 base layer, not through world ground. Build
    /// structures at their chunk's base; a floating platform is a
    /// chunk whose BODY sits in the air with the platform at local
    /// y = 0. Voxels with no path to the base layer survive
    /// creation but fragment on the first clearing edit anywhere in
    /// the chunk (the sweep is chunk-wide by design).
    ///
    /// @param world    The world.
    /// @param shape    The shape.
    /// @param x        The voxel's x index, 0 to 15.
    /// @param y        The voxel's y index, 0 to 15.
    /// @param z        The voxel's z index, 0 to 15.
    /// @param payload  The value the voxel carries.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3VoxelChunkShape_SetVoxel(m3World* world, m3ShapeId shape,
                                                            int32_t x, int32_t y, int32_t z,
                                                            uint16_t payload);

    /// Clears one voxel; an edit as m3VoxelChunkShape_SetVoxel describes.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @param x      The voxel's x index, 0 to 15.
    /// @param y      The voxel's y index, 0 to 15.
    /// @param z      The voxel's z index, 0 to 15.
    /// @return m3_success, or a refusal as m3VoxelChunkShape_SetVoxel lists.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3VoxelChunkShape_ClearVoxel(m3World* world, m3ShapeId shape,
                                                              int32_t x, int32_t y, int32_t z);

    /// Clears the inclusive box [lo, hi] per axis and writes the
    /// number of voxels it cleared to *clearedOut (zero is legal). A
    /// busy refusal leaves *clearedOut as it was; any other failure
    /// zeroes it, and a NULL clearedOut is refused with m3_errorInvalid.
    ///
    /// @param world       The world.
    /// @param shape       The shape.
    /// @param lo          The box's lower voxel indices, each 0 to 15.
    /// @param hi          The box's upper voxel indices, each at least lo's.
    /// @param clearedOut  Receives how many voxels were cleared.
    /// @return m3_success, or a refusal as m3VoxelChunkShape_SetVoxel lists,
    ///         with m3_errorInvalid also for a null lo, hi or clearedOut.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3VoxelChunkShape_ClearBox(m3World* world, m3ShapeId shape,
                                                            const int32_t lo[3],
                                                            const int32_t hi[3],
                                                            int32_t* clearedOut);

    /// Fill fraction: 255 is a whole voxel, 1 a sliver. A
    /// mass and destruction property, never geometry: the voxel
    /// still collides as a full box, but its fragment mass scales
    /// by fill / 255. Refuses zero fill (that is a clear: use
    /// m3VoxelChunkShape_ClearVoxel) and empty voxels. Journaled state,
    /// hashed, inside the rollback delta like every edit.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @param x      The voxel's x index, 0 to 15.
    /// @param y      The voxel's y index, 0 to 15.
    /// @param z      The voxel's z index, 0 to 15.
    /// @param fill   The fill, 1 to 255.
    /// @return m3_success, or a refusal as m3VoxelChunkShape_SetVoxel lists.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3VoxelChunkShape_SetFill(m3World* world, m3ShapeId shape,
                                                           int32_t x, int32_t y, int32_t z,
                                                           uint8_t fill);

    /// True while the id names a live shape of this world; false for
    /// the null id and stale generations, without counting misuse.
    ///
    /// @param world  The world; null gives false.
    /// @param shape  The id.
    /// @return Whether the shape is alive.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Shape_IsValid(const m3World* world, m3ShapeId shape);

    /// The owning body, so a mover can tell its own shape from the
    /// world's. Null id for stale shapes.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return The body's id; the null id for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3BodyId m3Shape_GetBody(const m3World* world, m3ShapeId shape);

    /// Returns the shape's kind.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return The kind; m3_sphereShape for a null world, and for a stale or
    ///         null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3ShapeType m3Shape_GetType(const m3World* world, m3ShapeId shape);

    /// Returns the host's value the def or m3Shape_SetUserData set.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return The value; 0 for a null world, and for a stale or null id, which
    ///         counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API uint64_t m3Shape_GetUserData(const m3World* world, m3ShapeId shape);

    /// Sets the host's value, carried unchanged. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world     The world.
    /// @param shape     The shape.
    /// @param userData  The value.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Shape_SetUserData(m3World* world, m3ShapeId shape, uint64_t userData);

    /// Returns whether the shape is a sensor.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return Whether it is; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Shape_IsSensor(const m3World* world, m3ShapeId shape);

    /// Returns the shape's broadphase bounds, margin included.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return The bounds; a box at the origin for a null world, and for a
    ///         stale or null id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API m3WorldAabb m3Shape_GetAabb(const m3World* world, m3ShapeId shape);

    /// Who touches this shape now: fills up to capacity entries and
    /// returns the total, which may exceed capacity; a NULL array with
    /// capacity 0 measures. A stale shape is a refusal. A refusal zeroes the array over its whole
    /// capacity; a success, an empty one included, writes only its entries; a list call only reads,
    /// so none is refused as busy. See m3ContactData.
    ///
    /// @param world     The world.
    /// @param shape     The shape.
    /// @param dataOut   Receives up to capacity contacts.
    /// @param capacity  dataOut's length, at least 0.
    /// @return The total number of the shape's touching contacts; 0 on a
    ///         refusal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API int32_t m3Shape_GetContactData(const m3World* world, m3ShapeId shape,
                                          m3ContactData* dataOut, int32_t capacity);

    /// Destroy one shape and rebuild the owner's mass books.
    /// Journaled; contacts involving the shape dissolve at the next
    /// step. The last shape leaves a shapeless dynamic body at unit
    /// mass. Refused with m3_errorLimit, counting no misuse, when a
    /// joint on the owner would pass its lever ratio at the mass left.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return m3_success; m3_errorLimit as above; m3_errorInvalid for a null
    ///         world, the null id or a character's capsule; m3_errorStale for a
    ///         gone shape; m3_errorBusy from host code the world is running.
    ///         Each refusal but the lever's and a null world's counts one
    ///         misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3DestroyShape(m3World* world, m3ShapeId shape);

    /// Runtime material setters, with the def's bounds. Journaled;
    /// contacts read materials at prepare, so changes bind from the
    /// next step. A sleeping stack keeps its old mix until something
    /// wakes it.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world     The world.
    /// @param shape     The shape.
    /// @param friction  The friction, finite and not negative.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Shape_SetFriction(m3World* world, m3ShapeId shape, float friction);

    /// Returns the shape's friction.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return The friction; 0 for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Shape_GetFriction(const m3World* world, m3ShapeId shape);

    /// Sets the shape's restitution. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world        The world.
    /// @param shape        The shape.
    /// @param restitution  The restitution, 0 to 1e4.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Shape_SetRestitution(m3World* world, m3ShapeId shape, float restitution);

    /// Returns the shape's restitution.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return The restitution; 0 for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Shape_GetRestitution(const m3World* world, m3ShapeId shape);

    /// Sets the shape's rolling resistance. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @param value  The resistance, finite and not negative.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Shape_SetRollingResistance(m3World* world, m3ShapeId shape, float value);

    /// Returns the shape's rolling resistance.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return The resistance; 0 for a null world, and for a stale or null id,
    ///         which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Shape_GetRollingResistance(const m3World* world, m3ShapeId shape);

    /// Set the density and rebuild the owning body's mass, center and
    /// inertia from all its shapes: a body's mass is always its shapes'.
    /// Journaled. Returns m3_success; m3_errorLimit when a dynamic
    /// body's total mass would pass M3_MASS_LIMIT or a joint on it its
    /// lever ratio, which changes nothing and counts no misuse;
    /// m3_errorStale for a stale id, m3_errorInvalid for the null id or a
    /// density that is not positive or is NaN, m3_errorLimit for one past
    /// M3_DENSITY_LIMIT or a shape below the mass floors, and m3_errorBusy
    /// inside a step callback, which change nothing and count one misuse.
    ///
    /// @param world    The world.
    /// @param shape    The shape.
    /// @param density  The density, kg/m^3.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3Shape_SetDensity(m3World* world, m3ShapeId shape, float density);

    /// Returns the shape's density.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return The density, kg/m^3; 0 for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API float m3Shape_GetDensity(const m3World* world, m3ShapeId shape);

    /// Opt a shape into hit events / the pre-solve veto.
    /// Journaled; the flags are state and snapshot with the world.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @param flag   Whether the shape reports hit events.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Shape_EnableHitEvents(m3World* world, m3ShapeId shape, bool flag);

    /// Returns whether the shape reports hit events.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return Whether it does; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Shape_IsHitEventsEnabled(const m3World* world, m3ShapeId shape);

    /// Opts the shape into the pre-solve veto. Journaled.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @param flag   Whether the pre-solve callback sees the shape's contacts.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Shape_EnablePreSolveEvents(m3World* world, m3ShapeId shape, bool flag);

    /// Returns whether the shape takes part in the pre-solve veto.
    ///
    /// @param world  The world.
    /// @param shape  The shape.
    /// @return Whether it does; false for a null world, and for a stale or null
    ///         id, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API bool m3Shape_IsPreSolveEventsEnabled(const m3World* world, m3ShapeId shape);

    /// The collision filter, changed at runtime: shapes it touched wake
    /// and every pair is filtered again on the next step. Journaled.
    /// Returns m3_success; m3_errorLimit when the filter lets a shape of a
    /// dynamic body collide that did not, and its reach would pass the
    /// body's M3_CONTACT_LEVER_LIMIT, which changes nothing and counts no
    /// misuse; m3_errorStale for a stale id, m3_errorInvalid for the null
    /// id or a NULL world, and m3_errorBusy inside a step callback, which
    /// change nothing and count one misuse. GetFilter's out pointers may
    /// be NULL.
    ///
    /// @param world         The world.
    /// @param shape         The shape.
    /// @param categoryBits  The categories the shape belongs to.
    /// @param maskBits      The categories it collides with.
    /// @param groupIndex    The group; see m3ShapeDef.
    /// @return m3_success, or one of the refusals above.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_NODISCARD M3_API m3Result m3Shape_SetFilter(m3World* world, m3ShapeId shape,
                                                   uint64_t categoryBits, uint64_t maskBits,
                                                   int32_t groupIndex);

    /// Reads the shape's collision filter.
    ///
    /// @param world            The world.
    /// @param shape            The shape.
    /// @param categoryBitsOut  Receives the categories, 0 for a missing shape;
    ///                         may be null.
    /// @param maskBitsOut      Receives the mask, 0 for a missing shape; may be
    ///                         null.
    /// @param groupIndexOut    Receives the group, 0 for a missing shape; may
    ///                         be null.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M3_API void m3Shape_GetFilter(const m3World* world, m3ShapeId shape, uint64_t* categoryBitsOut,
                                  uint64_t* maskBitsOut, int32_t* groupIndexOut);

    /// Conveyor: a world-frame surface velocity on the
    /// shape. Contacts drive B's tangential speed relative to A
    /// toward the difference of the two surfaces, within 1e5 m/s on
    /// every axis. Journaled; state, hashed when nonzero.
    /// A stale or null id, a refused value, or a call from host code the
    /// world is running changes nothing, is not journaled and counts one
    /// misuse.
    ///
    /// @param world     The world.
    /// @param shape     The shape.
    /// @param velocity  The surface velocity, m/s, world frame.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M3_API void m3Shape_SetSurfaceVelocity(m3World* world, m3ShapeId shape, m3Vec3 velocity);

    static const m3ShapeId m3_nullShapeId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_SHAPE_H
