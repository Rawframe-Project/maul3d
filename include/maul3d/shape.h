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

    typedef enum m3ShapeType
    {
        m3_sphereShape = 0,
        m3_planeShape = 1,       // static bodies only: an infinite half-space
        m3_hullShape = 2,        // convex hull: boxes and point clouds
        m3_capsuleShape = 3,     // a segment with a radius
        m3_meshShape = 4,        // static triangle soup
        m3_voxelShape = 5,       // dense 16^3 voxel chunk
        m3_heightFieldShape = 6, // native grid terrain
    } m3ShapeType;

    typedef struct m3Sphere
    {
        m3Vec3 center; // body frame, within 1e4 m; the origin on dynamic bodies in 2a
        m3real radius; // positive, at most 1e4 m
    } m3Sphere;

    /// Half-space: points p with dot(normal, p) <= offset are solid.
    /// A native plane, instead of a huge box or a height field.
    typedef struct m3Plane
    {
        m3Vec3 normal; // normalized on create
        m3real offset;
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
    M3_API m3ShapeDef m3DefaultShapeDef(void);

    /// Create a sphere on a body and recompute the body's mass from
    /// its shapes (sphere mass = density * 4/3 pi r^3, inertia
    /// 0.4 m r^2). Returns m3_errorInvalid for an invalid def or
    /// geometry, m3_errorStale for a stale body and m3_errorCapacity for
    /// an exhausted pool; every shape create leaves *shapeOut the null
    /// id on any failure.
    M3_NODISCARD M3_API m3Result m3CreateSphereShape(m3World* world, m3BodyId body,
                                                     const m3ShapeDef* def, const m3Sphere* sphere,
                                                     m3ShapeId* shapeOut);

    /// Create a half-space on a STATIC body (a plane on a dynamic body
    /// returns m3_errorInvalid). The normal is normalized on create.
    M3_NODISCARD M3_API m3Result m3CreatePlaneShape(m3World* world, m3BodyId body,
                                                    const m3ShapeDef* def, const m3Plane* plane,
                                                    m3ShapeId* shapeOut);

    /// Create a box (a convex hull with analytic mass properties:
    /// m = 8 rho hx hy hz, I = m/3 diag(hy^2+hz^2, ...) about the
    /// center). Half extents are positive and at most 1e4. Hull collision lands in the GJK and SAT
    /// slices; until then a box participates in the broadphase and in mass only.
    M3_NODISCARD M3_API m3Result m3CreateBoxShape(m3World* world, m3BodyId body,
                                                  const m3ShapeDef* def, m3Vec3 halfExtents,
                                                  m3ShapeId* shapeOut);

    /// A capsule: the segment p1-p2 swept by a radius. Mass is the
    /// closed-form cylinder plus two hemispheres about the midpoint.
    typedef struct m3Capsule
    {
        m3Vec3 point1; // body frame, within 1e4 m
        m3Vec3 point2; // body frame, within 1e4 m
        m3real radius; // positive, at most 1e4 m
    } m3Capsule;

    /// A capsule: the segment point1..point2 inflated by radius. A
    /// zero-length segment is refused (that shape is a sphere; use
    /// m3CreateSphereShape).
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
        m3real radius;
    } m3Cylinder;

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
    /// create's bounds, the mass floors and limit included.
    M3_API void m3Shape_SetSphere(m3World* world, m3ShapeId shape, const m3Sphere* sphere);
    M3_API void m3Shape_SetCapsule(m3World* world, m3ShapeId shape, const m3Capsule* capsule);

    /// A convex hull built from a point cloud (QuickHull, coplanar
    /// faces merged, mass integrated). Between 4 and 64 points, each
    /// coordinate within 1e4 of the body origin. A point within 512
    /// times the builder's tolerance (16 FLT_EPSILON of the cloud's
    /// summed extents) of an earlier point is merged into it, so the
    /// hull may lose detail up to about 1e-3 of the summed extents.
    /// Degenerate clouds (coplanar, collinear, or fewer than four
    /// points once merged) and hulls that exceed the 64-vertex budget
    /// (input clouds up to 256 points) return m3_errorInvalid.
    M3_NODISCARD M3_API m3Result m3CreateHullShape(m3World* world, m3BodyId body,
                                                   const m3ShapeDef* def, const m3Vec3* points,
                                                   int32_t count, m3ShapeId* shapeOut);

    /// A static triangle mesh (level geometry, voxel chunk surfaces).
    /// Static bodies only; up to 65,535 vertices and 65,535 triangles
    /// per mesh (three indices each, counter-clockwise from outside:
    /// contacts cull the back side). Ghost collisions on shared edges
    /// are filtered by feature welding. Refused loudly on a dynamic
    /// body, out-of-cap counts, or out-of-range indices.
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
    M3_API void m3Shape_SetMeshMaterials(m3World* world, m3ShapeId shape,
                                         const m3MeshSurfaceMaterial* materials,
                                         int32_t materialCount, const uint8_t* triangleMaterials);

    /// A heightfield chunk: an nx by nz grid of heights (row-major,
    /// x fastest) spaced `cellSize` apart in x and z, triangulated
    /// into a static mesh through the same welded triangle path.
    /// Grid limits per chunk: 2..32 in each direction (the mesh caps);
    /// larger terrain tiles as chunks, the voxel-world model. The
    /// grid's minimum corner sits at the body origin.
    M3_NODISCARD M3_API m3Result m3CreateHeightFieldShape(m3World* world, m3BodyId body,
                                                          const m3ShapeDef* def,
                                                          const float* heights, int32_t nx,
                                                          int32_t nz, m3real cellSize,
                                                          m3ShapeId* shapeOut);

    /// The NATIVE grid heightfield: the same nx-by-nz grid
    /// contract as above, but stored as raw heights (four bytes per
    /// sample) instead of triangulated into a mesh: the low-memory
    /// terrain path. Grid limits per chunk: 2..255 in each
    /// direction; larger terrain tiles as chunks. Static bodies
    /// only; the minimum corner sits at the body origin.
    M3_NODISCARD M3_API m3Result m3CreateHeightFieldGridShape(m3World* world, m3BodyId body,
                                                              const m3ShapeDef* def,
                                                              const float* heights, int32_t nx,
                                                              int32_t nz, m3real cellSize,
                                                              m3ShapeId* shapeOut);

    /// A voxel chunk: a dense 16x16x16 grid anchored at the body
    /// origin, cells of `cellSize` meters. `voxels` is one byte per
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
    M3_NODISCARD M3_API m3Result m3CreateVoxelChunkShape(m3World* world, m3BodyId body,
                                                         const m3ShapeDef* def,
                                                         const uint8_t* voxels,
                                                         const uint16_t* payload, m3real cellSize,
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
    /// A chunk's surface holds at most 2048 merged boxes, the
    /// checkerboard's count; the slot reserved that much when its
    /// first chunk was created, so no edit asks the allocator. An
    /// edit whose surface would pass the bound changes nothing,
    /// journals nothing and counts no misuse: it counts in
    /// m3Counters.voxelEditsRefused instead, and m3VoxelChunk_ClearBox
    /// returns -1. A create whose grid passes it is refused with
    /// m3_errorCapacity.
    ///
    /// The anchor convention: an island is anchored through the
    /// CHUNK's y = 0 base layer, not through world ground. Build
    /// structures at their chunk's base; a floating platform is a
    /// chunk whose BODY sits in the air with the platform at local
    /// y = 0. Voxels with no path to the base layer survive
    /// creation but fragment on the first clearing edit anywhere in
    /// the chunk (the sweep is chunk-wide by design).
    M3_API void m3VoxelChunk_SetVoxel(m3World* world, m3ShapeId shape, int32_t x, int32_t y,
                                      int32_t z, uint16_t payload);
    M3_API void m3VoxelChunk_ClearVoxel(m3World* world, m3ShapeId shape, int32_t x, int32_t y,
                                        int32_t z);
    /// Clears the inclusive box [lo, hi] per axis. Returns the
    /// number of voxels that were actually cleared (zero is legal),
    /// or -1 on a stale id, a non-voxel shape, a bad region, or a
    /// surface past the box bound.
    M3_API int32_t m3VoxelChunk_ClearBox(m3World* world, m3ShapeId shape, const int32_t lo[3],
                                         const int32_t hi[3]);
    /// Fill fraction: 255 is a whole voxel, 1 a sliver. A
    /// mass and destruction property, never geometry: the voxel
    /// still collides as a full box, but its fragment mass scales
    /// by fill / 255. Refuses zero fill (that is a clear: use
    /// m3VoxelChunk_ClearVoxel) and empty voxels. Journaled state,
    /// hashed, inside the rollback delta like every edit.
    M3_API void m3VoxelChunk_SetFill(m3World* world, m3ShapeId shape, int32_t x, int32_t y,
                                     int32_t z, uint8_t fill);

    /// True while the id names a live shape of this world; false for
    /// the null id and stale generations, without counting misuse.
    M3_API bool m3Shape_IsValid(const m3World* world, m3ShapeId shape);

    /// The owning body, so a mover can tell its own shape from the
    /// world's. Null id for stale shapes.
    M3_API m3BodyId m3Shape_GetBody(const m3World* world, m3ShapeId shape);
    /// Readback, as in Maul2D. GetAabb is the shape's broadphase bounds
    /// (margin included). Thread class: reader.
    M3_API m3ShapeType m3Shape_GetType(const m3World* world, m3ShapeId shape);
    M3_API uint64_t m3Shape_GetUserData(const m3World* world, m3ShapeId shape);
    /// Journaled. Thread class: writer.
    M3_API void m3Shape_SetUserData(m3World* world, m3ShapeId shape, uint64_t userData);
    M3_API bool m3Shape_IsSensor(const m3World* world, m3ShapeId shape);
    M3_API m3AabbResult m3Shape_GetAabb(const m3World* world, m3ShapeId shape);
    /// Who touches this shape now: fills up to capacity
    /// entries and returns the count written. See m3ContactData.
    M3_API int32_t m3Shape_GetContactData(const m3World* world, m3ShapeId shape, m3ContactData* out,
                                          int32_t capacity);

    /// Destroy one shape and rebuild the owner's mass books.
    /// Journaled; contacts involving the shape dissolve at the next
    /// step. The last shape leaves a shapeless dynamic body at unit
    /// mass.
    M3_NODISCARD M3_API m3Result m3DestroyShape(m3World* world, m3ShapeId shape);

    /// Runtime material setters, with the def's bounds. Journaled;
    /// contacts read materials at prepare, so changes bind from the
    /// next step. A sleeping stack keeps its old mix until something
    /// wakes it.
    M3_API void m3Shape_SetFriction(m3World* world, m3ShapeId shape, float friction);
    M3_API float m3Shape_GetFriction(const m3World* world, m3ShapeId shape);
    M3_API void m3Shape_SetRestitution(m3World* world, m3ShapeId shape, float restitution);
    M3_API float m3Shape_GetRestitution(const m3World* world, m3ShapeId shape);
    M3_API void m3Shape_SetRollingResistance(m3World* world, m3ShapeId shape, float value);
    M3_API float m3Shape_GetRollingResistance(const m3World* world, m3ShapeId shape);

    /// Set the density and rebuild the owning body's mass, center and
    /// inertia from all its shapes: a body's mass is always its shapes'.
    /// Journaled. The density must be positive and at most 1e6, and on
    /// a dynamic body keep the mass floors and limit, as at creation.
    M3_API void m3Shape_SetDensity(m3World* world, m3ShapeId shape, float density);
    M3_API float m3Shape_GetDensity(const m3World* world, m3ShapeId shape);

    /// Opt a shape into hit events / the pre-solve veto.
    /// Journaled; the flags are state and snapshot with the world.
    M3_API void m3Shape_EnableHitEvents(m3World* world, m3ShapeId shape, bool flag);
    M3_API bool m3Shape_IsHitEventsEnabled(const m3World* world, m3ShapeId shape);
    M3_API void m3Shape_EnablePreSolve(m3World* world, m3ShapeId shape, bool flag);
    M3_API bool m3Shape_IsPreSolveEnabled(const m3World* world, m3ShapeId shape);

    /// The collision filter, changed at runtime: shapes it touched wake
    /// and every pair is filtered again on the next step. Journaled.
    /// GetFilter's out pointers may be NULL.
    M3_API void m3Shape_SetFilter(m3World* world, m3ShapeId shape, uint64_t categoryBits,
                                  uint64_t maskBits, int32_t groupIndex);
    M3_API void m3Shape_GetFilter(const m3World* world, m3ShapeId shape, uint64_t* categoryBitsOut,
                                  uint64_t* maskBitsOut, int32_t* groupIndexOut);

    /// Conveyor: a world-frame surface velocity on the
    /// shape. Contacts drive B's tangential speed relative to A
    /// toward the difference of the two surfaces, within 1e5 m/s on
    /// every axis. Journaled; state, hashed when nonzero.
    M3_API void m3Shape_SetSurfaceVelocity(m3World* world, m3ShapeId shape, m3Vec3 velocity);

    static const m3ShapeId m3_nullShapeId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_SHAPE_H
