// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world: creation, stepping, tuning, snapshots, the command
// journal, queries, explosions, counters and diagnostics.

#ifndef MAUL3D_WORLD_H
#define MAUL3D_WORLD_H

#include "maul3d/core_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// A world: everything the simulation holds, owned by the program
    /// through this pointer from m3CreateWorld to m3DestroyWorld. Worlds
    /// share nothing, so separate worlds may live on separate threads.
    ///
    /// A call that computes runs in the library's own floating-point
    /// setting, whatever the calling thread's is: it rounds to nearest,
    /// keeps subnormals and masks every exception, and gives the thread
    /// back its rounding mode and flush-to-zero and denormals-are-zero
    /// bits as they were. A host may run its threads in any setting and
    /// still gets the same bits. The host code a world runs (the
    /// allocator, the task hooks, the debug draw callbacks) runs in the
    /// thread's own setting, a task range sets the library's on whatever
    /// thread the host runs it, and the pre-solve callback runs in the
    /// library's, since its answer decides contacts.
    typedef struct m3World m3World;

    /// The task interface: the HOST owns the threads, the
    /// library only describes work. enqueueTask receives a task
    /// function, the item count, and a minimum grain; the host splits
    /// [0, itemCount) into subranges, runs each range exactly once on
    /// any thread in any order, and returns an opaque handle that
    /// finishTask must block on. Determinism holds for ANY split and
    /// ANY schedule because parallel phases only ever write disjoint
    /// slots; the worker-twin gate proves it on every commit. Leave
    /// both hooks null for serial execution.
    typedef void m3TaskFn(int32_t startIndex, int32_t endIndex, void* taskContext);
    typedef void* m3EnqueueTaskFn(m3TaskFn* task, int32_t itemCount, int32_t minRange,
                                  void* taskContext, void* userContext);
    typedef void m3FinishTaskFn(void* userTask, void* userContext);

    /// Query-side filter: a query behaves like a shape with
    /// these bits. A shape is visible to the query when the
    /// query's category intersects the shape's mask AND the
    /// shape's category intersects the query's mask. The default
    /// sees everything.
    typedef struct m3QueryFilter
    {
        M3_ALIGN8 uint64_t categoryBits;
        M3_ALIGN8 uint64_t maskBits;
    } m3QueryFilter;

    M3_API m3QueryFilter m3DefaultQueryFilter(void);

    /// World definition. Build with m3DefaultWorldDef so the cookie and
    /// the ABI version are valid; a zeroed or hand-rolled def is refused.
    typedef struct m3WorldDef
    {
        // The def's cookie and the ABI version stay the first two fields in
        // every version, so that m3CreateWorld can refuse a def of another
        // layout before it reads anything else.
        int32_t internalValue;
        // M3_ABI_VERSION of the headers the program was built with.
        uint32_t version;
        m3Vec3 gravity; // m/s^2, within 1e4 on every axis
        int32_t bodyCapacity;
        int32_t shapeCapacity;
        int32_t meshCapacity; // static triangle-mesh slots (24 KB each)
        int32_t jointCapacity;
        int32_t voxelCapacity;        // voxel chunk slots
        int32_t characterCapacity;    // character controllers
        int32_t vehicleCapacity;      // raycast vehicles
        int32_t softBodyCapacity;     // XPBD lattices
        m3EnqueueTaskFn* enqueueTask; // both null = serial (the default)
        m3FinishTaskFn* finishTask;
        void* userTaskContext;
        // Tuning knobs. All journaled-settable at runtime too;
        // they are engine STATE: snapshots carry them and replays
        // reproduce them.
        float contactHertz;         // contact softness frequency (30), up to 1e5
        float contactDampingRatio;  // contact softness damping (10), up to 1e4
        float contactPushMaxSpeed;  // max depenetration speed (3), up to 1e5
        float restitutionThreshold; // impact speed under it: no bounce (1)
        float maximumLinearSpeed;   // hard velocity cap (400), up to 1e5
        float maximumAngularSpeed;  // spin cap, rad/s, unless a body opts out; up to 1e5
        bool enableSleeping;        // islands may sleep (default true)
        bool enableContinuous;      // the continuous phase runs (default true)
        float hitEventThreshold;    // hit events need approach speed
                                    // above this (default 1)
        // The allocator the world keeps for its lifetime; zeroed means the
        // C library's.
        m3Allocator allocator;
    } m3WorldDef;

    /// The library's world defaults: gravity (0, -10, 0), 1024 bodies,
    /// 2048 shapes, 64 joints, 4 meshes, 4 voxel chunks, 4 characters, 2
    /// vehicles, no task hooks, the C library's allocator, and the def
    /// cookie, with version left 0. Call m3DefaultWorldDef instead, which
    /// stamps the version of the program's headers.
    M3_API m3WorldDef m3DefaultWorldDefValues(void);

    /// The world defaults, stamped with M3_ABI_VERSION of the headers the
    /// program is built with, so m3CreateWorld can refuse a def made for
    /// another release's binary interface.
    static inline m3WorldDef m3DefaultWorldDef(void)
    {
        m3WorldDef def = m3DefaultWorldDefValues();
        def.version = M3_ABI_VERSION;
        return def;
    }

    /// Creates a world. The checks run in order: the CPU must run the
    /// library's SIMD backend (m3_errorUnsupported), the def's cookie must
    /// be valid (m3_errorInvalid), its version must be M3_ABI_VERSION
    /// (m3_errorVersion), and the rest of the def must hold
    /// (m3_errorInvalid). A def whose world could snapshot to more than
    /// M3_MAX_SNAPSHOT_BYTES, with every mesh and hull slot's content at
    /// its largest, is refused with m3_errorCapacity before anything is
    /// allocated; the allocator may refuse too (m3_errorCapacity).
    /// *worldOut is null on any failure.
    M3_NODISCARD M3_API m3Result m3CreateWorld(const m3WorldDef* def, m3World** worldOut);
    /// Destroys a world and everything it owns; ids it issued mean nothing
    /// after. Null is a no-op.
    M3_API void m3DestroyWorld(m3World* world);

    /// Set the gravity vector, within 1e4 m/s^2 on every axis.
    /// Journaled; sleeping islands stay asleep until disturbed.
    M3_API void m3World_SetGravity(m3World* world, m3Vec3 gravity);

    /// Rebuild the broadphase tree top-down into a balanced shape
    ///. Bulk shape creation grows the tree one insertion at
    /// a time and can leave it lopsided; call this once after
    /// building a level and queries walk a balanced tree instead.
    /// Deterministic and journaled: the rebuilt shape is a pure
    /// function of the live shapes, so twins and replays agree.
    /// Returns m3_errorCapacity, with the tree unchanged and nothing
    /// recorded, when the world's allocator refuses the scratch.
    M3_NODISCARD M3_API m3Result m3World_RebuildBroadphase(m3World* world);
    /// A water volume: a world-anchored axis-aligned box of
    /// still or flowing water. The surface is the box top; there is
    /// no fluid simulation BY LAW. Rigid bodies inside get buoyancy
    /// proportional to their submerged bounds, drag toward the flow
    /// velocity, and torque about their submerged centroid; the
    /// model is deliberately approximate and deterministically so.
    typedef struct m3WaterVolumeDef
    {
        m3Pos3 lo;             // within 1e15 m of the origin
        m3Pos3 hi;             // must exceed lo on every axis; within 1e15 m
        m3real density;        // kg/m3; 1000 is water, > 0 and at most 1e6
        m3real linearDrag;     // per-second rate toward the flow, >= 0
        m3real angularDrag;    // per-second spin damping, >= 0
        m3Vec3 flow;           // current velocity, world frame, within 1e5 m/s
        int32_t internalValue; // from m3DefaultWaterVolumeDef
    } m3WaterVolumeDef;

    typedef struct m3WaterVolumeId
    {
        uint32_t index1;
        uint32_t generation;
    } m3WaterVolumeId;

    static const m3WaterVolumeId m3_nullWaterVolumeId = {0, 0};

    M3_API m3WaterVolumeDef m3DefaultWaterVolumeDef(void);
    /// Up to 8 volumes per world; journaled, snapshotted, hashed
    /// off-default. Creating or destroying a volume wakes every
    /// body whose bounds touch it (the tide moves things).
    M3_NODISCARD M3_API m3Result m3CreateWaterVolume(m3World* world, const m3WaterVolumeDef* def,
                                                     m3WaterVolumeId* volumeOut);
    M3_NODISCARD M3_API m3Result m3DestroyWaterVolume(m3World* world, m3WaterVolumeId volume);
    M3_API bool m3WaterVolume_IsValid(const m3World* world, m3WaterVolumeId volume);

    /// Current gravity.
    M3_API m3Vec3 m3World_GetGravity(const m3World* world);

    /// Contact softness tuning: frequency (hertz, up to 1e5), damping
    /// ratio (up to 1e4) and the maximum depenetration speed (up to 1e5
    /// m/s), each positive. Journaled. Static contacts
    /// use twice the frequency: a soft row against something
    /// immovable stores energy under a tall stack.
    M3_API void m3World_SetContactTuning(m3World* world, float hertz, float dampingRatio,
                                         float pushMaxSpeed);

    /// Impact speed below the threshold does not bounce. Journaled.
    M3_API void m3World_SetRestitutionThreshold(m3World* world, float value);

    /// Hard cap on any body's linear speed, applied every substep:
    /// positive and at most 1e5 m/s, the speed the step holds every
    /// body within. Journaled.
    M3_API void m3World_SetMaximumLinearSpeed(m3World* world, float value);

    /// Hard cap on any body's angular speed in rad/s, applied every
    /// substep. The default (800) is a catastrophe guard, not a
    /// gameplay clamp; bodies flagged with
    /// m3Body_EnableFastRotation bypass it. Positive and at most 1e5
    /// rad/s; the step holds every body, flagged or not, within 1e5
    /// rad/s. Journaled.
    M3_API void m3World_SetMaximumAngularSpeed(m3World* world, float value);

    /// Turn island sleeping on or off. Turning it OFF wakes every
    /// sleeping body. Journaled.
    M3_API void m3World_EnableSleeping(m3World* world, bool flag);

    /// Whether island sleeping is enabled.
    M3_API bool m3World_IsSleepingEnabled(const m3World* world);

    /// Turn the continuous phase (CCD) on or off. Journaled.
    M3_API void m3World_EnableContinuous(m3World* world, bool flag);

    /// Whether the continuous phase is enabled.
    M3_API bool m3World_IsContinuousEnabled(const m3World* world);

    /// Wind: a deterministic field applied to soft-body
    /// particles as proportional drag toward the wind velocity.
    /// The gust is a sine of an ACCUMULATED phase (state, in the
    /// snapshot), so rollbacks resume the exact same wave: no host
    /// randomness exists anywhere in it. speed zero disables.
    /// direction must be near-unit when speed is nonzero; speed is
    /// within 1e5 m/s, gustHertz within 1e5 and gustScale within 1e4,
    /// each non-negative, on the call and on replay alike. Rigid
    /// bodies are not wind-blown in v1 (documented: the force API
    /// already serves them); journaled.
    M3_API void m3World_SetWind(m3World* world, m3Vec3 direction, float speed, float gustHertz,
                                float gustScale);

    /// Advance the simulation: collide, then the Soft Step solver
    /// with the given substep count (1..256) over dt, 1e-4 to 10
    /// seconds; out of range refuses loudly. Deterministic: same
    /// inputs, same bits, on every platform and backend. Journaled.
    /// THE THREADING CONTRACT. Per world:
    /// one writer at a time; Step and every mutating call demand
    /// exclusive access to that world. Between steps, any number of
    /// concurrent READERS (queries, casts, getters, snapshot) may
    /// run on the same world from any threads. DISTINCT worlds are
    /// fully independent: stepping two worlds on two host threads
    /// is supported and bit-identical to stepping them serially
    /// (the concurrency suite proves it; the TSAN cell polices it).
    /// The library keeps no process-wide state, so worlds may be
    /// created and destroyed on any threads at once. Reading a world
    /// WHILE it steps is undefined.
    M3_API void m3World_Step(m3World* world, float dt, int32_t substeps);

    /// Snapshot and rollback, first-class from day one. The format is
    /// portable and versioned: field blocks in little-endian order, laid
    /// out the same on every target, after a header carrying the state
    /// prefix (the writing release's M3_ABI_VERSION and the format's
    /// number), a config hash (solver revision, precision, FP policy)
    /// and the world's capacities. Only the release that wrote a
    /// snapshot reads it: restore refuses another release, format or
    /// config with m3_errorVersion and another world shape with
    /// m3_errorInvalid, restores in place, and the restored world
    /// resimulates bit-exactly. Before any byte lands, every block is
    /// checked: out-of-range indices and counts, flags that are not
    /// flags, and non-finite state refuse with m3_errorInvalid and leave
    /// the world as it was. Then the structures are checked as wholes:
    /// restore refuses, the same way, any snapshot whose links, lists,
    /// pools, trees or per-record values this engine could not have
    /// written (id pools, body to shape and joint lists, the broadphase
    /// tree, the contact pairs, shared content and its reference counts,
    /// and the records of characters, vehicles, soft bodies and joints),
    /// and any value past the limit its create or setter keeps (the
    /// guide's "Magnitude limits"), positions included: creates place
    /// every body and soft particle within 1e15 m and the step holds it
    /// there. Replay and restore take a shape's geometry only in the form
    /// its create leaves it. Mass, center, inertia and
    /// extents are not stored; restore rebuilds them from each body's
    /// type and shapes. A world restore accepts steps, queries and
    /// snapshots like one the API built.
    /// Snapshot writes m3World_SnapshotSize bytes, never more than
    /// M3_MAX_SNAPSHOT_BYTES, and sets *bytesOut; a smaller buffer
    /// returns m3_errorCapacity.
    M3_API int32_t m3World_SnapshotSize(const m3World* world);
    M3_NODISCARD M3_API m3Result m3World_Snapshot(const m3World* world, void* buffer,
                                                  int32_t capacity, int32_t* bytesOut);
    M3_NODISCARD M3_API m3Result m3World_Restore(m3World* world, const void* data, int32_t size);

    /// Contact events: begin fires on the step two shapes
    /// first touch, end fires on the step they separate. Derived from
    /// the canonical pair walk, so replay produces the identical
    /// stream. Pointers are valid until the next step or restore.
    /// Destroying a shape emits no end event (the id would be stale);
    /// events are transient observers and never snapshot state.
    typedef struct m3ContactBeginEvent
    {
        m3ShapeId shapeIdA; // the lower shape index of the pair
        m3ShapeId shapeIdB;
    } m3ContactBeginEvent;

    typedef struct m3ContactEndEvent
    {
        m3ShapeId shapeIdA; // ids as they were when the touch ended
        m3ShapeId shapeIdB;
    } m3ContactEndEvent;

    /// A hit event: two shapes collided with approach speed above the
    /// world threshold. Emitted at most once per contact per step, for
    /// the fastest-approaching manifold point, and only when either
    /// shape opted in (m3Shape_EnableHitEvents).
    typedef struct m3ContactHitEvent
    {
        m3ShapeId shapeIdA;
        m3ShapeId shapeIdB;
        m3Pos3 point;         // approximate contact point, world space
        m3Vec3 normal;        // from shape A to shape B
        m3real approachSpeed; // relative normal speed at impact, > 0
    } m3ContactHitEvent;

    /// The contact streams of the LAST step, in canonical deterministic
    /// order. Arrays are world-owned and valid until the next step,
    /// restore, or world destruction (restore clears them: events are
    /// observations, not state). Hits beyond capacity still simulate;
    /// only their events drop, and hitsDropped says how many.
    typedef struct m3ContactEvents
    {
        const m3ContactBeginEvent* beginEvents;
        const m3ContactEndEvent* endEvents;
        const m3ContactHitEvent* hitEvents;
        int32_t beginCount;
        int32_t endCount;
        int32_t hitCount;
        int32_t hitsDropped;
    } m3ContactEvents;

    M3_API m3ContactEvents m3World_GetContactEvents(const m3World* world);

    /// Sensor overlap events, the same law as contact events but in
    /// their own streams (a sensor touch is not a contact). shapeIdA
    /// is the lower shape index; either side may be the sensor.
    typedef struct m3SensorEvents
    {
        const m3ContactBeginEvent* beginEvents;
        const m3ContactEndEvent* endEvents;
        int32_t beginCount;
        int32_t endCount;
    } m3SensorEvents;

    M3_API m3SensorEvents m3World_GetSensorEvents(const m3World* world);

    /// A fragment-spawn event: a voxel edit disconnected an
    /// island from its chunk's base layer (y = 0 voxels anchor a
    /// chunk; an island with no path to the base cannot stand). The
    /// engine REMOVES the island from the grid as part of the edit's
    /// state transition and emits this event; it never spawns bodies
    /// (what a fragment becomes is the host's decision; the
    /// usual recipe feeds the island's voxels to the built-in
    /// QuickHull and creates a hull body). Events and recipes are
    /// valid until the next step, restore, or edit-free frame
    /// boundary of your choosing; the next m3World_Step clears them.
    typedef struct m3FragmentEvent
    {
        m3ShapeId chunkShapeId;
        int32_t voxelCount;
        /// Range into m3FragmentEvents.recipe: chunk-local linear
        /// voxel indices (v = x + 16 * (y + 16 * z)). recipeStart is
        /// -1 when the recipe buffer overflowed (the event and the
        /// removal still happened; the bounds below still describe
        /// the island).
        int32_t recipeStart;
        int32_t recipeCount;
        m3Vec3 comChunk;     /// island center of mass, chunk frame
        m3Pos3 comWorld;     /// the same point in world space at emit time
        m3real mass;         /// at density one (cell volume times count)
        uint8_t boundsLo[3]; /// island voxel bounds, inclusive
        uint8_t boundsHi[3];
        uint8_t pad[2];
    } m3FragmentEvent;

    /// The fragment events of the last step and their recipe buffer.
    /// Islands beyond the event capacity are still removed from the
    /// grid (state transitions stay pure); only their events drop, and
    /// fragmentsDropped says how many.
    typedef struct m3FragmentEvents
    {
        const m3FragmentEvent* fragmentEvents;
        const uint16_t* recipe;
        int32_t fragmentCount;
        int32_t recipeCount;
        int32_t fragmentsDropped;
    } m3FragmentEvents;

    M3_API m3FragmentEvents m3World_GetFragmentEvents(const m3World* world);

    /// Hit events require approach speed above this. Journaled.
    M3_API void m3World_SetHitEventThreshold(m3World* world, float value);

    /// A body move event: one per body that MOVED this step
    /// (every mover), in ascending body order, carrying the post-step
    /// transform. fellAsleep marks the step a body drops off; a
    /// sleeping body emits nothing until it wakes. Render sync reads
    /// this instead of polling every body.
    typedef struct m3BodyMoveEvent
    {
        m3BodyId bodyId;
        m3Transform transform;
        bool fellAsleep;
    } m3BodyMoveEvent;

    typedef struct m3BodyEvents
    {
        const m3BodyMoveEvent* moveEvents;
        int32_t moveCount;
    } m3BodyEvents;

    M3_API m3BodyEvents m3World_GetBodyEvents(const m3World* world);

    /// A joint break event. The id is already stale when the
    /// event is read: the joint destroyed itself. Use it as a key,
    /// not a handle.
    typedef struct m3JointBreakEvent
    {
        m3JointId jointId;
    } m3JointBreakEvent;

    typedef struct m3JointEvents
    {
        const m3JointBreakEvent* breakEvents;
        int32_t breakCount;
    } m3JointEvents;

    M3_API m3JointEvents m3World_GetJointEvents(const m3World* world);

    /// Pre-solve veto: called during contact preparation, in
    /// canonical pair order, for pairs where either shape opted in
    /// (m3Shape_EnablePreSolve). Return false to disable the contact
    /// for THIS step (the one-way platform tool). point is the
    /// deepest contact point, normal points from shape A to shape B.
    ///
    /// THE DETERMINISM CONTRACT, LOUD: the journal records inputs,
    /// never host whims. The callback MUST be a pure function of its
    /// arguments (and of host state that is itself bit-identical on
    /// replay). Register the SAME pure callback before replaying a
    /// journal or resimulating from a snapshot; a callback that
    /// consults a clock, a random source, or mutable host state
    /// breaks bit-exactness and that breakage is YOURS. The engine
    /// calls it from the serial prepare pass, never from workers.
    typedef bool m3PreSolveFn(m3ShapeId shapeA, m3ShapeId shapeB, m3Pos3 point, m3Vec3 normal,
                              void* context);

    /// Register (or clear with NULL) the pre-solve callback. The
    /// REGISTRATION is host wiring (never journaled, never snapshot
    /// state), but the DECISIONS are not: every veto a step makes
    /// is journaled as that step's annex, so a bare replay
    /// (m3replay verify included) applies the recorded vetoes with
    /// no callback installed and lands on the recorded bits. During
    /// such a step the recorded set wins and any installed callback
    /// stays silent; do not expect callbacks to fire under replay.
    M3_API void m3World_SetPreSolveCallback(m3World* world, m3PreSolveFn* fn, void* context);

    /// The closest hit of a ray or shape cast: origin in world doubles,
    /// translation = direction times reach, fraction in [0, 1] along
    /// the translation. Ties break to the lower shape index. A zero
    /// translation misses.
    typedef struct m3RayCastResult
    {
        m3ShapeId shapeId; // null when hit is false
        m3Pos3 point;
        m3Vec3 normal;
        m3real fraction;
        bool hit;
    } m3RayCastResult;

    /// One entry of m3World_CastRayAll.
    typedef struct m3RayHit
    {
        m3ShapeId shapeId;
        m3Pos3 point;
        m3Vec3 normal;
        m3real fraction;
    } m3RayHit;

    /// The mover toolkit: pure queries and a pure plane
    /// solver for hosts that roll their own character movers (the
    /// engine's kinematic controller remains the built-in path).
    /// A mover is a vertical capsule: center, half height between
    /// the cap centers, radius.
    typedef struct m3MoverPlane
    {
        m3ShapeId shapeId;
        m3Vec3 normal;     // from the shape toward the mover
        m3real separation; // gap along the normal; negative = overlap
    } m3MoverPlane;

/// Planes past this count take no part in the solve.
#define M3_MOVER_PLANES 16

    typedef struct m3MoverMove
    {
        m3Vec3 translation;
        uint32_t pressed; // bit i: plane i stops part of the wish
    } m3MoverMove;

    /// Cast the mover capsule along a translation; the closest
    /// blocking hit (sensors are invisible to movers).
    M3_API m3RayCastResult m3World_CastMover(const m3World* world, m3Pos3 center, m3real halfHeight,
                                             m3real radius, m3Vec3 translation);

    /// Collect contact planes for the mover at rest: every shape
    /// within `skin` of the capsule contributes one plane. Returns
    /// the count written (ascending shape order, deterministic).
    M3_API int32_t m3World_CollideMover(const m3World* world, m3Pos3 center, m3real halfHeight,
                                        m3real radius, m3real skin, m3MoverPlane* planes,
                                        int32_t capacity);

    /// The closest translation to the wish that no plane blocks, and
    /// the planes it rests on. Pure function, world-free. Only the
    /// first M3_MOVER_PLANES planes take part.
    M3_API m3MoverMove m3SolveMover(m3Vec3 wish, const m3MoverPlane* planes, int32_t count);

    /// The velocity closest to the given one that no pressed plane
    /// opposes: what is left after hitting them. Pure function.
    M3_API m3Vec3 m3ClipMoverVelocity(m3Vec3 velocity, const m3MoverPlane* planes, int32_t count,
                                      uint32_t pressed);

    /// Every query takes an m3QueryFilter and sees the shapes a shape
    /// with those bits would touch; m3DefaultQueryFilter sees them
    /// all. The closest front-face hit along origin + t * translation
    /// for t in [0, 1]: rays MISS shapes they start inside or exactly
    /// on (front faces only); ask m3World_TestPoint for containment.
    M3_API m3RayCastResult m3World_CastRayClosest(const m3World* world, m3Pos3 origin,
                                                  m3Vec3 translation, m3QueryFilter filter);

    /// Every ray hit along the translation, one entry point per
    /// shape, sorted by fraction (ties to the lower shape index).
    /// Returns the count written (never more than capacity; excess
    /// hits are dropped from the FAR end, deterministically).
    M3_API int32_t m3World_CastRayAll(const m3World* world, m3Pos3 origin, m3Vec3 translation,
                                      m3RayHit* hits, int32_t capacity, m3QueryFilter filter);

    /// Closest-hit shape casts: sweep a sphere or a capsule along a
    /// translation. fraction is the earliest touch in [0, 1]; a cast
    /// that STARTS overlapped hits at fraction zero with a zero
    /// normal (the documented start-inside contract; rays instead
    /// MISS shapes they start inside, front faces only).
    /// Generic convex casts: a box (with orientation) or a
    /// caller point cloud (2..24 points, base-relative) swept along
    /// a translation, skinless. Same contracts as every cast: the
    /// earliest touch in [0, 1], start-overlapped reports fraction
    /// zero with a zero normal, hostile inputs miss. A single point
    /// is a ray: use the ray casts.
    M3_API m3RayCastResult m3World_CastBoxClosest(const m3World* world, m3Pos3 center,
                                                  m3Vec3 halfExtents, m3Quat rotation,
                                                  m3Vec3 translation, m3QueryFilter filter);
    M3_API m3RayCastResult m3World_CastHullClosest(const m3World* world, m3Pos3 base,
                                                   const m3Vec3* points, int32_t count,
                                                   m3Vec3 translation, m3QueryFilter filter);
    M3_API m3RayCastResult m3World_CastSphereClosest(const m3World* world, m3Pos3 center,
                                                     m3real radius, m3Vec3 translation,
                                                     m3QueryFilter filter);
    M3_API m3RayCastResult m3World_CastCapsuleClosest(const m3World* world, m3Pos3 center,
                                                      m3Vec3 point1, m3Vec3 point2, m3real radius,
                                                      m3Vec3 translation, m3QueryFilter filter);

    /// The first shape (lowest index) whose volume contains the
    /// point, or the null id. Meshes are open surfaces and never
    /// contain points; planes are solid half spaces.
    M3_API m3ShapeId m3World_TestPoint(const m3World* world, m3Pos3 point);

    /// Shapes whose tight bounds overlap the box, in ascending shape
    /// index order. Returns the count written.
    M3_API int32_t m3World_OverlapAabb(const m3World* world, m3Pos3 lo, m3Pos3 hi,
                                       m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter);

    /// Shapes within reach of the sphere (exact per family), in
    /// ascending shape index order. Returns the count written.
    M3_API int32_t m3World_OverlapSphere(const m3World* world, m3Pos3 center, m3real radius,
                                         m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter);

    /// The exact overlap family: capsule, oriented box, and
    /// raw convex cloud queries with the same contract as the
    /// sphere: shapes within EXACT reach (GJK per candidate, planes
    /// and voxel/mesh handled per family), ascending shape index,
    /// filtered, read-only. Clouds carry base-relative points (the
    /// cast convention), at most 64 of them.
    M3_API int32_t m3World_OverlapCapsule(const m3World* world, m3Pos3 p1, m3Pos3 p2, m3real radius,
                                          m3ShapeId* shapes, int32_t capacity,
                                          m3QueryFilter filter);
    M3_API int32_t m3World_OverlapBox(const m3World* world, m3Pos3 center, m3Vec3 halfExtents,
                                      m3Quat rotation, m3ShapeId* shapes, int32_t capacity,
                                      m3QueryFilter filter);
    M3_API int32_t m3World_OverlapHullPoints(const m3World* world, m3Pos3 base,
                                             const m3Vec3* points, int32_t count, m3real radius,
                                             m3ShapeId* shapes, int32_t capacity,
                                             m3QueryFilter filter);

    /// Explosion definition: one journaled call pushes every
    /// dynamic convex shape in range. The impulse scales with the
    /// area the shape shows to the blast,
    /// fades linearly to zero across the falloff band past the
    /// radius, wakes everything it reaches, and may be negative for
    /// an implosion. Spheres, capsules, and hulls respond; meshes
    /// and planes are static; voxel chunks couple through charges.
    typedef struct m3ExplosionDef
    {
        m3Pos3 position;      // world-space center, within 1e15 m
        m3QueryFilter filter; // which shapes the blast may touch
        float radius;         // full-impulse radius, >= 0
        float falloff;        // fade band past the radius, >= 0
        float impulsePerArea; // impulse per m^2 of facing area, within 1e10
        float voxelCarve;     // carve radius into voxel chunks, 0 = off
        float softPush;       // soft particle push scale, default 1, within 1e4
        int32_t internalValue;
    } m3ExplosionDef;

    /// Returns a def with pinned defaults (10 m radius, 5 m falloff,
    /// zero impulse, no carve, soft push 1, open filter) and a valid
    /// cookie.
    M3_API m3ExplosionDef m3DefaultExplosionDef(void);

    /// Apply the explosion now. Journaled; hostile defs apply
    /// nothing and journal nothing. Every push leaves its body within
    /// 1e5 m/s and rad/s.
    M3_API void m3World_Explode(m3World* world, const m3ExplosionDef* def);

    /// FNV-1a 64 over the deterministic state in canonical slot order:
    /// the step count, gravity and any tuning off its default, every id
    /// pool's identity (liveness, generations, free queue), and the
    /// bodies, shapes (the height field each one reads, voxel grids and
    /// mesh paint included), characters, soft bodies, vehicles, joints
    /// (limits, motors and collide flags included), water volumes and
    /// persistent contacts. Mesh geometry and height field samples are
    /// left out, since nothing changes them after create, and so are
    /// derived data and observers (trees, events, counters, the
    /// profile). The value every gate compares.
    M3_API uint64_t m3World_Hash(const m3World* world);

    /// Every live body or joint, up to capacity ids in ascending slot
    /// order; returns the total. The step count is the number of steps
    /// taken (snapshot state). Thread class: reader.
    M3_API int32_t m3World_GetBodies(const m3World* world, m3BodyId* ids, int32_t capacity);
    M3_API int32_t m3World_GetJoints(const m3World* world, m3JointId* ids, int32_t capacity);
    M3_API uint64_t m3World_GetStepCount(const m3World* world);

    /// Live contact readback: one entry per manifold touching
    /// the queried body or shape, in canonical pair order. Pure
    /// observer data; points are world space at read time.
    typedef struct m3ContactData
    {
        m3ShapeId shapeIdA;
        m3ShapeId shapeIdB;
        m3Vec3 normal; // from shape A toward shape B
        int32_t pointCount;
        m3Pos3 points[4];
        float separations[4];    // negative = penetration
        float normalImpulses[4]; // last step's warm-start payload
    } m3ContactData;

    /// Live-world counters: pure observer data. Reading them
    /// never touches the simulation, the snapshot, or the hash. The
    /// count fields are themselves deterministic (twins report
    /// identical counts); islandCount, colorCount, and scratchPeak
    /// describe the last completed step and are zero before it.
    typedef struct m3Counters
    {
        int32_t bodyCount;
        int32_t shapeCount;
        int32_t jointCount;
        int32_t contactCount; // live broadphase pairs with manifolds
        int32_t awakeCount;   // awake dynamic bodies
        int32_t islandCount;  // dynamic islands in the last step
        int32_t colorCount;   // solver graph colors in the last step
        int32_t characterCount;
        int32_t vehicleCount;
        int32_t softBodyCount;
        int32_t voxelChunkCount;
        int32_t hullCount; // interned hull slabs
        int32_t meshCount; // interned mesh slabs
        int32_t treeHeight;
        int32_t scratchPeak;     // step scratch high water, bytes
        int32_t scratchCapacity; // step scratch reserve, bytes
        int32_t snapshotBytes;   // exact m3World_Snapshot size now
        int64_t allocCalls;      // process-global persistent allocs
        int64_t freeCalls;
        uint64_t misuse; // stale-id, wrong-kind or bad-argument refusals, cumulative
        // Voxel edits and explosion carves refused because the chunk's
        // surface would merge into more boxes than its tree holds,
        // cumulative; each changed nothing and journaled nothing. A
        // counter, not state: it is not hashed or snapshotted, and a
        // restore or a replay's back-out leaves it as it is, so a replay
        // refused after an edit was refused leaves it higher.
        uint64_t voxelEditsRefused;
        // Pushes the step lands on a body, a character's, a soft
        // particle's or a wheel's, whose float velocity or spin was not
        // finite and that were taken again in double, cumulative; each
        // still left the body within the speed limit. A counter, not
        // state: it is not hashed or snapshotted, and a restore or a
        // replay's back-out leaves it as it is.
        uint64_t heldPushRetakes;
    } m3Counters;

    M3_API m3Counters m3World_GetCounters(const m3World* world);

    /// Per-world memory accounting, computed
    /// on demand so it cannot drift: persistentBytes is the fixed
    /// array footprint decided at create (pools, tree, and every
    /// snapshot-walked array), contentBytes is the live
    /// count-derived payload (mesh vertices/indices/flags, mesh BVH
    /// nodes, heightfield samples, the surface tree each voxel slot
    /// takes with its first chunk), and the scratch numbers are the
    /// step arena's capacity and last high-water mark. The step
    /// itself allocates nothing else.
    typedef struct m3MemoryUsage
    {
        int64_t persistentBytes;
        int64_t contentBytes;
        int32_t scratchCapacity;
        int32_t scratchPeak;
    } m3MemoryUsage;
    M3_API m3MemoryUsage m3World_GetMemoryUsage(const m3World* world);

    /// Wall-clock milliseconds per phase of the last completed step,
    /// measured with a monotonic clock for observation only. Never
    /// deterministic, never hashed, never serialized; zero before
    /// the first step, and a step that stalls on scratch growth
    /// keeps the previous profile.
    typedef struct m3Profile
    {
        float step;        // the whole of m3World_Step
        float vehicles;    // suspension and drivetrain pass
        float broadphase;  // pair update
        float narrowphase; // manifold generation
        float events;      // contact/sensor event merge walk
        float prepare;     // sizing, sweep capture, island wake
        float solve;       // constraint preparation and substeps
        float continuous;  // CCD sweeps including voxel TOI
        float sleep;       // island sleep pass
        float characters;  // rider carry
        float softBodies;  // the soft body pass
    } m3Profile;

    M3_API m3Profile m3World_GetProfile(const m3World* world);

    /// Journal: every mutation of the world is a discrete recorded op,
    /// and replaying the stream through the same internal functions
    /// reproduces the world bit for bit. Start hands the world a
    /// caller-owned buffer (m3_errorState while a journal is already
    /// running); Stop sets *bytesOut to the bytes written, or returns
    /// m3_errorCapacity when the buffer overflowed (m3_errorState when
    /// none was running); Replay applies a stream to this world. A tape
    /// is laid out the same on every target (little-endian, each record
    /// as on x86-64) and never holds a def's padding, so the same calls
    /// write the same bytes anywhere. It starts with the 16-byte state
    /// prefix, written when the journal starts, so the buffer holds at
    /// least 16 bytes and a session of no op is those 16. Only the
    /// release that wrote a tape replays it.
    M3_NODISCARD M3_API m3Result m3World_StartJournal(m3World* world, void* buffer,
                                                      int32_t capacity);
    M3_NODISCARD M3_API m3Result m3World_StopJournal(m3World* world, int32_t* bytesOut);
    /// Replay a recorded session into this world. ATOMIC: on any
    /// refusal the world is restored to its pre-call state; a
    /// half-applied session is impossible. Returns m3_errorVersion for a
    /// tape another release wrote, m3_errorInvalid for a damaged tape
    /// (truncation, corruption, an op that cannot re-mint its recorded
    /// id) and m3_errorCapacity when the world's
    /// allocator refuses what the replay or one of its ops needs.
    /// The back-out is taken before the first op so that it cannot
    /// fail: for the whole call the replay holds a snapshot of the
    /// world (m3World_SnapshotSize bytes) and a second copy of its
    /// mesh and height field content and mesh trees (about
    /// contentBytes of m3World_GetMemoryUsage) and arrays sized by the
    /// mesh, shape and voxel capacities. A tape that fits a tight allocator without that
    /// headroom is refused with m3_errorCapacity before any op runs.
    M3_NODISCARD M3_API m3Result m3World_ReplayJournal(m3World* world, const void* data,
                                                       int32_t size);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_WORLD_H
