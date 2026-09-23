# Maul3D API reference

Generated from the public headers by `tools/gen_api.py`. The headers
are the source of truth; this file mirrors them.

## `base.h`

Base definitions shared by every Maul3D header: the version, the export macro, the result codes, the allocator and assert hooks, and hashing.

```c
int m3GetVersion(void);
```
The linked library's version as major * 10000 + minor * 100 + patch (0.3.0 returns 300). Compare against the M3_VERSION macros to catch a header/library mismatch at startup.

```c
void m3SetAllocator(m3AllocFn* allocFn, m3FreeFn* freeFn, void* context);
```

```c
uint64_t m3Hash64(uint64_t h, const void* bytes, int32_t count);
```

```c
void m3AssertFail(const char* condition, const char* file, int line);
```
The debug assert sink (prints and aborts). Internal invariants only; never called for user input, never present in release.

```c
void m3SetAssertHandlerCtx(m3AssertCtxFn* handler, void* context);
```

```c
void m3SetAssertHandler(m3AssertFn handler);
```

## `math.h`

Deterministic 3D math. World positions are 64-bit (m3Pos3) and everything local is 32-bit (m3real). Every operation is plain IEEE arithmetic under -ffp-contract=off, so the bits agree on every platform; the transcendentals are the engine's own.

```c
static inline m3Vec3 m3MulMV3(m3Mat3 m, m3Vec3 v);
```
Column-major matrix times vector: cx*v.x + cy*v.y + cz*v.z.

```c
static inline m3Mat3 m3MakeZeroMat3(void);
```

```c
static inline m3real m3MinF(m3real a, m3real b);
```
Pinned minimum: exactly (a < b ? a : b), in this operand order, on every platform. MSVC x64 lowers the ternary through MINSS which matches; see Maul2D's arm64 lesson for why this is spelled out rather than assumed.

```c
static inline m3real m3MaxF(m3real a, m3real b);
```
Pinned maximum: exactly (a > b ? a : b), in this operand order.

```c
static inline m3real m3ClampF(m3real a, m3real lo, m3real hi);
```

```c
static inline m3real m3AbsF(m3real a);
```

```c
static inline m3Vec3 m3Add3(m3Vec3 a, m3Vec3 b);
```

```c
static inline m3Vec3 m3Sub3(m3Vec3 a, m3Vec3 b);
```

```c
static inline m3Vec3 m3MulSV3(m3real s, m3Vec3 v);
```

```c
static inline m3Vec3 m3Neg3(m3Vec3 v);
```

```c
static inline m3real m3Dot3(m3Vec3 a, m3Vec3 b);
```

```c
static inline m3Vec3 m3Cross3(m3Vec3 a, m3Vec3 b);
```

```c
static inline m3real m3LengthSquared3(m3Vec3 v);
```

```c
static inline m3real m3Length3(m3Vec3 v);
```

```c
static inline m3Vec3 m3Normalize3(m3Vec3 v);
```
Normalize with a fixed degenerate fallback of {0, 1, 0}: one rule, documented, so a zero vector never turns into NaN and never depends on the caller.

```c
static inline m3real m3UnwindAngle(m3real radians);
```
Convert any angle into the range [-pi, pi]. remainderf is IEEE exact (like sqrt), so this is deterministic (reference note).

```c
static inline m3Quat m3MakeIdentityQuat(void);
```

```c
static inline m3Quat m3MulQuat(m3Quat a, m3Quat b);
```

```c
static inline m3Quat m3NormalizeQuat(m3Quat q);
```
Normalize with a fixed degenerate fallback of identity.

```c
static inline m3Vec3 m3RotateVec3(m3Quat q, m3Vec3 v);
```
Rotate a vector by a unit quaternion: v' = v + w*t + q x t with t = 2 (q x v). Plain mul and add only, no fused ops.

```c
static inline m3Vec3 m3InvRotateVec3(m3Quat q, m3Vec3 v);
```
Rotate by the conjugate (inverse for unit quaternions).

```c
static inline m3Quat m3IntegrateRotation(m3Quat q, m3Vec3 deltaRotation);
```
Integrate a rotation by an angular displacement (radians, world frame): q2 = normalize(q1 + 0.5 * (dr, 0) * q1), the reference quaternion-derivative form. Renormalization every call is the 3D numeric contract: drift never accumulates.

```c
m3CosSin m3ComputeCosSin(m3real radians);
```
Deterministic cosine and sine (Bhaskara I rational form) and atan2 (minimax polynomial): hand rolled because platform libm implementations disagree in the last bits. Reference technique.

```c
m3real m3Atan2(m3real y, m3real x);
```

## `world.h`

The world: creation, stepping, tuning, snapshots, the command journal, queries, explosions, counters and diagnostics.

```c
m3QueryFilter m3DefaultQueryFilter(void);
```

```c
m3WorldDef m3DefaultWorldDef(void);
```
The pinned world defaults: gravity (0, -10, 0), 1024 bodies, 2048 shapes, 64 joints, 4 meshes, 4 voxel chunks, 4 characters, 2 vehicles, one worker, no task hooks, and the def cookie every create demands.

```c
m3WorldId m3CreateWorld(const m3WorldDef* def);
```
Create a world. Returns the null id on an invalid def or an exhausted world table (diagnostic in debug builds).

```c
void m3DestroyWorld(m3WorldId worldId);
```

```c
bool m3World_IsValid(m3WorldId worldId);
```

```c
void m3World_SetGravity(m3WorldId worldId, m3Vec3 gravity);
```
Set the gravity vector. Journaled; sleeping islands stay asleep until disturbed (the reference behavior).

```c
void m3World_RebuildBroadphase(m3WorldId worldId);
```
Rebuild the broadphase tree top-down into a balanced shape (17-4). Bulk shape creation grows the tree one insertion at a time and can leave it lopsided; call this once after building a level and queries walk a balanced tree instead. Deterministic and journaled: the rebuilt shape is a pure function of the live shapes, so twins and replays agree.

```c
m3WaterVolumeDef m3DefaultWaterVolumeDef(void);
```

```c
m3WaterVolumeId m3CreateWaterVolume(m3WorldId worldId, const m3WaterVolumeDef* def);
```
Up to 8 volumes per world; journaled, snapshotted, hashed off-default. Creating or destroying a volume wakes every body whose bounds touch it (the tide moves things).

```c
void m3DestroyWaterVolume(m3WaterVolumeId id);
```

```c
bool m3WaterVolume_IsValid(m3WaterVolumeId id);
```

```c
m3Vec3 m3World_GetGravity(m3WorldId worldId);
```
Current gravity.

```c
void m3World_SetContactTuning(m3WorldId worldId, float hertz, float dampingRatio, float pushMaxSpeed);
```
Contact softness tuning: frequency (hertz), damping ratio and the maximum depenetration speed. Journaled. Static contacts use twice the frequency, like the reference.

```c
void m3World_SetRestitutionThreshold(m3WorldId worldId, float value);
```
Impact speed below the threshold does not bounce. Journaled.

```c
void m3World_SetMaximumLinearSpeed(m3WorldId worldId, float value);
```
Hard cap on any body's linear speed, applied every substep. Journaled.

```c
void m3World_SetMaximumAngularSpeed(m3WorldId worldId, float value);
```
Hard cap on any body's angular speed in rad/s, applied every substep. The default (800) is a catastrophe guard, not a gameplay clamp; bodies flagged with m3Body_SetAllowFastRotation bypass it. Journaled.

```c
void m3World_EnableSleeping(m3WorldId worldId, bool flag);
```
Turn island sleeping on or off. Turning it OFF wakes every sleeping body (the reference behavior). Journaled.

```c
bool m3World_IsSleepingEnabled(m3WorldId worldId);
```
Whether island sleeping is enabled.

```c
void m3World_EnableContinuous(m3WorldId worldId, bool flag);
```
Turn the continuous phase (CCD) on or off. Journaled.

```c
bool m3World_IsContinuousEnabled(m3WorldId worldId);
```
Whether the continuous phase is enabled.

```c
void m3World_SetWind(m3WorldId worldId, m3Vec3 direction, float speed, float gustHertz, float gustScale);
```
Wind (11-3): a deterministic field applied to soft-body particles as proportional drag toward the wind velocity. The gust is a sine of an ACCUMULATED phase (state, in the snapshot), so rollbacks resume the exact same wave: no host randomness exists anywhere in it. speed zero disables. direction must be near-unit when speed is nonzero. Rigid bodies are not wind-blown in v1 (documented: the force API already serves them); journaled.

```c
void m3World_Step(m3WorldId worldId, float dt, int32_t substeps);
```
Advance the simulation: collide, then the Soft Step solver with the given substep count (1..256; out of range refuses loudly). Deterministic: same inputs, same bits, on every platform and backend. Journaled. THE THREADING CONTRACT (integration audit B2/D3). Per world: one writer at a time; Step and every mutating call demand exclusive access to that world. Between steps, any number of concurrent READERS (queries, casts, getters, snapshot) may run on the same world from any threads. DISTINCT worlds are fully independent: stepping two worlds on two host threads is supported and bit-identical to stepping them serially (the concurrency suite proves it; the TSAN cell polices it). m3CreateWorld and m3DestroyWorld touch a process-wide slot registry and must be serialized BY THE HOST across threads; the library adds no lock (zero-dependency law). Reading a world WHILE it steps is undefined.

```c
int32_t m3World_SnapshotSize(m3WorldId worldId);
```
Snapshot and rollback, first-class from day one. The format is portable and versioned: field blocks in little-endian order with a header carrying a config hash (engine version, solver revision, precision, FP policy). Restore refuses a mismatched config or format loudly, restores in place, and the restored world resimulates bit-exactly (the rollback gate, task 10).

```c
int32_t m3World_Snapshot(m3WorldId worldId, void* out, int32_t capacity);
```

```c
bool m3World_Restore(m3WorldId worldId, const void* data, int32_t size);
```

```c
const m3ContactEvent* m3World_ContactBeginEvents(m3WorldId worldId, int32_t* count);
```
Contact begin and end streams for the LAST step, in canonical deterministic order. Valid until the next step, restore, or world destruction (restore clears them: events are observations, not state). Pass a non-null count.

```c
const m3ContactEvent* m3World_ContactEndEvents(m3WorldId worldId, int32_t* count);
```

```c
const m3FragmentEvent* m3World_FragmentEvents(m3WorldId worldId, int32_t* count);
```

```c
const uint16_t* m3World_FragmentRecipe(m3WorldId worldId, int32_t* count);
```

```c
int32_t m3World_FragmentEventsDropped(m3WorldId worldId);
```
Islands beyond the event capacity are still removed from the grid (state transitions stay pure); only their EVENTS drop, and this counter says how many, loudly.

```c
const m3ContactEvent* m3World_SensorBeginEvents(m3WorldId worldId, int32_t* count);
```
Sensor overlap events, the same law as contact events but in their own streams (a sensor touch is not a contact). shapeA is the lower shape index; either side may be the sensor.

```c
const m3ContactEvent* m3World_SensorEndEvents(m3WorldId worldId, int32_t* count);
```

```c
const m3HitEvent* m3World_HitEvents(m3WorldId worldId, int32_t* count);
```

```c
int32_t m3World_HitEventsDropped(m3WorldId worldId);
```
Hits beyond capacity still simulate; only their events drop, and this counter says how many, loudly.

```c
void m3World_SetHitEventThreshold(m3WorldId worldId, float value);
```
Hit events require approach speed above this. Journaled.

```c
const m3BodyMoveEvent* m3World_BodyMoveEvents(m3WorldId worldId, int32_t* count);
```

```c
const m3JointBreakEvent* m3World_JointBreakEvents(m3WorldId worldId, int32_t* count);
```

```c
void m3World_SetPreSolveCallback(m3WorldId worldId, m3PreSolveFn* fn, void* context);
```
Register (or clear with NULL) the pre-solve callback. The REGISTRATION is host wiring (never journaled, never snapshot state), but the DECISIONS are not: every veto a step makes is journaled as that step's annex (R5-4), so a bare replay (m3replay verify included) applies the recorded vetoes with no callback installed and lands on the recorded bits. During such a step the recorded set wins and any installed callback stays silent; do not expect callbacks to fire under replay.

```c
m3RayHit m3World_CastMover(m3WorldId worldId, m3Pos3 center, m3real halfHeight, m3real radius, m3Vec3 translation);
```
Cast the mover capsule along a translation; the closest blocking hit (sensors are invisible to movers).

```c
int32_t m3World_CollideMover(m3WorldId worldId, m3Pos3 center, m3real halfHeight, m3real radius, m3real skin, m3MoverPlane* planes, int32_t capacity);
```
Collect contact planes for the mover at rest: every shape within `skin` of the capsule contributes one plane. Returns the count written (ascending shape order, deterministic).

```c
m3Vec3 m3SolvePlanes(m3Vec3 translation, const m3MoverPlane* planes, int32_t count, int32_t iterations);
```
Clamp a desired translation against contact planes (the reference's iterative accumulator): each iteration pushes the translation out of every violated plane, push impulses stay nonnegative per plane. Pure function, world-free. Only the first 64 planes take part; later planes are ignored.

```c
m3RayHit m3World_CastRayClosest(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation);
```

```c
m3RayHit m3World_CastRayClosestEx(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation, m3QueryFilter filter);
```
Filtered variants (8-1): the query carries an m3QueryFilter (defined in shape.h) and behaves like a shape with those bits; the unfiltered forms see everything.

```c
int32_t m3World_CastRayAll(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation, m3RayHit* hits, int32_t capacity);
```
Every ray hit along the translation, one entry point per shape, sorted by fraction (ties to the lower shape index). Returns the count written (never more than capacity; excess hits are dropped from the FAR end, deterministically).

```c
int32_t m3World_CastRayAllEx(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation, m3RayHit* hits, int32_t capacity, m3QueryFilter filter);
```

```c
m3RayHit m3World_CastBoxClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents, m3Quat rotation, m3Vec3 translation);
```
Closest-hit shape casts: sweep a sphere or a capsule along a translation. fraction is the earliest touch in [0, 1]; a cast that STARTS overlapped hits at fraction zero with a zero normal (the documented start-inside contract; rays instead MISS shapes they start inside, front faces only). Generic convex casts (4-1): a box (with orientation) or a caller point cloud (2..24 points, base-relative) swept along a translation, skinless. Same contracts as every cast: the earliest touch in [0, 1], start-overlapped reports fraction zero with a zero normal, hostile inputs miss. A single point is a ray: use the ray casts.

```c
m3RayHit m3World_CastBoxClosestEx(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents, m3Quat rotation, m3Vec3 translation, m3QueryFilter filter);
```

```c
m3RayHit m3World_CastHullClosest(m3WorldId worldId, m3Pos3 base, const m3Vec3* points, int32_t count, m3Vec3 translation);
```

```c
m3RayHit m3World_CastHullClosestEx(m3WorldId worldId, m3Pos3 base, const m3Vec3* points, int32_t count, m3Vec3 translation, m3QueryFilter filter);
```

```c
m3RayHit m3World_CastSphereClosest(m3WorldId worldId, m3Pos3 center, m3real radius, m3Vec3 translation);
```

```c
m3RayHit m3World_CastSphereClosestEx(m3WorldId worldId, m3Pos3 center, m3real radius, m3Vec3 translation, m3QueryFilter filter);
```

```c
m3RayHit m3World_CastCapsuleClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 point1, m3Vec3 point2, m3real radius, m3Vec3 translation);
```

```c
m3RayHit m3World_CastCapsuleClosestEx(m3WorldId worldId, m3Pos3 center, m3Vec3 point1, m3Vec3 point2, m3real radius, m3Vec3 translation, m3QueryFilter filter);
```

```c
m3ShapeId m3World_PointInside(m3WorldId worldId, m3Pos3 point);
```
The first shape (lowest index) whose volume contains the point, or the null id. Meshes are open surfaces and never contain points; planes are solid half spaces.

```c
int32_t m3World_OverlapAabb(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes, int32_t capacity);
```
Shapes whose tight bounds overlap the box, in ascending shape index order. Returns the count written.

```c
int32_t m3World_OverlapAabbEx(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter);
```

```c
int32_t m3World_OverlapSphere(m3WorldId worldId, m3Pos3 center, m3real radius, m3ShapeId* shapes, int32_t capacity);
```
Shapes within reach of the sphere (exact per family), in ascending shape index order. Returns the count written.

```c
int32_t m3World_OverlapSphereEx(m3WorldId worldId, m3Pos3 center, m3real radius, m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter);
```

```c
int32_t m3World_OverlapCapsule(m3WorldId worldId, m3Pos3 p1, m3Pos3 p2, m3real radius, m3ShapeId* shapes, int32_t capacity);
```
The exact overlap family (15-3): capsule, oriented box, and raw convex cloud queries with the same contract as the sphere: shapes within EXACT reach (GJK per candidate, planes and voxel/mesh handled per family), ascending shape index, filtered, read-only. Clouds carry base-relative points (the cast convention), at most 64 of them.

```c
int32_t m3World_OverlapCapsuleEx(m3WorldId worldId, m3Pos3 p1, m3Pos3 p2, m3real radius, m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter);
```

```c
int32_t m3World_OverlapBox(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents, m3Quat rotation, m3ShapeId* shapes, int32_t capacity);
```

```c
int32_t m3World_OverlapBoxEx(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents, m3Quat rotation, m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter);
```

```c
int32_t m3World_OverlapHullPoints(m3WorldId worldId, m3Pos3 base, const m3Vec3* points, int32_t count, m3real radius, m3ShapeId* shapes, int32_t capacity);
```

```c
int32_t m3World_OverlapHullPointsEx(m3WorldId worldId, m3Pos3 base, const m3Vec3* points, int32_t count, m3real radius, m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter);
```

```c
m3ExplosionDef m3DefaultExplosionDef(void);
```
Returns a def with pinned defaults (10 m radius, 5 m falloff, zero impulse, no carve, soft push 1, open filter) and a valid cookie.

```c
void m3World_Explode(m3WorldId worldId, const m3ExplosionDef* def);
```
Apply the explosion now. Journaled; hostile defs apply nothing and journal nothing.

```c
uint64_t m3World_Hash(m3WorldId worldId);
```
FNV-1a 64 over the curated deterministic state (transforms, velocities, mass, types, step count, gravity) in canonical slot order. The value every gate compares.

```c
m3Counters m3World_GetCounters(m3WorldId worldId);
```

```c
m3MemoryUsage m3World_MemoryUsage(m3WorldId worldId);
```

```c
m3Profile m3World_GetProfile(m3WorldId worldId);
```

```c
bool m3World_JournalBegin(m3WorldId worldId, void* buffer, int32_t capacity);
```
Journal: every mutation of the world is a discrete recorded op, and replaying the stream through the same internal functions reproduces the world bit for bit. Begin hands the world a caller-owned buffer; End returns the bytes written (or -1 after an overflow, loudly); Replay applies a stream to this world.

```c
int32_t m3World_JournalEnd(m3WorldId worldId);
```

```c
bool m3World_JournalReplay(m3WorldId worldId, const void* data, int32_t size);
```
Replay a recorded session into this world. ATOMIC: on any refusal (truncation, corruption, an op that cannot re-mint its recorded id) the world is restored to its pre-call state and false returns; a half-applied session is impossible.

## `body.h`

Rigid bodies: creation and destruction, motion, forces and impulses, mass, sleep, and state readback.

```c
m3BodyDef m3DefaultBodyDef(void);
```
Returns a def with pinned defaults (identity rotation, gravity scale one) and a valid cookie.

```c
m3BodyId m3CreateBody(m3WorldId worldId, const m3BodyDef* def);
```
Create a body. Returns the null id on an invalid def, a stale world, or an exhausted body pool (loud in debug builds). A shapeless dynamic body has unit mass and zero inertia until a shape provides the real values (task 7).

```c
void m3DestroyBody(m3BodyId bodyId);
```
Destroy a body. The id goes stale; the slot recycles FIFO with a generation bump and retires instead of wrapping.

```c
bool m3Body_IsValid(m3BodyId bodyId);
```

```c
m3Pos3 m3Body_GetPosition(m3BodyId bodyId);
```

```c
m3Quat m3Body_GetRotation(m3BodyId bodyId);
```

```c
m3Vec3 m3Body_GetLinearVelocity(m3BodyId bodyId);
```

```c
m3Vec3 m3Body_GetAngularVelocity(m3BodyId bodyId);
```

```c
uint64_t m3Body_GetUserData(m3BodyId bodyId);
```

```c
m3BodyType m3Body_GetType(m3BodyId bodyId);
```

```c
void m3Body_SetTransform(m3BodyId bodyId, m3Pos3 position, m3Quat rotation);
```
Journaled setters: every mutation is a discrete op. Runtime control (8-3), all journaled. SetTransform is the teleport: the pose lands instantly, velocities stay, and bodies around BOTH the old and new locations wake so nothing keeps sleeping under or inside a teleported crate.

```c
void m3Body_SetTargetTransform(m3BodyId bodyId, m3Pos3 position, m3Quat rotation);
```
Kinematic servo: velocities are chosen at the next step so the body lands ON the target pose after that step, then the target clears. Kinematic bodies only (the correct way to drive elevators and doors).

```c
void m3Body_SetType(m3BodyId bodyId, m3BodyType type);
```
Switch dynamic, kinematic, static at runtime. Mass rebuilds from shapes when turning dynamic; velocities zero when turning static; the neighborhood wakes.

```c
void m3Body_SetEnabled(m3BodyId bodyId, bool enabled);
```
A disabled body vanishes from simulation AND queries without being destroyed; enabling wakes its neighborhood.

```c
bool m3Body_IsEnabled(m3BodyId bodyId);
```

```c
void m3Body_SetMotionLocks(m3BodyId bodyId, uint32_t locks);
```
Motion locks: bits 0..2 freeze linear x, y, z; bits 3..5 freeze angular x, y, z. Locked components re-zero every substep, so 2.5D scenes and upright enemies stay exact.

```c
uint32_t m3Body_GetMotionLocks(m3BodyId bodyId);
```

```c
void m3Body_SetAllowFastRotation(m3BodyId bodyId, bool allow);
```
Let this body spin past the world's angular speed cap (for wheels and other legal fast spinners). Journaled.

```c
bool m3Body_GetAllowFastRotation(m3BodyId bodyId);
```

```c
void m3Body_SetName(m3BodyId bodyId, const char* name);
```
Debug name, up to 31 bytes plus the terminator; longer names truncate silently. Journaled and carried by snapshots, never part of the hash (a label moves no matter). GetName returns the empty string for unnamed bodies and stale ids.

```c
const char* m3Body_GetName(m3BodyId bodyId);
```

```c
int32_t m3Body_GetContactData(m3BodyId bodyId, m3ContactData* out, int32_t capacity);
```
Who touches me now (14-3): fills up to capacity entries and returns the count written. See m3ContactData in world.h.

```c
void m3Body_SetSleepControls(m3BodyId bodyId, float threshold, bool canSleep);
```
Sleep controls: a per-body velocity threshold (zero restores the world default) and a can-sleep switch.

```c
bool m3Body_IsAwake(m3BodyId bodyId);
```

```c
void m3Body_SetAwake(m3BodyId bodyId, bool awake);
```
SetAwake(true) wakes; SetAwake(false) puts the single body to sleep and zeroes its velocities.

```c
void m3Body_ApplyForce(m3BodyId bodyId, m3Vec3 force);
```
Forces and impulses (8-2), journaled like every mutation. Forces and torques ACCUMULATE and act over the next step, then clear; impulses change velocity immediately. Only awake-able dynamic bodies respond: static and kinematic targets and non-finite values are documented no-ops. A nonzero application wakes the body. Points are world-space; an off-center application adds the matching angular part.

```c
void m3Body_ApplyTorque(m3BodyId bodyId, m3Vec3 torque);
```

```c
void m3Body_ApplyLinearImpulse(m3BodyId bodyId, m3Vec3 impulse);
```

```c
void m3Body_ApplyAngularImpulse(m3BodyId bodyId, m3Vec3 impulse);
```

```c
void m3Body_ApplyForceAtPoint(m3BodyId bodyId, m3Vec3 force, m3Pos3 point);
```

```c
void m3Body_ApplyLinearImpulseAtPoint(m3BodyId bodyId, m3Vec3 impulse, m3Pos3 point);
```

```c
void m3Body_SetLinearVelocity(m3BodyId bodyId, m3Vec3 velocity);
```

```c
void m3Body_SetAngularVelocity(m3BodyId bodyId, m3Vec3 velocity);
```

## `character.h`

The character controller: a kinematic capsule moved by collide-and-slide over the engine's convex casts. Its body never moves during m3World_Step; every displacement comes from m3Character_Move, a journaled command that rollback covers. Gravity is the host's job: add it to each move.

```c
m3CharacterDef m3DefaultCharacterDef(void);
```

```c
m3CharacterId m3CreateCharacter(m3WorldId worldId, const m3CharacterDef* def);
```
Creates the character and its internal kinematic capsule body (destroyed together). The body collides like any kinematic: dynamics bounce off it; it walks through nothing.

```c
void m3DestroyCharacter(m3CharacterId characterId);
```

```c
bool m3Character_IsValid(m3CharacterId characterId);
```

```c
void m3Character_Move(m3CharacterId characterId, m3Vec3 translation);
```
Collide-and-slide by `translation`, immediately: cast the capsule, advance to the first hit, slide the remainder along the surface, repeat (bounded), then snap to walkable ground within snapDistance when descending. Walkable is measured against maxSlopeAngle; ceilings and steep walls only ever block and slide. Journaled; hostile translations no-op.

```c
m3Pos3 m3Character_GetPosition(m3CharacterId characterId);
```

```c
bool m3Character_IsGrounded(m3CharacterId characterId);
```

```c
m3Vec3 m3Character_GetGroundNormal(m3CharacterId characterId);
```
The last walkable surface normal (zeros while airborne).

```c
m3BodyId m3Character_GetGroundBody(m3CharacterId characterId);
```
The body under the character's feet (null while airborne or after that body is destroyed). Each m3World_Step carries grounded characters along with this body's motion through the regular slide casts: kinematic platforms, elevators, and dynamic fragments all ferry their riders (4-6).

```c
bool m3Character_SetStance(m3CharacterId characterId, m3real halfHeight, m3real radius);
```
Stance (12-3): resize the capsule in place, FEET ANCHORED (the center moves so the capsule bottom stays level; a mid air crouch therefore lands shorter, not higher). Shrinking always applies. Growing runs the stand-up veto: the grown capsule is cast at its new pose through the same core every move uses, and ANY contact within the skin (a pressing ceiling, a wall against a wider radius) refuses the whole change and returns false; stance is untouched. Journaled only when applied; the veto's refusal is bit-deterministic so twins and replays refuse together. Stance is state: snapshotted, hashed, rolled back like everything else.

```c
void m3Character_GetStance(m3CharacterId characterId, m3real* halfHeight, m3real* radius);
```
Read the live stance (zeros on a stale id).

## `draw.h`

Debug draw: the world describes itself as segments, points and triangles through host callbacks. A draw pass only reads simulation state; a test hashes the world across a draw to check that.

```c
void m3World_Draw(m3WorldId worldId, const m3DebugDraw* draw);
```
Emit the world through the callbacks. Null callbacks are skipped; the call itself never mutates the world.

```c
void m3World_DrawSolid(m3WorldId worldId, const m3SolidDraw* draw);
```
Emit every live shape as triangles: spheres and capsules tessellate at fixed counts, hulls fan their face loops, meshes emit their stored triangles, voxel surfaces emit their merged-box faces. Infinite planes are SKIPPED (a viewer draws its own ground). Read-only like m3World_Draw and held by the same purity test: a draw pass never moves the world hash.

```c
void m3World_DrawExtras(m3WorldId worldId, const m3ExtraDraw* draw);
```

## `joint.h`

Joints: the seven joint types, their defs, limits, motors, springs, constraint force readback and breaking.

```c
m3JointDef m3DefaultJointDef(void);
```
The pinned joint defaults: spherical type, zero anchors, unit z axes, everything disabled, collideConnected off, and the def cookie every create demands.

```c
m3JointId m3CreateJoint(const m3JointDef* def);
```
Create a joint between two distinct bodies of the same world (at least one dynamic). Returns the null id on a bad def, a stale body, or an exhausted pool. Journaled; replay verifies the minted id. Destroying either body destroys the joint.

```c
void m3DestroyJoint(m3JointId jointId);
```

```c
bool m3Joint_IsValid(m3JointId jointId);
```

```c
void m3Joint_SetLimits(m3JointId jointId, bool enable, float lower, float upper);
```
Runtime joint control (8-6a). All journaled; both bodies wake on any change. Limits and motor reuse the def semantics per type (angles for revolute and spherical twist, meters for prismatic and distance); toggling zeroes the row's stored impulse so a stale warm start cannot kick.

```c
void m3Joint_SetMotor(m3JointId jointId, bool enable, float speed, float maxEffort);
```

```c
void m3Joint_SetMotorPose(m3JointId jointId, m3Vec3 offset, m3Quat rotation);
```
Aim the MOTOR joint (16-5): offset in body A's joint frame, rotation as the target relative orientation. Journaled; refused loudly on other types and hostile values.

```c
void m3Joint_SetSteer(m3JointId jointId, bool enable, float targetAngle, float hertz, float zeta, float maxEffort);
```
Wheel steering (16-3): a soft target-angle drive about the strut axis. While enabled, the wheel's frame-x lock becomes the drive (frame y stays locked), so the axle yaws toward targetAngle (radians, |target| <= 1) at the given stiffness; maxEffort > 0 caps the torque, 0 leaves it uncapped. Wheel joints only; refused loudly elsewhere. Journaled.

```c
float m3Joint_GetSteerAngle(m3JointId jointId);
```
The current strut twist in radians (wheel joints; 0 for every other type and stale ids).

```c
void m3Joint_SetCollideConnected(m3JointId jointId, bool collide);
```
Let (or forbid) the two jointed bodies collide with each other. Journaled; binds at the next step's pair scan.

```c
bool m3Joint_GetCollideConnected(m3JointId jointId);
```

```c
void m3Joint_SetBreakThresholds(m3JointId jointId, float maxForce, float maxTorque);
```
Breakage (8-6a), a deliberate addition over the reference: rollback games need breakage as a deterministic in-step state transition, not a host poll racing the journal. When either reaction magnitude exceeds its cap at the end of a step, the joint destroys itself and emits the joint break event (see world.h). Zero disables a cap; both zero (the default) means unbreakable.

```c
m3real m3Joint_GetConstraintForce(m3JointId jointId);
```
Reaction readback (8-6a): MAGNITUDES of the last step's constraint reactions, assembled per type from the stored solver rows (linear rows into force, angular rows into torque; the generic joint reports a conservative sum). Reads 0 before the first step after a restore (documented transient). The reference's vector form waits for a consumer with a direction to point at (argued in the plan).

```c
m3real m3Joint_GetConstraintTorque(m3JointId jointId);
```

```c
m3real m3Joint_GetAngle(m3JointId jointId);
```
Geometry reads: the revolute twist angle about the hinge (radians) and the prismatic translation along the slide axis (meters). Wrong-type calls read 0.

```c
m3real m3Joint_GetTranslation(m3JointId jointId);
```

```c
void m3Joint_SetSpring(m3JointId jointId, bool enable, float hertz, float dampingRatio);
```
Position drive (8-6b), the reference spring rows: a soft constraint with the given frequency and damping ratio pulls the joint toward its target. Revolute drives the hinge angle, prismatic the translation, spherical the relative rotation; other types refuse. Toggling zeroes the spring's stored impulse. All journaled; both bodies wake.

```c
void m3Joint_SetTargetAngle(m3JointId jointId, float radians);
```

```c
void m3Joint_SetTargetTranslation(m3JointId jointId, float meters);
```

```c
void m3Joint_SetTargetRotation(m3JointId jointId, m3Quat target);
```
The target must be a unit rotation; garbage refuses loudly by doing nothing. The target lives in the JOINT FRAMES (frame z is the create-time local axis): it is the desired rotation of frame B relative to frame A, the reference semantic. Pick your axes at create time so the frame reads naturally.

## `replay.h`

Replays: the M3J1 container that seals a snapshot, a journal and the final hash into one artifact, and the functions that write, verify, seek and compare replays.

```c
bool m3JournalDescribe(const void* journal, int32_t bytes, m3JournalInfo* out);
```

```c
int32_t m3ReplayEncodeSize(int32_t snapshotBytes, int32_t journalBytes);
```
Exact encoded size for the given payload sizes.

```c
int32_t m3ReplayEncode(const void* snapshot, int32_t snapshotBytes, const void* journal, int32_t journalBytes, uint64_t finalHash, void* out, int32_t capacity);
```
Encode a container. finalHash is the recorder's end hash (m3World_Hash after the session); verifiers compare against it. Returns bytes written, or 0 on refusal (bad sizes, small capacity, malformed journal).

```c
bool m3ReplayDecode(const void* data, int32_t bytes, m3ReplayView* out);
```
Decode and validate framing (magic, version, lengths, journal record walk). Returns false on any corruption; the view is untouched on refusal.

```c
int32_t m3World_DiffReport(m3WorldId worldA, m3WorldId worldB, m3BodyDiff* out, int32_t capacity, int32_t* outCount);
```
Compare every body slot of two worlds (same capacities expected; mismatched capacities refuse). Writes up to capacity rows sorted worst-first, sets outCount to the number written, and returns the TOTAL number of differing slots (which may exceed capacity), or -1 on refusal. Zero means the worlds agree body-for-body.

## `shape.h`

Shapes: geometry, materials, filters, compounds, meshes, heightfields and voxel chunks.

```c
m3ShapeDef m3DefaultShapeDef(void);
```
Returns a def with pinned defaults (density 1, friction 0.6, restitution 0) and a valid cookie.

```c
m3ShapeId m3CreateSphereShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Sphere* sphere);
```
Create a sphere on a body and recompute the body's mass from its shapes (sphere mass = density * 4/3 pi r^3, inertia 0.4 m r^2). Returns the null id on an invalid def, a stale body, an exhausted pool, or an off-origin center on a dynamic body (the 2a rule: scalar inertia stays exact; the full tensor arrives in 2b).

```c
m3ShapeId m3CreatePlaneShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Plane* plane);
```
Create a half-space on a STATIC body (a plane on a dynamic body returns the null id loudly). The normal is normalized on create.

```c
m3ShapeId m3CreateBoxShape(m3BodyId bodyId, const m3ShapeDef* def, m3Vec3 halfExtents);
```
Create a box (a convex hull with analytic mass properties: m = 8 rho hx hy hz, I = m/3 diag(hy^2+hz^2, ...) about the center). Hull collision lands in the GJK and SAT slices; until then a box participates in the broadphase and in mass only.

```c
m3ShapeId m3CreateCapsuleShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Capsule* capsule);
```
A capsule: the segment point1..point2 inflated by radius. A zero-length segment is refused (that shape is a sphere; use m3CreateSphereShape).

```c
m3ShapeId m3CreateCylinderShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Cylinder* cylinder, int32_t segments);
```

```c
bool m3Shape_SetSphere(m3ShapeId shapeId, const m3Sphere* sphere);
```
Runtime geometry replacement (15-2): swap a sphere or capsule shape's geometry in place; conversions between the two are legal. Hulls, meshes, voxels, and planes refuse loudly (interned slabs are immutable by law). Journaled; mass, extents, and the broadphase follow, and sleepers around both the old and the new bounds wake.

```c
bool m3Shape_SetCapsule(m3ShapeId shapeId, const m3Capsule* capsule);
```

```c
m3ShapeId m3CreateHullShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Vec3* points, int32_t count);
```
A convex hull built from a point cloud (QuickHull, coplanar faces merged, mass integrated). Between 4 and 64 finite points; degenerate clouds (coplanar, collinear) and hulls that exceed the 64-vertex budget (input clouds up to 256 points) return the null id, loudly.

```c
m3ShapeId m3CreateMeshShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Vec3* vertices, int32_t vertexCount, const uint16_t* indices, int32_t triangleCount);
```
A static triangle mesh (level geometry, voxel chunk surfaces). Static bodies only; up to 65,535 vertices and 65,535 triangles per mesh (three indices each, counter-clockwise from outside: contacts cull the back side). Ghost collisions on shared edges are filtered by feature welding. Refused loudly on a dynamic body, out-of-cap counts, or out-of-range indices.

```c
void m3Shape_SetMeshMaterials(m3ShapeId shapeId, const m3MeshSurfaceMaterial* materials, int32_t materialCount, const uint8_t* triangleMaterials);
```
Paint material groups onto a mesh shape: up to 8 entries and one group index byte per triangle (triangleMaterials, length = the mesh's triangle count, every byte < materialCount). Journaled; refuses loudly on a non-mesh shape, hostile values, or out-of-range group bytes. A mesh without painted materials keeps using its shape material everywhere.

```c
m3ShapeId m3CreateHeightFieldShape(m3BodyId bodyId, const m3ShapeDef* def, const float* heights, int32_t nx, int32_t nz, m3real cellSize);
```
A heightfield chunk: an nx by nz grid of heights (row-major, x fastest) spaced `cellSize` apart in x and z, triangulated into a static mesh through the same welded triangle path. Grid limits per chunk: 2..32 in each direction (the mesh caps); larger terrain tiles as chunks, the voxel-world model. The grid's minimum corner sits at the body origin.

```c
m3ShapeId m3CreateHeightFieldGridShape(m3BodyId bodyId, const m3ShapeDef* def, const float* heights, int32_t nx, int32_t nz, m3real cellSize);
```
The NATIVE grid heightfield (19-1): the same nx-by-nz grid contract as above, but stored as raw heights (four bytes per sample) instead of triangulated into a mesh: the low-memory terrain path. Grid limits per chunk: 2..255 in each direction; larger terrain tiles as chunks. Static bodies only; the minimum corner sits at the body origin.

```c
m3ShapeId m3CreateVoxelChunkShape(m3BodyId bodyId, const m3ShapeDef* def, const uint8_t* voxels, const uint16_t* payload, m3real cellSize);
```
True while the id names a live shape in a live world; false for the null id, stale generations, and destroyed worlds. A voxel chunk: a dense 16x16x16 grid anchored at the body origin, cells of `cellSize` meters. `voxels` is one byte per voxel (zero = empty), x fastest then y then z; `payload` is an optional uint16 per voxel (material, health, type), carried as state and hashed. Static bodies only; sensors refused (sensor volumes are convex by contract); an empty grid refused. Solid voxels are CLOSED volumes: point-inside answers true inside filled cells. Rays, contacts, shape casts, and continuous collision all see voxel chunks; fast bodies and bullets sweep the merged surface, so a one-voxel-thick wall stops a bullet, and a wall whose voxel was cleared last step is a REAL hole.  Seam welding: chunks weld automatically when both bodies carry the exact identity rotation, cell sizes match, and world positions differ by exactly one chunk extent along one axis (grid-laid level geometry). Welded borders are interior geometry and never produce contact features, so bodies roll and slide across chunk boundaries without seam impulses. Rotated or misaligned chunks still collide correctly; they just do not weld.

```c
bool m3VoxelChunk_SetVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z, uint16_t payload);
```
Voxel edits (3-2): deterministic state transitions, journaled and replayed like every other mutation, fully inside the rollback delta. The collision surface rebuilds as a pure function of the grid after any occupancy change (a payload-only set touches no geometry), and dynamic bodies whose bounds touch the edited region are woken (a floor vanishing under a sleeper is a disturbance). Coordinates are voxel indices in [0, 15]; out-of-range and inverted regions refuse loudly. A chunk edited down to empty stays a valid shape that collides with nothing (the natural end state of destruction).  The anchor convention: an island is anchored through the CHUNK's y = 0 base layer, not through world ground. Build structures at their chunk's base; a floating platform is a chunk whose BODY sits in the air with the platform at local y = 0. Voxels with no path to the base layer survive creation but fragment on the first clearing edit anywhere in the chunk (the sweep is chunk-wide by design).

```c
bool m3VoxelChunk_ClearVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z);
```

```c
int32_t m3VoxelChunk_ClearBox(m3ShapeId shapeId, const int32_t lo[3], const int32_t hi[3]);
```
Clears the inclusive box [lo, hi] per axis. Returns the number of voxels that were actually cleared (zero is legal), or -1 on a stale id, a non-voxel shape, or a bad region.

```c
bool m3VoxelChunk_SetFill(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z, uint8_t fill);
```
Fill fraction (3-6): 255 is a whole voxel, 1 a sliver. A mass and destruction property, never geometry: the voxel still collides as a full box, but its fragment mass scales by fill / 255. Refuses zero fill (that is a clear: use m3VoxelChunk_ClearVoxel) and empty voxels. Journaled state, hashed, inside the rollback delta like every edit.

```c
bool m3Shape_IsValid(m3ShapeId shapeId);
```

```c
m3BodyId m3Shape_GetBody(m3ShapeId shapeId);
```
The owning body (22-2: the mover recipe needs to tell its own shape from the world's). Null id for stale shapes.

```c
int32_t m3Shape_GetContactData(m3ShapeId shapeId, m3ContactData* out, int32_t capacity);
```
Who touches this shape now (14-3): fills up to capacity entries and returns the count written. See m3ContactData.

```c
void m3DestroyShape(m3ShapeId shapeId);
```
Destroy one shape and rebuild the owner's mass books (10-4). Journaled; contacts involving the shape dissolve at the next step. The last shape leaves a shapeless dynamic body at unit mass (the reference convention).

```c
void m3Shape_SetFriction(m3ShapeId shapeId, float friction);
```
Runtime material setters (8-4). Journaled; contacts read materials at prepare, so changes bind from the next step. A sleeping stack keeps its old mix until something wakes it (the reference behavior, documented).

```c
float m3Shape_GetFriction(m3ShapeId shapeId);
```

```c
void m3Shape_SetRestitution(m3ShapeId shapeId, float restitution);
```

```c
float m3Shape_GetRestitution(m3ShapeId shapeId);
```

```c
void m3Shape_SetRollingResistance(m3ShapeId shapeId, float value);
```

```c
float m3Shape_GetRollingResistance(m3ShapeId shapeId);
```

```c
void m3Shape_SetDensity(m3ShapeId shapeId, float density, bool updateBodyMass);
```
Set the density and optionally rebuild the owning body's mass, center and inertia from all its shapes. Journaled.

```c
float m3Shape_GetDensity(m3ShapeId shapeId);
```

```c
void m3Shape_EnableHitEvents(m3ShapeId shapeId, bool flag);
```
Opt a shape into hit events / the pre-solve veto (8-5). Journaled; the flags are state and snapshot with the world.

```c
bool m3Shape_AreHitEventsEnabled(m3ShapeId shapeId);
```

```c
void m3Shape_EnablePreSolve(m3ShapeId shapeId, bool flag);
```

```c
bool m3Shape_IsPreSolveEnabled(m3ShapeId shapeId);
```

```c
void m3Shape_SetSurfaceVelocity(m3ShapeId shapeId, m3Vec3 velocity);
```
Conveyor (11-3): a world-frame surface velocity on the shape. Contacts drive the tangential target toward it, the reference tangentVelocity semantic the central-friction port carried at zero until now. Journaled; state, hashed when nonzero.

## `softbody.h`

Soft bodies: XPBD particle lattices bound by distance constraints and solved in a fixed index order each substep. Rope, cloth and jelly come from one factory. Ids are pooled, operations journaled and state snapshotted and hashed, so rollback covers them exactly.

```c
m3SoftBodyDef m3DefaultSoftBodyDef(void);
```

```c
m3SoftBodyId m3CreateSoftBody(m3WorldId worldId, const m3SoftBodyDef* def);
```
Creates the lattice with structural edges and face diagonals in fixed index order. Refuses (null id) hostile counts, non-finite or non-positive geometry, or a full pool. Journaled with id verification.

```c
m3SoftBodyId m3CreateSoftBodyTet(m3WorldId worldId, const m3SoftBodyDef* def, const m3Vec3* points, int32_t pointCount, const uint16_t* tets, int32_t tetCount);
```
A tetrahedral soft body (20-3): explicit points (world positions after adding def->position) and tets (four point indices each, positive volume required). Edges come from the tet edges, deduplicated in first-touch order, at def->compliance; every tet holds its create volume rigidly (the incompressible jelly). def->countX/Y/Z, spacing, bendCompliance, and pressure must be left at defaults (the lattice knobs; hostile mixes refuse loudly). Pins, anchors, wind, water, explosions, and collision all treat the particles exactly like lattice particles.

```c
void m3DestroySoftBody(m3SoftBodyId softId);
```

```c
bool m3SoftBody_IsValid(m3SoftBodyId softId);
```

```c
void m3SoftBody_PinParticle(m3SoftBodyId softId, int32_t particle);
```
Pins one particle in place (inverse mass zero): how a rope hangs and a flag flies. Journaled; out-of-range or stale pins are quiet no-ops. Particle index is x + countX * (y + countY * z).

```c
void m3SoftBody_AnchorParticle(m3SoftBodyId softId, int32_t particle, m3BodyId bodyId);
```
Anchors one particle to a body at the particle's CURRENT position, expressed in the body's frame: the particle rides the body from then on, and the lattice's pull on it lands on the body as an impulse at the anchor (two-way, 7-3). Cloth hangs from beams and jelly rides trucks through this. Anchoring to a static body is a moving pin; the anchor RELEASES silently if its body dies. Journaled; stale ids, out-of-range particles, and a full anchor table (32 per soft body) are quiet no-ops.

```c
void m3SoftBody_AnchorToSoft(m3SoftBodyId softIdA, int32_t particleA, m3SoftBodyId softIdB, int32_t particleB);
```
Pin a particle of one lattice to a particle of ANOTHER lattice (11-2): a position equality split by inverse mass, solved each substep after soft-vs-soft contact. Released silently when EITHER lattice dies. Journaled; the pin lives in the lower slot's table (one canonical home per pair).

```c
int32_t m3SoftBody_GetParticleCount(m3SoftBodyId softId);
```

```c
m3Pos3 m3SoftBody_GetParticlePosition(m3SoftBodyId softId, int32_t particle);
```

## `vehicle.h`

Raycast vehicles: each wheel is a suspension ray from the chassis with a spring and damper at the contact and tire friction in the contact plane. Vehicles are stepped inside m3World_Step in slot order and fully snapshotted, so a replayed or rolled-back drive lands on identical bits.

```c
m3VehicleDef m3DefaultVehicleDef(void);
```

```c
m3VehicleId m3CreateVehicle(m3WorldId worldId, const m3VehicleDef* def);
```
Creates the vehicle bound to its chassis body. Refuses (null id) a non-dynamic or stale chassis, a wheel count outside [1, M3_VEHICLE_MAX_WHEELS], or any non-finite or non-positive geometry. Journaled with id verification.

```c
void m3DestroyVehicle(m3VehicleId vehicleId);
```

```c
bool m3Vehicle_IsValid(m3VehicleId vehicleId);
```

```c
m3real m3Vehicle_GetCompression(m3VehicleId vehicleId, int32_t wheel);
```
Last step's suspension compression of one wheel, in meters from rest length (zero when airborne, stale, or out of range). Frozen while the chassis sleeps, like the chassis.

```c
bool m3Vehicle_IsWheelGrounded(m3VehicleId vehicleId, int32_t wheel);
```
Whether the wheel's suspension cast found ground last step.

```c
void m3Vehicle_SetTankCommands(m3VehicleId vehicleId, m3real left, m3real right, m3real brake);
```
Drive commands (5-2): journaled STATE, not per-step parameters, so replay and rollback hold to the bit. Values clamp to their ranges (throttle and steer to [-1, 1], brake to [0, 1]); non-finite commands are hostile no-ops that never journal. Setting commands wakes the chassis. The chassis-local +x axis is the vehicle's forward by convention; steer rotates each steerable wheel's frame about its suspension axis by steer * maxSteerAngle. Tank commands (23-1): per-side throttle for skid steering. Driven wheels split by the sign of their chassis-local anchor z (+z right); each side takes its own throttle in [-1, 1], the brake rides all wheels. Engaging tank mode suspends the normal throttle until m3Vehicle_SetCommands is called again (which disengages it). Drivetrain vehicles refuse: a gearbox and a skid steer are different machines.

```c
void m3Vehicle_SetCommands(m3VehicleId vehicleId, m3real throttle, m3real steer, m3real brake);
```

```c
m3real m3Vehicle_GetWheelSpin(m3VehicleId vehicleId, int32_t wheel);
```
Accumulated spin angle of one wheel in radians (rendering state: hosts spin their wheel meshes with it).

```c
m3DrivetrainDef m3DefaultDrivetrainDef(void);
```

```c
void m3Vehicle_SetDrivetrain(m3VehicleId vehicleId, const m3DrivetrainDef* def);
```
Attach a drivetrain (journaled). Replaces the flat driveForce model for this vehicle; starts in first gear, clutch closed. Invalid defs (bad cookie, non-ascending curve, out-of-range counts, non-finite numbers) are documented no-ops.

```c
void m3Vehicle_SelectGear(m3VehicleId vehicleId, int32_t gear);
```
Select a gear (journaled): -1 reverse, 0 neutral, 1..gearCount. Opens the clutch for clutchSteps. A no-op without a drivetrain, out of range, or when reverseRatio is zero and gear is -1.

```c
int32_t m3Vehicle_GetGear(m3VehicleId vehicleId);
```
Current gear; 0 when no drivetrain is attached.

```c
m3real m3Vehicle_GetEngineRpm(m3VehicleId vehicleId);
```
Engine speed computed by the last step, idle-floored like the torque lookup (a tachometer, not raw wheel math).

---

234 functions across 11 headers.
