# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release could change the API, the ABI and the
snapshot and journal formats. From 1.0.0 every 1.x release keeps the
API, and an entry says what each changed result changes.

## [1.0.1](https://github.com/Rawframe-Project/maul3d/compare/v1.0.0...v1.0.1) (2026-11-03)

### Fixed

- **A journal replay never asks the pre-solve callback.** A replayed
  step that recorded no veto used to call a callback installed on the
  replaying world. A callback that vetoed then made the replay land on
  other bits while it still returned `m3_success`. The replay now
  applies exactly the vetoes each step recorded, as
  `m3World_SetPreSolveCallback` promised; the `m3PreSolveFunction`
  comment, which said to register the callback before replaying, was
  the wrong one and is corrected. Results change only for a replay run
  with a callback installed, which now lands on the recorded bits. A
  1.0.0 snapshot steps to the same bits, so 1.0.1 restores it.
  ([#363](https://github.com/Rawframe-Project/maul3d/pull/363))

## [1.0.0](https://github.com/Rawframe-Project/maul3d/compare/v0.13.1...v1.0.0) (2026-11-03)

### Changed

- **The API is settled, and every 1.x release keeps it.** A program
  that builds against 1.0.0 builds against any later 1.x and means the
  same: a minor release adds functions, types, enum values and fields
  at the end of a struct, and never removes or renames one; a patch
  release only fixes. A change that gives existing inputs a different
  result ships as a `feat` in a minor release, or as a `fix` in a patch
  release when it corrects a result the documentation already
  promised, and its changelog entry says what changes. There is no
  2.0.0. The API and the results are those of 0.13.1.
  ([#355](https://github.com/Rawframe-Project/maul3d/pull/355))
- **The binary interface holds within a minor version.**
  `M3_ABI_VERSION` is now the major and minor version, so a program
  built against 1.0.x headers runs on any 1.0.x library, and
  `m3CreateWorld` refuses a def from another minor version's headers
  with `m3_errorVersion`. The shared library's `SOVERSION` is `1.0`,
  and `find_package(maul3d 1.0)` accepts any later 1.x.
  ([#355](https://github.com/Rawframe-Project/maul3d/pull/355))
- **The C# binding keeps the same promise as the C headers.**
  `bindings/csharp/Maul3D.cs` ships with every release; a name it binds
  is never removed or renamed in 1.x. Bind more of the API in a file of
  your own, so a later `Maul3D.cs` replaces this one. The Godot starter
  kit stays outside the promise.
  ([#355](https://github.com/Rawframe-Project/maul3d/pull/355))
- **Formats.** The snapshot keeps format 6, the journal tape format 7
  and the replay container format 4. They carry the new
  `M3_ABI_VERSION`, so 1.0.0 refuses a snapshot, tape or container
  that 0.13.1 or any earlier release wrote with `m3_errorVersion`.
  From 1.0.0 the patch releases of a minor version read each other's,
  except that a release that changed a result refuses a snapshot
  written before it.

## [0.13.1](https://github.com/Rawframe-Project/maul3d/compare/v0.13.0...v0.13.1) (2026-11-03)

### Changed

- **Formats.** The snapshot keeps format 6, the journal tape format 7
  and the replay container format 4. As in every release, a snapshot,
  tape or container written by another release is refused with
  `m3_errorVersion`.

### Fixed

- **Restore refuses a touching contact pair on a dead shape.** A
  shape's destroy clears the contacts of its pairs, but their keys stay
  until the next step, so a snapshot taken in between holds pairs that
  name a dead shape. Restore checked only that a touching pair was not
  two sensors, so a snapshot could also hold a touching pair on a dead
  shape. After such a restore, `m3Body_GetContactData` and
  `m3Shape_GetContactData` read 24 bytes before the world's transforms
  array, and `m3World_Draw` and `m3Shape_SetFilter` read the same
  missing body. Restore now refuses such bytes with `m3_errorFormat`
  and leaves the world as it was. Snapshots the library writes still
  restore. ([#350](https://github.com/Rawframe-Project/maul3d/pull/350))
- **The contact draw skips a pair that does not touch.**
  `m3World_Draw` with `drawContacts` read the body of each pair's first
  shape before checking whether the pair touched. Drawing between a
  shape's destroy and the next step therefore read in front of the
  transforms array for each pair whose first shape was the dead one.
  Nothing was drawn from those bytes. The draw now skips a pair with no contact points first
  and draws the same primitives as before.
  ([#350](https://github.com/Rawframe-Project/maul3d/pull/350))

## [0.13.0](https://github.com/Rawframe-Project/maul3d/compare/v0.12.0...v0.13.0) (2026-11-02)

### Changed

- **Breaking: five names after the final shape review.** Each keeps
  its value, type and meaning; rename your uses as below. Neither
  binding exposes any of them.

  | 0.12.0 | 0.13.0 |
  |---|---|
  | `m3_voxelShape` | `m3_voxelChunkShape` |
  | `m3Vehicle_GetCompression` | `m3Vehicle_GetWheelCompression` |
  | `m3SoftBody_AnchorToSoft` | `m3SoftBody_AnchorToSoftBody` |
  | `m3DrivetrainDef.autoShift` | `enableAutoShift` |
  | `m3DrivetrainDef.diffMode`, `diffCouple` | `differentialMode`, `differentialCoupling` |
  | `M3_MAX_DRIVETRAIN_CURVE` | `M3_MAX_DRIVETRAIN_CURVE_POINTS` |
  | `M3_SERVO_REACH_LIMIT` | `M3_TARGET_REACH_LIMIT` |
  | `M3_BLAST_LIMIT` | `M3_EXPLOSION_IMPULSE_LIMIT` |

  ([#344](https://github.com/Rawframe-Project/maul3d/pull/344))
- **Breaking: a dropped explosion carve has its own counter.**
  `m3Counters.voxelEditsRefused` counted both voxel edits refused with
  `m3_errorLimit` and explosion carves `m3World_Explode` skipped for
  the same surface bound, which no status reports. It now counts the
  edits only, and the new `m3Counters.voxelCarvesDropped` counts the
  skipped carves. A program that watched `voxelEditsRefused` for
  carves reads `voxelCarvesDropped`.
  ([#346](https://github.com/Rawframe-Project/maul3d/pull/346))
- **Breaking: `m3Counters` drops three fields other calls report.**
  `scratchPeak` and `scratchCapacity` are gone: read them from
  `m3World_GetMemoryUsage`. `snapshotBytes` is gone: call
  `m3World_GetSnapshotSize`. `m3World_GetCounters` no longer measures
  the snapshot.
  ([#347](https://github.com/Rawframe-Project/maul3d/pull/347))
- **Formats.** The snapshot keeps format 6, the journal tape format 7
  and the replay container format 4. As in every release, a snapshot,
  tape or container written by another release is refused with
  `m3_errorVersion`.

### Fixed

- **Breaking: a spherical def refuses a negative cone angle.**
  `m3CreateSphericalJoint` took a negative `coneAngle` while
  `enableCone` was false, a value `m3SphericalJoint_SetConeAngle`
  refuses with the cone on or off. The create now refuses it with
  `m3_errorInvalid` and one misuse whether the cone is on or off, and
  restore and replay refuse such bytes. Pass an angle of zero or more.
  ([#341](https://github.com/Rawframe-Project/maul3d/pull/341))

## [0.12.0](https://github.com/Rawframe-Project/maul3d/compare/v0.11.0...v0.12.0) (2026-11-02)

### Changed

- **Breaking: a def, a default and a create per joint type.**
  `m3JointDef`, `m3DefaultJointDef` and `m3CreateJoint` are gone. Each
  of the twelve types has its own def, default and create:
  `m3SphericalJointDef` with `m3DefaultSphericalJointDef` and
  `m3CreateSphericalJoint`, and the same for the revolute, prismatic,
  weld, distance, generic, wheel, filter, parallel, motor, gear and
  pulley joints. Each def holds only the fields its type reads, so a
  cone on a wheel or a spring on a weld can no longer be written.
  Rename the fields you set as below. A create refuses another type's
  def with `m3_errorInvalid` and one misuse. The default wheel def
  builds: its suspension points down -y with its axle along z. The C#
  `JointDef`, `DefaultJointDef` and `CreateJoint` are replaced by the
  same twelve structs, defaults and creates.

  | 0.11.0 `m3JointDef` field | 0.12.0 |
  |---|---|
  | `type` | gone: call the create for your type |
  | `lowerLimit`, `upperLimit` | spherical `lowerTwist`, `upperTwist`; revolute `lowerAngle`, `upperAngle`; prismatic and wheel `lowerTranslation`, `upperTranslation` |
  | wheel `localAxisB` | `localSpinAxisB` |
  | `genericLinear`, `genericAngular` | `linearModes`, `angularModes` |
  | `genericLinearLower`, `genericLinearUpper` | `linearLower`, `linearUpper` |
  | `genericAngularLower`, `genericAngularUpper` | `angularLower`, `angularUpper` |
  | `genericMotorAxis` | `motorAxis`, `M3_GENERIC_NO_MOTOR` for none |

  ([#299](https://github.com/Rawframe-Project/maul3d/pull/299))
- **Breaking: a dynamic body bounds its contact lever ratio.** A
  colliding shape far from its body's center of mass, on a body a
  joint also holds, let the contact go: past a ratio of about 1e7,
  mass times the squared arm over the moment that arm turns, the
  contact and the joint stopped settling and the body fell through.
  `M3_CONTACT_LEVER_LIMIT`, 1e6, is the new bound. A shape create,
  destroy, resize or density change, a filter change that lets a
  shape collide, and a type change to dynamic are refused with
  `m3_errorLimit` when they would carry the ratio past it. They change
  nothing and count no misuse. Restore and replay refuse such bytes.
  `m3Shape_SetFilter` now returns `m3Result`, so check its status.
  ([#302](https://github.com/Rawframe-Project/maul3d/pull/302))
- **Breaking: body names are UTF-8 with a byte count.**
  `m3Body_SetName(world, body, name, byteCount)` returns `m3Result`.
  It refuses a name longer than `M3_MAX_BODY_NAME_BYTES`, 31 bytes,
  with `m3_errorLimit` where it cut the name, and bytes that are not
  well-formed UTF-8 or hold a zero byte with `m3_errorInvalid`. Each
  refusal keeps the old name and counts one misuse. A count of 0
  clears the name. `m3Body_GetName(world, body, nameOut, capacity)`
  copies the name into your buffer with no terminator and returns its
  byte length; call it with a NULL buffer and capacity 0 to measure.
  ([#293](https://github.com/Rawframe-Project/maul3d/pull/293))
- **Breaking: `m3World_Compare` returns a status.** It takes
  `rowsOut`, `capacity` and `totalOut` in place of `out` and
  `outCount`, writes up to `capacity` rows and sets `*totalOut` to the
  number of differing slots, which may exceed the capacity. Read
  `min(total, capacity)` rows. Worlds of different capacities now
  compare slot by slot instead of being refused. A refusal returns
  `m3_errorInvalid` and counts one misuse, where it returned -1 and
  counted none. `m3EncodeReplay`'s buffer parameter is named
  `containerOut`.
  ([#294](https://github.com/Rawframe-Project/maul3d/pull/294))
- **Breaking: the point test takes a filter, the capsule overlap a
  center.** `m3World_TestPoint(world, point, filter)` answers with the
  lowest shape the filter sees; pass `m3DefaultQueryFilter()` to keep
  the old answer. `m3World_OverlapCapsule` takes a center and two
  points relative to it, as `m3World_CastCapsuleClosest` does, in
  place of two world points.
  ([#297](https://github.com/Rawframe-Project/maul3d/pull/297))
- **Breaking: seven internal math helpers leave `core_math.h`.**
  `m3MinF`, `m3MaxF`, `m3ClampF`, `m3AbsF`, `m3MakeZeroMat3`,
  `m3UnwindAngle` and `m3IntegrateRotation` are no longer declared.
  Use your own. The vector, matrix and quaternion math, `m3ComputeCosSin`
  and `m3Atan2` stay.
  ([#298](https://github.com/Rawframe-Project/maul3d/pull/298))
- **Formats.** The snapshot keeps format 6, the journal tape format 7
  and the replay container format 4. As in every release, a snapshot,
  tape or container written by another release is refused with
  `m3_errorVersion`.

### Fixed

- **A generic joint's motor follows its switch.** It drove the named
  motor axis whether the motor was on or off, so
  `m3Joint_EnableMotor(world, joint, false)` did not stop it. It now
  drives only while the switch is on, like every other motor; set
  `enableMotor` in the def to keep it driving.
  ([#301](https://github.com/Rawframe-Project/maul3d/pull/301))
- **A plane sits in its body's frame everywhere.** The contacts, the
  fast-body sweep, the mover and the shape casts read a plane as world
  space, while the ray casts, overlaps and point test placed it with
  its body. A plane on a moved or turned static body collided in a
  different place than queries found it. A plane on a body at the
  origin collides exactly as before.
  ([#312](https://github.com/Rawframe-Project/maul3d/pull/312))
- **A fast body is swept only against shapes it can touch.** The
  continuous phase moved a fast body back to the time it hit a shape
  it never gets a contact with, so it lost most of a step's travel and
  went through on the next one. It hit planes that are sensors, hidden
  by its filter or on a disabled body, static surfaces its filter
  hides, and shapes of a body it is jointed to with connected
  collision off. The sweep now asks the same pair rule as contacts.
  Scenes with such joints step to different results.
  ([#310](https://github.com/Rawframe-Project/maul3d/pull/310),
  [#315](https://github.com/Rawframe-Project/maul3d/pull/315))
- **The fast-body sweep places offset shapes.** It swept every shape
  as if it sat at its body's origin with no turn, so a fast body could
  pass through an offset box, voxel chunk or plane, or stop in the air
  above one. Bodies with offset shapes step to different results.
  ([#319](https://github.com/Rawframe-Project/maul3d/pull/319))
- **Queries skip what they should.** `m3World_TestPoint` no longer
  reports shapes of disabled bodies, and `m3World_OverlapAabb` and
  `m3World_OverlapSphere` no longer report planes the filter hides or
  planes on disabled bodies.
  ([#296](https://github.com/Rawframe-Project/maul3d/pull/296),
  [#309](https://github.com/Rawframe-Project/maul3d/pull/309))
- **A box overlap refuses a rotation that is not unit length.**
  `m3World_OverlapBox` took any finite quaternion and answered for a
  stretched or shrunk box. A rotation whose squared length is not
  between 0.98 and 1.02 is now refused as the box cast refuses it: the
  call returns 0, zeroes the array and counts one misuse.
  ([#306](https://github.com/Rawframe-Project/maul3d/pull/306))
- **An explosion carves only chunks its filter sees.**
  `m3World_Explode` carved every voxel chunk in reach, including
  chunks its filter hides and chunks on disabled bodies, while its
  push already skipped both.
  ([#318](https://github.com/Rawframe-Project/maul3d/pull/318))

## [0.11.0](https://github.com/Rawframe-Project/maul3d/compare/v0.10.0...v0.11.0) (2026-11-01)

### Added

- **The spherical joint's cone at run time.**
  `m3SphericalJoint_EnableCone`, `IsConeEnabled`, `SetConeAngle` and
  `GetConeAngle` switch and move the swing cone without recreating the
  joint, so a ragdoll keeps its warm impulses. Each call wakes the
  joint's bodies. The angle must be finite and not negative, and
  `EnableCone` refuses to switch the cone on over a negative angle a
  def left. Any other joint type or a stale id is refused with one
  misuse, and the getters then read false and 0.
  ([#205](https://github.com/Rawframe-Project/maul3d/pull/205))

### Changed

- **Breaking: names follow maul2d's and the API's own patterns.**
  Rename your calls as below. Each does what the old name did, except
  where the two height field creates trade names: a call written for
  the old `m3CreateHeightFieldShape` still compiles and now builds the
  native grid, so a program that wants the triangle mesh must call
  `m3CreateHeightFieldMeshShape`.

  | 0.10.0 | 0.11.0 |
  |---|---|
  | `m3AabbResult` in `body.h` | `m3WorldAabb` in `world.h` |
  | `m3Joint_GetConstraintForce`, `GetConstraintTorque` | `m3Joint_GetReactionForce`, `GetReactionTorque` |
  | `m3Joint_SetBreakThresholds` | `m3Joint_SetBreakLimits` |
  | `m3_fixedJoint`, value 3 | `m3_weldJoint`, value 3 |
  | `m3CreateHeightFieldGridShape` | `m3CreateHeightFieldShape` |
  | `m3CreateHeightFieldShape`, the triangle mesh | `m3CreateHeightFieldMeshShape` |
  | `m3CreateSoftBodyTet` | `m3CreateTetSoftBody` |
  | `M3_MOVER_PLANES` | `M3_MAX_MOVER_PLANES` |
  | `m3World_OverlapHullPoints` | `m3World_OverlapHull` |
  | `m3FragmentEvent`'s `comChunk`, `comWorld`, `boundsLo`, `boundsHi` | `localCenter`, `worldCenter`, `lowerBound`, `upperBound` |
  | `m3WaterVolumeDef`'s `lo`, `hi` | `bounds.lowerBound`, `bounds.upperBound` |
  | `m3WheelDef`'s `hertz`, `zeta` | `springHertz`, `springDampingRatio` |
  | `m3Counters`' `misuse`, `allocCalls`, `freeCalls`, `heldPushRetakes` | `misuseCount`, `allocCount`, `freeCount`, `heldPushRetakeCount` |
  | `m3Body_GetRotationalInertia` | `m3Body_GetCentralInertia` |
  | C# `JointType.Fixed` | `JointType.Weld` |
  | Godot `substeps` property | `substep_count` |

  `m3Joint_GetBreakLimits` is new and reads back what
  `m3Joint_SetBreakLimits` set, 0 for a cap that is off.
  ([#207](https://github.com/Rawframe-Project/maul3d/pull/207),
  [#255](https://github.com/Rawframe-Project/maul3d/pull/255),
  [#256](https://github.com/Rawframe-Project/maul3d/pull/256),
  [#257](https://github.com/Rawframe-Project/maul3d/pull/257),
  [#258](https://github.com/Rawframe-Project/maul3d/pull/258),
  [#263](https://github.com/Rawframe-Project/maul3d/pull/263))
- **Breaking: the distance joint has its own range, spring and rest
  length.** Its range is `m3JointDef.minLength` and `maxLength`, set
  with `m3DistanceJoint_SetLengthRange` and read with
  `GetLengthRange`. Its spring is switched by the def's `enableSpring`
  with `springHertz` and `springDampingRatio`, and by
  `m3Joint_EnableSpring` and the spring setters at run time. It pulls
  toward `m3JointDef.length`, set with `m3DistanceJoint_SetLength`,
  which need not lie inside the range. The joint takes no limit and no
  motor, so a def that still sets the range through `enableLimit` or
  the spring through `enableMotor` is refused at create, as are the
  limit and motor calls on it. The def's spring fields now switch the
  spring at create on every joint type that takes one. The C#
  `JointDef` gains `MinLength`, `MaxLength`, `Length`, `EnableSpring`,
  `SpringHertz` and `SpringDampingRatio`.
  ([#191](https://github.com/Rawframe-Project/maul3d/pull/191),
  [#202](https://github.com/Rawframe-Project/maul3d/pull/202))
- **Breaking: kinds and modes are fixed-width integers.**
  `m3BodyType`, `m3ShapeType`, `m3JointType` and `m3AxisMode` are
  `int32_t` typedefs with the same named values, so source that uses
  the names compiles as before. `m3DrivetrainDef.diffMode` is an
  `m3DifferentialMode`: `m3_differentialOpen`,
  `m3_differentialLimitedSlip` or `m3_differentialLocked`. The generic
  joint's modes are `m3AxisMode` arrays, so `m3JointDef` grows from
  248 to 272 bytes. The C# defs hold `BodyType`, `JointType` and a new
  `AxisMode` enum.
  ([#282](https://github.com/Rawframe-Project/maul3d/pull/282))
- **Breaking: refusals a correct program can meet return a status.**
  These calls carry `M3_NODISCARD` and return `m3Result`:
  - `m3VoxelChunkShape_SetVoxel`, `ClearVoxel`, `SetFill` and
    `ClearBox` return `m3_errorLimit` when the chunk's surface would
    pass `M3_MAX_VOXEL_SURFACE_BOXES`, 2048 boxes, and change nothing.
    `ClearBox` writes its count through a new `int32_t* clearedOut`.
  - `m3SoftBody_PinParticle`, `AnchorParticle` and `AnchorToSoft`
    return `m3_errorLimit` for a full table of
    `M3_MAX_SOFTBODY_ANCHORS`.
  - `m3Body_SetType`, `m3Body_SetTargetTransform`,
    `m3Shape_SetDensity`, `m3Shape_SetSphere` and `m3Shape_SetCapsule`
    return `m3_errorLimit` for a body mass past `M3_MASS_LIMIT`, a
    joint past its lever ratio, a type change the body's shapes, joints
    or speed refuse, or a servo target out of reach.

  A refusal the world's state decides no longer counts misuse, in
  these calls and in the shape and joint creates and `m3DestroyShape`.
  A stale id is `m3_errorStale`, a density past `M3_DENSITY_LIMIT`
  `m3_errorLimit` and a bad argument `m3_errorInvalid`, each still
  counting one misuse.
  ([#265](https://github.com/Rawframe-Project/maul3d/pull/265),
  [#266](https://github.com/Rawframe-Project/maul3d/pull/266),
  [#268](https://github.com/Rawframe-Project/maul3d/pull/268))
- **Breaking: `m3World_Step` returns `m3_errorLimit` past its
  published bounds.** A dt below `M3_MIN_STEP` or above
  `M3_STEP_LIMIT`, an infinite one included, and more than
  `M3_MAX_SUBSTEPS` substeps were `m3_errorInvalid`. A NaN dt, a dt of
  zero or below and fewer than one substep stay `m3_errorInvalid`.
  Each still counts one misuse. The step's second parameter is named
  `substepCount`.
  ([#283](https://github.com/Rawframe-Project/maul3d/pull/283),
  [#263](https://github.com/Rawframe-Project/maul3d/pull/263))
- **Breaking: list calls take the filter before the array and return
  the true total.** `m3World_OverlapAabb`, `OverlapSphere`,
  `OverlapCapsule`, `OverlapBox`, `OverlapHull` and `CastRayAll` take
  the query filter before the output array. `m3World_OverlapAabb`
  takes one `m3WorldAabb` in place of its two corners. Those six,
  `m3World_CollideMover`, `m3Body_GetContactData` and
  `m3Shape_GetContactData` write at most capacity entries and return
  the total, which may exceed the capacity, where they returned the
  count written. A NULL array with capacity 0 measures. A NULL array
  with a positive capacity, or a negative capacity, is refused with one
  misuse by these calls and by `m3World_GetBodies`, `GetJoints`,
  `m3Body_GetShapes` and `GetJoints`, which took both before.
  ([#207](https://github.com/Rawframe-Project/maul3d/pull/207),
  [#216](https://github.com/Rawframe-Project/maul3d/pull/216))
- **Breaking: a refused list call or snapshot zeroes the caller's
  buffer.** Every list call above and `m3World_Compare` zero their
  array over its whole capacity when they refuse, a stale id included,
  so an earlier answer is never read as this one. A successful answer,
  an empty one included, writes only its entries. A refused
  `m3World_Snapshot` zeroes its buffer over its capacity, so a host
  that rings snapshot buffers should size each with
  `m3World_GetSnapshotSize` first.
  ([#259](https://github.com/Rawframe-Project/maul3d/pull/259))
- **Breaking: one hit type for every cast.** `m3CastHit`
  {`shapeId`, `point`, `normal`, `fraction`} replaces
  `m3RayCastResult` and `m3RayHit`. `m3World_CastRayClosest`,
  `CastSphereClosest`, `CastCapsuleClosest`, `CastBoxClosest`,
  `CastHullClosest` and `CastMover` return `bool` and take
  `m3CastHit* hitOut` last. Each clears it first, so a miss leaves
  zeros, and a NULL `hitOut` returns false with one misuse.
  `m3World_CastRayAll` fills `m3CastHit`. The C# `RayCastResult` is
  `CastHit`, and `CastRayClosest` takes `out CastHit hit`.
  ([#211](https://github.com/Rawframe-Project/maul3d/pull/211))
- **Breaking: sensor events name the sensor, the visitor and the
  step.** `m3SensorEvents` holds arrays of `m3SensorBeginEvent` and
  `m3SensorEndEvent`, each {`sensorShapeId`, `visitorShapeId`,
  `step`}, in place of the contact event structs with `shapeIdA` and
  `shapeIdB`. `step` is the world's step count before the step, so the
  first step's events carry 0.
  ([#217](https://github.com/Rawframe-Project/maul3d/pull/217))
- **Breaking: snapshot, journal and replay sizes are 64-bit, with no
  cap.** `m3World_GetSnapshotSize`, `m3World_Snapshot`,
  `m3World_Restore`, `m3World_StartJournal`, `m3World_StopJournal`,
  `m3World_ReplayJournal`, `m3DescribeJournal`,
  `m3GetEncodedReplaySize`, `m3EncodeReplay` and `m3DecodeReplay` take
  and return `int64_t` sizes, as do `m3ReplayView`'s sizes,
  `m3JournalInfo`'s counts and `m3Counters.snapshotBytes`, and the C#
  snapshot calls take `long`. `M3_MAX_SNAPSHOT_BYTES` is removed, and
  `m3CreateWorld` no longer limits mesh and hull slots, and a height
  field grid's create no longer refuses samples, to keep a snapshot
  under 2 GiB. On a 32-bit target a size past
  `SIZE_MAX` is `m3_errorInvalid`.
  ([#226](https://github.com/Rawframe-Project/maul3d/pull/226))
- **Breaking: what a failed call leaves in `bytesOut`.**
  `m3World_Snapshot` and `m3World_StopJournal` write 0 when the world
  is NULL, where they left it. A stop refused as busy, from the world's
  own callback, leaves it as it was, where it wrote 0.
  `m3EncodeReplay` leaves it as it was on any failure, where it
  wrote 0.
  ([#210](https://github.com/Rawframe-Project/maul3d/pull/210),
  [#215](https://github.com/Rawframe-Project/maul3d/pull/215))
- **Breaking: `m3IsCpuBackendSupported` returns `bool`** in place of
  an `int32_t` 1 or 0, and `m3Hash64` takes an `int64_t` byte count.
  ([#260](https://github.com/Rawframe-Project/maul3d/pull/260))
- **Breaking: joints solve their coupled rows as one block.** A
  slider, weld or generic weld whose anchor sits far from its body's
  center of mass gave way by metres: on a 100 m arm a slider sagged
  2.4 m off its axis in a second. It now holds within 1e-4 m. The
  prismatic, weld, revolute, wheel and generic joints step to
  different results, and the joint-heavy bench scene costs about 18%
  more a step, the others about 5%.
  ([#232](https://github.com/Rawframe-Project/maul3d/pull/232))
- **Breaking: `m3Counters.awakeCount` counts only enabled bodies.** A
  body disabled while awake keeps its awake flag and was counted,
  though it takes no part in the step.
  ([#287](https://github.com/Rawframe-Project/maul3d/pull/287))
- **Formats.** Journal tapes are format 7 and the replay container is
  format 4, with a 56-byte header. As in every release, a snapshot,
  tape or container written by another release is refused with
  `m3_errorVersion`.
  ([#191](https://github.com/Rawframe-Project/maul3d/pull/191),
  [#202](https://github.com/Rawframe-Project/maul3d/pull/202),
  [#205](https://github.com/Rawframe-Project/maul3d/pull/205),
  [#226](https://github.com/Rawframe-Project/maul3d/pull/226),
  [#282](https://github.com/Rawframe-Project/maul3d/pull/282))

### Removed

- **Breaking: the `m3real` typedef.** Every call and field that used
  it takes `float`, the same type, so replace the name and nothing else
  moves.
  ([#262](https://github.com/Rawframe-Project/maul3d/pull/262))

### Fixed

- A shape created in a destroyed shape's slot took over its contact
  pairs: an end event for a touch it never had, a lost begin event, the
  dead pair's warm impulses, and for a sleeping pair a world that no
  longer stepped like one restored at the same moment. A destroy now
  forgets its shape's pairs. The first destroy after a step builds an
  index of the pairs, about 0.4 ms in a world of 138,000 pairs, and
  later destroys cost what they did.
  ([#219](https://github.com/Rawframe-Project/maul3d/pull/219),
  [#223](https://github.com/Rawframe-Project/maul3d/pull/223))
- A free hook that read a shape or body back during its destroy found
  a live id over content already freed. The id is now stale before any
  content is freed.
  ([#278](https://github.com/Rawframe-Project/maul3d/pull/278))
- A reader called from the free hook during `m3DestroyWorld` walked
  pools already freed, a use after free under AddressSanitizer. The
  hook now finds an empty world, and every writer it calls is refused
  as busy.
  ([#285](https://github.com/Rawframe-Project/maul3d/pull/285))
- `m3EncodeReplay` copied into an output that overlapped the snapshot
  or the tape, which C leaves undefined. It now refuses with
  `m3_errorInvalid` and writes nothing.
  ([#281](https://github.com/Rawframe-Project/maul3d/pull/281))
- A call given a NULL world and a live id returned `m3_errorStale`. It
  returns `m3_errorInvalid`.
  ([#275](https://github.com/Rawframe-Project/maul3d/pull/275))
- `m3World_StopJournal` on an overflowed journal returned
  `m3_errorState` with one misuse, where its documentation says
  `m3_errorCapacity`. It returns `m3_errorCapacity` and counts none.
  ([#221](https://github.com/Rawframe-Project/maul3d/pull/221))
- The joint create accepted `enableCone` on every type. On a wheel it
  switched steering on at 0 Hz, and the world's own snapshot was then
  refused by its restore. Only the spherical joint takes it now.
  ([#204](https://github.com/Rawframe-Project/maul3d/pull/204))
- `m3World_Restore` accepted values no call can write, and now refuses
  them with `m3_errorFormat`: a motor joint aim past 4e15 m on any
  axis, a nonzero third drive value on a joint type that never writes
  it, and a touching pair of two sensors, which the next step reported
  as a sensor event.
  ([#198](https://github.com/Rawframe-Project/maul3d/pull/198),
  [#201](https://github.com/Rawframe-Project/maul3d/pull/201),
  [#225](https://github.com/Rawframe-Project/maul3d/pull/225))
- `m3World_Restore` took the body name slots without a check, so a
  snapshot whose name slot held no zero byte restored, and
  `m3Body_GetName` then returned text that ran past its slot. Restore
  refuses such a slot with `m3_errorFormat`.
  ([#292](https://github.com/Rawframe-Project/maul3d/pull/292))
- `m3replay` sized a container with a 32-bit offset on Windows and
  32-bit Linux, so a container past 2 GiB read as invalid. It uses a
  64-bit offset and names each read failure.
  ([#231](https://github.com/Rawframe-Project/maul3d/pull/231))

## [0.10.0](https://github.com/Rawframe-Project/maul3d/compare/v0.9.0...v0.10.0) (2026-10-29)

### Added

- **Readers for wheel steering.** `m3WheelJoint_IsSteeringEnabled`,
  `m3WheelJoint_GetTargetSteeringAngle`, `GetSteeringHertz`,
  `GetSteeringDampingRatio` and `GetMaxSteeringTorque` read back what
  the steering setters store.
  ([#188](https://github.com/Rawframe-Project/maul3d/pull/188))

### Changed

- **Breaking: public names follow the Maul family's naming rule.** A call
  on one object is named after that object. A call that accepts only one
  joint or shape kind is named after that kind. Getters take `Get`, and
  every on/off switch is an `Enable` / `Is...Enabled` pair. Hook types end
  in `Function`. Rename your calls as below. Each one does what the old
  name did, except `m3Body_Enable`, which now takes a bool, and steering,
  covered in the next item.

  | 0.9.0 | 0.10.0 |
  |---|---|
  | `m3TaskFn`, `m3EnqueueTaskFn`, `m3FinishTaskFn` | `m3TaskFunction`, `m3EnqueueTaskFunction`, `m3FinishTaskFunction` |
  | `m3PreSolveFn` | `m3PreSolveFunction` |
  | `m3ResultName` | `m3GetResultName` |
  | `m3CpuSupportsBackend` | `m3IsCpuBackendSupported` |
  | `m3World_SnapshotSize` | `m3World_GetSnapshotSize` |
  | `m3Body_Enable` and `m3Body_Disable` | `m3Body_Enable` with `true` or `false` |
  | `m3BodyDef.isEnabled` | `m3BodyDef.enable` |
  | `m3Body_SetBullet`, `m3Body_IsBullet` | `m3Body_EnableBullet`, `m3Body_IsBulletEnabled` |
  | `m3BodyDef.isBullet` | `m3BodyDef.enableBullet` |
  | `m3Shape_EnablePreSolve`, `m3Shape_IsPreSolveEnabled` | `m3Shape_EnablePreSolveEvents`, `m3Shape_IsPreSolveEventsEnabled` |
  | `m3Shape_SetMeshMaterials` | `m3MeshShape_SetMaterials` |
  | `m3VoxelChunk_SetVoxel`, `ClearVoxel`, `ClearBox`, `SetFill` | `m3VoxelChunkShape_SetVoxel`, `ClearVoxel`, `ClearBox`, `SetFill` |
  | `m3Joint_SetTargetAngle` | `m3RevoluteJoint_SetTargetAngle` |
  | `m3Joint_SetTargetRotation` | `m3SphericalJoint_SetTargetRotation` |
  | `m3Joint_SetSteer` | see the next item |
  | `m3Joint_GetSteerAngle` | `m3WheelJoint_GetSteeringAngle` |
  | `m3Vehicle_SelectGear` | `m3Vehicle_SetGear` |
  | C# `Native.ResultName` | `Native.GetResultName` |
  | C# `Native.SnapshotSize` | `Native.GetSnapshotSize` |
  | C# `BodyDef.IsBullet` | `BodyDef.EnableBullet` |
  | C# `BodyDef.IsEnabled` | `BodyDef.Enable` |

  Struct layouts and the snapshot format are unchanged. The journal stays
  at format 3. ([#188](https://github.com/Rawframe-Project/maul3d/pull/188))
- **Breaking: wheel steering has a switch and a setter per value.**
  `m3Joint_SetSteer(enable, targetAngle, hertz, zeta, maxEffort)` becomes
  `m3WheelJoint_EnableSteering(enable)` and four setters:
  `m3WheelJoint_SetTargetSteeringAngle`, `SetSteeringHertz`,
  `SetSteeringDampingRatio` and `SetMaxSteeringTorque`. `zeta` is now
  the damping ratio, and the budget is a torque, 0 for no cap. Steering
  keeps its values while it is off. As before, enabled steering needs a
  positive frequency. Because the switch is now a call of its own, set
  the frequency first and then enable steering: enabling it before any
  frequency is set is refused and counts one misuse. Each call records
  its own journal entry. A tape that steers is therefore longer than the
  same scene recorded with 0.9.0, and its bytes hash differently, while
  the simulation itself is unchanged.
  ([#188](https://github.com/Rawframe-Project/maul3d/pull/188))

### Fixed

- The motor joint's getters read zeros for a joint of another type
  without counting a misuse. They now count one, as the wheel joint's
  steering getters do.
  ([#188](https://github.com/Rawframe-Project/maul3d/pull/188))

## [0.9.0](https://github.com/Rawframe-Project/maul3d/compare/v0.8.0...v0.9.0) (2026-10-29)

### Added

- **Getters for joint limits, motors and springs.** `m3Joint_IsLimitEnabled`
  and `m3Joint_GetLimits`, `m3Joint_IsMotorEnabled`,
  `m3Joint_GetMotorSpeed` and `m3Joint_GetMaxMotorEffort`, and
  `m3Joint_IsSpringEnabled`, `m3Joint_GetSpringHertz` and
  `m3Joint_GetSpringDampingRatio` read back what the setters store.
  ([#185](https://github.com/Rawframe-Project/maul3d/pull/185))
- **Getters for the motor joint.** `m3MotorJoint_GetMaxForce`,
  `GetMaxTorque`, `GetLinearOffset` and `GetAngularOffset`.
  ([#187](https://github.com/Rawframe-Project/maul3d/pull/187))

### Changed

- **Breaking: a joint's limits, motor and spring each have a switch and a
  setter per value.** `m3Joint_SetLimits(enable, lower, upper)` becomes
  `m3Joint_EnableLimit(enable)` and `m3Joint_SetLimits(lower, upper)`.
  `m3Joint_SetMotor(enable, speed, maxEffort)` becomes
  `m3Joint_EnableMotor`, `m3Joint_SetMotorSpeed` and
  `m3Joint_SetMaxMotorEffort`.
  `m3Joint_SetSpring(enable, hertz, dampingRatio)` becomes
  `m3Joint_EnableSpring`, `m3Joint_SetSpringHertz` and
  `m3Joint_SetSpringDampingRatio`. A part keeps its values while it is
  off. A limit range must be ordered when it is set and whenever the
  limit is on, and a spring may hold 0 Hz while it is off. Gear, pulley,
  fixed, parallel, filter, generic and motor joints now refuse limits
  instead of ignoring them.
  ([#185](https://github.com/Rawframe-Project/maul3d/pull/185),
  [#187](https://github.com/Rawframe-Project/maul3d/pull/187))
- **Breaking: the motor joint has its own budgets and aim.** Its force
  and torque budgets are `m3JointDef.maxForce` and `maxTorque`, set at
  run time with `m3MotorJoint_SetMaxForce` and `m3MotorJoint_SetMaxTorque`,
  instead of `lowerLimit`, `upperLimit` and `m3Joint_SetLimits`.
  `m3Joint_SetMotorPose(offset, rotation)` becomes
  `m3MotorJoint_SetOffsets(linearOffset, angularOffset)`. `m3JointDef`
  grows from 216 to 224 bytes, and journal tapes use format 3, so tapes
  from earlier releases are refused. The C# `JointDef` gains `MaxForce`
  and `MaxTorque`.
  ([#187](https://github.com/Rawframe-Project/maul3d/pull/187))
- **Breaking: two switches are renamed.** `m3JointDef.collideConnected`
  becomes `enableConnectedCollision`, with
  `m3Joint_EnableConnectedCollision` and
  `m3Joint_IsConnectedCollisionEnabled` in place of
  `m3Joint_SetCollideConnected` and `m3Joint_GetCollideConnected`.
  `m3WorldDef.enableSleeping` becomes `enableSleep`, with
  `m3World_EnableSleep` and `m3World_IsSleepEnabled` in place of
  `m3World_EnableSleeping` and `m3World_IsSleepingEnabled`. The C#
  fields follow. The renames alone leave the def layouts and the
  snapshot unchanged.
  ([#182](https://github.com/Rawframe-Project/maul3d/pull/182))

### Fixed

- `m3Joint_SetMotor` wrote into the motor slot of any joint. On a gear it
  overwrote the bodies' reference spins, so meshed gears at rest turned
  to a new relation, and past a half turn the world's snapshot could no
  longer be restored. On a motor joint it moved the servo's aim. Only
  revolute, prismatic, wheel, generic and distance joints take a motor
  now, and the motor calls refuse every other type.
  ([#184](https://github.com/Rawframe-Project/maul3d/pull/184))

## [0.8.0](https://github.com/Rawframe-Project/maul3d/compare/v0.7.0...v0.8.0) (2026-10-28)

### Added

- **`m3WorldDef.pairCapacity` and the dropped pair count.** The pair
  table's size is now set by the def: 0 takes 8 pairs per shape, the
  old fixed size, and the bound is `M3_MAX_PAIR_CAPACITY`. When a step
  finds more overlapping pairs than the table holds, it keeps the pairs
  of the lowest keys, drops the rest for that step, and counts them in
  `m3Counters.pairsDropped`. A program that sees the count grow should
  raise `pairCapacity`.
  ([#167](https://github.com/Rawframe-Project/maul3d/pull/167))

### Changed

- **Breaking: `m3World_Step` returns `m3Result` and must not be
  discarded.** It returns `m3_errorInvalid` for a null world or a time
  step or substep count out of range, and `m3_errorBusy` from host code
  the world is running. A world now takes its whole step scratch at
  create, sized at its capacities, so a step never asks the allocator
  and can no longer stall for memory. `m3CreateWorld` needs about
  7.2 KiB more per shape slot than before, 14.5 MiB more for the default
  world, and refuses with `m3_errorCapacity` a world whose scratch would
  pass 2 GiB. `m3World_GetMemoryUsage` gives the exact figure. The C#
  binding's `Step` returns `Result`.
  ([#165](https://github.com/Rawframe-Project/maul3d/pull/165))
- **Breaking: a full pair table no longer stalls the step.** It drops
  the pairs of the highest keys for that step, as above, and the step
  returns `m3_success`. The snapshot format moves to 6: its header is
  60 bytes and carries the pair capacity, so snapshots from earlier
  releases are refused with `m3_errorVersion`.
  ([#167](https://github.com/Rawframe-Project/maul3d/pull/167))
- **Breaking: refusals return the family's format, limit and busy
  statuses.** `m3_errorFormat` (-8) is returned for malformed snapshot, journal tape and replay
  container bytes, and counts no misuse. `m3_errorLimit` (-9) is
  returned by a create, `m3CreateWorld`, `m3Character_SetStance` and
  `m3DestroyShape` for an argument past a published bound, and counts
  one misuse; a NaN stays `m3_errorInvalid`. `m3_errorBusy` (-10) is
  returned by every writer refused because a step, a query, a draw or
  a library callback is running on the world, and counts one misuse.
  Every fixed bound a call refuses is now a constant in
  `maul3d/base.h` or in the soft body and vehicle headers. Code that
  compared against `m3_errorState` or `m3_errorInvalid` for these
  cases must compare against the new statuses.
  ([#141](https://github.com/Rawframe-Project/maul3d/pull/141))
- **Breaking: the public counts are renamed.**
  `M3_SOFTBODY_MAX_PARTICLES`, `_EDGES`, `_ANCHORS` and `_TETS` become
  `M3_MAX_SOFTBODY_PARTICLES`, `_EDGES`, `_ANCHORS` and `_TETS`;
  `M3_SOFTBODY_MAX_PRESSURE` becomes `M3_SOFTBODY_PRESSURE_LIMIT`;
  `M3_VEHICLE_MAX_WHEELS` becomes `M3_MAX_VEHICLE_WHEELS`; and
  `M3_DRIVETRAIN_MAX_CURVE` and `_GEARS` become
  `M3_MAX_DRIVETRAIN_CURVE` and `_GEARS`.
  ([#141](https://github.com/Rawframe-Project/maul3d/pull/141))
- **Breaking: `m3JointDef` drops `genericReserved`** and shrinks from
  224 to 216 bytes, and `m3FragmentEvent` drops `pad`. Journal tapes
  use format 2, so tapes from earlier releases are refused.
  ([#139](https://github.com/Rawframe-Project/maul3d/pull/139))
- **Breaking: free bodies turn by a splitting of Euler's equations.**
  The implicit spin step damped tumbling bodies: a 0.5 by 1 by 1.5 m
  box lost up to 14% of its energy in a second at 20 rad/s. A free
  dynamic body now keeps its world angular momentum to rounding and its
  energy within a bounded error, and symmetric tops and spheres turn
  exactly. Every tumbling body's trajectory changes.
  ([#135](https://github.com/Rawframe-Project/maul3d/pull/135),
  [#140](https://github.com/Rawframe-Project/maul3d/pull/140))
- **Breaking: static mesh, height field and voxel geometry is
  bounded.** Mesh vertices and height samples must lie within
  `M3_GEOMETRY_LIMIT`, 1e8 m, of their body, and height field and
  voxel cells within `M3_CELL_LIMIT`, 1e5 m. Create and replay return
  `m3_errorLimit` past either bound, and restore refuses such content
  with `m3_errorFormat`. `M3_MAX_WATER_VOLUMES` moves to
  `maul3d/world.h`.
  ([#152](https://github.com/Rawframe-Project/maul3d/pull/152))
- **Breaking: a capsule's moment about its own axis is the closed
  form.** It was a difference of two large terms in float: a capsule of
  radius 0.5 m with a half length of 5000 m was refused below the
  inertia floor, and a 1000 m one spun about its axis with only float
  noise for a moment. Capsule trajectories change by rounding.
  ([#172](https://github.com/Rawframe-Project/maul3d/pull/172))
- **Breaking: soft particles in water take the drag implicitly.** A
  particle's water drag was an explicit step after its speed cap, so a
  large drag flipped and multiplied its velocity every substep until the
  lattice went NaN. It now takes the rigid body's implicit form and
  never passes the flow; soft bodies in water move slightly differently
  for every drag.
  ([#171](https://github.com/Rawframe-Project/maul3d/pull/171))
- Shape casts and continuous collision over a height field grid, a mesh
  or a voxel chunk try only the pieces within reach of their path, so a
  long diagonal path no longer pays for the whole surface: a cast
  skimming a 255 by 255 grid along its diagonal took 37.6 ms and now
  takes under 2 ms. No result changes.
  ([#153](https://github.com/Rawframe-Project/maul3d/pull/153))
- A BVH gather fills an 8 KiB bitmap instead of 128 KiB stack arrays,
  so the narrow phase keeps under 48 KiB on the stack and a thread that
  runs task ranges needs 64 KiB of stack. Long ray casts over meshes run
  about four times faster. No result changes.
  ([#166](https://github.com/Rawframe-Project/maul3d/pull/166))

### Fixed

- Shape casts, continuous collision and character moves took a height
  field grid for a point at its body's origin: casts over the grid
  missed it, a fast body passed through it and a character walking onto
  it fell through. They now see the grid's triangles, and a mover on a
  mesh, grid or voxel chunk gets the plane of its nearest piece. Casts
  also no longer stop after 64 pieces of a mesh or voxel chunk.
  ([#146](https://github.com/Rawframe-Project/maul3d/pull/146))
- A ray from far away at a sphere or a capsule lost its normal to
  cancellation: from 10 km it got a normal of (0, 1, 0), and from 1e10 m
  a NaN hit. Rays are now solved about their closest approach.
  ([#147](https://github.com/Rawframe-Project/maul3d/pull/147))
- A writer that the host's allocator called from inside a content
  create, a broadphase rebuild, a restore, a replay or a destroy ran
  and changed the world under the call, which read freed or moved
  memory. It now returns `m3_errorBusy`, changes nothing and counts one
  misuse.
  ([#160](https://github.com/Rawframe-Project/maul3d/pull/160))
- A hull create the allocator refused returned `m3_errorInvalid` and
  counted a misuse, as for a degenerate cloud. It now returns
  `m3_errorCapacity` and counts nothing.
  ([#161](https://github.com/Rawframe-Project/maul3d/pull/161))
- With task hooks, height field pairs on different threads shared one
  scratch allocator without synchronization, which raced and could
  crash. Each pair now builds its cell window in a buffer of its own.
  ([#164](https://github.com/Rawframe-Project/maul3d/pull/164))
- `m3Joint_SetTargetTranslation` on a wheel joint zeroed its steering's
  frequency, damping and budget, so the wheel stopped steering and its
  world's snapshot was refused by restore. The steering now keeps its
  values.
  ([#175](https://github.com/Rawframe-Project/maul3d/pull/175))
- A crafted snapshot with a gear spin near 1e30 restored, and the next
  step never returned: the gear's angle wrap stepped 2π at a time.
  Restore now refuses a gear spin outside a half turn, and the wrap runs
  in constant time.
  ([#177](https://github.com/Rawframe-Project/maul3d/pull/177))
- Restore took values no create or setter stores: a body sleep
  threshold of 0, a target rotation other than the identity on a joint
  that keeps it, an enabled limit with lower above upper, a negative
  cone angle or pulley rope constant, and a character slope cosine
  below that of 1.5 rad. It now refuses each with `m3_errorFormat`.
  ([#179](https://github.com/Rawframe-Project/maul3d/pull/179))

## [0.7.0](https://github.com/Rawframe-Project/maul3d/compare/v0.6.0...v0.7.0) (2026-10-27)

### Changed

- **Breaking: a joint whose bodies have a lever ratio past 1e10, the
  guide's joint lever limit, is refused.** The ratio is the sum of
  each body's largest turn, the top eigenvalue of [r]ᵀ I⁻¹ [r] in its
  own frame, over the sum of their inverse masses; static and
  kinematic bodies add nothing. It applies to the spherical, revolute,
  prismatic, fixed, wheel and motor joints, whose blocks undo each
  other well before the 1e15 the guide used to give; the distance,
  generic, parallel, gear, pulley and filter joints keep no such
  limit. Joint create returns `m3_errorInvalid` and counts one misuse,
  and so does a shape create on a jointed body. `m3DestroyShape`,
  `m3Shape_SetSphere`, `m3Shape_SetCapsule`, `m3Shape_SetDensity` and
  `m3Body_SetType` refuse a change that would carry a joint past the
  limit, change nothing and count one misuse. Journal replay and
  restore refuse it with `m3_errorInvalid`. No result changes within
  the limit.
  ([#118](https://github.com/Rawframe-Project/maul3d/pull/118))
- **Breaking: shape geometry is taken only in the form create leaves
  it.** Create, replay, the sphere and capsule setters and restore
  check every shape slot with one rule: a field the shape type does
  not use is zero, and a plane's normal is unit length.
  `m3CreatePlaneShape` refuses an offset past 1e15 m, and
  `m3CreateSoftBody` and `m3CreateSoftBodyTet` refuse a particle past
  1e15 m on any axis. The step now holds every body and soft particle
  within 1e15 m, the position limit every create keeps, rather than at
  1e18 m, and restore bounds positions there.
  ([#123](https://github.com/Rawframe-Project/maul3d/pull/123))
- **Breaking: a world's snapshot is bounded by `M3_MAX_SNAPSHOT_BYTES`
  (`INT32_MAX`) and sized in 64 bits.** `m3CreateWorld` refuses a def
  whose capacities could pass the bound with `m3_errorCapacity` before
  allocating anything; the guide gives the bytes per slot, near
  278,000 shape slots or 1,638 mesh slots with nothing else.
  `m3CreateHeightFieldGridShape` refuses a grid that would carry the
  snapshot past the bound with `m3_errorCapacity`, and replay refuses
  it with `m3_errorInvalid`. Before, the snapshot size was counted in
  32 bits and a world large enough wrapped it. No result changes.
  ([#126](https://github.com/Rawframe-Project/maul3d/pull/126))

### Fixed

- A soft particle whose collision query met more than 32 boxes of a
  voxel chunk or triangles of a mesh wrote past a 32-entry stack
  array. The gather now stops at 32 and keeps the same first 32 hits
  in the same order, so results are unchanged otherwise.
  ([#120](https://github.com/Rawframe-Project/maul3d/pull/120))

## [0.6.0](https://github.com/Rawframe-Project/maul3d/compare/v0.5.0...v0.6.0) (2026-10-27)

### Changed

- **Breaking: only the release that wrote a snapshot, a journal tape
  or a replay container reads it.** All three formats now start with
  the 16-byte state prefix shared with maul2d: a magic, the ABI
  version, the format number and the header size. The snapshot format
  is 5, the journal tape format is 1 (a tape had no header before) and
  the replay container format is 3, with the magic `M3RP` in place of
  `M3J1`. Each magic reads in file order, so a hex dump starts with
  `M3SN`, `M3JN` or `M3RP`. Bytes from another release, or a snapshot
  from another build, are refused with `m3_errorVersion` instead of
  `m3_errorInvalid`, and a container holding another release's
  snapshot or tape is refused the same way. `m3World_StartJournal`
  now needs a buffer of at least 16 bytes, and `m3World_ReplayJournal`
  refuses zero bytes as `m3_errorInvalid` instead of replaying them as
  an empty tape. `m3GetEncodedReplaySize` returns 0 for sizes that
  cannot be a snapshot and a tape, and for a container past
  `INT32_MAX` bytes. A host keeps assets, not state, across an
  upgrade; `docs/internals/state-formats.md` gives the layouts
  ([#109](https://github.com/Rawframe-Project/maul3d/pull/109),
  [#110](https://github.com/Rawframe-Project/maul3d/pull/110)).

### Added

- `m3Counters.heldPushRetakes` counts each push a character, a soft
  particle or a wheel lands that is taken again in double because the
  float result was not finite. Like `voxelEditsRefused`, it is a
  counter, not state: it is not hashed, snapshotted or journaled, and
  a restore leaves it as it is
  ([#111](https://github.com/Rawframe-Project/maul3d/pull/111)).

### Fixed

- The gyroscopic step no longer gains energy. One Newton iteration of
  the implicit step could overshoot near a singular Jacobian: 50
  boxes tumbling with the angular cap raised gained up to 1e5 times
  their rotational energy, and rods and plates gained energy at far
  smaller turns. A large change of spin is now redone in pieces of at
  most half a radian, any remaining gain is scaled away, and a step
  that leaves float range is retaken at unit inertia scale. In the
  issue's scenes no box now ends above its start energy by more than
  1e-6; implicit Euler's own damping is unchanged.
  Golden and bench hashes are unchanged; the whole bench costs about
  1.6% more instructions
  ([#107](https://github.com/Rawframe-Project/maul3d/pull/107)).

### Security

- Restore now bounds the state values that were checked only for
  being finite, each by the range that create, the setters or the step
  give it: soft-body rest lengths, tet rest volumes and closed rest
  volume, a wheel's compression, a character's slope limit and ground
  normal, joint targets by joint type and the ranges of limited
  generic axes, a body's sleep time and the gust phase. A value outside its range is refused with
  `m3_errorInvalid`, no misuse counted and the world left as it was.
  Every world the API builds still restores, and no golden line moves
  ([#114](https://github.com/Rawframe-Project/maul3d/pull/114),
  [GHSA-mg76-rhjf-fr34](https://github.com/Rawframe-Project/maul3d/security/advisories/GHSA-mg76-rhjf-fr34)).

## [0.5.0](https://github.com/Rawframe-Project/maul3d/compare/v0.4.0...v0.5.0) (2026-10-26)

### Changed

- **Breaking: every id pool keeps 32-bit generations, and the snapshot
  format is 4.** A slot retires at its last generation rather than
  wrapping, and with 16-bit generations a pool allowed only
  `capacity × 65536` creates over a world's life: a 1000-slot pool
  creating and destroying 1000 objects a second ran out after about 18
  hours. A pool of capacity n now makes n × 2^32 objects, about 136
  years in that case. All 11 pools change, the internal hull, mesh,
  height field and voxel pools included, and so do the generations
  stored for a character's ground body, a vehicle's chassis and soft
  body anchors. Public ids, the API and the ABI are unchanged. A
  format 3 snapshot, or a replay container that starts with one, is
  refused; take snapshots again with this release. Every golden hash
  except the journal tape's moves, because the world hash covers the
  wider generations
  ([#92](https://github.com/Rawframe-Project/maul3d/pull/92)).

### Fixed

- Joints keep their hold on very heavy bodies and long arms. A joint
  block whose float determinant underflowed to zero now takes the
  scaled solve, and the point block and the joint row pair are built
  and solved in double when float loses their mass term. A
  ball-and-socket or hinge on a 1000 m arm now holds its pivot within
  half a millimetre at every angle measured, 0° to 60°, where it fell
  freely or was flung 2 km. Hung cubes of 8e16 kg and more and welded
  cubes from 8e9 kg stay held, and rolling resistance works at
  4.2e12 kg. The guide states the limit that remains, `mass * arm^2 / moment` below about 1e15. A slider on
  a 1000 m arm holds again at 30° below its axis and above it, where it
  was flung hundreds of metres; it still sags about 2 m, and is still
  flung at 45° and 60° below its axis
  ([#96](https://github.com/Rawframe-Project/maul3d/issues/96)).
  Results of ordinary systems are unchanged bit for bit. Long-arm
  joints cost up to about 22% more instructions, and ordinary joint
  chains within 3%
  ([#97](https://github.com/Rawframe-Project/maul3d/pull/97)).

### Security

- The contact reduction's picks always stay within the candidate
  points, for every caller: plane against hull, hull faces, meshes and
  triangle clipping. The points kept are unchanged wherever the old
  picks stayed in range
  ([#103](https://github.com/Rawframe-Project/maul3d/pull/103),
  [GHSA-6vg5-vpc9-p7cw](https://github.com/Rawframe-Project/maul3d/security/advisories/GHSA-6vg5-vpc9-p7cw)).

## [0.4.0](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.4.0) (2026-10-25)

### Changed

- **Breaking: the API and replay refuse magnitudes past fixed limits.**
  Every value a call or a recorded op takes is bounded, so every product
  the step forms from them stays finite. A call past a limit refuses
  with `m3_errorInvalid` (a create returns the null id), counts one
  misuse, journals nothing and changes nothing, and replay refuses the
  same values through the same checks. Among the limits: speeds within
  1e5 m/s and rad/s per component (a kinematic body's linear speed
  within 1e10 m/s), gravity within 1e4 m/s^2, frequencies within 1e5 Hz,
  dimensionless ratios within 1e4, step lengths from 1e-4 to 10 s,
  forces and torques within 1e20, and on a dynamic body every
  mass-bearing shape at least 1e-6 kg with every principal moment at
  least 1e-12 kg m^2 and the body at most 1e20 kg. The guide's
  "Magnitude limits" chapter and the header docs list every limit per
  call. A body holding static-only scenery cannot leave static, and a
  teleport or a type change cancels a pending servo target
  ([#81](https://github.com/Rawframe-Project/maul3d/pull/81)).
- **Breaking: `m3Shape_SetDensity` always rebuilds the body's mass.** It
  takes no `updateBodyMass` argument, and its journal record shrinks
  from 16 to 12 bytes. A body's mass, center and inertia are now always
  derived from its type and shapes
  ([#82](https://github.com/Rawframe-Project/maul3d/pull/82)).
- **Breaking: the snapshot format is 3.** Mass, center, inertia and
  extents are rebuilt on restore rather than stored, bit for bit as the
  source world had them, and a format 2 snapshot is refused
  ([#82](https://github.com/Rawframe-Project/maul3d/pull/82)).
- On 32-bit x86 the library is built with SSE2 math
  (`-msse2 -mfpmath=sse`), so it needs a processor with SSE2, and every
  build now leaves trapping math at the compiler's default, from the
  family's compiler flags. On AVX2 the solver's min and max are one
  instruction each, with the same bits. Every golden scene and bench
  hash is unchanged on x64 and i686, with GCC and Clang
  ([#85](https://github.com/Rawframe-Project/maul3d/pull/85)).

### Fixed

- Restore accepts only what a world built through the API could hold.
  It refuses any value past the limit its create or setter keeps, a
  position past 1e18 m, a dynamic body whose shapes break the mass
  floors or limit, and a contact manifold with a float that is not
  finite. The step holds every body and soft particle within 1e18 m and
  a loaded warm start within a limit. Every ordinary impulse keeps its
  bits, and every golden scene's result is unchanged; only the snapshot
  and journal byte hashes moved with the formats
  ([#82](https://github.com/Rawframe-Project/maul3d/pull/82)).
- The step keeps every dynamic body, particle and joint row within
  1e5 m/s and rad/s after the warm start, restitution and the soft-body
  pass, and caps a joint row's correction speed there, so an error of
  any size closes no faster than a body may move. A body's inverse
  inertia is exact at any scale
  ([#76](https://github.com/Rawframe-Project/maul3d/pull/76)).
- The gyroscopic step and the solver's small inverses (the contact
  tangent pair, the joint row pair and the 3x3 solve) solve at unit
  scale when the direct result is not finite or the determinant is not
  a normal float, instead of dropping the term or giving zero. Every
  ordinary system goes through the same operations and gives the same
  bits, and every golden and bench hash is unchanged
  ([#79](https://github.com/Rawframe-Project/maul3d/pull/79),
  [#80](https://github.com/Rawframe-Project/maul3d/pull/80)).
- Replay applies each op's public checks: the sleep controls, waking a
  body, a hull's points, tank commands, joint targets and voxel edits
  are refused on replay exactly where their calls refuse them, and the
  journal reader checks each record's length as the replay reader does.
  A soft lattice def is refused, by the call and on replay, when any one
  of its counts passes the particle cap
  ([#86](https://github.com/Rawframe-Project/maul3d/pull/86)).

## [0.3.0](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.3.0) (2026-10-25)

### Changed

- **Breaking: a character owns its body and capsule.** Queries report a
  character's capsule like any shape, and `m3Shape_GetBody` gives its
  body, so a program could destroy, retype or reshape either one, or add
  a shape to the body, while the character lived on, which the character
  does not support. `m3DestroyShape`, `m3DestroyBody`, `m3Body_SetType`,
  `m3Shape_SetSphere`, `m3Shape_SetCapsule` and a shape create on the
  character's body now refuse them with `m3_errorInvalid`, count a
  misuse and change nothing; replay refuses the same ops in a tape.
  Destroy the character instead: that still takes its body and capsule
  with it, and `m3Character_SetStance` remains the way to resize it.
  Replay also refuses a body type the public call refuses (outside
  static, kinematic and dynamic); it used to set any byte.
  ([#73](https://github.com/Rawframe-Project/maul3d/pull/73))

### Fixed

- **Restore refuses a snapshot whose structures this engine could not
  have written.** It checked each snapshot block on its own
  (indices in range, flags, finite floats) but not the structures the
  blocks form together. It now also checks, before any byte lands: the
  id pools, the body to shape and joint lists, the broadphase tree, the
  contact pairs, shared content and its reference counts, and the
  records of characters, vehicles, soft bodies and joints. Such a
  snapshot is refused with `m3_errorInvalid` like every other restore
  refusal, and the world is left as it was. Nothing is rebuilt, so
  restored worlds still resimulate bit for bit, and every snapshot this
  engine writes still restores, also one taken between a call and the
  next step. The snapshot format is unchanged.
  ([#70](https://github.com/Rawframe-Project/maul3d/pull/70))
- **`m3World_TestPoint` reports a point far from a voxel chunk as
  outside it.** The cell of a point's chunk-local coordinates is now
  bounded like every other grid-cell cast, and the library's other
  float-to-int conversions were checked the same way. No result changes
  for any value in range, and no golden or benchmark hash moved.
  ([#71](https://github.com/Rawframe-Project/maul3d/pull/71))

## [0.2.1](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.2.1) (2026-10-24)

### Changed

- **The soft body pass skips the shapes no particle can reach.** Each
  substep it tested every particle of every soft body against every
  shape in the world. It now first tests the box around a soft body's
  particles against each shape: a bounding sphere for spheres, capsules,
  voxel chunks, meshes and height fields, and a separating face for
  hulls. Planes are always tested. Each test keeps a stated rounding
  margin, so a skipped pair is one the projection would leave alone. A
  bench scene of four cloths and two jellies among 2000 static shapes 40
  to 90 m away runs on 3.57 billion instructions instead of 282.1
  billion, 79 times fewer. The skip changes no result: hashes,
  snapshots and replays are the same with it as without it, and there is
  no API change. ([#47](https://github.com/Rawframe-Project/maul3d/pull/47))
- **The API reference lives on the documentation site.** The README and
  the Godot binding's README link
  https://rawframe-project.github.io/maul3d/api.html, which is built from
  the headers. The committed `docs/api.md`, which showed an older API, is
  gone. `m3World_Hash`'s doc now lists what the hash covers and what it
  leaves out: mesh geometry and height field samples are not hashed.
  `m3Counters.voxelEditsRefused` is documented as a counter that is
  neither hashed nor snapshotted.
  ([#46](https://github.com/Rawframe-Project/maul3d/pull/46))

### Fixed

- **Soft body particles collide with thin mesh triangles and with
  triangles that repeat a corner; they used to pass through.** On a
  triangle well under a millimetre wide, the closest point could come
  out up to a metre from a particle a few centimetres away, so the
  particle was not pushed. A triangle whose first two corners are one
  vertex gave no closest point at all. On other triangles a result
  changes only where the point lies within rounding of an edge, and no
  golden or bench result moved.
  ([#48](https://github.com/Rawframe-Project/maul3d/pull/48))

## [0.2.0](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.2.0) (2026-10-24)

### Added

- **32-bit x86 builds.** The `linux-i686-gcc-14` and
  `linux-i686-clang-20` presets build for i686 with
  `-m32 -msse2 -mfpmath=sse`, and CI runs both. A host's own i686 build
  needs those flags too: the library refuses to build with x87 float
  arithmetic. `base.h` adds `M3_ALIGN8`, which the public headers use to
  give 64-bit members 8-byte alignment.
  ([#41](https://github.com/Rawframe-Project/maul3d/pull/41),
  [#44](https://github.com/Rawframe-Project/maul3d/pull/44))

### Changed

- **Breaking, on 32-bit x86 only: the public ABI.** i686 hosts must
  rebuild against the new headers. The doubles and 64-bit integers of
  `m3Pos3`, `m3QueryFilter` and the body, character, soft body, vehicle
  and shape defs are now 8-byte aligned there, as on every other target.
  These public structs change size on i686:
  - `m3BodyDef`, 108 to 120 bytes
  - `m3CharacterDef`, 68 to 72
  - `m3ShapeDef`, 92 to 104
  - `m3VehicleDef`, 460 to 464
  - `m3WaterVolumeDef`, 76 to 80
  - `m3JointDef`, 216 to 224
  - `m3RayCastResult`, 52 to 56
  - `m3BodyMoveEvent`, 52 to 56
  - `m3FragmentEvent`, 68 to 72

  Every other public struct that holds an aligned member keeps its size
  and changes only its alignment, from 4 to 8. An i686 tape recorded
  with 0.1.x is refused with `m3_errorInvalid` when it holds one of the
  twelve journal records whose size changed: a body, character, vehicle,
  water volume or joint create, any shape create, a filter change, or a
  force or impulse at a point. Other i686 tapes, and every i686
  snapshot, still load. On x86-64, arm64, Windows and wasm nothing
  changes but tape padding (below): existing tapes and snapshots load as
  before.
  ([#44](https://github.com/Rawframe-Project/maul3d/pull/44))

### Fixed

- **Journal tapes have one layout on every target.** Before, a tape's
  records were laid out as the recording target laid out its structs,
  so a tape recorded on i686 was refused on x86-64 and the other way
  round. Every journal record and every type a snapshot walks now has
  its x86-64 layout on every target, and the build checks each size and
  member offset, so a target that would lay one out otherwise fails to
  compile. The formats are little-endian, and a big-endian target fails
  to build.
  ([#44](https://github.com/Rawframe-Project/maul3d/pull/44))
- **A tape's padding bytes are always zero, on every target.** A create
  copied the caller's def into its record whole, so the tape's padding
  bytes held whatever the host left in the def's padding. Records now
  copy each def member by member into zeroed storage. Replay never read
  that padding, so worlds are unchanged, but the bytes of a tape can
  change for a host that left anything there, and the same calls now
  write the same tape.
  ([#44](https://github.com/Rawframe-Project/maul3d/pull/44))
- **The misuse and busy counters are atomic on 32-bit x86.** Their
  64-bit atomics were 4-byte aligned on i686: clang could not make them
  lock-free and the build did not link, and under GCC they were not
  guaranteed atomic. Both counters are now 8-byte aligned on every
  target.
  ([#41](https://github.com/Rawframe-Project/maul3d/pull/41))
- **A 32-bit build no longer gets AVX2.** The AVX2 backend was chosen
  from the host's processor name, so an `-m32` build on an x86-64 host
  was built for AVX2. It is now chosen only for 64-bit builds.
  ([#41](https://github.com/Rawframe-Project/maul3d/pull/41))

## [0.1.1](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.1.1) (2026-10-24)

### Added

- **`m3Counters.voxelEditsRefused`.** A chunk's surface holds at most
  2048 merged boxes, the checkerboard's count. An edit whose surface
  would pass that bound changes nothing, journals nothing and counts no
  misuse. It counts here instead, and `m3VoxelChunk_ClearBox` returns -1.
  An explosion skips that chunk's carve and still applies its impulses.
  A chunk create past the bound is refused with `m3_errorCapacity`, a
  snapshot chunk past it with `m3_errorInvalid`, and a replayed edit past
  it refuses the replay, which leaves the world as it was. No grid
  measured so far reaches the bound. The field is a counter, not
  state: it is not hashed, and a restore or a replay's back-out leaves
  it as it is. A program that wants to know whether an edit landed can
  compare it before and after.
  ([#38](https://github.com/Rawframe-Project/maul3d/pull/38))

### Changed

- **Voxel memory is reserved per chunk slot.** A voxel slot takes about
  84 KB for its collision tree when its first chunk is created, and
  keeps it for later chunks in that slot. Every world also takes about
  88 KB at creation: a 76 KB build scratch and a 12.5 KB edit staging
  copy. `m3CreateVoxelChunkShape` returns `m3_errorCapacity` when the
  allocator refuses the slot's tree, with the world as it was. A restore
  takes the tree for each live chunk slot that has none yet, and returns
  `m3_errorCapacity` when refused. `m3MemoryUsage.contentBytes` counts
  the slot trees. A replay's up-front headroom no longer counts voxel
  surfaces, only arrays sized by the mesh, shape and voxel capacities.
  Budget 84 KB per voxel slot a program fills, on top of the
  world's 88 KB.
  ([#38](https://github.com/Rawframe-Project/maul3d/pull/38))
- A mesh shape's create takes a build scratch sized by its triangle
  count rather than for 65535 triangles, so it needs less temporary
  memory. Mesh trees and every hash stay the same.
  ([#38](https://github.com/Rawframe-Project/maul3d/pull/38))

### Fixed

- **The same bits in any host's floating-point setting.** Every public
  function that computes now runs in the library's own setting (round to
  nearest, subnormals kept, every exception masked) whatever setting the
  calling thread has, and gives the thread its setting back on return,
  sticky exception flags included on x86. Before, a host that ran its
  threads with flush-to-zero, denormals-are-zero or another rounding
  mode got different results from the same calls. Host code runs in the
  host's setting: the allocator, the task hooks and the debug draw
  callbacks. The pre-solve callback runs in the library's setting,
  since its answer decides which contacts the solver keeps. Each range
  a task hook runs enters the library's setting on whatever thread the
  host gives it. A program changes nothing.
  ([#35](https://github.com/Rawframe-Project/maul3d/pull/35))
- **Voxel edits no longer fail for memory.** A set, a clear, a box
  clear, an explosion's carve or the fracture a clear causes used to
  rebuild the chunk's collision tree with memory from the allocator.
  When the allocator refused, the edit still landed and was journaled,
  but the chunk stopped colliding and answering queries until a later
  edit or restore. A replay with memory then diverged from the
  recording. Edits now rebuild in storage reserved ahead of them and ask
  the allocator for nothing.
  ([#38](https://github.com/Rawframe-Project/maul3d/pull/38))

## [0.1.0](https://github.com/Rawframe-Project/maul3d/releases/tag/v0.1.0) (2026-10-24)

### Changed

- **Breaking: the family API shape.** `m3CreateWorld` returns
  `m3Result` and hands the world back through an out parameter, and
  every call takes the world first; readers take `const m3World*`.
  `m3WorldId`, `m3World_IsValid`, `m3Body_GetWorld`, `m3Shape_GetWorld`,
  `m3Joint_GetWorld`, `m3SetAllocator`, `m3SetAssertHandler` and
  `m3LastResult` are removed; the allocator is part of `m3WorldDef`,
  which begins with `internalValue, version` and loses `workerCount`.
  Every create and destroy returns `m3Result`, with the id in an out
  parameter (null on failure). A destroy of a stale id returns
  `m3_errorStale`, and a destroy of the null id returns `m3_errorInvalid`.
  Ids are `{index1, generation}`. Snapshots, restores, the journal calls
  and the replay container codec return `m3Result`, and
  `m3World_RebuildBroadphase` does too. Journal tapes recorded with
  0.0.1 no longer replay, and the replay container is version 2.
  ([#13](https://github.com/Rawframe-Project/maul3d/pull/13))
- **Breaking: hulls.** Hulls are built from welded points (points
  within 512 QuickHull tolerances of an earlier point, about 1e-3 of the
  summed extents) with planes worked in double precision, so a cloud of
  near-duplicate points now gives a valid hull. Hulls built from the
  same points can differ from 0.0.1's, so the hashes and trajectories of
  scenes with hulls change. Snapshots are format 2: `m3World_Restore`
  refuses format 1, and replay files that hold one, with
  `m3_errorInvalid`.
  ([#34](https://github.com/Rawframe-Project/maul3d/pull/34))
- **Breaking: the world hash** now covers every id pool's identity, a
  height field shape's grid, a body's gravity scale and dampings, a
  joint's limits, motor and collide-connected flag, and a mesh's painted
  materials, so state the step reads can no longer differ under equal
  hashes. Every world's hash changes.
  ([#22](https://github.com/Rawframe-Project/maul3d/pull/22))
- **Breaking: refusals.** Arguments the engine cannot simulate are now
  refused at creation with `m3_errorInvalid`, and they count as misuse.
  This covers coordinates, half extents and local positions beyond 1e4,
  densities above 1e6, non-finite joint limits and rope lengths, soft
  body pressure above `M3_SOFTBODY_MAX_PRESSURE`, and the other bounds
  the headers list. A stale id or a refused argument counts as misuse in
  every call. A writer called while `Step` or `Draw` runs changes
  nothing and counts as misuse.
  ([#14](https://github.com/Rawframe-Project/maul3d/pull/14),
  [#19](https://github.com/Rawframe-Project/maul3d/pull/19))
- **Breaking: what a restore or replay accepts.** `m3World_Restore`
  leaves the world unchanged and returns `m3_errorInvalid` for content
  the builders would not make (hull planes, mass and vertices; height
  field and mesh content), and `m3_errorCapacity` when the allocator
  refuses. Before, it could land such content or leave the world half
  restored. `m3World_ReplayJournal` refuses an op the public call would
  refuse. It returns `m3_errorCapacity` with the world as it was when
  memory runs out. To do that it reserves its memory up front, so a
  replay that fit a tight allocator before can now be refused before any
  op runs.
  ([#27](https://github.com/Rawframe-Project/maul3d/pull/27),
  [#29](https://github.com/Rawframe-Project/maul3d/pull/29),
  [#30](https://github.com/Rawframe-Project/maul3d/pull/30),
  [#33](https://github.com/Rawframe-Project/maul3d/pull/33))
- The library is built as C23 and needs GCC 14 or Clang 19 or newer.
  On Windows it is built with clang-cl. The public headers stay C17 and
  C++17, and MSVC's cl still compiles them. A shared library's
  SOVERSION is the whole version, and the CMake package accepts only its
  exact version, since before 1.0 any release may change the ABI.

### Fixed

- A mesh or voxel chunk whose tree cannot be built is refused with
  `m3_errorCapacity`, where before it was created and collided with
  nothing.
- An explosion applies to shapes in slot order, so the result no
  longer depends on the broadphase's internal order.
- The broadphase rebuild takes its scratch memory before recording
  itself, and records nothing when that fails.
- Two joints that can never both hold no longer risk turning the world
  to NaN. Every joint row holds its accumulated impulse within a bound
  no ordinary scene reaches.
- Replaying a damaged journal no longer reads a body def flag holding a
  byte other than 0 or 1, which was undefined behavior. A mesh's
  materials are recorded without an allocation.
- A damaged snapshot whose voxel chunk count disagrees with its
  occupancy bits is refused.
