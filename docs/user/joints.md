# Joints

## Joints

Twelve types, every one built from the same proven row blocks
(point, axial, rotation-vector) so a new joint is a recipe, not
new math. Each type has its own def, holding only what that type
reads, its default and its create: `m3RevoluteJointDef`,
`m3DefaultRevoluteJointDef` and `m3CreateRevoluteJoint`, and so on.
A create refuses a def with another type's cookie.

- `m3_sphericalJoint`: a shared point, plus an optional swing cone
  (`enableCone`, `coneAngle`) and twist limit pair (`lowerTwist`,
  `upperTwist`). The cone is the spherical joint's alone: no other
  type's def holds it.
  `m3SphericalJoint_EnableCone` and `m3SphericalJoint_SetConeAngle`
  change it at run time; the angle is never negative, with the cone
  on or off.
- `m3_revoluteJoint`: hinge; angle limits (`lowerAngle`,
  `upperAngle`) and a motor (`motorSpeed`, `maxMotorEffort`).
- `m3_prismaticJoint`: slider; translation limits
  (`lowerTranslation`, `upperTranslation`) and a motor.
- `m3_weldJoint`: the relative pose at create time IS the
  weld pose; its def holds only the bodies and anchors.
- `m3_distanceJoint`: rope or rod along the anchor axis, held
  within `minLength` and `maxLength`. Equal lengths make a rod.
  The joint spring (`enableSpring`, `springHertz`,
  `springDampingRatio`) pulls toward `length`, which need not lie
  in the range: the range holds whatever the spring pulls toward.
  `m3DistanceJoint_SetLengthRange` and `m3DistanceJoint_SetLength`
  change them at run time.
- `m3_genericJoint`: per-axis `m3AxisMode` (locked, free, limited)
  over three linear and three angular axes, plus one motor on any
  movable axis, `motorAxis`, which drives while its switch is on,
  `enableMotor` in the def or `m3Joint_EnableMotor` at run time, as
  on every type with a motor. The v1 angular contract: at most one limited
  angular axis, and its neighbors both locked or both free.
- `m3_wheelJoint`: suspension slide plus free axle spin, its own
  chapter below.
- `m3_filterJoint`: no rows at all, BY LAW. Its whole effect is
  the connected-pair collision filter: ragdoll limbs stop
  grinding without buying a single constraint.
- `m3_parallelJoint`: keeps `localAxisA` on body A parallel to
  `localAxisB` on body B (two angular rows); every translation
  and the twist about the shared axis stay free.
- `m3_motorJoint`: the servo weld, its own chapter below.
- `m3_gearJoint` and `m3_pulleyJoint`: transmissions, their own
  chapter below.

Anchors are body-local; the joint frame's axis is local z. Joint
impulses persist in the snapshot (warm starts are simulation
state). Creation validates everything and returns the null id on
any violation; destroying a body cascades through its joints.

## Joint runtime control, drives, and breakage

Limits, the motor and the spring each have a switch and value
setters, and rebind mid-run: `m3Joint_EnableLimit` with
`m3Joint_SetLimits(lower, upper)`, `m3Joint_EnableMotor` with
`m3Joint_SetMotorSpeed` and `m3Joint_SetMaxMotorEffort`, and
`m3Joint_EnableSpring` with `m3Joint_SetSpringHertz` and
`m3Joint_SetSpringDampingRatio`. `IsLimitEnabled`,
`IsMotorEnabled`, `IsSpringEnabled` and a getter per value read
them back. A def can switch the spring on at create through
`enableSpring`, `springHertz` and `springDampingRatio`. A part keeps its values while it is off, and each
call zeroes the stored impulse of the rows it changes. A limit
range is ordered when it is set and whenever the limit is on; a
spring is positive in hertz while it is on. Only revolute,
prismatic, wheel and generic joints take a motor, only revolute,
prismatic, spherical, wheel, motor and distance joints a spring, and
only revolute, prismatic, spherical and wheel joints take limits;
the calls refuse every other type, the motor joint and the distance
joint included, whose budgets and length range have calls of their
own.
`m3Joint_EnableConnectedCollision` lets the two jointed bodies collide.
The spring arms the position drive on revolute (target angle),
prismatic and wheel (target translation) and spherical (target
rotation) joints; `m3RevoluteJoint_SetTargetAngle`,
`m3Joint_SetTargetTranslation` and
`m3SphericalJoint_SetTargetRotation` move the goal. The rotation target lives in the JOINT FRAMES
(frame z is the create-time local axis). Reaction readback:
`m3Joint_GetReactionForce` and `m3Joint_GetReactionTorque` return last-step
magnitudes assembled per type; they read 0 until the first step
after a restore. Breakage: `m3Joint_SetBreakLimits` arms
force/torque caps (zero = off); an over-threshold reaction
destroys the joint at the end of the step and emits the break
event. Breakage is an in-step deterministic transition on
purpose: a host poll would race the journal under rollback.

The four-state drive law: a velocity motor alone brakes, a
position spring alone holds, and TOGETHER they share ONE budget,
the motor's `maxMotorEffort`. A starved budget makes the drive
sag instead of borrowing force from nowhere; a zero
budget with the motor off leaves the spring unbudgeted. This is
the difference between a servo that stalls under load and one
that lies.

## The wheel joint

m3_wheelJoint is the OPTIONAL rigid-wheel path: the wheel is a
real body on real contacts (trucks over rubble, wheels in
fragments), while the raycast vehicle stays the default.
localAxisA is the suspension axis in the chassis frame,
localSpinAxisB the axle in the wheel frame; the axle is captured at
create and snapped exactly perpendicular to the strut (more
than a small skew refuses loudly). One joint composes both
halves: the wheel slides along the strut (travel limits from
enableLimit with lowerTranslation and upperTranslation,
suspension spring from m3Joint_EnableSpring plus
m3Joint_SetTargetTranslation) and spins freely about its axle
(drive with m3Joint_EnableMotor; read the spin with
m3Joint_GetAngle and the travel with m3Joint_GetTranslation).

Breakage works as for every joint: the force cap snaps a
wheel torn sideways, the torque cap a drive axle overdriven,
in-step, evented, and deterministic through rollback. The def
grew nothing and there is no new state: a wheel joint is
ordinary joint state and rides every snapshot and hash law that
already existed.

Steering: `m3WheelJoint_EnableSteering` switches a soft drive
that yaws the wheel about its strut toward
`m3WheelJoint_SetTargetSteeringAngle` (radians, at most 1.0 either
way), with its own `SetSteeringHertz`, `SetSteeringDampingRatio`
and `SetMaxSteeringTorque` (0 = uncapped). The values hold while
steering is off, so set the frequency before enabling it.
`m3WheelJoint_GetSteeringAngle` reads the live angle back. Steer the front pair of a wheel-joint cart and you
have rack-and-pinion steering on real contacts, no vehicle
required. Journaled, snapshotted, bit-exact through rollback.

## The servo weld

`m3_motorJoint` drives body B toward a target pose relative to
body A with NO hard rows: a soft 3-DOF rotation drive plus a
soft 3-DOF translation drive, each clamped to its own budget.
Tune stiffness with the joint spring (shared by both drives),
aim with `m3MotorJoint_SetOffsets(linearOffset, angularOffset)`
(the offset in A's create-time frame, the rotation relative to
the create pose), and budget with the def's `maxForce` and
`maxTorque` or `m3MotorJoint_SetMaxForce` and
`m3MotorJoint_SetMaxTorque`, 0 = uncapped. The getters read the
aim and the budgets back. A fresh servo aims at
its own create pose; without a spring it holds NOTHING and the
bodies drift free. Use it for animated platforms, grabbers, and
kinematic-feeling props that still lose to a bigger
force: a starved servo sags, it does not teleport.

## Gears and pulleys

`m3_gearJoint` couples spin so `spinA + ratio * spinB` keeps its
create value, where each spin is that body's rotation about its
own `localAxis`. Positive ratio is the external mesh (counter-
rotation); negative couples same-direction like a belt. The
mounting contract, documented and unchecked: both bodies should
be hinged on a common rigid frame (revolute or wheel joints to
the same chassis); the gear holds no other degree of freedom and
puts no reaction on that frame. Angle drift is corrected softly,
so a mesh cannot creep under load.

`m3_pulleyJoint` runs a rope from `localAnchorA` over the WORLD
point `groundAnchorA`, across to `groundAnchorB`, down to
`localAnchorB`: `length1 + ratio * length2` keeps its create
value, `ratio > 0`. The rope is RIGID both ways by contract (it
can push); pair it with a distance-joint rope when slack
matters. Ratio 2 is a block and tackle: B moves half as fast and
carries twice the force. Both types create through the same def
(`ratio`, `groundAnchorA/B` at the tail), journal, snapshot, and
hash like every other joint.
