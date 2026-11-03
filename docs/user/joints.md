# Joints

A joint constrains how two bodies move against each other. Each type
has its own def, from its own `m3Default...JointDef`, and its own
create; a def of another type is refused. Anchors and axes are given in
each body's frame. By default the two bodies of a joint do not collide
with each other; `enableConnectedCollision` lets them.

## A door on a hinge

```c
m3RevoluteJointDef def = m3DefaultRevoluteJointDef();
// The door frame is a static body at the origin, the door a dynamic
// body 0.5 m along x; the hinge is given in each body's frame.
def.bodyIdA = frame;
def.bodyIdB = door;
def.localAnchorA = (m3Vec3){0.0f, 1.1f, 0.0f};
def.localAnchorB = (m3Vec3){-0.5f, 0.0f, 0.0f};
def.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f}; // a vertical hinge
def.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
def.enableMotor = true;
def.motorSpeed = 1.0f;       // rad/s
def.maxMotorEffort = 500.0f; // N m
def.enableLimit = true;
def.lowerAngle = 0.0f;
def.upperAngle = 1.5f; // it opens to about 86 degrees
m3JointId hinge;
m3Result result = m3CreateRevoluteJoint(world, &def, &hinge);
```

## The types

| Type | Holds | Parts |
|---|---|---|
| Spherical | Two anchors together, a ball joint. | Twist limit about the A frame's axis, a swing cone around it, a spring toward a target rotation. |
| Revolute | One rotation axis, a hinge. | Limit, motor, spring toward a target angle. |
| Prismatic | One translation axis, a slider. | Limit, motor, spring toward a target translation. |
| Weld | Everything, at the pose the bodies had when it was created. | |
| Distance | The anchors' distance within `[minLength, maxLength]`: a rod when the two are equal, a rope when they differ. | A spring toward `length`, which the range still bounds. |
| Generic | Each of six axes of the joint frame locked, free or limited. | One motor on a free or limited axis. At most one angular axis may be limited, and the other two must then be both locked or both free. |
| Wheel | Body B slides along a suspension axis of A and spins freely about its own axle. | Suspension limit and spring, a motor on the spin, and steering. |
| Filter | Nothing; it only stops its two bodies colliding. | |
| Parallel | An axis of A parallel to an axis of B; translations and the twist about the axis stay free. | |
| Motor | B driven toward a target pose relative to A, a servo weld. | Force and torque budgets; the spring sets the stiffness. |
| Gear | `spinA + ratio * spinB` constant: a positive ratio counter-rotates, a negative one turns the same way. | Both bodies must be hinged to one common frame. |
| Pulley | A rope from each body over a fixed world point: `length1 + ratio * length2` constant. | The rope is rigid both ways. |

A wheel joint's axle is set exactly perpendicular to its suspension
axis when it is created; one more than slightly skewed is refused. The
wheel joint is the rigid-wheel path for vehicles; the raycast
[vehicles](vehicles.md) are the default.

## Limits, motors and springs

The setters change a joint's parts while the game runs:
`m3Joint_EnableLimit` and `m3Joint_SetLimits`, `m3Joint_EnableMotor`,
`m3Joint_SetMotorSpeed` and `m3Joint_SetMaxMotorEffort`,
`m3Joint_EnableSpring`, `m3Joint_SetSpringHertz` and
`m3Joint_SetSpringDampingRatio`. A motor runs at its speed within its
effort budget, a torque for an angular motor and a force for a linear
one. A spring pulls toward its target with a frequency up to 1e5 Hz and
a damping ratio up to 1e4: `m3RevoluteJoint_SetTargetAngle` (in
[-pi, pi]), `m3Joint_SetTargetTranslation` and
`m3SphericalJoint_SetTargetRotation`. A setter for a part the joint's
type does not have changes nothing and counts one misuse; a getter for
it returns 0. Changing a part wakes both bodies.

- `m3MotorJoint_SetOffsets` aims a motor joint, and
  `m3MotorJoint_SetMaxForce` and `m3MotorJoint_SetMaxTorque` set its
  budgets, 0 for no cap. A motor joint without a spring holds nothing.
- `m3DistanceJoint_SetLengthRange` and `m3DistanceJoint_SetLength` change
  a distance joint's range and rest length.
- `m3WheelJoint_EnableSteering` and the other `m3WheelJoint_` setters
  steer a wheel joint toward a target angle with a stiffness, a damping
  and a torque budget of their own.
- `m3SphericalJoint_EnableCone` and `m3SphericalJoint_SetConeAngle`
  shape a ball joint's swing.

`m3Joint_GetAngle` and `m3Joint_GetTranslation` read a joint's current
angle and slide, the spin and the suspension travel for a wheel joint.

## Breaking

`m3Joint_SetBreakLimits` gives a joint a force cap and a torque cap.
When either reaction at the end of a step passes its cap, the joint
destroys itself inside that step, and `m3World_GetJointEvents` reports
it. Because the break happens in the step, it replays and rolls back
like everything else. A cap of 0 is no cap; both 0, the default, make
a joint unbreakable. `m3Joint_GetReactionForce` and
`m3Joint_GetReactionTorque` read the size of the last step's reaction,
and read 0 until the first step after a restore.

## The lever limit

A joint holding a heavy body by a long arm, or a light one by a short
arm against a heavy one, can ask more of the solver than float
arithmetic can give. Creates and changes that would carry a joint past
`M3_JOINT_LEVER_LIMIT` are refused with `m3_errorLimit`, which counts
no misuse since the bodies' masses decide it; [the magnitude
limits](limits.md#magnitude-limits) explain the ratio.

## The joint's life

`m3DestroyJoint` removes a joint, and destroying either body removes
it too. `m3Joint_GetType`, `m3Joint_GetBodyA` and `m3Joint_GetBodyB`
read a joint back, and `m3Body_GetJoints` lists a body's joints.
Creating, changing and destroying joints are journal operations, and a
joint's settings and stored impulses are part of a snapshot and the
world hash.
