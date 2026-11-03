# Vehicles

A vehicle is a dynamic body, the chassis, carried by up to eight
wheels. A wheel is not a body: it is a ray cast from the chassis along
its suspension, with a spring and damper where the ray meets the
ground and a tire that grips in the ground's plane. The same model
covers cars, trucks, skid-steer tanks and, with its lean stabilizer,
two-wheelers.

## A car

```c
m3VehicleDef def = m3DefaultVehicleDef();
def.chassisId = chassis; // a dynamic body, forward along its local +x
def.wheelCount = 4;
const float wheelX[4] = {1.2f, 1.2f, -1.2f, -1.2f};
const float wheelZ[4] = {-0.8f, 0.8f, -0.8f, 0.8f};
for (int32_t w = 0; w < 4; ++w)
{
    def.wheels[w].anchor = (m3Vec3){wheelX[w], -0.2f, wheelZ[w]};
    def.wheels[w].steerable = w < 2; // the front pair steers
    def.wheels[w].driven = w >= 2;   // the rear pair drives
}
m3VehicleId car;
m3Result result = m3CreateVehicle(world, &def, &car);
m3Vehicle_SetCommands(world, car, 1.0f, 0.0f, 0.0f); // full throttle, straight
```

The chassis's local +x is forward and +z is its right side. Every
wheel slot of the default def hangs straight down, chassis-local -y,
with a 0.4 m rest length, 0.25 m of travel, a 1.5 Hz spring with a
damping ratio of 0.6, a 0.3 m radius and a quarter of the brake. The
vehicle defaults are a 0.6 rad steering lock, 800 N of drive per
driven wheel, 1600 N of brake, a tire grip of 1.5 and no lean
stabilization.

| Wheel field | Meaning |
|---|---|
| `anchor` | Where the suspension attaches, in the chassis frame. |
| `direction` | The suspension's direction in the chassis frame, within 10 percent of unit length; usually down. |
| `restLength` | The suspension's length with no load. |
| `travel` | How far it compresses from rest, at most `restLength`. |
| `springHertz`, `springDampingRatio` | The spring's frequency and damping. |
| `radius` | The wheel's radius: the ray reaches `restLength + radius`. |
| `steerable`, `driven` | Whether it turns with the steering and takes the drive. |
| `brakeShare` | Its share of `brakeForce`. |

A vehicle takes 1 to `M3_MAX_VEHICLE_WHEELS` (8) wheels. The create
refuses a chassis that is not a live dynamic body, and each bad field,
with `m3_errorInvalid`, and a value past its bound with
`m3_errorLimit`; the header lists every bound. Destroying the chassis
body destroys its vehicle, and `m3DestroyVehicle` removes the vehicle
and leaves the chassis.

## What a step does

Once per step, before the broad phase, each vehicle whose chassis is
awake runs its wheels. All of a vehicle's wheels read the chassis's
velocity as the step found it, and their impulses land together, so
no wheel sees another's push.

- **Suspension.** The ray finds the ground. The spring pushes along
  the suspension with a stiffness and damping set by the wheel's
  frequency and damping ratio, on its share of the chassis's mass: a
  quarter on a four-wheeler. It pushes and never pulls, and its
  compression stops at `travel`: past that, the spring pushes no
  harder.
- **Tire.** In the ground's plane, the wheel applies its drive, its
  brake and a sideways force that cancels its share of the sideways
  slip. All three together are held inside a friction circle of
  `tireGrip` times the wheel's suspension load. What the circle cuts
  off is the skid. A brake slows rolling and never reverses it.
- **Moving ground.** The tire works against the ground's own motion,
  so a car parked on a moving platform rides it. A wheel on a dynamic
  body pushes that body back with the same impulse.

The impulses act at the wheel's hub, not at the ground point. The
step then solves contacts with the chassis already sprung.

`m3Vehicle_GetWheelCompression` and `m3Vehicle_IsWheelGrounded` read
each wheel's last result, and `m3Vehicle_GetWheelSpin` its
accumulated rolling angle, to turn a wheel mesh. A sleeping chassis
runs no wheels, and its readings hold still.

## Driving

`m3Vehicle_SetCommands` sets the throttle and the steering, each
clamped to [-1, 1], and the brake, clamped to [0, 1]. They are state:
they stay until the next call, and journal, snapshot and rollback
carry them. The flat model gives each driven wheel `throttle` times
`driveForce`. Steering turns each steerable wheel about its suspension
axis by `steer` times `maxSteerAngle`. A command wakes the chassis.

`m3Vehicle_SetTankCommands` drives each side on its own, for skid
steering: driven wheels whose anchor has a z of 0 or more are the
right side, the others the left. It takes a throttle per side and one
brake, and holds until `m3Vehicle_SetCommands` is called again. A
vehicle with a drivetrain refuses it.

A NaN or infinite command is refused: it changes nothing, is not
recorded and counts one misuse.

## The drivetrain

`m3Vehicle_SetDrivetrain` replaces the flat drive force with an
engine and a gearbox:

- **Engine speed** comes from the chassis's forward speed through the
  driven wheels' mean radius, the gear's ratio and the final drive.
  It never reads below the first point of the torque curve, the idle
  speed, so a car at a standstill still has torque to pull away, and
  never above the last.
- **Torque** comes from the curve, 2 to 8 points of engine speed and
  torque, read linearly between points. It goes through the gear and
  the final drive and is shared equally by the driven wheels, then
  meets the tire's friction circle like any drive.
- **Gears.** 1 to 6 forward gears, reverse as gear -1, and neutral as
  gear 0. `m3Vehicle_SetGear` selects one; reverse needs a non-zero
  `reverseRatio`. With a drivetrain, the throttle only drives, and
  reverse is the gear's job. With `enableAutoShift`, the gearbox
  shifts up above `shiftUpRpm` and down below `shiftDownRpm`, forward
  gears only.
- **Clutch.** Each shift cuts the drive for `clutchSteps` steps, and
  the automatic gearbox does not shift again until it closes.
- **Differential.** Open shares the drive equally. Limited slip and
  locked pull each driven wheel's speed toward the driven wheels' mean
  speed of the previous step, with `differentialCoupling` newtons per
  m/s of difference; limited slip holds that pull within the
  engine's own force per wheel.

The default drivetrain is a small car's: a curve from 120 N m at
1000 rpm to 200 N m at 3000 rpm, five gears from 3.5 to 0.9, a 3.2
reverse, a 3.7 final drive, shifts at 5500 and 2000 rpm with a
12-step clutch, and an open differential. It starts in first gear
with the clutch closed. `m3Vehicle_GetGear` and
`m3Vehicle_GetEngineRpm` read the gear and the engine speed. A bad
drivetrain def changes nothing and counts one misuse.

## Two-wheelers

A bike on two wheels is an inverted pendulum and falls over. A
positive `leanStabilization` gain rolls the chassis toward the lean
that its speed and steering call for, critically damped, at a natural
frequency of the gain's square root, in radians per second. It acts
only about the forward axis, so it adds no yaw or pitch, and it rolls
the bike about the line of its wheels, as a real bike leans.

## Rollback

Creating and destroying vehicles, commands, drivetrains and gears are
journal operations, and a vehicle's settings, commands, wheel state,
gear, clutch and engine speed are part of a snapshot and the world
hash.
