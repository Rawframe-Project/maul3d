# Vehicles

## Vehicles

A vehicle (`m3CreateVehicle`) is the classic raycast car: the
chassis is a normal dynamic body you create and shape; wheels are
NOT bodies. Each wheel is a suspension ray cast from a
chassis-local anchor (the chassis never blocks its own rays), a
spring and damper impulse scaled by springHertz, springDampingRatio, and a per-wheel
share of the chassis mass, and a tire working in the contact
plane. All wheel impulses land at the top of the step, before the
solver, from the same pass-start velocities.

- Commands (`m3Vehicle_SetCommands`: throttle, steer, brake) are
  journaled STATE, clamped to range; non-finite values are hostile
  no-ops that never journal; a commanded car wakes. Replay and
  rollback reproduce a drive to the bit.
- The chassis-local +x axis is forward. Steerable wheels rotate
  their frame about the suspension axis by steer * maxSteerAngle.
- The tire: driven wheels push with a flat driveForce, brakes
  oppose rolling and never reverse it, and lateral slip dies at
  the solver's effective mass split across wheels. One friction
  circle per wheel clamps the tangent sum against tireGrip times
  the suspension load: what surrenders under a hard corner is the
  drift.
- Tires work in SURFACE-RELATIVE velocity: a car parked (brake on)
  on a moving platform rides it, and wheel impulses push back on
  dynamic ground (driving across a loose slab kicks the slab).
- Free-rolling wheels have no longitudinal grip by design; parking
  on a slope or a ferry takes the brake, like a real car.
- Destruction interplay: carving the deck from under a parked,
  even sleeping, car wakes it the same step (wheel rays count as
  presence for the voxel wake).
- Suspension compression, wheel contact, and spin angle read back
  per wheel for rendering; destroying the chassis destroys the
  vehicle.
- Tracks (`m3Vehicle_SetTankCommands`: left, right, brake): a
  drivetrain-free vehicle can drive its wheels per side instead,
  split by the chassis-local anchor z sign (+z is the right
  side). Opposite throttles pivot the hull in place; equal ones
  run straight. Engaging suspends the normal throttle path,
  `m3Vehicle_SetCommands` disengages, and a vehicle with an
  active drivetrain refuses. Journaled state like every command.
  Pivot turns want a stop first: at speed, the lateral tire kill
  is a strong yaw damper and the attempt reads as a rollover
  hazard, exactly like the real machine.
- The lean stabilizer (`m3VehicleDef.leanStabilization`, 0 = off):
  a two-wheeler is an inverted pendulum, so an opt-in controller
  rolls the chassis toward the lean angle the current speed and
  steer imply, critically damped, natural frequency sqrt(gain)
  rad/s. It rotates about the wheel hub line (a pure center roll
  would be vetoed by the tires' own lateral kill) and touches
  only the roll axis, so it cannot mint yaw or drive. With it a
  bike sprints and corners leaned in; without it the same bike is
  on its side before the turn starts.

## The drivetrain

The raycast vehicle's flat driveForce grows an engine when you
attach an m3DrivetrainDef (m3Vehicle_SetDrivetrain, journaled):
a torque curve of 2..8 pinned control points with deterministic
linear interpolation (only +,-,*,/ touch the numbers), 1..6
forward gear ratios plus reverse, a final drive, pinned shift
RPMs, and a clutch measured in steps of torque cut. The first
control point is the idle line (a standing car keeps launch
torque); past the last point the curve extends flat. Engine
speed derives from chassis forward speed through the driven
wheels' mean radius and the active ratio; crank torque splits
equally across driven wheels (an open differential) and the
friction circle still has the last word at each tire.

With a drivetrain attached, throttle drives forward gears only:
reverse is gear -1 (m3Vehicle_SetGear, journaled; 0 is
neutral) and takes throttle magnitude with direction from the
gear. Auto shift climbs above shiftUpRpm and drops below
shiftDownRpm, forward gears only, never while the clutch is
open (the thrash brake). Current gear, clutch countdown, and
the tachometer (m3Vehicle_GetGear, m3Vehicle_GetEngineRpm) are
state: snapshotted, hashed when attached, rolled back mid-shift
to the bit. Vehicles without a drivetrain keep the flat model
untouched.

Differentials: the def's `differentialMode` picks `m3_differentialOpen`
(the classic equal split), `m3_differentialLimitedSlip` or
`m3_differentialLocked`, and `differentialCoupling` sets
the coupling force per m/s of wheel-speed disparity. The
coupling reads each driven wheel's contact speed with one step
of lag (the drivetrain's own wheel-reading convention) and
squeezes the split toward the driven mean; limited clamps the
transfer to the wheel's own drive force. The couple is
HOST-TUNED TO MASS SCALE: a 300 kg buggy locks solid near 20000
and oscillates at absurd values, so tune against your own car,
not a constant from this manual. All of it lives inside the
friction circle, so the effect is emergent grip management: a
locked diff resists yaw, a soft limited one LIVENS rotation by
feeding the unloaded wheel less.
