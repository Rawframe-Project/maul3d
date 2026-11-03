# The character controller

A character is a capsule that walks: it slides along walls, climbs
stairs, stays on the floor going down slopes, rides moving platforms
and shoves light bodies out of its way. The engine owns its body, a
kinematic capsule, and moves it only when the host calls
`m3Character_Move`. The step never moves a character, except to carry
it with the floor it stands on.

## Creating and moving a character

```c
m3CharacterDef def = m3DefaultCharacterDef();
def.position = (m3Pos3){0.0, 1.0, 0.0}; // the capsule's centre
m3CharacterId hero;
m3Result result = m3CreateCharacter(world, &def, &hero);

float fallSpeed = 0.0f; // the host's own vertical speed, m/s
for (int32_t tick = 0; tick < 120; ++tick)
{
    float dt = 1.0f / 60.0f;
    fallSpeed -= 9.81f * dt; // gravity is the host's job
    m3Vec3 move = {2.0f * dt, fallSpeed * dt, 0.0f};
    m3Character_Move(world, hero, move);
    if (m3Character_IsGrounded(world, hero))
    {
        fallSpeed = 0.0f; // standing: no fall carries over
    }
    (void)m3World_Step(world, dt, 4);
}
```

A move is a displacement in metres, not a velocity, and it happens
at once, inside the call. World gravity does not touch a character:
the host adds its own fall to each move, as above. Keeping a little
downward motion in every move, even while standing, is what keeps a
walker glued to the floor down a slope or a stair.

The def's fields, with their defaults:

| Field | Default | Meaning |
|---|---|---|
| `position` | the origin | The capsule's centre at spawn. |
| `radius` | 0.4 m | The capsule's radius. |
| `halfHeight` | 0.5 m | Half the length of the capsule's segment; the full height is twice `halfHeight + radius`. |
| `maxSlopeAngle` | 0.8 rad | The steepest ground, from horizontal, that counts as floor. |
| `snapDistance` | 0.3 m | How far down a descending character reaches for the floor. |
| `skin` | 0.02 m | The gap the character keeps from every surface. |
| `stepHeight` | 0.35 m | The tallest riser it climbs in one move. |
| `mass` | 80 kg | What its pushes weigh. |
| `pushMaxMassRatio` | 2 | The heaviest body it pushes, as a multiple of `mass`; 0 never pushes. |
| `userData` | 0 | The host's own value. |

`m3CreateCharacter` refuses a def that `m3DefaultCharacterDef` did
not make, a slope outside (0, 1.5] rad, a negative snap distance or
step height, a skin, radius or mass that is not positive, a negative
half height or push ratio, or a NaN, with `m3_errorInvalid`. A
position past 1e15 m, a radius or half height past 1e4 m, a mass
outside 1e-6 to 1e20 kg, or a push ratio past 1e4 is refused with
`m3_errorLimit`. `m3DestroyCharacter` removes the character with its
body and capsule.

The capsule is a shape like any other for queries, and dynamic bodies
collide with it as with any kinematic body. It belongs to the
character, though: `m3DestroyShape`, `m3DestroyBody`, `m3Body_SetType`,
`m3Shape_SetSphere`, `m3Shape_SetCapsule` and a new shape on its body
are refused with `m3_errorInvalid`. Its rotation is always the
identity; which way the character faces is for the host to draw.

## What a move does

- **Collide and slide.** The move casts the capsule along the
  displacement, stops one skin short of the first surface it meets,
  and slides what is left along that surface, up to four times.
- **Floors and walls.** A surface whose normal is within
  `maxSlopeAngle` of straight up, +y, is floor; anything steeper is a
  wall. Unless the move itself asks to rise, a slide along a wall
  never turns sideways intent into upward motion, so a character
  cannot creep up a cliff.
- **Stairs.** A grounded character whose horizontal move is blocked
  by a wall tries to lift by up to `stepHeight`, move forward and
  land. The landing counts only when a ray, cast down half a radius
  ahead of the capsule's axis, finds floor, and the rise fits one
  step height. Otherwise the riser is a wall like any other.
- **Snapping down.** After a descending move, or a move that ended on
  floor, the character looks down by `snapDistance` and settles onto
  any floor it finds there. With none in reach, it is airborne.
- **Pushing.** A move blocked by a dynamic body that is not floor and
  weighs at most `pushMaxMassRatio` times `mass` shoves that body
  with an impulse of `mass` times the distance the move lost against
  it, counted into its surface. The impulse acts at the contact point
  and wakes the body. A
  character pushing at a steady speed therefore delivers its mass
  times that speed every second. A heavier body is a wall.

`m3Character_IsGrounded` says whether the character stands on floor,
`m3Character_GetGroundNormal` gives that floor's normal (zero in the
air), and `m3Character_GetGroundBody` the body it stands on.
`m3Character_GetPosition` gives the capsule's centre.

A move that is NaN or infinite, or larger than 1e18 m on an axis, does
nothing, and is not recorded. Neither is a move with an id that names
no character. Each counts one misuse.

## Riding platforms

At the end of every step, each grounded character is carried by the
motion its ground body made during that step: a kinematic platform,
an elevator, a turning carousel or a dynamic body all carry their
riders. The carry is itself a move, so a platform that ferries a
walker into a wall leaves the walker at the wall. A character whose
ground body is destroyed is carried no more, and
`m3Character_GetGroundBody` returns the null id.

When a voxel edit removes the floor under a grounded character, the
character becomes airborne in that same call, not on the next move.

## Crouching: stance

`m3Character_SetStance` gives the capsule a new half height and
radius, keeping its feet where they were: the centre moves so the
bottom of the capsule stays level. `m3Character_GetStance` reads the
current pair.

Shrinking always succeeds. Growing first checks the room: the larger
capsule is cast at its new pose, one skin upward, and any contact (a
ceiling over a crouching character, a wall next to a wider one)
refuses the change with `m3_errorState`, leaving the stance as it was.
That refusal is an ordinary outcome and counts no misuse. A NaN
dimension is refused with `m3_errorInvalid`, and one outside
`M3_MIN_STANCE` (0.05 m) to `M3_STANCE_LIMIT` (10 m) with
`m3_errorLimit`.

## Rollback

Creating, destroying, moving and resizing a character are journal
operations, recorded and replayed like any other, and a refused move
or stance change is not recorded. The character's capsule, pose,
settings, stance and ground are all part of a snapshot and of the
world hash, so rollback and replay bring a character back exactly.

## A character with real mass

The controller above is kinematic: it pushes bodies but is never
pushed back. A character that should be knocked around is better built
from a dynamic body on the public API:

- a capsule on a dynamic body, with `m3Body_SetMotionLocks` locking
  all three angular axes, so it never tips over;
- a friction of 0 on its shape: a capsule driven by forces against
  static friction stands still, so the drive does the braking too;
- a force each tick toward the wanted velocity, clamped to what the
  character's legs can deliver;
- grounding read from `m3World_CollideMover`, which lists the
  surfaces around a capsule. Skip the planes on the character's own
  shape, found with `m3Shape_GetBody`, and look for one facing up.

`m3World_CastMover`, `m3SolveMover` and `m3ClipMoverVelocity` answer
the rest of a custom mover's questions without changing the world.
The `test_mover` suite builds exactly this walker.
