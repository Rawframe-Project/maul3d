# Bindings

- [The C# binding](#the-c-binding)
- [The Godot starter kit](#the-godot-starter-kit)

Both bindings live under `bindings/`. Neither toolchain runs in this
repository's CI, so `tools/check_bindings.py` checks them against the C
API instead: every C# `DllImport` entry point and every `m3` call in the
Godot sources names a public function with that many parameters, and
every C# struct has the size and the field offsets of the C struct it
mirrors.

## The C# binding

`bindings/csharp/Maul3D.cs` is one file with no dependencies. It binds
the core surface over P/Invoke: the world, bodies, the sphere, box,
capsule and plane shapes, every joint type, the step, the closest ray
cast, the hash and the snapshot. It ships with every release and keeps
the same 1.x promise as the C headers: a name it binds is never removed
or renamed, and a release adds to it only when the C API grows. Copy it
into your project. To bind more of the API, follow the same pattern in a
file of your own, so that a later release's `Maul3D.cs` replaces this one
cleanly.

### Build the native library

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release -DMAUL3D_BUILD_SHARED=ON
cmake --build build
```

Put the result next to your managed binary:

| Platform | File |
| --- | --- |
| Windows | `maul3d.dll` |
| Linux | `libmaul3d.so` |
| macOS | `libmaul3d.dylib` |

The `DllImport` name is `maul3d`, and the runtime adds the platform's
prefix and suffix.

### Use

```csharp
using Maul3D;

Native.LayoutCheck(); // once at startup, before anything

var wd = Native.DefaultWorldDef();
if (Native.CreateWorld(ref wd, out IntPtr world) != Result.Success)
{
    throw new InvalidOperationException("the world was refused");
}

var bd = Native.DefaultBodyDef();
bd.Type = BodyType.Dynamic;
bd.Position = new Pos3(0, 5, 0);
Native.CreateBody(world, ref bd, out BodyId body);

var sd = Native.DefaultShapeDef();
var ball = new Sphere { Radius = 0.5f };
Native.CreateSphereShape(world, body, ref sd, ref ball, out ShapeId shape);

for (int i = 0; i < 60; i++)
{
    Native.Step(world, 1.0f / 60.0f, 4);
}
Console.WriteLine($"hash {Native.Hash(world):x16}");

// Rollback: snapshot, run ahead, restore, run again: same hash.
var snap = new byte[Native.GetSnapshotSize(world)];
Native.Snapshot(world, snap, snap.Length, out long bytes);
// ... later ...
Native.Restore(world, snap, bytes);

Native.DestroyWorld(world);
```

### The rules

- Start every def from its `Default...Def()` function. Every def carries
  a cookie, and a zeroed or hand-filled one is refused.
  `DefaultWorldDef()` also stamps `Native.AbiVersion`, so a native
  library of another minor version refuses the world with
  `Result.ErrorVersion`.
- Check the `Result` of every create, destroy, snapshot and restore. A
  refused create leaves its out id null.
- Call `LayoutCheck()` at startup. It compares the size of every
  mirrored struct with the C compiler's and throws before a mismatch
  can corrupt memory.
- The bools inside defs are `byte` fields, since C's `_Bool` is one
  byte. Set them to 0 or 1.
- Ids are values that hold no pointer, and a restore brings back the
  generations the snapshot recorded, so an id stays valid across it. An
  id means something only to the world that issued it.
- The binding passes every call through unchanged, so
  [determinism](determinism.md) holds through it as it does in C.

## The Godot starter kit

`bindings/godot/` is a GDExtension skeleton that puts a Maul3D world in
the scene tree as `M3World3D`, a `Node3D`. It follows Godot's own
versions, which this project does not control, so it is outside the 1.x
promise: a release may change it in any way. It is a seed, not a plugin
release. The node creates its world when it enters the tree, destroys
it when it leaves, steps it from `_physics_process`, and exposes to
GDScript the minimum a rollback game needs:

- `create_dynamic_box(position, half_extents, density) -> int`
- `create_static_floor() -> int`
- `body_transform(body) -> Transform3D`, to drive your meshes
- `set_body_velocity(body, velocity)`
- `snapshot() -> PackedByteArray` and `restore(data) -> bool`
- `world_hash() -> String`, the hash in hexadecimal, to compare
  across peers for a desync
- the `substep_count` property, 4 by default and at least 1

A body id is the engine's 8-byte id packed into an `int`: opaque,
stable across a snapshot restore, and never a pointer.

### Build

From `bindings/godot/`:

```sh
# 1. the Maul3D static library
cmake -B ../../build/release -DCMAKE_BUILD_TYPE=Release ../..
cmake --build ../../build/release

# 2. godot-cpp, matching your Godot version (4.2 at least)
git clone -b 4.2 https://github.com/godotengine/godot-cpp

# 3. the extension
scons platform=linux target=template_release
```

Copy `bin/` and `maul3d.gdextension` into your Godot project. Extend
`M3World3D` along the same pattern: every engine call you need is one
bound method away, and the C API is documented on these pages and in the
[API reference](https://rawframe-project.github.io/maul3d/api.html).

### Stepping for rollback

Godot's `_physics_process` delta is fixed, but a game with rollback
netcode should step the world itself from its netcode tick and keep
`_physics_process` for interpolation only. Snapshots, the journal and
the hash behave as these pages describe; the kit adds nothing and takes
nothing away.
