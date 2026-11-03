# Maul3D C# binding

One file, no dependencies: `Maul3D.cs` binds the core surface
(world, body, shape, joint, step, queries, snapshot) over P/Invoke.
It ships with every release and keeps the same 1.x promise as the C
headers: a name it binds is never removed or renamed, and a release
adds to it only when the C API grows. Copy it into your project; to
bind more of the API, follow the same pattern in a file of your own,
so a later release's `Maul3D.cs` replaces this one cleanly.

## Build the native library

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

The `DllImport` name is `maul3d`; the runtime resolves the
platform prefix and suffix itself.

## Use

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
Native.Snapshot(world, snap, snap.Length, out int bytes);
// ... later ...
Native.Restore(world, snap, bytes);

Native.DestroyWorld(world);
```

## The rules that keep you safe

- Always start from the `Default*Def()` functions. Every def
  carries a cookie; a zeroed or hand-rolled def is refused loudly
  by design. `DefaultWorldDef()` also stamps `Native.AbiVersion`, so
  a native library of another minor version refuses the world with
  `Result.ErrorVersion`.
- Check the `Result` of every create, destroy, snapshot and restore.
  A refused create leaves its out id null.
- Call `LayoutCheck()` at startup. It catches a struct mirror
  drifting from the C headers before it can corrupt memory. CI runs
  the same check ahead of time: `tools/check_bindings.py` compares
  every import and every mirrored field offset with the C side.
- Bools inside defs are `byte` fields (C `_Bool` is one byte).
  Set them to 0 or 1.
- Ids are value types and stay valid across snapshot restore;
  they encode no pointers. An id means something only to the world
  that issued it.
- Determinism holds through this binding: commands in, hashes
  out, the same on every platform the engine supports.
