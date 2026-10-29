# Maul3D C# starter kit

One file, no dependencies: `Maul3D.cs` binds the core surface
(world, body, shape, joint, step, queries, snapshot) over P/Invoke.
It is a seed, not a package: copy it into your project and extend
it along the same pattern as your game needs more of the API.

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
bd.Type = (int)BodyType.Dynamic;
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
  a native library built from other headers refuses the world with
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
