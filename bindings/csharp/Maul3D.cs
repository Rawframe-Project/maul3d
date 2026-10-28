// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Maul3D C# starter kit: P/Invoke over the core surface (world,
// body, shape, joint, step, queries, snapshot). Single file, no
// dependencies, blittable structs only. Build the native library
// with -DMAUL3D_BUILD_SHARED=ON and drop it next to your binary
// (maul3d.dll / libmaul3d.so / libmaul3d.dylib).
//
// The layout law: every struct here mirrors the C header byte for
// byte. Call Maul3D.Native.LayoutCheck() once at startup; it
// compares Marshal.SizeOf against sizes probed from the C compiler
// and throws before a mismatch can corrupt anything. Bools in defs
// are bytes here (C _Bool is one byte; C# bool marshals as four).
// A world is an opaque pointer, every call takes it first, and a
// create or destroy returns a Result with the new id in an out
// parameter. The rest of the API follows the same pattern; extend as
// needed.

using System;
using System.Runtime.InteropServices;

namespace Maul3D
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Vec3
    {
        public float X, Y, Z;
        public Vec3(float x, float y, float z) { X = x; Y = y; Z = z; }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Quat
    {
        public float X, Y, Z, W;
        public static Quat Identity => new Quat { X = 0, Y = 0, Z = 0, W = 1 };
    }

    /// World positions are double precision (the hybrid precision law:
    /// positions in double, everything else in float).
    [StructLayout(LayoutKind.Sequential)]
    public struct Pos3
    {
        public double X, Y, Z;
        public Pos3(double x, double y, double z) { X = x; Y = y; Z = z; }
    }

    /// What a fallible call returns: zero is success, negative values
    /// are errors (m3Result).
    public enum Result : int
    {
        Success = 0, ErrorInvalid = -1, ErrorCapacity = -2, ErrorStale = -3,
        ErrorUnsupported = -4, ErrorState = -6, ErrorVersion = -7,
        ErrorFormat = -8, ErrorLimit = -9, ErrorBusy = -10
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Version { public ushort Major, Minor, Patch; }

    /// Ids name an object of the world that issued them: a 1-based slot
    /// (0 is the null id) and the slot's generation.
    [StructLayout(LayoutKind.Sequential)]
    public struct BodyId { public uint Index1; public uint Generation; }

    [StructLayout(LayoutKind.Sequential)]
    public struct ShapeId { public uint Index1; public uint Generation; }

    [StructLayout(LayoutKind.Sequential)]
    public struct JointId { public uint Index1; public uint Generation; }

    /// A host allocator: function pointers taking (size, alignment,
    /// context) and (memory, size, alignment, context). All zero means
    /// the C library's allocator.
    [StructLayout(LayoutKind.Sequential)]
    public struct Allocator { public IntPtr Alloc, Free, Context; }

    public enum BodyType : int { Static = 0, Kinematic = 1, Dynamic = 2 }

    public enum JointType : int
    {
        Spherical = 0, Revolute = 1, Prismatic = 2, Fixed = 3, Distance = 4,
        Generic = 5, Wheel = 6, Filter = 7, Parallel = 8, Motor = 9,
        Gear = 10, Pulley = 11
    }

    /// Build with Native.DefaultWorldDef(); the cookie in InternalValue
    /// and the binary interface in Version are demanded by CreateWorld,
    /// so a hand-rolled def refuses loudly.
    [StructLayout(LayoutKind.Sequential)]
    public struct WorldDef
    {
        public int InternalValue;
        public uint Version;
        public Vec3 Gravity;
        public int BodyCapacity;
        public int ShapeCapacity;
        public int MeshCapacity;
        public int JointCapacity;
        public int VoxelCapacity;
        public int CharacterCapacity;
        public int VehicleCapacity;
        public int SoftBodyCapacity;
        public IntPtr EnqueueTask;   // both null = serial (the default)
        public IntPtr FinishTask;
        public IntPtr UserTaskContext;
        public float ContactHertz;
        public float ContactDampingRatio;
        public float ContactPushMaxSpeed;
        public float RestitutionThreshold;
        public float MaximumLinearSpeed;
        public float MaximumAngularSpeed;
        public byte EnableSleeping;   // C _Bool
        public byte EnableContinuous; // C _Bool
        public float HitEventThreshold;
        public Allocator Allocator;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BodyDef
    {
        public int Type; // BodyType
        public Pos3 Position;
        public Quat Rotation;
        public Vec3 LinearVelocity;
        public Vec3 AngularVelocity;
        public float GravityScale;
        public float LinearDamping;
        public float AngularDamping;
        public ulong UserData;
        public byte IsBullet; // C _Bool: one byte, 0 or 1
        public MotionLocks MotionLocks;
        public byte EnableSleep;
        public float SleepThreshold;
        public byte IsEnabled;
        public byte EnableFastRotation;
        public int InternalValue;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct MotionLocks
    {
        public byte LinearX, LinearY, LinearZ, AngularX, AngularY, AngularZ;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ShapeDef
    {
        public float Density;
        public float Friction;
        public float Restitution;
        public ulong CategoryBits;
        public ulong MaskBits;
        public int GroupIndex;
        public float RollingResistance;
        public Vec3 SurfaceVelocity;
        public ulong UserData;
        public byte IsSensor;
        public byte EnableHitEvents;
        public byte EnablePreSolveEvents;
        public Vec3 LocalPosition;
        public Quat LocalRotation;
        public int InternalValue;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JointDef
    {
        public int Type; // JointType
        public BodyId BodyIdA;
        public BodyId BodyIdB;
        public Vec3 LocalAnchorA;
        public Vec3 LocalAnchorB;
        public Vec3 LocalAxisA;
        public Vec3 LocalAxisB;
        public byte EnableLimit;
        public float LowerLimit;
        public float UpperLimit;
        public byte EnableMotor;
        public float MotorSpeed;
        public float MaxMotorEffort;
        public byte EnableCone;
        public float ConeAngle;
        public byte CollideConnected;
        public byte GenericLinear0, GenericLinear1, GenericLinear2;
        public byte GenericAngular0, GenericAngular1, GenericAngular2;
        public byte GenericMotorAxis;
        public float GenericLinearLower0, GenericLinearLower1, GenericLinearLower2;
        public float GenericLinearUpper0, GenericLinearUpper1, GenericLinearUpper2;
        public float GenericAngularLower0, GenericAngularLower1, GenericAngularLower2;
        public float GenericAngularUpper0, GenericAngularUpper1, GenericAngularUpper2;
        public float Ratio;
        public Pos3 GroundAnchorA;
        public Pos3 GroundAnchorB;
        public int InternalValue;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Sphere { public Vec3 Center; public float Radius; }

    [StructLayout(LayoutKind.Sequential)]
    public struct Capsule { public Vec3 Point1; public Vec3 Point2; public float Radius; }

    [StructLayout(LayoutKind.Sequential)]
    public struct Plane { public Vec3 Normal; public float Offset; }

    [StructLayout(LayoutKind.Sequential)]
    public struct RayCastResult
    {
        public ShapeId ShapeId;
        public Pos3 Point;
        public Vec3 Normal;
        public float Fraction;
        public byte Hit;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct QueryFilter { public ulong CategoryBits; public ulong MaskBits; }

    public static class Native
    {
        const string Lib = "maul3d";

        // The version of the C headers this file mirrors. release-please
        // moves it between the markers, as it does in base.h.
        // x-release-please-start-major
        public const uint VersionMajor = 0;
        // x-release-please-end
        // x-release-please-start-minor
        public const uint VersionMinor = 7;
        // x-release-please-end
        // x-release-please-start-patch
        public const uint VersionPatch = 0;
        // x-release-please-end

        /// M3_ABI_VERSION of the C headers this file mirrors, computed as
        /// base.h computes it. DefaultWorldDef stamps it, so a library built
        /// from other headers refuses the def with Result.ErrorVersion
        /// instead of misreading it.
        public const uint AbiVersion = 0x80000000u | (VersionMajor << 16) |
            (VersionMajor == 0 ? (VersionMinor << 8) | VersionPatch : VersionMinor);

        [DllImport(Lib, EntryPoint = "m3GetVersion")]
        public static extern Version GetVersion();
        [DllImport(Lib, EntryPoint = "m3ResultName")]
        public static extern IntPtr ResultName(Result result);

        // World
        [DllImport(Lib, EntryPoint = "m3DefaultWorldDefValues")]
        public static extern WorldDef DefaultWorldDefValues();
        public static WorldDef DefaultWorldDef()
        {
            WorldDef def = DefaultWorldDefValues();
            def.Version = AbiVersion;
            return def;
        }
        [DllImport(Lib, EntryPoint = "m3CreateWorld")]
        public static extern Result CreateWorld(ref WorldDef def, out IntPtr world);
        [DllImport(Lib, EntryPoint = "m3DestroyWorld")]
        public static extern void DestroyWorld(IntPtr world);
        [DllImport(Lib, EntryPoint = "m3World_Step")]
        public static extern void Step(IntPtr world, float dt, int substeps);
        [DllImport(Lib, EntryPoint = "m3World_Hash")]
        public static extern ulong Hash(IntPtr world);

        // Snapshot and rollback: the buffer is opaque bytes; restore
        // refuses any config or version mismatch (loudly, by contract,
        // never a partial restore).
        [DllImport(Lib, EntryPoint = "m3World_SnapshotSize")]
        public static extern int SnapshotSize(IntPtr world);
        [DllImport(Lib, EntryPoint = "m3World_Snapshot")]
        public static extern Result Snapshot(IntPtr world, byte[] buffer, int capacity,
                                             out int bytes);
        [DllImport(Lib, EntryPoint = "m3World_Restore")]
        public static extern Result Restore(IntPtr world, byte[] data, int size);

        // Body
        [DllImport(Lib, EntryPoint = "m3DefaultBodyDef")]
        public static extern BodyDef DefaultBodyDef();
        [DllImport(Lib, EntryPoint = "m3CreateBody")]
        public static extern Result CreateBody(IntPtr world, ref BodyDef def, out BodyId body);
        [DllImport(Lib, EntryPoint = "m3DestroyBody")]
        public static extern Result DestroyBody(IntPtr world, BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_GetPosition")]
        public static extern Pos3 GetPosition(IntPtr world, BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_GetRotation")]
        public static extern Quat GetRotation(IntPtr world, BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_GetLinearVelocity")]
        public static extern Vec3 GetLinearVelocity(IntPtr world, BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_GetAngularVelocity")]
        public static extern Vec3 GetAngularVelocity(IntPtr world, BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_SetLinearVelocity")]
        public static extern void SetLinearVelocity(IntPtr world, BodyId body, Vec3 velocity);
        [DllImport(Lib, EntryPoint = "m3Body_SetAngularVelocity")]
        public static extern void SetAngularVelocity(IntPtr world, BodyId body, Vec3 velocity);
        [DllImport(Lib, EntryPoint = "m3Body_SetTransform")]
        public static extern void SetTransform(IntPtr world, BodyId body, Pos3 position,
                                               Quat rotation);

        // Shapes
        [DllImport(Lib, EntryPoint = "m3DefaultShapeDef")]
        public static extern ShapeDef DefaultShapeDef();
        [DllImport(Lib, EntryPoint = "m3CreateSphereShape")]
        public static extern Result CreateSphereShape(IntPtr world, BodyId body, ref ShapeDef def,
                                                      ref Sphere sphere, out ShapeId shape);
        [DllImport(Lib, EntryPoint = "m3CreateBoxShape")]
        public static extern Result CreateBoxShape(IntPtr world, BodyId body, ref ShapeDef def,
                                                   Vec3 halfExtents, out ShapeId shape);
        [DllImport(Lib, EntryPoint = "m3CreateCapsuleShape")]
        public static extern Result CreateCapsuleShape(IntPtr world, BodyId body, ref ShapeDef def,
                                                       ref Capsule capsule, out ShapeId shape);
        [DllImport(Lib, EntryPoint = "m3CreatePlaneShape")]
        public static extern Result CreatePlaneShape(IntPtr world, BodyId body, ref ShapeDef def,
                                                     ref Plane plane, out ShapeId shape);

        // Joints
        [DllImport(Lib, EntryPoint = "m3DefaultJointDef")]
        public static extern JointDef DefaultJointDef();
        [DllImport(Lib, EntryPoint = "m3CreateJoint")]
        public static extern Result CreateJoint(IntPtr world, ref JointDef def, out JointId joint);
        [DllImport(Lib, EntryPoint = "m3DestroyJoint")]
        public static extern Result DestroyJoint(IntPtr world, JointId joint);

        // Queries
        [DllImport(Lib, EntryPoint = "m3DefaultQueryFilter")]
        public static extern QueryFilter DefaultQueryFilter();
        [DllImport(Lib, EntryPoint = "m3World_CastRayClosest")]
        public static extern RayCastResult CastRayClosest(IntPtr world, Pos3 origin,
                                                          Vec3 translation, QueryFilter filter);

        /// Sizes probed from the C compiler (x86-64 System V and
        /// Windows x64 agree on all of these). A mismatch here means
        /// a header drifted; fail before anything corrupts.
        public static void LayoutCheck()
        {
            Check<Vec3>(12);
            Check<Quat>(16);
            Check<Pos3>(24);
            Check<WorldDef>(136);
            Check<BodyDef>(120);
            Check<ShapeDef>(104);
            Check<JointDef>(216);
            Check<RayCastResult>(56);
            Check<QueryFilter>(16);
            Check<Version>(6);
            Check<Allocator>(24);
            Check<BodyId>(8);
            Check<ShapeId>(8);
            Check<JointId>(8);
            Check<MotionLocks>(6);
            Check<Sphere>(16);
            Check<Capsule>(28);
            Check<Plane>(16);
        }

        static void Check<T>(int expected)
        {
            int actual = Marshal.SizeOf<T>();
            if (actual != expected)
            {
                throw new InvalidOperationException(
                    $"Maul3D layout drift: {typeof(T).Name} is {actual} bytes, expected {expected}");
            }
        }
    }
}
