// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world: SoA body state in persistent arrays (every one of them a
// future M3_BLOCK in the snapshot walker), the id pool behind body
// handles, the per-step scratch stack, and the journal cursor. No
// pointers inside persistent state; slots cross-reference by index.

#ifndef MAUL3D_SRC_WORLD_INTERNAL_H
#define MAUL3D_SRC_WORLD_INTERNAL_H

#include "allocator.h"
#include "dynamic_tree.h"

#include "maul3d/body.h"
#include "maul3d/character.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "maul3d/vehicle.h"
#include "maul3d/world.h"

// (int32_t) from a float is UB on NaN and outside the int range:
// x86 shrugs INT_MIN, wasm TRAPS, and the wide fuzz walked a
// mutated-snapshot NaN into a heightfield gather (shape_data.c). Every
// grid-cell cast goes through this park-and-clamp. Legitimate
// values are bit-identical: no real grid nears two billion cells.
// nanPark picks which way a poisoned bound falls so lo/hi ranges
// come out EMPTY, never huge.
static inline int32_t m3CellFromF(m3real f, m3real nanPark)
{
    f = f != f ? nanPark : f;
    f = f < -2.0e9f ? -2.0e9f : (f > 2.0e9f ? 2.0e9f : f);
    return (int32_t)f;
}

#define M3_MAX_WORLDS 64
_Static_assert(M3_MAX_WORLDS <= (1 << M3_WORLD_SLOT_BITS), "a world slot fits the id bits");

// Bumped on any change that alters simulation behavior (solver math,
// integration order, constants). It is part of the snapshot config hash,
// so a snapshot from another behavior revision is refused instead of
// silently diverging.
#define M3_SOLVER_REV 21

// Def cookies: a def that did not come from its m3Default*Def factory
// is rejected loudly (the Maul2D pattern).
#define M3_COOKIE       0x4D33u // 'M3'
#define M3_WORLD_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3WorldDef) << 8)))
#define M3_BODY_COOKIE  ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3BodyDef) << 8) ^ 1))
#define M3_SHAPE_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3ShapeDef) << 8) ^ 2))

// Fat AABB margin: pairs exist slightly before touch so speculative
// contacts have something to work with.
#define M3_AABB_MARGIN (4.0f * 0.005f)

// Hard ceiling on substeps per step, enforced at the public wall
// and the replay wall alike: far above any legitimate use,
// low enough that a flipped tape bit cannot buy a billion substeps.
#define M3_MAX_SUBSTEPS 256

// Debug names: journaled and snapshot like userData, NEVER
// hashed (a name moves no matter).
#define M3_BODY_NAME_CAPACITY 32

// Water volume slots per world: a fixed, small table.
#define M3_MAX_WATER_VOLUMES 8

// Immutable interned hull data (lifetime 3): vertices, face planes,
// and face vertex loops for the SAT, plus unit-density mass
// properties. Content-deduplicated on intern; shapes reference by
// index and refcount. Fixed arrays keep it one snapshot block.
// Euler-consistent worst case for 24 vertices: a simplicial hull has
// F = 2V - 4 = 44 faces and E = 3V - 6 = 66 edges (132 half edges).
// Sized so no valid 24-vertex hull can ever overflow the fixed block.
// The parity caps. Euler for V = 64: F <= 2V - 4 = 124,
// half-edges <= 6V - 12 = 372. Geometry is create-time state the
// hash covers by INDEX, so raising the caps moves no hash; only
// the snapshot block size.
#define M3_HULL_MAX_VERTS        64
#define M3_HULL_MAX_FACES        124
#define M3_HULL_MAX_FACE_INDICES 372
#define M3_HULL_MAX_HALF_EDGES   372
#define M3_HULL_MAX_INPUT        256 // QuickHull input point cap

// Half-edge adjacency (the Gauss-map edge query reads the two faces
// flanking every edge). Twins sit at 2k and 2k+1 by construction.
typedef struct m3HullHalfEdge
{
    uint16_t twin; // half-edge indices outgrow a byte at 64 verts
    uint16_t next;
    uint8_t origin; // vertex (< 64) and face (< 124) stay bytes
    uint8_t face;
} m3HullHalfEdge;

typedef struct m3HullData
{
    int32_t vertexCount;
    int32_t faceCount;
    int32_t indexCount;
    float unitMass;        // at density one
    m3Vec3 unitCom;        // body-local centroid
    m3Mat3 unitInertiaCom; // at density one, about the centroid
    m3Vec3 vertices[M3_HULL_MAX_VERTS];
    m3Vec3 faceNormals[M3_HULL_MAX_FACES];
    m3real faceOffsets[M3_HULL_MAX_FACES];
    uint8_t faceVertCounts[M3_HULL_MAX_FACES];
    uint16_t faceVertStart[M3_HULL_MAX_FACES]; // offsets reach 372
    uint8_t faceIndices[M3_HULL_MAX_FACE_INDICES];
    int32_t edgeCount; // half-edge count (twice the undirected edges)
    m3HullHalfEdge edges[M3_HULL_MAX_HALF_EDGES];
    m3Vec3 center; // vertex centroid, orients the edge separation
} m3HullData;

_Static_assert(sizeof(m3HullData) == 5808, "hull data must be padding-free");

// Static triangle mesh content: immutable, heap-allocated per slot and
// sized by its counts (a fixed block at 65k triangles would cost
// megabytes per empty slot), with 16-bit vertex indices.
#define M3_MESH_MAX_VERTS 65535
#define M3_MESH_MAX_TRIS  65535

// Per-triangle surface material: a mesh carries up to eight
// entries (the public m3MeshSurfaceMaterial) and a byte per
// triangle naming one; count 0 means "use the shape material"
// everywhere.
#define M3_MESH_MAX_MATERIALS 8

typedef struct m3MeshData
{
    int32_t vertexCount;
    int32_t triangleCount;
    m3Vec3* vertices;
    uint16_t* indices; // CCW from outside
    // Bit k set = triangle edge k (vertex k to k+1) is CONVEX or a
    // boundary: a REAL contact feature. Clear = flat or concave: a
    // ghost candidate the welding filter may silence. Baked at
    // create time, deterministic.
    uint8_t* edgeFlags;
    // Material groups. materialCount 0 = the mesh defers to
    // its shape's material everywhere (canonical zeros throughout).
    int32_t materialCount;
    uint8_t* triMaterials; // one group index per triangle
    m3MeshSurfaceMaterial materials[M3_MESH_MAX_MATERIALS];
} m3MeshData;

// Count-derived ownership: Alloc frees any old arrays and
// sizes new ones from the counts already in the struct; Free
// releases and zeroes. The snapshot read pass and every create
// and destroy path share these two gates.
bool m3MeshDataAlloc(m3MeshData* mesh);
void m3MeshDataFree(m3MeshData* mesh);

// Bakes the edge flags. The scratch holds m3MeshEdgeScratchCount ints.
int32_t m3MeshEdgeScratchCount(const m3MeshData* mesh);
void m3BakeMeshEdgeFlags(m3MeshData* mesh, int32_t* scratch);

// Native heightfield content: immutable, count-derived raw
// samples, the low-memory terrain path beside meshes. The cell at
// (ix, iz) spans x in [ix, ix+1] * cellSize, z likewise, and
// splits into two CCW-from-above triangles along the ix+iz
// diagonal parity (a fixed, deterministic split).
#define M3_HEIGHTFIELD_MAX_DIM 255

typedef struct m3HeightFieldData
{
    int32_t nx;
    int32_t nz;
    float cellSize;
    float minHeight; // baked at create: the AABB floor
    float maxHeight; // baked at create: the AABB ceiling
    float* heights;  // nx * nz samples, row-major, x fastest
} m3HeightFieldData;

// Count-derived ownership: Alloc sizes from the counts already in the
// struct, Free releases and zeroes.
bool m3HeightFieldDataAlloc(m3HeightFieldData* hf);
void m3HeightFieldDataFree(m3HeightFieldData* hf);

// One cell's two triangles, split by diagonal parity: the one rule every
// consumer shares (contacts, rays, queries, draw, soft particles).
static inline void m3HeightFieldCellTris(const m3HeightFieldData* hf, int32_t cx, int32_t cz,
                                         m3Vec3 out[2][3])
{
    m3real cs = hf->cellSize;
    m3Vec3 a = {(m3real)cx * cs, hf->heights[cz * hf->nx + cx], (m3real)cz * cs};
    m3Vec3 b = {(m3real)(cx + 1) * cs, hf->heights[cz * hf->nx + cx + 1], (m3real)cz * cs};
    m3Vec3 c = {(m3real)(cx + 1) * cs, hf->heights[(cz + 1) * hf->nx + cx + 1],
                (m3real)(cz + 1) * cs};
    m3Vec3 d = {(m3real)cx * cs, hf->heights[(cz + 1) * hf->nx + cx], (m3real)(cz + 1) * cs};
    if ((cx + cz) % 2 == 0)
    {
        out[0][0] = a;
        out[0][1] = c;
        out[0][2] = b;
        out[1][0] = a;
        out[1][1] = d;
        out[1][2] = c;
    }
    else
    {
        out[0][0] = b;
        out[0][1] = a;
        out[0][2] = d;
        out[1][0] = b;
        out[1][1] = d;
        out[1][2] = c;
    }
}

// Bounded triangle gather from a local-frame box: fills up
// to cap corner triples in ascending cell order and returns the
// count. Consumers that must NEVER truncate (the raycast) walk
// cells themselves.
int32_t m3HeightFieldGather(const m3HeightFieldData* hf, m3Vec3 lo, m3Vec3 hi, m3Vec3 (*tris)[3],
                            int32_t cap);

// Static per-mesh BVH: median split on the longest centroid
// axis, ties broken by triangle index, leaves of up to four
// triangles. Derived acceleration data: NEVER snapshotted, never
// hashed; rebuilt deterministically wherever mesh content lands
// (create, journal replay, restore).
#define M3_MESH_BVH_LEAF 4
// Node budget per build: 2 * triCount, allocated exactly.

// Quantized node bounds: uint16 grid coordinates against
// the root box, rounded OUTWARD at build and the query rounded
// outward again at gather, so the visit set stays a superset of
// the exact overlaps (visiting more is legal, missing one never).
// Every consumer keeps its exact reject test, so results stay
// bit-identical to the float tree this replaces; 20 bytes per node
// instead of 36.
typedef struct m3MeshBvhNode
{
    uint16_t qlo[3];
    uint16_t qhi[3];
    int32_t right;  // internal: right child (left child = self + 1)
    uint16_t start; // leaf only: first slot in order[]
    uint16_t count; // leaf: triangle count; 0 marks an internal node
} m3MeshBvhNode;

typedef struct m3MeshBvh
{
    int32_t nodeCount;
    m3MeshBvhNode* nodes; // derived data: built at create and after
    uint16_t* order;      // restore, sized 2 * triCount and triCount
    m3Vec3 rootLo;        // the quantization frame
    m3Vec3 rootHi;        // float early-out: a query outside the root
                          // is FREE, not clamped onto the grid edge
    m3Vec3 quantScale;    // world-to-grid; 0 on a flat axis (always hits)
} m3MeshBvh;

void m3MeshBvhBuild(m3MeshBvh* bvh, const m3MeshData* mesh);

// Gather every triangle in a leaf whose box overlaps [lo, hi], in
// ascending triangle order (each triangle lives in exactly one leaf,
// so the list is duplicate-free). The result is a SUPERSET of the
// exact per-triangle overlaps: every consumer keeps its own exact
// reject test, so behavior stays bit-identical to the full scan this
// replaces. `out` must hold M3_MESH_MAX_TRIS entries.
int32_t m3MeshBvhGather(const m3MeshBvh* bvh, m3Vec3 lo, m3Vec3 hi, uint16_t* out);
void m3MeshBvhFree(m3MeshBvh* bvh);

// Generalized build: a BVH over caller-supplied bounds (the mesh
// build wraps this with triangle boxes; the voxel surface feeds
// merged-box bounds). Count is capped at M3_MESH_MAX_TRIS.
void m3MeshBvhBuildBounds(m3MeshBvh* bvh, const m3Vec3* los, const m3Vec3* his, int32_t count);

// Voxel chunks: dense 16^3 occupancy + payload, one
// padding-free snapshot block per slot. STATE ends there; the
// merged-box surface and its BVH are derived (never snapshotted,
// never hashed, rebuilt wherever content lands).
#define M3_VOXEL_DIM       16
#define M3_VOXEL_COUNT     (M3_VOXEL_DIM * M3_VOXEL_DIM * M3_VOXEL_DIM)
#define M3_VOXEL_MAX_BOXES 2048 // the isolated-voxel (checkerboard) worst case

typedef struct m3VoxelChunkData
{
    m3real cellSize;
    int32_t filledCount;
    uint8_t occupancy[M3_VOXEL_COUNT / 8];
    uint16_t payload[M3_VOXEL_COUNT];
    // Fill fraction: 255 = a whole voxel, 1 = a sliver; zero
    // never occurs on an occupied voxel (clearing is ClearVoxel's
    // job). Fill is a MASS and destruction property, never a
    // geometry property: a half-destroyed voxel still collides as
    // a full box, but its fragment weighs half.
    uint8_t fill[M3_VOXEL_COUNT];
} m3VoxelChunkData;

_Static_assert(sizeof(m3VoxelChunkData) == 8 + 512 + 8192 + 4096,
               "voxel chunk must be padding-free");

typedef struct m3VoxelSurface
{
    int32_t boxCount;
    uint8_t boxLo[M3_VOXEL_MAX_BOXES][3]; // inclusive voxel coords
    uint8_t boxHi[M3_VOXEL_MAX_BOXES][3];
    // Seam welding: bit k of boxCovered[b] says face k of box
    // b (-x,+x,-y,+y,-z,+z) is fully flush against filled voxels,
    // in this chunk or a welded neighbor. A covered face is
    // interior geometry and can never be a contact feature.
    uint8_t boxCovered[M3_VOXEL_MAX_BOXES];
    m3MeshBvh bvh;
} m3VoxelSurface;

bool m3VoxelGet(const m3VoxelChunkData* chunk, int32_t x, int32_t y, int32_t z);
int32_t m3VoxelPack(m3VoxelChunkData* chunk, const uint8_t* voxels, const uint16_t* payload,
                    m3real cellSize);
void m3VoxelSurfaceBuild(m3VoxelSurface* surface, const m3VoxelChunkData* chunk);
void m3VoxelBoxBounds(const m3VoxelSurface* surface, m3real cellSize, int32_t box, m3Vec3* lo,
                      m3Vec3* hi);
void m3VoxelBoxHull(const m3VoxelSurface* surface, m3real cellSize, int32_t box, m3HullData* out);

// Geometry is one padding-free 32-byte record per shape, interpreted
// by type: sphere {v=center, s=radius}, plane {v=normal, s=offset},
// hull {v=box half extents for the journaled rebuild, s unused},
// capsule {v=point1, v2=point2, s=radius}.
typedef struct m3ShapeGeom
{
    m3Vec3 v;  // sphere center | plane normal | box half extents | capsule p1
    m3real s;  // radius | offset | unused | radius
    m3Vec3 v2; // capsule p2
    m3real s2; // reserved
} m3ShapeGeom;

_Static_assert(sizeof(m3ShapeGeom) == 32, "shape geom must be padding-free");

// One contact point: anchors are measured from each body's center in
// world orientation (float is exact enough near contact), impulses are
// the warm-start payload that persists across steps and through the
// snapshot. Padding-free by construction.
typedef struct m3ManifoldPoint
{
    m3Vec3 anchorA;
    m3Vec3 anchorB;
    m3real separation; // negative = penetration, positive = speculative
    m3real normalImpulse;
    uint16_t id;    // feature id (spheres have exactly one feature: 0)
    uint16_t flags; // bit 0: persisted (impulses carried this rebuild)
} m3ManifoldPoint;

_Static_assert(sizeof(m3ManifoldPoint) == 36, "manifold point must be padding-free");

// Up to four contact points: a hull face on a plane or a face
// pair needs four; spheres use one. Feature ids identify points across
// rebuilds for the warm-start carry.
#define M3_MANIFOLD_MAX_POINTS 4

typedef struct m3Manifold
{
    m3Vec3 normal; // from shape A to shape B, world frame
    int32_t pointCount;
    // Central friction warm-start payload: stored in the
    // WORLD frame so the next step's tangent basis re-projects it.
    // The rolling row warm-starts every substep from here too.
    m3Vec3 frictionImpulse;
    m3real twistImpulse;
    m3Vec3 rollingImpulse;
    m3ManifoldPoint points[M3_MANIFOLD_MAX_POINTS];
} m3Manifold;

_Static_assert(sizeof(m3Manifold) == 188, "manifold must be padding-free");

// Bodies: the id pool and parallel SoA state, hot fields first.
typedef struct m3Bodies
{
    int32_t bodyCapacity;
    // Body identity.
    m3IdPool bodyPool;
    // SoA body state, hot fields first. All persistent, all walked by
    // the snapshot.
    m3Transform* transforms;
    m3Vec3* linearVelocities;
    m3Vec3* angularVelocities;
    m3real* invMass;
    m3Mat3* invInertiaLocal; // inverse inertia about the COM, body frame
    m3Mat3* inertiaLocal;    // forward tensor (the gyroscopic solve reads it)
    m3Vec3* localCenters;    // center of mass in the body frame
    m3real* gravityScales;
    m3real* linearDamping;
    m3real* angularDamping;
    uint8_t* types;
    uint8_t* awake;       // 0 = sleeping (dynamic bodies only), frozen solid
    float* sleepTimes;    // seconds below the sleep threshold
    uint8_t* bulletFlags; // 1 = full continuous vs dynamics
    float* minExtents;    // per body: thinnest shape measure (CCD trigger)
    float* maxExtents;    // per body: farthest point from the COM (CCD arc bound)
    uint64_t* userData;
    m3Vec3* bodyForce;         // host force accumulator: integrated
    m3Vec3* bodyTorque;        // each substep, cleared after the step,
                               // hashed only when nonzero
    uint8_t* bodyEnabled;      // Disabled = invisible everywhere
    uint8_t* bodyLocks;        // bits 0..2 linear xyz, 3..5 angular xyz,
                               // bit 6 allowFastRotation
    float* bodySleepThreshold; // per-body; default the world constant
    uint8_t* bodyCanSleep;     // 0 = never sleeps
    uint8_t* bodyHasTarget;    // kinematic servo pending (one step)
    m3Transform* bodyTarget;   // the servo pose
    int32_t* bodyIsland;       // OBSERVER ONLY: the island
                               // root labeled by the last step's
                               // census, -1 before any step. Never
                               // a snapshot block, never hashed.
    char* bodyNames;           // cap * M3_BODY_NAME_CAPACITY:
                               // journaled + snapshot, never hashed
    int32_t* bodyShapeHead;    // head of each body's shape list, -1 none
} m3Bodies;

// Shapes: the id pool and SoA state, with each shape's link into the
// interned hull, mesh, heightfield and voxel content.
typedef struct m3Shapes
{
    int32_t shapeCapacity;
    // Shape identity and SoA shape state (persistent, walked).
    m3IdPool shapePool;
    int32_t* shapeBody;
    uint8_t* shapeType;
    m3ShapeGeom* shapeGeom;
    float* shapeDensity;
    float* shapeFriction;
    float* shapeRestitution;
    uint8_t* shapeHitEvents;       // Emit hit events (default 0)
    m3Vec3* shapeLocalPos;         // compound offset (default zero)
    m3Quat* shapeLocalRot;         // compound rotation (default identity)
    uint8_t* shapeHasOffset;       // fast identity short-circuit
    m3Vec3* shapeSurfaceVel;       // conveyor speed (default zero)
    uint8_t* shapePreSolve;        // Run the pre-solve veto (default 0)
    float* shapeRollingResistance; // hashed only when nonzero
    uint64_t* shapeCategory;       // filters: hashed only when a
    uint64_t* shapeMask;           // value differs from its default
    int32_t* shapeGroup;
    uint64_t* shapeUserData;
    int32_t* shapeNext;       // next shape on the same body, -1 end
    int32_t* shapeHullIndex;  // interned hull slot, -1 for non-hulls
    int32_t* shapeMeshIndex;  // mesh slot, -1 for non-meshes
    int32_t* shapeVoxelIndex; // voxel chunk slot, -1 otherwise
    uint8_t* shapeSensor;     // 1 = overlap detector, never contact response
    int32_t* shapeHfIndex;
    // Derived: the live plane shapes in slot order. Planes stay out of
    // the tree, so every query that must see them walks this list.
    int32_t* planeShapes;
    int32_t planeCount;
} m3Shapes;

// The interned hull pool: immutable content, refcounted.
typedef struct m3Hulls
{
    // The interned hull pool (immutable content, refcounted).
    m3IdPool hullPool;
    m3HullData* hullData;
    int32_t* hullRefCounts;
} m3Hulls;

// Static mesh slots: immutable content, refcounted, and each mesh's BVH
// (derived data, rebuilt on create and restore, never hashed).
typedef struct m3Meshes
{
    // Static mesh slots (immutable content, refcounted, no content
    // dedupe: meshes are big and user-authored).
    int32_t meshCapacity;
    m3IdPool meshPool;
    m3MeshData* meshData;
    int32_t* meshRefCounts;
    // slot (-1 none), from exact
    // grid-aligned transforms

    // Per-mesh static BVH: DERIVED data, never in the
    // snapshot or the hash. Rebuilt from mesh content on create and
    // on restore; the build is a pure function of the triangle set,
    // so twin worlds always agree bit for bit.
    struct m3MeshBvh* meshBvh;
} m3Meshes;

// Native heightfields: interned like meshes, one slot per shape.
typedef struct m3HeightFields
{
    // Native heightfields: interned like meshes, one slot
    // per shape capacity, count-derived content.
    m3IdPool hfPool;
    m3HeightFieldData* hfData;
    int32_t* hfRefCounts;
} m3HeightFields;

// Voxel chunk slots: the mesh-slot pattern plus the derived surface.
typedef struct m3Voxels
{
    // Voxel chunk slots: the mesh-slot pattern (pool,
    // refcounts, per-slot state block), plus the DERIVED surface.
    int32_t voxelCapacity;
    m3IdPool voxelPool;
    m3VoxelChunkData* voxelData;
    int32_t* voxelRefCounts;
    struct m3VoxelSurface* voxelSurface; // derived, not snapshot
    int32_t* voxelShape;                 // owning shape per slot, -1 free
    int32_t* voxelNeighbors;             // derived: 6 welded slots per
} m3Voxels;

// Broadphase: the fat-AABB tree and each shape's proxy.
typedef struct m3Broadphase
{
    // Broadphase: the fat-AABB tree (spheres only; infinite planes
    // stay out and take a dedicated pass), per-shape proxy ids, both
    // persistent snapshot state.
    m3Tree tree;
    int32_t* proxyIds; // M3_TREE_NULL for planes and dead shapes
    // Derived: every pair of overlapping leaves (static-static aside)
    // and the shapes whose leaf changed since the last update. A
    // restore clears candidatesFresh and the next update requeries.
    uint64_t* candidateKeys;
    uint8_t* moved;
    int32_t candidateCount;
    uint8_t candidatesFresh;
} m3Broadphase;

// Contacts: candidate pairs in ascending key order and their manifolds
// (warm-start impulses live here, so this is snapshot state), last
// step's pairs for the impulse carry, the cold-pair harvest, and the
// pre-solve veto bookkeeping.
typedef struct m3Contacts
{
    // Candidate pairs in canonical ascending key order, and their
    // manifolds (persistent: warm-start impulses live here and ride
    // the snapshot).
    uint64_t* pairKeys;
    // The previous step's pairs and manifolds, copied here before the
    // scan overwrites them so the narrowphase can carry warm-start
    // impulses. Sized to the pair capacity once at creation, so a step
    // never allocates. Scratch only: never snapshotted or hashed.
    uint64_t* stashPairKeys;
    m3Manifold* stashManifolds;
    // Cold pairs: pairs whose every endpoint is cold (no awake dynamic
    // body) are harvested from the previous step's list and merged
    // back without re-querying the tree. Derived state: never
    // snapshotted; a restore raises pairsFullQuery and the next
    // update rebuilds everything from the tree.
    uint64_t* sleepingPairKeys;
    int32_t sleepingPairCount;
    uint8_t pairsFullQuery;
    uint8_t frozenDirty;
    // Test hook: solve every contact with the scalar rows, the reference
    // the lane kernel must match bit for bit.
    uint8_t scalarRows;
    // Veto bookkeeping. stepVeto* collects what the live
    // callback vetoed this step (journal fodder); replayVeto* is
    // the pending recorded set the NEXT step must apply, consumed
    // by PrepareContacts and cleared by restore like events.
    uint64_t* stepVetoKeys;
    int32_t stepVetoCount;
    uint64_t* replayVetoKeys;
    int32_t replayVetoCount;
    m3Manifold* manifolds;
    int32_t pairCount;
    int32_t pairCapacity;
} m3Contacts;

// Joints: SoA arenas, all persistent snapshot state, and the per-body
// joint lists that drive the jointed-pair filter, the destroy cascade
// and island coupling.
typedef struct m3Joints
{
    // Generic 6-DOF state: packed modes (2 bits per axis,
    // linear 0..5, angular 6..11, motor axis 12..15) and per-axis
    // limit vectors. Folded into the hash only for generic-typed
    // joints.
    uint16_t* jointGenericModes;
    m3Vec3* jointGenLinLower;
    m3Vec3* jointGenLinUpper;
    m3Vec3* jointGenAngLower;
    m3Vec3* jointGenAngUpper;
    // Pulley world anchors: fixed points the two rope
    // segments hang from, double like every world position. Folded
    // into the hash only for pulley-typed joints.
    m3Pos3* jointGroundA;
    m3Pos3* jointGroundB;
    m3IdPool jointPool;
    uint8_t* jointType;
    int32_t* jointBodyA;
    int32_t* jointBodyB;
    m3Vec3* jointLocalA;
    m3Vec3* jointLocalB;
    uint8_t* jointCollide;
    m3Vec3* jointImpulse;        // warm-start linear impulse
    m3Vec3* jointPerpImpulse;    // x, y = collinearity rows; z = motor
    m3Vec3* jointLimitImpulse;   // x = lower, y = upper, z unused
    m3Vec3* jointAngularImpulse; // prismatic 3-DOF rotation lock
    m3Quat* jointFrameQA;        // joint frame in body A (axis = local z)
    m3Quat* jointFrameQB;
    uint8_t* jointFlags;        // bit0 limit, bit1 motor
    m3Vec3* jointBreak;         // X = max force, y = max torque, 0 = off
    m3Vec3* jointSpring;        // X = hertz, y = damping ratio (flags bit 3)
    float* jointTargetScalar;   // revolute angle or prismatic translation
    m3Quat* jointTargetQ;       // spherical rotation drive target
    m3Vec3* jointSpringImpulse; // warm payload: x scalar rows, xyz spherical
    m3Vec3* jointMotor;         // x = motorSpeed, y = maxMotorTorque, z unused
    m3Vec3* jointLimits;        // x = lower angle, y = upper angle, z unused
    int32_t* bodyJointHead;
    int32_t* jointNextA; // next joint in body A's list
    int32_t* jointNextB; // next joint in body B's list
    int32_t jointCapacity;
    m3JointBreakEvent* jointBreakEvents;
    int32_t jointBreakEventCount;
} m3Joints;

// Characters: the pool and per-slot state (config and the grounded story
// are simulation state).
typedef struct m3Characters
{
    // Joints: SoA arenas, all persistent snapshot state. The
    // warm-start impulse is simulation state (the architecture doc
    // names it); the body lists drive the jointed-pair contact
    // filter, the destroy cascade, and island coupling.
    // Characters: pool + per-slot state. Config floats and
    // the grounded story are all simulation state (hashed for live
    // slots, snapshotted, rolled back).
    int32_t characterCapacity;
    m3IdPool charPool;
    int32_t* charBody; // the internal kinematic body slot
    m3real* charRadius;
    m3real* charHalfHeight;
    m3real* charCosSlope;
    m3real* charSnap;
    m3real* charSkin;
    m3real* charStepHeight;
    uint8_t* charGrounded;
    m3Vec3* charGroundNormal;
    m3real* charMass;        // kilograms; scales push impulses
    m3real* charPushMax;     // heaviest pushable body / mass ratio
    int32_t* charGroundBody; // body slot under our feet, -1 none
    uint16_t* charGroundGen; // that body's generation when recorded
} m3Characters;

// Vehicles: pooled raycast vehicles; per-slot config, per-wheel arrays at
// slot * M3_VEHICLE_MAX_WHEELS + wheel, tank mode and the drivetrain.
typedef struct m3Vehicles
{
    // Vehicles: pooled raycast vehicles. Per-slot config plus
    // per-wheel arrays at slot * M3_VEHICLE_MAX_WHEELS + wheel. All
    // persistent simulation state: snapshotted, hashed for live
    // slots.
    int32_t vehicleCapacity;
    m3IdPool vehPool;
    int32_t* vehChassis;     // chassis body slot, -1 free
    uint16_t* vehChassisGen; // its generation at create
    int32_t* vehWheelCount;
    m3real* vehMaxSteer;
    m3real* vehDriveForce;
    m3real* vehBrakeForce;
    uint64_t* vehUserData;
    m3Vec3* vehWheelAnchor;
    m3Vec3* vehWheelDir;
    m3real* vehWheelRest;
    m3real* vehWheelTravel;
    m3real* vehWheelHertz;
    m3real* vehWheelZeta;
    m3real* vehWheelRadius;
    uint8_t* vehWheelFlags; // bit0 steerable, bit1 driven
    m3real* vehWheelBrake;
    // Tank mode: per-side throttles; mode 0 = the normal
    // single-throttle path.
    uint8_t* vehTrackMode;
    m3real* vehTrackLeft;
    m3real* vehTrackRight;
    m3real* vehLeanGain;
    m3real* vehWheelCompression; // last step's suspension state
    uint8_t* vehWheelContact;
    m3real* vehTireGrip; // friction circle scale
    m3real* vehThrottle; // command state, journaled
    m3real* vehSteer;
    m3real* vehBrake;
    m3real* vehWheelSpin; // accumulated spin angle, render state
    // The drivetrain: per-vehicle engine curve and gearbox.
    // Def fields land by journaled op; gear, clutch countdown, and
    // last engine speed are simulation state (snapshot always,
    // hashed only when a drivetrain is attached).
    uint8_t* vehDtActive;
    int32_t* vehDtCurveCount;
    m3real* vehDtCurveRpm;    // capacity * M3_DRIVETRAIN_MAX_CURVE
    m3real* vehDtCurveTorque; // capacity * M3_DRIVETRAIN_MAX_CURVE
    int32_t* vehDtGearCount;
    m3real* vehDtGearRatio; // capacity * M3_DRIVETRAIN_MAX_GEARS
    m3real* vehDtReverse;
    m3real* vehDtFinal;
    int32_t* vehDtDiffMode;  // 0 open, 1 limited, 2 locked
    m3real* vehDtDiffCouple; // newtons per m/s of wheel disparity
    m3real* vehWheelLon;     // per wheel: last step's contact speed
                             // (cap * M3_VEHICLE_MAX_WHEELS); feeds
                             // the diff coupling, hashed only when a
                             // nonzero diff mode is attached
    m3real* vehDtShiftUp;
    m3real* vehDtShiftDown;
    int32_t* vehDtClutchSteps;
    uint8_t* vehDtAutoShift;
    int8_t* vehDtGear;    // -1 reverse, 0 neutral, 1..gearCount
    int32_t* vehDtClutch; // torque-cut countdown, steps
    m3real* vehDtRpm;     // engine speed from the last pass
} m3Vehicles;

// Soft bodies: XPBD particle lattices in fixed-size blocks per slot.
typedef struct m3SoftBodies
{
    // Soft bodies: XPBD particle lattices, fixed-size blocks
    // per slot (the snapshot walker's simplest shape). Positions
    // and previous positions in double like every body; all of it
    // persistent state, hashed for live slots only.
    int32_t softBodyCapacity;
    m3IdPool softPool;
    int32_t* softParticleCount;
    int32_t* softEdgeCount;
    m3real* softCompliance;
    m3real* softRadius;
    m3real* softGravityScale;
    uint64_t* softUserData;
    m3Pos3* softPos;     // cap * M3_SOFTBODY_MAX_PARTICLES
    m3Pos3* softPrev;    // cap * M3_SOFTBODY_MAX_PARTICLES
    m3real* softInvMass; // cap * M3_SOFTBODY_MAX_PARTICLES
    // Bend tethers: edges from softBendStart[slot] onward
    // solve with softBendCompliance[slot] instead of the stretch
    // compliance.
    int32_t* softBendStart;
    m3real* softBendCompliance;
    // Pressure: lattice dims + the create volume + target
    // multiplier; all folded off-default.
    uint16_t* softDimX;
    uint16_t* softDimY;
    uint16_t* softDimZ;
    m3real* softRestVolume;
    m3real* softPressure;
    // Tet volumes: four particle ids and a rest 6-volume per
    // tet, slot-strided like edges; count 0 = a lattice body.
    int32_t* softTetCount;
    uint16_t* softTetA;
    uint16_t* softTetB;
    uint16_t* softTetC;
    uint16_t* softTetD;
    m3real* softTetRestV6;
    // Bind-pose tether: the create positions and the clamp
    // radius; radius 0 = off and the bind block stays canonical
    // zeros.
    m3Pos3* softBindPos;
    m3real* softMaxDeviation;
    m3Vec3* softKick;    // pending explosion velocity kicks:
                         // integrated once next step, then zeroed,
                         // hashed only when nonzero (additive rule)
    uint16_t* softEdgeA; // cap * M3_SOFTBODY_MAX_EDGES
    uint16_t* softEdgeB;
    m3real* softEdgeRest;
    int32_t* softAnchorCount;    // per slot
    int32_t* softAnchorParticle; // cap * M3_SOFTBODY_MAX_ANCHORS
    int32_t* softAnchorBody;
    uint16_t* softAnchorGen;
    m3Vec3* softAnchorLocal;
    // Soft-to-soft anchors: lattice A's particle pinned to
    // lattice B's particle, released silently when EITHER side
    // dies.
    int32_t* softSoftCount;     // per slot (owner = LOWER slot)
    int32_t* softSoftParticleA; // cap * M3_SOFTBODY_MAX_ANCHORS
    int32_t* softSoftSlotB;
    uint16_t* softSoftGenB;
    int32_t* softSoftParticleB;
} m3SoftBodies;

// Water volumes: fixed slots of world-anchored boxes.
typedef struct m3WaterVolumes
{
    // Water volumes: fixed 8 slots, world-anchored boxes.
    // Hashed only while any volume is alive; fixed snapshot blocks.
    m3IdPool waterPool;
    m3Pos3 waterLo[M3_MAX_WATER_VOLUMES];
    m3Pos3 waterHi[M3_MAX_WATER_VOLUMES];
    float waterDensity[M3_MAX_WATER_VOLUMES];
    float waterLinDrag[M3_MAX_WATER_VOLUMES];
    float waterAngDrag[M3_MAX_WATER_VOLUMES];
    m3Vec3 waterFlow[M3_MAX_WATER_VOLUMES];
} m3WaterVolumes;

// Event streams: transient observers, never snapshotted.
typedef struct m3Events
{
    // Contact events (transient observers, never snapshotted;
    // cleared on step and on restore).
    m3ContactHitEvent* hitEvents; // hit events, transient observers
    int32_t hitEventCount;
    int32_t hitEventsDropped;
    m3BodyMoveEvent* moveEvents;
    int32_t moveEventCount;
    m3ContactBeginEvent* beginEvents;
    m3ContactEndEvent* endEvents;
    m3ContactBeginEvent* sensorBeginEvents;
    m3ContactEndEvent* sensorEndEvents;
    int32_t beginEventCount;
    int32_t endEventCount;
    // Fragment events: transient like every event stream;
    // cleared by the next step and by restore, never snapshotted.
    struct m3FragmentEvent* fragmentEvents;
    uint16_t* fragmentRecipe;
    int32_t fragmentEventCount;
    int32_t fragmentRecipeCount;
    int32_t fragmentDropped;
    int32_t sensorBeginEventCount;
    int32_t sensorEndEventCount;
} m3Events;

// The journal cursor over a caller-owned buffer.
typedef struct m3Recorder
{
    // transient (restore zeroes it, documented)

    // Journal cursor over a caller-owned buffer.
    uint8_t* journalBuffer;
    int32_t journalCapacity;
    int32_t journalCursor;
    int32_t journalActive;
    int32_t journalOverflow;
} m3Recorder;

// The world: global state and knobs, then one block per subsystem. Every
// array is described once in the state table (world_state.c).
typedef struct m3World
{
    // World-global state (snapshot header material).
    m3Vec3 gravity;
    // Tuning knobs: state, not config. They journal, they snapshot,
    // and they fold into the hash only off their defaults. The config
    // hash stays version + solver revision + precision + FP policy:
    // knobs must be replayable, and a config-hash knob would refuse the
    // journal instead.
    float contactHertz;
    float contactDampingRatio;
    float contactPushMaxSpeed;
    float restitutionThreshold;
    float maximumLinearSpeed;
    float maximumAngularSpeed; // rad/s spin cap
    uint8_t sleepEnabled;
    uint8_t continuousEnabled;
    float hitEventThreshold; // approach speed gate for hit events
    // Wind: a deterministic field. The gust phase
    // ACCUMULATES as state so a rollback resumes the same wave;
    // speed zero means no wind and nothing folds into the hash.
    m3Vec3 windDir;
    float windSpeed;
    float windGustHertz;
    float windGustScale;
    float windPhase;
    uint64_t stepCount;
    int32_t workerCount;
    m3EnqueueTaskFn* enqueueTask; // host threading hooks (never state)
    m3FinishTaskFn* finishTask;
    void* userTaskContext;
    uint16_t generation; // this world slot's generation
    uint16_t slot;       // 0-based slot in the world table
    uint16_t idWorld;    // the world field of every id this world hands out
    // Persistent bytes this world holds (arrays at create plus
    // count-derived content while it lives) and the step scratch
    // capacity; the scratch PEAK already rides m3Counters.
    int64_t memoryBytes;
    m3PreSolveFn* preSolveFn; // host wiring, never state (like the task hooks)
    void* preSolveContext;
    // Per-step scratch, never snapshotted.
    m3Stack scratch;
    m3real lastInvH; // last step's substep invH: readback scale,
    // Observer tail: profile and step statistics. NEVER
    // snapshot blocks, NEVER hash inputs; the walker and the hash
    // enumerate arrays explicitly and skip this region by design.
    m3Profile profile;       // last completed step
    int32_t lastIslandCount; // last completed step
    int32_t lastColorCount;  // last completed step
    int32_t lastScratchPeak; // bytes, last completed step
    volatile long long
        misuseCount; // cumulative refusals; atomic access only (m3Refuse, m3MisuseCount)

    m3Bodies bodies;
    m3Shapes shapes;
    m3Hulls hulls;
    m3Meshes meshes;
    m3HeightFields heightFields;
    m3Voxels voxels;
    m3Broadphase broadphase;
    m3Contacts contacts;
    m3Joints joints;
    m3Characters characters;
    m3Vehicles vehicles;
    m3SoftBodies softBodies;
    m3WaterVolumes water;
    m3Events events;
    m3Recorder recorder;
} m3World;

#endif // MAUL3D_SRC_WORLD_INTERNAL_H
