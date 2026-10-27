// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The per-record half of restore's whole-structure checks; the links are
// in snapshot_links.c, the magnitudes in snapshot_bounds.c. Each live
// record's values must be ones its setters accept, which the per-block
// checks (world_state.c) cannot know: body locks, unit rotations, shape
// geometry its create accepted, and the indices that characters,
// vehicles, soft bodies and joints hold landing where the engine would
// have put them.

#include "snapshot_values.h"

#include "body_state.h"
#include "joint_solver.h"
#include "shape.h"

#include <stddef.h>
#include <string.h>

// --- Bodies and shape geometry --------------------------------------------

// A live body's locks are the six axis bits and the fast rotation bit.
static bool BodiesValid(const m3Incoming* inc)
{
    const uint8_t* alive = m3InField(inc, offsetof(m3World, bodies.bodyPool.alive));
    const uint8_t* locks = m3InField(inc, offsetof(m3World, bodies.bodyLocks));
    for (int32_t b = 0; b < inc->world->bodies.bodyPool.capacity; ++b)
    {
        if (alive[b] != 0 && (locks[b] & ~(0x3Fu | M3_LOCKS_ALLOW_FAST_ROTATION)) != 0)
        {
            return false;
        }
    }
    return true;
}

// A live shape's geometry is what its create wrote, by the same check
// create runs, and a voxel chunk's cell is its content's.
static bool ShapeGeomValid(const m3Incoming* inc)
{
    const uint8_t* alive = m3InField(inc, offsetof(m3World, shapes.shapePool.alive));
    const uint8_t* types = m3InField(inc, offsetof(m3World, shapes.shapeType));
    const uint8_t* geoms = m3InField(inc, offsetof(m3World, shapes.shapeGeom));
    const uint8_t* voxelIndex = m3InField(inc, offsetof(m3World, shapes.shapeVoxelIndex));
    const uint8_t* voxelData = m3InField(inc, offsetof(m3World, voxels.voxelData));
    for (int32_t s = 0; s < inc->world->shapes.shapePool.capacity; ++s)
    {
        if (alive[s] == 0)
        {
            continue;
        }
        m3ShapeGeom geom;
        memcpy(&geom, geoms + (size_t)s * sizeof(m3ShapeGeom), sizeof(geom));
        uint8_t type = types[s];
        if (!m3ShapeGeomValid(type, &geom))
        {
            return false;
        }
        if (type == (uint8_t)m3_voxelShape)
        {
            // ContentValid has put the index in range and its slot alive.
            m3real cell;
            memcpy(&cell,
                   voxelData + (size_t)m3InI32(voxelIndex, s) * sizeof(m3VoxelChunkData) +
                       offsetof(m3VoxelChunkData, cellSize),
                   sizeof(cell));
            if (geom.s != cell)
            {
                return false;
            }
        }
    }
    return true;
}

// --- Rotations -----------------------------------------------------------------

// A rotation the engine could hold: within the band of unit length its
// setters accept. Rotation code assumes unit length; a long one scales
// what it rotates.
static bool UnitQuat(const uint8_t* block, size_t stride, size_t at, int32_t i)
{
    m3Quat q;
    memcpy(&q, block + stride * (size_t)i + at, sizeof(q));
    m3real qq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return qq > 0.98f && qq < 1.02f;
}

// Every live body's pose and, when it has one, its target; every live
// shape's offset; every live joint's frames and drive target, but a wheel
// joint's, which holds its steering drive's hertz, damping and effort.
static bool RotationsValid(const m3Incoming* inc)
{
    const m3World* w = inc->world;
    const uint8_t* bodyAlive = m3InField(inc, offsetof(m3World, bodies.bodyPool.alive));
    const uint8_t* poses = m3InField(inc, offsetof(m3World, bodies.transforms));
    const uint8_t* hasTarget = m3InField(inc, offsetof(m3World, bodies.bodyHasTarget));
    const uint8_t* targets = m3InField(inc, offsetof(m3World, bodies.bodyTarget));
    size_t xf = sizeof(m3Transform);
    size_t rot = offsetof(m3Transform, q);
    for (int32_t b = 0; b < w->bodies.bodyPool.capacity; ++b)
    {
        if (bodyAlive[b] != 0 &&
            (!UnitQuat(poses, xf, rot, b) || (hasTarget[b] != 0 && !UnitQuat(targets, xf, rot, b))))
        {
            return false;
        }
    }
    const uint8_t* shapeAlive = m3InField(inc, offsetof(m3World, shapes.shapePool.alive));
    const uint8_t* offsets = m3InField(inc, offsetof(m3World, shapes.shapeLocalRot));
    for (int32_t s = 0; s < w->shapes.shapePool.capacity; ++s)
    {
        if (shapeAlive[s] != 0 && !UnitQuat(offsets, sizeof(m3Quat), 0, s))
        {
            return false;
        }
    }
    const uint8_t* jointAlive = m3InField(inc, offsetof(m3World, joints.jointPool.alive));
    const uint8_t* frameA = m3InField(inc, offsetof(m3World, joints.jointFrameQA));
    const uint8_t* frameB = m3InField(inc, offsetof(m3World, joints.jointFrameQB));
    const uint8_t* target = m3InField(inc, offsetof(m3World, joints.jointTargetQ));
    const uint8_t* types = m3InField(inc, offsetof(m3World, joints.jointType));
    for (int32_t j = 0; j < w->joints.jointPool.capacity; ++j)
    {
        if (jointAlive[j] != 0 &&
            (!UnitQuat(frameA, sizeof(m3Quat), 0, j) || !UnitQuat(frameB, sizeof(m3Quat), 0, j) ||
             (types[j] != (uint8_t)m3_wheelJoint && !UnitQuat(target, sizeof(m3Quat), 0, j))))
        {
            return false;
        }
    }
    return true;
}

// --- Characters and vehicles ---------------------------------------------

// A live character owns a live kinematic body whose only shape is its
// capsule (no call may destroy, retype, reshape or add to them on their
// own), keeps the skin, snap and step its create accepted, and stands on
// nothing or on a live body (a body's destroy clears the characters
// standing on it). Its capsule and mass are bounded in snapshot_bounds.c.
static bool CharactersValid(const m3Incoming* inc)
{
    const uint8_t* alive = m3InField(inc, offsetof(m3World, characters.charPool.alive));
    const uint8_t* body = m3InField(inc, offsetof(m3World, characters.charBody));
    const uint8_t* ground = m3InField(inc, offsetof(m3World, characters.charGroundBody));
    const uint8_t* skin = m3InField(inc, offsetof(m3World, characters.charSkin));
    const uint8_t* snap = m3InField(inc, offsetof(m3World, characters.charSnap));
    const uint8_t* stepHeight = m3InField(inc, offsetof(m3World, characters.charStepHeight));
    const uint8_t* bodyAlive = m3InField(inc, offsetof(m3World, bodies.bodyPool.alive));
    const uint8_t* bodyTypes = m3InField(inc, offsetof(m3World, bodies.types));
    const uint8_t* shapeHead = m3InField(inc, offsetof(m3World, bodies.bodyShapeHead));
    const uint8_t* shapeType = m3InField(inc, offsetof(m3World, shapes.shapeType));
    const uint8_t* shapeNext = m3InField(inc, offsetof(m3World, shapes.shapeNext));
    for (int32_t c = 0; c < inc->world->characters.charPool.capacity; ++c)
    {
        if (alive[c] == 0)
        {
            continue;
        }
        int32_t b = m3InI32(body, c);
        int32_t under = m3InI32(ground, c);
        int32_t capsule = b < 0 ? -1 : m3InI32(shapeHead, b);
        if (b < 0 || bodyAlive[b] == 0 || bodyTypes[b] != (uint8_t)m3_kinematicBody ||
            capsule < 0 || shapeType[capsule] != (uint8_t)m3_capsuleShape ||
            m3InI32(shapeNext, capsule) != -1 || (under != -1 && bodyAlive[under] == 0))
        {
            return false;
        }
        if (!(m3InF32(skin, c) > 0.0f) || m3InF32(snap, c) < 0.0f || m3InF32(stepHeight, c) < 0.0f)
        {
            return false;
        }
    }
    return true;
}

// A live vehicle rides a live chassis of the generation it names (a
// chassis's destroy takes its vehicle with it), has the wheel count its
// create accepted and wheel flags that are the steer and drive bits. Its
// drivetrain, when set, has the curve and gear counts its setter
// accepted, a gear between reverse and the top one and a clutch inside
// its steps; a vehicle without one holds the zeros the reset wrote.
static bool VehiclesValid(const m3Incoming* inc)
{
    const uint8_t* alive = m3InField(inc, offsetof(m3World, vehicles.vehPool.alive));
    const uint8_t* wheelCount = m3InField(inc, offsetof(m3World, vehicles.vehWheelCount));
    const uint8_t* wheelFlags = m3InField(inc, offsetof(m3World, vehicles.vehWheelFlags));
    const uint8_t* active = m3InField(inc, offsetof(m3World, vehicles.vehDtActive));
    const uint8_t* curveCount = m3InField(inc, offsetof(m3World, vehicles.vehDtCurveCount));
    const uint8_t* gearCount = m3InField(inc, offsetof(m3World, vehicles.vehDtGearCount));
    const uint8_t* diffMode = m3InField(inc, offsetof(m3World, vehicles.vehDtDiffMode));
    const uint8_t* clutchSteps = m3InField(inc, offsetof(m3World, vehicles.vehDtClutchSteps));
    const uint8_t* clutch = m3InField(inc, offsetof(m3World, vehicles.vehDtClutch));
    const uint8_t* gears = m3InField(inc, offsetof(m3World, vehicles.vehDtGear));
    const uint8_t* chassis = m3InField(inc, offsetof(m3World, vehicles.vehChassis));
    const uint8_t* chassisGen = m3InField(inc, offsetof(m3World, vehicles.vehChassisGen));
    const uint8_t* bodyAlive = m3InField(inc, offsetof(m3World, bodies.bodyPool.alive));
    const uint8_t* bodyGen = m3InField(inc, offsetof(m3World, bodies.bodyPool.generations));
    for (int32_t v = 0; v < inc->world->vehicles.vehPool.capacity; ++v)
    {
        if (alive[v] == 0)
        {
            continue;
        }
        int32_t wheels = m3InI32(wheelCount, v);
        int32_t body = m3InI32(chassis, v);
        if (wheels < 1 || body < 0 || bodyAlive[body] == 0 ||
            m3InU32(bodyGen, body) != m3InU32(chassisGen, v))
        {
            return false;
        }
        for (int32_t w = 0; w < wheels; ++w)
        {
            if ((wheelFlags[v * M3_VEHICLE_MAX_WHEELS + w] & ~3u) != 0)
            {
                return false;
            }
        }
        int32_t gear = gears[v] < 128 ? gears[v] : gears[v] - 256; // the int8 the byte holds
        int32_t steps = m3InI32(clutchSteps, v);
        if (active[v] == 0)
        {
            if (m3InI32(curveCount, v) != 0 || m3InI32(gearCount, v) != 0 ||
                m3InI32(diffMode, v) != 0 || steps != 0 || m3InI32(clutch, v) != 0 || gear != 0)
            {
                return false;
            }
            continue;
        }
        if (m3InI32(curveCount, v) < 2 || m3InI32(gearCount, v) < 1 || m3InI32(diffMode, v) < 0 ||
            m3InI32(diffMode, v) > 2 || steps < 0 || steps > 600 || m3InI32(clutch, v) < 0 ||
            m3InI32(clutch, v) > steps || gear < -1 || gear > m3InI32(gearCount, v))
        {
            return false;
        }
    }
    return true;
}

// --- Soft bodies -------------------------------------------------------------

typedef struct SoftFields
{
    const uint8_t* alive;
    const uint8_t* generations;
    const uint8_t* particles;
    const uint8_t* edgeCount;
    const uint8_t* bendStart;
    const uint8_t* tetCount;
    const uint8_t* pressure;
    const uint8_t* restVolume;
    const uint8_t* dims[3];
    const uint8_t* edgeEnds[2];
    const uint8_t* tetCorners[4];
    const uint8_t* anchorCount;
    const uint8_t* anchorParticle;
    const uint8_t* softCount;
    const uint8_t* particleA;
    const uint8_t* slotB;
    const uint8_t* genB;
    const uint8_t* particleB;
} SoftFields;

static SoftFields SoftFieldsOf(const m3Incoming* inc)
{
    SoftFields f;
    f.alive = m3InField(inc, offsetof(m3World, softBodies.softPool.alive));
    f.generations = m3InField(inc, offsetof(m3World, softBodies.softPool.generations));
    f.particles = m3InField(inc, offsetof(m3World, softBodies.softParticleCount));
    f.edgeCount = m3InField(inc, offsetof(m3World, softBodies.softEdgeCount));
    f.bendStart = m3InField(inc, offsetof(m3World, softBodies.softBendStart));
    f.tetCount = m3InField(inc, offsetof(m3World, softBodies.softTetCount));
    f.pressure = m3InField(inc, offsetof(m3World, softBodies.softPressure));
    f.restVolume = m3InField(inc, offsetof(m3World, softBodies.softRestVolume));
    f.dims[0] = m3InField(inc, offsetof(m3World, softBodies.softDimX));
    f.dims[1] = m3InField(inc, offsetof(m3World, softBodies.softDimY));
    f.dims[2] = m3InField(inc, offsetof(m3World, softBodies.softDimZ));
    f.edgeEnds[0] = m3InField(inc, offsetof(m3World, softBodies.softEdgeA));
    f.edgeEnds[1] = m3InField(inc, offsetof(m3World, softBodies.softEdgeB));
    f.tetCorners[0] = m3InField(inc, offsetof(m3World, softBodies.softTetA));
    f.tetCorners[1] = m3InField(inc, offsetof(m3World, softBodies.softTetB));
    f.tetCorners[2] = m3InField(inc, offsetof(m3World, softBodies.softTetC));
    f.tetCorners[3] = m3InField(inc, offsetof(m3World, softBodies.softTetD));
    f.anchorCount = m3InField(inc, offsetof(m3World, softBodies.softAnchorCount));
    f.anchorParticle = m3InField(inc, offsetof(m3World, softBodies.softAnchorParticle));
    f.softCount = m3InField(inc, offsetof(m3World, softBodies.softSoftCount));
    f.particleA = m3InField(inc, offsetof(m3World, softBodies.softSoftParticleA));
    f.slotB = m3InField(inc, offsetof(m3World, softBodies.softSoftSlotB));
    f.genB = m3InField(inc, offsetof(m3World, softBodies.softSoftGenB));
    f.particleB = m3InField(inc, offsetof(m3World, softBodies.softSoftParticleB));
    return f;
}

// The live soft body in slot s: its edges and tets name its own
// particles, its bend tethers start inside the edges, a pressured body is
// a closed lattice whose dimensions multiply to its particle count, and
// only a lattice has a closed rest volume.
static bool SoftShapeValid(const SoftFields* f, int32_t s)
{
    int32_t count = m3InI32(f->particles, s);
    int32_t edges = m3InI32(f->edgeCount, s);
    if (m3InI32(f->bendStart, s) > edges || m3InF32(f->pressure, s) < 0.0f)
    {
        return false;
    }
    // In 64 bits, where three 16-bit factors always fit.
    int64_t nx = m3InU16(f->dims[0], s);
    int64_t ny = m3InU16(f->dims[1], s);
    int64_t nz = m3InU16(f->dims[2], s);
    bool lattice = nx != 0 || ny != 0 || nz != 0;
    if ((lattice && nx * ny * nz != count) ||
        (m3InF32(f->pressure, s) > 0.0f && (nx < 2 || ny < 2 || nz < 2)) ||
        (!lattice && m3InF32(f->restVolume, s) != 0.0f))
    {
        return false;
    }
    for (int32_t e = 0; e < edges; ++e)
    {
        int32_t k = s * M3_SOFTBODY_MAX_EDGES + e;
        if (m3InU16(f->edgeEnds[0], k) >= count || m3InU16(f->edgeEnds[1], k) >= count)
        {
            return false;
        }
    }
    for (int32_t t = 0; t < m3InI32(f->tetCount, s); ++t)
    {
        for (int32_t c = 0; c < 4; ++c)
        {
            if (m3InU16(f->tetCorners[c], s * M3_SOFTBODY_MAX_TETS + t) >= count)
            {
                return false;
            }
        }
    }
    return true;
}

// Its anchors name its own particles, and each soft-to-soft anchor a
// particle of the partner too while the partner is the live one of the
// generation it names.
static bool SoftAnchorsValid(const SoftFields* f, int32_t s)
{
    int32_t count = m3InI32(f->particles, s);
    for (int32_t a = 0; a < m3InI32(f->anchorCount, s); ++a)
    {
        int32_t particle = m3InI32(f->anchorParticle, s * M3_SOFTBODY_MAX_ANCHORS + a);
        if (particle < 0 || particle >= count)
        {
            return false;
        }
    }
    for (int32_t a = 0; a < m3InI32(f->softCount, s); ++a)
    {
        int32_t k = s * M3_SOFTBODY_MAX_ANCHORS + a;
        int32_t other = m3InI32(f->slotB, k);
        int32_t pa = m3InI32(f->particleA, k);
        int32_t pb = m3InI32(f->particleB, k);
        if (pa < 0 || pa >= count || other < 0 || pb < 0)
        {
            return false;
        }
        if (f->alive[other] != 0 && m3InU32(f->generations, other) == m3InU32(f->genB, k) &&
            pb >= m3InI32(f->particles, other))
        {
            return false;
        }
    }
    return true;
}

static bool SoftBodiesValid(const m3Incoming* inc)
{
    SoftFields f = SoftFieldsOf(inc);
    for (int32_t s = 0; s < inc->world->softBodies.softPool.capacity; ++s)
    {
        if (f.alive[s] != 0 && (!SoftShapeValid(&f, s) || !SoftAnchorsValid(&f, s)))
        {
            return false;
        }
    }
    return true;
}

// --- Joint modes ---------------------------------------------------------------

// A live joint's flags are the limit, motor, steer and spring bits. A
// generic joint's packed modes name no axis mode past limited and a
// motor on a real axis or none; every other joint packs none. A gear
// joint's reference spins are the wrapped angles its create took, which
// the step's wrap walks down to the seam.
static bool JointModesValid(const m3Incoming* inc)
{
    const uint8_t* alive = m3InField(inc, offsetof(m3World, joints.jointPool.alive));
    const uint8_t* types = m3InField(inc, offsetof(m3World, joints.jointType));
    const uint8_t* flags = m3InField(inc, offsetof(m3World, joints.jointFlags));
    const uint8_t* modes = m3InField(inc, offsetof(m3World, joints.jointGenericModes));
    const uint8_t* motors = m3InField(inc, offsetof(m3World, joints.jointMotor));
    uint32_t known = M3_JOINT_LIMIT | M3_JOINT_MOTOR | M3_JOINT_STEER | M3_JOINT_SPRING;
    for (int32_t j = 0; j < inc->world->joints.jointPool.capacity; ++j)
    {
        if (alive[j] == 0)
        {
            continue;
        }
        if ((flags[j] & ~known) != 0)
        {
            return false;
        }
        uint32_t packed = m3InU16(modes, j);
        if (types[j] != (uint8_t)m3_genericJoint)
        {
            if (packed != 0)
            {
                return false;
            }
        }
        else
        {
            for (int32_t k = 0; k < 6; ++k)
            {
                if (((packed >> (2 * k)) & 3u) == 3u)
                {
                    return false;
                }
            }
            uint32_t axis = packed >> 12;
            if (axis > 5 && axis != 15)
            {
                return false;
            }
        }
        if (types[j] == (uint8_t)m3_gearJoint)
        {
            // x and y of the motor vector: the spins at create.
            float spinA = m3InF32(motors, 3 * j);
            float spinB = m3InF32(motors, 3 * j + 1);
            if (!(spinA >= -4.0f && spinA <= 4.0f && spinB >= -4.0f && spinB <= 4.0f))
            {
                return false;
            }
        }
    }
    return true;
}

bool m3SnapshotValuesValid(const m3Incoming* inc)
{
    return BodiesValid(inc) && RotationsValid(inc) && ShapeGeomValid(inc) && CharactersValid(inc) &&
           VehiclesValid(inc) && SoftBodiesValid(inc) && JointModesValid(inc);
}
