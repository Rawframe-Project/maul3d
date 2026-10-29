// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The spherical joint's cone: its switch and its angle. Public functions
// validate and journal; the internal setter in joint_params.c mutates and
// is what replay drives.

#include "fpenv.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "misuse.h"
#include "world_internal.h"

#include <string.h>

static bool ResolveJoint(const m3World* world, m3JointId id, int32_t* slotOut)
{
    int32_t slot = m3JointSlot(world, id);
    if (slot < 0)
    {
        m3Refuse(world, m3_errorInvalid);
        return false;
    }
    *slotOut = slot;
    return true;
}

// The switch rides the cone bit and the angle the limit slot's z. Each
// call records the whole cone: the part it sets and the rest as the joint
// holds it. SetConeAngle takes no negative angle, and EnableCone refuses
// to switch on over one a def left.
static M3_NOINLINE void SphericalJointSetCone(m3World* world, m3JointId joint, bool setsSwitch,
                                              bool enable, float angle)
{
    if (m3WorldBusy(world))
    {
        m3Refuse(world, m3_errorBusy);
        return;
    }
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return;
    }
    int32_t held = (world->joints.jointFlags[slot] & M3_JOINT_CONE) != 0 ? 1 : 0;
    int32_t on = setsSwitch ? (enable ? 1 : 0) : held;
    float value = setsSwitch ? world->joints.jointLimits[slot].z : angle;
    if (world->joints.jointType[slot] != (uint8_t)m3_sphericalJoint ||
        (!setsSwitch && !(angle >= 0.0f)) || !m3SphericalConeFits(on, value))
    {
        m3Refuse(world, m3_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m3OpSphericalJointSetCone record;
        memset(&record, 0, sizeof(record));
        record.id = joint;
        record.enable = on;
        record.angle = value;
        m3JournalRecord(world, m3_opSphericalJointSetCone, &record, (int32_t)sizeof(record));
    }
    m3SphericalJointSetConeInternal(world, slot, on, value);
}

void m3SphericalJoint_EnableCone(m3World* world, m3JointId joint, bool enable)
{
    uint64_t env = m3FloatEnvEnter();
    SphericalJointSetCone(world, joint, true, enable, 0);
    m3FloatEnvLeave(env);
}

void m3SphericalJoint_SetConeAngle(m3World* world, m3JointId joint, float angle)
{
    uint64_t env = m3FloatEnvEnter();
    SphericalJointSetCone(world, joint, false, false, angle);
    m3FloatEnvLeave(env);
}

// The slot of a live spherical joint, or -1 after counting one misuse for
// a stale id or another type.
static int32_t SphericalSlot(const m3World* world, m3JointId joint)
{
    int32_t slot;
    if (!ResolveJoint(world, joint, &slot))
    {
        return -1;
    }
    if (world->joints.jointType[slot] != (uint8_t)m3_sphericalJoint)
    {
        m3Refuse(world, m3_errorInvalid);
        return -1;
    }
    return slot;
}

// False for a stale id and for a joint that is not spherical, each of
// which counts one misuse.
bool m3SphericalJoint_IsConeEnabled(const m3World* world, m3JointId joint)
{
    int32_t slot = SphericalSlot(world, joint);
    return slot >= 0 && (world->joints.jointFlags[slot] & M3_JOINT_CONE) != 0;
}

// Zero for a stale id and for a joint that is not spherical, each of which
// counts one misuse.
float m3SphericalJoint_GetConeAngle(const m3World* world, m3JointId joint)
{
    int32_t slot = SphericalSlot(world, joint);
    return slot >= 0 ? world->joints.jointLimits[slot].z : 0.0f;
}
