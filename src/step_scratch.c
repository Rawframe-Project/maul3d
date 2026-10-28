// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "step_scratch.h"

#include "broad_phase.h"
#include "contact_kernel.h"
#include "joint_solver.h"

int64_t m3StepScratchBytes(const m3World* world)
{
    // Every array a step takes from its scratch, at the world's
    // capacities: per body, per shape, per pair and per joint, with the
    // contact blocks of every color at most one short of full. The pair
    // update's cache is given back before the rest is taken.
    int64_t bodies = world->bodies.bodyCapacity;
    int64_t pairs = world->contacts.pairCapacity;
    int64_t blocks = (pairs + (int64_t)M3_GRAPH_COLORS * M3_LANES) / M3_LANES;
    int64_t solve =
        m3StackBytes(bodies, (int64_t)sizeof(m3Pos3)) +  // sweep start centers
        m3StackBytes(bodies, (int64_t)sizeof(m3Quat)) +  // and rotations
        m3StackBytes(bodies, (int64_t)sizeof(int32_t)) + // island parents
        2 * m3StackBytes(bodies, 1) +                    // wake and sleep flags
        m3StackBytes(pairs, (int64_t)sizeof(m3ContactConstraint)) +
        m3StackBytes(bodies, (int64_t)sizeof(m3Vec3)) +   // position deltas
        m3StackBytes(bodies, (int64_t)sizeof(m3Quat)) +   // rotation deltas
        m3StackBytes(pairs, 1) +                          // contact colors
        m3StackBytes(bodies, (int64_t)sizeof(uint32_t)) + // colors used per body
        m3StackBytes(pairs, (int64_t)sizeof(int32_t)) +   // contacts by color
        m3StackBytes(blocks, (int64_t)sizeof(m3ContactBlock)) +
        m3StackBytes(world->joints.jointCapacity, (int64_t)sizeof(m3JointConstraint)) +
        m3StackBytes(bodies, (int64_t)sizeof(int32_t)) +    // movers
        3 * m3StackBytes(bodies, (int64_t)sizeof(m3Vec3)) + // buoyancy force, torque, flow
        2 * m3StackBytes(bodies, (int64_t)sizeof(float));   // buoyancy drag
    int64_t pairUpdate = m3PairUpdateScratchBytes(world->shapes.shapeCapacity);
    return solve > pairUpdate ? solve : pairUpdate;
}
