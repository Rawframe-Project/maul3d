// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The generic joint: six axes, each locked, limited or free, with one
// optional motor.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include <math.h>
#include <string.h>

static void PrepareGeneric(m3World* world, m3JointConstraint* c, const m3JointFrame* f)
{
    int32_t j = f->joint;
    const m3Transform* xfA = f->xfA;
    const m3Transform* xfB = f->xfB;
    // The 6-DOF: the joint frame's world basis rides
    // in the three axis slots; impulses ride the slot map
    // documented at the store.
    c->frameQA = m3MulQuat(xfA->q, world->joints.jointFrameQA[j]);
    c->frameQB = m3MulQuat(xfB->q, world->joints.jointFrameQB[j]);
    c->perpAxisX = m3RotateVec3(c->frameQA, (m3Vec3){1.0f, 0.0f, 0.0f});
    c->perpAxisY = m3RotateVec3(c->frameQA, (m3Vec3){0.0f, 1.0f, 0.0f});
    c->rotationAxis = m3RotateVec3(c->frameQA, (m3Vec3){0.0f, 0.0f, 1.0f});
    c->genModes = world->joints.jointGenericModes[j];
    c->genLinLower = world->joints.jointGenLinLower[j];
    c->genLinUpper = world->joints.jointGenLinUpper[j];
    c->genAngLower = world->joints.jointGenAngLower[j];
    c->genAngUpper = world->joints.jointGenAngUpper[j];
    c->perpImpulseX = world->joints.jointPerpImpulse[j].x;
    c->perpImpulseY = world->joints.jointPerpImpulse[j].y;
    c->swingImpulse = world->joints.jointPerpImpulse[j].z;
    c->angularImpulse = world->joints.jointAngularImpulse[j];
    c->upperImpulse = world->joints.jointLimitImpulse[j].x;
    c->motorImpulse = world->joints.jointLimitImpulse[j].y;
    c->motorSpeed = world->joints.jointMotor[j].x;
    c->maxMotorEffort = world->joints.jointMotor[j].y;
}

static void WarmStartGeneric(const m3World* world, const m3JointConstraint* c,
                             m3JointWarmContext* w)
{
    (void)world; // the shared tail applies the impulse
    // Slot map: impulse.xyz = linear primary (equality or
    // lower), perpX/perpY/swing = linear uppers,
    // w->angularImpulse = angular primaries, upperImpulse =
    // the one angular upper, motorImpulse = the motor.
    const m3Vec3 basis[3] = {c->perpAxisX, c->perpAxisY, c->rotationAxis};
    const m3real prim[3] = {c->impulse.x, c->impulse.y, c->impulse.z};
    const m3real lup[3] = {c->perpImpulseX, c->perpImpulseY, c->swingImpulse};
    const m3real aprim[3] = {c->angularImpulse.x, c->angularImpulse.y, c->angularImpulse.z};
    m3Vec3 P = {0.0f, 0.0f, 0.0f};
    m3Vec3 L = {0.0f, 0.0f, 0.0f};
    for (int32_t k = 0; k < 3; ++k)
    {
        uint32_t lmode = (uint32_t)(c->genModes >> (2 * k)) & 3u;
        uint32_t amode = (uint32_t)(c->genModes >> (6 + 2 * k)) & 3u;
        if (lmode != 1u)
        {
            P = m3Add3(P, m3MulSV3(prim[k] - lup[k], basis[k]));
        }
        if (amode != 1u)
        {
            L = m3Add3(L, m3MulSV3(aprim[k], basis[k]));
        }
        if (amode == 2u)
        {
            L = m3Sub3(L, m3MulSV3(c->upperImpulse, basis[k]));
        }
    }
    uint32_t motorAxis = (uint32_t)(c->genModes >> 12) & 15u;
    if (motorAxis < 3u)
    {
        P = m3Add3(P, m3MulSV3(c->motorImpulse, basis[motorAxis]));
    }
    else if (motorAxis < 6u)
    {
        L = m3Add3(L, m3MulSV3(c->motorImpulse, basis[motorAxis - 3u]));
    }
    w->linearExtra = m3Sub3(P, c->impulse); // primary already rides
                                            // c->impulse below; keep
                                            // the sum honest
    w->angularImpulse = L;
}

static void SolveGeneric(m3World* world, m3JointConstraint* c, const m3JointSolveContext* s)
{
    m3Vec3 rA = s->rA;
    m3Vec3 rB = s->rB;
    m3Vec3 vA = s->vA;
    m3Vec3 wA = s->wA;
    m3Vec3 vB = s->vB;
    m3Vec3 wB = s->wB;
    const m3Vec3* deltaPos = s->deltaPos;
    const m3Quat* deltaRot = s->deltaRot;
    m3real hSub = s->hSub;
    m3real invHSub = s->invHSub;
    int useBias = s->useBias;
    // The 6-DOF: per-axis rows from the proven
    // recipes. Linear locked = prismatic perp row, linear
    // limited = translation bounds in the revolute limit
    // recipe, angular locked = per-axis lock row with the
    // rev-19 sign convention, angular limited = the
    // revolute angle recipe about that basis axis, motor =
    // the prismatic or revolute motor verbatim.
    m3Vec3 sep = m3Add3(m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), m3Sub3(rB, rA)),
                        c->deltaCenter);
    m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
    m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
    if (quatA.x * quatB.x + quatA.y * quatB.y + quatA.z * quatB.z + quatA.w * quatB.w < 0.0f)
    {
        quatB = (m3Quat){-quatB.x, -quatB.y, -quatB.z, -quatB.w};
    }
    m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
    m3Quat relQ = m3MulQuat(conjA, quatB);
    m3Vec3 rotErr = m3RotateVec3(quatA, m3JointQuatToRotationVec(relQ));
    m3Vec3 basis[3];
    basis[0] = m3RotateVec3(deltaRot[c->bodyA], c->perpAxisX);
    basis[1] = m3RotateVec3(deltaRot[c->bodyA], c->perpAxisY);
    basis[2] = m3RotateVec3(deltaRot[c->bodyA], c->rotationAxis);
    m3real linPrim[3] = {c->impulse.x, c->impulse.y, c->impulse.z};
    m3real linUp[3] = {c->perpImpulseX, c->perpImpulseY, c->swingImpulse};
    m3real angPrim[3] = {c->angularImpulse.x, c->angularImpulse.y, c->angularImpulse.z};
    const m3real linLower[3] = {c->genLinLower.x, c->genLinLower.y, c->genLinLower.z};
    const m3real linUpper[3] = {c->genLinUpper.x, c->genLinUpper.y, c->genLinUpper.z};
    const m3real angLower[3] = {c->genAngLower.x, c->genAngLower.y, c->genAngLower.z};
    const m3real angUpper[3] = {c->genAngUpper.x, c->genAngUpper.y, c->genAngUpper.z};

    for (int32_t k = 0; k < 3; ++k)
    {
        uint32_t lmode = (uint32_t)(c->genModes >> (2 * k)) & 3u;
        if (lmode == 1u)
        {
            continue; // free translation
        }
        m3Vec3 a = basis[k];
        m3Vec3 sA = m3Cross3(m3Add3(rA, sep), a);
        m3Vec3 sB = m3Cross3(rB, a);
        m3real kk = c->invMassA + c->invMassB + m3Dot3(sA, m3MulMV3(c->invIA, sA)) +
                    m3Dot3(sB, m3MulMV3(c->invIB, sB));
        m3real mass = kk > 0.0f ? 1.0f / kk : 0.0f;
        m3real coord = m3Dot3(a, sep);
        if (lmode == 0u)
        {
            // Locked: an equality row (the prismatic perp
            // recipe on one axis).
            m3real bias = 0.0f;
            m3real massScale = 1.0f;
            m3real impulseScale = 0.0f;
            if (useBias)
            {
                bias = c->softness.biasRate * coord;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m3real cd =
                m3Dot3(a, m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA))));
            m3real delta = -massScale * mass * (cd + bias) - impulseScale * linPrim[k];
            linPrim[k] += delta;
            vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, a));
            wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sA)));
            vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, a));
            wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sB)));
        }
        else
        {
            // Limited: both bounds in the revolute limit
            // recipe, translated.
            {
                m3real cc = coord - linLower[k];
                m3real bias = 0.0f;
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (cc > 0.0f)
                {
                    bias = cc * invHSub;
                }
                else if (useBias)
                {
                    bias = c->softness.biasRate * cc;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3real cd =
                    m3Dot3(a, m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA))));
                m3real old = linPrim[k];
                m3real delta = -massScale * mass * (cd + bias) - impulseScale * old;
                linPrim[k] = m3MaxF(old + delta, 0.0f);
                m3real applied = linPrim[k] - old;
                vA = m3Sub3(vA, m3MulSV3(c->invMassA * applied, a));
                wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(applied, sA)));
                vB = m3Add3(vB, m3MulSV3(c->invMassB * applied, a));
                wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(applied, sB)));
            }
            {
                m3real cc = linUpper[k] - coord;
                m3real bias = 0.0f;
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (cc > 0.0f)
                {
                    bias = cc * invHSub;
                }
                else if (useBias)
                {
                    bias = c->softness.biasRate * cc;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3real cd =
                    -m3Dot3(a, m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA))));
                m3real old = linUp[k];
                m3real delta = -massScale * mass * (cd + bias) - impulseScale * old;
                linUp[k] = m3MaxF(old + delta, 0.0f);
                m3real applied = linUp[k] - old;
                vA = m3Add3(vA, m3MulSV3(c->invMassA * applied, a));
                wA = m3Add3(wA, m3MulMV3(c->invIA, m3MulSV3(applied, sA)));
                vB = m3Sub3(vB, m3MulSV3(c->invMassB * applied, a));
                wB = m3Sub3(wB, m3MulMV3(c->invIB, m3MulSV3(applied, sB)));
            }
        }
    }

    for (int32_t k = 0; k < 3; ++k)
    {
        uint32_t amode = (uint32_t)(c->genModes >> (6 + 2 * k)) & 3u;
        m3Vec3 a = basis[k];
        m3Vec3 sum = m3Add3(m3MulMV3(c->invIA, a), m3MulMV3(c->invIB, a));
        m3real kk = m3Dot3(a, sum);
        m3real mass = kk > 0.0f ? 1.0f / kk : 0.0f;
        if (amode == 0u)
        {
            // Locked: the per-axis lock row, rev-19 signs.
            m3real bias = 0.0f;
            m3real massScale = 1.0f;
            m3real impulseScale = 0.0f;
            if (useBias)
            {
                bias = c->softness.biasRate * m3Dot3(a, rotErr);
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m3real cd = m3Dot3(a, m3Sub3(wB, wA));
            m3real delta = -massScale * mass * (cd + bias) - impulseScale * angPrim[k];
            angPrim[k] += delta;
            wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, a)));
            wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, a)));
        }
        else if (amode == 2u)
        {
            // Limited: the revolute angle recipe about this
            // basis axis (the v1 contract guarantees a
            // well-posed configuration).
            m3real comp = k == 0 ? relQ.x : (k == 1 ? relQ.y : relQ.z);
            m3real angle = 2.0f * m3Atan2(comp, relQ.w);
            {
                m3real cc = angle - angLower[k];
                m3real bias = 0.0f;
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (cc > 0.0f)
                {
                    bias = cc * invHSub;
                }
                else if (useBias)
                {
                    bias = c->softness.biasRate * cc;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3real cd = m3Dot3(m3Sub3(wB, wA), a);
                m3real old = angPrim[k];
                m3real delta = -massScale * mass * (cd + bias) - impulseScale * old;
                angPrim[k] = m3MaxF(old + delta, 0.0f);
                m3real applied = angPrim[k] - old;
                wA = m3Sub3(wA, m3MulSV3(applied, m3MulMV3(c->invIA, a)));
                wB = m3Add3(wB, m3MulSV3(applied, m3MulMV3(c->invIB, a)));
            }
            {
                m3real cc = angUpper[k] - angle;
                m3real bias = 0.0f;
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (cc > 0.0f)
                {
                    bias = cc * invHSub;
                }
                else if (useBias)
                {
                    bias = c->softness.biasRate * cc;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3real cd = m3Dot3(m3Sub3(wA, wB), a);
                m3real old = c->upperImpulse;
                m3real delta = -massScale * mass * (cd + bias) - impulseScale * old;
                c->upperImpulse = m3MaxF(old + delta, 0.0f);
                m3real applied = c->upperImpulse - old;
                wA = m3Add3(wA, m3MulSV3(applied, m3MulMV3(c->invIA, a)));
                wB = m3Sub3(wB, m3MulSV3(applied, m3MulMV3(c->invIB, a)));
            }
        }
    }

    uint32_t motorAxis = (uint32_t)(c->genModes >> 12) & 15u;
    if (motorAxis < 6u)
    {
        m3Vec3 a = basis[motorAxis < 3u ? motorAxis : motorAxis - 3u];
        if (motorAxis < 3u)
        {
            m3Vec3 sA = m3Cross3(m3Add3(rA, sep), a);
            m3Vec3 sB = m3Cross3(rB, a);
            m3real kk = c->invMassA + c->invMassB + m3Dot3(sA, m3MulMV3(c->invIA, sA)) +
                        m3Dot3(sB, m3MulMV3(c->invIB, sB));
            m3real mass = kk > 0.0f ? 1.0f / kk : 0.0f;
            m3real cd =
                m3Dot3(a, m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA)))) -
                c->motorSpeed;
            m3real delta = -mass * cd;
            m3real maxImpulse = c->maxMotorEffort * hSub;
            m3real next = m3MaxF(-maxImpulse, m3MinF(maxImpulse, c->motorImpulse + delta));
            delta = next - c->motorImpulse;
            c->motorImpulse = next;
            vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, a));
            wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sA)));
            vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, a));
            wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sB)));
        }
        else
        {
            m3Vec3 sum = m3Add3(m3MulMV3(c->invIA, a), m3MulMV3(c->invIB, a));
            m3real kk = m3Dot3(a, sum);
            m3real mass = kk > 0.0f ? 1.0f / kk : 0.0f;
            m3real cd = m3Dot3(m3Sub3(wB, wA), a) - c->motorSpeed;
            m3real delta = -mass * cd;
            m3real maxImpulse = c->maxMotorEffort * hSub;
            m3real next = m3MaxF(-maxImpulse, m3MinF(maxImpulse, c->motorImpulse + delta));
            delta = next - c->motorImpulse;
            c->motorImpulse = next;
            wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, a)));
            wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, a)));
        }
    }

    c->impulse = (m3Vec3){linPrim[0], linPrim[1], linPrim[2]};
    c->perpImpulseX = linUp[0];
    c->perpImpulseY = linUp[1];
    c->swingImpulse = linUp[2];
    c->angularImpulse = (m3Vec3){angPrim[0], angPrim[1], angPrim[2]};
    world->bodies.linearVelocities[c->bodyA] = vA;
    world->bodies.angularVelocities[c->bodyA] = wA;
    world->bodies.linearVelocities[c->bodyB] = vB;
    world->bodies.angularVelocities[c->bodyB] = wB;
    return;
}

const m3JointKind m3_genericJointKind = {PrepareGeneric, WarmStartGeneric, SolveGeneric};
