// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Every switch pair takes effect in the call that sets it: right after
// EnableThing(x), before any step, IsThingEnabled reads x. A switch that
// waited for a later call to apply it would be a staged switch, named
// StageEnable by the family rule; maul3d has none.

#include "helpers.h"
#include "test_harness.h"

#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <stdio.h>

// The objects the switches act on, in a world that never steps.
typedef struct Rig
{
    m3World* world;
    m3BodyId body;
    m3ShapeId shape;
    m3JointId hinge;
    m3JointId wheel;
    m3JointId ball;
} Rig;

static Rig BuildRig(void)
{
    Rig r;
    m3WorldDef wd = m3DefaultWorldDef();
    r.world = NewWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = NewBody(r.world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    r.body = NewBody(r.world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    r.shape = NewBoxShape(r.world, r.body, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    bd.position = (m3Pos3){2.0, 1.0, 0.0};
    m3BodyId rim = NewBody(r.world, &bd);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    NewSphereShape(r.world, rim, &sd, &ball);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyIdA = ground;
    jd.bodyIdB = r.body;
    r.hinge = NewJoint(r.world, &jd);
    jd = m3DefaultJointDef();
    jd.type = m3_wheelJoint;
    jd.bodyIdA = ground;
    jd.bodyIdB = rim;
    jd.localAnchorA = (m3Vec3){2.0f, 1.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    r.wheel = NewJoint(r.world, &jd);
    jd = m3DefaultJointDef();
    jd.bodyIdA = r.body;
    jd.bodyIdB = rim;
    r.ball = NewJoint(r.world, &jd);
    // The spring and the steering switch on only with a frequency.
    m3Joint_SetSpringHertz(r.world, r.hinge, 4.0f);
    m3WheelJoint_SetSteeringHertz(r.world, r.wheel, 4.0f);
    return r;
}

typedef void SwitchSetFunction(const Rig* r, bool on);
typedef bool SwitchReadFunction(const Rig* r);

typedef struct Switch
{
    const char* name;
    SwitchSetFunction* set;
    SwitchReadFunction* read;
} Switch;

#define SWITCH(name, object, setCall, readCall)                                                    \
    static void Set##name(const Rig* r, bool on)                                                   \
    {                                                                                              \
        setCall(r->world, r->object, on);                                                          \
    }                                                                                              \
    static bool Read##name(const Rig* r)                                                           \
    {                                                                                              \
        return readCall(r->world, r->object);                                                      \
    }

SWITCH(Body, body, m3Body_Enable, m3Body_IsEnabled)
SWITCH(Bullet, body, m3Body_EnableBullet, m3Body_IsBulletEnabled)
SWITCH(FastRotation, body, m3Body_EnableFastRotation, m3Body_IsFastRotationEnabled)
SWITCH(BodySleep, body, m3Body_EnableSleep, m3Body_IsSleepEnabled)
SWITCH(HitEvents, shape, m3Shape_EnableHitEvents, m3Shape_IsHitEventsEnabled)
SWITCH(PreSolve, shape, m3Shape_EnablePreSolveEvents, m3Shape_IsPreSolveEventsEnabled)
SWITCH(Limit, hinge, m3Joint_EnableLimit, m3Joint_IsLimitEnabled)
SWITCH(Motor, hinge, m3Joint_EnableMotor, m3Joint_IsMotorEnabled)
SWITCH(Spring, hinge, m3Joint_EnableSpring, m3Joint_IsSpringEnabled)
SWITCH(ConnectedCollision, hinge, m3Joint_EnableConnectedCollision,
       m3Joint_IsConnectedCollisionEnabled)
SWITCH(Steering, wheel, m3WheelJoint_EnableSteering, m3WheelJoint_IsSteeringEnabled)
SWITCH(Cone, ball, m3SphericalJoint_EnableCone, m3SphericalJoint_IsConeEnabled)

static void SetWorldSleep(const Rig* r, bool on)
{
    m3World_EnableSleep(r->world, on);
}

static bool ReadWorldSleep(const Rig* r)
{
    return m3World_IsSleepEnabled(r->world);
}

static void SetContinuous(const Rig* r, bool on)
{
    m3World_EnableContinuous(r->world, on);
}

static bool ReadContinuous(const Rig* r)
{
    return m3World_IsContinuousEnabled(r->world);
}

static const Switch s_switches[] = {
    {"body", SetBody, ReadBody},
    {"bullet", SetBullet, ReadBullet},
    {"fast rotation", SetFastRotation, ReadFastRotation},
    {"body sleep", SetBodySleep, ReadBodySleep},
    {"hit events", SetHitEvents, ReadHitEvents},
    {"pre-solve events", SetPreSolve, ReadPreSolve},
    {"joint limit", SetLimit, ReadLimit},
    {"joint motor", SetMotor, ReadMotor},
    {"joint spring", SetSpring, ReadSpring},
    {"connected collision", SetConnectedCollision, ReadConnectedCollision},
    {"wheel steering", SetSteering, ReadSteering},
    {"spherical cone", SetCone, ReadCone},
    {"world sleep", SetWorldSleep, ReadWorldSleep},
    {"continuous", SetContinuous, ReadContinuous},
};

// Each switch flips from its default and back, read after each call with
// no step between, and neither call counts a misuse.
static void TestSwitchesTakeEffectAtOnce(void)
{
    Rig r = BuildRig();
    for (size_t i = 0; i < sizeof(s_switches) / sizeof(s_switches[0]); ++i)
    {
        const Switch* s = &s_switches[i];
        uint64_t misuse = m3World_GetCounters(r.world).misuse;
        bool start = s->read(&r);
        s->set(&r, !start);
        bool flipped = s->read(&r) == !start;
        s->set(&r, start);
        bool back = s->read(&r) == start;
        bool taken = m3World_GetCounters(r.world).misuse == misuse;
        if (!flipped || !back || !taken)
        {
            printf("  %s: flipped %d, back %d, taken %d\n", s->name, flipped, back, taken);
        }
        CHECK(flipped && back, "the switch reads what its call set, before any step");
        CHECK(taken, "and neither call is refused");
    }
    CHECK(m3World_GetStepCount(r.world) == 0, "the world never stepped");
    m3DestroyWorld(r.world);
}

int main(void)
{
    TestSwitchesTakeEffectAtOnce();
    if (s_failures == 0)
    {
        printf("test_switches: all checks passed (%d switch pairs)\n",
               (int)(sizeof(s_switches) / sizeof(s_switches[0])));
        return 0;
    }
    return 1;
}
