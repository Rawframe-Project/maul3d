// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A journal's joint create carries the whole internal def (src/journal.h),
// and each public create fills only the fields its type's def has. Replay
// takes a create op only in that form: a tape that builds one joint of
// several types replays, and the same tape with one field set that the
// joint's type has no public field for is refused.

#include "helpers.h"
#include "journal.h"
#include "state_prefix.h"
#include "test_harness.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    TAPE_BYTES = 1 << 16
};

typedef enum Kind
{
    kind_ball,
    kind_hinge,
    kind_weld,
    kind_filter,
    kind_rope,
    kind_count
} Kind;

// One field of one create's def, set to a value its type's public def
// cannot give it.
typedef struct FieldCase
{
    const char* name;
    Kind kind;
    size_t at;
    size_t width;
    float value;
} FieldCase;

static const FieldCase s_cases[] = {
    {"a motor on a ball joint", kind_ball, offsetof(m3JointDef, enableMotor), 1, 0.0f},
    {"a motor speed on a weld", kind_weld, offsetof(m3JointDef, motorSpeed), 4, 1.0f},
    {"a limit range on a weld", kind_weld, offsetof(m3JointDef, upperLimit), 4, 1.0f},
    {"a cone angle on a hinge", kind_hinge, offsetof(m3JointDef, coneAngle), 4, 0.5f},
    {"a spring on a weld", kind_weld, offsetof(m3JointDef, springHertz), 4, 2.0f},
    {"a ratio on a hinge", kind_hinge, offsetof(m3JointDef, ratio), 4, 2.0f},
    {"a servo budget on a hinge", kind_hinge, offsetof(m3JointDef, maxForce), 4, 1.0f},
    {"a rope length on a ball joint", kind_ball, offsetof(m3JointDef, maxLength), 4, 1.0f},
    {"an axis on a rope", kind_rope, offsetof(m3JointDef, localAxisA), 4, 1.0f},
    {"an anchor on a filter joint", kind_filter, offsetof(m3JointDef, localAnchorB), 4, 1.0f},
    {"connected collision on a filter joint", kind_filter,
     offsetof(m3JointDef, enableConnectedCollision), 1, 0.0f},
    {"a generic range on a hinge", kind_hinge, offsetof(m3JointDef, genericLinearUpper), 4, 1.0f},
    {"a generic motor axis on a hinge", kind_hinge, offsetof(m3JointDef, genericMotorAxis), 1,
     0.0f},
};

static m3World* DefaultWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    return NewWorld(&def);
}

// Two crates and one joint of each kind between them, in kind order.
static void Build(m3World* world)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId a = NewBody(world, &bd);
    bd.position = (m3Pos3){1.5, 3.0, 0.0};
    m3BodyId b = NewBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    (void)NewBoxShape(world, a, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    (void)NewBoxShape(world, b, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    m3SphericalJointDef ball = m3DefaultSphericalJointDef();
    ball.bodyIdA = a;
    ball.bodyIdB = b;
    (void)NewJoint(world, &ball);
    m3RevoluteJointDef hinge = m3DefaultRevoluteJointDef();
    hinge.bodyIdA = a;
    hinge.bodyIdB = b;
    (void)NewJoint(world, &hinge);
    m3WeldJointDef weld = m3DefaultWeldJointDef();
    weld.bodyIdA = a;
    weld.bodyIdB = b;
    (void)NewJoint(world, &weld);
    m3FilterJointDef filter = m3DefaultFilterJointDef();
    filter.bodyIdA = a;
    filter.bodyIdB = b;
    (void)NewJoint(world, &filter);
    m3DistanceJointDef rope = m3DefaultDistanceJointDef();
    rope.bodyIdA = a;
    rope.bodyIdB = b;
    rope.minLength = 1.0f;
    rope.maxLength = 2.0f;
    (void)NewJoint(world, &rope);
}

static void TestReplayTakesOnlyTypeFields(void)
{
    uint8_t* tape = (uint8_t*)malloc(TAPE_BYTES);
    uint8_t* edited = (uint8_t*)malloc(TAPE_BYTES);
    m3World* world = DefaultWorld();
    CHECK(tape != NULL && edited != NULL && StartJournal(world, tape, TAPE_BYTES),
          "the journal starts");
    if (tape == NULL || edited == NULL)
    {
        free(tape);
        free(edited);
        m3DestroyWorld(world);
        return;
    }
    Build(world);
    CHECK(m3World_GetJoints(world, NULL, 0) == kind_count, "every joint is built");
    int64_t bytes = StopJournal(world);
    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);

    m3World* again = DefaultWorld();
    CHECK(ReplayJournal(again, tape, bytes) && m3World_Hash(again) == hash, "the tape replays");
    m3DestroyWorld(again);

    // The joint creates on the tape, in create order.
    int32_t creates[kind_count];
    int32_t found = 0;
    for (int32_t at = M3_JOURNAL_HEADER_BYTES; at + 8 <= bytes;)
    {
        int32_t op;
        int32_t size;
        memcpy(&op, tape + at, sizeof(op));
        memcpy(&size, tape + at + 4, sizeof(size));
        if (op == (int32_t)m3_opCreateJoint && found < kind_count)
        {
            creates[found] = at + 8;
            found += 1;
        }
        at += 8 + size;
    }
    CHECK(found == kind_count, "every joint's create is on the tape");
    for (size_t c = 0; found == kind_count && c < sizeof(s_cases) / sizeof(s_cases[0]); ++c)
    {
        const FieldCase* k = &s_cases[c];
        memcpy(edited, tape, (size_t)bytes);
        uint8_t* field = edited + creates[k->kind] + offsetof(m3CreateJointOp, def) + k->at;
        if (k->width == 1)
        {
            *field = (uint8_t)(*field == 255 ? 0 : 1);
        }
        else
        {
            memcpy(field, &k->value, sizeof(k->value));
        }
        m3World* refused = DefaultWorld();
        CHECK(m3World_ReplayJournal(refused, edited, bytes) == m3_errorFormat, k->name);
        m3DestroyWorld(refused);
    }
    free(edited);
    free(tape);
}

int main(void)
{
    TestReplayTakesOnlyTypeFields();
    if (s_failures == 0)
    {
        printf("test_joint_defs: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
