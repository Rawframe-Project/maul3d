// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "m3_world_3d.h"

#include <cstring>

#include <godot_cpp/core/class_db.hpp>

namespace maul3d_godot
{

static int64_t PackBody(m3BodyId id)
{
    int64_t packed = 0;
    std::memcpy(&packed, &id, sizeof(id));
    return packed;
}

static m3BodyId UnpackBody(int64_t packed)
{
    m3BodyId id;
    std::memcpy(&id, &packed, sizeof(id));
    return id;
}

void M3World3D::_enter_tree()
{
    m3WorldDef def = m3DefaultWorldDef();
    if (m3CreateWorld(&def, &world_) != m3_success)
    {
        world_ = nullptr;
    }
}

void M3World3D::_exit_tree()
{
    m3DestroyWorld(world_);
    world_ = nullptr;
}

void M3World3D::_physics_process(double delta)
{
    if (world_ != nullptr)
    {
        m3World_Step(world_, (float)delta, substeps_);
    }
}

int64_t M3World3D::create_dynamic_box(godot::Vector3 position, godot::Vector3 half_extents,
                                      float density)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = m3Pos3{position.x, position.y, position.z};
    m3BodyId body = m3_nullBodyId;
    m3ShapeId shape = m3_nullShapeId;
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = density;
    if (m3CreateBody(world_, &bd, &body) != m3_success ||
        m3CreateBoxShape(world_, body, &sd,
                         m3Vec3{half_extents.x, half_extents.y, half_extents.z},
                         &shape) != m3_success)
    {
        return 0;
    }
    return PackBody(body);
}

int64_t M3World3D::create_static_floor()
{
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId body = m3_nullBodyId;
    m3ShapeId shape = m3_nullShapeId;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    if (m3CreateBody(world_, &bd, &body) != m3_success ||
        m3CreatePlaneShape(world_, body, &sd, &floor, &shape) != m3_success)
    {
        return 0;
    }
    return PackBody(body);
}

godot::Transform3D M3World3D::body_transform(int64_t body) const
{
    m3BodyId id = UnpackBody(body);
    m3Pos3 p = m3Body_GetPosition(world_, id);
    m3Quat q = m3Body_GetRotation(world_, id);
    godot::Quaternion gq(q.x, q.y, q.z, q.w);
    godot::Vector3 gp((real_t)p.x, (real_t)p.y, (real_t)p.z);
    return godot::Transform3D(godot::Basis(gq), gp);
}

void M3World3D::set_body_velocity(int64_t body, godot::Vector3 velocity)
{
    m3Body_SetLinearVelocity(world_, UnpackBody(body),
                             m3Vec3{velocity.x, velocity.y, velocity.z});
}

godot::PackedByteArray M3World3D::snapshot() const
{
    godot::PackedByteArray out;
    int32_t size = m3World_SnapshotSize(world_);
    int32_t written = 0;
    out.resize(size);
    if (m3World_Snapshot(world_, out.ptrw(), size, &written) != m3_success)
    {
        out.resize(0);
    }
    return out;
}

bool M3World3D::restore(const godot::PackedByteArray& data)
{
    return m3World_Restore(world_, data.ptr(), (int32_t)data.size()) == m3_success;
}

godot::String M3World3D::world_hash() const
{
    return godot::String::num_uint64(m3World_Hash(world_), 16);
}

void M3World3D::set_substeps(int substeps)
{
    substeps_ = substeps < 1 ? 1 : substeps;
}

int M3World3D::get_substeps() const
{
    return substeps_;
}

void M3World3D::_bind_methods()
{
    godot::ClassDB::bind_method(godot::D_METHOD("create_dynamic_box", "position", "half_extents",
                                                "density"),
                                &M3World3D::create_dynamic_box);
    godot::ClassDB::bind_method(godot::D_METHOD("create_static_floor"),
                                &M3World3D::create_static_floor);
    godot::ClassDB::bind_method(godot::D_METHOD("body_transform", "body"),
                                &M3World3D::body_transform);
    godot::ClassDB::bind_method(godot::D_METHOD("set_body_velocity", "body", "velocity"),
                                &M3World3D::set_body_velocity);
    godot::ClassDB::bind_method(godot::D_METHOD("snapshot"), &M3World3D::snapshot);
    godot::ClassDB::bind_method(godot::D_METHOD("restore", "data"), &M3World3D::restore);
    godot::ClassDB::bind_method(godot::D_METHOD("world_hash"), &M3World3D::world_hash);
    godot::ClassDB::bind_method(godot::D_METHOD("set_substeps", "substeps"),
                                &M3World3D::set_substeps);
    godot::ClassDB::bind_method(godot::D_METHOD("get_substeps"), &M3World3D::get_substeps);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::INT, "substeps"), "set_substeps",
                 "get_substeps");
}

} // namespace maul3d_godot
