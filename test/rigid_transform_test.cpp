// © Joseph Cameron - All Rights Reserved

#include <jfc/catch.hpp>

#include <gdk/collisions/box_collider.h>
#include <gdk/collisions/capsule_collider.h>
#include <gdk/collisions/compound_collider.h>
#include <gdk/collisions/exception.h>
#include <gdk/collisions/heightfield_collider.h>
#include <gdk/collisions/impl_collision_scene.h>
#include <gdk/collisions/impl_heightfield_data.h>
#include <gdk/collisions/impl_mesh_data.h>
#include <gdk/collisions/mesh_collider.h>
#include <gdk/collisions/obb_collider.h>
#include <gdk/collisions/plane_collider.h>
#include <gdk/collisions/rigid_transform.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/sphere_collider.h>

#include <cmath>
#include <cstdint>
#include <vector>

using namespace gdk;
using namespace gdk::collisions;

namespace {
    [[nodiscard]] scene_ptr_type make_scene() {
        return impl_collision_scene::make(
            [](const collision_event &) {}, [](const trigger_event &) {});
    }

    [[nodiscard]] matrix4x4_type rotated() {
        matrix4x4_type m;
        m.set_rotation(quaternion_type::from_euler({0.3f, 0.6f, 0.4f}));
        m.set_translation({1, 2, 3});
        return m;
    }

    [[nodiscard]] matrix4x4_type scaled(const float aScale) {
        matrix4x4_type m;
        m.set_rotation_and_scale(quaternion_type::identity, vector3_type(aScale));
        return m;
    }

    [[nodiscard]] mesh_data_ptr_type a_mesh() {
        return impl_mesh_data::make({{0, 0, 0}, {1, 0, 0}, {0, 0, 1}}, {0, 1, 2});
    }

    [[nodiscard]] heightfield_data_ptr_type a_heightfield() {
        return impl_heightfield_data::make(2, 2,
            std::vector<floating_point_type>{0, 0, 0, 0}, 1);
    }
}

TEST_CASE("gdk::collisions::find_transform_defect", "[rigid_transform]")
{
    SECTION("identity and pure rotation with translation are rigid")
    {
        REQUIRE(is_rigid_transform(matrix4x4_type::identity));
        REQUIRE(is_rigid_transform(rotated()));
    }

    SECTION("a scale is reported as a scale, in either direction")
    {
        REQUIRE(find_transform_defect(scaled(2.0f)) == transform_defect::scaled);
        REQUIRE(find_transform_defect(scaled(0.5f)) == transform_defect::scaled);
    }

    SECTION("**a scale small enough to be rounding is still accepted**")
    {
        REQUIRE(is_rigid_transform(scaled(1.0f + 1e-5f)));
        REQUIRE_FALSE(is_rigid_transform(scaled(1.0f + 1e-1f)));
    }

    SECTION("a shear is distinguished from a scale")
    {
        auto m = matrix4x4_type::identity;

        m.set(1, 0, 0.5f);
        m.set(1, 1, std::sqrt(0.75f));

        REQUIRE(find_transform_defect(m) == transform_defect::sheared);
    }

    SECTION("a reflection is orthonormal but still refused")
    {
        auto m = matrix4x4_type::identity;
        m.set(0, 0, -1.0f);

        REQUIRE(find_transform_defect(m) == transform_defect::reflected);
    }
}

TEST_CASE("gdk::collisions::scene refuses a non-rigid static transform", "[rigid_transform]")
{
    const auto pScene = make_scene();

    const auto rigid = rotated();
    const auto notRigid = scaled(2.0f);

    SECTION("axis aligned box")
    {
        REQUIRE_NOTHROW(pScene->make_static_axis_aligned_box_collider(rigid, {1, 1, 1}));
        REQUIRE_THROWS_AS(pScene->make_static_axis_aligned_box_collider(notRigid, {1, 1, 1}), exception);
        REQUIRE_THROWS_AS(pScene->make_static_axis_aligned_box_trigger(notRigid, {1, 1, 1}), exception);
    }

    SECTION("sphere")
    {
        REQUIRE_NOTHROW(pScene->make_static_sphere_collider(rigid, 1));
        REQUIRE_THROWS_AS(pScene->make_static_sphere_collider(notRigid, 1), exception);
        REQUIRE_THROWS_AS(pScene->make_static_sphere_trigger(notRigid, 1), exception);
    }

    SECTION("capsule")
    {
        REQUIRE_NOTHROW(pScene->make_static_capsule_collider(rigid, 1, 1));
        REQUIRE_THROWS_AS(pScene->make_static_capsule_collider(notRigid, 1, 1), exception);
    }

    SECTION("obb")
    {
        REQUIRE_NOTHROW(pScene->make_static_obb_collider(rigid, {1, 1, 1}));
        REQUIRE_THROWS_AS(pScene->make_static_obb_collider(notRigid, {1, 1, 1}), exception);
    }

    SECTION("mesh")
    {
        REQUIRE_NOTHROW(pScene->make_static_mesh_collider(rigid, a_mesh()));
        REQUIRE_THROWS_AS(pScene->make_static_mesh_collider(notRigid, a_mesh()), exception);
        REQUIRE_THROWS_AS(pScene->make_static_mesh_trigger(notRigid, a_mesh()), exception);
    }

    SECTION("heightfield")
    {
        REQUIRE_NOTHROW(pScene->make_static_heightfield_collider(rigid, a_heightfield()));
        REQUIRE_THROWS_AS(pScene->make_static_heightfield_collider(notRigid, a_heightfield()), exception);
    }

    SECTION("compound")
    {
        const auto build = [](compound_collider &aCollider) {
            aCollider.add_box({0, 0, 0}, {1, 1, 1});
        };

        REQUIRE_NOTHROW(pScene->make_static_compound_collider(rigid, build));
        REQUIRE_THROWS_AS(pScene->make_static_compound_collider(notRigid, build), exception);
    }

    SECTION("plane")
    {
        REQUIRE_NOTHROW(pScene->make_static_plane_collider(rigid));
        REQUIRE_THROWS_AS(pScene->make_static_plane_collider(notRigid), exception);
    }

    SECTION("the message says what was wrong and what to do instead")
    {
        try {
            pScene->make_static_mesh_collider(notRigid, a_mesh());
            FAIL("expected a throw");
        }
        catch (const exception &aException) {
            const std::string what = aException.what();

            REQUIRE(what.find("make_static_mesh_collider") != std::string::npos);
            REQUIRE(what.find("scale") != std::string::npos);
        }
    }
}
