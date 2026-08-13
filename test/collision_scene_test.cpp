// © Joseph Cameron - All Rights Reserved

#include <jfc/catch.hpp>

#include <gdk/collisions/collider.h>
#include <gdk/collisions/box_collider.h>
#include <gdk/collisions/capsule_collider.h>
#include <gdk/collisions/events.h>
#include <gdk/collisions/compound_collider.h>
#include <gdk/collisions/obb_collider.h>
#include <gdk/collisions/plane_collider.h>
#include <gdk/collisions/raycast_hit.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/impl_collision_scene.h>
#include <gdk/collisions/heightfield_collider.h>
#include <gdk/collisions/impl_capsule_collider.h>
#include <gdk/collisions/impl_heightfield_data.h>
#include <gdk/collisions/impl_mesh_data.h>
#include <gdk/collisions/mesh_collider.h>
#include <gdk/collisions/sphere_collider.h>

#include <algorithm>
#include <limits>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <span>

using namespace gdk::collisions;

namespace {
    struct fixture final {
        std::vector<collision_event> collisions;
        std::vector<trigger_event> triggers;
        scene_ptr_type scene;

        fixture()
        : scene(impl_collision_scene::make(
            [this](collision_event e) { collisions.push_back(e); },
            [this](trigger_event e) { triggers.push_back(e); }))
        {}

        void update(const delta_time_type aDeltaTime) {
            scene->update(aDeltaTime);
            scene->process_events();
        }

        [[nodiscard]] std::size_t count(const event_type aType) const {
            std::size_t n = 0;
            for (const auto &e : collisions) if (e.type == aType) ++n;
            return n;
        }
    };

    void require_vector_near(const vector3_type &aActual, const vector3_type &aExpected,
        const floating_point_type aMargin) {
        REQUIRE(aActual.x == Approx(aExpected.x).margin(aMargin));
        REQUIRE(aActual.y == Approx(aExpected.y).margin(aMargin));
        REQUIRE(aActual.z == Approx(aExpected.z).margin(aMargin));
    }

    [[nodiscard]] mesh_data_ptr_type make_quad_mesh(const floating_point_type aHalfSize) {
        return impl_mesh_data::make(
            {{-aHalfSize, 0, -aHalfSize}, {aHalfSize, 0, -aHalfSize},
             {aHalfSize, 0, aHalfSize}, {-aHalfSize, 0, aHalfSize}},
            {0, 1, 2, 0, 2, 3});
    }

    [[nodiscard]] mesh_data_ptr_type make_open_box_mesh() {
        std::vector<vector3_type> vertices;
        std::vector<std::uint32_t> indices;

        const auto quad = [&](const vector3_type &aA, const vector3_type &aB,
            const vector3_type &aC, const vector3_type &aD) {
            const auto base = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {aA, aB, aC, aD});
            indices.insert(indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
        };

        quad({-1.5f, -1, -1.5f}, {1.5f, -1, -1.5f}, {1.5f, -1, 1.5f}, {-1.5f, -1, 1.5f});   
        quad({-1.5f, -1, -1.5f}, {-1.5f, 1, -1.5f}, {1.5f, 1, -1.5f}, {1.5f, -1, -1.5f});   
        quad({-1.5f, -1, -1.5f}, {-1.5f, -1, 1.5f}, {-1.5f, 1, 1.5f}, {-1.5f, 1, -1.5f});   
        quad({1.5f, -1, -1.5f}, {1.5f, 1, -1.5f}, {1.5f, 1, 1.5f}, {1.5f, -1, 1.5f});       

        return impl_mesh_data::make(std::move(vertices), std::move(indices));
    }

    template <typename sampler_type>
    [[nodiscard]] heightfield_data_ptr_type make_terrain(const std::size_t aColumns,
        const std::size_t aRows, const sampler_type &aSampler) {
        std::vector<floating_point_type> heights(aColumns * aRows);

        for (std::size_t row = 0; row < aRows; ++row)
            for (std::size_t column = 0; column < aColumns; ++column)
                heights[row * aColumns + column] = aSampler(column, row);

        return impl_heightfield_data::make(aColumns, aRows, std::move(heights));
    }

    [[nodiscard]] heightfield_data_ptr_type make_flat_terrain(const std::size_t aColumns,
        const std::size_t aRows) {
        return make_terrain(aColumns, aRows, [](std::size_t, std::size_t) { return 0.0f; });
    }

    const_box_collider_ptr_type add_static_box_at_origin(const fixture &aFixture) {
        matrix4x4_type identity;
        return aFixture.scene->make_static_axis_aligned_box_collider(identity, {0.5f, 0.5f, 0.5f});
    }
}

TEST_CASE("gdk::scene stops a head on approach at the surface", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    pSphere->add_velocity({1, 0, 0});
    f.update(1.0f);

    const auto position = pSphere->transform().translation();

    REQUIRE(position.x <= Approx(-1.0f).margin(1e-3f));
    REQUIRE(position.y == Approx(0.0f).margin(1e-3f));
    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene slides along a surface instead of sticking", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    pSphere->add_velocity({1.0f, 0.25f, 0});
    f.update(1.0f);

    const auto position = pSphere->transform().translation();

    REQUIRE(position.x <= Approx(-1.0f).margin(1e-3f));
    REQUIRE(position.y > 0.2f);
}

TEST_CASE("gdk::scene reports enter then stay then exit", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    pSphere->add_velocity({1, 0, 0});
    f.update(1.0f);
    REQUIRE(f.count(event_type::enter) == 1);

    pSphere->add_velocity({1, 0, 0});
    f.update(1.0f);
    REQUIRE(f.count(event_type::enter) == 1);
    REQUIRE(f.count(event_type::stay) >= 1);

    pSphere->set_position({-5, 0, 0});
    f.update(1.0f);
    REQUIRE(f.count(event_type::exit) == 1);
}

TEST_CASE("gdk::scene slides a sphere held against a surface", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.0f, 0, 0});

    for (int frame = 0; frame < 3; ++frame) {
        pSphere->add_velocity({0, 1, 0});
        f.update(0.1f);
    }

    REQUIRE(pSphere->transform().translation().y == Approx(0.3f).margin(1e-2f));
}

TEST_CASE("gdk::scene slides a box held against a surface", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pBox = f.scene->make_axis_aligned_box_collider();
    pBox->set_position({-1.0f, 0, 0});

    for (int frame = 0; frame < 3; ++frame) {
        pBox->add_velocity({0, 1, 0});
        f.update(0.1f);
    }

    REQUIRE(pBox->transform().translation().y == Approx(0.3f).margin(1e-2f));
}

TEST_CASE("gdk::scene moves freely when clear of a surface", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.05f, 0, 0});

    for (int frame = 0; frame < 3; ++frame) {
        pSphere->add_velocity({0, 1, 0});
        f.update(0.1f);
    }

    REQUIRE(pSphere->transform().translation().y == Approx(0.3f).margin(1e-2f));
    REQUIRE(pSphere->transform().translation().x == Approx(-1.05f).margin(1e-3f));
}

TEST_CASE("gdk::scene sizes a box from its half extents", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pBox = f.scene->make_axis_aligned_box_collider();
    pBox->set_half_extents({1.0f, 0.5f, 0.5f});
    pBox->set_position({-3, 0, 0});

    pBox->add_velocity({1, 0, 0});
    f.update(3.0f);

    REQUIRE(pBox->transform().translation().x == Approx(-1.5f).margin(1e-2f));
    REQUIRE(pBox->half_extents().x == Approx(1.0f));
}

TEST_CASE("gdk::scene sizes a sphere from its radius", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_radius(1.0f);
    pSphere->set_position({-3, 0, 0});

    pSphere->add_velocity({1, 0, 0});
    f.update(3.0f);

    REQUIRE(pSphere->transform().translation().x == Approx(-1.5f).margin(1e-2f));
}

TEST_CASE("gdk::scene separates two dynamic colliders", "[gdk::collision]")
{
    fixture f;
    const auto pA = f.scene->make_sphere_collider();
    const auto pB = f.scene->make_sphere_collider();
    pA->set_position({-2, 0, 0});
    pB->set_position({+2, 0, 0});

    pA->add_velocity({+1, 0, 0});
    pB->add_velocity({-1, 0, 0});
    f.update(2.0f);

    const auto separation = pB->transform().translation().x - pA->transform().translation().x;
    REQUIRE(separation >= Approx(1.0f).margin(1e-2f));
}

namespace {
    [[nodiscard]] mesh_data_ptr_type open_box_mesh() {
        std::vector<vector3_type> vertices;
        std::vector<std::uint32_t> indices;

        const auto quad = [&](const vector3_type &a, const vector3_type &b,
            const vector3_type &c, const vector3_type &d) {
            const auto base = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {a, b, c, d});
            indices.insert(indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
        };

        quad({-1.5f, -1, -1.5f}, {1.5f, -1, -1.5f}, {1.5f, -1, 1.5f}, {-1.5f, -1, 1.5f});
        quad({-1.5f, -1, -1.5f}, {-1.5f, 1, -1.5f}, {1.5f, 1, -1.5f}, {1.5f, -1, -1.5f});
        quad({-1.5f, -1, -1.5f}, {-1.5f, -1, 1.5f}, {-1.5f, 1, 1.5f}, {-1.5f, 1, -1.5f});
        quad({1.5f, -1, -1.5f}, {1.5f, 1, -1.5f}, {1.5f, 1, 1.5f}, {1.5f, -1, 1.5f});

        return impl_mesh_data::make(std::move(vertices), std::move(indices));
    }

    [[nodiscard]] floating_point_type worst_step_when_settled(
        const vector3_type &aDrive, const floating_point_type aSpin,
        const response_handler &aHandler) {
        fixture f;

        const auto pMesh = f.scene->make_static_mesh_collider(matrix4x4_type::identity,
            open_box_mesh());

        const auto pCapsule = f.scene->make_capsule_collider(aHandler);
        pCapsule->set_position({0, 0.2f, 0});

        std::vector<vector3_type> path;

        for (int frame = 0; frame < 400; ++frame) {
            pCapsule->add_velocity(aDrive);

            if (aSpin != 0) {
                quaternion_type turn;
                turn.set_from_euler({0, 0, aSpin});
                pCapsule->add_rotation(turn);
            }

            f.update(1.0f / 60.0f);
            path.push_back(pCapsule->transform().translation());
        }

        floating_point_type worst = 0;
        for (std::size_t i = path.size() - 100; i + 1 < path.size(); ++i)
            worst = std::max(worst, (path[i + 1] - path[i]).length());

        static_cast<void>(pMesh);
        return worst;
    }
}

namespace {
    [[nodiscard]] mesh_data_ptr_type descending_valley(const floating_point_type aHalfWidth,
        const floating_point_type aWallHeight) {
        std::vector<vector3_type> vertices;
        std::vector<std::uint32_t> indices;

        const auto quad = [&](const vector3_type &a, const vector3_type &b,
            const vector3_type &c, const vector3_type &d) {
            const auto base = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {a, b, c, d});
            indices.insert(indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
        };

        for (int i = 0; i < 40; ++i) {
            const auto z0 = static_cast<floating_point_type>(i);
            const auto z1 = static_cast<floating_point_type>(i + 1);
            const auto y0 = -0.25f * z0;
            const auto y1 = -0.25f * z1;

            quad({-aHalfWidth, y0 + aWallHeight, z0}, {0, y0, z0}, {0, y1, z1},
                {-aHalfWidth, y1 + aWallHeight, z1});
            quad({0, y0, z0}, {aHalfWidth, y0 + aWallHeight, z0},
                {aHalfWidth, y1 + aWallHeight, z1}, {0, y1, z1});
        }

        return impl_mesh_data::make(std::move(vertices), std::move(indices));
    }

    [[nodiscard]] floating_point_type valley_travel(const response_handler &aHandler) {
        fixture f;

        const auto pValley = f.scene->make_static_mesh_collider(matrix4x4_type::identity,
            descending_valley(1.0f, 4.0f));   

        const auto pBody = f.scene->make_sphere_collider(aHandler);
        pBody->set_position({0, 1.0f, 2.0f});

        for (int frame = 0; frame < 400; ++frame) {
            pBody->add_velocity({0, -6.0f, 0});
            f.update(1.0f / 60.0f);
        }

        static_cast<void>(pValley);
        return pBody->transform().translation().z - 2.0f;
    }
}

TEST_CASE("gdk::response_handler can slide along a crease rather than wedging in it",
    "[gdk::collision]")
{
    const auto wedging = valley_travel([](collider &aThis, const contact_context &aContact) {
        const auto velocity = aThis.velocity();
        const auto into = velocity.dot_product(aContact.normal);
        if (into >= 0) return vector3_type::zero;
        return (velocity - aContact.normal * into) - velocity;
    });

    REQUIRE(wedging < 0.5f);
    REQUIRE(valley_travel(response_handlers::slide_preserving_speed()) > 5.0f);
}

TEST_CASE("gdk::scene settles a capsule driven into a mesh", "[gdk::collision]")
{
    SECTION("against one surface, turning")
    {
        REQUIRE(worst_step_when_settled({0, -4, 0}, 0.02f, response_handlers::slide_projecting()) < 0.03f);
    }

    SECTION("into a three-surface corner, not turning")
    {
        REQUIRE(worst_step_when_settled({-4, -4, -4}, 0.0f, response_handlers::slide_projecting())
            == Approx(0.0f).margin(1e-5f));
    }

    SECTION("into a three-surface corner, turning")
    {
        REQUIRE(worst_step_when_settled({-4, -4, -4}, 0.02f, response_handlers::slide_projecting()) < 0.03f);
    }
}

TEST_CASE("gdk::collisions::response_handlers collide_and_slide keeps its speed in a corner",
    "[gdk::collision]")
{
    const auto rescaling = worst_step_when_settled({-4, -4, -4}, 0.02f,
        response_handlers::slide_preserving_speed());
    const auto projecting = worst_step_when_settled({-4, -4, -4}, 0.02f, response_handlers::slide_projecting());

    REQUIRE(projecting < 0.03f);
    REQUIRE(rescaling > projecting * 4);
}

TEST_CASE("gdk::collider add_rotation accumulates identically in both directions",
    "[gdk::collision]")
{
    fixture f;

    const auto z_angle_of = [](const collider &aBody) {
        const auto turned = aBody.rotation() * vector3_type{1, 0, 0};
        return std::atan2(turned.y, turned.x);
    };

    const auto accumulate = [&f](const floating_point_type aStep, const int aTimes) {
        const auto pBody = f.scene->make_capsule_collider();

        quaternion_type turn;
        turn.set_from_euler({0, 0, aStep});

        for (int i = 0; i < aTimes; ++i) pBody->add_rotation(turn);
        return pBody;
    };

    SECTION("fifty steps of 0.02 reach 1.0 radian, whichever way they go")
    {
        const auto pPositive = accumulate(+0.02f, 50);
        const auto pNegative = accumulate(-0.02f, 50);

        REQUIRE(z_angle_of(*pPositive) == Approx(+1.0f).margin(1e-3f));
        REQUIRE(z_angle_of(*pNegative) == Approx(-1.0f).margin(1e-3f));
        REQUIRE(z_angle_of(*pPositive) == Approx(-z_angle_of(*pNegative)).margin(1e-4f));
    }

    SECTION("every single step moves the body, in both directions")
    {
        for (const auto step : {+0.02f, -0.02f}) {
            const auto pBody = f.scene->make_capsule_collider();

            quaternion_type turn;
            turn.set_from_euler({0, 0, step});

            auto previous = z_angle_of(*pBody);

            for (int i = 0; i < 50; ++i) {
                pBody->add_rotation(turn);

                const auto current = z_angle_of(*pBody);
                REQUIRE(current != Approx(previous).margin(1e-5f));
                REQUIRE((current - previous) == Approx(step).margin(1e-4f));
                previous = current;
            }
        }
    }

    SECTION("the transform stays consistent with the rotation as they accumulate")
    {
        for (const auto step : {+0.02f, -0.02f}) {
            const auto pBody = accumulate(step, 50);

            const vector3_type probe{0.3f, 0.5f, -0.8f};
            const auto viaQuaternion = pBody->rotation() * probe;

            const vector4_type probe4{probe.x, probe.y, probe.z, 1.0f};
            const auto viaMatrix = pBody->transform() * probe4;

            REQUIRE(viaQuaternion.x == Approx(viaMatrix.x).margin(1e-4f));
            REQUIRE(viaQuaternion.y == Approx(viaMatrix.y).margin(1e-4f));
            REQUIRE(viaQuaternion.z == Approx(viaMatrix.z).margin(1e-4f));
        }
    }

    SECTION("the transform stays orthonormal as rotations accumulate")
    {
        for (const auto step : {+0.02f, -0.02f}) {
            const auto scale = accumulate(step, 500)->transform().scale();

            REQUIRE(scale.x == Approx(1.0f).margin(1e-3f));
            REQUIRE(scale.y == Approx(1.0f).margin(1e-3f));
            REQUIRE(scale.z == Approx(1.0f).margin(1e-3f));
        }
    }

    SECTION("update() never alters a collider's orientation")
    {
        const auto pBox = f.scene->make_axis_aligned_box_collider();
        pBox->set_position({0, 0, 0});
        pBox->set_inverse_overlap_weight(0);

        const auto pCapsule = f.scene->make_capsule_collider();
        pCapsule->set_position({0, 1.5f, 0});

        quaternion_type turn;
        turn.set_from_euler({0, 0, 0.02f});

        for (int frame = 0; frame < 120; ++frame) {
            pCapsule->add_velocity({0, -4.0f, 0});
            pCapsule->add_rotation(turn);

            const auto before = z_angle_of(*pCapsule);
            f.update(1.0f / 60.0f);
            REQUIRE(z_angle_of(*pCapsule) == Approx(before).margin(1e-6f));
        }
    }
}

TEST_CASE("gdk::collider can be oriented after creation", "[gdk::collision]")
{
    fixture f;

    SECTION("add_rotation composes in world space, matching add_velocity")
    {
        const auto pBody = f.scene->make_capsule_collider();

        quaternion_type quarter;
        quarter.set_from_euler({0, 0, 3.14159265f * 0.5f});

        pBody->add_rotation(quarter);
        pBody->add_rotation(quarter);

        const auto up = pBody->rotation();
        const vector3_type localY{0, 1, 0};
        const auto turned = up * localY;

        REQUIRE(turned.y == Approx(-1.0f).margin(1e-4f));
    }

    SECTION("a transform's rotation survives set_transform")
    {
        const auto pBody = f.scene->make_obb_collider();

        quaternion_type turn;
        turn.set_from_euler({0.3f, -0.7f, 1.1f});

        matrix4x4_type transform;
        transform.set_rotation(turn);

        std::dynamic_pointer_cast<impl_collider>(pBody)->set_transform(transform);

        const vector3_type probe{0.3f, 0.5f, -0.8f};
        const auto expected = turn * probe;
        const auto actual = pBody->rotation() * probe;

        REQUIRE(actual.x == Approx(expected.x).margin(1e-4f));
        REQUIRE(actual.y == Approx(expected.y).margin(1e-4f));
        REQUIRE(actual.z == Approx(expected.z).margin(1e-4f));
    }

    SECTION("a rotated static puts its geometry where the transform says")
    {
        quaternion_type quarter;
        quarter.set_from_euler({0, 0, 3.14159265f * 0.5f});

        matrix4x4_type transform;
        transform.set_rotation(quarter);

        const auto pRotated = f.scene->make_static_compound_collider(transform,
            [](compound_collider &aBuild) { aBuild.add_sphere({2.0f, 0, 0}, 0.5f); });

        const auto pProbe = f.scene->make_sphere_collider();
        pProbe->set_position({0, 2.0f, 0});

        f.update(1.0f / 60.0f);

        REQUIRE(f.count(event_type::enter) == 1);
        static_cast<void>(pRotated);
    }

    SECTION("a turned capsule collides along its new axis")
    {
        const auto pGround = f.scene->make_static_plane_collider(matrix4x4_type::identity);

        const auto pBody = f.scene->make_capsule_collider();
        pBody->set_radius(0.25f);
        pBody->set_half_height(1.5f);
        pBody->set_position({0, 4, 0});

        const auto pWall = f.scene->make_static_sphere_collider(
            [] { matrix4x4_type t; t.set_translation({1.4f, 4, 0}); return t; }(), 0.5f);

        pBody->add_velocity({0, 0, 0});
        f.update(1.0f / 60.0f);
        REQUIRE(f.count(event_type::enter) == 0);

        quaternion_type quarter;
        quarter.set_from_euler({0, 0, 3.14159265f * 0.5f});
        pBody->add_rotation(quarter);

        f.update(1.0f / 60.0f);
        REQUIRE(f.count(event_type::enter) == 1);

        static_cast<void>(pGround);
        static_cast<void>(pWall);
    }
}

TEST_CASE("gdk::scene reports a dynamic pair once, not once per participant", "[gdk::collision]")
{
    fixture f;
    const auto pA = f.scene->make_sphere_collider();
    const auto pB = f.scene->make_sphere_collider();
    pA->set_position({-2, 0, 0});
    pB->set_position({0, 0, 0});

    pA->add_velocity({1, 0, 0});
    f.update(2.0f);

    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene splits penetration between two dynamic colliders", "[gdk::collision]")
{
    fixture f;
    const auto pA = f.scene->make_sphere_collider();
    const auto pB = f.scene->make_sphere_collider();
    pA->set_position({-0.25f, 0, 0});
    pB->set_position({+0.25f, 0, 0});

    f.update(1.0f);

    const auto a = pA->transform().translation().x;
    const auto b = pB->transform().translation().x;

    REQUIRE(b - a == Approx(1.0f).margin(1e-3f));
    REQUIRE(a == Approx(-0.5f).margin(1e-3f));
    REQUIRE(b == Approx(+0.5f).margin(1e-3f));
}

TEST_CASE("gdk::scene leaves a struck body alone at a clean impact", "[gdk::collision]")
{
    fixture f;
    const auto pMover = f.scene->make_sphere_collider();
    const auto pIdle = f.scene->make_sphere_collider();
    pMover->set_position({-2, 0, 0});
    pIdle->set_position({0, 0, 0});

    pMover->add_velocity({1, 0, 0});
    f.update(2.0f);

    REQUIRE(pIdle->transform().translation().x == Approx(0.0f).margin(1e-3f));
    REQUIRE(pMover->transform().translation().x == Approx(-1.0f).margin(1e-2f));
}

TEST_CASE("gdk::scene does not slide when the movement mode is none", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider(response_handlers::null_opt);
    pSphere->set_position({-1.5f, 0, 0});

    pSphere->add_velocity({1.0f, 0.25f, 0});
    f.update(1.0f);

    const auto position = pSphere->transform().translation();

    REQUIRE(position.x <= Approx(-1.0f).margin(1e-3f));
    REQUIRE(position.y == Approx(0.125f).margin(1e-2f));
    REQUIRE(pWall->half_extents().x == Approx(0.5f));   
}

TEST_CASE("gdk::scene lets bodies pass through a trigger", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pVolume = f.scene->make_static_sphere_trigger(identity, 0.5f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-2, 0, 0});

    for (int frame = 0; frame < 5; ++frame) {
        pSphere->add_velocity({1, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(pSphere->transform().translation().x == Approx(3.0f).margin(1e-2f));
    REQUIRE(f.collisions.empty());
}

TEST_CASE("gdk::scene reports trigger enter, stay and exit", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pVolume = f.scene->make_static_sphere_trigger(identity, 0.5f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-2, 0, 0});

    const auto count = [&](const event_type aType) {
        std::size_t n = 0;
        for (const auto &e : f.triggers) if (e.type == aType) ++n;
        return n;
    };

    for (int frame = 0; frame < 5; ++frame) {
        pSphere->add_velocity({1, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(count(event_type::enter) == 1);
    REQUIRE(count(event_type::stay) >= 1);
    REQUIRE(count(event_type::exit) == 1);
}

TEST_CASE("gdk::scene does not displace a trigger", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_axis_aligned_box_collider(identity, {0.5f, 0.5f, 0.5f});
    const auto pTrigger = f.scene->make_sphere_trigger();
    pTrigger->set_position({-2, 0, 0});

    const auto count = [&](const event_type aType) {
        std::size_t n = 0;
        for (const auto &e : f.triggers) if (e.type == aType) ++n;
        return n;
    };

    for (int frame = 0; frame < 4; ++frame) {
        pTrigger->add_velocity({1, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(pTrigger->transform().translation().x == Approx(2.0f).margin(1e-2f));
    REQUIRE(count(event_type::enter) == 1);
    REQUIRE(count(event_type::exit) == 1);
}

TEST_CASE("gdk::scene collides against a compound's individual parts", "[gdk::collision]")
{
    const auto build = [](const fixture &f) {
        matrix4x4_type identity;
        auto pWall = f.scene->make_static_compound_collider(identity, [](compound_collider &aBuild) {
            aBuild.add_box({0, +1.5f, 0}, {0.5f, 0.5f, 0.5f});
            aBuild.add_box({0, -1.5f, 0}, {0.5f, 0.5f, 0.5f});
        });
        return pWall;
    };

    SECTION("a body aimed at the gap passes through")
    {
        fixture f;
        const auto pWall = build(f);
        const auto pSphere = f.scene->make_sphere_collider();
        pSphere->set_position({-3, 0, 0});

        for (int frame = 0; frame < 6; ++frame) {
            pSphere->add_velocity({1, 0, 0});
            f.update(1.0f);
        }

        REQUIRE(pSphere->transform().translation().x == Approx(3.0f).margin(1e-2f));
        REQUIRE(f.collisions.empty());
    }

    SECTION("a body aimed at an arm is stopped by it")
    {
        fixture f;
        const auto pWall = build(f);
        const auto pSphere = f.scene->make_sphere_collider();
        pSphere->set_position({-3, 1.5f, 0});

        for (int frame = 0; frame < 6; ++frame) {
            pSphere->add_velocity({1, 0, 0});
            f.update(1.0f);
        }

        REQUIRE(pSphere->transform().translation().x <= Approx(-1.0f).margin(1e-2f));
        REQUIRE(f.count(event_type::enter) == 1);
    }
}

TEST_CASE("gdk::scene reports one contact for a compound, not one per part", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_compound_collider(identity, [](compound_collider &aBuild) {
        aBuild.add_box({0, +0.4f, 0}, {0.5f, 0.5f, 0.5f});
        aBuild.add_box({0, -0.4f, 0}, {0.5f, 0.5f, 0.5f});
        aBuild.add_box({0, 0, 0}, {0.5f, 0.5f, 0.5f});
    });
    REQUIRE(pWall->part_count() == 3);

    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-3, 0, 0});

    pSphere->add_velocity({1, 0, 0});
    f.update(3.0f);

    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene keeps a rotating capsule out of the surface it is pressed against",
    "[gdk::collision]")
{
    fixture f;

    const auto pBox = f.scene->make_axis_aligned_box_collider();
    pBox->set_position({0, 0, 0});

    pBox->set_inverse_overlap_weight(0);

    const auto pCapsule = f.scene->make_capsule_collider();
    pCapsule->set_radius(0.5f);
    pCapsule->set_half_height(0.5f);
    pCapsule->set_position({0, 1.5f, 0});   

    const auto lowest_point_of = [&pCapsule] {
        const auto centre = pCapsule->transform().translation();
        const auto axis = pCapsule->rotation() * vector3_type{0, 0.5f, 0};
        return std::min(centre.y + axis.y, centre.y - axis.y) - 0.5f;
    };

    REQUIRE(lowest_point_of() == Approx(0.5f).margin(1e-4f));

    quaternion_type turn;
    turn.set_from_euler({0, 0, 0.02f});   

    constexpr float NEAR_PARALLEL_LOW = 1.396f;    
    constexpr float NEAR_PARALLEL_HIGH = 1.745f;   

    auto deepestAway = 0.0f;
    auto deepestNearParallel = 0.0f;
    auto highest = lowest_point_of();

    for (int frame = 0; frame < 120; ++frame) {
        pCapsule->add_velocity({0, -4.0f, 0});
        pCapsule->add_rotation(turn);
        f.update(1.0f / 60.0f);

        const auto angle = (frame + 1) * 0.02f;
        const auto depth = 0.5f - lowest_point_of();

        if (angle > NEAR_PARALLEL_LOW && angle < NEAR_PARALLEL_HIGH)
            deepestNearParallel = std::max(deepestNearParallel, depth);
        else
            deepestAway = std::max(deepestAway, depth);

        highest = std::max(highest, lowest_point_of());
    }

    REQUIRE(deepestAway < 1e-4f);

    REQUIRE(deepestNearParallel < 0.02f);

    REQUIRE(highest < 0.5f + 1e-2f);

    REQUIRE(pBox->transform().translation().y == Approx(0.0f).margin(1e-5f));

    const auto finalAxis = pCapsule->rotation() * vector3_type{0, 0.5f, 0};
    REQUIRE(std::abs(finalAxis.y) < 0.45f);   
    REQUIRE(lowest_point_of() == Approx(0.5f).margin(1e-2f));
}

TEST_CASE("gdk::collider turning into a neighbour does not displace an immovable one",
    "[gdk::collision]")
{
    const auto settle = [](const floating_point_type aWeight) {
        fixture f;

        const auto pBox = f.scene->make_axis_aligned_box_collider();
        pBox->set_position({0, 0, 0});
        pBox->set_inverse_overlap_weight(aWeight);

        const auto pCapsule = f.scene->make_capsule_collider();
        pCapsule->set_position({0, 1.6f, 0});

        quaternion_type turn;
        turn.set_from_euler({0, 0, 0.02f});

        for (int frame = 0; frame < 120; ++frame) {
            pCapsule->add_velocity({0, -4.0f, 0});
            pCapsule->add_rotation(turn);
            f.update(1.0f / 60.0f);
        }

        return pBox->transform().translation().y;
    };

    REQUIRE(settle(1.0f) < -0.05f);

    REQUIRE(settle(0.0f) == Approx(0.0f).margin(1e-5f));
}

TEST_CASE("gdk::scene rotates a compound's parts with it", "[gdk::collision]")
{
    const auto arm_reaches = [](const bool aTurned, const vector3_type &aWhere) {
        fixture f;
        matrix4x4_type transform;
        if (aTurned) transform.set_rotation(quaternion_type(
            vector3_type{0, 0, 3.14159265f / 2.0f}));

        const auto pWall = f.scene->make_static_compound_collider(transform, [](compound_collider &aBuild) {
            aBuild.add_box({0, 2.0f, 0}, {0.5f, 0.5f, 0.5f});
        });

        const auto pProbe = f.scene->make_sphere_collider();
        pProbe->set_position(aWhere);

        f.update(0.01f);
        return !f.collisions.empty();
    };

    SECTION("unrotated, the arm is where it was placed")
    {
        REQUIRE(arm_reaches(false, {0, 2, 0}));
        REQUIRE_FALSE(arm_reaches(false, {2, 0, 0}));
    }

    SECTION("a quarter turn about Z carries it onto the negative X axis")
    {
        REQUIRE(arm_reaches(true, {-2, 0, 0}));
        REQUIRE_FALSE(arm_reaches(true, {2, 0, 0}));
        REQUIRE_FALSE(arm_reaches(true, {0, 2, 0}));
    }
}

TEST_CASE("gdk::scene rests a body on a ground plane", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pGround = f.scene->make_static_plane_collider(identity);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({0, 5, 0});

    for (int frame = 0; frame < 10; ++frame) {
        pSphere->add_velocity({0, -1, 0});
        f.update(1.0f);
    }

    REQUIRE(pSphere->transform().translation().y == Approx(0.5f).margin(1e-2f));
}

TEST_CASE("gdk::scene planes are infinite", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pGround = f.scene->make_static_plane_collider(identity);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({500, 5, -300});

    for (int frame = 0; frame < 10; ++frame) {
        pSphere->add_velocity({0, -1, 0});
        f.update(1.0f);
    }

    const auto settled = pSphere->transform().translation();
    REQUIRE(settled.y == Approx(0.5f).margin(1e-2f));
    REQUIRE(settled.x == Approx(500.0f).margin(1e-2f));
}

TEST_CASE("gdk::scene tilts with its plane's transform", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type tilted;
    tilted.set_rotation(quaternion_type(vector3_type{0, 0, 3.14159265f / 6.0f}));

    const auto pSlope = f.scene->make_static_plane_collider(tilted);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({0, 5, 0});

    for (int frame = 0; frame < 10; ++frame) {
        pSphere->add_velocity({0, -1, 0});
        f.update(1.0f);
    }

    REQUIRE(std::abs(pSphere->transform().translation().x) > 0.1f);
    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene sizes a static from its factory argument", "[gdk::collision]")
{
    fixture f;

    SECTION("axis aligned box") {
        matrix4x4_type beneath;
        beneath.set_translation({0, -0.5f, 0});
        const auto pGround = f.scene->make_static_axis_aligned_box_collider(beneath, {4, 0.5f, 4});

        const auto pSphere = f.scene->make_sphere_collider();
        pSphere->set_position({3, 5, 0});
        for (int frame = 0; frame < 20; ++frame) { pSphere->add_velocity({0, -1, 0}); f.update(1.0f); }

        REQUIRE(pSphere->transform().translation().y == Approx(0.5f).margin(1e-2f));
        REQUIRE(pGround->half_extents().x == Approx(4.0f));
    }

    SECTION("sphere") {
        matrix4x4_type identity;
        const auto pBall = f.scene->make_static_sphere_collider(identity, 3.0f);

        const auto pBody = f.scene->make_sphere_collider();
        pBody->set_position({-8, 0, 0});
        for (int frame = 0; frame < 20; ++frame) { pBody->add_velocity({1, 0, 0}); f.update(1.0f); }

        REQUIRE(pBody->transform().translation().x == Approx(-3.5f).margin(1e-2f));
        REQUIRE(pBall->radius() == Approx(3.0f));
    }

    SECTION("capsule") {
        matrix4x4_type identity;
        const auto pPost = f.scene->make_static_capsule_collider(identity, 1.0f, 4.0f);

        const auto pBody = f.scene->make_sphere_collider();
        pBody->set_position({-8, 3, 0});
        for (int frame = 0; frame < 20; ++frame) { pBody->add_velocity({1, 0, 0}); f.update(1.0f); }

        REQUIRE(pBody->transform().translation().x == Approx(-1.5f).margin(1e-2f));
        REQUIRE(pPost->half_height() == Approx(4.0f));
    }

    SECTION("oriented box") {
        matrix4x4_type identity;
        const auto pBox = f.scene->make_static_obb_collider(identity, {2, 2, 2});

        const auto pBody = f.scene->make_sphere_collider();
        pBody->set_position({-8, 0, 0});
        for (int frame = 0; frame < 20; ++frame) { pBody->add_velocity({1, 0, 0}); f.update(1.0f); }

        REQUIRE(pBody->transform().translation().x == Approx(-2.5f).margin(1e-2f));
        REQUIRE(pBox->half_extents().y == Approx(2.0f));
    }
}

TEST_CASE("gdk::scene handles a static body far larger than a broadphase cell", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type beneath;
    beneath.set_translation({0, -0.5f, 0});

    const auto pGround = f.scene->make_static_axis_aligned_box_collider(beneath, {200.0f, 0.5f, 200.0f});

    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({0, 5, 0});

    for (int frame = 0; frame < 10; ++frame) {
        pSphere->add_velocity({0, -1, 0});
        f.update(1.0f);
    }

    REQUIRE(pSphere->transform().translation().y == Approx(0.5f).margin(1e-2f));
}

TEST_CASE("gdk::scene raycast finds the nearest collider", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_axis_aligned_box_collider(identity, {0.5f, 0.5f, 0.5f});

    SECTION("a ray aimed at a box reports the distance to its face")
    {
        const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
        REQUIRE(hit->collider == const_collider_ptr_type(pWall));
        require_vector_near(hit->normal, {-1, 0, 0}, 1e-2f);
    }

    SECTION("a ray that stops short reports nothing")
    {
        REQUIRE_FALSE(f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 5.0f).has_value());
    }

    SECTION("a ray pointing away reports nothing")
    {
        REQUIRE_FALSE(f.scene->raycast({-10, 0, 0}, {-1, 0, 0}, 20.0f).has_value());
    }

    SECTION("direction need not be normalised")
    {
        const auto hit = f.scene->raycast({-10, 0, 0}, {7, 0, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
    }

    SECTION("a ray from inside a collider it is told to ignore goes through it to what is beyond")
    {
        const auto pCapsule = f.scene->make_capsule_collider();
        pCapsule->set_position({-3, 0, 0});

        const auto first = f.scene->raycast({-3, 0, 0}, {1, 0, 0}, 20.0f);

        REQUIRE(first.has_value());
        REQUIRE(first->collider == const_collider_ptr_type(pCapsule));

        const auto through = f.scene->raycast({-3, 0, 0}, {1, 0, 0}, 20.0f, pCapsule.get());

        REQUIRE(through.has_value());
        REQUIRE(through->collider == const_collider_ptr_type(pWall));
        REQUIRE(through->distance == Approx(2.5f).margin(1e-2f));
    }

    SECTION("a ray looks only for colliders on its layers, and goes through the rest")
    {
        constexpr collision_layers_type BODIES = 2, WORLD = DEFAULT_COLLISION_LAYER;

        const auto pCapsule = f.scene->make_capsule_collider();
        pCapsule->set_position({-3, 0, 0});
        pCapsule->set_collision_layers(BODIES, ALL_COLLISION_LAYERS);

        REQUIRE(f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f)->collider == const_collider_ptr_type(pCapsule));

        const auto world = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f, nullptr, WORLD);

        REQUIRE(world.has_value());
        REQUIRE(world->collider == const_collider_ptr_type(pWall));
        REQUIRE(world->distance == Approx(9.5f).margin(1e-2f));

        REQUIRE(f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f, nullptr, BODIES)->collider
            == const_collider_ptr_type(pCapsule));
        REQUIRE_FALSE(f.scene->raycast({-2, 0, 0}, {1, 0, 0}, 20.0f, nullptr, BODIES).has_value());

        REQUIRE_FALSE(f.scene->raycast_static({-10, 0, 0}, {1, 0, 0}, 20.0f, BODIES).has_value());
        REQUIRE(f.scene->raycast_static({-10, 0, 0}, {1, 0, 0}, 20.0f, WORLD).has_value());
    }

    SECTION("a ray for the world alone goes through whatever moves")
    {
        const auto pCapsule = f.scene->make_capsule_collider();
        pCapsule->set_position({-3, 0, 0});

        const auto hit = f.scene->raycast_static({-10, 0, 0}, {1, 0, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pWall));
        REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));

        REQUIRE_FALSE(f.scene->raycast_static({-3, 5, 0}, {0, -1, 0}, 4.0f).has_value());
        REQUIRE(f.scene->raycast({-3, 5, 0}, {0, -1, 0}, 4.0f).has_value());
    }

    SECTION("a ray aimed down at an oriented box reports the normal of the face it struck, facing back up the ray")
    {
        matrix4x4_type placed;
        placed.set_translation({0, -5, 0});

        const auto pSlab = f.scene->make_static_obb_collider(placed, {2.0f, 0.5f, 2.0f});

        const auto hit = f.scene->raycast({0, -2, 0}, {0, -1, 0}, 10.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pSlab));
        REQUIRE(hit->distance == Approx(2.5f).margin(1e-2f));
        require_vector_near(hit->normal, {0, 1, 0}, 1e-2f);
    }

    SECTION("a degenerate ray reports nothing rather than misbehaving")
    {
        REQUIRE_FALSE(f.scene->raycast({-10, 0, 0}, {0, 0, 0}, 20.0f).has_value());
        REQUIRE_FALSE(f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 0.0f).has_value());
    }
}

TEST_CASE("gdk::scene raycast returns the closest of several hits", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type near;
    near.set_translation({-3, 0, 0});
    matrix4x4_type far;
    far.set_translation({3, 0, 0});

    const auto pNear = f.scene->make_static_axis_aligned_box_collider(near, {0.5f, 0.5f, 0.5f});
    const auto pFar = f.scene->make_static_axis_aligned_box_collider(far, {0.5f, 0.5f, 0.5f});

    const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 30.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider == const_collider_ptr_type(pNear));
    REQUIRE(hit->distance == Approx(6.5f).margin(1e-2f));
}

TEST_CASE("gdk::scene raycast reaches every shape", "[gdk::collision]")
{
    matrix4x4_type identity;

    SECTION("sphere")
    {
        fixture f;
        const auto pTarget = f.scene->make_static_sphere_collider(identity, 0.5f);
        const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);
        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pTarget));
        REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
    }

    SECTION("axis aligned box")
    {
        fixture f;
        const auto pTarget = f.scene->make_static_axis_aligned_box_collider(identity, {0.5f, 0.5f, 0.5f});
        const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);
        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pTarget));
        REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
    }

    SECTION("capsule")
    {
        fixture f;
        const auto pTarget = f.scene->make_static_capsule_collider(identity, 0.5f, 0.5f);
        const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);
        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pTarget));
        REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
    }

    SECTION("oriented box")
    {
        fixture f;
        const auto pTarget = f.scene->make_static_obb_collider(identity, {0.5f, 0.5f, 0.5f});
        const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);
        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pTarget));
        REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
    }

    SECTION("plane, struck from above")
    {
        fixture f;
        const auto pTarget = f.scene->make_static_plane_collider(identity);
        const auto hit = f.scene->raycast({0, 10, 0}, {0, -1, 0}, 20.0f);
        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pTarget));
        REQUIRE(hit->distance == Approx(10.0f).margin(1e-2f));
    }
}

TEST_CASE("gdk::scene raycast ignores triggers", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    matrix4x4_type infront;
    infront.set_translation({-3, 0, 0});

    const auto pVolume = f.scene->make_static_sphere_trigger(infront, 0.5f);
    const auto pWall = f.scene->make_static_axis_aligned_box_collider(identity, {0.5f, 0.5f, 0.5f});

    const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider == const_collider_ptr_type(pWall));
    REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
}

TEST_CASE("gdk::scene rests a body on a static mesh floor", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pFloor = f.scene->make_static_mesh_collider(identity, make_quad_mesh(5.0f));

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 5, 0});

    for (int frame = 0; frame < 60; ++frame) {
        pBall->add_velocity({0, -10, 0});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(pBall->transform().translation().y == Approx(0.5f).margin(1e-2f));
    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene passes a body through a mesh trigger", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pZone = f.scene->make_static_mesh_trigger(identity, make_quad_mesh(5.0f));

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 5, 0});

    for (int frame = 0; frame < 10; ++frame) {
        pBall->add_velocity({0, -1, 0});
        f.update(1.0f);
    }

    REQUIRE(pBall->transform().translation().y < -1.0f);
    REQUIRE_FALSE(f.triggers.empty());
    REQUIRE(f.count(event_type::enter) == 0);
}

TEST_CASE("gdk::scene lifts a body on a kinematic mesh", "[gdk::collision]")
{
    fixture f;
    const auto pLift = f.scene->make_mesh_collider();
    pLift->set_mesh(make_quad_mesh(5.0f));
    pLift->set_position({0, -2, 0});
    pLift->set_kinematic(true);

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 0, 0});

    for (int frame = 0; frame < 14; ++frame) {
        pLift->add_velocity({0, 0.25f, 0});
        f.update(1.0f);
    }

    const auto gap = pBall->transform().translation().y - pLift->transform().translation().y;
    REQUIRE(gap == Approx(0.5f).margin(1e-2f));
    REQUIRE(pBall->transform().translation().y > 1.0f);
}

TEST_CASE("gdk::scene holds events until they are asked for", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    pSphere->add_velocity({1, 0, 0});
    f.scene->update(1.0f);                  

    REQUIRE(f.collisions.empty());          

    f.scene->process_events();
    REQUIRE(f.count(event_type::enter) == 1);

    f.scene->process_events();
    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene accumulates events across several updates", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    for (int frame = 0; frame < 3; ++frame) {
        pSphere->add_velocity({1, 0, 0});
        f.scene->update(1.0f);
    }

    REQUIRE(f.collisions.empty());
    f.scene->process_events();

    REQUIRE(f.count(event_type::enter) == 1);
    REQUIRE(f.count(event_type::stay) >= 1);
    REQUIRE(f.collisions.front().type == event_type::enter);
}

TEST_CASE("gdk::scene lets a collider go though events naming it are waiting", "[gdk::collision]")
{
    fixture f;
    auto pWall = add_static_box_at_origin(f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    for (int frame = 0; frame < 5; ++frame) {
        pSphere->add_velocity({1, 0, 0});
        f.scene->update(1.0f);
    }

    const const_collider_weak_ptr_type wall = pWall;

    pWall.reset();

    REQUIRE(wall.expired());
    REQUIRE_FALSE(f.scene->raycast({0, 5, 0}, {0, -1, 0}, 10));

    f.scene->process_events();

    REQUIRE(f.collisions.empty());
}

TEST_CASE("gdk::scene with nobody listening lets its colliders go, drained or not", "[gdk::collision]")
{
    const auto pScene = impl_collision_scene::make({}, {});

    matrix4x4_type identity;
    auto pWall = pScene->make_static_axis_aligned_box_collider(identity, {0.5f, 0.5f, 0.5f});
    const auto pSphere = pScene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    for (int frame = 0; frame < 5; ++frame) {
        pSphere->add_velocity({1, 0, 0});
        pScene->update(1.0f);
    }

    REQUIRE(pSphere->transform().translation().x <= Approx(-1.0f).margin(1e-3f));

    const const_collider_weak_ptr_type wall = pWall;

    pWall.reset();

    REQUIRE(wall.expired());

    pScene->process_events();
}

TEST_CASE("gdk::scene takes a static's geometry at construction", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pFloor = f.scene->make_static_mesh_collider(identity, make_quad_mesh(5.0f));

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({3, 5, 3});          

    for (int frame = 0; frame < 120; ++frame) {
        pBall->add_velocity({0, -10, 0});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(pBall->transform().translation().y == Approx(0.5f).margin(1e-2f));

    const auto pStack = f.scene->make_static_compound_collider(identity, [](compound_collider &aBuild) {
        aBuild.add_box({0, 0, 0}, {0.5f, 0.5f, 0.5f});
        aBuild.add_sphere({0, 1.0f, 0}, 0.5f);
    });

    REQUIRE(pStack->part_count() == 2);
}

TEST_CASE("gdk::scene shares one mesh between colliders", "[gdk::collision]")
{
    fixture f;
    const auto pShared = make_quad_mesh(2.0f);

    matrix4x4_type leftTransform, rightTransform;
    leftTransform.set_translation({-10, 0, 0});
    rightTransform.set_translation({10, 0, 0});

    const auto pLeft = f.scene->make_static_mesh_collider(leftTransform, pShared);
    const auto pRight = f.scene->make_static_mesh_collider(rightTransform, pShared);

    REQUIRE(pLeft->mesh() == pRight->mesh());

    const auto drop = [&](const floating_point_type aX) {
        const auto pBall = f.scene->make_sphere_collider();
        pBall->set_position({aX, 5, 0});
        for (int frame = 0; frame < 60; ++frame) {
            pBall->add_velocity({0, -10, 0});
            f.update(1.0f / 60.0f);
        }
        return pBall->transform().translation().y;
    };

    REQUIRE(drop(-10.0f) == Approx(0.5f).margin(1e-2f));
    REQUIRE(drop(10.0f) == Approx(0.5f).margin(1e-2f));
    REQUIRE(drop(0.0f) < -1.0f);
}

TEST_CASE("gdk::scene tolerates a mesh collider with no mesh", "[gdk::collision]")
{
    fixture f;
    const auto pEmpty = f.scene->make_mesh_collider();
    pEmpty->set_position({0, 0, 0});

    REQUIRE(pEmpty->mesh() == nullptr);

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 5, 0});

    for (int frame = 0; frame < 60; ++frame) {
        pBall->add_velocity({0, -10, 0});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(pBall->transform().translation().y < -1.0f);
    REQUIRE(f.count(event_type::enter) == 0);
}

TEST_CASE("gdk::scene lets two meshes coexist without throwing", "[gdk::collision]")
{
    fixture f;
    const auto pA = f.scene->make_mesh_collider();
    pA->set_mesh(make_quad_mesh(5.0f));
    pA->set_position({0, 0, 0});

    const auto pB = f.scene->make_mesh_collider();
    pB->set_mesh(make_quad_mesh(5.0f));
    pB->set_position({0, 2, 0});

    REQUIRE_NOTHROW([&] {
        for (int frame = 0; frame < 10; ++frame) {
            pB->add_velocity({0, -1, 0});
            f.update(1.0f);
        }
    }());

    REQUIRE(pB->transform().translation().y < -5.0f);
    REQUIRE(f.count(event_type::enter) == 0);
}

TEST_CASE("gdk::scene raycasts against a mesh", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pFloor = f.scene->make_static_mesh_collider(identity, make_quad_mesh(5.0f));

    f.update(1.0f / 60.0f);   

    const auto hit = f.scene->raycast({0, 10, 0}, {0, -1, 0}, 20.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->distance == Approx(10.0f).margin(1e-2f));
    require_vector_near(hit->normal, {0, 1, 0}, 1e-2f);

    REQUIRE_FALSE(f.scene->raycast({20, 10, 0}, {0, -1, 0}, 20.0f).has_value());
}

TEST_CASE("gdk::scene reports the velocity a body finished its step with", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_axis_aligned_box_collider(identity, {5, 5, 0.5f});

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 0, -3});

    require_vector_near(pBall->resolved_velocity(), vector3_type::zero, 1e-5f);

    for (int frame = 0; frame < 40; ++frame) {
        pBall->add_velocity({2, 0, 4});
        f.update(1.0f / 60.0f);
    }

    require_vector_near(pBall->velocity(), vector3_type::zero, 1e-5f);

    const auto resolved = pBall->resolved_velocity();
    REQUIRE(resolved.x > 1.0f);                                  
    REQUIRE(resolved.z == Approx(0.0f).margin(1e-2f));           
    REQUIRE(pBall->transform().translation().z <= Approx(-0.9f).margin(1e-2f));
}

TEST_CASE("gdk::scene lets a caller keep its own velocity across frames", "[gdk::collision]")
{
    const auto settle = [](const bool aCloseTheLoop) {
        fixture f;
        matrix4x4_type identity;
        const auto pTerrain = f.scene->make_static_heightfield_collider(identity,
            make_terrain(17, 17,
            [](const std::size_t aColumn, const std::size_t aRow) {
                return std::sin(aColumn * 0.6f) * 0.5f + std::cos(aRow * 0.45f) * 0.5f;
            }));

        const auto pBall = f.scene->make_sphere_collider(
            response_handlers::slide_projecting());
        pBall->set_position({0, 2, 0});

        constexpr auto dt = 1.0f / 60.0f;
        vector3_type myVelocity;
        vector3_type previous;
        floating_point_type travelled = 0;

        for (int frame = 0; frame < 600; ++frame) {
            myVelocity += vector3_type{0, -9.8f, 0} * dt;  
            myVelocity *= std::pow(0.05f, dt);                       
            pBall->add_velocity(myVelocity);
            f.update(dt);
            if (aCloseTheLoop) myVelocity = pBall->resolved_velocity();

            const auto position = pBall->transform().translation();
            if (frame > 540) travelled += (position - previous).length();
            previous = position;
        }

        return std::make_pair(pBall->transform().translation(), travelled);
    };

    const auto open = settle(false);
    const auto closed = settle(true);

    REQUIRE(closed.second < open.second * 0.1f);
    REQUIRE(closed.first.y == Approx(-0.498f).margin(0.03f));
    REQUIRE(closed.first.z == Approx(-1.0f).margin(0.05f));
    REQUIRE(std::abs(closed.first.x) < 0.2f);
}

TEST_CASE("gdk::scene settles under a held input from every direction", "[gdk::collision]")
{
    const auto settles = [](std::span<const vector3_type> aPath, const std::size_t aTail) {
        const auto first = aPath.size() - aTail;
        floating_point_type walked = 0;
        for (auto i = first + 1; i < aPath.size(); ++i) walked += (aPath[i] - aPath[i - 1]).length();

        if (walked < 1e-3f) return true;
        return (aPath.back() - aPath[first]).length() > walked * 0.5f;
    };

    const std::vector<std::string> shapes{"sphere", "capsule", "box", "obb"};
    const std::vector<std::string> worlds{"mesh box", "flat terrain", "static box"};

    for (const auto &shape : shapes)
        for (const auto &world : worlds)
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (!dx && !dy && !dz) continue;

                        INFO(shape << " in " << world << ", driven ("
                            << dx << "," << dy << "," << dz << ")");

                        fixture f;
                        matrix4x4_type identity;

                        std::vector<const_collider_ptr_type> heldWorld;

                        if (world == "mesh box") {
                            const auto pBox = f.scene->make_static_mesh_collider(identity, make_open_box_mesh());
                            heldWorld.push_back(pBox);
                        }
                        else if (world == "flat terrain") {
                            const auto pTerrain = f.scene->make_static_heightfield_collider(identity,
                                make_terrain(17, 17,
                                [](std::size_t, std::size_t) { return 0.0f; }));
                            heldWorld.push_back(pTerrain);
                        }
                        else {
                            const auto pSolid = f.scene->make_static_axis_aligned_box_collider(identity, {2, 2, 2});
                            heldWorld.push_back(pSolid);
                        }

                        collider_ptr_type pBody;
                        if (shape == "sphere") pBody = f.scene->make_sphere_collider();
                        else if (shape == "capsule") {
                            const auto pCapsule = f.scene->make_capsule_collider();
                            pCapsule->set_radius(0.4f);
                            pCapsule->set_half_height(0.6f);
                            pBody = pCapsule;
                        }
                        else if (shape == "box") pBody = f.scene->make_axis_aligned_box_collider();
                        else pBody = f.scene->make_obb_collider();

                        pBody->set_position(world == "static box"
                            ? vector3_type{0, 3.5f, 0}
                            : vector3_type{0, -0.2f, 0});

                        const auto drive = vector3_type{
                            static_cast<floating_point_type>(dx),
                            static_cast<floating_point_type>(dy),
                            static_cast<floating_point_type>(dz)}.normal() * 5.0f;

                        std::vector<vector3_type> path;
                        for (int frame = 0; frame < 180; ++frame) {
                            pBody->add_velocity(drive);
                            f.update(1.0f / 60.0f);
                            path.push_back(pBody->transform().translation());
                        }

                        REQUIRE(settles(path, 40));
                    }
}

TEST_CASE("gdk::scene settles in a concave mesh corner", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pBox = f.scene->make_static_mesh_collider(identity, make_open_box_mesh());

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, -0.5f, 0});

    const auto step = [&] {
        pBall->add_velocity({-4, -4, 0});
        f.update(1.0f / 60.0f);
        return pBall->transform().translation();
    };

    for (int frame = 0; frame < 30; ++frame) step();

    const auto settled = step();
    REQUIRE(settled.x == Approx(-1.0f).margin(1e-3f));
    REQUIRE(settled.y == Approx(-0.5f).margin(1e-3f));

    for (int frame = 0; frame < 10; ++frame) {
        const auto position = step();
        REQUIRE(position.x == Approx(settled.x).margin(1e-4f));
        REQUIRE(position.y == Approx(settled.y).margin(1e-4f));
    }
}

TEST_CASE("gdk::scene slides down a uniform slope without catching", "[gdk::collision]")
{
    for (const int degrees : {5, 10, 15, 20, 25, 30, 35, 40})
        for (const auto z : {0.0f, 0.13f, 0.25f}) {
            INFO(degrees << " degree slope, travelling along z = " << z);

            fixture f;
            matrix4x4_type identity;

            const auto rise = std::tan(degrees * 3.14159265f / 180.0f) * 0.5f;
            const auto pSlope = f.scene->make_static_heightfield_collider(identity, impl_heightfield_data::make(33, 33, [&] {
                std::vector<floating_point_type> heights(33 * 33);
                for (std::size_t row = 0; row < 33; ++row)
                    for (std::size_t column = 0; column < 33; ++column)
                        heights[row * 33 + column] = column * rise;
                return heights;
            }(), {0.5f, 0.5f}));

            const auto pBall = f.scene->make_sphere_collider();
            const auto surface = (3.0f + 8.0f) / 0.5f * rise;      
            pBall->set_position({3.0f, surface + 1.5f, z});

            auto previous = pBall->transform().translation();
            const auto start = previous;
            std::size_t freeze = 0;
            std::size_t longestFreeze = 0;

            for (int frame = 0; frame < 300; ++frame) {
                pBall->add_velocity({0, -8, 0});
                f.update(1.0f / 60.0f);

                const auto position = pBall->transform().translation();
                if (frame > 30) {
                    if ((position - previous).length() < 1e-4f)
                        longestFreeze = std::max(++freeze, longestFreeze);
                    else freeze = 0;
                }
                previous = position;
            }

            REQUIRE(longestFreeze < 4);
            REQUIRE(start.x - pBall->transform().translation().x > 5.0f);
        }
}

TEST_CASE("gdk::scene keeps a body moving while it slides on terrain", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pTerrain = f.scene->make_static_heightfield_collider(identity,
        make_terrain(17, 17,
        [](const std::size_t aColumn, const std::size_t aRow) {
            return std::sin(aColumn * 0.6f) * 0.5f + std::cos(aRow * 0.45f) * 0.5f;
        }));

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({2.2f, 3, 1.1f});

    auto previous = pBall->transform().translation();
    std::size_t longestFreeze = 0;
    std::size_t currentFreeze = 0;

    for (int frame = 0; frame < 240; ++frame) {
        pBall->add_velocity({0, -6, 0});
        f.update(1.0f / 60.0f);

        const auto position = pBall->transform().translation();

        if (frame > 40) {
            if ((position - previous).length() < 1e-4f) longestFreeze = std::max(++currentFreeze, longestFreeze);
            else currentFreeze = 0;
        }

        previous = position;
    }

    REQUIRE(longestFreeze < 4);
}

TEST_CASE("gdk::scene pushes a body out when its rotation drives it in", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_axis_aligned_box_collider(identity, {5, 5, 0.5f});

    const auto pCapsule = f.scene->make_capsule_collider();
    pCapsule->set_radius(0.4f);
    pCapsule->set_half_height(1.0f);
    pCapsule->set_position({0, 0, -3});

    for (int frame = 0; frame < 60; ++frame) {
        pCapsule->add_velocity({0, 0, 4});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(pCapsule->transform().translation().z == Approx(-0.9f).margin(1e-3f));

    const auto pImplementation = std::dynamic_pointer_cast<impl_capsule_collider>(pCapsule);
    REQUIRE(pImplementation != nullptr);

    for (int frame = 0; frame < 30; ++frame) {
        pImplementation->set_rotation(quaternion_type::from_euler(
            {(frame + 1) / 30.0f * 3.14159265f / 2.0f, 0, 0}));
        f.update(1.0f / 60.0f);
    }

    const auto z = pCapsule->transform().translation().z;
    REQUIRE(z <= Approx(-1.9f).margin(1e-2f));
    REQUIRE(z > -4.0f);
}

TEST_CASE("gdk::scene settles two bodies closing on each other", "[gdk::collision]")
{
    fixture f;
    const auto pA = f.scene->make_sphere_collider();
    const auto pB = f.scene->make_sphere_collider();
    pA->set_position({-4, 0, 0});
    pB->set_position({+4, 0, 0});

    const auto step = [&] {
        pA->add_velocity({3, 0, 0});
        pB->add_velocity({-3, 0, 0});
        f.update(1.0f / 60.0f);
    };

    for (int frame = 0; frame < 120; ++frame) step();

    REQUIRE(pB->transform().translation().x - pA->transform().translation().x
        == Approx(1.0f).margin(1e-3f));
    REQUIRE(pA->transform().translation().x == Approx(-0.5f).margin(1e-3f));
    REQUIRE(pB->transform().translation().x == Approx(+0.5f).margin(1e-3f));

    const auto restingA = pA->transform().translation().x;
    for (int frame = 0; frame < 10; ++frame) {
        step();
        REQUIRE(pA->transform().translation().x == Approx(restingA).margin(1e-4f));
    }
}

TEST_CASE("gdk::scene rests a body on static terrain", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pTerrain = f.scene->make_static_heightfield_collider(identity, make_flat_terrain(5, 5));

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 5, 0});

    for (int frame = 0; frame < 60; ++frame) {
        pBall->add_velocity({0, -10, 0});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(pBall->transform().translation().y == Approx(0.5f).margin(1e-2f));
    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene slides a body down a terrain slope", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pSlope = f.scene->make_static_heightfield_collider(identity, make_terrain(9, 9,
        [](const std::size_t aColumn, const std::size_t) { return static_cast<float>(aColumn); }));

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 5.0f, 0});

    for (int frame = 0; frame < 30; ++frame) {
        pBall->add_velocity({0, -10, 0});
        f.update(1.0f / 60.0f);
    }

    const auto position = pBall->transform().translation();

    REQUIRE(position.x < -3.0f);
    REQUIRE(position.x > -4.0f);       
    REQUIRE(position.y - (position.x + 4.0f) == Approx(0.70711f).margin(1e-2f));
}

TEST_CASE("gdk::scene shares one heightfield between colliders", "[gdk::collision]")
{
    fixture f;
    const auto pShared = make_flat_terrain(5, 5);

    matrix4x4_type leftTransform, rightTransform;
    leftTransform.set_translation({-20, 0, 0});
    rightTransform.set_translation({20, 0, 0});

    const auto pLeft = f.scene->make_static_heightfield_collider(leftTransform, pShared);
    const auto pRight = f.scene->make_static_heightfield_collider(rightTransform, pShared);

    REQUIRE(pLeft->heightfield() == pRight->heightfield());

    const auto drop = [&](const floating_point_type aX) {
        const auto pBall = f.scene->make_sphere_collider();
        pBall->set_position({aX, 5, 0});
        for (int frame = 0; frame < 60; ++frame) {
            pBall->add_velocity({0, -10, 0});
            f.update(1.0f / 60.0f);
        }
        return pBall->transform().translation().y;
    };

    REQUIRE(drop(-20.0f) == Approx(0.5f).margin(1e-2f));
    REQUIRE(drop(20.0f) == Approx(0.5f).margin(1e-2f));
    REQUIRE(drop(0.0f) < -1.0f);       
}

TEST_CASE("gdk::scene tolerates a heightfield collider with no terrain", "[gdk::collision]")
{
    fixture f;
    const auto pEmpty = f.scene->make_heightfield_collider();
    pEmpty->set_position({0, 0, 0});

    REQUIRE(pEmpty->heightfield() == nullptr);

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 5, 0});

    for (int frame = 0; frame < 60; ++frame) {
        pBall->add_velocity({0, -10, 0});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(pBall->transform().translation().y < -1.0f);
    REQUIRE(f.count(event_type::enter) == 0);
}

TEST_CASE("gdk::scene raycasts against terrain", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pTerrain = f.scene->make_static_heightfield_collider(identity, make_flat_terrain(5, 5));

    f.update(1.0f / 60.0f);     

    const auto hit = f.scene->raycast({0, 10, 0}, {0, -1, 0}, 20.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->distance == Approx(10.0f).margin(1e-2f));
    REQUIRE_FALSE(f.scene->raycast({20, 10, 0}, {0, -1, 0}, 20.0f).has_value());
}

TEST_CASE("gdk::scene halts a weight zero dynamic at a contact", "[gdk::collision]")
{
    fixture f;
    const auto pMover = f.scene->make_axis_aligned_box_collider();
    pMover->set_position({-4, 0, 0});
    pMover->set_inverse_overlap_weight(0.0f);

    const auto pActor = f.scene->make_sphere_collider();
    pActor->set_position({0, 0, 0});

    for (int frame = 0; frame < 6; ++frame) {
        pMover->add_velocity({1, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(pMover->transform().translation().x == Approx(-1.0f).margin(1e-2f));
    REQUIRE(pActor->transform().translation().x == Approx(0.0f).margin(1e-2f));
}

TEST_CASE("gdk::scene does not stop a kinematic body", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);

    const auto pLift = f.scene->make_axis_aligned_box_collider();
    pLift->set_position({-4, 0, 0});
    pLift->set_kinematic(true);

    REQUIRE(f.scene->raycast({-4, 0, 0}, {1, 0, 0}, 10.0f).has_value());

    for (int frame = 0; frame < 6; ++frame) {
        pLift->add_velocity({1, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(pLift->transform().translation().x == Approx(2.0f).margin(1e-2f));
}

TEST_CASE("gdk::scene tests nothing between two kinematic bodies", "[gdk::collision]")
{
    fixture f;

    const auto pHull = f.scene->make_axis_aligned_box_collider();
    pHull->set_position({0, 0, 0});
    pHull->set_kinematic(true);

    const auto pOther = f.scene->make_axis_aligned_box_collider();
    pOther->set_position({0.5f, 0, 0});
    pOther->set_kinematic(true);

    for (int frame = 0; frame < 3; ++frame) {
        pHull->add_velocity({0.1f, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(f.collisions.empty());

    SECTION("**and a dynamic one still meets them**") {
        const auto pBall = f.scene->make_sphere_collider();
        pBall->set_position({0.4f, 0.9f, 0});

        f.update(1.0f);

        REQUIRE_FALSE(f.collisions.empty());
    }
}

TEST_CASE("gdk::scene keeps no body alive after an update, with kinematic bodies about", "[gdk::collision]")
{
    fixture f;

    const auto pLift = f.scene->make_axis_aligned_box_collider();
    pLift->set_kinematic(true);

    auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 2, 0});

    f.update(1.0f);

    const std::weak_ptr<gdk::collisions::sphere_collider> ball = pBall;
    pBall.reset();

    REQUIRE(ball.expired());
}

TEST_CASE("gdk::scene lets a kinematic body push a dynamic one", "[gdk::collision]")
{
    fixture f;
    const auto pPlatform = f.scene->make_axis_aligned_box_collider();
    pPlatform->set_half_extents({2, 2, 2});
    pPlatform->set_position({-4, 0, 0});
    pPlatform->set_kinematic(true);

    const auto pActor = f.scene->make_sphere_collider();
    pActor->set_position({0, 0, 0});

    for (int frame = 0; frame < 8; ++frame) {
        pPlatform->add_velocity({0.25f, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(pActor->transform().translation().x > 0.1f);

    const auto gap = pActor->transform().translation().x - pPlatform->transform().translation().x;
    REQUIRE(gap == Approx(2.5f).margin(1e-2f));
}

TEST_CASE("gdk::scene sweeps a dynamic body against a kinematic one", "[gdk::collision]")
{
    fixture f;
    const auto pPlatform = f.scene->make_axis_aligned_box_collider();
    pPlatform->set_position({0, 0, 0});
    pPlatform->set_kinematic(true);

    const auto pActor = f.scene->make_sphere_collider();
    pActor->set_position({-10, 0, 0});

    pActor->add_velocity({9, 0, 0});
    f.update(1.0f);

    REQUIRE(pActor->transform().translation().x == Approx(-1.0f).margin(1e-2f));
    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene reports a kinematic body's contacts", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);

    const auto pLift = f.scene->make_axis_aligned_box_collider();
    pLift->set_position({-4, 0, 0});
    pLift->set_kinematic(true);

    pLift->add_velocity({4, 0, 0});
    f.update(1.0f);

    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene restores a weight when a body stops being kinematic", "[gdk::collision]")
{
    fixture f;
    const auto pBody = f.scene->make_sphere_collider();
    pBody->set_inverse_overlap_weight(3.0f);
    pBody->set_kinematic(true);

    REQUIRE(pBody->is_kinematic());
    REQUIRE(pBody->inverse_overlap_weight() == Approx(3.0f));

    pBody->set_kinematic(false);

    REQUIRE_FALSE(pBody->is_kinematic());
    REQUIRE(pBody->inverse_overlap_weight() == Approx(3.0f));
}

TEST_CASE("gdk::scene forgets a static once its owner does", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    {
        const auto pWall = f.scene->make_static_axis_aligned_box_collider(identity, {0.5f, 0.5f, 0.5f});

        pSphere->add_velocity({1, 0, 0});
        f.update(1.0f);
        REQUIRE(f.count(event_type::enter) == 1);
        REQUIRE(pSphere->transform().translation().x <= Approx(-1.0f).margin(1e-2f));
    }
    f.collisions.clear();
    f.triggers.clear();

    for (int frame = 0; frame < 6; ++frame) {
        pSphere->add_velocity({1, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(pSphere->transform().translation().x > 1.0f);
}

TEST_CASE("gdk::scene raycasts against a persistent static", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_axis_aligned_box_collider(identity, {0.5f, 0.5f, 0.5f});

    const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider == const_collider_ptr_type(pWall));
    REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
}

namespace {
    [[nodiscard]] vector3_type turn_by(const quaternion_type &aQ,
        const vector3_type &aV) {
        const vector3_type axis{aQ.x, aQ.y, aQ.z};
        const vector3_type t{
            2.0f * (axis.y * aV.z - axis.z * aV.y),
            2.0f * (axis.z * aV.x - axis.x * aV.z),
            2.0f * (axis.x * aV.y - axis.y * aV.x)};
        return vector3_type{
            aV.x + aQ.w * t.x + (axis.y * t.z - axis.z * t.y),
            aV.y + aQ.w * t.y + (axis.z * t.x - axis.x * t.z),
            aV.z + aQ.w * t.z + (axis.x * t.y - axis.y * t.x)};
    }

    [[nodiscard]] floating_point_type penetration_into_box(
        const vector3_type &aCapsuleCentre, const quaternion_type &aCapsuleRotation,
        const floating_point_type aRadius, const floating_point_type aHalfHeight,
        const vector3_type &aBoxCentre, const quaternion_type &aBoxRotation,
        const vector3_type &aHalfExtents) {
        const auto axis = turn_by(aCapsuleRotation, vector3_type{0, aHalfHeight, 0});
        const auto a = aCapsuleCentre - axis;
        const auto b = aCapsuleCentre + axis;

        quaternion_type inverse;
        inverse.w = aBoxRotation.w;
        inverse.x = -aBoxRotation.x;
        inverse.y = -aBoxRotation.y;
        inverse.z = -aBoxRotation.z;

        auto nearest = std::numeric_limits<floating_point_type>::max();
        constexpr int SAMPLES = 2000;
        for (int i = 0; i <= SAMPLES; ++i) {
            const auto local = turn_by(inverse,
                (a + (b - a) * (static_cast<floating_point_type>(i) / SAMPLES)) - aBoxCentre);

            const auto clamp = [](const floating_point_type aValue,
                const floating_point_type aLimit) {
                return aValue < -aLimit ? -aLimit : (aValue > aLimit ? aLimit : aValue);
            };
            const vector3_type onBox{clamp(local.x, aHalfExtents.x),
                clamp(local.y, aHalfExtents.y), clamp(local.z, aHalfExtents.z)};

            nearest = std::min(nearest, (local - onBox).length());
        }
        return aRadius - nearest;
    }

    [[nodiscard]] floating_point_type slide_penetration(
        const quaternion_type &aBoxRotation, const vector3_type &aStart,
        const vector3_type &aVelocity) {
        fixture f;

        const vector3_type halfExtents{0.5f, 0.5f, 0.5f};
        const vector3_type boxCentre = vector3_type::zero;

        matrix4x4_type boxTransform;
        boxTransform.set_rotation(aBoxRotation);
        const auto pBox = f.scene->make_static_obb_collider(boxTransform, halfExtents);

        const auto pCapsule = f.scene->make_capsule_collider(
            response_handlers::slide_projecting());
        pCapsule->set_position(aStart);

        auto worst = std::numeric_limits<floating_point_type>::lowest();
        for (int frame = 0; frame < 240; ++frame) {
            pCapsule->add_velocity(aVelocity);
            f.scene->update(1.0f / 60.0f);

            worst = std::max(worst, penetration_into_box(pCapsule->transform().translation(),
                pCapsule->rotation(), 0.5f, 0.5f, boxCentre, aBoxRotation, halfExtents));
        }

        REQUIRE(pBox->half_extents().x == Approx(0.5f));   
        return worst;
    }
}

TEST_CASE("gdk::scene a capsule sliding on an unrotated box stays outside it", "[gdk::collision]")
{
    REQUIRE(slide_penetration(quaternion_type::identity, {-3, 0, 0}, {3, 0, 0}) < 1e-3f);
}

TEST_CASE("gdk::scene a capsule sliding on a rotated obb stays outside it", "[gdk::collision]")
{
    quaternion_type rotation;
    rotation.set_from_euler({0.3f, 0.6f, 0.4f});   

    REQUIRE(slide_penetration(rotation, {-3, 0, 0}, {3, 0, 0}) < 1e-3f);
}

TEST_CASE("gdk::scene a capsule slid shallowly across a rotated obb stays outside it",
    "[gdk::collision]")
{
    quaternion_type rotation;
    rotation.set_from_euler({0.3f, 0.6f, 0.4f});

    REQUIRE(slide_penetration(rotation, {-2, 1.2f, 0}, {3, -0.4f, 0}) < 1e-3f);
}

namespace {
    [[nodiscard]] std::vector<floating_point_type> obb_slide_gaps(
        const quaternion_type &aFrame, const vector3_type &aStart,
        const vector3_type &aVelocity, const int aFrames) {
        fixture f;

        const vector3_type halfExtents{0.5f, 0.5f, 0.5f};
        matrix4x4_type boxTransform;
        boxTransform.set_rotation(aFrame);
        const auto pBox = f.scene->make_static_obb_collider(boxTransform, halfExtents);

        const auto pCapsule = f.scene->make_capsule_collider(
            response_handlers::slide_projecting());
        pCapsule->set_position(turn_by(aFrame, aStart));
        pCapsule->set_rotation(aFrame);

        std::vector<floating_point_type> gaps;
        for (int frame = 0; frame < aFrames; ++frame) {
            pCapsule->add_velocity(turn_by(aFrame, aVelocity));
            f.scene->update(1.0f / 60.0f);

            gaps.push_back(-penetration_into_box(pCapsule->transform().translation(),
                pCapsule->rotation(), 0.5f, 0.5f, vector3_type::zero, aFrame, halfExtents));
        }

        REQUIRE(pBox->half_extents().x == Approx(0.5f));   
        return gaps;
    }
}

TEST_CASE("gdk::scene slides a capsule along an obb the same way in any frame",
    "[gdk::collision]")
{
    quaternion_type rotated;
    rotated.set_from_euler({0.3f, 0.6f, 0.4f});

    const auto plain = obb_slide_gaps(quaternion_type::identity, {-2, 0.9f, 0}, {3, -0.6f, 0}, 240);
    const auto spun = obb_slide_gaps(rotated, {-2, 0.9f, 0}, {3, -0.6f, 0}, 240);

    REQUIRE(plain.size() == spun.size());

    auto worst = floating_point_type{0};
    for (std::size_t i = 0; i < plain.size(); ++i)
        worst = std::max(worst, std::abs(plain[i] - spun[i]));

    REQUIRE(worst < 0.01f);
}

namespace {
    struct held_slide_extremes final {
        floating_point_type deepest_penetration;
        floating_point_type furthest_separation;
    };

    [[nodiscard]] held_slide_extremes held_slide(const quaternion_type &aBoxRotation,
        const vector3_type &aStartDirection, const vector3_type &aOrbitAxis,
        const floating_point_type aPressRadians, const int aFrames) {
        fixture f;

        const vector3_type halfExtents{0.5f, 0.5f, 0.5f};
        matrix4x4_type boxTransform;
        boxTransform.set_rotation(aBoxRotation);
        const auto pBox = f.scene->make_static_obb_collider(boxTransform, halfExtents);

        const auto pCapsule = f.scene->make_capsule_collider(
            response_handlers::slide_projecting());
        pCapsule->set_position(aStartDirection * 1.3f);

        const auto cross = [](const vector3_type &a, const vector3_type &b) {
            return vector3_type{
                a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        };

        held_slide_extremes extremes{0, 0};
        bool contacted = false;
        const auto press = std::sin(aPressRadians);
        const auto along = std::cos(aPressRadians);

        for (int frame = 0; frame < aFrames; ++frame) {
            auto toCentre = pCapsule->transform().translation() * -1.0f;
            const auto distance = toCentre.length();
            if (distance > 1e-4f) toCentre = toCentre * (1.0f / distance);

            auto tangent = cross(toCentre, aOrbitAxis);
            const auto tangentLength = tangent.length();
            if (tangentLength > 1e-4f) tangent = tangent * (1.0f / tangentLength);

            pCapsule->add_velocity((toCentre * press + tangent * along) * 10.0f);   
            f.scene->update(1.0f / 60.0f);

            const auto gap = -penetration_into_box(pCapsule->transform().translation(),
                pCapsule->rotation(), 0.5f, 0.5f, vector3_type::zero, aBoxRotation,
                halfExtents);

            if (gap < 0.05f) contacted = true;
            if (contacted) {
                extremes.deepest_penetration = std::min(extremes.deepest_penetration, gap);
                extremes.furthest_separation = std::max(extremes.furthest_separation, gap);
            }
        }

        REQUIRE(pBox->half_extents().x == Approx(0.5f));   
        return extremes;
    }
}

TEST_CASE("gdk::scene a capsule held against a rotated obb never sinks into it",
    "[gdk::collision]")
{
    quaternion_type rotation;
    rotation.set_from_euler({0.3f, 0.6f, 0.4f});   

    const vector3_type starts[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const vector3_type axes[] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}};

    for (const auto &start : starts) for (const auto &axis : axes) {
        if (std::abs(start.dot_product(axis)) > 0.99f) continue;   
        for (const auto degrees : {5.0f, 15.0f, 30.0f, 60.0f}) {
            const auto extremes = held_slide(rotation, start, axis,
                degrees * 3.14159265f / 180.0f, 240);

            REQUIRE(extremes.deepest_penetration > -1e-3f);
        }
    }
}

TEST_CASE("gdk::scene a capsule held against an obb behaves the same at any rotation",
    "[gdk::collision]")
{
    const vector3_type start{0.577f, 0.577f, 0.577f};
    const vector3_type axis{0, 1, 0};
    constexpr auto PRESS = 30.0f * 3.14159265f / 180.0f;

    const auto control = held_slide(quaternion_type::identity, start, axis, PRESS, 240);
    REQUIRE(control.deepest_penetration > -1e-3f);

    const vector3_type eulers[] = {
        {0.05f, 0, 0},              
        {0.2f, 0.1f, 0},
        {0.3f, 0.6f, 0.4f},         
        {0, 0, 0.7853982f},         
        {1.1f, 0.7f, 0.3f},
        {0.9f, 1.3f, 2.1f}};

    for (const auto &euler : eulers) {
        quaternion_type rotation;
        rotation.set_from_euler(euler);

        const auto extremes = held_slide(rotation, start, axis, PRESS, 240);

        REQUIRE(extremes.deepest_penetration > -1e-3f);
        REQUIRE(extremes.furthest_separation < control.furthest_separation + 0.25f);
    }
}

TEST_CASE("gdk::collider transform() and rotation() describe the same box", "[gdk::collision]")
{
    quaternion_type rotation;
    rotation.set_from_euler({0.3f, 0.6f, 0.4f});   

    matrix4x4_type transform;
    transform.set_translation({-7, 0, 0});
    transform.set_rotation(rotation);

    fixture f;
    const vector3_type halfExtents{0.9f, 0.3f, 0.5f};
    const auto pBox = f.scene->make_static_obb_collider(transform, halfExtents);

    const auto position = pBox->transform().translation();
    const auto matrix = pBox->transform();

    for (int corner = 0; corner < 8; ++corner) {
        const vector3_type local{
            (corner & 1) ? halfExtents.x : -halfExtents.x,
            (corner & 2) ? halfExtents.y : -halfExtents.y,
            (corner & 4) ? halfExtents.z : -halfExtents.z};

        const auto tested = turn_by(pBox->rotation(), local) + position;

        const vector3_type drawn{
            matrix.get(0, 0) * local.x + matrix.get(1, 0) * local.y + matrix.get(2, 0) * local.z + matrix.get(3, 0),
            matrix.get(0, 1) * local.x + matrix.get(1, 1) * local.y + matrix.get(2, 1) * local.z + matrix.get(3, 1),
            matrix.get(0, 2) * local.x + matrix.get(1, 2) * local.y + matrix.get(2, 2) * local.z + matrix.get(3, 2)};

        REQUIRE((drawn - tested).length() < 1e-5f);
    }

    const auto recovered = pBox->rotation();
    REQUIRE(recovered.w == Approx(rotation.w).margin(1e-5f));
    REQUIRE(recovered.x == Approx(rotation.x).margin(1e-5f));
    REQUIRE(recovered.y == Approx(rotation.y).margin(1e-5f));
    REQUIRE(recovered.z == Approx(rotation.z).margin(1e-5f));
}

TEST_CASE("gdk::scene a response handler is given the scene's own tolerances",
    "[gdk::collision]")
{
    fixture f;

    const impl_collision_policy policy;   

    struct { bool called; floating_point_type minSweptSpeed;
        floating_point_type normalizationThreshold;
        std::size_t clipPlaneCount; vector3_type lastPlane;
        vector3_type normal; delta_time_type remainingTime; } seen{};

    const auto pGround = f.scene->make_static_plane_collider(matrix4x4_type());

    const auto pBall = f.scene->make_sphere_collider(
        [&seen](collider &, const contact_context &aContact) {
            seen.called = true;
            seen.minSweptSpeed = aContact.min_swept_speed;
            seen.normalizationThreshold = aContact.normalization_threshold;
            seen.clipPlaneCount = aContact.clip_planes.size();
            if (!aContact.clip_planes.empty()) seen.lastPlane = aContact.clip_planes.back();
            seen.normal = aContact.normal;
            seen.remainingTime = aContact.remaining_time;
            return vector3_type::zero;
        });
    pBall->set_position({0, 1.0f, 0});

    for (int frame = 0; frame < 30 && !seen.called; ++frame) {
        pBall->add_velocity({0, -3, 0});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(seen.called);                                    
    REQUIRE(seen.minSweptSpeed == Approx(policy.MIN_SWEPT_SPEED));
    REQUIRE(seen.normalizationThreshold == Approx(policy.NORMALIZATION_THRESHOLD));
    REQUIRE(seen.clipPlaneCount >= 1);
    REQUIRE(seen.lastPlane.x == Approx(seen.normal.x).margin(1e-5f));
    REQUIRE(seen.lastPlane.y == Approx(seen.normal.y).margin(1e-5f));
    REQUIRE(seen.lastPlane.z == Approx(seen.normal.z).margin(1e-5f));
    REQUIRE(seen.normal.y == Approx(1.0f).margin(1e-4f));
    REQUIRE(seen.remainingTime >= 0.0f);
    REQUIRE(seen.remainingTime <= Approx(1.0f / 60.0f).margin(1e-6f));
}

TEST_CASE("gdk::scene reports the contacts of the last step", "[gdk::collision]")
{
    fixture f;

    matrix4x4_type ground;
    ground.set_translation({0, -1, 0});
    const auto pGround = f.scene->make_static_plane_collider(ground);

    const auto pBall = f.scene->make_sphere_collider();
    pBall->set_position({0, 1.5f, 0});

    SECTION("nothing has touched before the first update")
    {
        REQUIRE(f.scene->contacts().empty());
    }

    SECTION("a landed body reports its contact, with geometry")
    {
        for (int frame = 0; frame < 60; ++frame) {
            pBall->add_velocity({0, -3, 0});
            f.update(1.0f / 60.0f);
        }

        const auto contacts = f.scene->contacts();
        REQUIRE(contacts.size() == 1);

        const auto &landed = contacts.front();

        const auto touchesBall = landed.a.get() == pBall.get() || landed.b.get() == pBall.get();
        const auto touchesGround = landed.a.get() == pGround.get() || landed.b.get() == pGround.get();
        REQUIRE(touchesBall);
        REQUIRE(touchesGround);

        REQUIRE(landed.point.y == Approx(-1.0f).margin(0.05f));

        const auto towardBall = landed.a.get() == pBall.get() ? landed.normal : landed.normal * -1.0f;
        REQUIRE(towardBall.y == Approx(1.0f).margin(1e-3f));
        REQUIRE(std::abs(towardBall.x) < 1e-3f);
        REQUIRE(std::abs(towardBall.z) < 1e-3f);

        REQUIRE(landed.penetration < 0.01f);
    }

    SECTION("contacts_for filters to one collider, and is a subset of contacts()")
    {
        const auto pSecond = f.scene->make_sphere_collider();
        pSecond->set_position({5, 1.5f, 0});

        for (int frame = 0; frame < 60; ++frame) {
            pBall->add_velocity({0, -3, 0});
            pSecond->add_velocity({0, -3, 0});
            f.update(1.0f / 60.0f);
        }

        REQUIRE(f.scene->contacts().size() == 2);          

        const auto forBall = f.scene->contacts_for(*pBall);
        REQUIRE(forBall.size() == 1);

        const auto involvesBall = forBall.front().a.get() == pBall.get()
            || forBall.front().b.get() == pBall.get();
        REQUIRE(involvesBall);
    }

    SECTION("the query is a read-only view of the last step, not a fresh detection pass")
    {
        for (int frame = 0; frame < 60; ++frame) {
            pBall->add_velocity({0, -3, 0});
            f.update(1.0f / 60.0f);
        }
        REQUIRE(f.scene->contacts().size() == 1);

        pBall->set_position({100, 100, 100});
        REQUIRE(f.scene->contacts().size() == 1);

        f.update(1.0f / 60.0f);
        REQUIRE(f.scene->contacts().empty());
    }

    REQUIRE(pGround->transform().translation().y == Approx(-1.0f));   
}

TEST_CASE("gdk::scene contact reporting is stable and outlives nothing", "[gdk::collision]")
{
    fixture f;

    matrix4x4_type ground;
    ground.set_translation({0, -1, 0});
    const auto pGround = f.scene->make_static_plane_collider(ground);

    std::vector<sphere_collider_ptr_type> balls;
    for (int i = 0; i < 5; ++i) {
        auto pBall = f.scene->make_sphere_collider();
        pBall->set_position({i * 3.0f, 1.5f, 0});
        balls.push_back(pBall);
    }

    for (int frame = 0; frame < 60; ++frame) {
        for (auto &pBall : balls) pBall->add_velocity({0, -3, 0});
        f.update(1.0f / 60.0f);
    }

    const auto first = f.scene->contacts();
    REQUIRE(first.size() == 5);

    const auto second = f.scene->contacts();
    REQUIRE(second.size() == first.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        REQUIRE(second[i].a.get() == first[i].a.get());
        REQUIRE(second[i].b.get() == first[i].b.get());
    }

    f.collisions.clear();
    f.triggers.clear();
}

TEST_CASE("gdk::scene holds no strong reference to a collider between steps",
    "[gdk::collision]")
{
    fixture f;

    matrix4x4_type ground;
    ground.set_translation({0, -1, 0});
    const auto pGround = f.scene->make_static_plane_collider(ground);

    std::weak_ptr<const collider> watch;
    {
        auto pBall = f.scene->make_sphere_collider();
        pBall->set_position({0, 1.5f, 0});
        watch = pBall;

        for (int frame = 0; frame < 60; ++frame) {
            pBall->add_velocity({0, -3, 0});
            f.update(1.0f / 60.0f);
        }

        REQUIRE(f.scene->contacts().size() == 1);

        f.collisions.clear();
        f.triggers.clear();

        REQUIRE(!watch.expired());   
    }

    REQUIRE(watch.expired());

    f.update(1.0f / 60.0f);
    REQUIRE(f.scene->contacts().empty());
    REQUIRE(pGround->transform().translation().y == Approx(-1.0f));   
}

TEST_CASE("gdk::scene a destroyed collider's contact is dropped, not dangled",
    "[gdk::collision]")
{
    fixture f;

    matrix4x4_type ground;
    ground.set_translation({0, -1, 0});
    const auto pGround = f.scene->make_static_plane_collider(ground);

    auto pKeep = f.scene->make_sphere_collider();
    pKeep->set_position({0, 1.5f, 0});

    auto pGoing = f.scene->make_sphere_collider();
    pGoing->set_position({4, 1.5f, 0});

    for (int frame = 0; frame < 60; ++frame) {
        pKeep->add_velocity({0, -3, 0});
        pGoing->add_velocity({0, -3, 0});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(f.scene->contacts().size() == 2);

    f.collisions.clear();
    f.triggers.clear();

    const auto *pDestroyed = pGoing.get();
    pGoing.reset();

    const auto remaining = f.scene->contacts();
    REQUIRE(remaining.size() == 1);
    REQUIRE(remaining.front().a.get() != pDestroyed);
    REQUIRE(remaining.front().b.get() != pDestroyed);

    const auto involvesKeep = remaining.front().a.get() == pKeep.get()
        || remaining.front().b.get() == pKeep.get();
    REQUIRE(involvesKeep);
    REQUIRE(pGround->transform().translation().y == Approx(-1.0f));
}

TEST_CASE("gdk::scene releases colliders held by a chunk that stops running",
    "[gdk::collision]")
{
    const auto dispatcher = [](std::size_t aCount, const chunk_type &aChunk) {
        for (std::size_t i = 0; i < aCount; ++i) aChunk(i);
    };

    const auto scene = impl_collision_scene::make([](collision_event) {}, [](trigger_event) {},
        impl_collision_policy{}, dispatcher);

    matrix4x4_type ground;
    ground.set_translation({0, -1, 0});
    const auto pGround = scene->make_static_plane_collider(ground);

    constexpr int COUNT = 200;

    std::vector<sphere_collider_ptr_type> balls;
    std::vector<std::weak_ptr<const collider>> watches;
    for (int i = 0; i < COUNT; ++i) {
        auto pBall = scene->make_sphere_collider();
        pBall->set_position({(i % 20) * 0.9f, 1.5f + (i / 20) * 0.9f, 0});
        watches.push_back(pBall);
        balls.push_back(std::move(pBall));
    }

    for (int frame = 0; frame < 20; ++frame) {
        for (auto &pBall : balls) pBall->add_velocity({0, -3, 0});
        scene->update(1.0f / 60.0f);
    }
    scene->process_events();

    balls.clear();

    scene->update(1.0f / 60.0f);
    scene->process_events();

    std::size_t stillAlive = 0;
    for (const auto &watch : watches) if (!watch.expired()) ++stillAlive;

    REQUIRE(stillAlive == 0);
    REQUIRE(pGround->transform().translation().y == Approx(-1.0f));   
}

namespace {
    vector3_type slide(const vector3_type &aVelocity,
        const contact_context &aContact, const bool aPreserveSpeed) {
        const auto &aCollisionNormal = aContact.normal;
        const auto &aClipPlanes = aContact.clip_planes;
        const auto slideThreshold = aContact.min_swept_speed;

        const auto &velocity = aVelocity;
        const auto vn = velocity.dot_product(aCollisionNormal);

        if (vn >= -slideThreshold) return velocity;

        auto slideVelocity = velocity - aCollisionNormal * vn;

        for (std::size_t first = 0; first + 1 < aClipPlanes.size(); ++first) {
            const auto &plane = aClipPlanes[first];

            if (slideVelocity.dot_product(plane) >= 0) continue;

            const auto facing = plane.dot_product(aCollisionNormal);

            if (facing <= -aContact.clip_plane_parallel_cosine) return vector3_type::zero;

            if (facing >= aContact.clip_plane_parallel_cosine) continue;

            auto along = plane.cross_product(aCollisionNormal);
            const auto length = along.length();

            along = along * (1.0f / length);
            slideVelocity = along * slideVelocity.dot_product(along);

            for (std::size_t other = 0; other + 1 < aClipPlanes.size(); ++other)
                if (other != first && slideVelocity.dot_product(aClipPlanes[other]) < 0)
                    return vector3_type::zero;

            break;
        }

        if (!aPreserveSpeed) return slideVelocity;

        return slideVelocity.normal() * velocity.length();
    }

    [[nodiscard]] vector3_type ask_slide(const collider &aSubject,
        const vector3_type &aVelocity, const vector3_type &aContactNormal,
        const clip_planes_type &aClipPlanes) {
        const impl_collision_policy policy;
        const overlap result;

        const contact_context context{aSubject, result, aContactNormal, 1.0f / 60.0f, aClipPlanes, 0,
            policy.MIN_SWEPT_SPEED, policy.NORMALIZATION_THRESHOLD, policy.CLIP_PLANE_PARALLEL_COSINE};

        return slide(aVelocity, context, true);
    }
}

TEST_CASE("gdk::collisions::response_handlers collide_and_slide tells a vice from a flat surface",
    "[gdk::collision]")
{
    fixture f;
    const auto pSubject = f.scene->make_sphere_collider();

    const vector3_type contactNormal{0, 1, 0};

    constexpr float TILT = 5e-4f;

    SECTION("two surfaces facing the same way are one surface, and it slides")
    {
        pSubject->add_velocity({0, -1, -1});
        const auto before = pSubject->velocity();

        vector3_type nearlySame{0, 1, TILT};
        nearlySame = nearlySame.normal();

        const auto after = ask_slide(*pSubject, before, contactNormal, {nearlySame, contactNormal});

        REQUIRE(after.length() > 1e-4f);
        REQUIRE(after.y == Approx(0.0f).margin(1e-3f));
        REQUIRE(after.length() == Approx(before.length()).margin(1e-3f));
    }

    SECTION("two surfaces facing each other are a vice, and it stops")
    {
        pSubject->set_velocity({0, -1, -1});
        const auto before = pSubject->velocity();

        vector3_type opposed{0, -1, TILT};
        opposed = opposed.normal();

        const auto after = ask_slide(*pSubject, before, contactNormal, {opposed, contactNormal});

        REQUIRE(after.length() < 1e-4f);
    }
}

TEST_CASE("gdk::scene the stock response handlers are the two velocity-ownership pairings",
    "[gdk::collision]")
{
    const auto drive = [](const response_handler &aHandler) {
        fixture f;
        const auto pWall = add_static_box_at_origin(f);

        const auto pSphere = f.scene->make_sphere_collider(aHandler);
        pSphere->set_position({-1.2f, 0, 0});

        for (int frame = 0; frame < 30; ++frame) {
            pSphere->add_velocity({1.0f, 0.25f, 0});
            f.update(1.0f / 60.0f);
        }

        REQUIRE(pWall->half_extents().x == Approx(0.5f));   
        return pSphere->transform().translation();
    };

    SECTION("the three give three different answers to the same contact")
    {
        const auto stopped = drive(response_handlers::null_opt);
        const auto projected = drive(response_handlers::slide_projecting());
        const auto preserved = drive(response_handlers::slide_preserving_speed());

        REQUIRE(stopped.x <= Approx(-1.0f).margin(1e-2f));
        REQUIRE(projected.x <= Approx(-1.0f).margin(1e-2f));
        REQUIRE(preserved.x <= Approx(-1.0f).margin(1e-2f));
        REQUIRE(projected.y > stopped.y + 0.02f);
        REQUIRE(preserved.y > projected.y * 2);
    }

    SECTION("the factories default to the speed-preserving slide")
    {
        const auto defaulted = drive(response_handlers::slide_preserving_speed());

        fixture f;
        const auto pWall = add_static_box_at_origin(f);
        const auto pSphere = f.scene->make_sphere_collider();   
        pSphere->set_position({-1.2f, 0, 0});

        for (int frame = 0; frame < 30; ++frame) {
            pSphere->add_velocity({1.0f, 0.25f, 0});
            f.update(1.0f / 60.0f);
        }

        REQUIRE(pSphere->transform().translation().y == Approx(defaulted.y).margin(1e-4f));
        REQUIRE(pWall->half_extents().x == Approx(0.5f));
    }
}

TEST_CASE("gdk::scene a handler can express behaviour the stock ones do not",
    "[gdk::collision]")
{
    const auto drive = [](const response_handler &aHandler) {
        fixture f;
        const auto pWall = add_static_box_at_origin(f);

        const auto pSphere = f.scene->make_sphere_collider(aHandler);
        pSphere->set_position({-1.2f, 0, 0});

        auto mostLeftward = floating_point_type{0};

        for (int frame = 0; frame < 20; ++frame) {
            pSphere->add_velocity({1.0f, 0, 0});
            f.update(1.0f / 60.0f);
            mostLeftward = std::min(mostLeftward, pSphere->resolved_velocity().x);
        }

        REQUIRE(pWall->half_extents().x == Approx(0.5f));   
        return mostLeftward;
    };

    SECTION("a bounce, which none of the three modes can do")
    {
        const response_handler bounce = [](collider &aThis, const contact_context &aContact) {
            const auto velocity = aThis.velocity();
            const auto into = velocity.dot_product(aContact.normal);
            if (into >= 0) return vector3_type::zero;

            const auto reflected = velocity - aContact.normal * (2.0f * into);
            return reflected - velocity;   
        };

        const auto slid = drive(response_handlers::slide_preserving_speed());
        const auto bounced = drive(bounce);

        REQUIRE(slid == Approx(0.0f).margin(1e-3f));
        REQUIRE(bounced == Approx(-1.0f).margin(1e-2f));
    }

    SECTION("a handler can build on the library's slide instead of reimplementing it")
    {
        const response_handler halfSpeedSlide = [](collider &aThis,
            const contact_context &aContact) {
            const auto velocity = aThis.velocity();
            const auto slid = ::slide(velocity, aContact, true);

            return slid * 0.5f - velocity;   
        };

        fixture f;
        const auto pWall = add_static_box_at_origin(f);
        const auto pSphere = f.scene->make_sphere_collider(halfSpeedSlide);
        pSphere->set_position({-1.2f, 0, 0});

        for (int frame = 0; frame < 20; ++frame) {
            pSphere->add_velocity({1.0f, 0.5f, 0});
            f.update(1.0f / 60.0f);
        }

        const auto position = pSphere->transform().translation();

        REQUIRE(position.x <= Approx(-1.0f).margin(1e-2f));   
        REQUIRE(position.y > 0.02f);                          
        REQUIRE(pWall->half_extents().x == Approx(0.5f));
    }
}

namespace {
    [[nodiscard]] response_handler ride_platforms() {
        return [](collider &aThis, const contact_context &aContact) {
            const auto velocity = aThis.velocity();
            auto delta = ::slide(velocity, aContact, false) - velocity;

            if (aContact.normal.y > 0.5f) {
                const auto arm = aContact.result.contact_point
                    - aContact.other.transform().translation();
                const auto omega = aContact.other.angular_velocity();

                const vector3_type spin{
                    omega.y * arm.z - omega.z * arm.y,
                    omega.z * arm.x - omega.x * arm.z,
                    omega.x * arm.y - omega.y * arm.x};

                delta += aContact.other.velocity() + spin;
            }

            return delta;
        };
    }
}

TEST_CASE("gdk::collider angular velocity is per-frame input, like velocity", "[gdk::collision]")
{
    fixture f;
    const auto pBody = f.scene->make_obb_collider();
    pBody->set_half_extents({1.0f, 0.25f, 0.25f});

    SECTION("the scene integrates the orientation from it and then clears it")
    {
        constexpr auto QUARTER = 1.5707963f;
        pBody->set_angular_velocity({0, QUARTER, 0});

        f.update(1.0f / 60.0f);

        REQUIRE(pBody->angular_velocity().length() == Approx(0.0f).margin(1e-6f));

        const auto turned = pBody->rotation();
        REQUIRE(turned.y == Approx(std::sin(QUARTER / 60.0f * 0.5f)).margin(1e-5f));
    }

    SECTION("a held rate integrates to the angle it should over a second")
    {
        constexpr auto QUARTER = 1.5707963f;

        for (int frame = 0; frame < 60; ++frame) {
            pBody->add_angular_velocity({0, QUARTER, 0});
            f.update(1.0f / 60.0f);
        }

        const auto turnedX = turn_by(pBody->rotation(), vector3_type{1, 0, 0});
        REQUIRE(turnedX.x == Approx(0.0f).margin(2e-2f));
        REQUIRE(turnedX.z == Approx(-1.0f).margin(2e-2f));
    }

    SECTION("add accumulates within a frame, as add_velocity does")
    {
        pBody->add_angular_velocity({0, 1.0f, 0});
        pBody->add_angular_velocity({0, 2.0f, 0});
        REQUIRE(pBody->angular_velocity().y == Approx(3.0f));
    }
}

TEST_CASE("gdk::scene a handler can ride a moving platform", "[gdk::collision]")
{
    SECTION("a translating platform, which worked before angular velocity existed")
    {
        fixture f;
        const auto pPlatform = f.scene->make_axis_aligned_box_collider();
        pPlatform->set_kinematic(true);
        pPlatform->set_position({0, 0, 0});

        const auto pRider = f.scene->make_sphere_collider(ride_platforms());
        pRider->set_position({0, 1.0f, 0});
        const auto start = pRider->transform().translation();

        for (int frame = 0; frame < 120; ++frame) {
            pPlatform->add_velocity({2.0f, 0, 0});
            pRider->add_velocity({0, -1.0f, 0});
            f.update(1.0f / 60.0f);
        }

        const auto carried = pRider->transform().translation().x - start.x;
        REQUIRE(pPlatform->transform().translation().x == Approx(4.0f).margin(1e-3f));
        REQUIRE(carried == Approx(4.0f).margin(0.05f));           
        REQUIRE(pRider->transform().translation().y == Approx(1.0f).margin(1e-2f));   
    }

    SECTION("a rotating platform, which needs the angular velocity")
    {
        fixture f;

        const auto pPlatform = f.scene->make_obb_collider();
        pPlatform->set_half_extents({4.0f, 0.25f, 1.0f});
        pPlatform->set_kinematic(true);
        pPlatform->set_position({0, 0, 0});

        const auto pRider = f.scene->make_sphere_collider(ride_platforms());
        pRider->set_position({3.0f, 0.75f, 0});
        const auto start = pRider->transform().translation();

        constexpr auto RATE = 1.0f;   
        for (int frame = 0; frame < 60; ++frame) {
            pPlatform->add_angular_velocity({0, RATE, 0});
            pRider->add_velocity({0, -1.0f, 0});
            f.update(1.0f / 60.0f);
        }

        const auto end = pRider->transform().translation();

        REQUIRE(std::abs(end.z) > 1.5f);                       
        REQUIRE(end.x < start.x - 0.8f);                       
        REQUIRE(end.y == Approx(0.75f).margin(0.2f));          

        const auto radius = std::sqrt(end.x * end.x + end.z * end.z);
        REQUIRE(radius == Approx(3.0f).margin(0.6f));
    }
}

TEST_CASE("gdk::collider set_rotation normalises what it is given", "[gdk::collision]")
{
    fixture f;
    const auto pBody = f.scene->make_obb_collider();
    pBody->set_half_extents({1.0f, 0.5f, 0.25f});

    quaternion_type wanted;
    wanted.set_from_euler({0.3f, 0.6f, 0.4f});

    SECTION("a scaled quaternion is stored as the rotation it represents, at unit length")
    {
        quaternion_type overlong;
        overlong.w = wanted.w * 3.0f;
        overlong.x = wanted.x * 3.0f;
        overlong.y = wanted.y * 3.0f;
        overlong.z = wanted.z * 3.0f;

        pBody->set_rotation(overlong);

        const auto stored = pBody->rotation();
        const auto length = std::sqrt(stored.w * stored.w + stored.x * stored.x
            + stored.y * stored.y + stored.z * stored.z);

        REQUIRE(length == Approx(1.0f).margin(1e-5f));
        REQUIRE(stored.w == Approx(wanted.w).margin(1e-5f));
        REQUIRE(stored.x == Approx(wanted.x).margin(1e-5f));
        REQUIRE(stored.y == Approx(wanted.y).margin(1e-5f));
        REQUIRE(stored.z == Approx(wanted.z).margin(1e-5f));

        const auto scale = pBody->transform().scale();
        REQUIRE(scale.x == Approx(1.0f).margin(1e-4f));
        REQUIRE(scale.y == Approx(1.0f).margin(1e-4f));
        REQUIRE(scale.z == Approx(1.0f).margin(1e-4f));
    }

    SECTION("drift accumulated by a caller integrating its own orientation is absorbed")
    {
        quaternion_type drifting = quaternion_type::identity;
        quaternion_type step;
        step.set_from_euler({0, 0.02f, 0});

        for (int frame = 0; frame < 500; ++frame) {
            drifting = step * drifting;          
            pBody->set_rotation(drifting);
        }

        const auto scale = pBody->transform().scale();
        REQUIRE(scale.x == Approx(1.0f).margin(1e-3f));
        REQUIRE(scale.y == Approx(1.0f).margin(1e-3f));
        REQUIRE(scale.z == Approx(1.0f).margin(1e-3f));
    }

    SECTION("a zero-length quaternion gives identity rather than NaN")
    {
        quaternion_type zero;
        zero.w = 0; zero.x = 0; zero.y = 0; zero.z = 0;

        pBody->set_rotation(zero);

        const auto stored = pBody->rotation();
        REQUIRE(std::isfinite(stored.w));
        REQUIRE(stored.w == Approx(1.0f).margin(1e-5f));

        const auto scale = pBody->transform().scale();
        REQUIRE(std::isfinite(scale.x));
        REQUIRE(scale.x == Approx(1.0f).margin(1e-4f));
    }
}

TEST_CASE("gdk::scene raycast against a turned oriented box", "[gdk::collision]")
{
    fixture f;

    const float turn = 0.349066f;

    matrix4x4_type placed;
    placed.set_rotation(quaternion_type({0, 0, turn}));
    placed.set_translation({0, -5, 0});

    const auto pBox = f.scene->make_static_obb_collider(placed, {2.0f, 0.5f, 2.0f});

    const auto hit = f.scene->raycast({0, -2, 0}, {0, -1, 0}, 10.0f);

    REQUIRE(hit.has_value());
    require_vector_near(hit->normal, {-std::sin(turn), std::cos(turn), 0}, 1e-2f);
}

TEST_CASE("gdk::scene raycast finds a flat mesh floor at every point, far from the origin", "[gdk::collision]")
{
    fixture f;

    std::vector<vector3_type> vertices;
    std::vector<std::uint32_t> indices;

    for (int x = 8; x < 20; ++x)
        for (int z = 12; z < 20; ++z) {
            const auto base = static_cast<std::uint32_t>(vertices.size());
            const auto fx = static_cast<floating_point_type>(x), fz = static_cast<floating_point_type>(z);

            vertices.insert(vertices.end(), {{fx, 4, fz}, {fx, 4, fz + 1}, {fx + 1, 4, fz + 1}, {fx + 1, 4, fz}});
            indices.insert(indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
        }

    matrix4x4_type identity;
    const auto pFloor = f.scene->make_static_mesh_collider(identity, impl_mesh_data::make(std::move(vertices), std::move(indices)));

    f.update(1.0f / 30);

    int missed = 0, looked = 0;

    for (floating_point_type x = 8.5f; x < 19.5f; x += 0.013f)
        for (floating_point_type z = 12.5f; z < 19.5f; z += 0.29f) {
            ++looked;

            const auto hit = f.scene->raycast_static({x, 4.9f, z}, {0, -1, 0}, 1.8f);

            if (!hit || std::abs(hit->point.y - 4) > 1e-3f)
                if (++missed <= 5) UNSCOPED_INFO("missed at (" << x << ", " << z << ")");
        }

    INFO(missed << " of " << looked << " rays missed");

    REQUIRE(looked > 10000);
    REQUIRE(missed == 0);

    for (const auto &[x, z] : {std::pair{0x1.8e70e6p+3f, 0x1.0cf17ep+4f}, std::pair{0x1.af2956p+3f, 0x1.0ca3eep+4f},
            std::pair{0x1.b1263ep+3f, 0x1.08d5e6p+4f}}) {
        INFO("at (" << x << ", " << z << ")");

        REQUIRE(f.scene->raycast_static({x, 0x1.39999ap+2f, z}, {0, -1, 0}, 1.8f).has_value());
    }
}

TEST_CASE("gdk::collider is on the default layer, and collides with every layer, until told otherwise",
    "[gdk::collision][layers]")
{
    fixture f;
    const auto pSphere = f.scene->make_sphere_collider();

    REQUIRE(pSphere->collision_layers() == DEFAULT_COLLISION_LAYER);
    REQUIRE(pSphere->collides_with() == ALL_COLLISION_LAYERS);

    pSphere->set_collision_layers(4, 1 | 2);

    REQUIRE(pSphere->collision_layers() == 4u);
    REQUIRE(pSphere->collides_with() == 3u);
}

TEST_CASE("gdk::scene lets two colliders whose layers do not meet pass through each other, saying nothing",
    "[gdk::collision][layers]")
{
    fixture f;
    const auto pA = f.scene->make_sphere_collider();
    const auto pB = f.scene->make_sphere_collider();
    pA->set_position({-2, 0, 0});
    pB->set_position({+2, 0, 0});

    SECTION("**one that does not collide with the other's layer** is enough: a meeting needs both")
    {
        pA->set_collision_layers(2, 2);
    }

    SECTION("**and the other way about**")
    {
        pB->set_collision_layers(DEFAULT_COLLISION_LAYER, 2);
    }

    pA->add_velocity({+1, 0, 0});
    pB->add_velocity({-1, 0, 0});
    f.update(2.0f);

    REQUIRE(pA->transform().translation().x == Approx(0.0f).margin(1e-2f));
    REQUIRE(pB->transform().translation().x == Approx(0.0f).margin(1e-2f));
    REQUIRE(f.collisions.empty());
}

TEST_CASE("gdk::scene stops a body that meets only the world on the world, and lets it through everything else",
    "[gdk::collision][layers]")
{
    constexpr collision_layers_type CORPSES = 2, LIVING = 4;

    fixture f;
    const auto pGround = f.scene->make_static_plane_collider(matrix4x4_type::identity);

    const auto pCorpse = f.scene->make_sphere_collider();
    pCorpse->set_collision_layers(CORPSES, DEFAULT_COLLISION_LAYER);
    pCorpse->set_position({0, 0.5f, 0});

    const auto pLiving = f.scene->make_axis_aligned_box_collider();
    pLiving->set_collision_layers(LIVING, ALL_COLLISION_LAYERS);
    pLiving->set_position({-3, 0.5f, 0});

    SECTION("**walked through**: the living body crosses where the corpse lies, and neither is moved")
    {
        for (int frame = 0; frame < 6; ++frame) {
            pLiving->add_velocity({1, 0, 0});
            f.update(1.0f);
        }

        REQUIRE(pLiving->transform().translation().x == Approx(3.0f).margin(1e-2f));
        REQUIRE(pCorpse->transform().translation().x == Approx(0.0f).margin(1e-2f));
    }

    SECTION("**and held up by the ground**: pushed down into it, it stays on top")
    {
        pCorpse->add_velocity({0, -2, 0});
        f.update(1.0f);

        REQUIRE(pCorpse->transform().translation().y == Approx(0.5f).margin(1e-2f));
        REQUIRE(f.count(event_type::enter) >= 1);
    }

    static_cast<void>(pGround);
}

TEST_CASE("gdk::scene sets off a trigger only for a body whose layers meet it", "[gdk::collision][layers]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pVolume = f.scene->make_static_sphere_trigger(identity, 0.5f);
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-2, 0, 0});

    pSphere->set_collision_layers(DEFAULT_COLLISION_LAYER, 2);

    for (int frame = 0; frame < 5; ++frame) {
        pSphere->add_velocity({1, 0, 0});
        f.update(1.0f);
    }

    REQUIRE(f.triggers.empty());
}

TEST_CASE("gdk::scene raycast meets a collider whatever its layers", "[gdk::collision][layers]")
{
    fixture f;
    const auto pBox = f.scene->make_axis_aligned_box_collider();
    pBox->set_collision_layers(2, 2);

    const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider == const_collider_ptr_type(pBox));
}

TEST_CASE("gdk::scene raycast answers exactly, shape by shape", "[gdk::collision][raycast]")
{
    matrix4x4_type identity;

    SECTION("a box: the face it meets, its normal toward the ray, and from inside nought and the nearest face")
    {
        fixture f;
        const auto pKept = (f.scene->make_static_axis_aligned_box_collider(identity, {0.5f, 1.0f, 0.5f}));

        const auto hit = f.scene->raycast_static({0, 10, 0}, {0, -1, 0}, 20.0f);
        REQUIRE(hit);
        REQUIRE(hit->distance == Approx(9.0f).margin(1e-4f));
        REQUIRE(hit->normal.y == Approx(1.0f));
        REQUIRE(hit->point.y == Approx(1.0f).margin(1e-4f));

        const auto inside = f.scene->raycast_static({0.4f, 0, 0}, {-1, 0, 0}, 20.0f);
        REQUIRE(inside);
        REQUIRE(inside->distance == Approx(0.0f));
        REQUIRE(inside->normal.x == Approx(1.0f));

        REQUIRE_FALSE(f.scene->raycast_static({0.6f, 10, 0}, {0, -1, 0}, 20.0f));
        REQUIRE_FALSE(f.scene->raycast_static({0, 10, 0}, {0, -1, 0}, 8.5f));
    }

    SECTION("an oriented box, turned a quarter about y: met where its turned face is")
    {
        fixture f;
        matrix4x4_type turned;
        quaternion_type quarter;
        quarter.set_from_euler({0, 3.14159265f / 2, 0});
        turned.set_rotation(quarter);
        const auto pKept = (f.scene->make_static_obb_collider(turned, {2.0f, 0.5f, 0.5f}));

        const auto alongZ = f.scene->raycast_static({0, 0, -10}, {0, 0, 1}, 20.0f);
        REQUIRE(alongZ);
        REQUIRE(alongZ->distance == Approx(8.0f).margin(1e-3f));
        REQUIRE(alongZ->normal.z == Approx(-1.0f).margin(1e-4f));

        const auto alongX = f.scene->raycast_static({-10, 0, 0}, {1, 0, 0}, 20.0f);
        REQUIRE(alongX);
        REQUIRE(alongX->distance == Approx(9.5f).margin(1e-3f));
    }

    SECTION("a sphere, and from inside it")
    {
        fixture f;
        const auto pKept = (f.scene->make_static_sphere_collider(identity, 1.0f));

        const auto hit = f.scene->raycast_static({0, 0, -5}, {0, 0, 1}, 20.0f);
        REQUIRE(hit);
        REQUIRE(hit->distance == Approx(4.0f).margin(1e-4f));
        REQUIRE(hit->normal.z == Approx(-1.0f).margin(1e-4f));

        REQUIRE(f.scene->raycast_static({0, 0.5f, 0}, {1, 0, 0}, 20.0f)->distance == Approx(0.0f));
    }

    SECTION("a capsule: its side, and its round end from above")
    {
        fixture f;
        const auto pKept = (f.scene->make_static_capsule_collider(identity, 0.5f, 1.0f));

        const auto side = f.scene->raycast_static({-10, 0.5f, 0}, {1, 0, 0}, 20.0f);
        REQUIRE(side);
        REQUIRE(side->distance == Approx(9.5f).margin(1e-4f));
        REQUIRE(side->normal.x == Approx(-1.0f).margin(1e-4f));

        const auto top = f.scene->raycast_static({0, 10, 0}, {0, -1, 0}, 20.0f);
        REQUIRE(top);
        REQUIRE(top->point.y == Approx(1.5f).margin(1e-4f));
        REQUIRE(top->normal.y == Approx(1.0f).margin(1e-4f));
    }

    SECTION("a mesh: met from either side, and down the seam between its two triangles")
    {
        fixture f;
        const auto pKept = (f.scene->make_static_mesh_collider(identity, make_quad_mesh(5.0f)));

        const auto above = f.scene->raycast_static({1, 3, 2}, {0, -1, 0}, 20.0f);
        REQUIRE(above);
        REQUIRE(above->distance == Approx(3.0f).margin(1e-4f));
        REQUIRE(above->normal.y == Approx(1.0f).margin(1e-4f));

        const auto below = f.scene->raycast_static({1, -3, 2}, {0, 1, 0}, 20.0f);
        REQUIRE(below);
        REQUIRE(below->normal.y == Approx(-1.0f).margin(1e-4f));

        for (const float along : {-4.0f, -1.3f, 0.0f, 2.7f, 4.9f}) {
            INFO(along);

            const auto seam = f.scene->raycast_static({along, 2, along}, {0, -1, 0}, 20.0f);
            REQUIRE(seam);
            REQUIRE(seam->distance == Approx(2.0f).margin(1e-4f));
        }
    }

    SECTION("a heightfield, from above")
    {
        fixture f;
        const auto pKept = (f.scene->make_static_heightfield_collider(identity,
            make_terrain(5, 5, [](std::size_t aColumn, std::size_t) { return static_cast<floating_point_type>(aColumn) * 0.5f; })));

        const auto hit = f.scene->raycast_static({0.0f, 10, 0.5f}, {0, -1, 0}, 20.0f);
        REQUIRE(hit);
        REQUIRE(hit->point.y == Approx(1.0f).margin(1e-3f));
        REQUIRE(hit->normal.y > 0.8f);
        REQUIRE(hit->normal.x < 0.0f);
    }

    SECTION("a plane: the ground under it, so from below it is begun inside")
    {
        fixture f;
        const auto pKept = (f.scene->make_static_plane_collider(identity));

        REQUIRE(f.scene->raycast_static({0, -1, 0}, {1, 0, 0}, 20.0f)->distance == Approx(0.0f));
        REQUIRE_FALSE(f.scene->raycast_static({0, 1, 0}, {1, 0, 0}, 20.0f));
    }
}

TEST_CASE("gdk::scene overlaps_static answers for an upright cylinder, shape by shape", "[gdk::collision][overlap]")
{
    matrix4x4_type identity;

    const auto room_at = [](const floating_point_type aX, const floating_point_type aZ) {
        return upright_cylinder{{aX, 1.0f, aZ}, 1.0f, 0.5f};
    };


    SECTION("a box: in where it reaches, out past its radius, out exactly touching, in when wholly inside it")
    {
        fixture f;
        matrix4x4_type at;
        at.set_translation({0, 1.5f, 0});
        const auto pKept = f.scene->make_static_axis_aligned_box_collider(at, {0.5f, 1.5f, 0.5f});

        REQUIRE(f.scene->overlaps_static(room_at(0.9f, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(1.1f, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(1.0f, 0)));
        REQUIRE(f.scene->overlaps_static({{0, 1, 0}, 0.5f, 0.1f}));

        REQUIRE(f.scene->overlaps_static(room_at(0.8f, 0.8f)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0.9f, 0.9f)));

        REQUIRE_FALSE(f.scene->overlaps_static({{0, 3, 0}, 1.0f, 0.5f}));
        REQUIRE(f.scene->overlaps_static({{0, 2.99f, 0}, 1.0f, 0.5f}));
    }

    SECTION("an oriented box turned an eighth: its corner, now along x, reaches further")
    {
        fixture f;
        matrix4x4_type turned;
        quaternion_type eighth;
        eighth.set_from_euler({0, 3.14159265f / 4, 0});
        turned.set_rotation(eighth);
        turned.set_translation({0, 1.5f, 0});
        const auto pKept = f.scene->make_static_obb_collider(turned, {0.5f, 1.5f, 0.5f});

        REQUIRE(f.scene->overlaps_static(room_at(1.15f, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(1.25f, 0)));
        REQUIRE(f.scene->overlaps_static({{0, 1, 0}, 0.5f, 0.1f}));
    }

    SECTION("a sphere and a capsule: within the sum of the radii, and not")
    {
        fixture f;
        matrix4x4_type at;
        at.set_translation({0, 1.5f, 0});
        const auto pSphere = f.scene->make_static_sphere_collider(at, 0.5f);

        REQUIRE(f.scene->overlaps_static(room_at(0.9f, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(1.1f, 0)));

        fixture g;
        matrix4x4_type lying;
        quaternion_type quarter;
        quarter.set_from_euler({0, 0, 3.14159265f / 2});
        lying.set_rotation(quarter);
        lying.set_translation({0, 1.5f, 3});
        const auto pCapsule = g.scene->make_static_capsule_collider(lying, 0.5f, 2.0f);

        REQUIRE(g.scene->overlaps_static(room_at(1.5f, 2.1f)));
        REQUIRE_FALSE(g.scene->overlaps_static(room_at(1.5f, 1.9f)));
        REQUIRE(g.scene->overlaps_static(room_at(2.9f, 3)));
        REQUIRE_FALSE(g.scene->overlaps_static(room_at(3.1f, 3)));
    }

    SECTION("a plane: under the ground is in it, over it is not, and resting on it touches")
    {
        fixture f;
        const auto pKept = f.scene->make_static_plane_collider(identity);

        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static({{0, 0, 0}, 1.0f, 0.5f}));
        REQUIRE(f.scene->overlaps_static({{0, -0.1f, 0}, 1.0f, 0.5f}));
    }

    SECTION("a mesh: a wall seen edge on is in where its edge comes within the radius; a floor flush is not")
    {
        fixture f;
        const auto pWall = f.scene->make_static_mesh_collider(identity, impl_mesh_data::make(
            {{1, 0, -2}, {1, 0, 2}, {1, 3, 2}, {1, 3, -2}}, {0, 1, 2, 0, 2, 3}));

        REQUIRE(f.scene->overlaps_static(room_at(0.6f, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0.4f, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0.5f, 0)));
        REQUIRE(f.scene->overlaps_static(room_at(0.8f, 2.3f)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0.8f, 2.6f)));

        fixture g;
        const auto pFloor = g.scene->make_static_mesh_collider(identity, impl_mesh_data::make(
            {{-5, 1, -5}, {5, 1, -5}, {5, 1, 5}, {-5, 1, 5}}, {0, 1, 2, 0, 2, 3}));

        REQUIRE_FALSE(g.scene->overlaps_static(room_at(0, 0)));
        REQUIRE(g.scene->overlaps_static({{0, 0.9f, 0}, 1.0f, 0.5f}));
        REQUIRE(g.scene->overlaps_static({{1, 0.5f, -2}, 1.0f, 0.1f}));
    }

    SECTION("a heightfield: a hill rising into it, and the same hill moved away")
    {
        fixture f;
        std::vector<floating_point_type> heights(25, 0.0f);
        heights[12] = 2.0f;
        const auto pHill = f.scene->make_static_heightfield_collider(identity, impl_heightfield_data::make(5, 5, heights));

        REQUIRE(f.scene->overlaps_static({{0, 1.5f, 0}, 1.0f, 0.2f}));
        REQUIRE_FALSE(f.scene->overlaps_static({{0, 2.0f, 0}, 1.0f, 0.2f}));
        REQUIRE_FALSE(f.scene->overlaps_static({{1.8f, 0.5f, 1.8f}, 1.0f, 0.1f}));
    }

    SECTION("nothing there, and nothing of a volume")
    {
        fixture f;

        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0, 0)));

        matrix4x4_type at;
        at.set_translation({0, 1.5f, 0});
        const auto pKept = f.scene->make_static_axis_aligned_box_collider(at, {0.5f, 1.5f, 0.5f});

        REQUIRE_FALSE(f.scene->overlaps_static({{0, 1, 0}, 0.0f, 0.5f}));
        REQUIRE_FALSE(f.scene->overlaps_static({{0, 1, 0}, 1.0f, 0.0f}));
    }
}

TEST_CASE("gdk::scene overlaps_static catches whatever rays round the same cylinder catch", "[gdk::collision][overlap]")
{
    fixture f;
    std::vector<const_collider_ptr_type> kept;

    std::uint32_t state = 20261005u;
    const auto next = [&state]() {
        state = state * 1664525u + 1013904223u;

        return static_cast<floating_point_type>(state >> 8) / static_cast<floating_point_type>(1u << 24);
    };

    for (int i = 0; i < 40; ++i) {
        matrix4x4_type at;
        at.set_translation({next() * 20 - 10, next() * 3, next() * 20 - 10});
        kept.push_back(f.scene->make_static_axis_aligned_box_collider(at, {0.1f + next() * 0.6f, 0.1f + next() * 0.6f,
            0.1f + next() * 0.6f}));
    }

    for (int i = 0; i < 40; ++i) {
        const vector3_type corner{next() * 20 - 10, next() * 3, next() * 20 - 10};
        const auto spread = [&] { return vector3_type{next() * 2 - 1, next() * 2 - 1, next() * 2 - 1}; };
        kept.push_back(f.scene->make_static_mesh_collider(matrix4x4_type(), impl_mesh_data::make(
            {corner, corner + spread(), corner + spread()}, {0, 1, 2})));
    }

    constexpr floating_point_type RADIUS = 0.45f;
    std::size_t rayCaught = 0, volumeCaught = 0, missed = 0;

    for (int i = 0; i < 4000; ++i) {
        const vector3_type base{next() * 20 - 10, next() * 2.5f, next() * 20 - 10};

        bool rays = false;

        for (const floating_point_type up : {0.01f, 0.4f, 0.79f})
            for (int each = 0; each < 8 && !rays; ++each) {
                const auto turn = static_cast<floating_point_type>(each) * 3.14159265f / 4;

                rays = static_cast<bool>(f.scene->raycast_static({base.x, base.y + up, base.z},
                    {std::cos(turn), 0, std::sin(turn)}, RADIUS * 0.99f));
            }

        const bool volume = f.scene->overlaps_static({base, 0.8f, RADIUS});

        if (rays) ++rayCaught;
        if (volume) ++volumeCaught;
        if (rays && !volume) ++missed;
    }

    INFO("rays caught " << rayCaught << ", the volume " << volumeCaught);

    REQUIRE(rayCaught > 200);
    REQUIRE(missed == 0);
    REQUIRE(volumeCaught > rayCaught);
}

TEST_CASE("gdk::scene finds a static made outside an update at once, and one made inside it too", "[gdk::collision][raycast]")
{
    matrix4x4_type at;
    at.set_translation({5, 0.5f, 0});

    SECTION("made before any update: a ray and a volume find it straight away")
    {
        fixture f;
        const auto pKept = f.scene->make_static_axis_aligned_box_collider(at, {0.5f, 0.5f, 0.5f});

        REQUIRE(f.scene->raycast_static({5, 10, 0}, {0, -1, 0}, 20.0f));
        REQUIRE(f.scene->overlaps_static({{5, 0, 0}, 1.0f, 0.2f}));
    }

    SECTION("made by a response, during an update: found after it, and after the next")
    {
        fixture f;
        matrix4x4_type floor;
        floor.set_translation({0, -0.5f, 0});
        const auto pFloor = f.scene->make_static_axis_aligned_box_collider(floor, {10, 0.5f, 10});

        const_box_collider_ptr_type pMade;
        auto &scene = *f.scene;

        auto pBall = f.scene->make_sphere_collider([&](collider &, const contact_context &) {
            if (!pMade) pMade = scene.make_static_axis_aligned_box_collider(at, {0.5f, 0.5f, 0.5f});

            return vector3_type{};
        });
        pBall->set_position({0, 0.6f, 0});

        for (int frame = 0; frame < 10 && !pMade; ++frame) {
            pBall->add_velocity({0, -3, 0});
            f.update(1.0f / 30.0f);
        }

        REQUIRE(pMade);
        REQUIRE(f.scene->raycast_static({5, 10, 0}, {0, -1, 0}, 20.0f));
        REQUIRE(f.scene->overlaps_static({{5, 0, 0}, 1.0f, 0.2f}));

        f.update(1.0f / 30.0f);

        REQUIRE(f.scene->raycast_static({5, 10, 0}, {0, -1, 0}, 20.0f));
        REQUIRE(f.scene->overlaps_static({{5, 0, 0}, 1.0f, 0.2f}));
    }
}

TEST_CASE("gdk::scene overlaps_static answers for a box that leans, shape by shape", "[gdk::collision][overlap]")
{
    matrix4x4_type identity;

    const parallelepiped level{{0, 1, -0.5f}, {vector3_type{4, 0, 0}, vector3_type{0, 0, 1}, vector3_type{0, 1, 0}}};
    const parallelepiped rising{{0, 1, -0.5f}, {vector3_type{4, 1, 0}, vector3_type{0, 0, 1}, vector3_type{0, 1, 0}}};

    SECTION("a box: in where it stands in the way, out beside it, out exactly touching, in when wholly inside it")
    {
        fixture f;
        matrix4x4_type at;
        at.set_translation({2, 1.5f, 1.2f});
        const auto pPost = f.scene->make_static_axis_aligned_box_collider(at, {0.25f, 1.5f, 0.25f});

        REQUIRE_FALSE(f.scene->overlaps_static(level));

        fixture g;
        at.set_translation({2, 1.5f, 0.7f});
        const auto pIn = g.scene->make_static_axis_aligned_box_collider(at, {0.25f, 1.5f, 0.25f});
        REQUIRE(g.scene->overlaps_static(level));

        fixture h;
        at.set_translation({2, 1.5f, 0.75f});
        const auto pTouching = h.scene->make_static_axis_aligned_box_collider(at, {0.25f, 1.5f, 0.25f});
        REQUIRE_FALSE(h.scene->overlaps_static(level));

        fixture k;
        at.set_translation({2, 1.5f, 0});
        const auto pAround = k.scene->make_static_axis_aligned_box_collider(at, {5, 5, 5});
        REQUIRE(k.scene->overlaps_static(level));
    }

    SECTION("on a slope: a step that the level way would meet passes under the rising one, and a lintel over it meets it")
    {
        fixture f;
        matrix4x4_type at;
        at.set_translation({3.5f, 0.75f, 0});
        const auto pLedge = f.scene->make_static_axis_aligned_box_collider(at, {0.5f, 0.75f, 2});

        REQUIRE(f.scene->overlaps_static(level));
        REQUIRE_FALSE(f.scene->overlaps_static(rising));

        fixture g;
        at.set_translation({3.5f, 2.7f, 0});
        const auto pLintel = g.scene->make_static_axis_aligned_box_collider(at, {0.5f, 0.5f, 2});

        REQUIRE_FALSE(g.scene->overlaps_static(level));
        REQUIRE(g.scene->overlaps_static(rising));
    }

    SECTION("an oriented box, a sphere, a capsule and a plane")
    {
        fixture f;
        matrix4x4_type turned;
        quaternion_type eighth;
        eighth.set_from_euler({0, 3.14159265f / 4, 0});
        turned.set_rotation(eighth);
        turned.set_translation({2, 1.5f, 0.8f});
        const auto pTurned = f.scene->make_static_obb_collider(turned, {0.25f, 1.5f, 0.25f});
        REQUIRE(f.scene->overlaps_static(level));

        fixture g;
        turned.set_translation({2, 1.5f, 0.9f});
        const auto pClear = g.scene->make_static_obb_collider(turned, {0.25f, 1.5f, 0.25f});
        REQUIRE_FALSE(g.scene->overlaps_static(level));

        fixture s;
        matrix4x4_type at;
        at.set_translation({2, 1.5f, 0.9f});
        const auto pSphere = s.scene->make_static_sphere_collider(at, 0.5f);
        REQUIRE(s.scene->overlaps_static(level));
        fixture s2;
        at.set_translation({2, 1.5f, 1.1f});
        const auto pFar = s2.scene->make_static_sphere_collider(at, 0.5f);
        REQUIRE_FALSE(s2.scene->overlaps_static(level));

        fixture c;
        matrix4x4_type lying;
        quaternion_type quarter;
        quarter.set_from_euler({0, 0, 3.14159265f / 2});
        lying.set_rotation(quarter);
        lying.set_translation({4.6f, 1.5f, 0});
        const auto pCapsule = c.scene->make_static_capsule_collider(lying, 0.5f, 2.0f);
        REQUIRE(c.scene->overlaps_static(level));

        fixture c2;
        lying.set_translation({6.6f, 1.5f, 0});
        const auto pPast = c2.scene->make_static_capsule_collider(lying, 0.5f, 2.0f);
        REQUIRE_FALSE(c2.scene->overlaps_static(level));

        fixture p;
        const auto pPlane = p.scene->make_static_plane_collider(identity);
        REQUIRE_FALSE(p.scene->overlaps_static(level));
        REQUIRE(p.scene->overlaps_static(parallelepiped{{0, -0.1f, -0.5f},
            {vector3_type{4, 0, 0}, vector3_type{0, 0, 1}, vector3_type{0, 1, 0}}}));
    }

    SECTION("a mesh: a wall across the way, and the same wall past its end")
    {
        fixture f;
        const auto pWall = f.scene->make_static_mesh_collider(identity, impl_mesh_data::make(
            {{3, 0, -2}, {3, 0, 2}, {3, 3, 2}, {3, 3, -2}}, {0, 1, 2, 0, 2, 3}));
        REQUIRE(f.scene->overlaps_static(level));

        fixture g;
        const auto pPast = g.scene->make_static_mesh_collider(identity, impl_mesh_data::make(
            {{4, 0, -2}, {4, 0, 2}, {4, 3, 2}, {4, 3, -2}}, {0, 1, 2, 0, 2, 3}));
        REQUIRE_FALSE(g.scene->overlaps_static(level));

        fixture h;
        const auto pFloor = h.scene->make_static_mesh_collider(identity, impl_mesh_data::make(
            {{-5, 1, -5}, {5, 1, -5}, {5, 1, 5}, {-5, 1, 5}}, {0, 1, 2, 0, 2, 3}));
        REQUIRE_FALSE(h.scene->overlaps_static(level));
    }

    SECTION("a triangle just past an edge, which only the crossing of an edge of each tells apart from it")
    {
        fixture f;
        const auto pPast = f.scene->make_static_mesh_collider(matrix4x4_type(), impl_mesh_data::make(
            {{2, 3.1f, -0.5f}, {2, 0.1f, 2.5f}, {8, 4, 4}}, {0, 1, 2}));
        REQUIRE_FALSE(f.scene->overlaps_static(level));

        fixture g;
        const auto pAcross = g.scene->make_static_mesh_collider(matrix4x4_type(), impl_mesh_data::make(
            {{2, 2.9f, -0.5f}, {2, -0.1f, 2.5f}, {8, 3.8f, 4}}, {0, 1, 2}));
        REQUIRE(g.scene->overlaps_static(level));
    }

    SECTION("flat, it holds nothing")
    {
        fixture f;
        matrix4x4_type at;
        at.set_translation({2, 1.5f, 0});
        const auto pKept = f.scene->make_static_axis_aligned_box_collider(at, {5, 5, 5});

        REQUIRE_FALSE(f.scene->overlaps_static(parallelepiped{{0, 1, 0}, {vector3_type{4, 0, 0}, vector3_type{8, 0, 0},
            vector3_type{0, 1, 0}}}));
    }
}

TEST_CASE("gdk::scene overlaps_static catches whatever rays through the same leaning box catch", "[gdk::collision][overlap]")
{
    fixture f;
    std::vector<const_collider_ptr_type> kept;

    std::uint32_t state = 20261006u;
    const auto next = [&state]() {
        state = state * 1664525u + 1013904223u;

        return static_cast<floating_point_type>(state >> 8) / static_cast<floating_point_type>(1u << 24);
    };

    for (int i = 0; i < 40; ++i) {
        matrix4x4_type at;
        at.set_translation({next() * 20 - 10, next() * 3, next() * 20 - 10});
        kept.push_back(f.scene->make_static_axis_aligned_box_collider(at, {0.1f + next() * 0.6f, 0.1f + next() * 0.6f,
            0.1f + next() * 0.6f}));
    }

    for (int i = 0; i < 40; ++i) {
        const vector3_type corner{next() * 20 - 10, next() * 3, next() * 20 - 10};
        const auto spread = [&] { return vector3_type{next() * 2 - 1, next() * 2 - 1, next() * 2 - 1}; };
        kept.push_back(f.scene->make_static_mesh_collider(matrix4x4_type(), impl_mesh_data::make(
            {corner, corner + spread(), corner + spread()}, {0, 1, 2})));
    }

    std::size_t rayCaught = 0, volumeCaught = 0, missed = 0;

    for (int i = 0; i < 4000; ++i) {
        const vector3_type from{next() * 20 - 10, next() * 2.5f, next() * 20 - 10};
        const vector3_type along{next() * 4 - 2, next() * 1 - 0.5f, next() * 4 - 2};
        const auto flat = std::sqrt(along.x * along.x + along.z * along.z);

        if (flat < 0.1f) continue;

        const vector3_type side{-along.z / flat * 0.45f, 0, along.x / flat * 0.45f};
        const parallelepiped way{from - side, {along, side * 2.0f, vector3_type{0, 0.6f, 0}}};

        bool rays = false;

        for (const floating_point_type up : {0.01f, 0.3f, 0.59f})
            for (const floating_point_type off : {-0.99f, 0.0f, 0.99f}) {
                if (rays) break;

                const auto origin = from + side * off + vector3_type{0, up, 0} + along * 0.001f;

                rays = static_cast<bool>(f.scene->raycast_static(origin, along, along.length() * 0.998f));
            }

        const bool volume = f.scene->overlaps_static(way);

        if (rays) ++rayCaught;
        if (volume) ++volumeCaught;
        if (rays && !volume) ++missed;
    }

    INFO("rays caught " << rayCaught << ", the volume " << volumeCaught);

    REQUIRE(rayCaught > 200);
    REQUIRE(missed == 0);
    REQUIRE(volumeCaught > rayCaught);
}

TEST_CASE("gdk::scene lets a body walk out from under another pressing down on it", "[gdk::collision]")
{
    for (const float press : {0.06f, 0.93f}) {
        fixture f;

        matrix4x4_type beneath;
        beneath.set_translation({0, -0.5f, 0});
        const auto pGround = f.scene->make_static_axis_aligned_box_collider(beneath, {20, 0.5f, 20});

        const auto pUnder = f.scene->make_capsule_collider();
        pUnder->set_radius(0.4f);
        pUnder->set_half_height(0.5375f);
        pUnder->set_position({0, 0.9375f, 0});

        const auto pAbove = f.scene->make_capsule_collider();
        pAbove->set_radius(0.4f);
        pAbove->set_half_height(0.5375f);
        pAbove->set_position({0, 2.8125f, 0});

        for (int frame = 0; frame < 15; ++frame) {
            pUnder->add_velocity({1.8f, -0.93f, 0});
            pAbove->add_velocity({0, -press, 0});
            f.update(1.0f / 30);
        }

        INFO("pressed down at " << press);
        REQUIRE(pUnder->transform().translation().x == Approx(0.9f).margin(0.15f));
        REQUIRE(pUnder->transform().translation().y == Approx(0.9375f).margin(0.05f));
    }
}

TEST_CASE("gdk::scene counts the pairs it tests, and stacked kinematic boxes do not test each other", "[gdk::collision]")
{
    const auto pairs_with = [](const int aBoxes) {
        fixture f;

        std::vector<box_collider_ptr_type> boxes;

        for (int i = 0; i < aBoxes; ++i) {
            boxes.push_back(f.scene->make_axis_aligned_box_collider());
            boxes.back()->set_position({0, 0, 0});
            boxes.back()->set_kinematic(true);
        }

        const auto pBall = f.scene->make_sphere_collider();
        pBall->set_position({0, 1.0f, 0});

        static_cast<void>(f.scene->take_statistics());

        for (int frame = 0; frame < 10; ++frame) {
            for (auto &each : boxes) each->add_velocity({0.1f, 0, 0});
            pBall->add_velocity({0, -1, 0});
            f.update(1.0f / 30);
        }

        const auto seen = f.scene->take_statistics();

        REQUIRE(seen.steps == 10);

        return seen.pairs_tested;
    };

    const auto few = pairs_with(5), many = pairs_with(50);

    INFO("pairs tested in ten steps: " << few << " with five boxes, " << many << " with fifty");

    REQUIRE(few > 0);
    REQUIRE(many <= 10 * 50 * 4);
}
