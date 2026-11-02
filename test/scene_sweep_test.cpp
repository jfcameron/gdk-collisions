// © Joseph Cameron - MIT License

#include "scene_fixture.h"

#include <numbers>

using namespace gdk::collisions;
using namespace gdk::collisions::scene_testing;

namespace {
    [[nodiscard]] capsule standing_at(const vector3_type &aCentre) {
        return capsule{aCentre, 0.5f, 0.5f};
    }
}

TEST_CASE("gdk::scene sweep finds the nearest collider a capsule meets", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_aabb_collider(identity, {0.5f, 0.5f, 0.5f});

    SECTION("a capsule swept at a box stops at its face")
    {
        const auto hit = f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pWall));
        REQUIRE(hit->distance == Approx(9.0f).margin(1e-3f));
        REQUIRE(hit->point.x == Approx(-0.5f).margin(1e-3f));
        REQUIRE(hit->penetration == Approx(0.0f).margin(1e-4f));
        require_vector_near(hit->normal, {-1, 0, 0}, 1e-3f);
    }

    SECTION("a capsule that stops short meets nothing")
    {
        REQUIRE_FALSE(f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 8.5f).has_value());
    }

    SECTION("a capsule moving away meets nothing")
    {
        REQUIRE_FALSE(f.scene->sweep(standing_at({-10, 0, 0}), {-1, 0, 0}, 20.0f).has_value());
    }

    SECTION("direction need not be normalised")
    {
        const auto hit = f.scene->sweep(standing_at({-10, 0, 0}), {7, 0, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->distance == Approx(9.0f).margin(1e-3f));
    }

    SECTION("a degenerate sweep meets nothing rather than misbehaving")
    {
        REQUIRE_FALSE(f.scene->sweep(standing_at({-10, 0, 0}), {0, 0, 0}, 20.0f).has_value());
        REQUIRE_FALSE(f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 0.0f).has_value());
        REQUIRE_FALSE(f.scene->sweep(capsule{{-10, 0, 0}, 0.0f, 0.5f}, {1, 0, 0}, 20.0f).has_value());
        REQUIRE_FALSE(f.scene->sweep(capsule{{-10, 0, 0}, 0.5f, -0.5f}, {1, 0, 0}, 20.0f).has_value());
    }

    SECTION("a capsule told to ignore a collider goes through it to what is beyond")
    {
        const auto pPost = f.scene->make_capsule_collider();
        pPost->set_position({-3, 0, 0});
        pPost->set_radius(0.5f);
        pPost->set_half_height(0.5f);

        const auto first = f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f);

        REQUIRE(first.has_value());
        REQUIRE(first->collider == const_collider_ptr_type(pPost));
        REQUIRE(first->distance == Approx(6.0f).margin(1e-3f));

        const auto through = f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f, pPost.get());

        REQUIRE(through.has_value());
        REQUIRE(through->collider == const_collider_ptr_type(pWall));
        REQUIRE(through->distance == Approx(9.0f).margin(1e-3f));
    }

    SECTION("a capsule meets only colliders on its layers, and goes through the rest")
    {
        constexpr collision_layers_type BODIES = 2, WORLD = DEFAULT_COLLISION_LAYER;

        const auto pPost = f.scene->make_capsule_collider();
        pPost->set_position({-3, 0, 0});
        pPost->set_collision_layers(BODIES, ALL_COLLISION_LAYERS);

        REQUIRE(f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f)->collider
            == const_collider_ptr_type(pPost));

        const auto world = f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f, nullptr, WORLD);

        REQUIRE(world.has_value());
        REQUIRE(world->collider == const_collider_ptr_type(pWall));

        REQUIRE(f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f, nullptr, BODIES)->collider
            == const_collider_ptr_type(pPost));
        REQUIRE_FALSE(f.scene->sweep(standing_at({-1.5f, 0, 0}), {1, 0, 0}, 20.0f, nullptr, BODIES).has_value());

        REQUIRE_FALSE(f.scene->sweep_static(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f, BODIES).has_value());
        REQUIRE(f.scene->sweep_static(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f, WORLD).has_value());
    }

    SECTION("a sweep for the world alone goes through whatever moves")
    {
        const auto pPost = f.scene->make_capsule_collider();
        pPost->set_position({-3, 0, 0});

        const auto hit = f.scene->sweep_static(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pWall));
        REQUIRE(hit->distance == Approx(9.0f).margin(1e-3f));
    }

    SECTION("a moving collider is met where it stands, whatever its velocity")
    {
        const auto pRunner = f.scene->make_capsule_collider();
        pRunner->set_position({-3, 0, 0});
        pRunner->set_radius(0.5f);
        pRunner->set_half_height(0.5f);
        pRunner->set_velocity({-100, 0, 0});

        const auto hit = f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pRunner));
        REQUIRE(hit->distance == Approx(6.0f).margin(1e-3f));
    }

    SECTION("a trigger is never met")
    {
        matrix4x4_type placed;
        placed.set_translation({-3, 0, 0});
        const auto pZone = f.scene->make_static_aabb_trigger(placed, {0.5f, 0.5f, 0.5f});

        const auto hit = f.scene->sweep(standing_at({-10, 0, 0}), {1, 0, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pWall));
    }
}

TEST_CASE("gdk::scene sweep carries a capsule over a floor", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pFloor = f.scene->make_static_mesh_collider(identity, make_quad_mesh(10.0f));

    SECTION("falling, it lands on the floor")
    {
        const auto hit = f.scene->sweep(standing_at({2, 5, 1}), {0, -1, 0}, 10.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->collider == const_collider_ptr_type(pFloor));
        REQUIRE(hit->distance == Approx(4.0f).margin(1e-3f));
        require_vector_near(hit->normal, {0, 1, 0}, 1e-3f);
        require_vector_near(hit->point, {2, 0, 1}, 1e-3f);
    }

    SECTION("standing on it, it walks over its seam and lifts off freely, and meets it pressing down")
    {
        const auto standing = standing_at({-3, 1, -3});

        REQUIRE_FALSE(f.scene->sweep(standing, {1, 0, 0}, 6.0f).has_value());
        REQUIRE_FALSE(f.scene->sweep(standing, {1, 0, 1}, 6.0f).has_value());
        REQUIRE_FALSE(f.scene->sweep(standing, {0, 1, 0}, 6.0f).has_value());

        const auto pressed = f.scene->sweep(standing, {1, -1, 0}, 6.0f);

        REQUIRE(pressed.has_value());
        REQUIRE(pressed->distance == Approx(0.0f).margin(1e-4f));
        require_vector_near(pressed->normal, {0, 1, 0}, 1e-3f);
    }

    SECTION("sunk into it, it is met at once and told how deep, even moving out")
    {
        const auto hit = f.scene->sweep(standing_at({0, 0.8f, 0}), {0, 1, 0}, 5.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->distance == Approx(0.0f).margin(1e-4f));
        REQUIRE(hit->penetration == Approx(0.2f).margin(1e-3f));
        require_vector_near(hit->normal, {0, 1, 0}, 1e-3f);
    }

    SECTION("lying on its side, it lands a radius above the floor")
    {
        auto lying = standing_at({0, 5, 0});
        lying.rotation.set_from_euler({0, 0, std::numbers::pi_v<floating_point_type> / 2});

        const auto hit = f.scene->sweep(lying, {0, -1, 0}, 10.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->distance == Approx(4.5f).margin(1e-3f));
    }
}

TEST_CASE("gdk::scene sweep lands on a heightfield", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pGround = f.scene->make_static_heightfield_collider(identity, make_flat_terrain(9, 9));

    const auto hit = f.scene->sweep(standing_at({4, 5, 4}), {0, -1, 0}, 10.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider == const_collider_ptr_type(pGround));
    REQUIRE(hit->distance == Approx(4.0f).margin(1e-3f));
    require_vector_near(hit->normal, {0, 1, 0}, 1e-3f);
}

TEST_CASE("gdk::scene sweep carries a capsule held clear of two floors over the seam between them, a step at a time",
    "[gdk::collision]")
{
    fixture f;
    const auto pNear = f.scene->make_static_mesh_collider(placed_at({-2, 0, 0}), make_quad_mesh(2.0f));
    const auto pFar = f.scene->make_static_mesh_collider(placed_at({2, 0, 0}), make_quad_mesh(2.0f));

    auto walker = standing_at({-1, 1.01f, 0});

    REQUIRE(f.scene->sweep(walker, {0, -1, 0}, 1.0f)->collider == const_collider_ptr_type(pNear));

    for (int step = 0; step < 20; ++step) {
        INFO("step " << step << " from x " << walker.centre.x);
        REQUIRE_FALSE(f.scene->sweep(walker, {1, 0, 0}, 0.1f).has_value());
        walker.centre.x += 0.1f;
    }

    REQUIRE(f.scene->sweep(walker, {0, -1, 0}, 1.0f)->collider == const_collider_ptr_type(pFar));
}
