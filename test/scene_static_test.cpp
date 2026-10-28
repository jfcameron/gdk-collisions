// © Joseph Cameron - MIT License

#include "scene_fixture.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

using namespace gdk::collisions;
using namespace gdk::collisions::scene_testing;

TEST_CASE("gdk::scene sizes a box from its half extents", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);
    const auto pBox = f.scene->make_aabb_collider();
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
        const auto pGround = f.scene->make_static_aabb_collider(beneath, {4, 0.5f, 4});

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

    const auto pGround = f.scene->make_static_aabb_collider(beneath, {200.0f, 0.5f, 200.0f});

    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({0, 5, 0});

    for (int frame = 0; frame < 10; ++frame) {
        pSphere->add_velocity({0, -1, 0});
        f.update(1.0f);
    }

    REQUIRE(pSphere->transform().translation().y == Approx(0.5f).margin(1e-2f));
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

TEST_CASE("gdk::scene forgets a static once its owner does", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_position({-1.5f, 0, 0});

    {
        const auto pWall = f.scene->make_static_aabb_collider(identity, {0.5f, 0.5f, 0.5f});

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

TEST_CASE("gdk::scene gives an empty compound no geometry", "[gdk::collision]")
{
    fixture f;

    const auto pWall = f.scene->make_static_aabb_collider(placed_at({0, 0, 0}), {1, 1, 1});

    const auto pCompound = f.scene->make_compound_collider();
    pCompound->set_position({0, 0.5f, 0});

    f.update(0.016f);

    REQUIRE(pCompound->part_count() == 0);
    REQUIRE(f.collisions.empty());
    REQUIRE(pCompound->transform().translation().y == Approx(0.5f));

    SECTION("a ray passes through it") {
        const auto pFloating = f.scene->make_compound_collider();
        pFloating->set_position({0, 5, 0});

        const auto hit = f.scene->raycast({0, 10, 0}, {0, -1, 0}, 20.0f);

        REQUIRE(hit.has_value());
        REQUIRE(hit->collider.get() == pWall.get());
    }

    SECTION("parts added and cleared") {
        pCompound->add_sphere({0, 0, 0}, 0.25f);
        REQUIRE(pCompound->part_count() == 1);

        pCompound->clear_parts();
        REQUIRE(pCompound->part_count() == 0);
    }
}
