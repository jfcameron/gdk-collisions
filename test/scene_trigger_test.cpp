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
    const auto pWall = f.scene->make_static_aabb_collider(identity, {0.5f, 0.5f, 0.5f});
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

TEST_CASE("gdk::scene reports a box trigger only when a box reaches it", "[gdk::collision]")
{
    fixture f;

    const auto pBox = f.scene->make_aabb_collider();
    const auto pTrigger = f.scene->make_static_aabb_trigger(placed_at({1.6f, 0, 0}), {0.5f, 0.5f, 0.5f});

    pBox->set_velocity({1, 0, 0});
    f.update(0.016f);

    REQUIRE(f.triggers.empty());

    for (int frame = 0; frame < 40 && f.triggers.empty(); ++frame) {
        pBox->set_velocity({1, 0, 0});
        f.update(0.016f);
    }

    REQUIRE_FALSE(f.triggers.empty());
    REQUIRE(pBox->transform().translation().x >= 0.6f - 0.02f);
}

TEST_CASE("gdk::scene sweeps a moving trigger against where movers ended up", "[gdk::collision]")
{
    fixture f;

    const auto pBox = f.scene->make_aabb_collider();
    const auto pTrigger = f.scene->make_aabb_trigger();
    pTrigger->set_position({2.1f, 0, 0});

    pBox->set_velocity({10, 0, 0});
    f.update(0.1f);

    REQUIRE(pBox->transform().translation().x == Approx(1.0f).margin(1e-4f));
    REQUIRE(f.triggers.empty());
}
