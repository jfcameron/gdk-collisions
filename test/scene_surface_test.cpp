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
            const auto pSlope = f.scene->make_static_heightfield_collider(identity, make_heightfield_data(33, 33, [&] {
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

TEST_CASE("gdk::impl_mesh_data and impl_heightfield_data refuse numbers that are not finite", "[gdk::collision]")
{
    const auto nan = std::numeric_limits<floating_point_type>::quiet_NaN();
    const auto inf = std::numeric_limits<floating_point_type>::infinity();

    REQUIRE_THROWS_AS(make_mesh_data({{0, 0, 0}, {1, nan, 0}, {0, 0, 1}}, {0, 1, 2}), exception);
    REQUIRE_THROWS_AS(make_mesh_data({{0, 0, 0}, {1, 0, inf}, {0, 0, 1}}, {0, 1, 2}), exception);

    REQUIRE_THROWS_AS(make_heightfield_data(2, 2, {0, 0, nan, 0}), exception);
    REQUIRE_THROWS_AS(make_heightfield_data(2, 2, {0, 0, 0, 0}, {nan, 1}), exception);
    REQUIRE_THROWS_AS(make_heightfield_data(2, 2, {0, 0, 0, 0}, {1, inf}), exception);

    const auto huge = std::numeric_limits<std::size_t>::max() / 2 + 2;
    REQUIRE_THROWS_AS(make_heightfield_data(huge, 4, {0, 0, 0, 0}), exception);
}

TEST_CASE("gdk::scene lands a falling mesh on a plane", "[gdk::collision]")
{
    fixture f;

    const auto pGround = f.scene->make_static_plane_collider(matrix4x4_type::identity);

    const auto pRaft = f.scene->make_mesh_collider(response_handlers::slide_projecting());
    pRaft->set_mesh(make_mesh_data({{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, {0, 1, 2, 0, 2, 3}));
    pRaft->set_position({0, 1, 0});

    for (int frame = 0; frame < 60; ++frame) {
        pRaft->add_velocity({0, -5, 0});
        f.update(1.0f / 30);
    }

    REQUIRE(pRaft->transform().translation().y == Approx(0.0f).margin(1e-2f));
    REQUIRE(f.count(event_type::enter) == 1);
}
