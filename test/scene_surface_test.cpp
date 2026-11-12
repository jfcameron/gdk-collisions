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

TEST_CASE("gdk::scene does not brake a box above a thin steep triangle", "[gdk::collision][slope_seam]")
{
    const auto degrees = GENERATE(60.f, 75.f);
    const auto width = GENERATE(.01f, .001f, .0001f, .00001f);
    const auto angle = degrees * 3.14159265f / 180;
    const vector3_type normal{-std::sin(angle), std::cos(angle), 0};
    fixture f;
    const auto pSurface = f.scene->make_static_mesh_collider(
        matrix4x4_type::identity,
        make_mesh_data({{0, 0, 0}, {width, width * std::tan(angle), 0}, {0, 0, 5}}, {0, 1, 2}));
    const auto pBody = f.scene->make_obb_collider(response_handlers::slide_projecting());
    const auto position = vector3_type{0, 0, 8} + normal * .65f;
    const vector3_type velocity{0, 0, -40};
    pBody->set_half_extents({.82f, .4f, 1.7f});
    pBody->set_position(position);
    pBody->set_rotation(quaternion_type::from_euler({0, 0, angle - 15 * 3.14159265f / 180}));
    pBody->set_velocity(velocity);
    f.update(.05f);
    CAPTURE(degrees, width, pBody->resolved_velocity().z, pBody->transform().translation().z);
    REQUIRE(f.scene->contacts_for(*pBody).empty());
    require_vector_near(pBody->resolved_velocity(), velocity, .001f);
    require_vector_near(pBody->transform().translation(), position + velocity * .05f, .001f);
}

TEST_CASE("gdk::scene preserves travel across thin strips of a steep mesh", "[gdk::collision][slope_seam]")
{
    const auto degrees = GENERATE(60.f, 75.f);
    const auto width = GENERATE(.01f, .0001f, .00001f, .000001f);
    const auto angle = degrees * 3.14159265f / 180;
    const auto rise = std::tan(angle);
    const auto normal = vector3_type{-rise, 1, 0}.normal();
    const auto rotation = quaternion_type::from_euler({0, 0, angle - 15 * 3.14159265f / 180});
    const vector3_type extents{.82f, .4f, 1.7f};
    const auto radius = std::abs(normal.dot_product(rotation * vector3_type{1, 0, 0})) * extents.x +
                        std::abs(normal.dot_product(rotation * vector3_type{0, 1, 0})) * extents.y;
    std::vector<vector3_type> vertices;
    std::vector<std::uint32_t> indices;
    for (const auto limits : {std::pair{-10.f, 0.f}, std::pair{0.f, width}, std::pair{width, 10.f}}) {
        const auto base = static_cast<std::uint32_t>(vertices.size());
        vertices.insert(vertices.end(), {{-5, -5 * rise, limits.first},
                                         {5, 5 * rise, limits.first},
                                         {5, 5 * rise, limits.second},
                                         {-5, -5 * rise, limits.second}});
        indices.insert(indices.end(), {base, base + 2, base + 1, base, base + 3, base + 2});
    }
    fixture f;
    const auto pSurface = f.scene->make_static_mesh_collider(matrix4x4_type::identity,
                                                             make_mesh_data(std::move(vertices), std::move(indices)));
    const auto pBody = f.scene->make_obb_collider(response_handlers::slide_projecting());
    const auto position = vector3_type{0, 0, 3} + normal * (radius + .0000004f);
    const vector3_type velocity{0, 0, -40};
    pBody->set_half_extents(extents);
    pBody->set_position(position);
    pBody->set_rotation(rotation);
    pBody->set_velocity(velocity);
    f.update(.05f);
    CAPTURE(degrees, width, pBody->resolved_velocity().z, pBody->transform().translation().z);
    REQUIRE(pBody->resolved_velocity().z == Approx(velocity.z).margin(.001));
    REQUIRE(pBody->transform().translation().z == Approx(position.z + velocity.z * .05f).margin(.001));
}

TEST_CASE("gdk::heightfield keeps thin coplanar edges internal", "[gdk::collision][slope_seam]")
{
    const auto width = GENERATE(.01f, .0001f, .00001f, .000001f);
    const auto pData = make_heightfield_data(2, 3, {-10, 10, -10, 10, -10, 10}, {10, width});
    const auto pImplementation = std::dynamic_pointer_cast<const impl_heightfield_data>(pData);
    REQUIRE(pImplementation);
    CAPTURE(width);
    REQUIRE((pImplementation->internal_edges(0) & 4) != 0);
    REQUIRE((pImplementation->internal_edges(1) & 3) == 3);
}

TEST_CASE("gdk::scene still blocks an exposed thin mesh rim", "[gdk::collision][slope_seam]")
{
    const auto width = GENERATE(.01f, .0001f, .00001f, .000001f);
    const auto angle = 75 * 3.14159265f / 180;
    const auto rise = std::tan(angle);
    const auto normal = vector3_type{-rise, 1, 0}.normal();
    const auto rotation = quaternion_type::from_euler({0, 0, angle - 15 * 3.14159265f / 180});
    const vector3_type extents{.82f, .4f, 1.7f};
    const auto radius = std::abs(normal.dot_product(rotation * vector3_type{1, 0, 0})) * extents.x +
                        std::abs(normal.dot_product(rotation * vector3_type{0, 1, 0})) * extents.y;
    fixture f;
    const auto pSurface = f.scene->make_static_mesh_collider(
        matrix4x4_type::identity,
        make_mesh_data({{-5, -5 * rise, 0}, {5, 5 * rise, 0}, {5, 5 * rise, width}, {-5, -5 * rise, width}},
                       {0, 2, 1, 0, 3, 2}));
    const auto pBody = f.scene->make_obb_collider(response_handlers::slide_projecting());
    pBody->set_half_extents(extents);
    pBody->set_position(vector3_type{0, 0, 3} + normal * (radius - .1f));
    pBody->set_rotation(rotation);
    pBody->set_velocity({0, 0, -40});
    f.update(.05f);
    CAPTURE(width);
    REQUIRE_FALSE(f.scene->contacts_for(*pBody).empty());
    REQUIRE(std::abs(pBody->resolved_velocity().z) < .001f);
}

TEST_CASE("gdk::scene does not collide with the extended plane outside a finite triangle", "[gdk::collision][swept_sat]")
{
    fixture f;
    const auto pSurface = f.scene->make_static_mesh_collider(matrix4x4_type::identity,
        make_mesh_data({{0, 0, 0}, {5, 0.360625267f, 0}, {5, -0.827378498f, 5}}, {0, 1, 2}));
    const auto pBody = f.scene->make_obb_collider(response_handlers::slide_projecting());
    const vector3_type position{-1.386f, 0.597866341f, 0.329f};
    const vector3_type velocity{20.806f, 1.132f, -13.594f};
    pBody->set_half_extents({0.82f, 0.40f, 1.70f});
    pBody->set_position(position);
    pBody->set_rotation(quaternion_type::from_euler({0.077f, -1.012f, 0.048f}));
    pBody->set_velocity(velocity);
    constexpr float STEP = 1.f / 120;
    f.update(STEP);
    REQUIRE(f.scene->contacts_for(*pBody).empty());
    REQUIRE((pBody->resolved_velocity() - velocity).length() < 0.001f);
    REQUIRE((pBody->transform().translation() - (position + velocity * STEP)).length() < 0.001f);
}

TEST_CASE("gdk::scene rejects triangle sweeps whose axis overlap times are disjoint", "[gdk::collision][swept_sat]")
{
    for (const float scale : {1.f, 10.f, 100.f}) {
        fixture f({.CONSERVATIVE_ADVANCEMENT_MAX_STEPS = 1});
        const auto pSurface = f.scene->make_static_mesh_collider(matrix4x4_type::identity,
            make_mesh_data({{0, 0, 0}, {5, 0, 0}, {0, 0, 5}}, {0, 1, 2}));
        const auto pBody = f.scene->make_obb_collider(response_handlers::slide_projecting());
        const vector3_type position{-2, 1.1f, -2};
        const auto velocity = vector3_type{4, -4, 4} * scale;
        pBody->set_half_extents({0.1f, 0.1f, 0.1f});
        pBody->set_position(position);
        pBody->set_velocity(velocity);
        // It passes below the plane before its X/Z footprint reaches the triangle.
        f.update(1 / scale);
        REQUIRE(f.scene->contacts_for(*pBody).empty());
        REQUIRE((pBody->resolved_velocity() - velocity).length() < 0.001f);
        REQUIRE((pBody->transform().translation() - vector3_type{2, -2.9f, 2}).length() < 0.001f);
    }
}

TEST_CASE("gdk::scene still catches a fast box when all triangle sweep intervals overlap", "[gdk::collision][swept_sat]")
{
    fixture f({.CONSERVATIVE_ADVANCEMENT_MAX_STEPS = 1});
    const auto pSurface = f.scene->make_static_mesh_collider(matrix4x4_type::identity,
        make_mesh_data({{-10, 0, -10}, {10, 0, -10}, {0, 0, 10}}, {0, 1, 2}));
    const auto pBody = f.scene->make_obb_collider(response_handlers::slide_projecting());
    pBody->set_half_extents({0.5f, 0.5f, 0.5f});
    pBody->set_position({-2, 5, 0});
    pBody->set_velocity({20, -100, 0});
    f.update(0.1f);
    REQUIRE_FALSE(f.scene->contacts_for(*pBody).empty());
    REQUIRE(pBody->transform().translation().y == Approx(0.5f).margin(0.001f));
    REQUIRE(std::abs(pBody->resolved_velocity().y) < 0.001f);
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
