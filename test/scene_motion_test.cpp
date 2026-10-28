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
    const auto pBox = f.scene->make_aabb_collider();
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

TEST_CASE("gdk::scene reports the velocity a body finished its step with", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_aabb_collider(identity, {5, 5, 0.5f});

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
                            const auto pSolid = f.scene->make_static_aabb_collider(identity, {2, 2, 2});
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
                        else if (shape == "box") pBody = f.scene->make_aabb_collider();
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

TEST_CASE("gdk::scene halts a weight zero dynamic at a contact", "[gdk::collision]")
{
    fixture f;
    const auto pMover = f.scene->make_aabb_collider();
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

TEST_CASE("gdk::scene lets a body walk out from under another pressing down on it", "[gdk::collision]")
{
    for (const float press : {0.06f, 0.93f}) {
        fixture f;

        matrix4x4_type beneath;
        beneath.set_translation({0, -0.5f, 0});
        const auto pGround = f.scene->make_static_aabb_collider(beneath, {20, 0.5f, 20});

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

TEST_CASE("gdk::scene moves bodies through every step of an update", "[gdk::collision]")
{
    fixture f(scene_policy{.STEPS_PER_UPDATE = 4});

    const auto pSphere = f.scene->make_sphere_collider();
    pSphere->set_velocity({1, 0, 0});

    const auto pSpinner = f.scene->make_sphere_collider();
    pSpinner->set_position({0, 10, 0});
    pSpinner->set_angular_velocity({0, 3.14159265f, 0});

    f.update(1.0f);

    REQUIRE(pSphere->transform().translation().x == Approx(1.0f).margin(1e-4f));
    REQUIRE(pSphere->resolved_velocity().x == Approx(1.0f).margin(1e-4f));
    REQUIRE(pSphere->velocity().x == 0);

    const auto heading = pSpinner->rotation() * vector3_type{1, 0, 0};
    REQUIRE(heading.x == Approx(-1.0f).margin(1e-3f));
}

TEST_CASE("gdk::scene gathers neighbours for a body crossing a vast distance", "[gdk::collision]")
{
    fixture f;

    const auto pWall = f.scene->make_static_aabb_collider(placed_at({300, 300, 300}), {5, 5, 5});

    const auto pBullet = f.scene->make_sphere_collider(response_handlers::null_opt);
    pBullet->set_radius(0.1f);
    pBullet->set_velocity({1000, 1000, 1000});

    f.update(1.0f);

    REQUIRE(pBullet->transform().translation().x == Approx(295.0f - 0.1f / std::sqrt(3.0f)).margin(0.2f));
    REQUIRE(f.count(event_type::enter) == 1);
}

TEST_CASE("gdk::scene survives bodies at the edge of float range", "[gdk::collision]")
{
    fixture f;

    const auto pFar = f.scene->make_sphere_collider();
    pFar->set_position({3e10f, -3e10f, 1e30f});
    pFar->set_velocity({1, 0, 0});

    const auto pStatic = f.scene->make_static_sphere_collider(placed_at({-3e10f, 0, 0}), 1.0f);

    f.update(0.016f);

    REQUIRE_FALSE(f.scene->raycast({3e10f, 0, 0}, {1, 0, 0}, 1e9f).has_value());
    REQUIRE_FALSE(f.scene->overlaps_static(upright_cylinder{{3e10f, 0, 0}, 2, 1}));
}

TEST_CASE("gdk::make_scene refuses a policy it cannot run", "[gdk::collision]")
{
    REQUIRE_NOTHROW(make_scene(nullptr, nullptr, scene_policy{}));

    REQUIRE_THROWS_AS(make_scene(nullptr, nullptr, scene_policy{.STEPS_PER_UPDATE = 0}), exception);
    REQUIRE_THROWS_AS(make_scene(nullptr, nullptr,
        scene_policy{.CONSERVATIVE_ADVANCEMENT_MAX_STEPS = 0}), exception);
    REQUIRE_THROWS_AS(make_scene(nullptr, nullptr, scene_policy{.MIN_SWEPT_SPEED = -1}),
        exception);
    REQUIRE_THROWS_AS(make_scene(nullptr, nullptr,
        scene_policy{.CLIP_PLANE_PARALLEL_COSINE = 1.5f}), exception);
}

TEST_CASE("gdk::make_std_kernels refuses a policy it cannot run", "[gdk::collision]")
{
    REQUIRE_NOTHROW(make_std_kernels());

    REQUIRE_THROWS_AS(make_std_kernels(std_kernels_policy{.BROAD_PHASE_CELL_SIZE = 0}), exception);
    REQUIRE_THROWS_AS(make_std_kernels(
        std_kernels_policy{.BROAD_PHASE_CELL_SIZE = std::numeric_limits<floating_point_type>::infinity()}), exception);
    REQUIRE_THROWS_AS(make_std_kernels(std_kernels_policy{.PARALLEL_CHUNK_SIZE = 0}), exception);
    REQUIRE_THROWS_AS(make_std_kernels(std_kernels_policy{.SWEEP_MAX_STEPS = 0}), exception);
    REQUIRE_THROWS_AS(make_std_kernels(std_kernels_policy{.INTERNAL_EDGE_GHOST_COSINE = -1}), exception);
}
