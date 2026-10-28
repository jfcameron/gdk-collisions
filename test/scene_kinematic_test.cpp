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

TEST_CASE("gdk::scene does not stop a kinematic body", "[gdk::collision]")
{
    fixture f;
    const auto pWall = add_static_box_at_origin(f);

    const auto pLift = f.scene->make_aabb_collider();
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

    const auto pHull = f.scene->make_aabb_collider();
    pHull->set_position({0, 0, 0});
    pHull->set_kinematic(true);

    const auto pOther = f.scene->make_aabb_collider();
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

    const auto pLift = f.scene->make_aabb_collider();
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
    const auto pPlatform = f.scene->make_aabb_collider();
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
    const auto pPlatform = f.scene->make_aabb_collider();
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

    const auto pLift = f.scene->make_aabb_collider();
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

TEST_CASE("gdk::scene counts the pairs it tests, and stacked kinematic boxes do not test each other", "[gdk::collision]")
{
    const auto pairs_with = [](const int aBoxes) {
        fixture f;

        std::vector<aabb_collider_ptr_type> boxes;

        for (int i = 0; i < aBoxes; ++i) {
            boxes.push_back(f.scene->make_aabb_collider());
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
