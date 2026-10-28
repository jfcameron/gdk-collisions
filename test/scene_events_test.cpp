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
    const auto pScene = make_scene({}, {});

    matrix4x4_type identity;
    auto pWall = pScene->make_static_aabb_collider(identity, {0.5f, 0.5f, 0.5f});
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

    const collider *const pDestroyed = pGoing.get();
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

    const auto scene = make_scene([](collision_event) {}, [](trigger_event) {},
        scene_policy{}, make_std_kernels({}, dispatcher));

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

TEST_CASE("gdk::scene reuses a collider's id only after an update has forgotten it", "[gdk::collision]")
{
    fixture f;

    const auto pFloor = f.scene->make_static_aabb_collider(placed_at({0, -1, 0}), {5, 0.5f, 5});

    auto pFirst = f.scene->make_sphere_collider();
    pFirst->set_position({0, 0, 0});

    pFirst->set_velocity({0, -1, 0});
    f.update(0.016f);

    REQUIRE(f.count(event_type::enter) == 1);

    const auto firstId = id_of(pFirst);

    pFirst.reset();

    auto pSecond = f.scene->make_sphere_collider();
    pSecond->set_position({0, 0, 0});

    REQUIRE(id_of(pSecond) != firstId);

    f.collisions.clear();
    pSecond->set_velocity({0, -1, 0});
    f.update(0.016f);

    REQUIRE(f.count(event_type::enter) == 1);
    REQUIRE(f.count(event_type::stay) == 0);

    const auto pThird = f.scene->make_sphere_collider();

    REQUIRE(id_of(pThird) == firstId);

    SECTION("many made and dropped stay within a few ids") {
        collider_id_type highest = 0;

        for (int round = 0; round < 50; ++round) {
            std::vector<sphere_collider_ptr_type> batch;

            for (int i = 0; i < 20; ++i) {
                batch.push_back(f.scene->make_sphere_collider());
                highest = std::max(highest, id_of(batch.back()));
            }

            batch.clear();
            f.update(0.016f);
        }

        REQUIRE(highest < 64);
    }
}
