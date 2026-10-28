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

    const auto pLiving = f.scene->make_aabb_collider();
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
    const auto pBox = f.scene->make_aabb_collider();
    pBox->set_collision_layers(2, 2);

    const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider == const_collider_ptr_type(pBox));
}
