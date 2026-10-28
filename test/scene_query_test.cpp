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

TEST_CASE("gdk::scene raycast finds the nearest collider", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_aabb_collider(identity, {0.5f, 0.5f, 0.5f});

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

    const auto pNear = f.scene->make_static_aabb_collider(near, {0.5f, 0.5f, 0.5f});
    const auto pFar = f.scene->make_static_aabb_collider(far, {0.5f, 0.5f, 0.5f});

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
        const auto pTarget = f.scene->make_static_aabb_collider(identity, {0.5f, 0.5f, 0.5f});
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
    const auto pWall = f.scene->make_static_aabb_collider(identity, {0.5f, 0.5f, 0.5f});

    const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider == const_collider_ptr_type(pWall));
    REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
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

TEST_CASE("gdk::scene raycasts against a persistent static", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_aabb_collider(identity, {0.5f, 0.5f, 0.5f});

    const auto hit = f.scene->raycast({-10, 0, 0}, {1, 0, 0}, 20.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider == const_collider_ptr_type(pWall));
    REQUIRE(hit->distance == Approx(9.5f).margin(1e-2f));
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
    const auto pFloor = f.scene->make_static_mesh_collider(identity, make_mesh_data(std::move(vertices), std::move(indices)));

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

TEST_CASE("gdk::scene raycast answers exactly, shape by shape", "[gdk::collision][raycast]")
{
    matrix4x4_type identity;

    SECTION("a box: the face it meets, its normal toward the ray, and from inside nought and the nearest face")
    {
        fixture f;
        const auto pKept = (f.scene->make_static_aabb_collider(identity, {0.5f, 1.0f, 0.5f}));

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
        const auto pKept = f.scene->make_static_aabb_collider(at, {0.5f, 1.5f, 0.5f});

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
        const auto pWall = f.scene->make_static_mesh_collider(identity, make_mesh_data(
            {{1, 0, -2}, {1, 0, 2}, {1, 3, 2}, {1, 3, -2}}, {0, 1, 2, 0, 2, 3}));

        REQUIRE(f.scene->overlaps_static(room_at(0.6f, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0.4f, 0)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0.5f, 0)));
        REQUIRE(f.scene->overlaps_static(room_at(0.8f, 2.3f)));
        REQUIRE_FALSE(f.scene->overlaps_static(room_at(0.8f, 2.6f)));

        fixture g;
        const auto pFloor = g.scene->make_static_mesh_collider(identity, make_mesh_data(
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
        const auto pHill = f.scene->make_static_heightfield_collider(identity, make_heightfield_data(5, 5, heights));

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
        const auto pKept = f.scene->make_static_aabb_collider(at, {0.5f, 1.5f, 0.5f});

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
        kept.push_back(f.scene->make_static_aabb_collider(at, {0.1f + next() * 0.6f, 0.1f + next() * 0.6f,
            0.1f + next() * 0.6f}));
    }

    for (int i = 0; i < 40; ++i) {
        const vector3_type corner{next() * 20 - 10, next() * 3, next() * 20 - 10};
        const auto spread = [&] { return vector3_type{next() * 2 - 1, next() * 2 - 1, next() * 2 - 1}; };
        kept.push_back(f.scene->make_static_mesh_collider(matrix4x4_type(), make_mesh_data(
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
        const auto pKept = f.scene->make_static_aabb_collider(at, {0.5f, 0.5f, 0.5f});

        REQUIRE(f.scene->raycast_static({5, 10, 0}, {0, -1, 0}, 20.0f));
        REQUIRE(f.scene->overlaps_static({{5, 0, 0}, 1.0f, 0.2f}));
    }

    SECTION("made by a response, during an update: found after it, and after the next")
    {
        fixture f;
        matrix4x4_type floor;
        floor.set_translation({0, -0.5f, 0});
        const auto pFloor = f.scene->make_static_aabb_collider(floor, {10, 0.5f, 10});

        const_aabb_collider_ptr_type pMade;
        auto &scene = *f.scene;

        auto pBall = f.scene->make_sphere_collider([&](collider &, const contact_context &) {
            if (!pMade) pMade = scene.make_static_aabb_collider(at, {0.5f, 0.5f, 0.5f});

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
        const auto pPost = f.scene->make_static_aabb_collider(at, {0.25f, 1.5f, 0.25f});

        REQUIRE_FALSE(f.scene->overlaps_static(level));

        fixture g;
        at.set_translation({2, 1.5f, 0.7f});
        const auto pIn = g.scene->make_static_aabb_collider(at, {0.25f, 1.5f, 0.25f});
        REQUIRE(g.scene->overlaps_static(level));

        fixture h;
        at.set_translation({2, 1.5f, 0.75f});
        const auto pTouching = h.scene->make_static_aabb_collider(at, {0.25f, 1.5f, 0.25f});
        REQUIRE_FALSE(h.scene->overlaps_static(level));

        fixture k;
        at.set_translation({2, 1.5f, 0});
        const auto pAround = k.scene->make_static_aabb_collider(at, {5, 5, 5});
        REQUIRE(k.scene->overlaps_static(level));
    }

    SECTION("on a slope: a step that the level way would meet passes under the rising one, and a lintel over it meets it")
    {
        fixture f;
        matrix4x4_type at;
        at.set_translation({3.5f, 0.75f, 0});
        const auto pLedge = f.scene->make_static_aabb_collider(at, {0.5f, 0.75f, 2});

        REQUIRE(f.scene->overlaps_static(level));
        REQUIRE_FALSE(f.scene->overlaps_static(rising));

        fixture g;
        at.set_translation({3.5f, 2.7f, 0});
        const auto pLintel = g.scene->make_static_aabb_collider(at, {0.5f, 0.5f, 2});

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
        const auto pWall = f.scene->make_static_mesh_collider(identity, make_mesh_data(
            {{3, 0, -2}, {3, 0, 2}, {3, 3, 2}, {3, 3, -2}}, {0, 1, 2, 0, 2, 3}));
        REQUIRE(f.scene->overlaps_static(level));

        fixture g;
        const auto pPast = g.scene->make_static_mesh_collider(identity, make_mesh_data(
            {{4, 0, -2}, {4, 0, 2}, {4, 3, 2}, {4, 3, -2}}, {0, 1, 2, 0, 2, 3}));
        REQUIRE_FALSE(g.scene->overlaps_static(level));

        fixture h;
        const auto pFloor = h.scene->make_static_mesh_collider(identity, make_mesh_data(
            {{-5, 1, -5}, {5, 1, -5}, {5, 1, 5}, {-5, 1, 5}}, {0, 1, 2, 0, 2, 3}));
        REQUIRE_FALSE(h.scene->overlaps_static(level));
    }

    SECTION("a triangle just past an edge, which only the crossing of an edge of each tells apart from it")
    {
        fixture f;
        const auto pPast = f.scene->make_static_mesh_collider(matrix4x4_type(), make_mesh_data(
            {{2, 3.1f, -0.5f}, {2, 0.1f, 2.5f}, {8, 4, 4}}, {0, 1, 2}));
        REQUIRE_FALSE(f.scene->overlaps_static(level));

        fixture g;
        const auto pAcross = g.scene->make_static_mesh_collider(matrix4x4_type(), make_mesh_data(
            {{2, 2.9f, -0.5f}, {2, -0.1f, 2.5f}, {8, 3.8f, 4}}, {0, 1, 2}));
        REQUIRE(g.scene->overlaps_static(level));
    }

    SECTION("flat, it holds nothing")
    {
        fixture f;
        matrix4x4_type at;
        at.set_translation({2, 1.5f, 0});
        const auto pKept = f.scene->make_static_aabb_collider(at, {5, 5, 5});

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
        kept.push_back(f.scene->make_static_aabb_collider(at, {0.1f + next() * 0.6f, 0.1f + next() * 0.6f,
            0.1f + next() * 0.6f}));
    }

    for (int i = 0; i < 40; ++i) {
        const vector3_type corner{next() * 20 - 10, next() * 3, next() * 20 - 10};
        const auto spread = [&] { return vector3_type{next() * 2 - 1, next() * 2 - 1, next() * 2 - 1}; };
        kept.push_back(f.scene->make_static_mesh_collider(matrix4x4_type(), make_mesh_data(
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

TEST_CASE("gdk::scene walks a long ray cell by cell", "[gdk::collision]")
{
    fixture f;

    const auto pNear = f.scene->make_static_sphere_collider(placed_at({0, 0, 0}), 0.5f);
    const auto pFar = f.scene->make_static_sphere_collider(placed_at({400, 400, 400}), 0.5f);

    const auto hit = f.scene->raycast({-500, -500, -500}, {1, 1, 1}, 2000.0f);

    REQUIRE(hit.has_value());
    REQUIRE(hit->collider.get() == pNear.get());
    REQUIRE(hit->distance == Approx(500.0f * std::sqrt(3.0f) - 0.5f).margin(1e-3f));

    SECTION("and finds what lies beyond a miss") {
        const auto past = f.scene->raycast({1, -500, -500}, {0, 1, 1}, 2000.0f);

        REQUIRE_FALSE(past.has_value());

        const auto behind = f.scene->raycast({400, -600, -600}, {0, 1, 1}, 2000.0f);

        REQUIRE(behind.has_value());
        REQUIRE(behind->collider.get() == pFar.get());
    }
}

TEST_CASE("gdk::scene sees spheres and capsules against a long thin parallelepiped", "[gdk::collision]")
{
    fixture f;

    parallelepiped volume;
    volume.corner = {0, 0, 0};
    volume.edges = {vector3_type{100, 0, 0}, vector3_type{0, 0.1f, 0}, vector3_type{0, 0, 0.1f}};

    SECTION("a sphere reaching down into it") {
        const auto pBall = f.scene->make_static_sphere_collider(placed_at({50, 0.6f, 0.05f}), 0.52f);

        REQUIRE(f.scene->overlaps_static(volume));
    }

    SECTION("a sphere stopping short") {
        const auto pBall = f.scene->make_static_sphere_collider(placed_at({50, 0.6f, 0.05f}), 0.48f);

        REQUIRE_FALSE(f.scene->overlaps_static(volume));
    }

    SECTION("a capsule passing through with both ends outside") {
        const auto pPole = f.scene->make_static_capsule_collider(placed_at({50, 0.05f, 0.05f}), 0.01f, 3.0f);

        REQUIRE(f.scene->overlaps_static(volume));
    }

    SECTION("a capsule beside it") {
        const auto pPole = f.scene->make_static_capsule_collider(placed_at({50, 0.05f, 0.5f}), 0.3f, 3.0f);

        REQUIRE_FALSE(f.scene->overlaps_static(volume));
    }
}
