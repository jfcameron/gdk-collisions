// © Joseph Cameron - All Rights Reserved

#include <jfc/catch.hpp>

#include <gdk/collisions/scene.h>
#include <gdk/collisions/impl_broadphase_grid.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_collision_scene.h>
#include <gdk/collisions/impl_dynamic_broadphase_grid.h>
#include <gdk/collisions/impl_mesh_data.h>
#include <gdk/collisions/mesh_collider.h>
#include <gdk/collisions/sphere_collider.h>

#include <algorithm>
#include <memory>
#include <unordered_set>
#include <vector>

using namespace gdk::collisions;

namespace {
    constexpr delta_time_type DELTA_TIME = 1.0f / 60.0f;

    struct fixture final {
        scene_ptr_type pScene;
        std::vector<sphere_collider_ptr_type> handles;
        std::vector<impl_collider_ptr_type> bodies;

        void add(const vector3_type &aPosition, const floating_point_type aRadius,
            const vector3_type &aVelocity = vector3_type::zero) {
            auto pSphere = pScene->make_sphere_collider();
            pSphere->set_radius(aRadius);
            pSphere->set_position(aPosition);
            pSphere->set_velocity(aVelocity);

            handles.push_back(pSphere);
            bodies.push_back(std::dynamic_pointer_cast<impl_collider>(pSphere));
        }
    };

    [[nodiscard]] fixture make_fixture() {
        fixture result;
        result.pScene = impl_collision_scene::make(nullptr, nullptr, impl_collision_policy{});
        return result;
    }

    template <typename grid_type>
    [[nodiscard]] std::vector<collider_id_type> neighbour_ids(grid_type &aGrid, const impl_collider &aBody) {
        collider_id_set seen;
        std::vector<impl_broadphase_grid::neighbour> neighbours;
        aGrid.gather_neighbours(aBody, DELTA_TIME, seen, neighbours);

        std::vector<collider_id_type> ids;
        ids.reserve(neighbours.size());
        for (const auto &neighbour : neighbours) ids.push_back(neighbour.ptr->id());

        std::sort(ids.begin(), ids.end());
        return ids;
    }

    [[nodiscard]] std::size_t compare_grids(const fixture &aFixture) {
        const impl_collision_policy policy;

        impl_broadphase_grid nodeGrid(policy);
        impl_dynamic_broadphase_grid flatGrid(policy);

        for (const auto &pBody : aFixture.bodies) {
            const auto bounds = pBody->broad_phase_swept_bounds(DELTA_TIME);
            nodeGrid.insert(pBody, bounds, impl_broadphase_grid::body_kind::collider, false);
            flatGrid.add(pBody, bounds, impl_dynamic_broadphase_grid::body_kind::collider);
        }

        flatGrid.build();

        const auto overlapping = [](const impl_collider::broadphase_bounds &a, const impl_collider::broadphase_bounds &b) {
            return a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y
                && a.min.z <= b.max.z && b.min.z <= a.max.z;
        };

        std::size_t total = 0;

        for (const auto &pBody : aFixture.bodies) {
            const auto fromNode = neighbour_ids(nodeGrid, *pBody);
            const auto fromFlat = neighbour_ids(flatGrid, *pBody);

            REQUIRE(std::includes(fromNode.begin(), fromNode.end(), fromFlat.begin(), fromFlat.end()));

            const auto query = pBody->broad_phase_swept_bounds(DELTA_TIME);

            for (const auto &pOther : aFixture.bodies)
                if (pOther != pBody && overlapping(query, pOther->broad_phase_swept_bounds(DELTA_TIME)))
                    REQUIRE(std::binary_search(fromFlat.begin(), fromFlat.end(), pOther->id()));

            total += fromFlat.size();
        }

        return total;
    }
}

TEST_CASE("gdk::impl_dynamic_broadphase_grid agrees with the node grid on a sparse field",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int x = 0; x < 12; ++x)
        for (int z = 0; z < 12; ++z)
            fixture.add({137.0f + x * 4.0f, 3.0f, -211.0f + z * 4.0f}, 0.5f);

    fixture.add({50.0f, 3.0f, 50.0f}, 0.5f);
    fixture.add({50.4f, 3.0f, 50.0f}, 0.5f);

    REQUIRE(compare_grids(fixture) >= 2);
}

TEST_CASE("gdk::impl_dynamic_broadphase_grid agrees with the node grid on a dense field",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int x = 0; x < 10; ++x)
        for (int y = 0; y < 4; ++y)
            for (int z = 0; z < 10; ++z)
                fixture.add({71.0f + x * 0.6f, 5.0f + y * 0.6f, -33.0f + z * 0.6f}, 0.5f);

    REQUIRE(compare_grids(fixture) > 400);
}

TEST_CASE("gdk::impl_dynamic_broadphase_grid agrees with the node grid on mixed body sizes",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int i = 0; i < 40; ++i)
        fixture.add({100.0f + i * 1.7f, 4.0f, 55.0f}, 0.25f + i * 0.35f);

    fixture.add({100.0f, 4.0f, 55.0f}, 40.0f);

    REQUIRE(compare_grids(fixture) > 41);
}

TEST_CASE("gdk::impl_dynamic_broadphase_grid agrees with the node grid on swept bounds",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int i = 0; i < 60; ++i)
        fixture.add({20.0f + i * 2.0f, 9.0f, -14.0f}, 0.5f, {0.0f, -240.0f, 40.0f});

    fixture.add({21.0f, 5.0f, -13.0f}, 0.5f);

    REQUIRE(compare_grids(fixture) >= 1);
}

TEST_CASE("gdk::impl_dynamic_broadphase_grid grows when a scene spreads out", "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int x = 0; x < 20; ++x)
        for (int z = 0; z < 20; ++z)
            fixture.add({80.0f + x * 0.5f, 6.0f, 44.0f + z * 0.5f}, 0.5f);

    const impl_collision_policy policy;
    impl_dynamic_broadphase_grid grid(policy);

    for (const auto &pBody : fixture.bodies)
        grid.add(pBody, pBody->broad_phase_swept_bounds(DELTA_TIME),
            impl_dynamic_broadphase_grid::body_kind::collider);
    grid.build();

    const auto packedCells = grid.occupied_cell_count();

    for (std::size_t i = 0; i < fixture.handles.size(); ++i)
        fixture.handles[i]->set_position({static_cast<floating_point_type>(i) * 37.0f,
            6.0f, static_cast<floating_point_type>(i) * 23.0f});

    fixture.handles.back()->set_position(fixture.handles.front()->transform().translation());

    grid.clear();
    for (const auto &pBody : fixture.bodies)
        grid.add(pBody, pBody->broad_phase_swept_bounds(DELTA_TIME),
            impl_dynamic_broadphase_grid::body_kind::collider);
    grid.build();

    REQUIRE(grid.occupied_cell_count() > packedCells);

    REQUIRE(compare_grids(fixture) == 2);
}

TEST_CASE("gdk::impl_dynamic_broadphase_grid reports nothing between clear and build",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int i = 0; i < 30; ++i) fixture.add({12.0f + i * 0.4f, 2.0f, 9.0f}, 0.5f);

    const impl_collision_policy policy;
    impl_dynamic_broadphase_grid grid(policy);

    for (const auto &pBody : fixture.bodies)
        grid.add(pBody, pBody->broad_phase_swept_bounds(DELTA_TIME),
            impl_dynamic_broadphase_grid::body_kind::collider);
    grid.build();

    REQUIRE(neighbour_ids(grid, *fixture.bodies.front()).size() > 0);

    grid.clear();

    REQUIRE(neighbour_ids(grid, *fixture.bodies.front()).empty());
}

TEST_CASE("gdk::impl_collider bounds a mesh by its own corners, not round where its collider is", "[gdk::collision]") {
    auto pScene = impl_collision_scene::make(nullptr, nullptr, impl_collision_policy{});

    const auto pMesh = impl_mesh_data::make(
        {{64, 4, 64}, {80, 4, 64}, {80, 4, 80}, {64, 4, 80}}, {0, 1, 2, 0, 2, 3});

    const auto pGround = pScene->make_static_mesh_collider(matrix4x4_type::identity, pMesh);
    const auto bounds = std::dynamic_pointer_cast<const impl_collider>(pGround)->broad_phase_bounds();

    CHECK(bounds.min.x == Approx(64));
    CHECK(bounds.max.x == Approx(80));
    CHECK(bounds.min.z == Approx(64));
    CHECK(bounds.max.z == Approx(80));
    CHECK(bounds.min.y == Approx(4));

    const impl_collision_policy policy;
    impl_broadphase_grid grid(policy);

    grid.insert(std::const_pointer_cast<impl_collider>(std::dynamic_pointer_cast<const impl_collider>(pGround)), bounds,
        impl_broadphase_grid::body_kind::collider, true);

    auto fixture = make_fixture();
    fixture.add({0, 4.5f, 0}, 0.5f);
    fixture.add({70, 4.5f, 70}, 0.5f);

    CHECK(neighbour_ids(grid, *fixture.bodies[0]).empty());
    CHECK(neighbour_ids(grid, *fixture.bodies[1]).size() == 1);
}

TEST_CASE("gdk::impl_dynamic_broadphase_grid answers a moving body to a query its sweep could turn it into",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    fixture.add({0, 4, 0}, 0.5f, {60, 0, 0});
    fixture.add({0.5f, 4, 1.3f}, 0.5f);

    const impl_collision_policy policy;
    impl_dynamic_broadphase_grid grid(policy);

    for (const auto &pBody : fixture.bodies)
        grid.add(pBody, pBody->broad_phase_swept_bounds(DELTA_TIME), impl_dynamic_broadphase_grid::body_kind::collider);

    grid.build();

    REQUIRE(fixture.bodies[0]->broad_phase_swept_bounds(DELTA_TIME).max.z
        < fixture.bodies[1]->broad_phase_swept_bounds(DELTA_TIME).min.z);

    const auto met = neighbour_ids(grid, *fixture.bodies[1]);

    CHECK(std::find(met.begin(), met.end(), fixture.bodies[0]->id()) != met.end());

    fixture.add({0.5f, 4, 4.0f}, 0.5f);

    impl_dynamic_broadphase_grid again(policy);

    for (const auto &pBody : fixture.bodies)
        again.add(pBody, pBody->broad_phase_swept_bounds(DELTA_TIME), impl_dynamic_broadphase_grid::body_kind::collider);

    again.build();

    const auto far = neighbour_ids(again, *fixture.bodies[2]);

    CHECK(std::find(far.begin(), far.end(), fixture.bodies[0]->id()) == far.end());
}
