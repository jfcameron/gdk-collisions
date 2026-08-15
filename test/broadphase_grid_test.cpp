// © Joseph Cameron - All Rights Reserved

#include <jfc/catch.hpp>

#include <gdk/collision_scene.h>
#include <gdk/impl_broadphase_grid.h>
#include <gdk/impl_collider.h>
#include <gdk/impl_collision_policy.h>
#include <gdk/impl_collision_scene.h>
#include <gdk/impl_dynamic_broadphase_grid.h>
#include <gdk/sphere_collider.h>

#include <algorithm>
#include <memory>
#include <unordered_set>
#include <vector>

using namespace gdk;

namespace {
    constexpr collision_delta_time_type DELTA_TIME = 1.0f / 60.0f;

    struct fixture final {
        collision_scene_ptr_type pScene;
        std::vector<sphere_collider_ptr_type> handles;
        std::vector<impl_collider_ptr_type> bodies;

        void add(const collision_vector3_type &aPosition, const collision_floating_point_type aRadius,
            const collision_vector3_type &aVelocity = collision_vector3_type::zero) {
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

        std::size_t total = 0;

        for (const auto &pBody : aFixture.bodies) {
            const auto fromNode = neighbour_ids(nodeGrid, *pBody);
            const auto fromFlat = neighbour_ids(flatGrid, *pBody);

            REQUIRE(fromFlat == fromNode);
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
        fixture.handles[i]->set_position({static_cast<collision_floating_point_type>(i) * 37.0f,
            6.0f, static_cast<collision_floating_point_type>(i) * 23.0f});

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
