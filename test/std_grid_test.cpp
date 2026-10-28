// © Joseph Cameron - MIT License

#include <jfc/catch.hpp>

#include <gdk/collisions/mesh_collider.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/sphere_collider.h>
#include <gdk/collisions/std_kernels.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_mover_grid.h>
#include <gdk/collisions/impl_static_grid.h>

#include <algorithm>
#include <memory>
#include <vector>

using namespace gdk::collisions;

namespace {
    constexpr delta_time_type DELTA_TIME = 1.0f / 60.0f;

    struct fixture final {
        scene_ptr_type pScene = make_scene(nullptr, nullptr);
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

        [[nodiscard]] bounds swept(const std::uint32_t aRow) const {
            const auto standing = bodies[aRow]->world_bounds();
            const auto sweep = bodies[aRow]->velocity() * DELTA_TIME;

            return bounds{vector3_type::min(standing.min, standing.min + sweep),
                vector3_type::max(standing.max, standing.max + sweep)};
        }

        [[nodiscard]] std::uint32_t size() const { return static_cast<std::uint32_t>(bodies.size()); }
    };

    [[nodiscard]] fixture make_fixture() { return fixture{}; }

    [[nodiscard]] std::vector<std::uint32_t> met_by(const impl_static_grid &aGrid, const bounds &aQuery,
        const std::uint32_t aExclude) {
        impl_row_set seen;
        std::vector<std::uint32_t> rows;

        aGrid.visit_overlapping(aQuery, seen, [&](const std::uint32_t aRow) {
            if (aRow != aExclude) rows.push_back(aRow);
            return false;
        });

        std::sort(rows.begin(), rows.end());
        return rows;
    }

    [[nodiscard]] std::vector<std::uint32_t> met_by(const impl_mover_grid &aGrid, const bounds &aQuery,
        const std::uint32_t aExclude) {
        impl_row_set seen;
        std::vector<std::uint32_t> rows;

        aGrid.gather(aQuery, aExclude, seen, rows);

        std::sort(rows.begin(), rows.end());
        return rows;
    }

    void fill(impl_mover_grid &aGrid, const fixture &aFixture) {
        aGrid.clear();

        for (std::uint32_t row = 0; row < aFixture.size(); ++row)
            aGrid.add(row, aFixture.swept(row), aFixture.bodies[row]->world_bounds());

        aGrid.build();
    }

    [[nodiscard]] std::size_t compare_grids(const fixture &aFixture) {
        const std_kernels_policy policy;

        impl_static_grid staticGrid(policy);
        impl_mover_grid moverGrid(policy);

        for (std::uint32_t row = 0; row < aFixture.size(); ++row) staticGrid.insert(row, aFixture.swept(row));

        fill(moverGrid, aFixture);

        std::size_t total = 0;

        for (std::uint32_t row = 0; row < aFixture.size(); ++row) {
            const auto query = aFixture.swept(row);
            const auto fromStatic = met_by(staticGrid, query, row);
            const auto fromMovers = met_by(moverGrid, query, row);

            REQUIRE(std::includes(fromStatic.begin(), fromStatic.end(), fromMovers.begin(), fromMovers.end()));

            for (std::uint32_t other = 0; other < aFixture.size(); ++other)
                if (other != row && bounds_overlap(query, aFixture.swept(other)))
                    REQUIRE(std::binary_search(fromMovers.begin(), fromMovers.end(), other));

            total += fromMovers.size();
        }

        return total;
    }
}

TEST_CASE("gdk::impl_mover_grid agrees with the static grid on a sparse field",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int x = 0; x < 12; ++x)
        for (int z = 0; z < 12; ++z)
            fixture.add({137.0f + x * 4.0f, 3.0f, -211.0f + z * 4.0f}, 0.5f);

    fixture.add({50.0f, 3.0f, 50.0f}, 0.5f);
    fixture.add({50.4f, 3.0f, 50.0f}, 0.5f);

    REQUIRE(compare_grids(fixture) >= 2);
}

TEST_CASE("gdk::impl_mover_grid agrees with the static grid on a dense field",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int x = 0; x < 10; ++x)
        for (int y = 0; y < 4; ++y)
            for (int z = 0; z < 10; ++z)
                fixture.add({71.0f + x * 0.6f, 5.0f + y * 0.6f, -33.0f + z * 0.6f}, 0.5f);

    REQUIRE(compare_grids(fixture) > 400);
}

TEST_CASE("gdk::impl_mover_grid agrees with the static grid on mixed body sizes",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int i = 0; i < 40; ++i)
        fixture.add({100.0f + i * 1.7f, 4.0f, 55.0f}, 0.25f + i * 0.35f);

    fixture.add({100.0f, 4.0f, 55.0f}, 40.0f);

    REQUIRE(compare_grids(fixture) > 41);
}

TEST_CASE("gdk::impl_mover_grid agrees with the static grid on swept bounds",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int i = 0; i < 60; ++i)
        fixture.add({20.0f + i * 2.0f, 9.0f, -14.0f}, 0.5f, {0.0f, -240.0f, 40.0f});

    fixture.add({21.0f, 5.0f, -13.0f}, 0.5f);

    REQUIRE(compare_grids(fixture) >= 1);
}

TEST_CASE("gdk::impl_mover_grid grows when a scene spreads out", "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int x = 0; x < 20; ++x)
        for (int z = 0; z < 20; ++z)
            fixture.add({80.0f + x * 0.5f, 6.0f, 44.0f + z * 0.5f}, 0.5f);

    const std_kernels_policy policy;
    impl_mover_grid grid(policy);

    fill(grid, fixture);

    const auto packedCells = grid.occupied_cell_count();

    for (std::size_t i = 0; i < fixture.handles.size(); ++i)
        fixture.handles[i]->set_position({static_cast<floating_point_type>(i) * 37.0f,
            6.0f, static_cast<floating_point_type>(i) * 23.0f});

    fixture.handles.back()->set_position(fixture.handles.front()->transform().translation());

    fill(grid, fixture);

    REQUIRE(grid.occupied_cell_count() > packedCells);

    REQUIRE(compare_grids(fixture) == 2);
}

TEST_CASE("gdk::impl_mover_grid reports nothing between clear and build",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    for (int i = 0; i < 30; ++i) fixture.add({12.0f + i * 0.4f, 2.0f, 9.0f}, 0.5f);

    const std_kernels_policy policy;
    impl_mover_grid grid(policy);

    fill(grid, fixture);

    REQUIRE(met_by(grid, fixture.swept(0), 0).size() > 0);

    grid.clear();

    REQUIRE(met_by(grid, fixture.swept(0), 0).empty());
}

TEST_CASE("gdk::impl_collider bounds a mesh by its own corners, not round where its collider is", "[gdk::collision]") {
    auto pScene = make_scene(nullptr, nullptr);

    const auto pMesh = make_mesh_data(
        {{64, 4, 64}, {80, 4, 64}, {80, 4, 80}, {64, 4, 80}}, {0, 1, 2, 0, 2, 3});

    const auto pGround = pScene->make_static_mesh_collider(matrix4x4_type::identity, pMesh);
    const auto extent = std::dynamic_pointer_cast<const impl_collider>(pGround)->world_bounds();

    CHECK(extent.min.x == Approx(64));
    CHECK(extent.max.x == Approx(80));
    CHECK(extent.min.z == Approx(64));
    CHECK(extent.max.z == Approx(80));
    CHECK(extent.min.y == Approx(4));

    const std_kernels_policy policy;
    impl_static_grid grid(policy);

    constexpr std::uint32_t GROUND = 1000;

    grid.insert(GROUND, extent);

    auto fixture = make_fixture();
    fixture.add({0, 4.5f, 0}, 0.5f);
    fixture.add({70, 4.5f, 70}, 0.5f);

    CHECK(met_by(grid, fixture.swept(0), 0).empty());
    CHECK(met_by(grid, fixture.swept(1), 1).size() == 1);

    SECTION("and forgets it once removed") {
        grid.remove(GROUND);

        CHECK(met_by(grid, fixture.swept(1), 1).empty());
    }
}

TEST_CASE("gdk::impl_mover_grid answers a moving body to a query its sweep could turn it into",
    "[gdk::collision]") {
    auto fixture = make_fixture();

    fixture.add({0, 4, 0}, 0.5f, {60, 0, 0});
    fixture.add({0.5f, 4, 1.3f}, 0.5f);

    const std_kernels_policy policy;
    impl_mover_grid grid(policy);

    fill(grid, fixture);

    REQUIRE(fixture.swept(0).max.z < fixture.swept(1).min.z);

    const auto met = met_by(grid, fixture.swept(1), 1);

    CHECK(std::find(met.begin(), met.end(), 0u) != met.end());

    fixture.add({0.5f, 4, 4.0f}, 0.5f);

    impl_mover_grid again(policy);

    fill(again, fixture);

    const auto far = met_by(again, fixture.swept(2), 2);

    CHECK(std::find(far.begin(), far.end(), 0u) == far.end());
}
