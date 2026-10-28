// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_TEST_SCENE_FIXTURE_H
#define GDK_COLLISIONS_TEST_SCENE_FIXTURE_H

#include <jfc/catch.hpp>

#include <gdk/collisions/aabb_collider.h>
#include <gdk/collisions/capsule_collider.h>
#include <gdk/collisions/collider.h>
#include <gdk/collisions/compound_collider.h>
#include <gdk/collisions/events.h>
#include <gdk/collisions/exception.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/heightfield_collider.h>
#include <gdk/collisions/mesh_collider.h>
#include <gdk/collisions/obb_collider.h>
#include <gdk/collisions/plane_collider.h>
#include <gdk/collisions/raycast_hit.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/sphere_collider.h>
#include <gdk/collisions/std_kernels.h>
#include <gdk/collisions/impl_capsule_collider.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_heightfield_data.h>
#include <gdk/collisions/impl_mesh_data.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace gdk::collisions::scene_testing {
    struct fixture final {
        std::vector<collision_event> collisions;
        std::vector<trigger_event> triggers;
        scene_ptr_type scene;

        explicit fixture(scene_policy aPolicy = {}, scene_kernels_ptr_type aKernels = {})
        : scene(make_scene(
            [this](collision_event e) { collisions.push_back(e); },
            [this](trigger_event e) { triggers.push_back(e); }, aPolicy, std::move(aKernels)))
        {}

        void update(const delta_time_type aDeltaTime) {
            scene->update(aDeltaTime);
            scene->process_events();
        }

        [[nodiscard]] std::size_t count(const event_type aType) const {
            std::size_t n = 0;
            for (const auto &e : collisions) if (e.type == aType) ++n;
            return n;
        }
    };

    inline void require_vector_near(const vector3_type &aActual, const vector3_type &aExpected,
        const floating_point_type aMargin) {
        REQUIRE(aActual.x == Approx(aExpected.x).margin(aMargin));
        REQUIRE(aActual.y == Approx(aExpected.y).margin(aMargin));
        REQUIRE(aActual.z == Approx(aExpected.z).margin(aMargin));
    }

    [[nodiscard]] inline matrix4x4_type placed_at(const vector3_type &aPosition) {
        auto transform = matrix4x4_type::identity;
        transform.set_translation(aPosition);
        return transform;
    }

    [[nodiscard]] inline collider_id_type id_of(const collider_ptr_type &aCollider) {
        return std::dynamic_pointer_cast<const impl_collider>(aCollider)->id();
    }

    [[nodiscard]] inline mesh_data_ptr_type make_quad_mesh(const floating_point_type aHalfSize) {
        return make_mesh_data(
            {{-aHalfSize, 0, -aHalfSize}, {aHalfSize, 0, -aHalfSize},
             {aHalfSize, 0, aHalfSize}, {-aHalfSize, 0, aHalfSize}},
            {0, 1, 2, 0, 2, 3});
    }

    [[nodiscard]] inline mesh_data_ptr_type make_open_box_mesh() {
        std::vector<vector3_type> vertices;
        std::vector<std::uint32_t> indices;

        const auto quad = [&](const vector3_type &aA, const vector3_type &aB,
            const vector3_type &aC, const vector3_type &aD) {
            const auto base = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {aA, aB, aC, aD});
            indices.insert(indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
        };

        quad({-1.5f, -1, -1.5f}, {1.5f, -1, -1.5f}, {1.5f, -1, 1.5f}, {-1.5f, -1, 1.5f});
        quad({-1.5f, -1, -1.5f}, {-1.5f, 1, -1.5f}, {1.5f, 1, -1.5f}, {1.5f, -1, -1.5f});
        quad({-1.5f, -1, -1.5f}, {-1.5f, -1, 1.5f}, {-1.5f, 1, 1.5f}, {-1.5f, 1, -1.5f});
        quad({1.5f, -1, -1.5f}, {1.5f, 1, -1.5f}, {1.5f, 1, 1.5f}, {1.5f, -1, 1.5f});

        return make_mesh_data(std::move(vertices), std::move(indices));
    }

    template <typename sampler_type>
    [[nodiscard]] heightfield_data_ptr_type make_terrain(const std::size_t aColumns,
        const std::size_t aRows, const sampler_type &aSampler) {
        std::vector<floating_point_type> heights(aColumns * aRows);

        for (std::size_t row = 0; row < aRows; ++row)
            for (std::size_t column = 0; column < aColumns; ++column)
                heights[row * aColumns + column] = aSampler(column, row);

        return make_heightfield_data(aColumns, aRows, std::move(heights));
    }

    [[nodiscard]] inline heightfield_data_ptr_type make_flat_terrain(const std::size_t aColumns,
        const std::size_t aRows) {
        return make_terrain(aColumns, aRows, [](std::size_t, std::size_t) { return 0.0f; });
    }

    inline const_aabb_collider_ptr_type add_static_box_at_origin(const fixture &aFixture) {
        return aFixture.scene->make_static_aabb_collider(matrix4x4_type::identity, {0.5f, 0.5f, 0.5f});
    }
}

#endif
