// © Joseph Cameron - All Rights Reserved

#include <jfc/catch.hpp>

#include <gdk/collider.h>
#include <gdk/collision_scene.h>
#include <gdk/collision_types.h>
#include <gdk/impl_collision_policy.h>
#include <gdk/impl_collision_scene.h>
#include <gdk/raycast_hit.h>
#include <gdk/sphere_collider.h>

#include <algorithm>
#include <numeric>
#include <thread>
#include <vector>

using namespace gdk;

namespace {
    constexpr collision_delta_time_type DELTA_TIME = 1.0f / 60.0f;
    constexpr int FRAMES = 40;

    struct world final {
        collision_scene_ptr_type scene;
        const_plane_collider_ptr_type ground;
        std::vector<sphere_collider_ptr_type> bodies;
    };

    [[nodiscard]] world make_world(const collision_task_dispatcher_type &aDispatcher) {
        world result;
        result.scene = impl_collision_scene::make(nullptr, nullptr, impl_collision_policy{}, aDispatcher);
        result.ground = result.scene->make_static_plane_collider(collision_matrix4x4_type::identity);

        for (int x = 0; x < 14; ++x) {
            for (int z = 0; z < 14; ++z) {
                auto pSphere = result.scene->make_sphere_collider();
                pSphere->set_position({37.0f + x * 0.85f, 0.5f + (x % 3) * 0.4f, -19.0f + z * 0.85f});
                pSphere->set_inverse_overlap_weight(1.0f);
                result.bodies.push_back(pSphere);
            }
        }

        return result;
    }

    [[nodiscard]] std::vector<collision_vector3_type> run(const collision_task_dispatcher_type &aDispatcher) {
        auto w = make_world(aDispatcher);

        for (int frame = 0; frame < FRAMES; ++frame) {
            for (auto &pBody : w.bodies) pBody->add_velocity({0.35f, -4.0f, 0.2f});
            w.scene->update(DELTA_TIME);
        }

        std::vector<collision_vector3_type> positions;
        positions.reserve(w.bodies.size());
        for (const auto &pBody : w.bodies) positions.push_back(pBody->transform().translation());
        return positions;
    }

    [[nodiscard]] bool identical(const std::vector<collision_vector3_type> &aFirst,
        const std::vector<collision_vector3_type> &aSecond) {
        if (aFirst.size() != aSecond.size()) return false;

        for (std::size_t i = 0; i < aFirst.size(); ++i)
            if (aFirst[i].x != aSecond[i].x || aFirst[i].y != aSecond[i].y || aFirst[i].z != aSecond[i].z)
                return false;

        return true;
    }

    [[nodiscard]] bool moved(const std::vector<collision_vector3_type> &aPositions) {
        return !aPositions.empty()
            && std::any_of(aPositions.begin(), aPositions.end(),
                [](const collision_vector3_type &aPosition) { return aPosition.x != 37.0f; });
    }
}

TEST_CASE("gdk::collision_scene gives the same result with a dispatcher as without one",
    "[gdk::collision]") {
    const auto serial = run({});

    const auto dispatched = run([](std::size_t aCount, const collision_chunk_type &aChunk) {
        for (std::size_t i = 0; i < aCount; ++i) aChunk(i);
    });

    REQUIRE(moved(serial));
    REQUIRE(identical(dispatched, serial));
}

TEST_CASE("gdk::collision_scene is unaffected by the order chunks run in", "[gdk::collision]") {
    const auto serial = run({});
    REQUIRE(moved(serial));

    SECTION("reversed") {
        const auto reversed = run([](std::size_t aCount, const collision_chunk_type &aChunk) {
            for (std::size_t i = aCount; i > 0; --i) aChunk(i - 1);
        });

        REQUIRE(identical(reversed, serial));
    }

    SECTION("odd chunks before even ones") {
        const auto interleaved = run([](std::size_t aCount, const collision_chunk_type &aChunk) {
            for (std::size_t i = 1; i < aCount; i += 2) aChunk(i);
            for (std::size_t i = 0; i < aCount; i += 2) aChunk(i);
        });

        REQUIRE(identical(interleaved, serial));
    }
}

TEST_CASE("gdk::collision_scene answers raycasts from several threads at once", "[gdk::collision]") {
    auto w = make_world({});

    std::vector<const_sphere_collider_ptr_type> targets;
    for (int i = 0; i < 24; ++i) {
        collision_matrix4x4_type transform;
        transform.set_translation({-40.0f, 2.0f + i * 3.0f, 0.0f});
        targets.push_back(w.scene->make_static_sphere_collider(transform, 0.5f));
    }

    const auto cast = [&w](const int aIndex) {
        return w.scene->raycast({-60.0f, 2.0f + aIndex * 3.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 100.0f);
    };

    std::vector<float> serial;
    for (int i = 0; i < 24; ++i) {
        const auto hit = cast(i);
        REQUIRE(hit);
        serial.push_back(hit->distance);
    }

    REQUIRE(serial.size() == 24);

    for (int attempt = 0; attempt < 8; ++attempt) {
        std::vector<float> concurrent(24, -1.0f);
        std::vector<std::thread> workers;

        for (int i = 0; i < 24; ++i)
            workers.emplace_back([&, i] {
                const auto hit = cast(i);
                concurrent[i] = hit ? hit->distance : -1.0f;
            });

        for (auto &worker : workers) worker.join();

        REQUIRE(concurrent == serial);
    }
}

TEST_CASE("gdk::collision_scene gives the same result on several threads", "[gdk::collision]") {
    const auto serial = run({});
    REQUIRE(moved(serial));

    const auto threaded = [](std::size_t aCount, const collision_chunk_type &aChunk) {
        std::vector<std::thread> workers;
        workers.reserve(aCount);

        for (std::size_t i = 0; i < aCount; ++i) workers.emplace_back([&aChunk, i] { aChunk(i); });
        for (auto &worker : workers) worker.join();
    };

    for (int attempt = 0; attempt < 8; ++attempt) REQUIRE(identical(run(threaded), serial));
}
