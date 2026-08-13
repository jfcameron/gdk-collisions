// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/collider.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_collision_profile.h>
#include <gdk/collisions/box_collider.h>
#include <gdk/collisions/capsule_collider.h>
#include <gdk/collisions/impl_collision_scene.h>
#include <gdk/collisions/impl_mesh_data.h>
#include <gdk/collisions/mesh_collider.h>
#include <gdk/collisions/plane_collider.h>
#include <gdk/collisions/sphere_collider.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <random>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <cmath>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace {
    using namespace gdk::collisions;

    constexpr delta_time_type DELTA_TIME = 1.0f / 60.0f;

    constexpr int WARMUP_FRAMES = 20;
    constexpr int MEASURED_FRAMES = 40;

    enum class ground_kind { plane, per_body_box, single_box, mesh };

    struct configuration final {
        int bodiesPerSide;
        floating_point_type spacing;
        const char *description;

        floating_point_type cellSize = 0;

        std::size_t chunkSize = 0;

        bool spatialOrder = true;

        ground_kind ground = ground_kind::plane;

        bool capsules = false;

        bool shuffled = false;
    };

    [[nodiscard]] impl_collision_policy policy_for(const configuration &aConfiguration) {
        if (aConfiguration.cellSize == 0 && aConfiguration.chunkSize == 0
            && aConfiguration.spatialOrder) return impl_collision_policy{};

        const impl_collision_policy defaults;
        const auto cellSize = aConfiguration.cellSize ? aConfiguration.cellSize : defaults.BROAD_PHASE_CELL_SIZE;
        const auto chunkSize = aConfiguration.chunkSize ? aConfiguration.chunkSize : defaults.PARALLEL_DETECT_CHUNK_SIZE;
        return impl_collision_policy{1, 1e-7f, 8, cellSize, 512, chunkSize,
            aConfiguration.spatialOrder};
    }

    struct phase_averages final {
        double total = 0;
        double rebuild = 0;
        double kinematic = 0;
        double dynamicLoop = 0;
        double triggerLoop = 0;
        double events = 0;

        double gatherQuery = 0;
        double gatherSort = 0;
        double narrowPhase = 0;
        double resolveRespond = 0;

        double bvhQuery = 0;
        double triangleTest = 0;

        double meshQueries = 0;
        double meshCandidates = 0;
        double triangleTests = 0;
        double advancementIterations = 0;
    };

    struct island_summary final {
        std::size_t count = 0;
        std::size_t largest = 0;
        double ceiling = 0;
    };

    struct islands final {
        island_summary byBounds;
        island_summary byCell;
    };

    class components final {
    public:
        explicit components(const std::size_t aCount) : m_Parent(aCount) {
            std::iota(m_Parent.begin(), m_Parent.end(), std::size_t{0});
        }

        std::size_t find(std::size_t aIndex) {
            while (m_Parent[aIndex] != aIndex) {
                m_Parent[aIndex] = m_Parent[m_Parent[aIndex]];
                aIndex = m_Parent[aIndex];
            }
            return aIndex;
        }

        void join(const std::size_t aFirst, const std::size_t aSecond) {
            const auto rootFirst = find(aFirst);
            const auto rootSecond = find(aSecond);
            if (rootFirst != rootSecond) m_Parent[rootFirst] = rootSecond;
        }

        [[nodiscard]] island_summary summarise() {
            std::vector<std::size_t> sizes(m_Parent.size(), 0);
            for (std::size_t i = 0; i < m_Parent.size(); ++i) ++sizes[find(i)];

            island_summary summary;
            for (const auto size : sizes) {
                if (size == 0) continue;
                ++summary.count;
                if (size > summary.largest) summary.largest = size;
            }

            summary.ceiling = summary.largest
                ? static_cast<double>(m_Parent.size()) / summary.largest
                : 0.0;
            return summary;
        }

    private:
        std::vector<std::size_t> m_Parent;
    };

    struct field final {
        scene_ptr_type pScene;

        const_plane_collider_ptr_type pGround;
        std::vector<const_box_collider_ptr_type> groundBoxes;
        const_mesh_collider_ptr_type pGroundMesh;
        mesh_data_ptr_type groundMeshData;

        std::vector<sphere_collider_ptr_type> spheres;
        std::vector<capsule_collider_ptr_type> capsules;

        std::vector<collider_ptr_type> bodies;
    };

    [[nodiscard]] mesh_data_ptr_type make_floor_mesh(const floating_point_type aOriginX,
        const floating_point_type aOriginZ, const floating_point_type aExtent,
        const int aCells) {
        std::vector<vector3_type> vertices;
        std::vector<std::uint32_t> indices;

        const auto step = aExtent / aCells;

        for (int z = 0; z <= aCells; ++z)
            for (int x = 0; x <= aCells; ++x)
                vertices.push_back({aOriginX + x * step, 0.0f, aOriginZ + z * step});

        const auto at = [aCells](const int aX, const int aZ) {
            return static_cast<std::uint32_t>(aZ * (aCells + 1) + aX);
        };

        for (int z = 0; z < aCells; ++z)
            for (int x = 0; x < aCells; ++x) {
                indices.push_back(at(x, z));
                indices.push_back(at(x, z + 1));
                indices.push_back(at(x + 1, z));

                indices.push_back(at(x + 1, z));
                indices.push_back(at(x, z + 1));
                indices.push_back(at(x + 1, z + 1));
            }

        return impl_mesh_data::make(std::move(vertices), std::move(indices));
    }

    void build_ground(field &aField, const configuration &aConfiguration);

    class thread_pool final {
    public:
        explicit thread_pool(const unsigned int aThreads) {
            m_Workers.reserve(aThreads);
            for (unsigned int i = 0; i < aThreads; ++i) m_Workers.emplace_back([this] { worker(); });
        }

        ~thread_pool() {
            {
                const std::lock_guard<std::mutex> lock(m_Mutex);
                m_Stop = true;
                ++m_Generation;
            }

            m_Wake.notify_all();
            for (auto &worker : m_Workers) worker.join();
        }

        void run(const std::size_t aCount, const chunk_type &aChunk) {
            {
                const std::lock_guard<std::mutex> lock(m_Mutex);
                m_Chunk = &aChunk;
                m_Count = aCount;
                m_Next.store(0, std::memory_order_relaxed);
                m_Remaining = m_Workers.size();
                ++m_Generation;
            }

            m_Wake.notify_all();

            drain();

            std::unique_lock<std::mutex> lock(m_Mutex);
            m_Done.wait(lock, [this] { return m_Remaining == 0; });
        }

    private:
        void drain() {
            for (;;) {
                const auto index = m_Next.fetch_add(1, std::memory_order_relaxed);
                if (index >= m_Count) return;
                (*m_Chunk)(index);
            }
        }

        void worker() {
            std::uint64_t seen = 0;

            for (;;) {
                {
                    std::unique_lock<std::mutex> lock(m_Mutex);
                    m_Wake.wait(lock, [&] { return m_Generation != seen; });
                    seen = m_Generation;
                    if (m_Stop) return;
                }

                drain();

                {
                    const std::lock_guard<std::mutex> lock(m_Mutex);
                    if (--m_Remaining == 0) m_Done.notify_one();
                }
            }
        }

        std::vector<std::thread> m_Workers;
        std::mutex m_Mutex;
        std::condition_variable m_Wake;
        std::condition_variable m_Done;

        const chunk_type *m_Chunk = nullptr;
        std::size_t m_Count = 0;
        std::atomic<std::size_t> m_Next{0};
        std::size_t m_Remaining = 0;
        std::uint64_t m_Generation = 0;
        bool m_Stop = false;
    };

    [[nodiscard]] task_dispatcher_type make_pool_dispatcher(
        const std::shared_ptr<thread_pool> &aPool) {
        return [aPool](std::size_t aCount, const chunk_type &aChunk) {
            aPool->run(aCount, aChunk);
        };
    }

    void build_ground(field &aField, const configuration &aConfiguration) {
        const floating_point_type originX = 137.0f;
        const floating_point_type originZ = -211.0f;
        const auto extent = aConfiguration.bodiesPerSide * aConfiguration.spacing;

        switch (aConfiguration.ground) {
            case ground_kind::plane:
                aField.pGround = aField.pScene->make_static_plane_collider(
                    matrix4x4_type::identity);
                break;

            case ground_kind::single_box: {
                const auto halfSpan = extent * 0.5f + 4.0f;

                matrix4x4_type transform;
                transform.set_translation({
                    originX + (aConfiguration.bodiesPerSide - 1) * aConfiguration.spacing * 0.5f,
                    -0.5f,
                    originZ + (aConfiguration.bodiesPerSide - 1) * aConfiguration.spacing * 0.5f});

                aField.groundBoxes.push_back(aField.pScene->make_static_axis_aligned_box_collider(
                    transform, {halfSpan, 0.5f, halfSpan}));
                break;
            }

            case ground_kind::per_body_box: {
                aField.groundBoxes.reserve(
                    static_cast<std::size_t>(aConfiguration.bodiesPerSide) * aConfiguration.bodiesPerSide);

                for (int x = 0; x < aConfiguration.bodiesPerSide; ++x)
                    for (int z = 0; z < aConfiguration.bodiesPerSide; ++z) {
                        matrix4x4_type transform;
                        transform.set_translation({originX + x * aConfiguration.spacing, -0.5f,
                            originZ + z * aConfiguration.spacing});
                        aField.groundBoxes.push_back(
                            aField.pScene->make_static_axis_aligned_box_collider(transform, {0.5f, 0.5f, 0.5f}));
                    }
                break;
            }

            case ground_kind::mesh: {
                aField.groundMeshData = make_floor_mesh(originX - 4.0f, originZ - 4.0f,
                    extent + 8.0f, std::max(aConfiguration.bodiesPerSide, 8));

                aField.pGroundMesh = aField.pScene->make_static_mesh_collider(
                    matrix4x4_type::identity, aField.groundMeshData);
                break;
            }
        }
    }

    [[nodiscard]] field build_field(const configuration &aConfiguration,
        const task_dispatcher_type &aDispatcher = {}) {
        field result;
        result.pScene = impl_collision_scene::make(nullptr, nullptr, policy_for(aConfiguration), aDispatcher);

        build_ground(result, aConfiguration);

        const auto side = aConfiguration.bodiesPerSide;
        const auto spacing = aConfiguration.spacing;

        const floating_point_type originX = 137.0f;
        const floating_point_type originZ = -211.0f;

        result.bodies.reserve(static_cast<std::size_t>(side) * side);

        std::vector<vector3_type> positions;
        positions.reserve(static_cast<std::size_t>(side) * side);

        for (int x = 0; x < side; ++x)
            for (int z = 0; z < side; ++z)
                positions.push_back({originX + x * spacing, 0.5f, originZ + z * spacing});

        if (aConfiguration.shuffled) {
            std::mt19937 engine(20260804u);
            std::shuffle(positions.begin(), positions.end(), engine);
        }

        for (const auto &position : positions) {
            if (aConfiguration.capsules) {
                auto pCapsule = result.pScene->make_capsule_collider();
                pCapsule->set_radius(0.5f);
                pCapsule->set_half_height(0.5f);
                pCapsule->set_position({position.x, position.y + 0.5f, position.z});
                pCapsule->set_inverse_overlap_weight(1.0f);
                result.capsules.push_back(pCapsule);
                result.bodies.push_back(pCapsule);
            }
            else {
                auto pSphere = result.pScene->make_sphere_collider();
                pSphere->set_position(position);
                pSphere->set_inverse_overlap_weight(1.0f);
                result.spheres.push_back(pSphere);
                result.bodies.push_back(pSphere);
            }
        }

        return result;
    }

    void drive(field &aField) {
        for (auto &pBody : aField.bodies) pBody->add_velocity({0.0f, -4.0f, 0.0f});
        aField.pScene->update(DELTA_TIME);
    }

    [[nodiscard]] bool sanity_check(const field &aField) {
        if (aField.bodies.empty()) return false;

        for (const auto &pBody : aField.bodies)
            if (pBody->transform().translation().y < -0.5f) return false;

        return true;
    }

    [[nodiscard]] islands measure_islands(const field &aField) {
        std::vector<impl_collider::broadphase_bounds> bounds;
        bounds.reserve(aField.bodies.size());

        for (const auto &pSphere : aField.bodies) {
            const auto pImpl = std::dynamic_pointer_cast<impl_collider>(pSphere);
            if (!pImpl) {
                std::fprintf(stderr, "island analysis: collider is not an impl_collider\n");
                std::exit(1);
            }
            bounds.push_back(pImpl->broad_phase_swept_bounds(DELTA_TIME));
        }

        const auto n = bounds.size();

        const auto overlaps = [](const impl_collider::broadphase_bounds &a,
            const impl_collider::broadphase_bounds &b) {
            return a.min.x <= b.max.x && a.max.x >= b.min.x
                && a.min.y <= b.max.y && a.max.y >= b.min.y
                && a.min.z <= b.max.z && a.max.z >= b.min.z;
        };

        components byBounds(n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j)
                if (overlaps(bounds[i], bounds[j])) byBounds.join(i, j);

        const impl_collision_policy policy;
        const auto coordinate = [&policy](const floating_point_type aValue) {
            return static_cast<long long>(std::floor(aValue / policy.BROAD_PHASE_CELL_SIZE));
        };

        components byCell(n);
        std::unordered_map<long long, std::size_t> firstBodyInCell;

        for (std::size_t i = 0; i < n; ++i) {
            for (auto x = coordinate(bounds[i].min.x); x <= coordinate(bounds[i].max.x); ++x)
            for (auto y = coordinate(bounds[i].min.y); y <= coordinate(bounds[i].max.y); ++y)
            for (auto z = coordinate(bounds[i].min.z); z <= coordinate(bounds[i].max.z); ++z) {
                const auto key = (x * 73856093LL) ^ (y * 19349663LL) ^ (z * 83492791LL);
                const auto found = firstBodyInCell.find(key);
                if (found == firstBodyInCell.end()) firstBodyInCell.emplace(key, i);
                else byCell.join(i, found->second);
            }
        }

        return islands{byBounds.summarise(), byCell.summarise()};
    }

    [[nodiscard]] phase_averages measure_phases(field &aField) {
        for (int frame = 0; frame < WARMUP_FRAMES; ++frame) drive(aField);

        if (!sanity_check(aField)) {
            std::fprintf(stderr, "the measured scene is not the intended one -- bodies fell through "
                "the ground, so a collider handle was dropped somewhere\n");
            std::exit(1);
        }

#ifdef GDK_COLLISION_PROFILE
        profile::reset();
#endif

        const auto wallStart = std::chrono::steady_clock::now();
        for (int frame = 0; frame < MEASURED_FRAMES; ++frame) drive(aField);
        const auto wallMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - wallStart).count();

        phase_averages averages;
        averages.total = wallMilliseconds / MEASURED_FRAMES;

#ifdef GDK_COLLISION_PROFILE
        const auto &accumulated = profile::accumulated();
        const double frames = accumulated.steps ? static_cast<double>(accumulated.steps) : 1.0;
        averages.total = accumulated.total_ms / frames;
        averages.rebuild = accumulated.broadphase_rebuild_ms / frames;
        averages.kinematic = accumulated.kinematic_pass_ms / frames;
        averages.dynamicLoop = accumulated.dynamic_loop_ms / frames;
        averages.triggerLoop = accumulated.trigger_loop_ms / frames;
        averages.events = accumulated.events_ms / frames;
        averages.gatherQuery = accumulated.gather_query_ms / frames;
        averages.gatherSort = accumulated.gather_sort_ms / frames;
        averages.narrowPhase = accumulated.narrow_phase_ms / frames;
        averages.resolveRespond = accumulated.resolve_respond_ms / frames;
        averages.bvhQuery = accumulated.bvh_query_ms / frames;
        averages.triangleTest = accumulated.triangle_test_ms / frames;
        averages.meshQueries = static_cast<double>(accumulated.mesh_queries) / frames;
        averages.meshCandidates = static_cast<double>(accumulated.mesh_candidates) / frames;
        averages.triangleTests = static_cast<double>(accumulated.triangle_tests) / frames;
        averages.advancementIterations = static_cast<double>(accumulated.advancement_iterations) / frames;
#endif

        return averages;
    }
}

int main() {
    const configuration configurations[] = {
        {32, 3.00f, "sparse"},
        {96, 3.00f, "sparse"},
        {32, 1.02f, "near"},
        {96, 1.02f, "near"},
        {32, 1.00f, "touching"},
        {96, 1.00f, "touching"},
    };

#ifndef GDK_COLLISION_PROFILE
    std::printf("built without GDK_COLLISION_PROFILE: only the total column is measured\n\n");
#endif

    std::printf("| bodies | spacing | total ms | grid rebuild | dynamic loop | events "
        "| .. gather | .. sort | .. narrow | .. rest |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n");

    std::vector<islands> measured;

    for (const auto &configuration : configurations) {
        auto scene = build_field(configuration);
        const auto averages = measure_phases(scene);
        measured.push_back(measure_islands(scene));

        std::printf("| %d | %.2f %s | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f |\n",
            configuration.bodiesPerSide * configuration.bodiesPerSide,
            configuration.spacing, configuration.description,
            averages.total, averages.rebuild, averages.dynamicLoop, averages.events,
            averages.gatherQuery, averages.gatherSort, averages.narrowPhase,
            averages.dynamicLoop - averages.gatherQuery - averages.gatherSort - averages.narrowPhase);
    }

    std::printf("\n| scene | 1 thread | 4 | 8 | scaling | rebuild | loop | narrow | events |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n");

    struct scene_case final { const char *name; ground_kind ground; bool capsules; };

    std::vector<std::pair<const char *, phase_averages>> details;

    const scene_case scenes[] = {
        {"sphere on plane (all prior numbers)", ground_kind::plane, false},
        {"sphere on one field-sized box", ground_kind::single_box, false},
        {"sphere on per-body boxes", ground_kind::per_body_box, false},
        {"sphere on mesh floor", ground_kind::mesh, false},
        {"capsule on one field-sized box", ground_kind::single_box, true},
        {"capsule on mesh floor", ground_kind::mesh, true},
    };

    for (const auto &scene : scenes) {
        configuration sweep{96, 3.00f, "sparse"};
        sweep.ground = scene.ground;
        sweep.capsules = scene.capsules;

        double serial = 0;
        double best = 0;
        phase_averages detail;

        std::printf("| %s ", scene.name);

        for (const unsigned int threads : {0u, 3u, 7u}) {
            const auto pool = std::make_shared<thread_pool>(threads);
            auto field = build_field(sweep, make_pool_dispatcher(pool));
            const auto averages = measure_phases(field);

            if (threads == 0) { serial = averages.total; detail = averages; }
            best = averages.total;

            std::printf("| %.1f ", averages.total);
        }

        std::printf("| %.2fx | %.1f | %.1f | %.1f | %.1f |\n",
            best > 0 ? serial / best : 0.0,
            detail.rebuild, detail.dynamicLoop, detail.narrowPhase, detail.events);

        details.push_back({scene.name, detail});
    }

    std::printf("\n| scene | narrow ms | bvh query | triangle tests ms "
        "| queries | candidates | tri tests | advance iters |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- |\n");

    for (const auto &entry : details) {
        const auto &d = entry.second;
        std::printf("| %s | %.1f | %.1f | %.1f | %.0f | %.0f | %.0f | %.0f |\n",
            entry.first, d.narrowPhase, d.bvhQuery, d.triangleTest,
            d.meshQueries, d.meshCandidates, d.triangleTests, d.advancementIterations);
    }

    std::printf("\n| bodies | spacing | body order | detect | total ms | rebuild | loop | gather |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- |\n");

    for (const auto spacing : {3.00f, 1.02f}) {
        for (const bool shuffled : {false, true}) {
            for (const bool spatial : {false, true}) {
                configuration sweep{96, spacing, spacing > 2.0f ? "sparse" : "near"};
                sweep.spatialOrder = spatial;
                sweep.shuffled = shuffled;
                auto scene = build_field(sweep);
                const auto averages = measure_phases(scene);

                std::printf("| %d | %.2f %s | %s | %s | %.1f | %.1f | %.1f | %.1f |\n",
                    sweep.bodiesPerSide * sweep.bodiesPerSide, sweep.spacing, sweep.description,
                    shuffled ? "shuffled" : "spatial at creation",
                    spatial ? "sorted" : "as created",
                    averages.total, averages.rebuild, averages.dynamicLoop, averages.gatherQuery);
            }
        }
    }

    std::printf("\n| bodies | spacing | 1 thread | 2 | 4 | 8 | measured | ceiling |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- |\n");

    for (const auto &configuration : configurations) {
        if (configuration.bodiesPerSide < 96) continue;

        double serial = 0;
        double best = 0;
        double ceiling = 0;
        std::printf("| %d | %.2f %s ", configuration.bodiesPerSide * configuration.bodiesPerSide,
            configuration.spacing, configuration.description);

        for (const unsigned int threads : {0u, 1u, 3u, 7u}) {
            const auto pool = std::make_shared<thread_pool>(threads);
            auto scene = build_field(configuration, make_pool_dispatcher(pool));
            const auto averages = measure_phases(scene);

            if (threads == 0) {
                serial = averages.total;

                const auto parallel = averages.gatherQuery + averages.gatherSort
                    + averages.narrowPhase + averages.rebuild;
                ceiling = averages.total > parallel ? averages.total / (averages.total - parallel) : 0.0;
            }

            best = averages.total;
            std::printf("| %.1f ", averages.total);
        }

        std::printf("| %.2fx | %.2fx |\n", best > 0 ? serial / best : 0.0, ceiling);
    }
    
    std::printf("\n| bodies | spacing | total ms | pure | serial | unaccounted "
        "| loop only | phase batch | islands |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n");

    for (const auto &configuration : configurations) {
        auto scene = build_field(configuration);
        const auto averages = measure_phases(scene);

        const auto pure = averages.rebuild + averages.gatherQuery + averages.gatherSort
            + averages.narrowPhase + averages.events;

        const auto serial = averages.resolveRespond;

        const auto unaccounted = averages.total - pure - serial;

        const auto ceiling = [&](const double aSerial) {
            return aSerial > 0 ? averages.total / aSerial : 0.0;
        };

        std::printf("| %d | %.2f %s | %.1f | %.1f | %.1f | %.1f | %.1fx | %.1fx | %.1fx |\n",
            configuration.bodiesPerSide * configuration.bodiesPerSide,
            configuration.spacing, configuration.description,
            averages.total, pure, serial, unaccounted,
            ceiling(averages.total - averages.dynamicLoop),
            ceiling(serial + unaccounted),
            ceiling(unaccounted));
    }

    std::printf("\n| bodies | spacing | cell size | total ms | rebuild | loop | gather | narrow |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- |\n");

    const floating_point_type cellSizes[] = {0.5f, 1.0f, 2.0f, 4.0f, 8.0f};

    for (const auto spacing : {3.00f, 1.02f}) {
        for (const auto cellSize : cellSizes) {
            configuration sweep{96, spacing, spacing > 2.0f ? "sparse" : "near", cellSize};
            auto scene = build_field(sweep);
            const auto averages = measure_phases(scene);

            std::printf("| %d | %.2f %s | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f |\n",
                sweep.bodiesPerSide * sweep.bodiesPerSide, sweep.spacing, sweep.description,
                cellSize, averages.total, averages.rebuild, averages.dynamicLoop,
                averages.gatherQuery, averages.narrowPhase);
        }
    }

    std::printf("\n| bodies | spacing | islands (bounds) | largest | ceiling "
        "| islands (cell) | largest | ceiling |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- |\n");

    for (std::size_t i = 0; i < measured.size(); ++i)
        std::printf("| %d | %.2f %s | %zu | %zu | %.1fx | %zu | %zu | %.1fx |\n",
            configurations[i].bodiesPerSide * configurations[i].bodiesPerSide,
            configurations[i].spacing, configurations[i].description,
            measured[i].byBounds.count, measured[i].byBounds.largest, measured[i].byBounds.ceiling,
            measured[i].byCell.count, measured[i].byCell.largest, measured[i].byCell.ceiling);

    return 0;
}
