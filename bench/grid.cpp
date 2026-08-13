// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/collider.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/impl_broadphase_grid.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_collision_scene.h>
#include <gdk/collisions/impl_dynamic_broadphase_grid.h>
#include <gdk/collisions/sphere_collider.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <numeric>
#include <random>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {
    using namespace gdk::collisions;

    constexpr delta_time_type DELTA_TIME = 1.0f / 60.0f;
    constexpr int MEASURED_FRAMES = 40;

    struct configuration final {
        int bodiesPerSide;
        floating_point_type spacing;
        const char *description;
    };

    struct rebuild_costs final {
        double clear = 0;
        double bounds = 0;
        double insert = 0;

        std::size_t cellInserts = 0;
        std::size_t occupiedCells = 0;
    };

    using clock_type = std::chrono::steady_clock;

    [[nodiscard]] double milliseconds_since(const clock_type::time_point aStart) {
        return std::chrono::duration<double, std::milli>(clock_type::now() - aStart).count();
    }

    struct field final {
        scene_ptr_type pScene;
        std::vector<sphere_collider_ptr_type> handles;
        std::vector<impl_collider_ptr_type> bodies;
    };

    [[nodiscard]] field build_field(const configuration &aConfiguration) {
        field result;
        result.pScene = impl_collision_scene::make(nullptr, nullptr, impl_collision_policy{});

        const auto side = aConfiguration.bodiesPerSide;
        const auto spacing = aConfiguration.spacing;

        const floating_point_type originX = 137.0f;
        const floating_point_type originZ = -211.0f;

        for (int x = 0; x < side; ++x) {
            for (int z = 0; z < side; ++z) {
                auto pSphere = result.pScene->make_sphere_collider();
                pSphere->set_position({originX + x * spacing, 0.5f, originZ + z * spacing});
                pSphere->set_velocity({0.0f, -4.0f, 0.0f});

                const auto pImpl = std::dynamic_pointer_cast<impl_collider>(pSphere);
                if (!pImpl) {
                    std::fprintf(stderr, "collider is not an impl_collider\n");
                    std::exit(1);
                }

                result.handles.push_back(pSphere);
                result.bodies.push_back(pImpl);
            }
        }

        return result;
    }

    /// \brief Morton (Z-order) code of a body's cell, for sorting bodies into spatial order.
    [[nodiscard]] std::uint64_t morton_of(const impl_collision_policy &aPolicy,
        const impl_collider::broadphase_bounds &aBounds) {
        const auto cell = broadphase_cell_coord(aPolicy, aBounds.min);

        const auto spread = [](std::uint64_t aValue) {
            aValue &= 0x1fffff;
            aValue = (aValue | (aValue << 32)) & 0x1f00000000ffffull;
            aValue = (aValue | (aValue << 16)) & 0x1f0000ff0000ffull;
            aValue = (aValue | (aValue <<  8)) & 0x100f00f00f00f00full;
            aValue = (aValue | (aValue <<  4)) & 0x10c30c30c30c30c3ull;
            aValue = (aValue | (aValue <<  2)) & 0x1249249249249249ull;
            return aValue;
        };

        const auto bias = [](const int aValue) {
            return static_cast<std::uint64_t>(static_cast<std::int64_t>(aValue) + (1 << 20));
        };

        return spread(bias(cell.x)) | (spread(bias(cell.y)) << 1) | (spread(bias(cell.z)) << 2);
    }

    enum class body_order { creation, spatial, shuffled };

    [[nodiscard]] std::vector<impl_collider_ptr_type> ordered_bodies(const field &aField,
        const body_order aOrder) {
        auto bodies = aField.bodies;
        if (aOrder == body_order::creation) return bodies;

        if (aOrder == body_order::shuffled) {
            std::mt19937 engine(20260804u);
            std::shuffle(bodies.begin(), bodies.end(), engine);
            return bodies;
        }

        const impl_collision_policy policy;
        std::sort(bodies.begin(), bodies.end(),
            [&policy](const impl_collider_ptr_type &aFirst, const impl_collider_ptr_type &aSecond) {
                return morton_of(policy, aFirst->broad_phase_swept_bounds(DELTA_TIME))
                    < morton_of(policy, aSecond->broad_phase_swept_bounds(DELTA_TIME));
            });

        return bodies;
    }

    struct access_probe final {
        double milliseconds = 0;
        std::size_t distinctSlots = 0;
        std::size_t tableSlots = 0;
        std::uint64_t checksum = 0;
    };

    [[nodiscard]] std::vector<broadphase_cell_key> all_cells(const field &aField) {
        const impl_collision_policy policy;
        std::vector<broadphase_cell_key> cells;

        for (const auto &pBody : aField.bodies) {
            const auto bounds = pBody->broad_phase_swept_bounds(DELTA_TIME);
            const auto min = broadphase_cell_coord(policy, bounds.min);
            const auto max = broadphase_cell_coord(policy, bounds.max);

            for (int z = min.z; z <= max.z; ++z)
            for (int y = min.y; y <= max.y; ++y)
            for (int x = min.x; x <= max.x; ++x)
                cells.push_back(broadphase_cell_key{x, y, z});
        }

        return cells;
    }

    constexpr int BLOCK_BITS = 2;
    constexpr int BLOCK_SIDE = 1 << BLOCK_BITS;
    constexpr std::size_t CELLS_PER_BLOCK = BLOCK_SIDE * BLOCK_SIDE * BLOCK_SIDE;

    [[nodiscard]] access_probe probe_access(const field &aField, const bool aBlockCoherent) {
        const impl_collision_policy policy;
        const auto cells = all_cells(aField);

        std::unordered_set<std::uint64_t> distinctCells;
        for (const auto &cell : cells)
            distinctCells.insert(broadphase_cell_hasher{}(cell));

        auto capacity = std::size_t{64};
        while (capacity < distinctCells.size() * 2) capacity <<= 1;

        auto blockCapacity = std::size_t{64};
        while (blockCapacity * CELLS_PER_BLOCK < capacity) blockCapacity <<= 1;

        const auto floor_div = [](const int aValue) {
            return aValue >= 0 ? (aValue >> BLOCK_BITS) : -(((-aValue) + BLOCK_SIDE - 1) >> BLOCK_BITS);
        };

        const auto index_of = [&](const broadphase_cell_key &aCell) -> std::size_t {
            if (!aBlockCoherent) return broadphase_cell_hasher{}(aCell) & (capacity - 1);

            const broadphase_cell_key block{floor_div(aCell.x), floor_div(aCell.y), floor_div(aCell.z)};
            const auto slot = broadphase_cell_hasher{}(block) & (blockCapacity - 1);

            const auto local = static_cast<std::size_t>(aCell.x - block.x * BLOCK_SIDE)
                | (static_cast<std::size_t>(aCell.y - block.y * BLOCK_SIDE) << BLOCK_BITS)
                | (static_cast<std::size_t>(aCell.z - block.z * BLOCK_SIDE) << (BLOCK_BITS * 2));

            return slot * CELLS_PER_BLOCK + local;
        };

        struct probe_slot final { std::uint32_t words[6]; };
        static_assert(sizeof(probe_slot) == 24, "probe slot must match the real slot's size");

        std::vector<probe_slot> slots(aBlockCoherent ? blockCapacity * CELLS_PER_BLOCK : capacity,
            probe_slot{});

        access_probe result;
        result.tableSlots = slots.size();

        std::unordered_set<std::size_t> touched;
        for (const auto &cell : cells) touched.insert(index_of(cell));
        result.distinctSlots = touched.size();

        for (const auto &cell : cells) ++slots[index_of(cell)].words[0];

        const auto start = clock_type::now();
        for (int frame = 0; frame < MEASURED_FRAMES; ++frame)
            for (const auto &cell : cells) ++slots[index_of(cell)].words[0];
        result.milliseconds = milliseconds_since(start) / MEASURED_FRAMES;

        for (const auto &slot : slots) result.checksum += slot.words[0];
        return result;
    }

    [[nodiscard]] rebuild_costs measure_node_grid(const field &aField) {
        const impl_collision_policy policy;
        impl_broadphase_grid grid(policy);

        for (const auto &pBody : aField.bodies)
            grid.insert(pBody, pBody->broad_phase_swept_bounds(DELTA_TIME),
                impl_broadphase_grid::body_kind::collider, false);

        rebuild_costs costs;

        std::vector<impl_collider::broadphase_bounds> bounds;
        bounds.reserve(aField.bodies.size());

        for (int frame = 0; frame < MEASURED_FRAMES; ++frame) {
            const auto clearStart = clock_type::now();
            grid.clear();
            grid.reserve(aField.bodies.size());
            costs.clear += milliseconds_since(clearStart);

            bounds.clear();
            const auto boundsStart = clock_type::now();
            for (const auto &pBody : aField.bodies)
                bounds.push_back(pBody->broad_phase_swept_bounds(DELTA_TIME));
            costs.bounds += milliseconds_since(boundsStart);

            const auto insertStart = clock_type::now();
            for (std::size_t i = 0; i < aField.bodies.size(); ++i)
                grid.insert(aField.bodies[i], bounds[i],
                    impl_broadphase_grid::body_kind::collider, false);
            costs.insert += milliseconds_since(insertStart);
        }

        costs.clear /= MEASURED_FRAMES;
        costs.bounds /= MEASURED_FRAMES;
        costs.insert /= MEASURED_FRAMES;
        return costs;
    }

    [[nodiscard]] rebuild_costs measure_flat_grid(const field &aField,
        const body_order aOrder = body_order::creation) {
        const impl_collision_policy policy;
        impl_dynamic_broadphase_grid grid(policy);

        const auto bodies = ordered_bodies(aField, aOrder);

        rebuild_costs costs;

        std::vector<impl_collider::broadphase_bounds> bounds;
        bounds.reserve(aField.bodies.size());

        for (const auto &pBody : bodies)
            grid.add(pBody, pBody->broad_phase_swept_bounds(DELTA_TIME),
                impl_dynamic_broadphase_grid::body_kind::collider);
        grid.build();

        for (int frame = 0; frame < MEASURED_FRAMES; ++frame) {
            const auto clearStart = clock_type::now();
            grid.clear();
            grid.reserve(aField.bodies.size());
            costs.clear += milliseconds_since(clearStart);

            bounds.clear();
            const auto boundsStart = clock_type::now();
            for (const auto &pBody : bodies)
                bounds.push_back(pBody->broad_phase_swept_bounds(DELTA_TIME));
            costs.bounds += milliseconds_since(boundsStart);

            const auto insertStart = clock_type::now();
            for (std::size_t i = 0; i < bodies.size(); ++i)
                grid.add(bodies[i], bounds[i], impl_dynamic_broadphase_grid::body_kind::collider);
            grid.build();
            costs.insert += milliseconds_since(insertStart);
        }

        costs.clear /= MEASURED_FRAMES;
        costs.bounds /= MEASURED_FRAMES;
        costs.insert /= MEASURED_FRAMES;
        return costs;
    }

    [[nodiscard]] double measure_queries(const field &aField, const body_order aOrder) {
        const impl_collision_policy policy;
        impl_dynamic_broadphase_grid grid(policy);

        const auto bodies = ordered_bodies(aField, aOrder);

        for (const auto &pBody : bodies)
            grid.add(pBody, pBody->broad_phase_swept_bounds(DELTA_TIME),
                impl_dynamic_broadphase_grid::body_kind::collider);
        grid.build();

        collider_id_set seen;
        std::vector<impl_broadphase_grid::neighbour> neighbours;

        for (const auto &pBody : bodies) {
            seen.clear();
            neighbours.clear();
            grid.gather_neighbours(*pBody, DELTA_TIME, seen, neighbours);
        }

        double total = 0;
        std::size_t found = 0;

        for (int frame = 0; frame < MEASURED_FRAMES; ++frame) {
            const auto start = clock_type::now();

            for (const auto &pBody : bodies) {
                seen.clear();
                neighbours.clear();
                grid.gather_neighbours(*pBody, DELTA_TIME, seen, neighbours);
                found += neighbours.size();
            }

            total += milliseconds_since(start);
        }

        if (found == 0 && aField.bodies.size() > 1000)
            std::printf("  (note: no neighbours found -- probe cost only)\n");

        return total / MEASURED_FRAMES;
    }

    [[nodiscard]] rebuild_costs measure_cells(const field &aField) {
        const impl_collision_policy policy;
        rebuild_costs costs;

        std::vector<impl_collider::broadphase_bounds> bounds;
        for (const auto &pBody : aField.bodies)
            bounds.push_back(pBody->broad_phase_swept_bounds(DELTA_TIME));

        const auto coordinate = [&policy](const floating_point_type aValue) {
            return static_cast<long long>(std::floor(aValue / policy.BROAD_PHASE_CELL_SIZE));
        };

        std::unordered_set<long long> occupied;

        for (const auto &bound : bounds) {
            for (auto x = coordinate(bound.min.x); x <= coordinate(bound.max.x); ++x)
            for (auto y = coordinate(bound.min.y); y <= coordinate(bound.max.y); ++y)
            for (auto z = coordinate(bound.min.z); z <= coordinate(bound.max.z); ++z) {
                ++costs.cellInserts;
                occupied.insert((x * 73856093LL) ^ (y * 19349663LL) ^ (z * 83492791LL));
            }
        }

        costs.occupiedCells = occupied.size();
        return costs;
    }
}

int main() {
    const configuration configurations[] = {
        {32, 3.00f, "sparse"},
        {96, 3.00f, "sparse"},
        {32, 1.02f, "dense"},
        {96, 1.02f, "dense"},
    };

    std::printf("| bodies | spacing | addressing | ms | distinct slots | table MB |\n");
    std::printf("| --- | --- | --- | --- | --- | --- |\n");

    for (const auto &configuration : configurations) {
        const auto scene = build_field(configuration);

        for (const bool coherent : {false, true}) {
            const auto probe = probe_access(scene, coherent);

            std::printf("| %zu | %.2f %s | %s | %.2f | %zu | %.1f |\n",
                scene.bodies.size(), configuration.spacing, configuration.description,
                coherent ? "block coherent" : "hashed (current)",
                probe.milliseconds, probe.distinctSlots,
                static_cast<double>(probe.tableSlots) * 24.0 / (1024.0 * 1024.0));
        }
    }

    std::printf("\n| bodies | spacing | order | rebuild ms | query ms |\n");
    std::printf("| --- | --- | --- | --- | --- |\n");

    for (const auto &configuration : configurations) {
        const auto scene = build_field(configuration);

        const std::pair<const char *, body_order> orders[] = {
            {"creation (already spatial here)", body_order::creation},
            {"morton", body_order::spatial},
            {"shuffled", body_order::shuffled},
        };

        for (const auto &order : orders) {
            const auto costs = measure_flat_grid(scene, order.second);

            std::printf("| %zu | %.2f %s | %s | %.1f | %.1f |\n",
                scene.bodies.size(), configuration.spacing, configuration.description, order.first,
                costs.clear + costs.bounds + costs.insert,
                measure_queries(scene, order.second));
        }
    }

    std::printf("\n| bodies | spacing | grid | clear ms | bounds ms | bin ms | rebuild ms "
        "| cell inserts | occupied cells | ns/cell insert |\n");
    std::printf("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n");

    for (const auto &configuration : configurations) {
        const auto scene = build_field(configuration);
        const auto cells = measure_cells(scene);

        const std::pair<const char *, rebuild_costs> runs[] = {
            {"node (was)", measure_node_grid(scene)},
            {"flat (is)", measure_flat_grid(scene)},
        };

        for (const auto &run : runs) {
            const auto &costs = run.second;
            const auto total = costs.clear + costs.bounds + costs.insert;

            std::printf("| %zu | %.2f %s | %s | %.1f | %.1f | %.1f | %.1f | %zu | %zu | %.0f |\n",
                scene.bodies.size(), configuration.spacing, configuration.description, run.first,
                costs.clear, costs.bounds, costs.insert, total,
                cells.cellInserts, cells.occupiedCells,
                costs.insert * 1e6 / static_cast<double>(cells.cellInserts));
        }
    }

    return 0;
}
