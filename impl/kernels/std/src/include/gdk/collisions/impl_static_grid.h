// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_STATIC_GRID_H
#define GDK_COLLISIONS_IMPL_STATIC_GRID_H

#include <gdk/collisions/kernels/tables.h>
#include <gdk/collisions/std_kernels.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <unordered_map>
#include <vector>

namespace gdk::collisions {
    struct broadphase_cell_key final {
        int x;
        int y;
        int z;

        bool operator==(const broadphase_cell_key &aOther) const noexcept = default;
    };

    struct broadphase_cell_hasher final {
        std::size_t operator()(const broadphase_cell_key &aKey) const noexcept;
    };

    /// \brief the cell a point falls in, clamped to a range an int can count
    [[nodiscard]] broadphase_cell_key broadphase_cell_coord(const floating_point_type aCellSize,
        const vector3_type &aValue);

    [[nodiscard]] bool bounds_are_finite(const bounds &aBounds);

    [[nodiscard]] bool bounds_overlap(const bounds &aFirst, const bounds &aSecond);

    /// \brief how many cells lie in the box from aMin to aMax inclusive
    [[nodiscard]] std::uint64_t broadphase_cell_span(const broadphase_cell_key &aMin, const broadphase_cell_key &aMax);

    /// \brief the rows a single query has already reported; clearing is constant time
    class impl_row_set final {
    public:
        void clear() {
            ++m_Generation;

            if (m_Generation == 0) {
                std::fill(m_Stamps.begin(), m_Stamps.end(), std::uint32_t{0});
                m_Generation = 1;
            }
        }

        /// \return true if aRow was not already present
        bool insert(const std::uint32_t aRow) {
            if (aRow >= m_Stamps.size()) m_Stamps.resize(static_cast<std::size_t>(aRow) + 1, 0);

            auto &stamp = m_Stamps[aRow];
            if (stamp == m_Generation) return false;

            stamp = m_Generation;
            return true;
        }

    private:
        std::vector<std::uint32_t> m_Stamps;
        std::uint32_t m_Generation = 1;
    };

    /// \brief a uniform spatial hash over the rows of the statics table, edited as rows come and go
    class impl_static_grid final {
    public:
        explicit impl_static_grid(const std_kernels_policy &aPolicy);

        void insert(const std::uint32_t aRow, const bounds &aBounds);

        void remove(const std::uint32_t aRow);

        [[nodiscard]] std::size_t cell_count() const { return m_Cells.size(); }

        /// \brief visit each row sharing a cell with aBounds once; aVisit returns true to stop
        template <typename visit_type>
        void visit_overlapping(const bounds &aBounds, impl_row_set &aSeen, visit_type &&aVisit) const {
            bool stopped = false;

            const auto visit = [&](std::span<const std::uint32_t> aRows) {
                for (const auto row : aRows) {
                    if (stopped) return;
                    if (!aSeen.insert(row)) continue;

                    stopped = aVisit(row);
                }
            };

            for (const auto row : m_Unbinned) {
                if (stopped) return;
                if (!bounds_overlap(m_Placements[row].extent, aBounds) || !aSeen.insert(row)) continue;

                stopped = aVisit(row);
            }

            for_each_cell(aBounds, [&](const std::vector<std::uint32_t> &aRows) {
                visit(aRows);
                return stopped;
            });
        }

        /// \brief visit the rows in each cell a ray crosses, nearest cell first.
        /// aVisit returns the distance of the nearest hit so far; the walk stops once that lies before the next cell
        template <typename visit_type>
        void visit_along_ray(const vector3_type &aOrigin, const vector3_type &aDirection,
            const floating_point_type aMaxDistance, impl_row_set &aSeen, visit_type &&aVisit) const;

    private:
        using cell_key = broadphase_cell_key;

        struct placement final {
            bounds extent;
            cell_key min{0, 0, 0};
            cell_key max{0, 0, 0};
            bool unbinned = false;
            bool present = false;
        };

        [[nodiscard]] cell_key cell_coord(const vector3_type &aValue) const {
            return broadphase_cell_coord(m_CellSize, aValue);
        }

        /// \brief call aCell with each occupied cell within aBounds until it returns true
        template <typename cell_type>
        void for_each_cell(const bounds &aBounds, cell_type &&aCell) const {
            if (!bounds_are_finite(aBounds)) return;

            const auto min = cell_coord(aBounds.min);
            const auto max = cell_coord(aBounds.max);

            if (broadphase_cell_span(min, max) > m_Cells.size()) {
                for (const auto &[key, rows] : m_Cells)
                    if (key.x >= min.x && key.x <= max.x && key.y >= min.y && key.y <= max.y
                        && key.z >= min.z && key.z <= max.z && aCell(rows)) return;

                return;
            }

            for (int z = min.z; z <= max.z; ++z)
            for (int y = min.y; y <= max.y; ++y)
            for (int x = min.x; x <= max.x; ++x)
                if (const auto it = m_Cells.find(cell_key{x, y, z}); it != m_Cells.end() && aCell(it->second)) return;
        }

        std::unordered_map<cell_key, std::vector<std::uint32_t>, broadphase_cell_hasher> m_Cells;

        std::vector<std::uint32_t> m_Unbinned;

        std::vector<placement> m_Placements;

        floating_point_type m_CellSize;
        std::size_t m_MaxCellsPerBody;
    };

    template <typename visit_type>
    void impl_static_grid::visit_along_ray(const vector3_type &aOrigin, const vector3_type &aDirection,
        const floating_point_type aMaxDistance, impl_row_set &aSeen, visit_type &&aVisit) const {
        auto nearest = aMaxDistance;

        const auto visit = [&](std::span<const std::uint32_t> aRows) {
            for (const auto row : aRows)
                if (aSeen.insert(row)) nearest = std::min(nearest, aVisit(row));
        };

        visit(m_Unbinned);

        const auto tip = aOrigin + aDirection * aMaxDistance;

        const bounds reach{vector3_type::min(aOrigin, tip), vector3_type::max(aOrigin, tip)};

        if (!bounds_are_finite(reach)) return;

        const auto first = cell_coord(aOrigin);
        const auto last = cell_coord(tip);

        const auto steps = static_cast<std::uint64_t>(std::abs(static_cast<std::int64_t>(last.x) - first.x))
            + static_cast<std::uint64_t>(std::abs(static_cast<std::int64_t>(last.y) - first.y))
            + static_cast<std::uint64_t>(std::abs(static_cast<std::int64_t>(last.z) - first.z)) + 3;

        if (steps > m_Cells.size()) {
            for_each_cell(reach, [&](const std::vector<std::uint32_t> &aRows) {
                visit(aRows);
                return false;
            });

            return;
        }

        int cell[3] = {first.x, first.y, first.z};
        int step[3];
        floating_point_type next[3];
        floating_point_type delta[3];

        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto direction = aDirection[axis];
            const auto origin = aOrigin[axis];

            if (direction > 0) {
                step[axis] = 1;
                next[axis] = ((static_cast<floating_point_type>(cell[axis]) + 1) * m_CellSize - origin) / direction;
                delta[axis] = m_CellSize / direction;
            }
            else if (direction < 0) {
                step[axis] = -1;
                next[axis] = (static_cast<floating_point_type>(cell[axis]) * m_CellSize - origin) / direction;
                delta[axis] = -m_CellSize / direction;
            }
            else {
                step[axis] = 0;
                next[axis] = std::numeric_limits<floating_point_type>::infinity();
                delta[axis] = std::numeric_limits<floating_point_type>::infinity();
            }
        }

        for (std::uint64_t walked = 0; walked < steps; ++walked) {
            if (const auto it = m_Cells.find(cell_key{cell[0], cell[1], cell[2]}); it != m_Cells.end()) visit(it->second);

            const auto axis = next[0] < next[1] ? (next[0] < next[2] ? 0 : 2) : (next[1] < next[2] ? 1 : 2);

            if (nearest < next[axis] || next[axis] > aMaxDistance) return;

            cell[axis] += step[axis];
            next[axis] += delta[axis];
        }
    }
}

#endif
