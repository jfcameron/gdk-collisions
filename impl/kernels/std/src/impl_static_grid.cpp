// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_static_grid.h>

#include <jfc/hash.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace gdk::collisions;

namespace {
    constexpr floating_point_type CELL_LIMIT = 1 << 30;

    void erase_row(std::vector<std::uint32_t> &aRows, const std::uint32_t aRow) {
        if (const auto it = std::find(aRows.begin(), aRows.end(), aRow); it != aRows.end()) {
            *it = aRows.back();
            aRows.pop_back();
        }
    }
}

std::size_t broadphase_cell_hasher::operator()(const broadphase_cell_key &aKey) const noexcept {
    return static_cast<std::size_t>(jfc::hash::values(aKey.x, aKey.y, aKey.z));
}

broadphase_cell_key gdk::collisions::broadphase_cell_coord(const floating_point_type aCellSize,
    const vector3_type &aValue) {
    const auto cell = [aCellSize](const floating_point_type aComponent) {
        const auto index = std::floor(aComponent / aCellSize);

        if (std::isnan(index)) return 0;

        return static_cast<int>(std::clamp(index, -CELL_LIMIT, CELL_LIMIT));
    };

    return broadphase_cell_key{cell(aValue.x), cell(aValue.y), cell(aValue.z)};
}

bool gdk::collisions::bounds_are_finite(const bounds &aBounds) {
    return std::isfinite(aBounds.min.x) && std::isfinite(aBounds.min.y) && std::isfinite(aBounds.min.z)
        && std::isfinite(aBounds.max.x) && std::isfinite(aBounds.max.y) && std::isfinite(aBounds.max.z);
}

bool gdk::collisions::bounds_overlap(const bounds &aFirst, const bounds &aSecond) {
    return aFirst.min.x <= aSecond.max.x && aSecond.min.x <= aFirst.max.x
        && aFirst.min.y <= aSecond.max.y && aSecond.min.y <= aFirst.max.y
        && aFirst.min.z <= aSecond.max.z && aSecond.min.z <= aFirst.max.z;
}

std::uint64_t gdk::collisions::broadphase_cell_span(const broadphase_cell_key &aMin, const broadphase_cell_key &aMax) {
    const auto extent = [](const int aLow, const int aHigh) {
        return aHigh < aLow ? std::uint64_t{0} : static_cast<std::uint64_t>(static_cast<std::int64_t>(aHigh) - aLow + 1);
    };

    return extent(aMin.x, aMax.x) * extent(aMin.y, aMax.y) * extent(aMin.z, aMax.z);
}

impl_static_grid::impl_static_grid(const std_kernels_policy &aPolicy)
: m_CellSize(aPolicy.BROAD_PHASE_CELL_SIZE)
, m_MaxCellsPerBody(aPolicy.BROAD_PHASE_MAX_CELLS_PER_BODY)
{}

void impl_static_grid::insert(const std::uint32_t aRow, const bounds &aBounds) {
    if (aRow >= m_Placements.size()) m_Placements.resize(static_cast<std::size_t>(aRow) + 1);

    auto &placement = m_Placements[aRow];

    if (placement.present) remove(aRow);

    placement.extent = aBounds;
    placement.present = true;

    if (bounds_are_finite(aBounds)) {
        placement.min = cell_coord(aBounds.min);
        placement.max = cell_coord(aBounds.max);
        placement.unbinned = broadphase_cell_span(placement.min, placement.max) > m_MaxCellsPerBody;
    }
    else placement.unbinned = true;

    if (placement.unbinned) {
        m_Unbinned.push_back(aRow);
        return;
    }

    for (int z = placement.min.z; z <= placement.max.z; ++z)
    for (int y = placement.min.y; y <= placement.max.y; ++y)
    for (int x = placement.min.x; x <= placement.max.x; ++x)
        m_Cells[cell_key{x, y, z}].push_back(aRow);
}

void impl_static_grid::remove(const std::uint32_t aRow) {
    if (aRow >= m_Placements.size() || !m_Placements[aRow].present) return;

    auto &placement = m_Placements[aRow];
    placement.present = false;

    if (placement.unbinned) {
        erase_row(m_Unbinned, aRow);
        return;
    }

    for (int z = placement.min.z; z <= placement.max.z; ++z)
    for (int y = placement.min.y; y <= placement.max.y; ++y)
    for (int x = placement.min.x; x <= placement.max.x; ++x)
        if (const auto it = m_Cells.find(cell_key{x, y, z}); it != m_Cells.end()) {
            erase_row(it->second, aRow);

            if (it->second.empty()) m_Cells.erase(it);
        }
}
