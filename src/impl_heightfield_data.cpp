// © Joseph Cameron - All Rights Reserved

#include <gdk/collision_exception.h>
#include <gdk/impl_heightfield_data.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

using namespace gdk;

namespace {
    /// \brief the two triangles of a cell, as offsets from its low corner.
    struct cell_corner final { std::size_t column; std::size_t row; };

    constexpr cell_corner FIRST_TRIANGLE[3] = {{0, 0}, {1, 0}, {1, 1}};
    constexpr cell_corner SECOND_TRIANGLE[3] = {{0, 0}, {1, 1}, {0, 1}};
}

heightfield_data_ptr_type impl_heightfield_data::make(const std::size_t aColumns, const std::size_t aRows,
    std::vector<collision_floating_point_type> aHeights, const collision_vector2_type &aCellSize,
    const collision_floating_point_type aCoplanarTolerance) {
    if (aColumns < 2 || aRows < 2)
        throw collision_exception("gdk::collision: a heightfield needs at least two samples on each axis");

    if (aHeights.size() != aColumns * aRows)
        throw collision_exception("gdk::collision: heightfield sample count does not match its dimensions");

    if (aCellSize.x <= 0 || aCellSize.y <= 0)
        throw collision_exception("gdk::collision: heightfield cell size must be positive on both axes");

    return heightfield_data_ptr_type(new impl_heightfield_data(aColumns, aRows, std::move(aHeights),
        aCellSize, aCoplanarTolerance));
}

impl_heightfield_data::impl_heightfield_data(const std::size_t aColumns, const std::size_t aRows,
    std::vector<collision_floating_point_type> aHeights, const collision_vector2_type &aCellSize,
    const collision_floating_point_type aCoplanarTolerance)
: m_Columns(aColumns)
, m_Rows(aRows)
, m_CellSize(aCellSize)
, m_Heights(std::move(aHeights))
{
    m_Origin = collision_vector3_type{
        -0.5f * (aColumns - 1) * aCellSize.x, 0, -0.5f * (aRows - 1) * aCellSize.y};

    const auto extremes = std::minmax_element(m_Heights.begin(), m_Heights.end());

    m_Bounds.min = collision_vector3_type{m_Origin.x, *extremes.first, m_Origin.z};
    m_Bounds.max = collision_vector3_type{-m_Origin.x, *extremes.second, -m_Origin.z};

    flag_internal_edges(aCoplanarTolerance);
}

std::size_t impl_heightfield_data::columns() const { return m_Columns; }
std::size_t impl_heightfield_data::rows() const { return m_Rows; }

collision_floating_point_type impl_heightfield_data::height(const std::size_t aColumn,
    const std::size_t aRow) const {
    return m_Heights[aRow * m_Columns + aColumn];
}

collision_vector2_type impl_heightfield_data::cell_size() const { return m_CellSize; }

std::size_t impl_heightfield_data::triangle_count() const {
    return (m_Columns - 1) * (m_Rows - 1) * 2;
}

collision_vector3_type impl_heightfield_data::corner(const std::size_t aColumn, const std::size_t aRow) const {
    return collision_vector3_type{
        m_Origin.x + aColumn * m_CellSize.x,
        height(aColumn, aRow),
        m_Origin.z + aRow * m_CellSize.y};
}

void impl_heightfield_data::triangle(const std::uint32_t aTriangle, collision_vector3_type &aA,
    collision_vector3_type &aB, collision_vector3_type &aC) const {
    const auto cell = aTriangle / 2;
    const auto column = cell % (m_Columns - 1);
    const auto row = cell / (m_Columns - 1);

    const auto *pOffsets = (aTriangle & 1) ? SECOND_TRIANGLE : FIRST_TRIANGLE;

    aA = corner(column + pOffsets[0].column, row + pOffsets[0].row);
    aB = corner(column + pOffsets[1].column, row + pOffsets[1].row);
    aC = corner(column + pOffsets[2].column, row + pOffsets[2].row);
}

const impl_heightfield_data::bounds &impl_heightfield_data::root_bounds() const {
    return m_Bounds;
}

std::uint8_t impl_heightfield_data::internal_edges(const std::uint32_t aTriangle) const {
    return m_InternalEdges[aTriangle];
}

void impl_heightfield_data::flag_internal_edges(const collision_floating_point_type aCoplanarTolerance) {
    const auto count = static_cast<std::uint32_t>(triangle_count());
    m_InternalEdges.assign(count, 0);

    const auto normal_of = [this](const std::uint32_t aTriangle) {
        collision_vector3_type a, b, c;
        triangle(aTriangle, a, b, c);
        return (b - a).cross_product(c - a).normal();
    };

    const auto cellsPerRow = m_Columns - 1;
    const auto cellRows = m_Rows - 1;

    for (std::uint32_t t = 0; t < count; ++t) {
        const auto cell = t / 2;
        const auto column = cell % cellsPerRow;
        const auto row = cell / cellsPerRow;
        const auto normal = normal_of(t);

        std::int64_t neighbours[3];

        if ((t & 1) == 0) {
            neighbours[0] = row > 0 ? static_cast<std::int64_t>(((row - 1) * cellsPerRow + column) * 2 + 1) : -1;
            neighbours[1] = column + 1 < cellsPerRow
                ? static_cast<std::int64_t>((row * cellsPerRow + column + 1) * 2 + 1) : -1;
            neighbours[2] = static_cast<std::int64_t>(t + 1);
        }
        else {
            neighbours[0] = static_cast<std::int64_t>(t - 1);
            neighbours[1] = row + 1 < cellRows
                ? static_cast<std::int64_t>(((row + 1) * cellsPerRow + column) * 2) : -1;
            neighbours[2] = column > 0
                ? static_cast<std::int64_t>((row * cellsPerRow + column - 1) * 2) : -1;
        }

        for (int e = 0; e < 3; ++e) {
            if (neighbours[e] < 0) continue;

            if (normal.dot_product(normal_of(static_cast<std::uint32_t>(neighbours[e])))
                >= 1.0f - aCoplanarTolerance)
                m_InternalEdges[t] |= static_cast<std::uint8_t>(1u << e);
        }
    }
}

void impl_heightfield_data::query(const bounds &aQuery, std::vector<std::uint32_t> &aOut) const {
    const auto cellsPerRow = m_Columns - 1;
    const auto cellRows = m_Rows - 1;

    if (aQuery.max.x < m_Bounds.min.x || aQuery.min.x > m_Bounds.max.x
        || aQuery.max.z < m_Bounds.min.z || aQuery.min.z > m_Bounds.max.z) return;

    const auto to_cell = [](const collision_floating_point_type aCoordinate,
        const collision_floating_point_type aOrigin, const collision_floating_point_type aSize,
        const std::size_t aLimit) {
        const auto index = static_cast<std::int64_t>(std::floor((aCoordinate - aOrigin) / aSize));
        return static_cast<std::size_t>(std::min<std::int64_t>(std::max<std::int64_t>(index, 0),
            static_cast<std::int64_t>(aLimit) - 1));
    };

    const auto firstColumn = to_cell(aQuery.min.x, m_Origin.x, m_CellSize.x, cellsPerRow);
    const auto lastColumn = to_cell(aQuery.max.x, m_Origin.x, m_CellSize.x, cellsPerRow);
    const auto firstRow = to_cell(aQuery.min.z, m_Origin.z, m_CellSize.y, cellRows);
    const auto lastRow = to_cell(aQuery.max.z, m_Origin.z, m_CellSize.y, cellRows);

    for (auto row = firstRow; row <= lastRow; ++row)
        for (auto column = firstColumn; column <= lastColumn; ++column) {
            const auto cell = static_cast<std::uint32_t>(row * cellsPerRow + column);
            aOut.push_back(cell * 2);
            aOut.push_back(cell * 2 + 1);
        }
}

void impl_heightfield_data::all(std::vector<std::uint32_t> &aOut) const {
    const auto count = static_cast<std::uint32_t>(triangle_count());
    for (std::uint32_t t = 0; t < count; ++t) aOut.push_back(t);
}
