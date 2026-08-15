// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_HEIGHTFIELD_DATA_H
#define GDK_COLLISION_IMPL_HEIGHTFIELD_DATA_H

#include <gdk/collision_types.h>
#include <gdk/heightfield_data.h>
#include <gdk/impl_mesh_data.h>

#include <cstdint>
#include <vector>

namespace gdk {
    class impl_heightfield_data final : public heightfield_data {
    public:
        using bounds = impl_mesh_data::bounds;

        /// \brief build terrain from a grid of samples.
        ///
        /// \param aColumns samples along local X; at least 2.
        /// \param aRows samples along local Z; at least 2.
        /// \param aHeights aColumns * aRows samples, row major -- index (row * columns + column).
        /// \param aCellSize spacing between samples on X and Z. Must be positive.
        /// \param aCoplanarTolerance as impl_mesh_data::make. \see impl_mesh_data::internal_edges
        ///
        /// The surface spans (aColumns - 1) * aCellSize.x by (aRows - 1) * aCellSize.y, centred on
        /// the origin, so a collider's position is the middle of its terrain rather than a corner.
        ///
        /// \exception collision_exception if the dimensions or the sample count disagree.
        [[nodiscard]] static heightfield_data_ptr_type make(const std::size_t aColumns,
            const std::size_t aRows, std::vector<collision_floating_point_type> aHeights,
            const collision_vector2_type &aCellSize = {1, 1},
            const collision_floating_point_type aCoplanarTolerance = 1e-4f);

        virtual ~impl_heightfield_data() = default;

        [[nodiscard]] virtual std::size_t columns() const override;
        [[nodiscard]] virtual std::size_t rows() const override;
        [[nodiscard]] virtual collision_floating_point_type height(const std::size_t aColumn,
            const std::size_t aRow) const override;
        [[nodiscard]] virtual collision_vector2_type cell_size() const override;

        /// \brief two per cell, so (columns - 1) * (rows - 1) * 2
        [[nodiscard]] std::size_t triangle_count() const;

        /// \brief the corners of triangle aTriangle, in local space.
        void triangle(const std::uint32_t aTriangle, collision_vector3_type &aA,
            collision_vector3_type &aB, collision_vector3_type &aC) const;

        [[nodiscard]] const bounds &root_bounds() const;

        [[nodiscard]] std::uint8_t internal_edges(const std::uint32_t aTriangle) const;

        /// \brief append every triangle whose cell the query's X/Z footprint reaches
        void query(const bounds &aQuery, std::vector<std::uint32_t> &aOut) const;

        /// \brief every triangle, for a query whose region cannot be bounded. \see impl_mesh_data::all
        void all(std::vector<std::uint32_t> &aOut) const;

    private:
        impl_heightfield_data(const std::size_t aColumns, const std::size_t aRows,
            std::vector<collision_floating_point_type> aHeights, const collision_vector2_type &aCellSize,
            const collision_floating_point_type aCoplanarTolerance);

        void flag_internal_edges(const collision_floating_point_type aCoplanarTolerance);

        /// \brief the sample at a grid coordinate, as a local-space point
        [[nodiscard]] collision_vector3_type corner(const std::size_t aColumn, const std::size_t aRow) const;

        std::size_t m_Columns = 0;
        std::size_t m_Rows = 0;
        collision_vector2_type m_CellSize{1, 1};
        collision_vector3_type m_Origin;

        std::vector<collision_floating_point_type> m_Heights;
        std::vector<std::uint8_t> m_InternalEdges;
        bounds m_Bounds;
    };
}

#endif
