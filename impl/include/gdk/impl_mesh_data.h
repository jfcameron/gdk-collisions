// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_MESH_DATA_H
#define GDK_COLLISION_IMPL_MESH_DATA_H

#include <gdk/collision_types.h>
#include <gdk/mesh_data.h>

#include <cstdint>
#include <vector>

namespace gdk {
    /// \brief triangle geometry plus a bounding volume hierarchy over it.
    class impl_mesh_data final : public mesh_data {
    public:
        struct bounds final {
            collision_vector3_type min;
            collision_vector3_type max;
        };

        /// \brief build a mesh from vertex positions and triangle indices.
        ///
        /// \param aVertices positions in the mesh's own space.
        /// \param aIndices three per triangle, indexing into aVertices. Its size must be a multiple
        ///        of three and every index must be in range.
        /// \param aCoplanarTolerance how nearly parallel two triangles sharing an edge must be for
        ///        that edge to count as flat, as a dot product of unit normals. \see internal_edges
        ///
        /// \exception collision_exception if the index buffer is malformed.
        [[nodiscard]] static mesh_data_ptr_type make(std::vector<collision_vector3_type> aVertices,
            std::vector<std::uint32_t> aIndices,
            const collision_floating_point_type aCoplanarTolerance = 1e-4f);

        virtual ~impl_mesh_data() = default;

        [[nodiscard]] virtual std::size_t triangle_count() const override;

        /// \brief the corners of triangle aTriangle in the mesh's local space
        void triangle(const std::uint32_t aTriangle, collision_vector3_type &aA,
            collision_vector3_type &aB, collision_vector3_type &aC) const;

        [[nodiscard]] virtual mesh_triangle triangle(const std::size_t aTriangle) const override;

        /// \brief bounds of the whole mesh in its local space
        [[nodiscard]] const bounds &root_bounds() const;

        /// \brief which of triangle aTriangle's edges lie flat against a neighbour.
        [[nodiscard]] std::uint8_t internal_edges(const std::uint32_t aTriangle) const;

        /// \brief append the indices of every triangle whose bounds meet aQuery
        void query(const bounds &aQuery, std::vector<std::uint32_t> &aOut) const;

        /// \brief get every triangle index
        void all(std::vector<std::uint32_t> &aOut) const;

    private:
        impl_mesh_data(std::vector<collision_vector3_type> aVertices, std::vector<std::uint32_t> aIndices,
            const collision_floating_point_type aCoplanarTolerance);

        struct node final {
            bounds box;
            std::uint32_t first = 0;
            std::uint32_t count = 0;
            std::uint32_t right = 0;
        };

        /// \brief recursively build the subtree covering m_Order[aFirst, aFirst + aCount) and
        /// return its node index
        std::uint32_t build(const std::uint32_t aFirst, const std::uint32_t aCount,
            const std::vector<collision_vector3_type> &aCentroids);

        [[nodiscard]] bounds bounds_of(const std::uint32_t aTriangle) const;

        /// \brief calculate which edges are flat and internal
        void flag_internal_edges(const collision_floating_point_type aCoplanarTolerance);

        std::vector<collision_vector3_type> m_Vertices;
        std::vector<std::uint32_t> m_Indices;

        std::vector<std::uint32_t> m_Order;
        std::vector<node> m_Nodes;

        std::vector<std::uint8_t> m_InternalEdges;
    };
}

#endif
