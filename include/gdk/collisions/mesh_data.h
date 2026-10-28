// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_MESH_DATA_H
#define GDK_COLLISIONS_MESH_DATA_H

#include <gdk/collisions/types.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gdk::collisions {
    /// \brief the three corners of one of a mesh's triangles
    struct mesh_triangle final {
        vector3_type a;
        vector3_type b;
        vector3_type c;
    };

    /// \brief a collection of triangles. used to construct mesh colliders
    class mesh_data {
    public:
        virtual ~mesh_data() = default;

        /// \brief how many triangles this mesh holds
        [[nodiscard]] virtual std::size_t triangle_count() const = 0;

        /// \brief the corners of one triangle, in the mesh's own space.
        /// \param aTriangle in [0, triangle_count()).
        [[nodiscard]] virtual mesh_triangle triangle(const std::size_t aTriangle) const = 0;
    };

    /// \brief triangles for mesh colliders, as positions and three indices per triangle.
    /// Two triangles sharing an edge within aCoplanarTolerance (a dot product of unit normals) of flat
    /// are treated as one surface across it.
    /// \exception exception if an index is out of range, the index count is not a multiple of three,
    /// or a position is not finite
    [[nodiscard]] mesh_data_ptr_type make_mesh_data(std::vector<vector3_type> aVertices,
        std::vector<std::uint32_t> aIndices, const floating_point_type aCoplanarTolerance = 1e-4f);
}

#endif
