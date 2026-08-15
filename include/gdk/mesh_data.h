// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_MESH_DATA_H
#define GDK_COLLISION_MESH_DATA_H

#include <gdk/collision_types.h>

#include <cstddef>

namespace gdk {
    /// \brief the three corners of one of a mesh's triangles
    struct mesh_triangle final {
        collision_vector3_type a;
        collision_vector3_type b;
        collision_vector3_type c;
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
}

#endif
