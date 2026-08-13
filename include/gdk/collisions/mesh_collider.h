// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_MESH_COLLIDER_H
#define GDK_COLLISIONS_MESH_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief a collider made of triangles
    class mesh_collider : public virtual collider {
    public:
        virtual ~mesh_collider() = default;

        /// \brief use the given geometry
        virtual void set_mesh(const mesh_data_ptr_type &aMesh) = 0;

        /// \brief the geometry currently in use, or null
        [[nodiscard]] virtual mesh_data_ptr_type mesh() const = 0;
    };
}

#endif
