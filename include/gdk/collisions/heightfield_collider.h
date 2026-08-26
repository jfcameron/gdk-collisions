// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_HEIGHTFIELD_COLLIDER_H
#define GDK_COLLISIONS_HEIGHTFIELD_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief a collider whose geometry is a grid of heights over its local XZ plane.
    ///
    /// \warning a heightfield collider cannot collide with other heightfields nor meshes.
    /// It is a quadratic collision case, so it is this library's policy to ignore them 
    class heightfield_collider : public virtual collider {
    public:
        virtual ~heightfield_collider() = default;

        /// \brief use the given terrain. 
        virtual void set_heightfield(const heightfield_data_ptr_type &aHeightfield) = 0;

        /// \brief the terrain currently in use, or null
        [[nodiscard]] virtual heightfield_data_ptr_type heightfield() const = 0;
    };
}

#endif
