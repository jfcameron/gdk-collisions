// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_AABB_COLLIDER_H
#define GDK_COLLISIONS_AABB_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief an axis aligned box collider
    class aabb_collider : public virtual collider {
    public:
        virtual ~aabb_collider() = default;

        /// \brief set the box's half extents
        virtual void set_half_extents(const vector3_type &aHalfExtents) = 0;

        /// \brief get the box's half extents
        [[nodiscard]] virtual vector3_type half_extents() const = 0;
    };
}

#endif
