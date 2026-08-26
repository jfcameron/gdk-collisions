// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_OBB_COLLIDER_H
#define GDK_COLLISIONS_OBB_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief an oriented box collider: a box that follows its collider's rotation.
    class obb_collider : public virtual collider {
    public:
        virtual ~obb_collider() = default;

        /// \brief set the box's half extents, measured along its own local axes
        virtual void set_half_extents(const vector3_type &aHalfExtents) = 0;

        /// \brief get the box's half extents
        [[nodiscard]] virtual vector3_type half_extents() const = 0;
    };
}

#endif
