// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_OBB_COLLIDER_H
#define GDK_COLLISION_OBB_COLLIDER_H

#include <gdk/collider.h>
#include <gdk/collision_types.h>

namespace gdk {
    /// \brief an oriented box collider: a box that follows its collider's rotation.
    class obb_collider : public virtual collider {
    public:
        virtual ~obb_collider() = default;

        /// \brief set the box's half extents, measured along its own local axes
        virtual void set_half_extents(const collision_vector3_type &aHalfExtents) = 0;

        /// \brief get the box's half extents
        [[nodiscard]] virtual collision_vector3_type half_extents() const = 0;
    };
}

#endif
