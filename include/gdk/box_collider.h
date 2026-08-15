// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_BOX_COLLIDER_H
#define GDK_COLLISION_BOX_COLLIDER_H

#include <gdk/collider.h>
#include <gdk/collision_types.h>

namespace gdk {
    /// \brief an axis aligned box collider
    /// TODO: probably should rename to avoid confusion since non-axis-aligned box has been added
    class box_collider : public virtual collider {
    public:
        virtual ~box_collider() = default;

        /// \brief set the box's half extents
        virtual void set_half_extents(const collision_vector3_type &aHalfExtents) = 0;

        /// \brief get the box's half extents
        [[nodiscard]] virtual collision_vector3_type half_extents() const = 0;
    };
}

#endif
