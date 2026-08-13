// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_BOX_COLLIDER_H
#define GDK_COLLISIONS_BOX_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief an axis aligned box collider
    /// TODO: probably should rename to avoid confusion since non-axis-aligned box has been added
    class box_collider : public virtual collider {
    public:
        virtual ~box_collider() = default;

        /// \brief set the box's half extents
        virtual void set_half_extents(const vector3_type &aHalfExtents) = 0;

        /// \brief get the box's half extents
        [[nodiscard]] virtual vector3_type half_extents() const = 0;
    };
}

#endif
