// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_CAPSULE_COLLIDER_H
#define GDK_COLLISIONS_CAPSULE_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief a capsule: every point within a radius of a line segment.
    class capsule_collider : public virtual collider {
    public:
        virtual ~capsule_collider() = default;

        /// \brief set the radius of the capsule's hemispherical caps and cylindrical body
        virtual void set_radius(const floating_point_type aRadius) = 0;

        /// \brief get the radius
        [[nodiscard]] virtual floating_point_type radius() const = 0;

        /// \brief set the distance from the centre to either end of the interior segment
        virtual void set_half_height(const floating_point_type aHalfHeight) = 0;

        /// \brief get the distance from the centre to either end of the interior segment
        [[nodiscard]] virtual floating_point_type half_height() const = 0;
    };
}

#endif
