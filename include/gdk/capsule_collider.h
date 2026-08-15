// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_CAPSULE_COLLIDER_H
#define GDK_COLLISION_CAPSULE_COLLIDER_H

#include <gdk/collider.h>
#include <gdk/collision_types.h>

namespace gdk {
    /// \brief a capsule: every point within a radius of a line segment.
    class capsule_collider : public virtual collider {
    public:
        virtual ~capsule_collider() = default;

        /// \brief set the radius of the capsule's hemispherical caps and cylindrical body
        virtual void set_radius(const collision_floating_point_type aRadius) = 0;

        /// \brief get the radius
        [[nodiscard]] virtual collision_floating_point_type radius() const = 0;

        /// \brief set the distance from the centre to either end of the interior segment
        virtual void set_half_height(const collision_floating_point_type aHalfHeight) = 0;

        /// \brief get the distance from the centre to either end of the interior segment
        [[nodiscard]] virtual collision_floating_point_type half_height() const = 0;
    };
}

#endif
