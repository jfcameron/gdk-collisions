// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_SWEEP_HIT_H
#define GDK_COLLISIONS_SWEEP_HIT_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief what a swept shape met first, and where
    struct sweep_hit final {
        const_collider_ptr_type collider;

        /// \brief how far the shape travelled before it met the collider, in world units
        floating_point_type distance = {0};

        /// \brief the point of contact in world space
        vector3_type point = vector3_type::zero;

        /// \brief the struck surface's normal facing back toward the shape
        vector3_type normal = vector3_type::zero;

        /// \brief how deep the shape already reached into the collider where it began, in world units.
        /// distance is 0 when this is not
        floating_point_type penetration = {0};
    };
}

#endif
