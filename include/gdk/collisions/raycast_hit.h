// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_RAYCAST_HIT_H
#define GDK_COLLISIONS_RAYCAST_HIT_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief what a ray struck, and where
    struct raycast_hit final {
        /// \brief the collider that was hit
        const_collider_ptr_type collider;

        /// \brief how far along the ray the contact occurred in world units
        floating_point_type distance = {0};

        /// \brief the point of contact in world space
        vector3_type point = vector3_type::zero;

        /// \brief the struck surface's normal facing back toward the ray's origin
        vector3_type normal = vector3_type::zero;
    };
}

#endif
