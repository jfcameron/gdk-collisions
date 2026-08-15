// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_RAYCAST_HIT_H
#define GDK_COLLISION_RAYCAST_HIT_H

#include <gdk/collision_types.h>

namespace gdk {
    /// \brief what a ray struck, and where
    struct raycast_hit final {
        /// \brief the collider that was hit
        const_collider_ptr_type collider;

        /// \brief how far along the ray the contact occurred in world units
        collision_floating_point_type distance = {0};

        /// \brief the point of contact in world space
        collision_vector3_type point = collision_vector3_type::zero;

        /// \brief the struck surface's normal facing back toward the ray's origin
        collision_vector3_type normal = collision_vector3_type::zero;
    };
}

#endif
