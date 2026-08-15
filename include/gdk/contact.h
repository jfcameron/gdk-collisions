// © Joseph Cameron - All Rights Reserved

#ifndef GDK_CONTACT_H
#define GDK_CONTACT_H

#include <gdk/collision_types.h>

namespace gdk {
    /// \brief two colliders that were touching, and the geometry of where.
    struct contact final {
        const_collider_ptr_type a;
        const_collider_ptr_type b;

        /// \brief points from `b` toward `a`, the same convention every narrow-phase cell reports
        collision_vector3_type normal = collision_vector3_type::zero;

        /// \brief where the contact occurred in world space
        collision_vector3_type point = collision_vector3_type::zero;

        /// \brief how deeply the two overlapped, in world units
        collision_floating_point_type penetration = {0};
    };
}

#endif
