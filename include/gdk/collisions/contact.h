// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_CONTACT_H
#define GDK_COLLISIONS_CONTACT_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief two colliders that were touching, and the geometry of where.
    struct contact final {
        const_collider_ptr_type a;
        const_collider_ptr_type b;

        /// \brief points from `b` toward `a`, the same convention every narrow-phase cell reports
        vector3_type normal = vector3_type::zero;

        /// \brief where the contact occurred in world space
        vector3_type point = vector3_type::zero;

        /// \brief how deeply the two overlapped, in world units
        floating_point_type penetration = {0};
    };
}

#endif
