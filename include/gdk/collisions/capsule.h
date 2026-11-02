// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_CAPSULE_H
#define GDK_COLLISIONS_CAPSULE_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief a segment along local Y through centre, swollen by radius
    struct capsule final {
        vector3_type centre = vector3_type::zero;
        floating_point_type radius = {0};
        /// \brief distance from the centre to either end of the segment
        floating_point_type half_height = {0};
        quaternion_type rotation = quaternion_type::identity;
    };
}

#endif
