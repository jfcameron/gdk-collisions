// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_OVERLAP_RESULT_H
#define GDK_COLLISIONS_OVERLAP_RESULT_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief stores data for a detected collider overlap
    struct overlap final {
        /// \brief which world axis a box-vs-box contact occurred on.
        //TODO: harmless for other types but unneeded by them. think about this
        enum class contact_axis : std::size_t {
            X = 0,
            Y = 1,
            Z = 2,
        };

        floating_point_type entry_time = {0};
        floating_point_type exit_time = {0};
        floating_point_type penetration = {0};
        vector3_type contact_normal = vector3_type::zero;
        vector3_type contact_point  = vector3_type::zero;
    };
}

#endif
