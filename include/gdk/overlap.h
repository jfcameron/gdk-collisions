// © Joseph Cameron - All Rights Reserved

#ifndef GDK_OVERLAP_RESULT_H
#define GDK_OVERLAP_RESULT_H

#include <gdk/collision_types.h>

namespace gdk {
    /// \brief stores data for a detected collider overlap
    struct overlap final {
        /// \brief which world axis a box-vs-box contact occurred on.
        //TODO: harmless for other types but unneeded by them. think about this
        enum class contact_axis : std::size_t {
            X = 0,
            Y = 1,
            Z = 2,
        };

        collision_floating_point_type entry_time = {0};
        collision_floating_point_type exit_time = {0};
        collision_floating_point_type penetration = {0};
        collision_vector3_type contact_normal = collision_vector3_type::zero;
        collision_vector3_type contact_point  = collision_vector3_type::zero;
    };
}

#endif
