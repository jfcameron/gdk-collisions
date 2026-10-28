// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_OVERLAP_RESULT_H
#define GDK_COLLISIONS_OVERLAP_RESULT_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief stores data for a detected collider overlap
    struct overlap final {
        floating_point_type entry_time = {0};
        floating_point_type exit_time = {0};
        floating_point_type penetration = {0};
        vector3_type contact_normal = vector3_type::zero;
        vector3_type contact_point  = vector3_type::zero;
    };
}

#endif
