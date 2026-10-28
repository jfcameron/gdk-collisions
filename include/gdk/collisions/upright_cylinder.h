// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_UPRIGHT_CYLINDER_H
#define GDK_COLLISIONS_UPRIGHT_CYLINDER_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    struct upright_cylinder final {
        vector3_type base = vector3_type::zero;

        floating_point_type height = {0};
        floating_point_type radius = {0};
    };
}

#endif
