// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_PARALLELEPIPED_H
#define GDK_COLLISIONS_PARALLELEPIPED_H

#include <gdk/collisions/types.h>

#include <array>

namespace gdk::collisions {
    struct parallelepiped final {
        vector3_type corner = vector3_type::zero;

        std::array<vector3_type, 3> edges{};
    };
}

#endif
