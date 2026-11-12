// © Joseph Cameron - MIT License
#ifndef GDK_COLLISIONS_IMPL_TRIANGLE_NORMAL_H
#define GDK_COLLISIONS_IMPL_TRIANGLE_NORMAL_H
#include <cmath>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    inline vector3_type impl_triangle_normal(const vector3_type &aA, const vector3_type &aB, const vector3_type &aC) {
        const auto precise = [](const vector3_type &aPoint) {
            return gdk::math::vector3<double>{aPoint.x, aPoint.y, aPoint.z};
        };
        const auto crossed = (precise(aB) - precise(aA)).cross_product(precise(aC) - precise(aA));
        const auto square = crossed.length_squared();
        if (square == 0)
            return vector3_type::zero;
        const auto normal = crossed * (1 / std::sqrt(square));
        return {static_cast<floating_point_type>(normal.x), static_cast<floating_point_type>(normal.y),
                static_cast<floating_point_type>(normal.z)};
    }
} // namespace gdk::collisions
#endif
