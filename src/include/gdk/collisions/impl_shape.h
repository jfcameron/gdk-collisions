// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_SHAPE_H
#define GDK_COLLISIONS_IMPL_SHAPE_H

#include <gdk/collisions/kernels/tables.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_heightfield_data.h>
#include <gdk/collisions/impl_mesh_data.h>

#include <variant>

namespace gdk::collisions {
    struct sphere_shape final {
        floating_point_type radius = {0.5};
    };

    struct capsule_shape final {
        floating_point_type radius = {0.5};
        floating_point_type half_height = {0.5};
    };

    struct box_shape final {
        vector3_type half_extents = {0.5, 0.5, 0.5};
    };

    struct obb_shape final {
        vector3_type half_extents = {0.5, 0.5, 0.5};
    };

    struct triangle_shape final {
        vector3_type a;
        vector3_type b;
        vector3_type c;
    };

    struct mesh_shape final {
        const impl_mesh_data *data = nullptr;
    };

    struct heightfield_shape final {
        const impl_heightfield_data *data = nullptr;
    };

    struct plane_shape final {};

    /// \brief a part_record seen as its kind, borrowing its surface
    using shape_type = std::variant<sphere_shape, box_shape, capsule_shape, obb_shape, plane_shape,
        triangle_shape, mesh_shape, heightfield_shape>;

    struct collider_part final {
        shape_type shape;
        vector3_type position = vector3_type::zero;
        quaternion_type rotation = quaternion_type::identity;
    };

    struct shape_kinematics final {
        vector3_type position;
        vector3_type velocity;
        quaternion_type orientation = quaternion_type::identity;
    };

    template <typename... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    template <typename... Ts> overloaded(Ts...) -> overloaded<Ts...>;

    [[nodiscard]] shape_type shape_of(const part_record &aPart, const impl_mesh_data *aMesh,
        const impl_heightfield_data *aHeightfield);

    [[nodiscard]] collider_part decoded(const part_record &aPart, const impl_mesh_data *aMesh,
        const impl_heightfield_data *aHeightfield);

    struct shape_offsets final {
        vector3_type low;
        vector3_type high;
    };

    //! a shape's world-space axis aligned bounds, about its centre
    [[nodiscard]] shape_offsets shape_bounds(const shape_type &aShape,
        const quaternion_type &aOrientation = quaternion_type::identity);

    //! the half extents of shape_bounds
    [[nodiscard]] vector3_type shape_extents(const shape_type &aShape,
        const quaternion_type &aOrientation = quaternion_type::identity);
}

#endif
