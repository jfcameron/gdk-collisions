// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_SHAPE_H
#define GDK_COLLISIONS_IMPL_SHAPE_H

#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_heightfield_data.h>
#include <gdk/collisions/impl_mesh_data.h>

#include <memory>
#include <variant>

namespace gdk::collisions {
    /// \brief geometry of a sphere collider
    struct sphere_shape final {
        floating_point_type radius = {0.5};
    };

    /// \brief geometry of a capsule collider: every point within radius of a segment.
    struct capsule_shape final {
        floating_point_type radius = {0.5};
        floating_point_type half_height = {0.5};
    };

    /// \brief geometry of an axis aligned box collider
    struct box_shape final {
        vector3_type half_extents = {0.5, 0.5, 0.5};
    };

    /// \brief geometry of an oriented box collider.
    struct obb_shape final {
        vector3_type half_extents = {0.5, 0.5, 0.5};
    };

    /// \brief a single triangle in the collider's local space.
    struct triangle_shape final {
        vector3_type a;
        vector3_type b;
        vector3_type c;
    };

    /// \brief geometry of a mesh collider: a reference to shared triangle data.
    struct mesh_shape final {
        std::shared_ptr<const impl_mesh_data> data;
    };

    /// \brief geometry of a heightfield collider: a reference to shared terrain.
    struct heightfield_shape final {
        std::shared_ptr<const impl_heightfield_data> data;
    };

    /// \brief geometry of an infinite half-space.
    struct plane_shape final {};

    using shape_type = std::variant<sphere_shape, box_shape, capsule_shape, obb_shape, plane_shape,
        triangle_shape, mesh_shape, heightfield_shape>;

    /// \brief one piece of a collider's geometry in the collider's local space
    struct collider_part final {
        shape_type shape;
        vector3_type position = vector3_type::zero;
        quaternion_type rotation = quaternion_type::identity;
    };

    /// \brief rotate a vector by a unit quaternion.
    //TODO: is this the correct place for this? look at gdk-math for alternative or possible refactor
    [[nodiscard]] inline vector3_type rotate(const quaternion_type &aRotation,
        const vector3_type &aVector) {
        return aRotation * aVector;
    }

    template <typename... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    template <typename... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}

#endif
