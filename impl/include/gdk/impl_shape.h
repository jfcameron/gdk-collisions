// © Joseph Cameron - All Rights Reserved

#ifndef GDK_IMPL_SHAPE_H
#define GDK_IMPL_SHAPE_H

#include <gdk/collision_types.h>
#include <gdk/impl_heightfield_data.h>
#include <gdk/impl_mesh_data.h>

#include <memory>
#include <variant>

namespace gdk {
    /// \brief geometry of a sphere collider
    struct sphere_shape final {
        collision_floating_point_type radius = {0.5};
    };

    /// \brief geometry of a capsule collider: every point within radius of a segment.
    struct capsule_shape final {
        collision_floating_point_type radius = {0.5};
        collision_floating_point_type half_height = {0.5};
    };

    /// \brief geometry of an axis aligned box collider
    struct box_shape final {
        collision_vector3_type half_extents = {0.5, 0.5, 0.5};
    };

    /// \brief geometry of an oriented box collider.
    struct obb_shape final {
        collision_vector3_type half_extents = {0.5, 0.5, 0.5};
    };

    /// \brief a single triangle in the collider's local space.
    struct triangle_shape final {
        collision_vector3_type a;
        collision_vector3_type b;
        collision_vector3_type c;
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

    using collision_shape_type = std::variant<sphere_shape, box_shape, capsule_shape, obb_shape, plane_shape,
        triangle_shape, mesh_shape, heightfield_shape>;

    /// \brief one piece of a collider's geometry in the collider's local space
    struct collider_part final {
        collision_shape_type shape;
        collision_vector3_type position = collision_vector3_type::zero;
        collision_quaternion_type rotation = collision_quaternion_type::identity;
    };

    /// \brief rotate a vector by a unit quaternion.
    //TODO: is this the correct place for this? look at gdk-math for alternative or possible refactor
    [[nodiscard]] inline collision_vector3_type rotate(const collision_quaternion_type &aRotation,
        const collision_vector3_type &aVector) {
        return aRotation * aVector;
    }

    template <typename... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    template <typename... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}

#endif
