// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_STD_QUERIES_H
#define GDK_COLLISIONS_IMPL_STD_QUERIES_H

#include <gdk/collisions/overlap.h>
#include <gdk/collisions/parallelepiped.h>
#include <gdk/collisions/upright_cylinder.h>
#include <gdk/collisions/impl_shape.h>

#include <optional>
#include <span>

namespace gdk::collisions {
    /// \brief where a ray first meets a collider's parts
    [[nodiscard]] std::optional<overlap> ray_against_parts(const vector3_type &aOrigin, const vector3_type &aDirection,
        floating_point_type aMaxDistance, std::span<const collider_part> aParts, const shape_kinematics &aCollider);

    /// \brief whether any of a collider's parts reaches into an upright cylinder
    [[nodiscard]] bool cylinder_meets_parts(const upright_cylinder &aCylinder,
        std::span<const collider_part> aParts, const shape_kinematics &aCollider);

    /// \brief whether any of a collider's parts reaches into a parallelepiped
    [[nodiscard]] bool parallelepiped_meets_parts(const parallelepiped &aVolume,
        std::span<const collider_part> aParts, const shape_kinematics &aCollider);

    //! the least and the most corner of a parallelepiped's box in the world
    [[nodiscard]] vector3_type parallelepiped_low(const parallelepiped &aVolume);
    [[nodiscard]] vector3_type parallelepiped_high(const parallelepiped &aVolume);
}

#endif
