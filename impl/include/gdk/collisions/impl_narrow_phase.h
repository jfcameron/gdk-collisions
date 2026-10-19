// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_NARROW_PHASE_H
#define GDK_COLLISIONS_IMPL_NARROW_PHASE_H

#include <gdk/collisions/parallelepiped.h>
#include <gdk/collisions/upright_cylinder.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_shape.h>
#include <gdk/collisions/overlap.h>

#include <optional>
#include <vector>
#include <span>

namespace gdk::collisions {
    /// \brief the kinematic state a narrow-phase test needs about one participant
    struct shape_kinematics final {
        vector3_type position;
        vector3_type velocity;

        /// \brief the collider's orientation. Identity for shapes that are rotation-invariant or
        /// axis-aligned by definition; a capsule's interior segment is rotated by it.
        quaternion_type orientation = quaternion_type::identity;
    };

    /// \brief is aSubject moving *toward* this contact, rather than resting against it or leaving it?
    [[nodiscard]] bool is_closing(const overlap &aOverlap, const shape_kinematics &aSubject,
        const shape_kinematics &aOther, const impl_collision_policy &aPolicy);

    /// \brief test one collider's geometry against another's for the coming timestep.
    [[nodiscard]] std::optional<overlap> narrow_phase_overlap(
        const shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const shape_type &aOtherShape, const shape_kinematics &aOther,
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy);

    /// \brief how deeply two shapes overlap *right now*, with no regard for motion.
    [[nodiscard]] std::optional<overlap> narrow_phase_penetration(
        const shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const shape_type &aOtherShape, const shape_kinematics &aOther,
        const impl_collision_policy &aPolicy);

    /// \brief test two colliders' full geometry, part against part.
    [[nodiscard]] std::optional<overlap> narrow_phase_overlap_parts(
        std::span<const collider_part> aSubjectParts, const shape_kinematics &aSubject,
        std::span<const collider_part> aOtherParts, const shape_kinematics &aOther,
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy);

    /// \brief where a ray first meets a collider's parts
    [[nodiscard]] std::optional<overlap> narrow_phase_ray_parts(const vector3_type &aOrigin, const vector3_type &aDirection,
        floating_point_type aMaxDistance, std::span<const collider_part> aParts, const shape_kinematics &aCollider);

    /// \brief whether any of a collider's parts reaches into an upright cylinder
    [[nodiscard]] bool narrow_phase_cylinder_parts(const upright_cylinder &aCylinder,
        std::span<const collider_part> aParts, const shape_kinematics &aCollider);

    /// \brief whether any of a collider's parts reaches into a parallelepiped
    [[nodiscard]] bool narrow_phase_parallelepiped_parts(const parallelepiped &aVolume,
        std::span<const collider_part> aParts, const shape_kinematics &aCollider);

    //! the least and the most corner of a parallelepiped's box in the world
    [[nodiscard]] vector3_type parallelepiped_low(const parallelepiped &aVolume);
    [[nodiscard]] vector3_type parallelepiped_high(const parallelepiped &aVolume);

    /// \brief gets the half extents of a shape's world-space axis aligned bounds
    [[nodiscard]] vector3_type shape_extents(const shape_type &aShape,
        const quaternion_type &aOrientation = quaternion_type::identity);

    //! a shape's world-space axis aligned bounds
    struct shape_offsets final {
        vector3_type low;
        vector3_type high;
    };

    [[nodiscard]] shape_offsets shape_bounds(const shape_type &aShape,
        const quaternion_type &aOrientation = quaternion_type::identity);
}

#endif
