// © Joseph Cameron - All Rights Reserved

#ifndef GDK_IMPL_NARROW_PHASE_H
#define GDK_IMPL_NARROW_PHASE_H

#include <gdk/collision_types.h>
#include <gdk/impl_collision_policy.h>
#include <gdk/impl_shape.h>
#include <gdk/overlap.h>

#include <optional>
#include <vector>

namespace gdk {
    /// \brief the kinematic state a narrow-phase test needs about one participant
    struct shape_kinematics final {
        collision_vector3_type position;
        collision_vector3_type velocity;

        /// \brief the collider's orientation. Identity for shapes that are rotation-invariant or
        /// axis-aligned by definition; a capsule's interior segment is rotated by it.
        collision_quaternion_type orientation = collision_quaternion_type::identity;
    };

    /// \brief is aSubject moving *toward* this contact, rather than resting against it or leaving it?
    [[nodiscard]] bool is_closing(const overlap &aOverlap, const shape_kinematics &aSubject,
        const shape_kinematics &aOther, const impl_collision_policy &aPolicy);

    /// \brief test one collider's geometry against another's for the coming timestep.
    [[nodiscard]] std::optional<overlap> narrow_phase_overlap(
        const collision_shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const collision_shape_type &aOtherShape, const shape_kinematics &aOther,
        const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy);

    /// \brief how deeply two shapes overlap *right now*, with no regard for motion.
    [[nodiscard]] std::optional<overlap> narrow_phase_penetration(
        const collision_shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const collision_shape_type &aOtherShape, const shape_kinematics &aOther,
        const impl_collision_policy &aPolicy);

    /// \brief test two colliders' full geometry, part against part.
    [[nodiscard]] std::optional<overlap> narrow_phase_overlap_parts(
        const std::vector<collider_part> &aSubjectParts, const shape_kinematics &aSubject,
        const std::vector<collider_part> &aOtherParts, const shape_kinematics &aOther,
        const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy);

    /// \brief gets the half extents of a shape's world-space axis aligned bounds
    [[nodiscard]] collision_vector3_type shape_extents(const collision_shape_type &aShape,
        const collision_quaternion_type &aOrientation = collision_quaternion_type::identity);
}

#endif
