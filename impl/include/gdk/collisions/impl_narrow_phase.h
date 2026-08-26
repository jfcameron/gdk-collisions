// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_NARROW_PHASE_H
#define GDK_COLLISIONS_IMPL_NARROW_PHASE_H

#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_shape.h>
#include <gdk/collisions/overlap.h>

#include <optional>
#include <vector>

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
        const std::vector<collider_part> &aSubjectParts, const shape_kinematics &aSubject,
        const std::vector<collider_part> &aOtherParts, const shape_kinematics &aOther,
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy);

    /// \brief gets the half extents of a shape's world-space axis aligned bounds
    [[nodiscard]] vector3_type shape_extents(const shape_type &aShape,
        const quaternion_type &aOrientation = quaternion_type::identity);
}

#endif
