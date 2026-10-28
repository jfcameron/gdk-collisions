// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_NARROW_PHASE_H
#define GDK_COLLISIONS_IMPL_NARROW_PHASE_H

#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_std_tuning.h>
#include <gdk/collisions/impl_shape.h>
#include <gdk/collisions/overlap.h>

#include <optional>
#include <vector>
#include <span>

namespace gdk::collisions {
    /// \brief is aSubject moving *toward* this contact, rather than resting against it or leaving it?
    [[nodiscard]] bool is_closing(const overlap &aOverlap, const shape_kinematics &aSubject,
        const shape_kinematics &aOther, const impl_std_tuning &aPolicy);

    /// \brief test one collider's geometry against another's for the coming timestep.
    [[nodiscard]] std::optional<overlap> narrow_phase_overlap(
        const shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const shape_type &aOtherShape, const shape_kinematics &aOther,
        const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy);

    /// \brief test two colliders' full geometry, part against part.
    [[nodiscard]] std::optional<overlap> narrow_phase_overlap_parts(
        std::span<const collider_part> aSubjectParts, const shape_kinematics &aSubject,
        std::span<const collider_part> aOtherParts, const shape_kinematics &aOther,
        const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy);
}

#endif
