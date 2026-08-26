// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_RESPONSE_HANDLER_H
#define GDK_COLLISIONS_RESPONSE_HANDLER_H

#include <gdk/collisions/types.h>

#include <functional>
#include <vector>
#include <memory>

namespace gdk::collisions {
    /// \brief the surfaces a collider has already been redirected along during the current step.
    using clip_planes_type = std::vector<vector3_type>;

    /// \brief everything a response handler is told about one contact.
    struct contact_context final {
        /// \brief the collider that was hit
        const collider &other;

        /// \brief the overlap as the narrow phase reported it
        const overlap &result;

        /// \brief points from the other body toward the subject
        vector3_type normal;

        /// \brief the time this handler's answer will act over
        delta_time_type remaining_time;

        /// \brief surfaces already redirected along this step
        const clip_planes_type &clip_planes;

        /// \brief which advancement iteration of the current step this contact was found on
        unsigned short int iteration;

        /// \brief the scene's MIN_SWEPT_SPEED: below this a closing contact is beneath the loop's notice
        floating_point_type min_swept_speed;

        /// \brief the scene's NORMALIZATION_THRESHOLD: below this a vector is too short to normalize
        floating_point_type normalization_threshold;

        /// \brief the scene's CLIP_PLANE_PARALLEL_COSINE: |dot| at or beyond this counts as parallel
        floating_point_type clip_plane_parallel_cosine;
    };

    /// \brief defines how a collider's velocity is altered after overlap resolution.
    ///
    /// The returned vector is a **delta**, added to the collider's velocity, not the collider's new
    /// world velocity. Returning zero therefore leaves the collider untouched.
    using response_handler = std::function<
        vector3_type ( //velocity delta
            collider&, //aThis
            const contact_context& //aContact
    )>;

    /// \brief the response handlers provided by the library serve as living documentation and convenience.
    /// Collision response behavior is extremely game dependent, so response handlers are entirely user-definable.
    namespace response_handlers {
        /// \brief velocity is not altered at all after overlap resolution
        inline const response_handler null_opt = [](collider&, const contact_context&){
            return vector3_type::zero;
        };

        /// \brief redirect along the surface then restore the original speed
        [[nodiscard]] const response_handler &slide_preserving_speed();

        /// \brief redirect along the surface, keeping whatever speed survives the projection
        ///
        /// **Pairs with caller-owned velocity** -- a game accumulating gravity, applying friction and
        /// keeping momentum across frames, reading the result back through
        /// `collider::resolved_velocity()`. Projection alone is what lets a body actually come to
        /// rest on a slope: the game decides what the speed should be, and this does not argue.
        [[nodiscard]] const response_handler &slide_projecting();
    }
}

#endif

