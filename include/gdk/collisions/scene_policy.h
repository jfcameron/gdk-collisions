// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_SCENE_POLICY_H
#define GDK_COLLISIONS_SCENE_POLICY_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief tuning for make_scene; the defaults suit bodies about a unit across
    struct scene_policy final {
        /// \brief how many equal steps one update is divided into
        unsigned short int STEPS_PER_UPDATE{1};

        /// \brief how many times a step may advance a body to its next contact
        unsigned short int CONSERVATIVE_ADVANCEMENT_MAX_STEPS{8};

        /// \brief minimal time step, to avoid infinite loops
        delta_time_type TIME_OF_IMPACT_EPSILON{1e-7f};

        /// \brief entry times this close count as simultaneous
        floating_point_type AXIS_ENTRY_EPSILON{1e-5f};

        /// \brief below this speed a body is treated as stationary
        floating_point_type MIN_SWEPT_SPEED{1e-3f};

        /// \brief minimum penetration depth
        floating_point_type PENETRATION_DEPTH_EPSILON{1e-6f};

        /// \brief used to determine when a vector is too small to reliably normalize
        floating_point_type NORMALIZATION_THRESHOLD{1e-6f};

        /// \brief how close |cos| between two clip planes must be to 1 for them to count as parallel
        floating_point_type CLIP_PLANE_PARALLEL_COSINE{0.9999995f};
    };
}

#endif
