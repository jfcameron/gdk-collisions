// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_STD_TUNING_H
#define GDK_COLLISIONS_IMPL_STD_TUNING_H

#include <gdk/collisions/kernels/tables.h>
#include <gdk/collisions/std_kernels.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief what the std narrow phase reads: the scene's contact rules and the kernels' own tuning
    struct impl_std_tuning final {
        unsigned short int CONSERVATIVE_ADVANCEMENT_MAX_STEPS{8};
        delta_time_type TIME_OF_IMPACT_EPSILON{1e-7f};
        floating_point_type AXIS_ENTRY_EPSILON{1e-5f};
        floating_point_type MIN_SWEPT_SPEED{1e-3f};
        floating_point_type PENETRATION_DEPTH_EPSILON{1e-6f};
        floating_point_type NORMALIZATION_THRESHOLD{1e-6f};
        floating_point_type BARYCENTRIC_EDGE_EPSILON{1e-4f};
        floating_point_type FACE_CONTACT_COSINE_EPSILON{1e-4f};
        floating_point_type INTERNAL_EDGE_GHOST_COSINE{0.1f};
    };

    [[nodiscard]] inline impl_std_tuning tuning_of(const std_kernels_policy &aPolicy, const contact_rules &aRules) {
        return impl_std_tuning{aPolicy.SWEEP_MAX_STEPS, aRules.time_of_impact_epsilon, aRules.axis_entry_epsilon,
            aRules.min_swept_speed, aRules.penetration_depth_epsilon, aRules.normalization_threshold,
            aPolicy.BARYCENTRIC_EDGE_EPSILON, aPolicy.FACE_CONTACT_COSINE_EPSILON, aPolicy.INTERNAL_EDGE_GHOST_COSINE};
    }
}

#endif
