// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_COLLISION_POLICY_H
#define GDK_COLLISIONS_IMPL_COLLISION_POLICY_H

#include <gdk/collisions/types.h>

#include <cstddef>

namespace gdk::collisions {
    /// \brief user-definable configuration for the implementation
    struct impl_collision_policy final {
        const unsigned short int STEPS_PER_UPDATE{1};

        /// \brief minimal time step, to avoid infinite loops
        const delta_time_type TIME_OF_IMPACT_EPSILON{1e-7f};

        /// \brief maximum number of iterations during conservative advancement loops, to prevent excessive iteration
        const unsigned short int CONSERVATIVE_ADVANCEMENT_MAX_STEPS{8};

        /// \brief size of a broadphase cell in world units.
        const floating_point_type BROAD_PHASE_CELL_SIZE{2.0f};

        /// \brief how many cells one body may occupy before it stops being binned at all.
        const std::size_t BROAD_PHASE_MAX_CELLS_PER_BODY{512};

        /// \brief how many dynamic bodies one parallel detection chunk covers.
        const std::size_t PARALLEL_DETECT_CHUNK_SIZE{64};

        /// \brief visit bodies in spatial order during the detect pass rather than creation order
        const bool SPATIAL_DETECT_ORDER{false};

        /// \brief minimum per-axis entry depth
        const floating_point_type AXIS_ENTRY_EPSILON{1e-5f};

        /// \brief below this speed a body is treated as stationary and unswept tests are used.
        const floating_point_type MIN_SWEPT_SPEED{1e-3f};

        /// \brief MIN_SWEPT_SPEED squared, for comparing against length_squared() and dot(v,v)
        const floating_point_type MIN_SWEPT_SPEED_SQUARED{1e-6f};

        /// \brief minimum penetration depth
        const floating_point_type PENETRATION_DEPTH_EPSILON{1e-6f};

        /// \brief used to determine when a vector is too small to reliably normalize
        const floating_point_type NORMALIZATION_THRESHOLD{1e-6f};

        /// \brief how close |cos| between two clip planes must be to 1 for them to count as parallel
        const floating_point_type CLIP_PLANE_PARALLEL_COSINE{0.9999995f};

        /// \brief how close to a triangle's edge a contact point must be to count as *on* it.
        const floating_point_type BARYCENTRIC_EDGE_EPSILON{1e-4f};

        /// \brief how nearly a contact normal must match a face normal to count as a face contact.
        const floating_point_type FACE_CONTACT_COSINE_EPSILON{1e-4f};

        /// \brief how nearly edge on a contact must be for its triangle to be disbelieved
        const floating_point_type INTERNAL_EDGE_GHOST_COSINE{0.1f};

        /// \brief minimum offset applied after collision resolution
        const floating_point_type POST_RESOLUTION_SEPARATION_EPSILON{1e-3f};
    };
}

#endif
