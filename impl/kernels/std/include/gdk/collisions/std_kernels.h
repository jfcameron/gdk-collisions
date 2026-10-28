// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_STD_KERNELS_H
#define GDK_COLLISIONS_STD_KERNELS_H

#include <gdk/collisions/kernels/scene_kernels.h>
#include <gdk/collisions/types.h>

#include <cstddef>

namespace gdk::collisions {
    /// \brief tuning for make_std_kernels
    struct std_kernels_policy final {
        /// \brief size of a broadphase cell in world units
        floating_point_type BROAD_PHASE_CELL_SIZE{2.0f};

        /// \brief how many cells one body may occupy before it stops being binned at all
        std::size_t BROAD_PHASE_MAX_CELLS_PER_BODY{512};

        /// \brief how many subjects or tests one parallel chunk covers
        std::size_t PARALLEL_CHUNK_SIZE{64};

        /// \brief the fewest subjects or tests handed to the dispatcher; fewer run serially
        std::size_t PARALLEL_MIN_WORK{80};

        /// \brief how many distance queries one swept test may take
        unsigned short int SWEEP_MAX_STEPS{8};

        /// \brief how close to a triangle's edge a contact point must be to count as on it
        floating_point_type BARYCENTRIC_EDGE_EPSILON{1e-4f};

        /// \brief how nearly a contact normal must match a face normal to count as a face contact
        floating_point_type FACE_CONTACT_COSINE_EPSILON{1e-4f};

        /// \brief how nearly edge on a contact must be for its triangle to be disbelieved
        floating_point_type INTERNAL_EDGE_GHOST_COSINE{0.1f};
    };

    /// \brief uniform grids, bounding volume hierarchies and the library's own narrow phase, on the standard library.
    /// An empty dispatcher runs every call serially.
    /// \exception exception if aPolicy holds a value the kernels cannot run with
    [[nodiscard]] scene_kernels_ptr_type make_std_kernels(std_kernels_policy aPolicy = {},
        task_dispatcher_type aDispatcher = {});
}

#endif
