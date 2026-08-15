// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_TYPES_H
#define GDK_COLLISION_TYPES_H

#include <gdk/math.h>

#include <functional>
#include <memory>

namespace gdk {
    class box_collider;
    class capsule_collider;
    class compound_collider;
    class heightfield_collider;
    class heightfield_data;
    class mesh_collider;
    class mesh_data;
    class plane_collider;
    class obb_collider;
    class collider;
    class collision_scene;
    struct overlap;
    struct contact;
    class sphere_collider;

    using collision_delta_time_type = float;
    using collision_floating_point_type = float; //TODO: possibly too broad? component_type? scalar_type?

    using collision_matrix4x4_type = matrix4x4<collision_floating_point_type>;
    using collision_quaternion_type = quaternion<collision_floating_point_type>;
    using collision_vector2_type = vector2<collision_floating_point_type>;
    using collision_vector3_type = vector3<collision_floating_point_type>;
    using collision_vector4_type = vector4<collision_floating_point_type>;

    /// \brief a chunk of parallelizable work
    using collision_chunk_type = std::function<void(const std::size_t &aChunkIndex)>;
    /// \brief how a caller lends the library its threads. This is optional. If you do not make use
    /// of it, the library will simply work in a single threaded manner.
    ///
    /// **The library creates no threads and owns no queue.** It splits its parallelisable work into
    /// `aChunkCount` independent chunks and asks the caller to run `aChunk(i)` for every `i` in
    /// `[0, aChunkCount)`, on whatever threads the caller likes, returning only once all of them
    /// have finished. Simple implementation example:
    ///
    /// \code
    /// [](std::size_t aChunkCount, const gdk::collision_chunk_type &aChunk) {
    ///     for (std::size_t i = 0; i < aChunkCount; ++i) aChunk(i);
    /// }
    /// \endcode
    using collision_task_dispatcher_type = std::function<void(const std::size_t &aChunkCount, const collision_chunk_type &aChunk)>;

    using box_collider_ptr_type = std::shared_ptr<box_collider>;
    using capsule_collider_ptr_type = std::shared_ptr<capsule_collider>;
    using collider_ptr_type = std::shared_ptr<collider>;
    using collider_weak_ptr_type = std::weak_ptr<collider>;
    using collision_scene_ptr_type = std::shared_ptr<collision_scene>;
    using compound_collider_ptr_type = std::shared_ptr<compound_collider>;
    using const_box_collider_ptr_type = std::shared_ptr<const box_collider>;
    using const_capsule_collider_ptr_type = std::shared_ptr<const capsule_collider>;
    using const_collider_ptr_type = std::shared_ptr<const collider>;
    using const_collider_weak_ptr_type = std::weak_ptr<const collider>;
    using const_compound_collider_ptr_type = std::shared_ptr<const compound_collider>;
    using const_heightfield_collider_ptr_type = std::shared_ptr<const heightfield_collider>;
    using const_mesh_collider_ptr_type = std::shared_ptr<const mesh_collider>;
    using const_obb_collider_ptr_type = std::shared_ptr<const obb_collider>;
    using const_plane_collider_ptr_type = std::shared_ptr<const plane_collider>;
    using const_sphere_collider_ptr_type = std::shared_ptr<const sphere_collider>;
    using heightfield_collider_ptr_type = std::shared_ptr<heightfield_collider>;
    using heightfield_data_ptr_type = std::shared_ptr<const heightfield_data>;
    using mesh_collider_ptr_type = std::shared_ptr<mesh_collider>;
    using mesh_data_ptr_type = std::shared_ptr<const mesh_data>;
    using obb_collider_ptr_type = std::shared_ptr<obb_collider>;
    using sphere_collider_ptr_type = std::shared_ptr<sphere_collider>;
    using sphere_collider_weak_ptr_type = std::weak_ptr<sphere_collider>;
}

#endif

