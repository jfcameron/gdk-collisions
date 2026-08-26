// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_COLLISION_TYPES_H
#define GDK_COLLISIONS_IMPL_COLLISION_TYPES_H

#include <gdk/collisions/types.h>

#include <cstdint>

namespace gdk::collisions {
    /// \brief unique identifier for a collider within a scene
    using collider_id_type = std::uint64_t;

    class impl_axis_aligned_box_collider;
    class impl_collider;
    struct impl_collision_policy;
    class impl_collision_scene;
    class impl_sphere_collider;

    using sweep_prune_scene_ptr_type = std::shared_ptr<impl_collision_scene>;
    using const_impl_collider_ptr_type = std::shared_ptr<const impl_collider>;
    using const_impl_collider_weak_ptr_type = std::weak_ptr<const impl_collider>;
    using impl_collider_ptr_type = std::shared_ptr<impl_collider>;
    using impl_collider_weak_ptr_type = std::weak_ptr<impl_collider>;
}

#endif

