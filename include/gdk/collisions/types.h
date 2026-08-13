// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_TYPES_H
#define GDK_COLLISIONS_TYPES_H

#include <gdk/math.h>

#include <cstdint>
#include <jfc/dispatch.h>

#include <functional>
#include <memory>

namespace gdk::collisions {
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
    class scene;
    struct overlap;
    struct contact;
    class sphere_collider;

    using collision_layers_type = std::uint32_t;

    inline constexpr collision_layers_type DEFAULT_COLLISION_LAYER = 1;

    inline constexpr collision_layers_type ALL_COLLISION_LAYERS = ~collision_layers_type{0};

    using delta_time_type = float;
    using floating_point_type = float; 

    using matrix4x4_type = matrix4x4<floating_point_type>;
    using quaternion_type = quaternion<floating_point_type>;
    using vector2_type = vector2<floating_point_type>;
    using vector3_type = vector3<floating_point_type>;
    using vector4_type = vector4<floating_point_type>;

    using chunk_type = jfc::dispatch::chunk_type;

    using task_dispatcher_type = jfc::dispatch::task_dispatcher_type;

    using box_collider_ptr_type = std::shared_ptr<box_collider>;
    using capsule_collider_ptr_type = std::shared_ptr<capsule_collider>;
    using collider_ptr_type = std::shared_ptr<collider>;
    using collider_weak_ptr_type = std::weak_ptr<collider>;
    using scene_ptr_type = std::shared_ptr<scene>;
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

