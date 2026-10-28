// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_COLLISION_TYPES_H
#define GDK_COLLISIONS_IMPL_COLLISION_TYPES_H

#include <gdk/collisions/types.h>

#include <cstdint>

namespace gdk::collisions {
    class impl_collider;

    using const_impl_collider_ptr_type = std::shared_ptr<const impl_collider>;
    using const_impl_collider_weak_ptr_type = std::weak_ptr<const impl_collider>;
    using impl_collider_ptr_type = std::shared_ptr<impl_collider>;
    using impl_collider_weak_ptr_type = std::weak_ptr<impl_collider>;
}

#endif

