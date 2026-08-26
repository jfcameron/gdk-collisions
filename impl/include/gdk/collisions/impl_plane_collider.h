// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_PLANE_COLLIDER_H
#define GDK_COLLISIONS_IMPL_PLANE_COLLIDER_H

#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/plane_collider.h>

namespace gdk::collisions {
    class impl_plane_collider final : public impl_collider, public plane_collider {
    public:
        impl_plane_collider(const impl_collision_policy &aPolicy,
            const response_handler &aResponseHandler,
            const floating_point_type aInverseOverlapWeight, const collider_id_type aId);

        virtual ~impl_plane_collider() = default;
    };
}

#endif
