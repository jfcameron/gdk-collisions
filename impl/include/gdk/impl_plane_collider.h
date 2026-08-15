// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_PLANE_COLLIDER_H
#define GDK_COLLISION_IMPL_PLANE_COLLIDER_H

#include <gdk/collision_response_handler.h>
#include <gdk/collision_types.h>
#include <gdk/impl_collider.h>
#include <gdk/plane_collider.h>

namespace gdk {
    class impl_plane_collider final : public impl_collider, public plane_collider {
    public:
        impl_plane_collider(const impl_collision_policy &aPolicy,
            const collision_response_handler &aResponseHandler,
            const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId);

        virtual ~impl_plane_collider() = default;
    };
}

#endif
