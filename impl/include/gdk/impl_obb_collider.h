// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_OBB_COLLIDER_H
#define GDK_COLLISION_IMPL_OBB_COLLIDER_H

#include <gdk/collision_response_handler.h>
#include <gdk/collision_types.h>
#include <gdk/impl_collider.h>
#include <gdk/obb_collider.h>

namespace gdk {
    class impl_obb_collider final : public impl_collider, public obb_collider {
    public:
        impl_obb_collider(const impl_collision_policy &aPolicy, const collision_response_handler &aResponseHandler,
            const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId,
            const collision_vector3_type &aHalfExtents = {0.5, 0.5, 0.5});

        virtual ~impl_obb_collider() = default;

        virtual void set_half_extents(const collision_vector3_type &aHalfExtents) override;
        [[nodiscard]] virtual collision_vector3_type half_extents() const override;
    };
}

#endif
