// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_CAPSULE_COLLIDER_H
#define GDK_COLLISION_IMPL_CAPSULE_COLLIDER_H

#include <gdk/capsule_collider.h>
#include <gdk/collision_response_handler.h>
#include <gdk/collision_types.h>
#include <gdk/impl_collider.h>

namespace gdk {
    class impl_capsule_collider final : public impl_collider, public capsule_collider {
    public:
        impl_capsule_collider(const impl_collision_policy &aPolicy, const collision_response_handler &aResponseHandler,
            const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId,
            const collision_floating_point_type aRadius = 0.5f,
            const collision_floating_point_type aHalfHeight = 0.5f);

        virtual ~impl_capsule_collider() = default;

        virtual void set_radius(const collision_floating_point_type aRadius) override;
        [[nodiscard]] virtual collision_floating_point_type radius() const override;

        virtual void set_half_height(const collision_floating_point_type aHalfHeight) override;
        [[nodiscard]] virtual collision_floating_point_type half_height() const override;
    };
}

#endif
