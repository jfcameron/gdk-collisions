// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_CAPSULE_COLLIDER_H
#define GDK_COLLISIONS_IMPL_CAPSULE_COLLIDER_H

#include <gdk/collisions/capsule_collider.h>
#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_collider.h>

namespace gdk::collisions {
    class impl_capsule_collider final : public impl_collider, public capsule_collider {
    public:
        impl_capsule_collider(const impl_collision_policy &aPolicy, const response_handler &aResponseHandler,
            const floating_point_type aInverseOverlapWeight, const collider_id_type aId,
            const floating_point_type aRadius = 0.5f,
            const floating_point_type aHalfHeight = 0.5f);

        virtual ~impl_capsule_collider() = default;

        virtual void set_radius(const floating_point_type aRadius) override;
        [[nodiscard]] virtual floating_point_type radius() const override;

        virtual void set_half_height(const floating_point_type aHalfHeight) override;
        [[nodiscard]] virtual floating_point_type half_height() const override;
    };
}

#endif
