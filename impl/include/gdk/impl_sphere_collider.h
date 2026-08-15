// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_SPHERE_COLLIDER_H
#define GDK_COLLISION_IMPL_SPHERE_COLLIDER_H

#include <gdk/collision_response_handler.h>
#include <gdk/collision_types.h>
#include <gdk/impl_collider.h>

#include <gdk/sphere_collider.h>

#include <memory>

namespace gdk {
    class impl_sphere_collider final : public impl_collider, public sphere_collider 
{
    public:
        impl_sphere_collider(const impl_collision_policy &aPolicy, const collision_response_handler &aResponseHandler,
            const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId,
            const collision_floating_point_type aRadius = 0.5f);

        virtual ~impl_sphere_collider() = default;

        void set_radius(const collision_floating_point_type aRadius) override;
        [[nodiscard]] collision_floating_point_type radius() const override;

    private:

    };
}

#endif

