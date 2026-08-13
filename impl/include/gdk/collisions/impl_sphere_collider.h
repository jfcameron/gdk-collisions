// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_SPHERE_COLLIDER_H
#define GDK_COLLISIONS_IMPL_SPHERE_COLLIDER_H

#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_collider.h>

#include <gdk/collisions/sphere_collider.h>

#include <memory>

namespace gdk::collisions {
    class impl_sphere_collider final : public impl_collider, public sphere_collider 
{
    public:
        impl_sphere_collider(const impl_collision_policy &aPolicy, const response_handler &aResponseHandler,
            const floating_point_type aInverseOverlapWeight, const collider_id_type aId,
            const floating_point_type aRadius = 0.5f);

        virtual ~impl_sphere_collider() = default;

        void set_radius(const floating_point_type aRadius) override;
        [[nodiscard]] floating_point_type radius() const override;

    private:

    };
}

#endif

