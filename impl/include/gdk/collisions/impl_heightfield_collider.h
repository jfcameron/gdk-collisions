// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_HEIGHTFIELD_COLLIDER_H
#define GDK_COLLISIONS_IMPL_HEIGHTFIELD_COLLIDER_H

#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/heightfield_collider.h>
#include <gdk/collisions/impl_collider.h>

namespace gdk::collisions {
    class impl_heightfield_collider final : public impl_collider, public heightfield_collider {
    public:
        impl_heightfield_collider(const impl_collision_policy &aPolicy,
            const response_handler &aResponseHandler,
            const floating_point_type aInverseOverlapWeight, const collider_id_type aId);

        virtual ~impl_heightfield_collider() = default;

        virtual void set_heightfield(const heightfield_data_ptr_type &aHeightfield) override;

        [[nodiscard]] virtual heightfield_data_ptr_type heightfield() const override;
    };
}

#endif
