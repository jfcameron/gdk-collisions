// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_HEIGHTFIELD_COLLIDER_H
#define GDK_COLLISION_IMPL_HEIGHTFIELD_COLLIDER_H

#include <gdk/collision_response_handler.h>
#include <gdk/collision_types.h>
#include <gdk/heightfield_collider.h>
#include <gdk/impl_collider.h>

namespace gdk {
    class impl_heightfield_collider final : public impl_collider, public heightfield_collider {
    public:
        impl_heightfield_collider(const impl_collision_policy &aPolicy,
            const collision_response_handler &aResponseHandler,
            const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId);

        virtual ~impl_heightfield_collider() = default;

        virtual void set_heightfield(const heightfield_data_ptr_type &aHeightfield) override;

        [[nodiscard]] virtual heightfield_data_ptr_type heightfield() const override;
    };
}

#endif
