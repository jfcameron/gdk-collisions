// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_AXIS_ALIGNED_BOX_H
#define GDK_COLLISION_IMPL_AXIS_ALIGNED_BOX_H

#include <gdk/box_collider.h>
#include <gdk/collision_response_handler.h>
#include <gdk/collision_types.h>
#include <gdk/impl_collider.h>

#include <memory>

namespace gdk {
    class impl_axis_aligned_box_collider final : public impl_collider, public box_collider {
    public:
        impl_axis_aligned_box_collider(const impl_collision_policy &aPolicy, const collision_response_handler &aResponseHandler,
            const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId,
            const collision_vector3_type &aHalfExtents = {0.5, 0.5, 0.5});

        virtual ~impl_axis_aligned_box_collider() = default;

        virtual void set_half_extents(const collision_vector3_type &aHalfExtents) override;
        [[nodiscard]] virtual collision_vector3_type half_extents() const override;

    private:

    };
}

#endif
