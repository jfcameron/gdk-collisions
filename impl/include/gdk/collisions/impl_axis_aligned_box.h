// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_AXIS_ALIGNED_BOX_H
#define GDK_COLLISIONS_IMPL_AXIS_ALIGNED_BOX_H

#include <gdk/collisions/box_collider.h>
#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_collider.h>

#include <memory>

namespace gdk::collisions {
    class impl_axis_aligned_box_collider final : public impl_collider, public box_collider {
    public:
        impl_axis_aligned_box_collider(const impl_collision_policy &aPolicy, const response_handler &aResponseHandler,
            const floating_point_type aInverseOverlapWeight, const collider_id_type aId,
            const vector3_type &aHalfExtents = {0.5, 0.5, 0.5});

        virtual ~impl_axis_aligned_box_collider() = default;

        virtual void set_half_extents(const vector3_type &aHalfExtents) override;
        [[nodiscard]] virtual vector3_type half_extents() const override;

    private:

    };
}

#endif
