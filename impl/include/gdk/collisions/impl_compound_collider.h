// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_COMPOUND_COLLIDER_H
#define GDK_COLLISIONS_IMPL_COMPOUND_COLLIDER_H

#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/compound_collider.h>
#include <gdk/collisions/impl_collider.h>

namespace gdk::collisions {
    class impl_compound_collider final : public impl_collider, public compound_collider {
    public:
        impl_compound_collider(const impl_collision_policy &aPolicy,
            const response_handler &aResponseHandler,
            const floating_point_type aInverseOverlapWeight, const collider_id_type aId);

        virtual ~impl_compound_collider() = default;

        virtual void add_sphere(const vector3_type &aPosition,
            const floating_point_type aRadius) override;

        virtual void add_box(const vector3_type &aPosition,
            const vector3_type &aHalfExtents) override;

        virtual void add_capsule(const vector3_type &aPosition,
            const quaternion_type &aRotation,
            const floating_point_type aRadius,
            const floating_point_type aHalfHeight) override;

        [[nodiscard]] virtual std::size_t part_count() const override;

        virtual void clear_parts() override;
    };
}

#endif
