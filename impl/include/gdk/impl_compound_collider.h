// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_COMPOUND_COLLIDER_H
#define GDK_COLLISION_IMPL_COMPOUND_COLLIDER_H

#include <gdk/collision_response_handler.h>
#include <gdk/collision_types.h>
#include <gdk/compound_collider.h>
#include <gdk/impl_collider.h>

namespace gdk {
    class impl_compound_collider final : public impl_collider, public compound_collider {
    public:
        impl_compound_collider(const impl_collision_policy &aPolicy,
            const collision_response_handler &aResponseHandler,
            const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId);

        virtual ~impl_compound_collider() = default;

        virtual void add_sphere(const collision_vector3_type &aPosition,
            const collision_floating_point_type aRadius) override;

        virtual void add_box(const collision_vector3_type &aPosition,
            const collision_vector3_type &aHalfExtents) override;

        virtual void add_capsule(const collision_vector3_type &aPosition,
            const collision_quaternion_type &aRotation,
            const collision_floating_point_type aRadius,
            const collision_floating_point_type aHalfHeight) override;

        [[nodiscard]] virtual std::size_t part_count() const override;

        virtual void clear_parts() override;
    };
}

#endif
