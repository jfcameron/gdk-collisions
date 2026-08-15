// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_IMPL_MESH_COLLIDER_H
#define GDK_COLLISION_IMPL_MESH_COLLIDER_H

#include <gdk/collision_response_handler.h>
#include <gdk/collision_types.h>
#include <gdk/impl_collider.h>
#include <gdk/mesh_collider.h>

namespace gdk {
    class impl_mesh_collider final : public impl_collider, public mesh_collider {
    public:
        impl_mesh_collider(const impl_collision_policy &aPolicy,
            const collision_response_handler &aResponseHandler,
            const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId);

        virtual ~impl_mesh_collider() = default;

        virtual void set_mesh(const mesh_data_ptr_type &aMesh) override;

        [[nodiscard]] virtual mesh_data_ptr_type mesh() const override;
    };
}

#endif
