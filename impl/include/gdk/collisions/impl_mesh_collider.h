// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_MESH_COLLIDER_H
#define GDK_COLLISIONS_IMPL_MESH_COLLIDER_H

#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/mesh_collider.h>

namespace gdk::collisions {
    class impl_mesh_collider final : public impl_collider, public mesh_collider {
    public:
        impl_mesh_collider(const impl_collision_policy &aPolicy,
            const response_handler &aResponseHandler,
            const floating_point_type aInverseOverlapWeight, const collider_id_type aId);

        virtual ~impl_mesh_collider() = default;

        virtual void set_mesh(const mesh_data_ptr_type &aMesh) override;

        [[nodiscard]] virtual mesh_data_ptr_type mesh() const override;
    };
}

#endif
