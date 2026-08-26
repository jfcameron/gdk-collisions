// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_mesh_collider.h>

using namespace gdk::collisions;

impl_mesh_collider::impl_mesh_collider(const impl_collision_policy &aPolicy,
    const response_handler &aResponseHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, mesh_shape{})
{}

void impl_mesh_collider::set_mesh(const mesh_data_ptr_type &aMesh) {
    if (aMesh) {
        const auto pImplementation = std::dynamic_pointer_cast<const impl_mesh_data>(aMesh);

        if (!pImplementation)
            throw exception("gdk::collision: mesh_data was not built by impl_mesh_data::make "
                "and cannot be used by this scene implementation");

        std::get<mesh_shape>(primary_shape()).data = pImplementation;
    }
    else std::get<mesh_shape>(primary_shape()).data = nullptr;
}

mesh_data_ptr_type impl_mesh_collider::mesh() const {
    return std::get<mesh_shape>(primary_shape()).data;
}
