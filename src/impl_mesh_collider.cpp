// © Joseph Cameron - All Rights Reserved

#include <gdk/collision_exception.h>
#include <gdk/impl_mesh_collider.h>

using namespace gdk;

impl_mesh_collider::impl_mesh_collider(const impl_collision_policy &aPolicy,
    const collision_response_handler &aResponseHandler,
    const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, mesh_shape{})
{}

void impl_mesh_collider::set_mesh(const mesh_data_ptr_type &aMesh) {
    if (aMesh) {
        const auto pImplementation = std::dynamic_pointer_cast<const impl_mesh_data>(aMesh);

        if (!pImplementation)
            throw collision_exception("gdk::collision: mesh_data was not built by impl_mesh_data::make "
                "and cannot be used by this collision_scene implementation");

        std::get<mesh_shape>(primary_shape()).data = pImplementation;
    }
    else std::get<mesh_shape>(primary_shape()).data = nullptr;
}

mesh_data_ptr_type impl_mesh_collider::mesh() const {
    return std::get<mesh_shape>(primary_shape()).data;
}
