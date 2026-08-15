// © Joseph Cameron - All Rights Reserved

#include <gdk/impl_obb_collider.h>

using namespace gdk;

impl_obb_collider::impl_obb_collider(const impl_collision_policy &aPolicy,
    const collision_response_handler &aResponseHandler, const collision_floating_point_type aInverseOverlapWeight,
    const collider_id_type aId, const collision_vector3_type &aHalfExtents)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, obb_shape{aHalfExtents})
{}

void impl_obb_collider::set_half_extents(const collision_vector3_type &aHalfExtents) {
    std::get<obb_shape>(primary_shape()).half_extents = aHalfExtents;
}

collision_vector3_type impl_obb_collider::half_extents() const {
    return std::get<obb_shape>(primary_shape()).half_extents;
}
