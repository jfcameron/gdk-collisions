// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/impl_obb_collider.h>

using namespace gdk::collisions;

impl_obb_collider::impl_obb_collider(const impl_collision_policy &aPolicy,
    const response_handler &aResponseHandler, const floating_point_type aInverseOverlapWeight,
    const collider_id_type aId, const vector3_type &aHalfExtents)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, obb_shape{aHalfExtents})
{}

void impl_obb_collider::set_half_extents(const vector3_type &aHalfExtents) {
    std::get<obb_shape>(primary_shape()).half_extents = aHalfExtents;
}

vector3_type impl_obb_collider::half_extents() const {
    return std::get<obb_shape>(primary_shape()).half_extents;
}
