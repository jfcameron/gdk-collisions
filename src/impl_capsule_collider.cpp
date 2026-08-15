// © Joseph Cameron - All Rights Reserved

#include <gdk/impl_capsule_collider.h>

using namespace gdk;

impl_capsule_collider::impl_capsule_collider(const impl_collision_policy &aPolicy,
    const collision_response_handler &aResponseHandler, const collision_floating_point_type aInverseOverlapWeight,
    const collider_id_type aId, const collision_floating_point_type aRadius,
    const collision_floating_point_type aHalfHeight)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, capsule_shape{aRadius, aHalfHeight})
{}

void impl_capsule_collider::set_radius(const collision_floating_point_type aRadius) {
    std::get<capsule_shape>(primary_shape()).radius = aRadius;
}

collision_floating_point_type impl_capsule_collider::radius() const {
    return std::get<capsule_shape>(primary_shape()).radius;
}

void impl_capsule_collider::set_half_height(const collision_floating_point_type aHalfHeight) {
    std::get<capsule_shape>(primary_shape()).half_height = aHalfHeight;
}

collision_floating_point_type impl_capsule_collider::half_height() const {
    return std::get<capsule_shape>(primary_shape()).half_height;
}
