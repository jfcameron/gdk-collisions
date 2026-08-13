// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/impl_capsule_collider.h>

using namespace gdk::collisions;

impl_capsule_collider::impl_capsule_collider(const impl_collision_policy &aPolicy,
    const response_handler &aResponseHandler, const floating_point_type aInverseOverlapWeight,
    const collider_id_type aId, const floating_point_type aRadius,
    const floating_point_type aHalfHeight)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, capsule_shape{aRadius, aHalfHeight})
{}

void impl_capsule_collider::set_radius(const floating_point_type aRadius) {
    std::get<capsule_shape>(primary_shape()).radius = aRadius;
}

floating_point_type impl_capsule_collider::radius() const {
    return std::get<capsule_shape>(primary_shape()).radius;
}

void impl_capsule_collider::set_half_height(const floating_point_type aHalfHeight) {
    std::get<capsule_shape>(primary_shape()).half_height = aHalfHeight;
}

floating_point_type impl_capsule_collider::half_height() const {
    return std::get<capsule_shape>(primary_shape()).half_height;
}
