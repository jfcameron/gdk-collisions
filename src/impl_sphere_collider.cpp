// © Joseph Cameron - All Rights Reserved

#include <gdk/impl_sphere_collider.h>


using namespace gdk;

impl_sphere_collider::impl_sphere_collider(const impl_collision_policy &aPolicy, const collision_response_handler &aCollisionHandler,
    const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId,
    const collision_floating_point_type aRadius)
: impl_collider(aPolicy, aCollisionHandler, aInverseOverlapWeight, aId, sphere_shape{aRadius})
{}

void impl_sphere_collider::set_radius(const collision_floating_point_type aRadius) {
    std::get<sphere_shape>(primary_shape()).radius = aRadius;
}

collision_floating_point_type impl_sphere_collider::radius() const {
    return std::get<sphere_shape>(primary_shape()).radius;
}
