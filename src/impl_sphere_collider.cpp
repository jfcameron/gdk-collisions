// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/impl_sphere_collider.h>


using namespace gdk::collisions;

impl_sphere_collider::impl_sphere_collider(const impl_collision_policy &aPolicy, const response_handler &aCollisionHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId,
    const floating_point_type aRadius)
: impl_collider(aPolicy, aCollisionHandler, aInverseOverlapWeight, aId, sphere_shape{aRadius})
{}

void impl_sphere_collider::set_radius(const floating_point_type aRadius) {
    std::get<sphere_shape>(primary_shape()).radius = aRadius;
}

floating_point_type impl_sphere_collider::radius() const {
    return std::get<sphere_shape>(primary_shape()).radius;
}
