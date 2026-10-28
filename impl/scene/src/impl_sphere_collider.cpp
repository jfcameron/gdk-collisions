// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_sphere_collider.h>


using namespace gdk::collisions;

impl_sphere_collider::impl_sphere_collider(const scene_policy &aPolicy, const response_handler &aCollisionHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId,
    const floating_point_type aRadius)
: impl_collider(aPolicy, aCollisionHandler, aInverseOverlapWeight, aId, part_record{shape_kind::sphere, {aRadius, 0, 0}})
{}

void impl_sphere_collider::set_radius(const floating_point_type aRadius) {
    primary_part().size.x = aRadius;
}

floating_point_type impl_sphere_collider::radius() const {
    return primary_part().size.x;
}
