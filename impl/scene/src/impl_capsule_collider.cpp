// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_capsule_collider.h>

using namespace gdk::collisions;

impl_capsule_collider::impl_capsule_collider(const scene_policy &aPolicy,
    const response_handler &aResponseHandler, const floating_point_type aInverseOverlapWeight,
    const collider_id_type aId, const floating_point_type aRadius,
    const floating_point_type aHalfHeight)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId,
    part_record{shape_kind::capsule, {aRadius, aHalfHeight, 0}})
{}

void impl_capsule_collider::set_radius(const floating_point_type aRadius) {
    primary_part().size.x = aRadius;
}

floating_point_type impl_capsule_collider::radius() const {
    return primary_part().size.x;
}

void impl_capsule_collider::set_half_height(const floating_point_type aHalfHeight) {
    primary_part().size.y = aHalfHeight;
}

floating_point_type impl_capsule_collider::half_height() const {
    return primary_part().size.y;
}
