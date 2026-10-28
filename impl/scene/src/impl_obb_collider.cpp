// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_obb_collider.h>

using namespace gdk::collisions;

impl_obb_collider::impl_obb_collider(const scene_policy &aPolicy,
    const response_handler &aResponseHandler, const floating_point_type aInverseOverlapWeight,
    const collider_id_type aId, const vector3_type &aHalfExtents)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, part_record{shape_kind::obb, aHalfExtents})
{}

void impl_obb_collider::set_half_extents(const vector3_type &aHalfExtents) {
    primary_part().size = aHalfExtents;
}

vector3_type impl_obb_collider::half_extents() const {
    return primary_part().size;
}
