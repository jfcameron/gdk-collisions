// © Joseph Cameron - MIT License

#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_aabb_collider.h>
#include <gdk/collisions/impl_sphere_collider.h>
#include <gdk/collisions/overlap.h>

#include <algorithm>
#include <cassert>

using namespace gdk::collisions;

impl_aabb_collider::impl_aabb_collider(const scene_policy &aPolicy, 
    const response_handler &aCollisionHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId,
    const vector3_type &aHalfExtents)
: impl_collider(aPolicy, aCollisionHandler, aInverseOverlapWeight, aId, part_record{shape_kind::aabb, aHalfExtents})
{}

void impl_aabb_collider::set_half_extents(const vector3_type &aHalfExtents) {
    primary_part().size = aHalfExtents;
}

vector3_type impl_aabb_collider::half_extents() const {
    return primary_part().size;
}

