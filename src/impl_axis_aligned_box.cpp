// © Joseph Cameron - All Rights Reserved

#include <gdk/collision_exception.h>
#include <gdk/impl_axis_aligned_box.h>
#include <gdk/impl_sphere_collider.h>
#include <gdk/overlap.h>

#include <algorithm>
#include <cassert>

using namespace gdk;

impl_axis_aligned_box_collider::impl_axis_aligned_box_collider(const impl_collision_policy &aPolicy, 
    const collision_response_handler &aCollisionHandler,
    const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId,
    const collision_vector3_type &aHalfExtents)
: impl_collider(aPolicy, aCollisionHandler, aInverseOverlapWeight, aId, box_shape{aHalfExtents})
{}

void impl_axis_aligned_box_collider::set_half_extents(const collision_vector3_type &aHalfExtents) {
    std::get<box_shape>(primary_shape()).half_extents = aHalfExtents;
}

collision_vector3_type impl_axis_aligned_box_collider::half_extents() const {
    return std::get<box_shape>(primary_shape()).half_extents;
}

