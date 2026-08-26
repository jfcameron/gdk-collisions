// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_axis_aligned_box.h>
#include <gdk/collisions/impl_sphere_collider.h>
#include <gdk/collisions/overlap.h>

#include <algorithm>
#include <cassert>

using namespace gdk::collisions;

impl_axis_aligned_box_collider::impl_axis_aligned_box_collider(const impl_collision_policy &aPolicy, 
    const response_handler &aCollisionHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId,
    const vector3_type &aHalfExtents)
: impl_collider(aPolicy, aCollisionHandler, aInverseOverlapWeight, aId, box_shape{aHalfExtents})
{}

void impl_axis_aligned_box_collider::set_half_extents(const vector3_type &aHalfExtents) {
    std::get<box_shape>(primary_shape()).half_extents = aHalfExtents;
}

vector3_type impl_axis_aligned_box_collider::half_extents() const {
    return std::get<box_shape>(primary_shape()).half_extents;
}

