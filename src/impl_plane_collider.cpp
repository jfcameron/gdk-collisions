// © Joseph Cameron - All Rights Reserved

#include <gdk/impl_plane_collider.h>

using namespace gdk;

impl_plane_collider::impl_plane_collider(const impl_collision_policy &aPolicy,
    const collision_response_handler &aResponseHandler,
    const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, plane_shape{})
{}
