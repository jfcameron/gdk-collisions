// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_plane_collider.h>

using namespace gdk::collisions;

impl_plane_collider::impl_plane_collider(const scene_policy &aPolicy,
    const response_handler &aResponseHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, part_record{shape_kind::plane})
{}
