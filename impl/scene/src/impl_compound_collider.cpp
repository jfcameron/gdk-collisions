// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_compound_collider.h>

using namespace gdk::collisions;

impl_compound_collider::impl_compound_collider(const scene_policy &aPolicy,
    const response_handler &aResponseHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId)
{}

void impl_compound_collider::add_sphere(const vector3_type &aPosition,
    const floating_point_type aRadius) {
    m_Parts.push_back(part_record{shape_kind::sphere, {aRadius, 0, 0}, aPosition});
}

void impl_compound_collider::add_box(const vector3_type &aPosition,
    const vector3_type &aHalfExtents) {
    m_Parts.push_back(part_record{shape_kind::obb, aHalfExtents, aPosition});
}

void impl_compound_collider::add_capsule(const vector3_type &aPosition,
    const quaternion_type &aRotation, const floating_point_type aRadius,
    const floating_point_type aHalfHeight) {
    m_Parts.push_back(part_record{shape_kind::capsule, {aRadius, aHalfHeight, 0}, aPosition, aRotation});
}

std::size_t impl_compound_collider::part_count() const {
    return m_Parts.size();
}

void impl_compound_collider::clear_parts() {
    m_Parts.clear();
}
