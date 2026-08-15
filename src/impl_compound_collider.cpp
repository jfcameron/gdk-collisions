// © Joseph Cameron - All Rights Reserved

#include <gdk/impl_compound_collider.h>

using namespace gdk;

namespace {
    const collision_floating_point_type EMPTY_PLACEHOLDER_RADIUS = 0.0f;
}

impl_compound_collider::impl_compound_collider(const impl_collision_policy &aPolicy,
    const collision_response_handler &aResponseHandler,
    const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId,
    sphere_shape{EMPTY_PLACEHOLDER_RADIUS})
{}

void impl_compound_collider::add_sphere(const collision_vector3_type &aPosition,
    const collision_floating_point_type aRadius) {
    if (part_count() == 0) m_Parts.clear();
    m_Parts.push_back(collider_part{sphere_shape{aRadius}, aPosition, collision_quaternion_type::identity});
}

void impl_compound_collider::add_box(const collision_vector3_type &aPosition,
    const collision_vector3_type &aHalfExtents) {
    if (part_count() == 0) m_Parts.clear();
    m_Parts.push_back(collider_part{obb_shape{aHalfExtents}, aPosition, collision_quaternion_type::identity});
}

void impl_compound_collider::add_capsule(const collision_vector3_type &aPosition,
    const collision_quaternion_type &aRotation, const collision_floating_point_type aRadius,
    const collision_floating_point_type aHalfHeight) {
    if (part_count() == 0) m_Parts.clear();
    m_Parts.push_back(collider_part{capsule_shape{aRadius, aHalfHeight}, aPosition, aRotation});
}

std::size_t impl_compound_collider::part_count() const {
    if (m_Parts.size() == 1) {
        const auto *pSphere = std::get_if<sphere_shape>(&m_Parts.front().shape);
        if (pSphere && pSphere->radius == EMPTY_PLACEHOLDER_RADIUS) return 0;
    }
    return m_Parts.size();
}

void impl_compound_collider::clear_parts() {
    m_Parts.clear();
    m_Parts.push_back(collider_part{sphere_shape{EMPTY_PLACEHOLDER_RADIUS}});
}
