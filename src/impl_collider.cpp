// © Joseph Cameron - All Rights Reserved

#include <gdk/impl_collider.h>
#include <gdk/impl_narrow_phase.h>

#include <cmath>

using namespace gdk;

impl_collider::impl_collider(const impl_collision_policy &aPolicy, const collision_response_handler &aResponseHandler,
    const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId,
    const collision_shape_type &aShape)
: m_Parts{collider_part{aShape}}
, m_Policy(aPolicy)
, m_ResponseHandler(aResponseHandler)
, m_Id(aId)
, m_InverseOverlapWeight(aInverseOverlapWeight)
{}

collision_vector3_type impl_collider::respond_to_collision_resolution(
    const contact_context &aContact) {
    return m_ResponseHandler(*this, aContact);
}

collider_id_type impl_collider::id() const {
    return m_Id;
}

const std::vector<collider_part> &impl_collider::parts() const {
    return m_Parts;
}

void impl_collider::set_position(const collision_vector3_type &aPos) {
    m_Transform.set_translation(aPos);
}

void impl_collider::set_rotation(const collision_quaternion_type &aRotation) {
    const auto normalised = aRotation.normalized();

    m_Transform.set_rotation(normalised);
    m_Rotation = normalised;
}

void impl_collider::add_rotation(const collision_quaternion_type &aRotation) {
    set_rotation(aRotation * m_Rotation);
}

void impl_collider::add_angular_velocity(const collision_vector3_type &aAngularVelocity) {
    m_AngularVelocity += aAngularVelocity;
}

void impl_collider::set_angular_velocity(const collision_vector3_type &aAngularVelocity) {
    m_AngularVelocity = aAngularVelocity;
}

collision_vector3_type impl_collider::angular_velocity() const {
    return m_AngularVelocity;
}

void impl_collider::integrate_angular_velocity(const collision_delta_time_type aDeltaTime) {
    const auto rate = m_AngularVelocity.length();

    if (rate > m_Policy.MIN_SWEPT_SPEED) {
        const auto angle = rate * aDeltaTime;
        const auto axis = m_AngularVelocity * (1.0f / rate);
        const auto sine = std::sin(angle * 0.5f);

        collision_quaternion_type turn;
        turn.w = std::cos(angle * 0.5f);
        turn.x = axis.x * sine;
        turn.y = axis.y * sine;
        turn.z = axis.z * sine;

        add_rotation(turn);
    }
}

void impl_collider::clear_angular_velocity() {
    m_AngularVelocity = collision_vector3_type::zero;
}

void impl_collider::add_velocity(const collision_vector3_type &aVelocity) {
    m_Velocity += aVelocity;
}

void impl_collider::clear_velocity() {
    m_ResolvedVelocity = m_Velocity;
    m_Velocity = collision_vector3_type::zero;
}

collision_vector3_type impl_collider::resolved_velocity() const {
    return m_ResolvedVelocity;
}

void impl_collider::set_transform(const collision_matrix4x4_type &aTransform) {
    m_Transform = aTransform;
    m_Rotation = aTransform.rotation().normalized();
}

collision_matrix4x4_type impl_collider::transform() const { 
    return m_Transform; 
}

void impl_collider::set_velocity(const collision_vector3_type &aVelocity) {
    m_Velocity = aVelocity;
}

collision_vector3_type impl_collider::velocity() const {
    return m_Velocity;
}

collision_vector3_type impl_collider::position() const {
    return m_Transform.translation();
}

collision_quaternion_type impl_collider::rotation() const {
    return m_Rotation;
}

impl_collider::broadphase_bounds impl_collider::broad_phase_bounds() const {
    const auto orientation = rotation();
    const auto origin = position();

    auto low = collision_vector3_type::zero;
    auto high = collision_vector3_type::zero;
    auto first = true;

    for (const auto &part : m_Parts) {
        const auto centre = origin + rotate(orientation, part.position);
        const auto extents = shape_extents(part.shape, orientation * part.rotation);

        const auto partLow = centre - extents;
        const auto partHigh = centre + extents;

        low = first ? partLow : collision_vector3_type::min(low, partLow);
        high = first ? partHigh : collision_vector3_type::max(high, partHigh);
        first = false;
    }

    return broadphase_bounds{low, high};
}

impl_collider::broadphase_bounds impl_collider::broad_phase_swept_bounds(const collision_delta_time_type aDeltaTime) const {
    const auto bounds = broad_phase_bounds();
    const auto sweepSize = velocity() * aDeltaTime;

    broadphase_bounds out;
    out.min = collision_vector3_type::min(bounds.min, bounds.min + sweepSize);
    out.max = collision_vector3_type::max(bounds.max, bounds.max + sweepSize);
    return out;
}

void impl_collider::set_inverse_overlap_weight(const collision_floating_point_type aInverseOverlapWeight) {
    m_InverseOverlapWeight = aInverseOverlapWeight;
}

collision_floating_point_type impl_collider::inverse_overlap_weight() const {
    return m_InverseOverlapWeight;
}

void impl_collider::set_kinematic(const bool aKinematic) {
    m_Kinematic = aKinematic;
}

bool impl_collider::is_kinematic() const {
    return m_Kinematic;
}

shape_kinematics impl_collider::sweep_kinematics() const {
    return shape_kinematics{position(),
        m_Kinematic ? collision_vector3_type::zero : velocity(),
        rotation()};
}

collision_floating_point_type impl_collider::displacement_share() const {
    return m_Kinematic ? 0 : m_InverseOverlapWeight;
}

