// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_narrow_phase.h>

#include <cmath>

using namespace gdk::collisions;

impl_collider::impl_collider(const impl_collision_policy &aPolicy, const response_handler &aResponseHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId,
    const shape_type &aShape)
: m_Parts{collider_part{aShape}}
, m_Policy(aPolicy)
, m_ResponseHandler(aResponseHandler)
, m_Id(aId)
, m_InverseOverlapWeight(aInverseOverlapWeight)
{}

vector3_type impl_collider::respond_to_collision_resolution(
    const contact_context &aContact) {
    return m_ResponseHandler(*this, aContact);
}

collider_id_type impl_collider::id() const {
    return m_Id;
}

const std::vector<collider_part> &impl_collider::parts() const {
    return m_Parts;
}

void impl_collider::set_position(const vector3_type &aPos) {
    m_Transform.set_translation(aPos);
}

void impl_collider::set_rotation(const quaternion_type &aRotation) {
    const auto normalised = aRotation.normalized();

    m_Transform.set_rotation(normalised);
    m_Rotation = normalised;
}

void impl_collider::add_rotation(const quaternion_type &aRotation) {
    set_rotation(aRotation * m_Rotation);
}

void impl_collider::add_angular_velocity(const vector3_type &aAngularVelocity) {
    m_AngularVelocity += aAngularVelocity;
}

void impl_collider::set_angular_velocity(const vector3_type &aAngularVelocity) {
    m_AngularVelocity = aAngularVelocity;
}

vector3_type impl_collider::angular_velocity() const {
    return m_AngularVelocity;
}

void impl_collider::integrate_angular_velocity(const delta_time_type aDeltaTime) {
    const auto rate = m_AngularVelocity.length();

    if (rate > m_Policy.MIN_SWEPT_SPEED) {
        const auto angle = rate * aDeltaTime;
        const auto axis = m_AngularVelocity * (1.0f / rate);
        const auto sine = std::sin(angle * 0.5f);

        quaternion_type turn;
        turn.w = std::cos(angle * 0.5f);
        turn.x = axis.x * sine;
        turn.y = axis.y * sine;
        turn.z = axis.z * sine;

        add_rotation(turn);
    }
}

void impl_collider::clear_angular_velocity() {
    m_AngularVelocity = vector3_type::zero;
}

void impl_collider::add_velocity(const vector3_type &aVelocity) {
    m_Velocity += aVelocity;
}

void impl_collider::clear_velocity() {
    m_ResolvedVelocity = m_Velocity;
    m_Velocity = vector3_type::zero;
}

vector3_type impl_collider::resolved_velocity() const {
    return m_ResolvedVelocity;
}

void impl_collider::set_transform(const matrix4x4_type &aTransform) {
    m_Transform = aTransform;
    m_Rotation = aTransform.rotation().normalized();
}

matrix4x4_type impl_collider::transform() const { 
    return m_Transform; 
}

void impl_collider::set_velocity(const vector3_type &aVelocity) {
    m_Velocity = aVelocity;
}

vector3_type impl_collider::velocity() const {
    return m_Velocity;
}

vector3_type impl_collider::position() const {
    return m_Transform.translation();
}

quaternion_type impl_collider::rotation() const {
    return m_Rotation;
}

impl_collider::broadphase_bounds impl_collider::broad_phase_bounds() const {
    const auto orientation = rotation();
    const auto origin = position();

    auto low = vector3_type::zero;
    auto high = vector3_type::zero;
    auto first = true;

    for (const auto &part : m_Parts) {
        const auto centre = origin + orientation * part.position;
        const auto extents = shape_extents(part.shape, orientation * part.rotation);

        const auto partLow = centre - extents;
        const auto partHigh = centre + extents;

        low = first ? partLow : vector3_type::min(low, partLow);
        high = first ? partHigh : vector3_type::max(high, partHigh);
        first = false;
    }

    return broadphase_bounds{low, high};
}

impl_collider::broadphase_bounds impl_collider::broad_phase_swept_bounds(const delta_time_type aDeltaTime) const {
    const auto bounds = broad_phase_bounds();
    const auto sweepSize = velocity() * aDeltaTime;

    broadphase_bounds out;
    out.min = vector3_type::min(bounds.min, bounds.min + sweepSize);
    out.max = vector3_type::max(bounds.max, bounds.max + sweepSize);
    return out;
}

void impl_collider::set_inverse_overlap_weight(const floating_point_type aInverseOverlapWeight) {
    m_InverseOverlapWeight = aInverseOverlapWeight;
}

floating_point_type impl_collider::inverse_overlap_weight() const {
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
        m_Kinematic ? vector3_type::zero : velocity(),
        rotation()};
}

floating_point_type impl_collider::displacement_share() const {
    return m_Kinematic ? 0 : m_InverseOverlapWeight;
}

