// © Joseph Cameron - All Rights Reserved

#ifndef GDK_IMPL_COLLISION_COLLIDER_H
#define GDK_IMPL_COLLISION_COLLIDER_H

#include <gdk/collider.h>
#include <gdk/collision_response_handler.h>
#include <gdk/impl_collision_policy.h>
#include <gdk/impl_narrow_phase.h>
#include <gdk/impl_shape.h>
#include <gdk/impl_collision_types.h>

#include <optional>
#include <vector>

namespace gdk {
    /// \brief state and behaviour common to every collider implementation.
    class impl_collider : public virtual collider {
    public:
        struct broadphase_bounds {
            collision_vector3_type min;
            collision_vector3_type max;
        };

        virtual ~impl_collider() = default;

        /// \brief the collider's identity within its scene
        [[nodiscard]] collider_id_type id() const;

        /// \brief the collider's geometry.
        [[nodiscard]] const std::vector<collider_part> &parts() const;

        [[nodiscard]] virtual collision_matrix4x4_type transform() const override;
        [[nodiscard]] virtual collision_vector3_type velocity() const override;
        virtual collision_floating_point_type inverse_overlap_weight() const override;
        virtual void add_velocity(const collision_vector3_type &aVelocity) override;
        virtual void set_inverse_overlap_weight(const collision_floating_point_type aInverseOverlapWeight) override;
        virtual void set_position(const collision_vector3_type &aPos) override;
        virtual void set_velocity(const collision_vector3_type &aVelocity) override;
        virtual void set_kinematic(const bool aKinematic) override;
        [[nodiscard]] virtual bool is_kinematic() const override;
        [[nodiscard]] virtual collision_vector3_type resolved_velocity() const override;
        [[nodiscard]] virtual collision_quaternion_type rotation() const override;
        virtual void set_rotation(const collision_quaternion_type &aRotation) override;
        virtual void add_rotation(const collision_quaternion_type &aRotation) override;

        virtual void add_angular_velocity(const collision_vector3_type &aAngularVelocity) override;
        virtual void set_angular_velocity(const collision_vector3_type &aAngularVelocity) override;
        [[nodiscard]] virtual collision_vector3_type angular_velocity() const override;

        /// \brief turn by the angular velocity over aDeltaTime.
        void integrate_angular_velocity(const collision_delta_time_type aDeltaTime);

        /// \brief zero the angular velocity
        void clear_angular_velocity();

        /// \brief the kinematics narrow phase should sweep this body along
        [[nodiscard]] shape_kinematics sweep_kinematics() const;

        /// \brief the share of a separation this body absorbs.
        [[nodiscard]] collision_floating_point_type displacement_share() const;

        /// \brief ask this collider's response handler for its velocity delta after resolution.
        [[nodiscard]] collision_vector3_type respond_to_collision_resolution(
            const contact_context &aContact);

        /// \brief sets the collider's velocity to zero
        void clear_velocity();

        /// \brief directly set the collider's transform matrix
        void set_transform(const collision_matrix4x4_type &);

        /// \brief get the position of the collider within its scene
        [[nodiscard]] collision_vector3_type position() const;

        /// \brief gets the bounds of the broadphase box
        [[nodiscard]] virtual broadphase_bounds broad_phase_bounds() const;

        /// \brief gets the swept bounds of the broadphase box
        [[nodiscard]] broadphase_bounds broad_phase_swept_bounds(const collision_delta_time_type aDeltaTime) const;

    protected:
        impl_collider(const impl_collision_policy &, const collision_response_handler &,
            const collision_floating_point_type, const collider_id_type, const collision_shape_type &);

        std::vector<collider_part> m_Parts;

        [[nodiscard]] collision_shape_type &primary_shape() { return m_Parts.front().shape; }
        [[nodiscard]] const collision_shape_type &primary_shape() const { return m_Parts.front().shape; }

        const impl_collision_policy m_Policy;

    private:
        collision_response_handler m_ResponseHandler;
        const collider_id_type m_Id;
        collision_matrix4x4_type m_Transform;

        collision_quaternion_type m_Rotation = collision_quaternion_type::identity;
        collision_vector3_type m_Velocity;
        collision_vector3_type m_AngularVelocity;
        collision_floating_point_type m_InverseOverlapWeight = 0;
        bool m_Kinematic = false;
        collision_vector3_type m_ResolvedVelocity;
    };
}

#endif
