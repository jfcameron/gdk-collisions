// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_COLLISION_COLLIDER_H
#define GDK_COLLISIONS_IMPL_COLLISION_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_narrow_phase.h>
#include <gdk/collisions/impl_shape.h>
#include <gdk/collisions/impl_collision_types.h>

#include <optional>
#include <vector>

namespace gdk::collisions {
    /// \brief state and behaviour common to every collider implementation.
    class impl_collider : public virtual collider {
    public:
        struct broadphase_bounds {
            vector3_type min;
            vector3_type max;
        };

        virtual ~impl_collider() = default;

        /// \brief the collider's identity within its scene
        [[nodiscard]] collider_id_type id() const;

        /// \brief the collider's geometry.
        [[nodiscard]] const std::vector<collider_part> &parts() const;

        [[nodiscard]] virtual matrix4x4_type transform() const override;
        [[nodiscard]] virtual vector3_type velocity() const override;
        virtual floating_point_type inverse_overlap_weight() const override;
        virtual void add_velocity(const vector3_type &aVelocity) override;
        virtual void set_inverse_overlap_weight(const floating_point_type aInverseOverlapWeight) override;
        virtual void set_position(const vector3_type &aPos) override;
        virtual void set_velocity(const vector3_type &aVelocity) override;
        virtual void set_kinematic(const bool aKinematic) override;
        [[nodiscard]] virtual bool is_kinematic() const override;
        [[nodiscard]] virtual vector3_type resolved_velocity() const override;
        [[nodiscard]] virtual quaternion_type rotation() const override;
        virtual void set_rotation(const quaternion_type &aRotation) override;
        virtual void add_rotation(const quaternion_type &aRotation) override;

        virtual void add_angular_velocity(const vector3_type &aAngularVelocity) override;
        virtual void set_angular_velocity(const vector3_type &aAngularVelocity) override;
        [[nodiscard]] virtual vector3_type angular_velocity() const override;

        /// \brief turn by the angular velocity over aDeltaTime.
        void integrate_angular_velocity(const delta_time_type aDeltaTime);

        /// \brief zero the angular velocity
        void clear_angular_velocity();

        /// \brief the kinematics narrow phase should sweep this body along
        [[nodiscard]] shape_kinematics sweep_kinematics() const;

        /// \brief the share of a separation this body absorbs.
        [[nodiscard]] floating_point_type displacement_share() const;

        /// \brief ask this collider's response handler for its velocity delta after resolution.
        [[nodiscard]] vector3_type respond_to_collision_resolution(
            const contact_context &aContact);

        /// \brief sets the collider's velocity to zero
        void clear_velocity();

        /// \brief directly set the collider's transform matrix
        void set_transform(const matrix4x4_type &);

        /// \brief get the position of the collider within its scene
        [[nodiscard]] vector3_type position() const;

        /// \brief gets the bounds of the broadphase box
        [[nodiscard]] virtual broadphase_bounds broad_phase_bounds() const;

        /// \brief gets the swept bounds of the broadphase box
        [[nodiscard]] broadphase_bounds broad_phase_swept_bounds(const delta_time_type aDeltaTime) const;

    protected:
        impl_collider(const impl_collision_policy &, const response_handler &,
            const floating_point_type, const collider_id_type, const shape_type &);

        std::vector<collider_part> m_Parts;

        [[nodiscard]] shape_type &primary_shape() { return m_Parts.front().shape; }
        [[nodiscard]] const shape_type &primary_shape() const { return m_Parts.front().shape; }

        const impl_collision_policy m_Policy;

    private:
        response_handler m_ResponseHandler;
        const collider_id_type m_Id;
        matrix4x4_type m_Transform;

        quaternion_type m_Rotation = quaternion_type::identity;
        vector3_type m_Velocity;
        vector3_type m_AngularVelocity;
        floating_point_type m_InverseOverlapWeight = 0;
        bool m_Kinematic = false;
        vector3_type m_ResolvedVelocity;
    };
}

#endif
