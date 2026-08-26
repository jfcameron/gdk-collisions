// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_COLLIDER_H
#define GDK_COLLISIONS_COLLIDER_H

#include <gdk/collisions/types.h>

namespace gdk::collisions {
    class collider {
    public:
        virtual ~collider() = default;

        /// \brief gets a copy of the transform
        [[nodiscard]] virtual matrix4x4_type transform() const = 0;

        /// \brief add a vector to the collider's velocity. 
        virtual void add_velocity(const vector3_type &aVelocity) = 0;

        /// \brief directly sets the velocity, this should be used with care
        virtual void set_velocity(const vector3_type &aVelocity) = 0;

        /// \brief gets a copy of the velocity
        virtual vector3_type velocity() const = 0;

        /// \brief move the collider to the given position. 
        virtual void set_position(const vector3_type &aPos) = 0;

        /// \brief set how readily this collider is displaced when it overlaps another.
        ///
        /// When two colliders interpenetrate they are pushed apart along the minimum translation
        /// vector. The amount that each collider is displaced by is based on the overlap weights of
        /// the two colliders involved.
        ///
        /// - `0` means immovable: it never absorbs any of a separation.
        /// - Equal weights split a separation evenly.
        /// - A smaller weight is displaced less.
        ///
        /// \warning Two overlapping colliders that both have a weight of 0 will never separate,
        /// because neither is permitted to move. 
        ///
        /// This is the only tuning that the collision resolution takes. Deciding what happens to a collider's
        /// *velocity* after a contact is the job of its response_handler, which is entirely user-definable.
        virtual void set_inverse_overlap_weight(const floating_point_type) = 0;

        /// \brief get the collider's inverse overlap weight
        virtual floating_point_type inverse_overlap_weight() const = 0;

        /// \brief make this collider move under its own control, unaffected by anything it meets.
        ///
        /// The third kind of body, alongside static and dynamic. A kinematic collider travels its
        /// full velocity every step no matter what is in the way, and resolution never displaces
        /// it, but it still blocks and displaces everything else and still reports contacts and
        /// trigger overlaps normally. Example uses: moving platforms, doors etc.
        virtual void set_kinematic(const bool aKinematic) = 0;

        /// \brief check if this collider is kinematic
        virtual bool is_kinematic() const = 0;

        /// \brief the velocity this collider finished its last update() with.
        [[nodiscard]] virtual vector3_type resolved_velocity() const = 0;

        /// \brief this collider's orientation within its scene.
        [[nodiscard]] virtual quaternion_type rotation() const = 0;

        /// \brief set the collider's orientation outright.
        virtual void set_rotation(const quaternion_type &aRotation) = 0;

        /// \brief turn the collider by aRotation on top of the orientation it already has in worldspace
        virtual void add_rotation(const quaternion_type &aRotation) = 0;

        /// \brief add to the collider's angular velocity in radians per second
        virtual void add_angular_velocity(const vector3_type &aAngularVelocity) = 0;

        /// \brief directly sets the angular velocity. this should be used with care
        virtual void set_angular_velocity(const vector3_type &aAngularVelocity) = 0;

        /// \brief gets a copy of the angular velocity in radians per second
        [[nodiscard]] virtual vector3_type angular_velocity() const = 0;
    };
}

#endif

