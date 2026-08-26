// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_COLLISION_EVENT_TRACKER_H
#define GDK_COLLISIONS_IMPL_COLLISION_EVENT_TRACKER_H

#include <gdk/collisions/events.h>
#include <gdk/collisions/impl_collision_types.h>

#include <functional>
#include <vector>

namespace gdk::collisions {
    /// \brief accumulates the contacts detected during a step and turns them into enter/stay/exit
    /// events by diffing against the previous step.
    class impl_collision_event_tracker final {
    public:
        using observer_type = std::function<void(collision_event)>;
        using trigger_overlap_observer_type = std::function<void(trigger_event)>;

        impl_collision_event_tracker(observer_type aCollisionObserver,
            trigger_overlap_observer_type aTriggerObserver);

        /// \brief record that two colliders are in contact during the current step
        void record_collision(const impl_collider_ptr_type &aA, const impl_collider_ptr_type &aB,
            const vector3_type &aNormal, const vector3_type &aPoint,
            const floating_point_type aPenetration);

        /// \brief record that two trigger volumes are overlapping during the current step
        void record_trigger(const impl_collider_ptr_type &aA, const impl_collider_ptr_type &aB);

        /// \brief work out enter/stay for everything recorded this step and exit for everything
        void flush();

        /// \brief hand everything queued so far to the observers
        void dispatch();

    private:
        /// \brief identifies a contact independently of the order its participants were passed in
        struct contact_key final {
            collider_id_type low;
            collider_id_type high;

            bool operator<(const contact_key &aOther) const noexcept {
                return low != aOther.low ? low < aOther.low : high < aOther.high;
            }

            bool operator==(const contact_key &aOther) const noexcept {
                return low == aOther.low && high == aOther.high;
            }
        };

        /// \brief contains a contact and its participants
        struct tracked_contact final {
            contact_key key;
            const_collider_weak_ptr_type a;
            const_collider_weak_ptr_type b;

            vector3_type normal = vector3_type::zero;
            vector3_type point = vector3_type::zero;
            floating_point_type penetration = {0};
        };

        using contact_collection_type = std::vector<tracked_contact>;

        static void record(contact_collection_type &aInto, const impl_collider_ptr_type &aA,
            const impl_collider_ptr_type &aB, const vector3_type &aNormal,
            const vector3_type &aPoint, const floating_point_type aPenetration);

        static void prepare(contact_collection_type &aContacts);

        template <typename event_type>
        static void emit(const contact_collection_type &aCurrent, const contact_collection_type &aPrevious,
            std::vector<event_type> &aInto);

        observer_type m_CollisionObserver;
        trigger_overlap_observer_type m_TriggerOverlapObserver;

        std::vector<collision_event> m_PendingCollisions;
        std::vector<trigger_event> m_PendingTriggers;

    public:
        /// \brief get the collisions of the most recently completed step, sorted by key and deduplicated
        [[nodiscard]] const contact_collection_type &last_step_collisions() const {
            return m_PreviousCollisions;
        }

    private:
        contact_collection_type m_Collisions;
        contact_collection_type m_Triggers;
        contact_collection_type m_PreviousCollisions;
        contact_collection_type m_PreviousTriggers;
    };
}

#endif
