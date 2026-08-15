// © Joseph Cameron - All Rights Reserved

#include <gdk/impl_collider.h>
#include <gdk/impl_collision_event_tracker.h>

#include <algorithm>
#include <utility>

using namespace gdk;

impl_collision_event_tracker::impl_collision_event_tracker(collision_observer_type aCollisionObserver,
    trigger_overlap_observer_type aTriggerObserver)
: m_CollisionObserver(std::move(aCollisionObserver))
, m_TriggerOverlapObserver(std::move(aTriggerObserver))
{}

void impl_collision_event_tracker::record(contact_collection_type &aInto, const impl_collider_ptr_type &aA,
    const impl_collider_ptr_type &aB, const collision_vector3_type &aNormal,
    const collision_vector3_type &aPoint, const collision_floating_point_type aPenetration) {
    auto low = aA;
    auto high = aB;
    auto normal = aNormal;

    if (high->id() < low->id()) {
        std::swap(low, high);
        normal = normal * -1.0f;
    }

    aInto.push_back(tracked_contact{contact_key{low->id(), high->id()}, low, high, normal, aPoint,
        aPenetration});
}

void impl_collision_event_tracker::prepare(contact_collection_type &aContacts) {
    std::stable_sort(aContacts.begin(), aContacts.end(),
        [](const tracked_contact &aFirst, const tracked_contact &aSecond) { return aFirst.key < aSecond.key; });

    aContacts.erase(std::unique(aContacts.begin(), aContacts.end(),
        [](const tracked_contact &aFirst, const tracked_contact &aSecond) { return aFirst.key == aSecond.key; }),
        aContacts.end());
}

void impl_collision_event_tracker::record_collision(const impl_collider_ptr_type &aA,
    const impl_collider_ptr_type &aB, const collision_vector3_type &aNormal,
    const collision_vector3_type &aPoint, const collision_floating_point_type aPenetration) {
    record(m_Collisions, aA, aB, aNormal, aPoint, aPenetration);
}

void impl_collision_event_tracker::record_trigger(const impl_collider_ptr_type &aA, const impl_collider_ptr_type &aB) {
    record(m_Triggers, aA, aB, collision_vector3_type::zero, collision_vector3_type::zero, 0);
}

template <typename event_type>
void impl_collision_event_tracker::emit(const contact_collection_type &aCurrent,
    const contact_collection_type &aPrevious, std::vector<event_type> &aInto) {
    const auto append = [&aInto](const tracked_contact &aContact, const gdk::event_type aKind) {
        const auto a = aContact.a.lock();
        const auto b = aContact.b.lock();
        if (!a || !b) return;

        aInto.push_back(event_type{a, b, aKind});
    };

    std::vector<const tracked_contact *> exits;

    std::size_t current = 0;
    std::size_t previous = 0;

    while (current < aCurrent.size() || previous < aPrevious.size()) {
        const bool haveCurrent = current < aCurrent.size();
        const bool havePrevious = previous < aPrevious.size();

        if (haveCurrent && (!havePrevious || aCurrent[current].key < aPrevious[previous].key)) {
            append(aCurrent[current], gdk::event_type::enter);
            ++current;
        }
        else if (havePrevious && (!haveCurrent || aPrevious[previous].key < aCurrent[current].key)) {
            exits.push_back(&aPrevious[previous]);
            ++previous;
        }
        else {
            append(aCurrent[current], gdk::event_type::stay);
            ++current;
            ++previous;
        }
    }

    for (const auto pExit : exits) append(*pExit, gdk::event_type::exit);
}

void impl_collision_event_tracker::flush() {
    prepare(m_Collisions);
    prepare(m_Triggers);

    emit<collision_event>(m_Collisions, m_PreviousCollisions, m_PendingCollisions);
    emit<trigger_event>(m_Triggers, m_PreviousTriggers, m_PendingTriggers);

    m_PreviousCollisions.swap(m_Collisions);
    m_PreviousTriggers.swap(m_Triggers);
    m_Collisions.clear();
    m_Triggers.clear();
}

void impl_collision_event_tracker::dispatch() {
    for (const auto &event : m_PendingCollisions) m_CollisionObserver(event);
    for (const auto &event : m_PendingTriggers) m_TriggerOverlapObserver(event);

    m_PendingCollisions.clear();
    m_PendingTriggers.clear();
}
