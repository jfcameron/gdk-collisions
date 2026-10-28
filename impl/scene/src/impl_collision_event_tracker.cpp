// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_collision_event_tracker.h>

#include <algorithm>
#include <utility>

using namespace gdk::collisions;

impl_collision_event_tracker::impl_collision_event_tracker(observer_type aCollisionObserver,
    trigger_overlap_observer_type aTriggerObserver)
: m_CollisionObserver(std::move(aCollisionObserver))
, m_TriggerOverlapObserver(std::move(aTriggerObserver))
{}

void impl_collision_event_tracker::record(contact_collection_type &aInto, const impl_collider_ptr_type &aA,
    const impl_collider_ptr_type &aB, const vector3_type &aNormal,
    const vector3_type &aPoint, const floating_point_type aPenetration) {
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
    const impl_collider_ptr_type &aB, const vector3_type &aNormal,
    const vector3_type &aPoint, const floating_point_type aPenetration) {
    record(m_Collisions, aA, aB, aNormal, aPoint, aPenetration);
}

void impl_collision_event_tracker::record_trigger(const impl_collider_ptr_type &aA, const impl_collider_ptr_type &aB) {
    record(m_Triggers, aA, aB, vector3_type::zero, vector3_type::zero, 0);
}

void impl_collision_event_tracker::emit(const contact_collection_type &aCurrent,
    const contact_collection_type &aPrevious, pending_collection_type &aInto) {
    const auto append = [&aInto](const tracked_contact &aContact, const gdk::collisions::event_type aKind) {
        if (aContact.a.expired() || aContact.b.expired()) return;

        aInto.push_back(pending_event{aContact.a, aContact.b, aKind});
    };

    std::vector<const tracked_contact *> exits;

    std::size_t current = 0;
    std::size_t previous = 0;

    while (current < aCurrent.size() || previous < aPrevious.size()) {
        const bool haveCurrent = current < aCurrent.size();
        const bool havePrevious = previous < aPrevious.size();

        if (haveCurrent && (!havePrevious || aCurrent[current].key < aPrevious[previous].key)) {
            append(aCurrent[current], gdk::collisions::event_type::enter);
            ++current;
        }
        else if (havePrevious && (!haveCurrent || aPrevious[previous].key < aCurrent[current].key)) {
            exits.push_back(&aPrevious[previous]);
            ++previous;
        }
        else {
            append(aCurrent[current], gdk::collisions::event_type::stay);
            ++current;
            ++previous;
        }
    }

    for (const auto pExit : exits) append(*pExit, gdk::collisions::event_type::exit);
}

void impl_collision_event_tracker::flush() {
    prepare(m_Collisions);
    prepare(m_Triggers);

    if (m_CollisionObserver) emit(m_Collisions, m_PreviousCollisions, m_PendingCollisions);
    if (m_TriggerOverlapObserver) emit(m_Triggers, m_PreviousTriggers, m_PendingTriggers);

    m_PreviousCollisions.swap(m_Collisions);
    m_PreviousTriggers.swap(m_Triggers);
    m_Collisions.clear();
    m_Triggers.clear();
}

template <typename delivered_type, typename observer_function>
void impl_collision_event_tracker::dispatch(pending_collection_type &aPending, const observer_function &aObserver) {
    for (const auto &each : aPending) {
        const auto a = each.a.lock();
        const auto b = each.b.lock();

        if (a && b) aObserver(delivered_type{a, b, each.type});
    }

    aPending.clear();
}

void impl_collision_event_tracker::dispatch() {
    dispatch<collision_event>(m_PendingCollisions, m_CollisionObserver);
    dispatch<trigger_event>(m_PendingTriggers, m_TriggerOverlapObserver);
}
