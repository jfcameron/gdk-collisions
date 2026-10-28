// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_EVENTS_H
#define GDK_COLLISIONS_EVENTS_H

#include <gdk/collisions/types.h>

#include <functional>

namespace gdk::collisions {
    enum class event_type {
        enter,
        exit,
        stay,
    };

    /// \brief two colliders that began, kept or stopped touching
    struct collider_event final {
        const_collider_ptr_type a;
        const_collider_ptr_type b;
        event_type type = event_type::enter;
    };

    using collision_event = collider_event;
    using trigger_event = collider_event;

    using collision_observer_type = std::function<void(collision_event)>;
    using trigger_observer_type = std::function<void(trigger_event)>;
}

#endif

