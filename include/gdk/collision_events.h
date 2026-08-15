// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_EVENTS_H
#define GDK_COLLISION_EVENTS_H

#include <gdk/collision_types.h>

namespace gdk {
    enum class event_type {
        enter,
        exit,
        stay,
    };

    struct trigger_event {
        const const_collider_ptr_type a;
        const const_collider_ptr_type b;
        const event_type type;

        trigger_event(const const_collider_ptr_type &aA, const const_collider_ptr_type &aB, const event_type aType)
        : a(aA)
        , b(aB)
        , type(aType)
        {}
    };

    struct collision_event {
        const const_collider_ptr_type a;
        const const_collider_ptr_type b;
        const event_type type;

        collision_event(const const_collider_ptr_type &aA, const const_collider_ptr_type &aB, const event_type aType)
        : a(aA)
        , b(aB)
        , type(aType)
        {}
    };
}

#endif

