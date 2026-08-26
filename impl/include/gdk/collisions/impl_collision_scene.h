// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_COLLISION_SCENE_H
#define GDK_COLLISIONS_IMPL_COLLISION_SCENE_H

#include <gdk/collisions/events.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_collision_types.h>

#include <functional>

namespace gdk::collisions {
    /// \brief a scene implemented with a uniform-grid broadphase and conservative
    /// advancement.
    /// TODO: offer alternatives. one obvious one would have a fixed world-size, which would let me use an array instead of a map
    /// so access would go from log to constant complexity
    class impl_collision_scene : public scene {
    public:
        using observer_type = std::function<void(collision_event)>;
        using trigger_overlap_observer_type = std::function<void(trigger_event)>;

        /// \brief create a collision scene using this implementation
        [[nodiscard]] static scene_ptr_type make(observer_type aCollisionObserver,
            trigger_overlap_observer_type aTriggerObserver, impl_collision_policy = {},
            task_dispatcher_type aDispatcher = {});

        virtual ~impl_collision_scene() = default;
    };
}

#endif
