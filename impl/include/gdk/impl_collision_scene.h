// © Joseph Cameron - All Rights Reserved

#ifndef GDK_IMPL_COLLISION_SCENE_H
#define GDK_IMPL_COLLISION_SCENE_H

#include <gdk/collision_events.h>
#include <gdk/collision_scene.h>
#include <gdk/impl_collision_policy.h>
#include <gdk/impl_collision_types.h>

#include <functional>

namespace gdk {
    /// \brief a collision_scene implemented with a uniform-grid broadphase and conservative
    /// advancement.
    /// TODO: offer alternatives. one obvious one would have a fixed world-size, which would let me use an array instead of a map
    /// so access would go from log to constant complexity
    class impl_collision_scene : public collision_scene {
    public:
        using collision_observer_type = std::function<void(collision_event)>;
        using trigger_overlap_observer_type = std::function<void(trigger_event)>;

        /// \brief create a collision scene using this implementation
        [[nodiscard]] static collision_scene_ptr_type make(collision_observer_type aCollisionObserver,
            trigger_overlap_observer_type aTriggerObserver, impl_collision_policy = {},
            collision_task_dispatcher_type aDispatcher = {});

        virtual ~impl_collision_scene() = default;
    };
}

#endif
