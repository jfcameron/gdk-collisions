// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_PLANE_COLLIDER_H
#define GDK_COLLISION_PLANE_COLLIDER_H

#include <gdk/collider.h>
#include <gdk/collision_types.h>

namespace gdk {
    /// \brief infinite plane in the XZ (horizontal) plane in worldspace
    class plane_collider : public virtual collider {
    public:
        virtual ~plane_collider() = default;
    };
}

#endif
