// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_PLANE_COLLIDER_H
#define GDK_COLLISIONS_PLANE_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief infinite plane in the XZ (horizontal) plane in worldspace
    class plane_collider : public virtual collider {
    public:
        virtual ~plane_collider() = default;
    };
}

#endif
