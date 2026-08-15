// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_SPHERE_COLLIDER_H
#define GDK_COLLISION_SPHERE_COLLIDER_H

#include <gdk/collider.h>
#include <gdk/collision_types.h>

namespace gdk {
    class sphere_collider : public virtual collider
{
    public:
        virtual ~sphere_collider() = default;

        /// \brief set the radius of the sphere
        virtual void set_radius(const collision_floating_point_type aRadius) = 0;

        /// \brief get the radius of the sphere
        [[nodiscard]] virtual collision_floating_point_type radius() const = 0;
    };
}

#endif

