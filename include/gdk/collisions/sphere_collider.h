// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_SPHERE_COLLIDER_H
#define GDK_COLLISIONS_SPHERE_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    class sphere_collider : public virtual collider
{
    public:
        virtual ~sphere_collider() = default;

        /// \brief set the radius of the sphere
        virtual void set_radius(const floating_point_type aRadius) = 0;

        /// \brief get the radius of the sphere
        [[nodiscard]] virtual floating_point_type radius() const = 0;
    };
}

#endif

