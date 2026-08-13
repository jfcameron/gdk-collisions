// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_COMPOUND_COLLIDER_H
#define GDK_COLLISIONS_COMPOUND_COLLIDER_H

#include <gdk/collisions/collider.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief a collider built from several primitives placed relative to it.
    ///
    /// This is how concave geometry is expressed: an L-shaped wall is two boxes, a character's hit
    /// volume is a capsule and some boxes. 
    class compound_collider : public virtual collider {
    public:
        virtual ~compound_collider() = default;

        /// \brief add a sphere at a position relative to this collider
        virtual void add_sphere(const vector3_type &aPosition,
            const floating_point_type aRadius) = 0;

        /// \brief add an axis aligned box at a position relative to this collider.
        virtual void add_box(const vector3_type &aPosition,
            const vector3_type &aHalfExtents) = 0;

        /// \brief add a capsule at a position and rotation relative to this collider
        virtual void add_capsule(const vector3_type &aPosition,
            const quaternion_type &aRotation,
            const floating_point_type aRadius,
            const floating_point_type aHalfHeight) = 0;

        /// \brief how many parts this collider is built from
        [[nodiscard]] virtual std::size_t part_count() const = 0;

        /// \brief discard every part
        virtual void clear_parts() = 0;
    };
}

#endif
