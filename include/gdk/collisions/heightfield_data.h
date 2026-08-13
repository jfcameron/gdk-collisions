// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_HEIGHTFIELD_DATA_H
#define GDK_COLLISIONS_HEIGHTFIELD_DATA_H

#include <gdk/collisions/types.h>

#include <cstddef>

namespace gdk::collisions {
    /// \brief terrain: a regular grid of height data over the XZ plane, shared between colliders.
    class heightfield_data {
    public:
        virtual ~heightfield_data() = default;

        /// \brief samples along the local X axis
        [[nodiscard]] virtual std::size_t columns() const = 0;

        /// \brief samples along the local Z axis
        [[nodiscard]] virtual std::size_t rows() const = 0;

        /// \brief the height at a sample
        [[nodiscard]] virtual floating_point_type height(const std::size_t aColumn,
            const std::size_t aRow) const = 0;

        /// \brief spacing between samples, on local X and local Z.
        [[nodiscard]] virtual vector2_type cell_size() const = 0;
    };
}

#endif
